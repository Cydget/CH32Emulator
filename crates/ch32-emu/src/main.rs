//! Web debugger and headless runner for the CH32V203 emulator.

use std::collections::HashMap;
use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use axum::extract::ws::{Message, WebSocket, WebSocketUpgrade};
use axum::extract::State;
use axum::response::Html;
use axum::routing::get;
use axum::Router;
use base64::Engine;
use clap::Parser;
use serde::Deserialize;
use serde_json::{json, Value};

use ch32_emu::elf::{self, StaticSymbol};
use ch32_emu::machine::{ExternalDrive, Machine};

const INDEX: &str = include_str!("../static/index.html");
const APP_JS: &str = include_str!("../static/app.js");
const APP_CSS: &str = include_str!("../static/app.css");

#[derive(Parser, Debug)]
#[command(
    name = "ch32-emu",
    about = "CH32V203 emulator with a web pin and variable debugger"
)]
struct Cli {
    /// ELF to load before serving or running.
    #[arg(long)]
    elf: Option<PathBuf>,
    /// Listen port for the web UI.
    #[arg(long, default_value_t = 8080)]
    port: u16,
    /// Headless: execute this many instructions, print a summary, and exit.
    #[arg(long)]
    steps: Option<u64>,
    /// Print each retired instruction. Useful while bringing up firmware.
    #[arg(long)]
    trace: bool,
    /// Guest cycles per second while running. 0 runs as fast as the host allows.
    #[arg(long, default_value_t = 8_000_000)]
    hz: u64,
}

struct Session {
    machine: Machine,
    symbols: Vec<StaticSymbol>,
    running: bool,
    hz: u64,
    /// Guest instructions per wall-clock second, measured while running.
    actual_hz: f64,
    watches: Vec<String>,
    history: HashMap<String, Vec<Value>>,
    breakpoint: Option<Breakpoint>,
    /// Symbol value observed before the most recent instruction.
    break_prev: Option<i64>,
    status: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum BreakKind {
    Increment,
    Decrement,
    Equals,
}

#[derive(Clone, Debug)]
struct Breakpoint {
    name: String,
    kind: BreakKind,
    value: i64,
}

fn main() {
    let cli = Cli::parse();
    let mut session = Session {
        machine: Machine::new(),
        symbols: Vec::new(),
        running: false,
        hz: cli.hz,
        actual_hz: 0.0,
        watches: Vec::new(),
        history: HashMap::new(),
        breakpoint: None,
        break_prev: None,
        status: "idle".into(),
    };
    session.machine.cpu.trace = cli.trace;
    if let Some(path) = &cli.elf {
        match std::fs::read(path) {
            Ok(bytes) => load_elf_bytes(&mut session, &bytes),
            Err(err) => {
                eprintln!("failed to read {}: {err}", path.display());
                std::process::exit(1);
            }
        }
    }
    if let Some(steps) = cli.steps {
        session.running = true;
        let mut faults = 0u64;
        for _ in 0..steps {
            if session.machine.step().is_err() {
                faults += 1;
                break;
            }
        }
        sample(&mut session);
        let pin = session.machine.pin(0, 15);
        println!(
            "pc={:08x} cycles={} faults={} PA15 mode={:?} level={:?} uart_bytes={}",
            session.machine.cpu.pc,
            session.machine.cpu.cycle,
            session.machine.faults.len() + faults as usize,
            pin.mode,
            pin.level,
            session.machine.uart_tx.len()
        );
        for symbol in &session.symbols {
            if let Some(value) = elf::read_symbol(&session.machine, symbol) {
                println!(
                    "  {:08x} {:>6}  {} = {value}",
                    symbol.address, symbol.size, symbol.demangled
                );
            }
        }
        if cli.trace {
            println!("{}", session.machine.debug_line());
        }
        if !session.machine.faults.is_empty() {
            println!("recent faults:");
            for fault in session.machine.faults.iter().rev().take(8) {
                println!("  {fault}");
            }
        }
        return;
    }

    let state = Arc::new(Mutex::new(session));
    let runner = Arc::clone(&state);
    std::thread::spawn(move || run_loop(runner));

    let app = Router::new()
        .route("/", get(index))
        .route("/app.js", get(app_js))
        .route("/app.css", get(app_css))
        .route("/ws", get(ws))
        .with_state(state);

    let addr = SocketAddr::from(([0, 0, 0, 0], cli.port));
    println!("CH32 emulator UI at http://{addr}");
    let runtime = tokio::runtime::Builder::new_multi_thread()
        .enable_all()
        .build()
        .expect("tokio runtime");
    runtime.block_on(async move {
        let listener = tokio::net::TcpListener::bind(addr).await.expect("bind");
        axum::serve(listener, app).await.expect("server");
    });
}

enum Pace {
    Idle,
    Unlimited,
    Limited(u64),
}

fn run_loop(state: Arc<Mutex<Session>>) {
    let mut window_start = Instant::now();
    let mut window_cycles = 0u64;
    let mut last_cycle = state.lock().expect("session").machine.cpu.cycle;
    let mut was_unlimited = false;
    loop {
        let pace = {
            let mut guard = state.lock().expect("session");
            if !guard.running {
                guard.actual_hz = 0.0;
                last_cycle = guard.machine.cpu.cycle;
                window_cycles = 0;
                window_start = Instant::now();
                was_unlimited = false;
                Pace::Idle
            } else {
                let unlimited = guard.hz == 0;
                if unlimited != was_unlimited {
                    window_cycles = 0;
                    window_start = Instant::now();
                    guard.actual_hz = 0.0;
                    was_unlimited = unlimited;
                }
                let batch_start = Instant::now();
                let mut stepped = 0u64;
                loop {
                    if guard.machine.step().is_err() {
                        guard.running = false;
                        guard.status = "halted on double fault".into();
                        break;
                    }
                    stepped += 1;
                    if stop_on_break(&mut guard) {
                        break;
                    }
                    if unlimited {
                        if stepped % 512 == 0 && batch_start.elapsed() >= Duration::from_millis(12)
                        {
                            break;
                        }
                        if stepped >= 200_000 {
                            break;
                        }
                    } else if stepped >= 2_000 {
                        break;
                    }
                }
                sample(&mut guard);
                let cycle = guard.machine.cpu.cycle;
                if cycle >= last_cycle {
                    window_cycles = window_cycles.saturating_add(cycle - last_cycle);
                } else {
                    window_cycles = 0;
                    window_start = Instant::now();
                }
                last_cycle = cycle;
                let elapsed = window_start.elapsed();
                if elapsed >= Duration::from_millis(200) && window_cycles > 0 {
                    guard.actual_hz = window_cycles as f64 / elapsed.as_secs_f64();
                    window_cycles = 0;
                    window_start = Instant::now();
                }
                if !guard.running {
                    Pace::Idle
                } else if unlimited {
                    Pace::Unlimited
                } else {
                    Pace::Limited(guard.hz.max(1))
                }
            }
        };
        match pace {
            Pace::Idle => std::thread::sleep(Duration::from_millis(20)),
            // Yield so the UI can publish the measured rate.
            Pace::Unlimited => std::thread::sleep(Duration::from_millis(2)),
            Pace::Limited(hz) => {
                let nanos = 2_000u128 * 1_000_000_000 / u128::from(hz);
                std::thread::sleep(Duration::from_nanos(nanos.max(1) as u64));
            }
        }
    }
}

fn sample(session: &mut Session) {
    let cycle = session.machine.cpu.cycle;
    for symbol in &session.symbols {
        if !watched(&session.watches, symbol) {
            continue;
        }
        let Some(value) = elf::read_symbol(&session.machine, symbol) else {
            continue;
        };
        let series = session.history.entry(symbol.demangled.clone()).or_default();
        series.push(json!({"cycle": cycle, "value": value}));
        if series.len() > 400 {
            let drain = series.len() - 400;
            series.drain(..drain);
        }
    }
}

fn watched(watches: &[String], symbol: &StaticSymbol) -> bool {
    if watches.is_empty() {
        return symbol.demangled.contains("BLINK_COUNT") || symbol.name.contains("BLINK_COUNT");
    }
    watches
        .iter()
        .any(|w| w == &symbol.demangled || w == &symbol.name || symbol.demangled.contains(w))
}

fn load_elf_bytes(session: &mut Session, bytes: &[u8]) {
    match elf::load_elf(bytes) {
        Ok(loaded) => {
            let segs: Vec<(u32, &[u8])> = loaded
                .segments
                .iter()
                .map(|s| (s.address, s.data.as_slice()))
                .collect();
            session.machine.load_image(bytes, loaded.entry, &segs);
            session.symbols = loaded.symbols;
            session.history.clear();
            session.running = false;
            session.status = format!(
                "loaded entry {:08x}, {} statics",
                loaded.entry,
                session.symbols.len()
            );
            if session.watches.is_empty() {
                session.watches = session
                    .symbols
                    .iter()
                    .filter(|s| {
                        s.demangled.contains("BLINK_COUNT") || s.name.contains("BLINK_COUNT")
                    })
                    .map(|s| s.demangled.clone())
                    .collect();
            }
        }
        Err(err) => session.status = format!("ELF load failed: {err}"),
    }
    refresh_break_prev(session);
}

fn value_crossed(kind: BreakKind, before: i64, after: i64, target: i64) -> bool {
    if before == after {
        return false;
    }
    match kind {
        BreakKind::Increment => after > before,
        BreakKind::Decrement => after < before,
        BreakKind::Equals => after == target,
    }
}

fn find_symbol<'a>(session: &'a Session, name: &str) -> Option<&'a StaticSymbol> {
    session
        .symbols
        .iter()
        .find(|symbol| symbol.demangled == name || symbol.name == name)
}

fn refresh_break_prev(session: &mut Session) {
    session.break_prev = session.breakpoint.as_ref().and_then(|bp| {
        find_symbol(session, &bp.name).and_then(|symbol| elf::read_symbol(&session.machine, symbol))
    });
}

/// Returns true when the armed breakpoint's transition happened on the step just executed.
fn stop_on_break(session: &mut Session) -> bool {
    let Some(bp) = session.breakpoint.clone() else {
        return false;
    };
    let Some(symbol) = find_symbol(session, &bp.name) else {
        return false;
    };
    let Some(now) = elf::read_symbol(&session.machine, symbol) else {
        return false;
    };
    let hit = session
        .break_prev
        .is_some_and(|prev| value_crossed(bp.kind, prev, now, bp.value));
    session.break_prev = Some(now);
    if hit {
        session.running = false;
        session.status = match bp.kind {
            BreakKind::Increment => format!("break: {} incremented to {now}", bp.name),
            BreakKind::Decrement => format!("break: {} decremented to {now}", bp.name),
            BreakKind::Equals => format!("break: {} reached {now}", bp.name),
        };
    }
    hit
}

const BREAK_STEP_CAP: u64 = 2_000_000;

fn step_until_break(session: &mut Session) {
    let Some(bp) = session.breakpoint.clone() else {
        return;
    };
    if find_symbol(session, &bp.name).is_none() {
        session.status = format!("no static named {}", bp.name);
        return;
    }
    if session.break_prev.is_none() {
        refresh_break_prev(session);
    }
    let mut n = 0u64;
    while n < BREAK_STEP_CAP {
        if session.machine.step().is_err() {
            session.status = "halted on double fault".into();
            sample(session);
            return;
        }
        n += 1;
        if stop_on_break(session) {
            session.status = format!("{} after {n} steps", session.status);
            sample(session);
            return;
        }
    }
    sample(session);
    session.status = format!("breakpoint not reached in {n} steps");
}

fn trim_history(session: &mut Session) {
    let cycle = session.machine.cpu.cycle;
    for points in session.history.values_mut() {
        points.retain(|point| point.get("cycle").and_then(Value::as_u64).unwrap_or(0) <= cycle);
    }
}

fn snapshot(session: &Session) -> Value {
    let cpu = &session.machine.cpu;
    let symbols: Vec<Value> = session
        .symbols
        .iter()
        .map(|s| {
            json!({
                "name": s.demangled,
                "raw": s.name,
                "address": s.address,
                "size": s.size,
                "value": elf::read_symbol(&session.machine, s),
                "watched": watched(&session.watches, s),
            })
        })
        .collect();
    let history: Vec<Value> = session
        .history
        .iter()
        .map(|(name, points)| json!({"name": name, "points": points}))
        .collect();
    let (insn, effect) = session.machine.current_insn();
    json!({
        "status": session.status,
        "running": session.running,
        "hz": session.hz,
        "actual_hz": session.actual_hz,
        "undo": session.machine.undo_available(),
        "breakpoint": session.breakpoint.as_ref().map(|bp| json!({
            "name": bp.name,
            "kind": match bp.kind {
                BreakKind::Increment => "inc",
                BreakKind::Decrement => "dec",
                BreakKind::Equals => "eq",
            },
            "value": bp.value,
        })),
        "peripherals": session.machine.peripherals(),
        "pc": cpu.pc,
        "cycles": cpu.cycle,
        "mstatus": cpu.mstatus,
        "mcause": cpu.mcause,
        "mepc": cpu.mepc,
        "sp": cpu.x(2),
        "insn": insn,
        "effect": effect,
        "pins": session.machine.pins(),
        "symbols": symbols,
        "history": history,
        "uart": session.machine.uart_tx,
        "faults": session.machine.faults,
    })
}

async fn index() -> Html<&'static str> {
    Html(INDEX)
}

async fn app_js() -> ([(axum::http::HeaderName, &'static str); 1], &'static str) {
    (
        [(
            axum::http::header::CONTENT_TYPE,
            "text/javascript; charset=utf-8",
        )],
        APP_JS,
    )
}

async fn app_css() -> ([(axum::http::HeaderName, &'static str); 1], &'static str) {
    (
        [(axum::http::header::CONTENT_TYPE, "text/css; charset=utf-8")],
        APP_CSS,
    )
}

async fn ws(
    ws: WebSocketUpgrade,
    State(state): State<Arc<Mutex<Session>>>,
) -> impl axum::response::IntoResponse {
    ws.on_upgrade(move |socket| handle_socket(socket, state))
}

async fn handle_socket(mut socket: WebSocket, state: Arc<Mutex<Session>>) {
    loop {
        let text = {
            let guard = state.lock().expect("session");
            snapshot(&guard).to_string()
        };
        if socket.send(Message::Text(text.into())).await.is_err() {
            break;
        }
        tokio::select! {
            incoming = socket.recv() => {
                match incoming {
                    Some(Ok(Message::Text(text))) => {
                        if let Ok(cmd) = serde_json::from_str::<Command>(text.as_str()) {
                            let mut guard = state.lock().expect("session");
                            apply(&mut guard, cmd);
                        }
                    }
                    Some(Ok(Message::Close(_))) | None | Some(Err(_)) => break,
                    _ => {}
                }
            }
            _ = tokio::time::sleep(Duration::from_millis(50)) => {}
        }
    }
}

#[derive(Deserialize)]
struct Command {
    op: String,
    #[serde(default)]
    count: u64,
    #[serde(default)]
    hz: Option<u64>,
    #[serde(default)]
    elf_base64: Option<String>,
    #[serde(default)]
    port: Option<String>,
    #[serde(default)]
    pin: Option<u8>,
    #[serde(default)]
    level: Option<String>,
    #[serde(default)]
    analog: Option<f32>,
    #[serde(default)]
    names: Vec<String>,
    #[serde(default)]
    break_name: Option<String>,
    #[serde(default)]
    break_kind: Option<String>,
    #[serde(default)]
    break_value: Option<i64>,
}

fn apply(session: &mut Session, cmd: Command) {
    match cmd.op.as_str() {
        "run" => {
            session.running = true;
            session.status = "running".into();
        }
        "pause" => {
            session.running = false;
            session.status = "paused".into();
        }
        "reset" => {
            session.running = false;
            session.machine.reset();
            session.history.clear();
            refresh_break_prev(session);
            session.status = "reset".into();
        }
        "step" => {
            session.running = false;
            let n = cmd.count.max(1).min(100_000);
            for _ in 0..n {
                if session.machine.step().is_err() {
                    session.status = "halted on double fault".into();
                    return;
                }
            }
            sample(session);
            session.status = format!("stepped {n}");
        }
        "until" => {
            session.running = false;
            if session.breakpoint.is_some() {
                step_until_break(session);
            } else {
                session.status = "no breakpoint armed".into();
            }
        }
        "back" => {
            session.running = false;
            if session.machine.reverse_step() {
                trim_history(session);
                refresh_break_prev(session);
                let left = session.machine.undo_available();
                session.status = format!("reversed one cycle, {left} earlier still saved");
            } else {
                session.status = "no earlier cycle saved".into();
            }
        }
        "breakpoint" => {
            let kind = cmd.break_kind.as_deref().unwrap_or("off");
            let name = cmd.break_name.clone().unwrap_or_default();
            if kind == "off" || name.is_empty() {
                let had = session.breakpoint.take().is_some();
                session.break_prev = None;
                if had {
                    session.status = "breakpoint off".into();
                }
            } else if find_symbol(session, &name).is_none() {
                session.breakpoint = None;
                session.break_prev = None;
                session.status = format!("no static named {name}");
            } else {
                let parsed = match kind {
                    "inc" => BreakKind::Increment,
                    "dec" => BreakKind::Decrement,
                    _ => BreakKind::Equals,
                };
                session.breakpoint = Some(Breakpoint {
                    name,
                    kind: parsed,
                    value: cmd.break_value.unwrap_or(0),
                });
                refresh_break_prev(session);
                session.status = match parsed {
                    BreakKind::Increment => "break when the static increments".into(),
                    BreakKind::Decrement => "break when the static decrements".into(),
                    BreakKind::Equals => format!(
                        "break when the static reaches {}",
                        cmd.break_value.unwrap_or(0)
                    ),
                };
            }
        }
        "speed" => {
            if let Some(hz) = cmd.hz {
                if hz != session.hz {
                    session.actual_hz = 0.0;
                }
                session.hz = hz;
            }
        }
        "watch" => {
            session.watches = cmd.names;
            session
                .history
                .retain(|name, _| session.watches.iter().any(|w| name.contains(w)));
        }
        "load" => {
            if let Some(b64) = cmd.elf_base64 {
                match base64::engine::general_purpose::STANDARD.decode(b64) {
                    Ok(bytes) => load_elf_bytes(session, &bytes),
                    Err(err) => session.status = format!("bad base64: {err}"),
                }
            }
        }
        "drive" => {
            let port = cmd.port.as_deref().unwrap_or("A");
            let pin = cmd.pin.unwrap_or(0);
            let drive = match cmd.level.as_deref().unwrap_or("float") {
                "high" => ExternalDrive::High,
                "low" => ExternalDrive::Low,
                _ => ExternalDrive::Float,
            };
            let index = match port.chars().next().unwrap_or('A') {
                'A' => 0,
                'B' => 1,
                'C' => 2,
                'D' => 3,
                _ => return,
            };
            session
                .machine
                .set_external(index, pin, drive, cmd.analog.unwrap_or(0.0));
        }
        _ => {}
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use ch32_emu::machine::{FLASH_BASE, SRAM_BASE};
    use std::collections::HashMap;

    fn command(op: &str) -> Command {
        Command {
            op: op.into(),
            count: 1,
            hz: None,
            elf_base64: None,
            port: None,
            pin: None,
            level: None,
            analog: None,
            names: Vec::new(),
            break_name: None,
            break_kind: None,
            break_value: None,
        }
    }

    fn session_with(code: &[u32], sram: &[u8]) -> Session {
        let mut image = Vec::new();
        for word in code {
            image.extend_from_slice(&word.to_le_bytes());
        }
        let mut session = Session {
            machine: ch32_emu::machine::Machine::new(),
            symbols: vec![StaticSymbol {
                name: "N".into(),
                demangled: "N".into(),
                address: SRAM_BASE,
                size: 4,
            }],
            running: false,
            hz: 0,
            actual_hz: 0.0,
            watches: Vec::new(),
            history: HashMap::new(),
            breakpoint: None,
            break_prev: None,
            status: String::new(),
        };
        session.machine.load_image(
            &image,
            FLASH_BASE,
            &[(FLASH_BASE, image.as_slice()), (SRAM_BASE, sram)],
        );
        session
    }

    fn arm(session: &mut Session, kind: &str, value: i64) {
        let mut cmd = command("breakpoint");
        cmd.break_name = Some("N".into());
        cmd.break_kind = Some(kind.into());
        cmd.break_value = Some(value);
        apply(session, cmd);
    }

    #[test]
    fn value_crossed_matches_each_condition() {
        assert!(value_crossed(BreakKind::Increment, 1, 2, 0));
        assert!(!value_crossed(BreakKind::Increment, 2, 2, 0));
        assert!(!value_crossed(BreakKind::Increment, 3, 1, 0));
        assert!(value_crossed(BreakKind::Decrement, 4, 3, 0));
        assert!(!value_crossed(BreakKind::Decrement, 3, 5, 0));
        assert!(value_crossed(BreakKind::Equals, 1, 7, 7));
        assert!(!value_crossed(BreakKind::Equals, 7, 8, 7));
        assert!(!value_crossed(BreakKind::Equals, 7, 7, 7));
    }

    #[test]
    fn step_advances_until_static_increments_or_hits_a_value() {
        let code = [
            0x2000_0137u32, // lui x2, 0x20000
            0x0010_0093,    // addi x1, x0, 1
            0x0011_2023,    // sw x1, 0(x2)
            0x0000_006f,    // jal x0, 0
        ];
        let mut session = session_with(&code, &[0, 0, 0, 0]);
        arm(&mut session, "inc", 0);
        apply(&mut session, command("step"));
        assert!(session.status.contains("stepped 1"), "{}", session.status);
        assert_eq!(
            elf::read_symbol(&session.machine, &session.symbols[0]),
            Some(0)
        );
        let (insn, effect) = session.machine.current_insn();
        assert!(insn.contains("addi"), "{insn}");
        assert!(effect.contains("ra = 0x1"), "{effect}");

        apply(&mut session, command("until"));
        assert!(
            session.status.contains("incremented to 1"),
            "{}",
            session.status
        );
        assert_eq!(
            elf::read_symbol(&session.machine, &session.symbols[0]),
            Some(1)
        );

        let mut equals = session_with(&code, &[0, 0, 0, 0]);
        arm(&mut equals, "eq", 1);
        apply(&mut equals, command("until"));
        assert!(equals.status.contains("reached 1"), "{}", equals.status);

        let dec_code = [
            0x2000_0137u32, // lui x2, 0x20000
            0x0040_0093,    // addi x1, x0, 4
            0x0011_2023,    // sw x1, 0(x2)
            0x0000_006f,
        ];
        let mut down = session_with(&dec_code, &[5, 0, 0, 0]);
        arm(&mut down, "dec", 0);
        apply(&mut down, command("until"));
        assert!(down.status.contains("decremented to 4"), "{}", down.status);

        apply(&mut down, command("back"));
        assert!(down.status.contains("reversed"), "{}", down.status);
        assert_eq!(elf::read_symbol(&down.machine, &down.symbols[0]), Some(5));
    }
}
