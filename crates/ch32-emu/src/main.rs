//! Web debugger and headless runner for the CH32V203 emulator.

use std::collections::HashMap;
use std::net::SocketAddr;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::Duration;

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
#[command(name = "ch32-emu", about = "CH32V203 emulator with a web pin and variable debugger")]
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
    watches: Vec<String>,
    history: HashMap<String, Vec<Value>>,
    status: String,
}

fn main() {
    let cli = Cli::parse();
    let mut session = Session {
        machine: Machine::new(),
        symbols: Vec::new(),
        running: false,
        hz: cli.hz,
        watches: Vec::new(),
        history: HashMap::new(),
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

fn run_loop(state: Arc<Mutex<Session>>) {
    loop {
        let hz = {
            let mut guard = state.lock().expect("session");
            if !guard.running {
                0
            } else {
                let batch = 2_000u64;
                for _ in 0..batch {
                    if guard.machine.step().is_err() {
                        guard.running = false;
                        guard.status = "halted on double fault".into();
                        break;
                    }
                }
                sample(&mut guard);
                guard.hz
            }
        };
        if hz == 0 {
            std::thread::sleep(Duration::from_millis(20));
        } else {
            let nanos = 2_000u128 * 1_000_000_000 / hz as u128;
            std::thread::sleep(Duration::from_nanos(nanos.max(1) as u64));
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
    watches.iter().any(|w| {
        w == &symbol.demangled || w == &symbol.name || symbol.demangled.contains(w)
    })
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
                    .filter(|s| s.demangled.contains("BLINK_COUNT") || s.name.contains("BLINK_COUNT"))
                    .map(|s| s.demangled.clone())
                    .collect();
            }
        }
        Err(err) => session.status = format!("ELF load failed: {err}"),
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
    let insn = session.machine.current_insn();
    json!({
        "status": session.status,
        "running": session.running,
        "hz": session.hz,
        "pc": cpu.pc,
        "cycles": cpu.cycle,
        "mstatus": cpu.mstatus,
        "mcause": cpu.mcause,
        "mepc": cpu.mepc,
        "sp": cpu.x(2),
        "insn": insn,
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

async fn ws(ws: WebSocketUpgrade, State(state): State<Arc<Mutex<Session>>>) -> impl axum::response::IntoResponse {
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
        "speed" => {
            if let Some(hz) = cmd.hz {
                session.hz = hz;
            }
        }
        "watch" => {
            session.watches = cmd.names;
            session.history.retain(|name, _| session.watches.iter().any(|w| name.contains(w)));
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
