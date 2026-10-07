const ports = document.querySelector("#ports");
const statusEl = document.querySelector("#status");
const coreEl = document.querySelector("#core");
const uartEl = document.querySelector("#uart");
const faultsEl = document.querySelector("#faults");
const symbolsEl = document.querySelector("#symbols");
const canvas = document.querySelector("#graph");
const drive = document.querySelector("#drive");
const driveTitle = document.querySelector("#drive-title");
const analog = document.querySelector("#analog");
const analogReadout = document.querySelector("#analog-readout");

let socket;
let latest = null;
let selected = null;
const colors = ["#3dd68c", "#6cb6ff", "#e6c15a", "#c084fc", "#f07178", "#7fd1c3"];

function connect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  socket = new WebSocket(`${proto}://${location.host}/ws`);
  socket.addEventListener("message", (event) => {
    latest = JSON.parse(event.data);
    render(latest);
  });
  socket.addEventListener("close", () => {
    statusEl.textContent = "disconnected, retrying";
    setTimeout(connect, 1000);
  });
}

function send(msg) {
  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify(msg));
  }
}

document.querySelector("#run").onclick = () => send({ op: "run" });
document.querySelector("#pause").onclick = () => send({ op: "pause" });
document.querySelector("#step").onclick = () => send({ op: "step", count: 1 });
document.querySelector("#reset").onclick = () => send({ op: "reset" });
document.querySelector("#speed").onchange = (event) => {
  send({ op: "speed", hz: Number(event.target.value) });
};
document.querySelector("#elf").onchange = async (event) => {
  const file = event.target.files[0];
  if (!file) return;
  const bytes = new Uint8Array(await file.arrayBuffer());
  let binary = "";
  const chunk = 0x8000;
  for (let i = 0; i < bytes.length; i += chunk) {
    binary += String.fromCharCode(...bytes.subarray(i, i + chunk));
  }
  send({ op: "load", elf_base64: btoa(binary) });
};

drive.addEventListener("click", (event) => {
  const level = event.target.dataset.level;
  if (!level || !selected) return;
  send({
    op: "drive",
    port: selected.port,
    pin: selected.pin,
    level,
    analog: Number(analog.value),
  });
});
analog.oninput = () => {
  analogReadout.textContent = `${Number(analog.value).toFixed(2)} V`;
  if (!selected) return;
  send({
    op: "drive",
    port: selected.port,
    pin: selected.pin,
    level: "float",
    analog: Number(analog.value),
  });
};

function render(snap) {
  statusEl.textContent = `${snap.status}  pc ${hex(snap.pc)}  ${snap.running ? "running" : "stopped"}`;
  coreEl.innerHTML = [
    ["pc", hex(snap.pc)],
    ["insn", snap.insn],
    ["sp", hex(snap.sp)],
    ["cycles", String(snap.cycles)],
    ["mstatus", hex(snap.mstatus)],
    ["mepc", hex(snap.mepc)],
    ["mcause", hex(snap.mcause)],
  ].map(([k, v]) => `<dt>${k}</dt><dd>${escapeHtml(v)}</dd>`).join("");
  uartEl.textContent = snap.uart || "(quiet)";
  faultsEl.textContent = (snap.faults || []).slice(-8).join("\n") || "(none)";
  renderPins(snap.pins || []);
  renderSymbols(snap.symbols || []);
  drawGraph(snap.history || []);
}

function renderPins(pins) {
  if (ports.childElementCount === 0) {
    const byPort = new Map();
    for (const pin of pins) {
      if (!byPort.has(pin.port)) byPort.set(pin.port, []);
      byPort.get(pin.port).push(pin);
    }
    for (const [port, list] of byPort) {
      const block = document.createElement("div");
      block.className = "port";
      block.innerHTML = `<h3>GPIO${port}</h3>`;
      const grid = document.createElement("div");
      grid.className = "grid";
      for (const pin of list) {
        const cell = document.createElement("button");
        cell.type = "button";
        cell.dataset.port = pin.port;
        cell.dataset.pin = String(pin.pin);
        cell.innerHTML = `<span class="n">P${pin.port}${pin.pin}</span><span class="m"></span>`;
        cell.onclick = () => {
          selected = pin;
          drive.hidden = false;
          driveTitle.textContent = `P${pin.port}${pin.pin}`;
          renderPins(latest.pins);
        };
        grid.appendChild(cell);
      }
      block.appendChild(grid);
      ports.appendChild(block);
    }
  }
  for (const pin of pins) {
    const cell = ports.querySelector(`button[data-port="${pin.port}"][data-pin="${pin.pin}"]`);
    if (!cell) continue;
    cell.className = `pin ${pin.level}`;
    if (selected && selected.port === pin.port && selected.pin === pin.pin) {
      cell.classList.add("selected");
      driveTitle.textContent = `P${pin.port}${pin.pin}  ${pin.mode}`;
    }
    const volts = pin.level === "analog" ? ` ${Number(pin.analog_v).toFixed(2)}V` : "";
    cell.querySelector(".m").textContent = `${pin.level}${volts}`;
    cell.title = `${pin.mode}${pin.clock_enabled ? "" : " (clock off)"}`;
  }
}

function renderSymbols(symbols) {
  if (symbols.length === 0) {
    if (symbolsEl.dataset.count !== "0") {
      symbolsEl.dataset.count = "0";
      symbolsEl.textContent = "Load an ELF to list statics in SRAM.";
    }
    return;
  }
  if (symbolsEl.dataset.count !== String(symbols.length)) {
    symbolsEl.dataset.count = String(symbols.length);
    symbolsEl.innerHTML = "";
    for (const symbol of symbols) {
      const row = document.createElement("label");
      row.className = "symbol";
      const box = document.createElement("input");
      box.type = "checkbox";
      box.checked = symbol.watched;
      box.dataset.name = symbol.name;
      box.onchange = () => {
        const names = [...symbolsEl.querySelectorAll("input:checked")].map((el) => el.dataset.name);
        send({ op: "watch", names });
      };
      const name = document.createElement("span");
      name.textContent = symbol.name;
      name.title = `${hex(symbol.address)} ${symbol.size} bytes`;
      const value = document.createElement("span");
      value.className = "value";
      row.append(box, name, value);
      symbolsEl.appendChild(row);
    }
  }
  const rows = symbolsEl.querySelectorAll(".symbol");
  symbols.forEach((symbol, index) => {
    const row = rows[index];
    if (!row) return;
    const box = row.querySelector("input");
    if (document.activeElement !== box) box.checked = symbol.watched;
    row.querySelector(".value").textContent = symbol.value === null ? "?" : String(symbol.value);
  });
}

function drawGraph(history) {
  const ctx = canvas.getContext("2d");
  const width = canvas.width;
  const height = canvas.height;
  ctx.clearRect(0, 0, width, height);
  ctx.strokeStyle = "#2c3644";
  ctx.strokeRect(40, 10, width - 50, height - 30);
  if (!history.length) {
    ctx.fillStyle = "#93a0b0";
    ctx.fillText("Check a static variable to graph it.", 52, 40);
    return;
  }
  let min = Infinity;
  let max = -Infinity;
  let minCycle = Infinity;
  let maxCycle = -Infinity;
  for (const series of history) {
    for (const point of series.points) {
      min = Math.min(min, point.value);
      max = Math.max(max, point.value);
      minCycle = Math.min(minCycle, point.cycle);
      maxCycle = Math.max(maxCycle, point.cycle);
    }
  }
  if (min === max) {
    min -= 1;
    max += 1;
  }
  if (minCycle === maxCycle) maxCycle += 1;
  ctx.fillStyle = "#93a0b0";
  ctx.font = "12px ui-monospace, monospace";
  ctx.fillText(String(max), 4, 20);
  ctx.fillText(String(min), 4, height - 24);
  history.forEach((series, index) => {
    ctx.beginPath();
    ctx.strokeStyle = colors[index % colors.length];
    series.points.forEach((point, i) => {
      const x = 40 + ((point.cycle - minCycle) / (maxCycle - minCycle)) * (width - 50);
      const y = 10 + (1 - (point.value - min) / (max - min)) * (height - 30);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();
    ctx.fillStyle = colors[index % colors.length];
    ctx.fillText(series.name, 48, 28 + index * 14);
  });
}

function hex(value) {
  return `0x${(Number(value) >>> 0).toString(16).padStart(8, "0")}`;
}

function escapeHtml(text) {
  return String(text).replace(/[&<>]/g, (ch) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;" }[ch]));
}

connect();
