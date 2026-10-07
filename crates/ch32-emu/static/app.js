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
const speedSelect = document.querySelector("#speed");
const speedActual = document.querySelector("#speed-actual");
const periGroup = document.querySelector("#peri-group");
const periId = document.querySelector("#peri-id");
const periAction = document.querySelector("#peri-action");
const periInputs = document.querySelector("#peri-inputs");
const periEffects = document.querySelector("#peri-effects");
const periOutputs = document.querySelector("#peri-outputs");
const periOutputsWrap = document.querySelector("#peri-outputs-wrap");
const stepBtn = document.querySelector("#step");
const untilBtn = document.querySelector("#until");
const backBtn = document.querySelector("#back");
const breakSymbol = document.querySelector("#break-symbol");
const breakKind = document.querySelector("#break-kind");
const breakValue = document.querySelector("#break-value");
const breakValueLabel = document.querySelector("#break-value-label");
const attachKind = document.querySelector("#attach-kind");
const attachBus = document.querySelector("#attach-bus");
const attachCs = document.querySelector("#attach-cs");
const deviceSettings = document.querySelector("#device-settings");
let periGroups = "";
let kindsReady = false;
let deviceCount = 0;
let settingsFor = "";

let socket;
let latest = null;
let selected = null;
let speedSynced = false;
let periMenuReady = false;
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

document.querySelector("#run").onclick = () => {
  sendBreakpoint();
  send({ op: "run" });
};
document.querySelector("#pause").onclick = () => send({ op: "pause" });
stepBtn.onclick = () => {
  sendBreakpoint();
  send({ op: "step", count: 1 });
};
untilBtn.onclick = () => {
  sendBreakpoint();
  send({ op: "until" });
};
backBtn.onclick = () => send({ op: "back" });
document.querySelector("#reset").onclick = () => send({ op: "reset" });
breakKind.onchange = () => {
  refreshBreakControls();
  sendBreakpoint();
};
breakSymbol.onchange = () => {
  refreshBreakControls();
  sendBreakpoint();
};
breakValue.onchange = () => sendBreakpoint();
speedSelect.onchange = (event) => {
  speedSynced = true;
  send({ op: "speed", hz: Number(event.target.value) });
  if (Number(event.target.value) === 0) {
    speedActual.textContent = latest && latest.running ? "measuring…" : "stopped";
  } else {
    speedActual.textContent = "";
  }
};
periGroup.onchange = () => {
  if (!latest) return;
  fillPeripherals(latest.peripherals || []);
  renderPeripheral(latest.peripherals || []);
};
periId.onchange = () => {
  if (!latest) return;
  renderPeripheral(latest.peripherals || []);
};
attachKind.onchange = () => fillAttachBuses();
document.querySelector("#attach").onclick = () => {
  send({ op: "attach", kind: attachKind.value, bus: attachBus.value, cs: attachCs.value.trim() });
};
deviceSettings.addEventListener("change", (event) => {
  const input = event.target;
  if (!input.dataset.key) return;
  send({ op: "device_set", device: settingsFor, key: input.dataset.key, value: Number(input.value) });
});
deviceSettings.addEventListener("click", (event) => {
  if (event.target.dataset.detach) send({ op: "detach", device: event.target.dataset.detach });
});
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
    ["does", snap.effect || ""],
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
  renderSpeed(snap);
  renderAttach(snap.device_kinds || []);
  renderPeripheral(snap.peripherals || []);
  renderDeviceSettings(snap.devices || []);
  const saved = Number(snap.undo) || 0;
  backBtn.disabled = saved === 0;
  backBtn.textContent = saved ? `Back (${saved})` : "Back";
}

function refreshBreakControls() {
  const armed = breakKind.value !== "off" && breakSymbol.value;
  untilBtn.disabled = !armed;
  breakValueLabel.hidden = breakKind.value !== "eq";
}

function sendBreakpoint() {
  send({
    op: "breakpoint",
    break_name: breakSymbol.value,
    break_kind: breakKind.value,
    break_value: Number(breakValue.value || 0),
  });
}

function renderSpeed(snap) {
  if (!speedSynced && snap.hz !== undefined && snap.hz !== null) {
    const hz = String(snap.hz);
    if (![...speedSelect.options].some((option) => option.value === hz)) {
      const option = document.createElement("option");
      option.value = hz;
      option.textContent = formatHz(Number(snap.hz));
      speedSelect.appendChild(option);
    }
    speedSelect.value = hz;
    speedSynced = true;
  }
  if (Number(speedSelect.value) !== 0) {
    speedActual.textContent = "";
    return;
  }
  if (!snap.running) {
    speedActual.textContent = "stopped";
    return;
  }
  const hz = Number(snap.actual_hz);
  speedActual.textContent = hz > 0 ? `running at ${formatHz(hz)}` : "measuring…";
}

function formatHz(hz) {
  if (hz >= 1_000_000) {
    const digits = hz >= 10_000_000 ? 1 : 2;
    return `${(hz / 1_000_000).toFixed(digits)} MHz`;
  }
  if (hz >= 1_000) {
    const digits = hz >= 10_000 ? 1 : 2;
    return `${(hz / 1_000).toFixed(digits)} kHz`;
  }
  return `${Math.round(hz)} Hz`;
}

function fillPeripherals(list) {
  const group = periGroup.value;
  const items = list.filter((item) => item.group === group);
  const previous = periId.value;
  periId.innerHTML = items
    .map((item) => `<option value="${escapeHtml(item.id)}">${escapeHtml(item.id)}</option>`)
    .join("");
  if (items.some((item) => item.id === previous)) periId.value = previous;
}

function renderPeripheral(list) {
  if (!list.length) return;
  const groups = [];
  for (const item of list) {
    if (!groups.includes(item.group)) groups.push(item.group);
  }
  const devices = list.filter((item) => item.group === "Devices");
  const signature = list.map((item) => `${item.group}/${item.id}`).join(",");
  if (signature !== periGroups) {
    const previous = periGroup.value;
    periGroup.innerHTML = groups
      .map((group) => `<option value="${escapeHtml(group)}">${escapeHtml(group)}</option>`)
      .join("");
    if (!periMenuReady) {
      if (groups.includes("TIM")) periGroup.value = "TIM";
    } else if (devices.length > deviceCount) {
      periGroup.value = "Devices";
    } else if (groups.includes(previous)) {
      periGroup.value = previous;
    }
    fillPeripherals(list);
    if (!periMenuReady && [...periId.options].some((option) => option.value === "TIM2")) {
      periId.value = "TIM2";
    } else if (devices.length > deviceCount && periGroup.value === "Devices") {
      periId.value = devices[devices.length - 1].id;
    }
    periMenuReady = true;
    periGroups = signature;
    deviceCount = devices.length;
  }
  const peripheral = list.find((item) => item.id === periId.value) || list[0];
  periAction.textContent = peripheral.action;
  periInputs.innerHTML = signalTable(peripheral.inputs);
  periEffects.innerHTML = effectTable(peripheral.effects);
  const outputs = peripheral.outputs || [];
  periOutputsWrap.hidden = outputs.length === 0;
  if (outputs.length) periOutputs.innerHTML = signalTable(outputs);
}

function renderAttach(kinds) {
  if (kindsReady || !kinds.length) return;
  attachKind.innerHTML = kinds
    .map((k) => `<option value="${escapeHtml(k.kind)}">${escapeHtml(k.label)}</option>`)
    .join("");
  attachKind.dataset.buses = JSON.stringify(Object.fromEntries(kinds.map((k) => [k.kind, k.buses])));
  fillAttachBuses();
  kindsReady = true;
}

function fillAttachBuses() {
  const buses = JSON.parse(attachKind.dataset.buses || "{}")[attachKind.value] || [];
  attachBus.innerHTML = buses
    .map((bus) => `<option value="${escapeHtml(bus)}">${escapeHtml(bus)}</option>`)
    .join("");
}

function renderDeviceSettings(devices) {
  const device = periGroup.value === "Devices" ? devices.find((d) => d.id === periId.value) : null;
  if (!device) {
    deviceSettings.hidden = true;
    settingsFor = "";
    return;
  }
  deviceSettings.hidden = false;
  const keys = device.settings.map((s) => s.key).join(",");
  if (settingsFor !== device.id || deviceSettings.dataset.keys !== keys) {
    deviceSettings.innerHTML =
      device.settings
        .map(
          (s) =>
            `<label>${escapeHtml(s.label)} <input type="number" step="any" data-key="${escapeHtml(s.key)}" value="${s.value}"></label>`,
        )
        .join("") + `<button type="button" data-detach="${escapeHtml(device.id)}">Detach ${escapeHtml(device.id)}</button>`;
    settingsFor = device.id;
    deviceSettings.dataset.keys = keys;
    return;
  }
  for (const s of device.settings) {
    const input = deviceSettings.querySelector(`input[data-key="${s.key}"]`);
    if (input && document.activeElement !== input) input.value = s.value;
  }
}

function signalTable(rows) {
  const body = (rows || [])
    .map(
      (row) =>
        `<tr><td>${escapeHtml(row.name)}</td><td>${addr(row.address)}</td><td>${escapeHtml(row.value)}</td></tr>`,
    )
    .join("");
  return `<thead><tr><th>Name</th><th>Address</th><th>Value</th></tr></thead><tbody>${
    body || `<tr><td colspan="3">(none)</td></tr>`
  }</tbody>`;
}

function effectTable(rows) {
  const body = (rows || [])
    .map(
      (row) =>
        `<tr><td>${escapeHtml(row.name)}</td><td>${addr(row.address)}</td><td>${formatNum(row["old"])}</td><td>${formatNum(row["new"])}</td><td>${escapeHtml(row.bits)}</td></tr>`,
    )
    .join("");
  return `<thead><tr><th>Name</th><th>Address</th><th>Old</th><th>New</th><th>Bits</th></tr></thead><tbody>${
    body || `<tr><td colspan="5">No memory changes this step.</td></tr>`
  }</tbody>`;
}

function addr(value) {
  if (value === null || value === undefined) return "—";
  return hex(value);
}

function formatNum(value) {
  const number = Number(value);
  if (!Number.isFinite(number)) return escapeHtml(value);
  if (number <= 0xffffffff) return `${number} (${hex(number)})`;
  return String(number);
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
      breakSymbol.innerHTML = "";
      refreshBreakControls();
    }
    return;
  }
  if (symbolsEl.dataset.count !== String(symbols.length)) {
    symbolsEl.dataset.count = String(symbols.length);
    const previous = breakSymbol.value;
    breakSymbol.innerHTML = symbols
      .map((symbol) => `<option value="${escapeHtml(symbol.name)}">${escapeHtml(symbol.name)}</option>`)
      .join("");
    if (symbols.some((symbol) => symbol.name === previous)) breakSymbol.value = previous;
    refreshBreakControls();
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
