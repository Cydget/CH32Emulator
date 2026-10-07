# CH32 emulator

A Rust interpreter for [WCH CH32V203](http://www.wch.cn/products/CH32V203.html) firmware, with a web UI for pin state and live graphs of static variables.

The first machine is the **CH32V203C8T6** (QingKe V4B, RV32IMC):

- 64 KiB zero-wait flash at `0x08000000`, also visible at `0x00000000`
- 20 KiB SRAM at `0x20000000`
- GPIOA–GPIOD, RCC, TIM2/TIM3/TIM4, SysTick, USART1, and the PFIC interrupt controller

It is an instruction emulator, not a cycle-accurate model. Other peripheral addresses behave as RAM so HAL init that only stores register values can proceed. GPIO mode comes from `CFGLR` / `CFGHR`. The UI shows high, low, floating, analog, pull-up, pull-down, push-pull, open-drain, and alternate function.

## Run the UI

```bash
cargo run -p ch32-emu -- --port 8080
```

Open `http://127.0.0.1:8080`. Load an ELF, press Run, and check SRAM statics to graph them. Click a pin to drive it from outside the chip.

Headless smoke test:

```bash
cargo run -p ch32-emu -- --elf firmware.elf --steps 2000000
```

## Firmware used to test it

`firmware/embassy-blink` is a `no_std` blink for PA15. It is built with:

- [`ch32-hal`](https://github.com/ch32-rs/ch32-hal) from the [ch32-rs](https://github.com/ch32-rs/ch32-rs) project, which is the Embassy HAL
- [`ch32-metapac`](https://github.com/ch32-rs/ch32-metapac), pulled in by the HAL and generated from [`ch32-data`](https://github.com/ch32-rs/ch32-data)
- [`embassy-time`](https://github.com/embassy-rs/embassy), with the HAL's TIM2 time driver started in `hal::init`

`BLINK_COUNT` is a SRAM static the graph can follow.

```bash
rustup target add riscv32imc-unknown-none-elf
cargo build --manifest-path firmware/embassy-blink/Cargo.toml --release --target riscv32imc-unknown-none-elf
cargo run -p ch32-emu -- --elf firmware/embassy-blink/target/riscv32imc-unknown-none-elf/release/embassy-blink
```

## Prompt log

Sessions that created this tool are written under [`AI_PROMPT`](AI_PROMPT/README.md).
