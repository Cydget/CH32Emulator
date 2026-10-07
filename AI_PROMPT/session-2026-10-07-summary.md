# Session 2026-10-07 summary

## Goal

Build a Rust emulator that loads CH32 firmware, exercise it with firmware built from the ch32-rs / ch32-data / ch32-metapac stack and Embassy, and provide a web UI that shows pin electrical state and live graphs of static variables from the ELF symbol table.

## What was built

- `crates/ch32-emu`: RV32IMC interpreter with QingKe V4 CSR, vectored-address interrupts, and hardware-stack save/restore. The machine is a CH32V203C8T6 with flash, SRAM, GPIOA–D, RCC, TIM2/TIM3/TIM4, SysTick, USART1, and the PFIC.
- Web UI at `ch32-emu` (default port 8080). Pins are colored by electrical level (high, low, floating, analog, and the GPIO mode). Clicking a pin drives it from outside the chip. SRAM statics from the ELF can be checked and graphed while the core runs.
- `firmware/embassy-blink`: `no_std` firmware using `ch32-hal` (git), which pulls `ch32-metapac` from crates.io (generated from `ch32-data`), plus `embassy-executor` and `embassy-time`. It blinks PA15 and counts in `BLINK_COUNT`.
- Prompt log for this session: `AI_PROMPT/session-2026-10-07.md`. This file is the summary. Later sessions should add a new pair of files, as described in `AI_PROMPT/README.md`.

## How it was checked

- Unit tests cover RV32 arithmetic, compressed instructions, the gintenr CSR, and a tiny GPIO blink program.
- The Embassy ELF was built for `riscv32imc-unknown-none-elf` and run for 2,000,000 instructions: no faults, PA15 in push-pull output, `BLINK_COUNT` advanced.
- The web server returned the page, script, and stylesheet. A WebSocket client loaded that ELF, pressed run, and saw PA15 and `BLINK_COUNT`. No browser was available in this environment, so the page was not clicked by hand.

## Limits

The model is an instruction emulator of one part, not a cycle-accurate CH32. USART, ADC, USB, DMA, and the rest of the peripheral set are not functionally modeled; unmapped-as-RAM MMIO lets HAL init store register values. The Embassy blink path (GPIO, RCC, TIM2, PFIC) is the path that was run.
