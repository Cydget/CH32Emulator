# Session 2026-10-07 summary

## Goal

Build a Rust emulator that loads CH32 firmware, exercise it with firmware built from the ch32-rs / ch32-data / ch32-metapac stack and Embassy, and provide a web UI that shows pin electrical state and live graphs of static variables from the ELF symbol table.

## What was built

- `crates/ch32-emu`: RV32IMC interpreter with QingKe V4 CSR, vectored-address interrupts, and hardware-stack save/restore. The machine is a CH32V203C8T6 with flash, SRAM, GPIOA–D, RCC, TIM2/TIM3/TIM4, SysTick, USART1, and the PFIC.
- Web UI at `ch32-emu` (default port 8080). Pins are colored by electrical level (high, low, floating, analog, and the GPIO mode). Clicking a pin drives it from outside the chip. SRAM statics from the ELF can be checked and graphed while the core runs.
- A later prompt the same day added a peripheral inspector. Menus select the block and the instance (TIM → TIM2, and the same for GPIO, RCC, SysTick, USART, and the PFIC). The panel lists current inputs and the register changes the peripheral's next update will make, including address, old value, and new value. Unlimited speed shows the measured guest rate.
- Another prompt added breakpoints on statics (increment, decrement, or a chosen value). Step to break runs until that change, and Run stops on it. Step itself always advances one instruction. Back reverses one instruction, up to 10 saved cycles.
- The Core panel shows the instruction at the program counter and a “does” line that says what it will do with the current registers and memory.
- A component interface (`devices.rs`: `SpiDevice`, `UartDevice`, `I2cDevice`) connects simulated parts to SPI1/SPI2, USART1, and I2C1/I2C2, which are now modeled. The example is a 4-channel SPI ADC that needs power-up and DRDY polling before it returns data. `firmware/spi-adc` reads it through `ch32-hal`. Components attach with `--attach spi1:adc:PA4` or from the peripheral panel, and appear under Devices in the inspector.
- `firmware/embassy-blink`: `no_std` firmware using `ch32-hal` (git), which pulls `ch32-metapac` from crates.io (generated from `ch32-data`), plus `embassy-executor` and `embassy-time`. It blinks PA15 and counts in `BLINK_COUNT`.
- Prompt log for this session: `AI_PROMPT/session-2026-10-07.md`. This file is the summary. Later sessions should add a new pair of files, as described in `AI_PROMPT/README.md`.

## How it was checked

- Unit tests cover RV32 arithmetic, compressed instructions, the gintenr CSR, instruction explanations (`addi`, `sw`, `lw`, `c.li`), a tiny GPIO blink program, a TIM2 preview that reports a counter reset at `0x40000024` (old 4, new 0), static breakpoints (including a single step while a break is armed), reverse step, the SPI ADC, a UART echo component, and an I2C write then read.
- The `spi-adc` firmware ran headless with the ADC attached. It read ID 0xAD and kept converting, with channel 0 following the sine.
- The Embassy ELF was built for `riscv32imc-unknown-none-elf` and run for 2,000,000 instructions: no faults, PA15 in push-pull output, `BLINK_COUNT` advanced.
- The web server returned the page, script, and stylesheet. A WebSocket client loaded that ELF, pressed run, and saw PA15 and `BLINK_COUNT`. A later check of the peripheral inspector confirmed TIM2's CNT address in the snapshot, and an unlimited run reported about 2.45 MHz. No browser was available in this environment, so the page was not clicked by hand.

## Limits

The model is an instruction emulator of one part, not a cycle-accurate CH32. SPI and I2C transfers finish instantly, and USART timing ignores the baud rate. The on-chip ADC, USB, DMA, and the rest of the peripheral set are not functionally modeled; unmapped-as-RAM MMIO lets HAL init store register values. The Embassy blink path (GPIO, RCC, TIM2, PFIC) is the path that was run.
