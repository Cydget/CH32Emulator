//! Embassy blink used to exercise the emulator.
//!
//! `ch32-hal` is the Embassy HAL from the ch32-rs project. It talks to the chip through
//! `ch32-metapac`, which is generated from `ch32-data`. `hal::init` starts the TIM2
//! Embassy time driver. `BLINK_COUNT` lives in SRAM so the web UI can graph it.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use hal::gpio::{Level, Output};
use {ch32_hal as hal, panic_halt as _};

#[no_mangle]
static mut BLINK_COUNT: u32 = 0;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(hal::Config::default());
    let mut led = Output::new(p.PA15, Level::Low, Default::default());
    loop {
        led.toggle();
        unsafe {
            let count = &raw mut BLINK_COUNT;
            count.write_volatile(count.read_volatile().wrapping_add(1));
        }
        embassy_time::Timer::after_millis(50).await;
    }
}
