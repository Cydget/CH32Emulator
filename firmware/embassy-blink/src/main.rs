//! Embassy blink used to exercise the emulator.
//!
//! `ch32-hal` is the Embassy HAL from the ch32-rs project. It talks to the chip through
//! `ch32-metapac`, which is generated from `ch32-data`. `hal::init` starts the TIM2
//! Embassy time driver. `BLINK_COUNT` lives in SRAM so the web UI can graph it.

#![no_std]
#![no_main]

use core::cell::RefCell;

use embassy_executor::Spawner;
use embassy_futures::join::join;
use hal::exti::ExtiInput;
use hal::gpio::{Level, Output, Pull};
use {ch32_hal as hal, panic_halt as _};

#[no_mangle]
static mut BLINK_COUNT: u32 = 0;

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(hal::Config::default());
    let led = RefCell::new(Output::new(p.PA15, Level::Low, Default::default()));
    let led2 = RefCell::new(Output::new(p.PA14, Level::Low, Default::default()));

    let mut sig_input = ExtiInput::new(p.PB1, p.EXTI1, Pull::Up);

    let loop_a = async {
        loop {
            led.borrow_mut().toggle();
            unsafe {
                let count = &raw mut BLINK_COUNT;
                count.write_volatile(count.read_volatile().wrapping_add(1));
            }
            embassy_time::Timer::after_millis(50).await;
        }
    };
    let loop_b = async {
        loop {
            sig_input.wait_for_rising_edge().await;
            led2.borrow_mut().toggle();
        }
    };

    join(loop_a, loop_b).await;
    core::future::pending().await
}
