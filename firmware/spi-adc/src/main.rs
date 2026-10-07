//! Reads the emulator's example SPI ADC (`--attach spi1:adc:PA4`).
//!
//! SPI1 uses the default pins: PA5 SCK, PA6 MISO, PA7 MOSI. PA4 is a GPIO chip select.
//! Setup checks the ID register, powers the converter on channel 0, and waits for DRDY.
//! The loop then reads a conversion every 5 ms and steps through the four channels.
//! The statics below live in SRAM so the web UI can graph them.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use hal::gpio::{Level, Output};
use hal::spi::Spi;
use {ch32_hal as hal, panic_halt as _};

const READ: u8 = 0x80;
const REG_ID: u8 = 0;
const REG_CONFIG: u8 = 1;
const REG_STATUS: u8 = 2;
const REG_DATA: u8 = 3;
const CONFIG_PWR: u8 = 1 << 0;
const STATUS_DRDY: u8 = 1 << 7;
const EXPECTED_ID: u8 = 0xAD;

/// ID byte read during setup. 0xAD means the ADC answered.
#[no_mangle]
static mut ADC_ID: u32 = 0;
/// 1 once setup finished, 2 if the ID was wrong.
#[no_mangle]
static mut ADC_STATE: u32 = 0;
/// Latest 12-bit result, and the channel it came from.
#[no_mangle]
static mut ADC_RAW: u32 = 0;
#[no_mangle]
static mut ADC_CHANNEL: u32 = 0;
/// Latest result in millivolts, assuming a 3.3 V reference.
#[no_mangle]
static mut ADC_MV: u32 = 0;
/// Channel 0 only, so the graph shows the sine input.
#[no_mangle]
static mut ADC_CH0_MV: u32 = 0;
#[no_mangle]
static mut ADC_READS: u32 = 0;

fn store(slot: *mut u32, value: u32) {
    unsafe { slot.write_volatile(value) }
}

struct Adc<'d> {
    spi: Spi<'d, hal::peripherals::SPI1, hal::mode::Blocking>,
    cs: Output<'d>,
}

impl Adc<'_> {
    fn read(&mut self, reg: u8, out: &mut [u8]) {
        self.cs.set_low();
        let _ = self.spi.blocking_write(&[READ | reg]);
        out.fill(0);
        let _ = self.spi.blocking_transfer_in_place(out);
        self.cs.set_high();
    }

    fn write(&mut self, reg: u8, value: u8) {
        self.cs.set_low();
        let _ = self.spi.blocking_write(&[reg, value]);
        self.cs.set_high();
    }

    fn read_u8(&mut self, reg: u8) -> u8 {
        let mut buf = [0u8; 1];
        self.read(reg, &mut buf);
        buf[0]
    }

    fn wait_ready(&mut self) {
        while self.read_u8(REG_STATUS) & STATUS_DRDY == 0 {}
    }

    fn select(&mut self, channel: u8) {
        self.write(REG_CONFIG, CONFIG_PWR | ((channel & 3) << 4));
    }

    fn sample(&mut self) -> u16 {
        self.wait_ready();
        let mut buf = [0u8; 2];
        self.read(REG_DATA, &mut buf);
        u16::from_be_bytes(buf) & 0x0fff
    }
}

#[embassy_executor::main(entry = "qingke_rt::entry")]
async fn main(_spawner: Spawner) -> ! {
    let p = hal::init(hal::Config::default());
    let spi = Spi::new_blocking(p.SPI1, p.PA5, p.PA7, p.PA6, hal::spi::Config::default());
    let cs = Output::new(p.PA4, Level::High, Default::default());
    let mut adc = Adc { spi, cs };

    let id = adc.read_u8(REG_ID);
    store(&raw mut ADC_ID, id as u32);
    if id != EXPECTED_ID {
        store(&raw mut ADC_STATE, 2);
        loop {
            embassy_time::Timer::after_millis(100).await;
        }
    }
    adc.select(0);
    adc.wait_ready();
    store(&raw mut ADC_STATE, 1);

    let mut reads: u32 = 0;
    loop {
        // Three channel-0 samples, then one from the next of channels 1-3.
        let channel = if reads % 4 == 3 { 1 + ((reads / 4) % 3) as u8 } else { 0 };
        adc.select(channel);
        let raw = adc.sample();
        let mv = raw as u32 * 3300 / 4095;
        store(&raw mut ADC_RAW, raw as u32);
        store(&raw mut ADC_CHANNEL, channel as u32);
        store(&raw mut ADC_MV, mv);
        if channel == 0 {
            store(&raw mut ADC_CH0_MV, mv);
        }
        reads = reads.wrapping_add(1);
        store(&raw mut ADC_READS, reads);
        embassy_time::Timer::after_millis(5).await;
    }
}
