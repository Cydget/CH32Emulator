//! CH32V203C8T6 machine: memory map plus the peripherals Embassy blink firmware touches.

use std::collections::VecDeque;

use crate::cpu::{Bus, Cpu, CpuHeap, CpuScalars, Fault};
use crate::devices::{
    Created, DeviceCtx, DeviceSetting, DeviceView, I2cDevice, SpiDevice, UartDevice,
};
use crate::serial::{I2c, I2cAction, I2cPhase, Spi};

const UNDO_LIMIT: usize = 10;

pub const FLASH_BASE: u32 = 0x0800_0000;
pub const FLASH_SIZE: usize = 64 * 1024;
pub const SRAM_BASE: u32 = 0x2000_0000;
pub const SRAM_SIZE: usize = 20 * 1024;
pub const MMIO_BASE: u32 = 0x4000_0000;
pub const MMIO_SIZE: usize = 0x4_0000;
pub const INFO_BASE: u32 = 0x1FFF_0000;
pub const INFO_SIZE: usize = 0x1_0000;
pub const PFIC_BASE: u32 = 0xE000_E000;
pub const PFIC_SIZE: usize = 0x1000;
pub const SYSTICK_BASE: u32 = 0xE000_F000;

const IRQ_TIM2: u32 = 44;
const IRQ_TIM3: u32 = 45;
const IRQ_TIM4: u32 = 46;
const IRQ_SYSTICK: u32 = 12;

const GPIO_BASES: [u32; 4] = [0x4001_0800, 0x4001_0C00, 0x4001_1000, 0x4001_1400];
const TIM_BASES: [u32; 3] = [0x4000_0000, 0x4000_0400, 0x4000_0800];
const TIM_IRQS: [u32; 3] = [IRQ_TIM2, IRQ_TIM3, IRQ_TIM4];
const RCC_BASE: u32 = 0x4002_1000;
const USART1_BASE: u32 = 0x4001_3800;
const AFIO_BASE: u32 = 0x4001_0000;
const EXTI_BASE: u32 = 0x4001_0400;
/// EXTI lines 0–4 have their own IRQ. Lines 5–9 share EXTI9_5, lines 10–15 share EXTI15_10.
const EXTI_IRQS: [(u32, u32); 7] = [
    (22, 1 << 0),
    (23, 1 << 1),
    (24, 1 << 2),
    (25, 1 << 3),
    (26, 1 << 4),
    (39, 0x03e0),
    (56, 0xfc00),
];
const SPI_BASES: [u32; 2] = [0x4001_3000, 0x4000_3800];
const SPI_IRQS: [u32; 2] = [51, 52];
const I2C_BASES: [u32; 2] = [0x4000_5400, 0x4000_5800];
const I2C_EV_IRQS: [u32; 2] = [47, 49];
const I2C_ER_IRQS: [u32; 2] = [48, 50];
const IRQ_USART1: u32 = 53;

const USART_RXNE: u32 = 1 << 5;
const USART_TC: u32 = 1 << 6;
const USART_TXE: u32 = 1 << 7;

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum PinMode {
    Analog,
    Floating,
    PullUp,
    PullDown,
    OutputPushPull,
    OutputOpenDrain,
    AlternatePushPull,
    AlternateOpenDrain,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum PinLevel {
    High,
    Low,
    Floating,
    Analog,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize)]
#[serde(rename_all = "kebab-case")]
pub enum ExternalDrive {
    High,
    Low,
    Float,
}

#[derive(Clone, Debug, serde::Serialize)]
pub struct PinView {
    pub port: char,
    pub pin: u8,
    pub mode: PinMode,
    pub level: PinLevel,
    pub analog_v: f32,
    pub clock_enabled: bool,
}

/// One register or signal a peripheral is sampling.
#[derive(Clone, Debug, serde::Serialize)]
pub struct SignalView {
    pub name: String,
    /// Guest address. `None` when the signal is internal or off-chip.
    pub address: Option<u32>,
    pub value: String,
}

/// A register the peripheral will change on its next update.
#[derive(Clone, Debug, serde::Serialize)]
pub struct MemEffect {
    pub name: String,
    /// Guest address. `None` when the value is not memory-mapped.
    pub address: Option<u32>,
    pub old: u64,
    pub new: u64,
    pub bits: String,
}

/// Live inputs and the memory writes predicted for the next peripheral update.
#[derive(Clone, Debug, serde::Serialize)]
pub struct PeripheralView {
    pub id: String,
    pub group: String,
    pub action: String,
    pub inputs: Vec<SignalView>,
    pub outputs: Vec<SignalView>,
    pub effects: Vec<MemEffect>,
}

#[derive(Clone, Debug)]
struct GpioPort {
    cfglr: u32,
    cfghr: u32,
    outdr: u32,
    lckr: u32,
    external: [ExternalDrive; 16],
    analog_v: [f32; 16],
}

impl GpioPort {
    fn new() -> Self {
        Self {
            cfglr: 0x4444_4444,
            cfghr: 0x4444_4444,
            outdr: 0,
            lckr: 0,
            external: [ExternalDrive::Float; 16],
            analog_v: [0.0; 16],
        }
    }

    fn cfg(&self, pin: u8) -> (u8, u8) {
        let reg = if pin < 8 { self.cfglr } else { self.cfghr };
        let shift = (pin % 8) * 4;
        let bits = (reg >> shift) & 0xf;
        ((bits & 3) as u8, ((bits >> 2) & 3) as u8)
    }

    fn mode_of(&self, pin: u8) -> PinMode {
        match self.cfg(pin) {
            (0, 0) => PinMode::Analog,
            (0, 1) => PinMode::Floating,
            (0, 2) => {
                if (self.outdr >> pin) & 1 == 1 {
                    PinMode::PullUp
                } else {
                    PinMode::PullDown
                }
            }
            (0, _) => PinMode::Floating,
            (_, 0) => PinMode::OutputPushPull,
            (_, 1) => PinMode::OutputOpenDrain,
            (_, 2) => PinMode::AlternatePushPull,
            _ => PinMode::AlternateOpenDrain,
        }
    }

    fn driven_high(&self, pin: u8) -> Option<bool> {
        let odr = (self.outdr >> pin) & 1 == 1;
        match self.mode_of(pin) {
            PinMode::OutputPushPull | PinMode::AlternatePushPull => Some(odr),
            PinMode::OutputOpenDrain | PinMode::AlternateOpenDrain => {
                if odr {
                    None
                } else {
                    Some(false)
                }
            }
            _ => None,
        }
    }

    fn level_of(&self, pin: u8) -> PinLevel {
        if let Some(high) = self.driven_high(pin) {
            return if high { PinLevel::High } else { PinLevel::Low };
        }
        match self.mode_of(pin) {
            PinMode::Analog => PinLevel::Analog,
            _ => match self.external[pin as usize] {
                ExternalDrive::High => PinLevel::High,
                ExternalDrive::Low => PinLevel::Low,
                ExternalDrive::Float => match self.mode_of(pin) {
                    PinMode::PullUp => PinLevel::High,
                    PinMode::PullDown => PinLevel::Low,
                    PinMode::Floating => PinLevel::Floating,
                    PinMode::OutputOpenDrain | PinMode::AlternateOpenDrain => PinLevel::Floating,
                    _ => PinLevel::Low,
                },
            },
        }
    }

    fn input_bit(&self, pin: u8) -> bool {
        match self.level_of(pin) {
            PinLevel::High => true,
            PinLevel::Low | PinLevel::Floating | PinLevel::Analog => false,
        }
    }

    fn indr(&self) -> u32 {
        let mut v = 0;
        for pin in 0..16 {
            if self.input_bit(pin) {
                v |= 1 << pin;
            }
        }
        v
    }

    fn read(&self, offset: u32) -> u32 {
        match offset {
            0x00 => self.cfglr,
            0x04 => self.cfghr,
            0x08 => self.indr(),
            0x0c => self.outdr,
            0x10 | 0x14 => 0,
            0x18 => self.lckr,
            _ => 0,
        }
    }

    fn write(&mut self, offset: u32, value: u32) {
        match offset {
            0x00 => self.cfglr = value,
            0x04 => self.cfghr = value,
            0x0c => self.outdr = value & 0xffff,
            0x10 => {
                let set = value & 0xffff;
                let reset = (value >> 16) & 0xffff;
                self.outdr = (self.outdr | set) & !reset;
                // If both set and reset are written, set wins, matching the reference manual.
                self.outdr |= set;
            }
            0x14 => self.outdr &= !(value & 0xffff),
            0x18 => self.lckr = value,
            _ => {}
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct Timer {
    ctlr1: u32,
    ctlr2: u32,
    smcfgr: u32,
    dier: u32,
    sr: u32,
    ccmr: [u32; 2],
    ccer: u32,
    cnt: u16,
    psc: u16,
    arr: u16,
    ccr: [u16; 4],
    presc: u16,
    irq: u32,
}

impl Timer {
    fn new(irq: u32) -> Self {
        Self {
            ctlr1: 0,
            ctlr2: 0,
            smcfgr: 0,
            dier: 0,
            sr: 0,
            ccmr: [0; 2],
            ccer: 0,
            cnt: 0,
            psc: 0,
            arr: 0xffff,
            ccr: [0; 4],
            presc: 0,
            irq,
        }
    }

    fn cen(&self) -> bool {
        self.ctlr1 & 1 != 0
    }

    fn urs_counter_only(&self) -> bool {
        self.ctlr1 & 4 != 0
    }

    fn generate_update(&mut self, from_counter: bool) {
        self.cnt = 0;
        self.presc = 0;
        if from_counter || !self.urs_counter_only() {
            self.sr |= 1;
        }
    }

    fn tick(&mut self) {
        if !self.cen() {
            return;
        }
        if self.presc < self.psc {
            self.presc = self.presc.wrapping_add(1);
            return;
        }
        self.presc = 0;
        let previous = self.cnt;
        if self.cnt == self.arr {
            self.cnt = 0;
            self.sr |= 1;
        } else {
            self.cnt = self.cnt.wrapping_add(1);
        }
        for i in 0..4 {
            if self.cnt == self.ccr[i] && previous != self.ccr[i] {
                self.sr |= 1 << (i + 1);
            }
        }
    }

    fn irq_asserted(&self) -> bool {
        // UIE and CC1IE..CC4IE share bit positions with UIF and CCxIF.
        (self.sr & self.dier & 0x1f) != 0
    }

    fn read(&self, offset: u32) -> u32 {
        match offset {
            0x00 => self.ctlr1,
            0x04 => self.ctlr2,
            0x08 => self.smcfgr,
            0x0c => self.dier,
            0x10 => self.sr,
            0x14 => 0,
            0x18 => self.ccmr[0],
            0x1c => self.ccmr[1],
            0x20 => self.ccer,
            0x24 => self.cnt as u32,
            0x28 => self.psc as u32,
            0x2c => self.arr as u32,
            0x34 => self.ccr[0] as u32,
            0x38 => self.ccr[1] as u32,
            0x3c => self.ccr[2] as u32,
            0x40 => self.ccr[3] as u32,
            _ => 0,
        }
    }

    fn write(&mut self, offset: u32, value: u32) {
        match offset {
            0x00 => self.ctlr1 = value,
            0x04 => self.ctlr2 = value,
            0x08 => self.smcfgr = value,
            0x0c => self.dier = value,
            0x10 => {
                // Status flags are write-0-to-clear.
                self.sr &= value;
            }
            0x14 => {
                if value & 1 != 0 {
                    self.generate_update(false);
                }
            }
            0x18 => self.ccmr[0] = value,
            0x1c => self.ccmr[1] = value,
            0x20 => self.ccer = value,
            0x24 => self.cnt = value as u16,
            0x28 => self.psc = value as u16,
            0x2c => self.arr = value as u16,
            0x34 => self.ccr[0] = value as u16,
            0x38 => self.ccr[1] = value as u16,
            0x3c => self.ccr[2] = value as u16,
            0x40 => self.ccr[3] = value as u16,
            _ => {}
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct SysTick {
    ctlr: u32,
    sr: u32,
    cnt: u64,
    cmp: u64,
}

impl SysTick {
    fn new() -> Self {
        Self {
            ctlr: 0,
            sr: 0,
            cnt: 0,
            cmp: 0,
        }
    }

    fn tick(&mut self) {
        if self.ctlr & 1 == 0 {
            return;
        }
        let down = self.ctlr & (1 << 4) != 0;
        if down {
            if self.cnt == 0 {
                self.cnt = self.cmp;
                self.sr |= 1;
            } else {
                self.cnt -= 1;
            }
        } else {
            self.cnt = self.cnt.wrapping_add(1);
            if self.cmp != 0 && self.cnt >= self.cmp {
                self.sr |= 1;
                if self.ctlr & (1 << 3) != 0 {
                    self.cnt = 0;
                }
            }
        }
    }

    fn irq_asserted(&self) -> bool {
        self.ctlr & 2 != 0 && self.sr & 1 != 0
    }

    fn read(&self, offset: u32) -> u32 {
        match offset {
            0x00 => self.ctlr,
            0x04 => self.sr,
            0x08 => self.cnt as u32,
            0x0c => (self.cnt >> 32) as u32,
            0x10 => self.cmp as u32,
            0x14 => (self.cmp >> 32) as u32,
            _ => 0,
        }
    }

    fn write(&mut self, offset: u32, value: u32) {
        match offset {
            0x00 => self.ctlr = value,
            0x04 => {
                if value & 1 == 0 {
                    self.sr &= !1;
                }
            }
            0x08 => self.cnt = (self.cnt & !0xffff_ffff) | value as u64,
            0x0c => self.cnt = (self.cnt & 0xffff_ffff) | ((value as u64) << 32),
            0x10 => self.cmp = (self.cmp & !0xffff_ffff) | value as u64,
            0x14 => self.cmp = (self.cmp & 0xffff_ffff) | ((value as u64) << 32),
            _ => {}
        }
    }
}

#[derive(Clone, Debug)]
struct Pfic {
    enable: [u32; 8],
    pending: [u32; 8],
    active: [u32; 8],
    priority: [u8; 256],
    other: [u8; PFIC_SIZE],
}

impl Pfic {
    fn new() -> Self {
        Self {
            enable: [0; 8],
            pending: [0; 8],
            active: [0; 8],
            priority: [0; 256],
            other: [0; PFIC_SIZE],
        }
    }

    fn bit(irq: u32) -> Option<(usize, u32)> {
        if irq >= 256 {
            None
        } else {
            Some(((irq / 32) as usize, 1 << (irq % 32)))
        }
    }

    fn set_pending(&mut self, irq: u32, pending: bool) {
        if let Some((word, mask)) = Self::bit(irq) {
            if pending {
                self.pending[word] |= mask;
            } else {
                self.pending[word] &= !mask;
            }
        }
    }

    fn read_word(&self, offset: u32) -> u32 {
        let idx = (offset / 4) as usize;
        match offset {
            0x000..=0x01c => self.enable.get(idx).copied().unwrap_or(0),
            0x020..=0x03c => self.pending.get(idx - 8).copied().unwrap_or(0),
            0x300..=0x31c => self
                .active
                .get((offset - 0x300) as usize / 4)
                .copied()
                .unwrap_or(0),
            0x400..=0x4ff => {
                let b = offset as usize;
                u32::from_le_bytes([
                    self.priority[b - 0x400],
                    self.priority.get(b - 0x3ff).copied().unwrap_or(0),
                    self.priority.get(b - 0x3fe).copied().unwrap_or(0),
                    self.priority.get(b - 0x3fd).copied().unwrap_or(0),
                ])
            }
            _ => {
                let o = offset as usize;
                if o + 4 <= self.other.len() {
                    u32::from_le_bytes(self.other[o..o + 4].try_into().unwrap())
                } else {
                    0
                }
            }
        }
    }

    fn write_word(&mut self, offset: u32, value: u32) {
        let idx = (offset / 4) as usize;
        match offset {
            0x100..=0x11c => {
                if idx >= 64 && idx < 72 {
                    self.enable[idx - 64] |= value;
                }
            }
            0x180..=0x19c => {
                if idx >= 96 && idx < 104 {
                    self.enable[idx - 96] &= !value;
                }
            }
            0x200..=0x21c => {
                if idx >= 128 && idx < 136 {
                    self.pending[idx - 128] |= value;
                }
            }
            0x280..=0x29c => {
                if idx >= 160 && idx < 168 {
                    self.pending[idx - 160] &= !value;
                }
            }
            0x400..=0x4ff => {
                let bytes = value.to_le_bytes();
                for (i, b) in bytes.iter().enumerate() {
                    let p = (offset as usize - 0x400) + i;
                    if p < self.priority.len() {
                        self.priority[p] = *b;
                    }
                }
            }
            _ => {
                let o = offset as usize;
                if o + 4 <= self.other.len() {
                    self.other[o..o + 4].copy_from_slice(&value.to_le_bytes());
                }
            }
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct Rcc {
    regs: [u32; 16],
}

impl Rcc {
    fn new() -> Self {
        let mut regs = [0; 16];
        regs[0] = 0x0000_0083;
        Self { regs }
    }

    fn sync(&mut self) {
        let ctlr = self.regs[0];
        let mut next = ctlr;
        if ctlr & 1 != 0 {
            next |= 1 << 1;
        } else {
            next &= !(1 << 1);
        }
        if ctlr & (1 << 16) != 0 {
            next |= 1 << 17;
        } else {
            next &= !(1 << 17);
        }
        if ctlr & (1 << 24) != 0 {
            next |= 1 << 25;
        } else {
            next &= !(1 << 25);
        }
        self.regs[0] = next;
        let sw = self.regs[1] & 0b11;
        self.regs[1] = (self.regs[1] & !0b1100) | (sw << 2);
    }

    fn apb1en(&self) -> u32 {
        self.regs[7]
    }

    fn apb2en(&self) -> u32 {
        self.regs[6]
    }
}

#[derive(Clone, Debug)]
struct Usart {
    cr1: u32,
    brr: u32,
    sr: u32,
    rx: u8,
    tx: Vec<u8>,
}

impl Usart {
    fn new() -> Self {
        Self {
            cr1: 0,
            brr: 0,
            sr: USART_TXE | USART_TC,
            rx: 0,
            tx: Vec::new(),
        }
    }

    fn read(&self, offset: u32) -> u32 {
        match offset {
            0x00 => self.sr,
            0x04 => self.rx as u32,
            0x08 => self.brr,
            0x0c => self.cr1,
            _ => 0,
        }
    }

    fn write(&mut self, offset: u32, value: u32) {
        match offset {
            // RXNE and TC clear when written as 0. The other flags are read-only.
            0x00 => self.sr &= value | !(USART_RXNE | USART_TC),
            0x04 => {
                self.tx.push(value as u8);
                self.sr |= USART_TXE | USART_TC;
            }
            0x08 => self.brr = value,
            0x0c => self.cr1 = value,
            _ => {}
        }
    }

    /// UE and RE.
    fn receiving(&self) -> bool {
        self.cr1 & (1 << 13) != 0 && self.cr1 & (1 << 2) != 0
    }

    fn irq_asserted(&self) -> bool {
        (self.cr1 & (1 << 5) != 0 && self.sr & USART_RXNE != 0)
            || (self.cr1 & (1 << 6) != 0 && self.sr & USART_TC != 0)
            || (self.cr1 & (1 << 7) != 0 && self.sr & USART_TXE != 0)
    }
}

/// AFIO external-interrupt port select. Four registers, one nibble per line.
#[derive(Clone, Copy, Debug)]
struct Afio {
    exti_cr: [u32; 4],
}

impl Afio {
    fn new() -> Self {
        Self { exti_cr: [0; 4] }
    }

    /// Port number (0 = A) wired to this EXTI line. Reset value selects port A.
    fn port(&self, line: u8) -> u8 {
        let line = u32::from(line & 15);
        let reg = self.exti_cr[(line / 4) as usize];
        ((reg >> ((line % 4) * 4)) & 0xf) as u8
    }
}

/// EXTI pending and trigger registers. An edge sets INTFR; INTENR gates the IRQ line.
#[derive(Clone, Copy, Debug)]
struct Exti {
    intenr: u32,
    evenr: u32,
    rtenr: u32,
    ftenr: u32,
    swievr: u32,
    intfr: u32,
}

impl Exti {
    fn new() -> Self {
        Self {
            intenr: 0,
            evenr: 0,
            rtenr: 0,
            ftenr: 0,
            swievr: 0,
            intfr: 0,
        }
    }

    fn read(&self, offset: u32) -> u32 {
        match offset {
            0x00 => self.intenr,
            0x04 => self.evenr,
            0x08 => self.rtenr,
            0x0c => self.ftenr,
            0x10 => self.swievr,
            0x14 => self.intfr,
            _ => 0,
        }
    }

    fn write(&mut self, offset: u32, value: u32) {
        match offset {
            0x00 => self.intenr = value,
            0x04 => self.evenr = value,
            0x08 => self.rtenr = value,
            0x0c => self.ftenr = value,
            // A 1 in SWIEVR raises the pending flag, the same as a pin edge.
            0x10 => {
                self.swievr = value;
                self.intfr |= value;
            }
            // INTFR is write-1-to-clear.
            0x14 => self.intfr &= !value,
            _ => {}
        }
    }

    fn irq_asserted(&self, mask: u32) -> bool {
        self.intfr & self.intenr & mask != 0
    }
}

enum Link {
    Spi {
        bus: usize,
        cs: Option<(usize, u8)>,
        selected: bool,
        dev: Box<dyn SpiDevice>,
    },
    Uart {
        dev: Box<dyn UartDevice>,
    },
    I2c {
        bus: usize,
        dev: Box<dyn I2cDevice>,
    },
}

impl Clone for Link {
    fn clone(&self) -> Self {
        match self {
            Link::Spi {
                bus,
                cs,
                selected,
                dev,
            } => Link::Spi {
                bus: *bus,
                cs: *cs,
                selected: *selected,
                dev: dev.clone_box(),
            },
            Link::Uart { dev } => Link::Uart {
                dev: dev.clone_box(),
            },
            Link::I2c { bus, dev } => Link::I2c {
                bus: *bus,
                dev: dev.clone_box(),
            },
        }
    }
}

#[derive(Clone)]
struct Attached {
    id: String,
    link: Link,
}

impl Attached {
    fn view(&self, ctx: &DeviceCtx) -> DeviceView {
        match &self.link {
            Link::Spi { dev, .. } => dev.view(ctx),
            Link::Uart { dev } => dev.view(ctx),
            Link::I2c { dev, .. } => dev.view(ctx),
        }
    }

    fn kind(&self) -> &'static str {
        match &self.link {
            Link::Spi { dev, .. } => dev.kind(),
            Link::Uart { dev } => dev.kind(),
            Link::I2c { dev, .. } => dev.kind(),
        }
    }

    fn settings(&self) -> Vec<DeviceSetting> {
        match &self.link {
            Link::Spi { dev, .. } => dev.settings(),
            Link::Uart { dev } => dev.settings(),
            Link::I2c { dev, .. } => dev.settings(),
        }
    }

    fn bus_name(&self) -> String {
        match &self.link {
            Link::Spi { bus, .. } => format!("SPI{}", bus + 1),
            Link::Uart { .. } => "USART1".into(),
            Link::I2c { bus, .. } => format!("I2C{}", bus + 1),
        }
    }

    fn cs_name(&self) -> Option<String> {
        match &self.link {
            Link::Spi {
                cs: Some((port, pin)),
                ..
            } => Some(pin_name(*port, *pin)),
            _ => None,
        }
    }
}

/// An attached component, as listed for the UI.
#[derive(Clone, Debug, serde::Serialize)]
pub struct DeviceInfo {
    pub id: String,
    pub kind: String,
    pub bus: String,
    pub cs: Option<String>,
    pub selected: bool,
    pub settings: Vec<DeviceSetting>,
}

fn pin_name(port: usize, pin: u8) -> String {
    format!("P{}{pin}", b"ABCD"[port] as char)
}

/// Parse `PA4` style pin names.
pub fn parse_pin(name: &str) -> Option<(usize, u8)> {
    let name = name.trim().to_ascii_uppercase();
    let rest = name.strip_prefix('P')?;
    let mut chars = rest.chars();
    let port = match chars.next()? {
        'A' => 0,
        'B' => 1,
        'C' => 2,
        'D' => 3,
        _ => return None,
    };
    let pin: u8 = chars.as_str().parse().ok()?;
    (pin < 16).then_some((port, pin))
}

#[derive(Clone, Copy)]
enum MemBank {
    Flash,
    Sram,
    Mmio,
    Info,
    Pfic,
    Priority,
}

#[derive(Clone, Copy)]
struct ByteUndo {
    bank: MemBank,
    index: u32,
    old: u8,
}

struct GpioRegs {
    cfglr: u32,
    cfghr: u32,
    outdr: u32,
    lckr: u32,
}

struct Undo {
    cpu: CpuScalars,
    cpu_heap: Option<CpuHeap>,
    gpio: [GpioRegs; 4],
    timers: [Timer; 3],
    systick: SysTick,
    rcc: Rcc,
    usart_cr1: u32,
    usart_brr: u32,
    usart_sr: u32,
    usart_rx: u8,
    usart_tx: Option<Vec<u8>>,
    spi: [Spi; 2],
    i2c: [I2c; 2],
    afio: Afio,
    exti: Exti,
    /// Attached components before this instruction, kept only when the instruction touched one.
    devices: Option<Vec<Attached>>,
    pfic_enable: [u32; 8],
    pfic_pending: [u32; 8],
    pfic_active: [u32; 8],
    mem: Vec<ByteUndo>,
    uart_tx: Option<String>,
    faults_len: usize,
}

pub struct Machine {
    pub cpu: Cpu,
    flash: Vec<u8>,
    sram: Vec<u8>,
    mmio: Vec<u8>,
    info: Vec<u8>,
    gpio: [GpioPort; 4],
    timers: [Timer; 3],
    systick: SysTick,
    pfic: Pfic,
    rcc: Rcc,
    usart1: Usart,
    spi: [Spi; 2],
    i2c: [I2c; 2],
    afio: Afio,
    exti: Exti,
    /// Last sampled electrical high for each pin, so a drive can be seen as an edge.
    pin_high: [[bool; 16]; 4],
    devices: Vec<Attached>,
    device_backup: Option<Vec<Attached>>,
    next_device: u32,
    /// Guest cycle handed to components. Set at the start of each instruction.
    now: u64,
    pub uart_tx: String,
    pub entry: u32,
    pub faults: Vec<String>,
    image: Vec<u8>,
    undo: VecDeque<Undo>,
    journal: Vec<ByteUndo>,
    journaling: bool,
}

impl Machine {
    pub fn new() -> Self {
        let mut info = vec![0u8; INFO_SIZE];
        // ESIG: chip id at 0x1FFFF704, flash size in KB at 0x1FFFF7E0.
        let id_off = (0x1FFF_F704 - INFO_BASE) as usize;
        info[id_off..id_off + 4].copy_from_slice(&0x2031_0500u32.to_le_bytes());
        let sz_off = (0x1FFF_F7E0 - INFO_BASE) as usize;
        info[sz_off..sz_off + 2].copy_from_slice(&64u16.to_le_bytes());
        Self {
            cpu: Cpu::default(),
            flash: vec![0; FLASH_SIZE],
            sram: vec![0; SRAM_SIZE],
            mmio: vec![0; MMIO_SIZE],
            info,
            gpio: std::array::from_fn(|_| GpioPort::new()),
            timers: std::array::from_fn(|i| Timer::new(TIM_IRQS[i])),
            systick: SysTick::new(),
            pfic: Pfic::new(),
            rcc: Rcc::new(),
            usart1: Usart::new(),
            spi: [Spi::new(); 2],
            i2c: [I2c::new(); 2],
            afio: Afio::new(),
            exti: Exti::new(),
            pin_high: [[false; 16]; 4],
            devices: Vec::new(),
            device_backup: None,
            next_device: 0,
            now: 0,
            uart_tx: String::new(),
            entry: FLASH_BASE,
            faults: Vec::new(),
            image: Vec::new(),
            undo: VecDeque::new(),
            journal: Vec::new(),
            journaling: false,
        }
    }

    pub fn load_image(&mut self, bytes: &[u8], entry: u32, segments: &[(u32, &[u8])]) {
        self.image = bytes.to_vec();
        self.entry = entry;
        self.reset_from_image(segments);
    }

    pub fn reset(&mut self) {
        let image = self.image.clone();
        if image.is_empty() {
            self.cpu.reset(self.entry);
            self.clear_undo();
            return;
        }
        if let Ok(info) = crate::elf::load_elf(&image) {
            let segs: Vec<(u32, Vec<u8>)> = info
                .segments
                .iter()
                .map(|s| (s.address, s.data.clone()))
                .collect();
            let borrowed: Vec<(u32, &[u8])> =
                segs.iter().map(|(a, d)| (*a, d.as_slice())).collect();
            self.entry = info.entry;
            self.reset_from_image(&borrowed);
        }
    }

    fn reset_from_image(&mut self, segments: &[(u32, &[u8])]) {
        self.clear_undo();
        self.flash.fill(0);
        self.sram.fill(0);
        self.mmio.fill(0);
        self.gpio = std::array::from_fn(|_| GpioPort::new());
        self.timers = std::array::from_fn(|i| Timer::new(TIM_IRQS[i]));
        self.systick = SysTick::new();
        self.pfic = Pfic::new();
        self.rcc = Rcc::new();
        self.usart1 = Usart::new();
        self.spi = [Spi::new(); 2];
        self.i2c = [I2c::new(); 2];
        self.afio = Afio::new();
        self.exti = Exti::new();
        self.uart_tx.clear();
        self.faults.clear();
        for (addr, data) in segments {
            let _ = self.write_bytes(*addr, data);
        }
        self.cpu.reset(self.entry);
        self.now = self.cpu.cycle;
        let ctx = self.ctx();
        for attached in &mut self.devices {
            match &mut attached.link {
                Link::Spi {
                    cs, selected, dev, ..
                } => {
                    dev.reset();
                    *selected = cs.is_none();
                    dev.select(*selected, &ctx);
                }
                Link::Uart { dev } => dev.reset(),
                Link::I2c { dev, .. } => dev.reset(),
            }
        }
        self.update_chip_selects();
        self.sample_pin_levels();
    }

    pub fn step(&mut self) -> Result<(), Fault> {
        let mut undo = self.capture_undo();
        self.journaling = true;
        let result = self.step_inner();
        self.journaling = false;
        undo.mem = self.journal.clone();
        undo.devices = self.device_backup.take();
        self.journal.clear();
        self.push_undo(undo);
        result
    }

    fn ctx(&self) -> DeviceCtx {
        DeviceCtx { cycle: self.now }
    }

    /// Keep the pre-instruction components so reverse step can restore them.
    fn touch_devices(&mut self) {
        if self.journaling && self.device_backup.is_none() && !self.devices.is_empty() {
            self.device_backup = Some(self.devices.clone());
        }
    }

    /// Attach a component on SPI1 (`bus` 0) or SPI2 (`bus` 1). With a chip-select pin the
    /// component is selected while that GPIO reads low. Without one it is always selected.
    pub fn attach_spi(
        &mut self,
        bus: usize,
        cs: Option<(usize, u8)>,
        mut dev: Box<dyn SpiDevice>,
    ) -> String {
        let id = self.device_id(dev.kind());
        self.now = self.cpu.cycle;
        let selected = cs.is_none();
        dev.select(selected, &self.ctx());
        self.devices.push(Attached {
            id: id.clone(),
            link: Link::Spi {
                bus: bus.min(1),
                cs,
                selected,
                dev,
            },
        });
        self.clear_undo();
        self.update_chip_selects();
        id
    }

    /// Attach a component to USART1 TX and RX.
    pub fn attach_uart(&mut self, dev: Box<dyn UartDevice>) -> String {
        let id = self.device_id(dev.kind());
        self.devices.push(Attached {
            id: id.clone(),
            link: Link::Uart { dev },
        });
        self.clear_undo();
        id
    }

    /// Attach a target on I2C1 (`bus` 0) or I2C2 (`bus` 1).
    pub fn attach_i2c(&mut self, bus: usize, dev: Box<dyn I2cDevice>) -> String {
        let id = self.device_id(dev.kind());
        self.devices.push(Attached {
            id: id.clone(),
            link: Link::I2c {
                bus: bus.min(1),
                dev,
            },
        });
        self.clear_undo();
        id
    }

    /// Attach a built-in component by kind (see [`crate::devices::KINDS`]).
    /// `bus` is `SPI1`, `SPI2`, `USART1`, `I2C1`, or `I2C2`. `cs` is a pin like `PA4`.
    pub fn attach_kind(
        &mut self,
        bus: &str,
        kind: &str,
        cs: Option<&str>,
    ) -> Result<String, String> {
        let bus = bus.trim().to_ascii_uppercase();
        let cs = match cs.map(str::trim).filter(|s| !s.is_empty()) {
            Some(name) => {
                Some(parse_pin(name).ok_or_else(|| format!("bad chip-select pin {name}"))?)
            }
            None => None,
        };
        let created =
            crate::devices::create(kind).ok_or_else(|| format!("unknown component {kind}"))?;
        match (bus.as_str(), created) {
            ("SPI1", Created::Spi(dev)) => Ok(self.attach_spi(0, cs, dev)),
            ("SPI2", Created::Spi(dev)) => Ok(self.attach_spi(1, cs, dev)),
            ("USART1", Created::Uart(dev)) => Ok(self.attach_uart(dev)),
            ("I2C1", Created::I2c(dev)) => Ok(self.attach_i2c(0, dev)),
            ("I2C2", Created::I2c(dev)) => Ok(self.attach_i2c(1, dev)),
            (bus, _) => Err(format!("{kind} does not connect to {bus}")),
        }
    }

    pub fn detach(&mut self, id: &str) -> bool {
        let before = self.devices.len();
        self.devices.retain(|d| d.id != id);
        let removed = self.devices.len() != before;
        if removed {
            self.clear_undo();
        }
        removed
    }

    /// Change a component setting from the UI.
    pub fn set_device(&mut self, id: &str, key: &str, value: f64) -> Result<(), String> {
        let attached = self
            .devices
            .iter_mut()
            .find(|d| d.id == id)
            .ok_or_else(|| format!("no component {id}"))?;
        let result = match &mut attached.link {
            Link::Spi { dev, .. } => dev.set(key, value),
            Link::Uart { dev } => dev.set(key, value),
            Link::I2c { dev, .. } => dev.set(key, value),
        };
        if result.is_ok() {
            self.clear_undo();
        }
        result
    }

    pub fn devices(&self) -> Vec<DeviceInfo> {
        self.devices
            .iter()
            .map(|d| DeviceInfo {
                id: d.id.clone(),
                kind: d.kind().into(),
                bus: d.bus_name(),
                cs: d.cs_name(),
                selected: matches!(d.link, Link::Spi { selected: true, .. }),
                settings: d.settings(),
            })
            .collect()
    }

    fn device_id(&mut self, kind: &str) -> String {
        let id = format!("{}{}", kind.to_ascii_uppercase(), self.next_device);
        self.next_device += 1;
        id
    }

    fn update_chip_selects(&mut self) {
        if self.devices.is_empty() {
            return;
        }
        let changes: Vec<(usize, bool)> = self
            .devices
            .iter()
            .enumerate()
            .filter_map(|(i, d)| match &d.link {
                Link::Spi {
                    cs: Some((port, pin)),
                    selected,
                    ..
                } => {
                    let want = self.gpio[*port].level_of(*pin) == PinLevel::Low;
                    (want != *selected).then_some((i, want))
                }
                _ => None,
            })
            .collect();
        if changes.is_empty() {
            return;
        }
        self.touch_devices();
        let ctx = self.ctx();
        for (i, want) in changes {
            if let Link::Spi { selected, dev, .. } = &mut self.devices[i].link {
                *selected = want;
                dev.select(want, &ctx);
            }
        }
    }

    fn spi_clock(&self, bus: usize) -> bool {
        if bus == 0 {
            self.rcc.apb2en() & (1 << 12) != 0
        } else {
            self.rcc.apb1en() & (1 << 14) != 0
        }
    }

    fn i2c_clock(&self, bus: usize) -> bool {
        self.rcc.apb1en() & (1 << (21 + bus)) != 0
    }

    /// Shift one frame to every selected component on the bus. MISO idles high, so
    /// overlapping replies AND together.
    fn spi_exchange(&mut self, bus: usize, frame: u16) -> u16 {
        let wire = self.spi[bus].wire_bytes(frame);
        let mut miso = vec![0xffu8; wire.len()];
        self.touch_devices();
        let ctx = self.ctx();
        for attached in &mut self.devices {
            if let Link::Spi {
                bus: b,
                selected: true,
                dev,
                ..
            } = &mut attached.link
            {
                if *b == bus {
                    for (out, &byte) in miso.iter_mut().zip(&wire) {
                        *out &= dev.transfer(byte, &ctx);
                    }
                }
            }
        }
        self.spi[bus].frame_from_wire(&miso)
    }

    fn i2c_target(&mut self, bus: usize, address: u8) -> Option<&mut Box<dyn I2cDevice>> {
        self.devices.iter_mut().find_map(|d| match &mut d.link {
            Link::I2c { bus: b, dev } if *b == bus && dev.address() == address => Some(dev),
            _ => None,
        })
    }

    fn i2c_act(&mut self, bus: usize, action: I2cAction) {
        if action == I2cAction::None {
            return;
        }
        self.touch_devices();
        let ctx = self.ctx();
        match action {
            I2cAction::None => {}
            I2cAction::Address(address, read) => {
                let ack = self
                    .i2c_target(bus, address)
                    .is_some_and(|dev| dev.start(read, &ctx));
                self.i2c[bus].addressed(address, read, ack);
            }
            I2cAction::Write(byte) => {
                let ack = match self.i2c[bus].target {
                    Some(address) => self
                        .i2c_target(bus, address)
                        .is_some_and(|dev| dev.write(byte, &ctx)),
                    None => false,
                };
                self.i2c[bus].written(ack);
            }
            I2cAction::Read => {
                let byte = match self.i2c[bus].target {
                    Some(address) => self
                        .i2c_target(bus, address)
                        .map_or(0xff, |dev| dev.read(&ctx)),
                    None => 0xff,
                };
                self.i2c[bus].received(byte);
            }
            I2cAction::Stop(address) => {
                if let Some(dev) = self.i2c_target(bus, address) {
                    dev.stop(&ctx);
                }
            }
        }
    }

    fn uart_send(&mut self, byte: u8) {
        if !self
            .devices
            .iter()
            .any(|d| matches!(d.link, Link::Uart { .. }))
        {
            return;
        }
        self.touch_devices();
        let ctx = self.ctx();
        for attached in &mut self.devices {
            if let Link::Uart { dev } = &mut attached.link {
                dev.receive(byte, &ctx);
            }
        }
    }

    fn poll_uart_devices(&mut self) {
        if self.devices.is_empty() || self.usart1.sr & USART_RXNE != 0 || !self.usart1.receiving() {
            return;
        }
        let ctx = self.ctx();
        let Some(index) = self.devices.iter().position(|d| match &d.link {
            Link::Uart { dev } => dev.has_byte(&ctx),
            _ => false,
        }) else {
            return;
        };
        self.touch_devices();
        if let Link::Uart { dev } = &mut self.devices[index].link {
            self.usart1.rx = dev.take_byte(&ctx);
            self.usart1.sr |= USART_RXNE;
        }
    }

    /// How many instructions can still be reversed. At most [`UNDO_LIMIT`].
    pub fn undo_available(&self) -> usize {
        self.undo.len()
    }

    /// Restore the machine to the state before the previous instruction.
    /// Pin drives applied after that instruction are kept.
    pub fn reverse_step(&mut self) -> bool {
        let Some(undo) = self.undo.pop_back() else {
            return false;
        };
        self.cpu.restore(undo.cpu, undo.cpu_heap);
        for (port, regs) in undo.gpio.into_iter().enumerate() {
            self.gpio[port].cfglr = regs.cfglr;
            self.gpio[port].cfghr = regs.cfghr;
            self.gpio[port].outdr = regs.outdr;
            self.gpio[port].lckr = regs.lckr;
        }
        self.timers = undo.timers;
        self.systick = undo.systick;
        self.rcc = undo.rcc;
        self.usart1.cr1 = undo.usart_cr1;
        self.usart1.brr = undo.usart_brr;
        self.usart1.sr = undo.usart_sr;
        self.usart1.rx = undo.usart_rx;
        self.spi = undo.spi;
        self.i2c = undo.i2c;
        self.afio = undo.afio;
        self.exti = undo.exti;
        self.sample_pin_levels();
        if let Some(devices) = undo.devices {
            self.devices = devices;
        }
        if let Some(tx) = undo.usart_tx {
            self.usart1.tx = tx;
        } else {
            self.usart1.tx.clear();
        }
        self.pfic.enable = undo.pfic_enable;
        self.pfic.pending = undo.pfic_pending;
        self.pfic.active = undo.pfic_active;
        self.journaling = false;
        for byte in undo.mem.into_iter().rev() {
            self.bank_set(byte.bank, byte.index as usize, byte.old);
        }
        if let Some(text) = undo.uart_tx {
            self.uart_tx = text;
        } else {
            self.uart_tx.clear();
        }
        self.faults.truncate(undo.faults_len);
        true
    }

    fn clear_undo(&mut self) {
        self.undo.clear();
        self.journal.clear();
        self.device_backup = None;
        self.journaling = false;
    }

    fn capture_undo(&self) -> Undo {
        let (cpu, cpu_heap) = self.cpu.checkpoint();
        Undo {
            cpu,
            cpu_heap,
            gpio: std::array::from_fn(|i| GpioRegs {
                cfglr: self.gpio[i].cfglr,
                cfghr: self.gpio[i].cfghr,
                outdr: self.gpio[i].outdr,
                lckr: self.gpio[i].lckr,
            }),
            timers: self.timers,
            systick: self.systick,
            rcc: self.rcc,
            usart_cr1: self.usart1.cr1,
            usart_brr: self.usart1.brr,
            usart_sr: self.usart1.sr,
            usart_rx: self.usart1.rx,
            usart_tx: if self.usart1.tx.is_empty() {
                None
            } else {
                Some(self.usart1.tx.clone())
            },
            spi: self.spi,
            i2c: self.i2c,
            afio: self.afio,
            exti: self.exti,
            devices: None,
            pfic_enable: self.pfic.enable,
            pfic_pending: self.pfic.pending,
            pfic_active: self.pfic.active,
            mem: Vec::new(),
            uart_tx: if self.uart_tx.is_empty() {
                None
            } else {
                Some(self.uart_tx.clone())
            },
            faults_len: self.faults.len(),
        }
    }

    fn push_undo(&mut self, undo: Undo) {
        if self.undo.len() == UNDO_LIMIT {
            self.undo.pop_front();
        }
        self.undo.push_back(undo);
    }

    fn bank_byte(&self, bank: MemBank, index: usize) -> u8 {
        match bank {
            MemBank::Flash => self.flash[index],
            MemBank::Sram => self.sram[index],
            MemBank::Mmio => self.mmio[index],
            MemBank::Info => self.info[index],
            MemBank::Pfic => self.pfic.other[index],
            MemBank::Priority => self.pfic.priority[index],
        }
    }

    fn bank_set(&mut self, bank: MemBank, index: usize, val: u8) {
        match bank {
            MemBank::Flash => self.flash[index] = val,
            MemBank::Sram => self.sram[index] = val,
            MemBank::Mmio => self.mmio[index] = val,
            MemBank::Info => self.info[index] = val,
            MemBank::Pfic => self.pfic.other[index] = val,
            MemBank::Priority => self.pfic.priority[index] = val,
        }
    }

    fn assign_byte(&mut self, bank: MemBank, index: usize, val: u8) {
        if self.journaling {
            let old = self.bank_byte(bank, index);
            if old != val {
                self.journal.push(ByteUndo {
                    bank,
                    index: index as u32,
                    old,
                });
            }
        }
        self.bank_set(bank, index, val);
    }

    fn step_inner(&mut self) -> Result<(), Fault> {
        self.now = self.cpu.cycle;
        self.sync_lines();
        // Move the core out so the machine can act as the bus during the instruction.
        let mut cpu = std::mem::take(&mut self.cpu);
        let stepped = cpu.step(self);
        self.cpu = cpu;
        match stepped {
            Ok(Some(fault)) => {
                if self.faults.len() < 40 {
                    self.faults.push(format!(
                        "{} at pc={:08x} mcause={:08x}",
                        fault.name(),
                        self.cpu.mepc,
                        self.cpu.mcause
                    ));
                }
                Ok(())
            }
            Ok(None) => {
                self.systick.tick();
                let tim2_clock = self.rcc.apb1en() & 1 != 0;
                if tim2_clock {
                    self.timers[0].tick();
                }
                if self.rcc.apb1en() & 2 != 0 {
                    self.timers[1].tick();
                }
                if self.rcc.apb1en() & 4 != 0 {
                    self.timers[2].tick();
                }
                self.drain_uart();
                self.poll_uart_devices();
                Ok(())
            }
            Err(f) => {
                self.faults.push(format!("double fault: {}", f.name()));
                Err(f)
            }
        }
    }

    fn drain_uart(&mut self) {
        if !self.usart1.tx.is_empty() {
            let bytes = std::mem::take(&mut self.usart1.tx);
            self.uart_tx.push_str(&String::from_utf8_lossy(&bytes));
            if self.uart_tx.len() > 8000 {
                let drain = self.uart_tx.len() - 8000;
                self.uart_tx.drain(..drain);
            }
        }
    }

    fn sync_lines(&mut self) {
        for (irq, asserted) in self.irq_lines() {
            self.pfic.set_pending(irq, asserted);
        }
    }

    /// Level of every peripheral interrupt line the model drives.
    fn irq_lines(&self) -> [(u32, bool); 18] {
        [
            (self.timers[0].irq, self.timers[0].irq_asserted()),
            (self.timers[1].irq, self.timers[1].irq_asserted()),
            (self.timers[2].irq, self.timers[2].irq_asserted()),
            (IRQ_SYSTICK, self.systick.irq_asserted()),
            (SPI_IRQS[0], self.spi[0].irq_asserted()),
            (SPI_IRQS[1], self.spi[1].irq_asserted()),
            (I2C_EV_IRQS[0], self.i2c[0].ev_irq()),
            (I2C_ER_IRQS[0], self.i2c[0].er_irq()),
            (I2C_EV_IRQS[1], self.i2c[1].ev_irq()),
            (I2C_ER_IRQS[1], self.i2c[1].er_irq()),
            (IRQ_USART1, self.usart1.irq_asserted()),
            (EXTI_IRQS[0].0, self.exti.irq_asserted(EXTI_IRQS[0].1)),
            (EXTI_IRQS[1].0, self.exti.irq_asserted(EXTI_IRQS[1].1)),
            (EXTI_IRQS[2].0, self.exti.irq_asserted(EXTI_IRQS[2].1)),
            (EXTI_IRQS[3].0, self.exti.irq_asserted(EXTI_IRQS[3].1)),
            (EXTI_IRQS[4].0, self.exti.irq_asserted(EXTI_IRQS[4].1)),
            (EXTI_IRQS[5].0, self.exti.irq_asserted(EXTI_IRQS[5].1)),
            (EXTI_IRQS[6].0, self.exti.irq_asserted(EXTI_IRQS[6].1)),
        ]
    }

    fn afio_exti_read(&self, addr: u32) -> Option<u32> {
        if (AFIO_BASE..AFIO_BASE + 0x400).contains(&addr) {
            let offset = addr - AFIO_BASE;
            if (0x08..0x18).contains(&offset) {
                return Some(self.afio.exti_cr[((offset - 0x08) / 4) as usize]);
            }
        }
        if (EXTI_BASE..EXTI_BASE + 0x400).contains(&addr) {
            return Some(self.exti.read(addr - EXTI_BASE));
        }
        None
    }

    fn sample_pin_levels(&mut self) {
        for port in 0..4 {
            for pin in 0..16u8 {
                self.pin_high[port][pin as usize] = self.gpio[port].input_bit(pin);
            }
        }
    }

    /// Set EXTI pending bits for pins whose electrical level just rose or fell.
    fn note_pin_edges(&mut self) {
        let mut hit = 0u32;
        for port in 0..4 {
            for pin in 0..16u8 {
                let high = self.gpio[port].input_bit(pin);
                let was = self.pin_high[port][pin as usize];
                self.pin_high[port][pin as usize] = high;
                if high == was || self.afio.port(pin) != port as u8 {
                    continue;
                }
                let mask = 1u32 << pin;
                let rise = high && (self.exti.rtenr & mask) != 0;
                let fall = !high && (self.exti.ftenr & mask) != 0;
                if rise || fall {
                    hit |= mask;
                }
            }
        }
        self.exti.intfr |= hit;
    }

    pub fn pin(&self, port: usize, pin: u8) -> PinView {
        let clock_bit = match port {
            0 => 1 << 2,
            1 => 1 << 3,
            2 => 1 << 4,
            _ => 1 << 5,
        };
        let g = &self.gpio[port];
        PinView {
            port: b"ABCD"[port] as char,
            pin,
            mode: g.mode_of(pin),
            level: g.level_of(pin),
            analog_v: g.analog_v[pin as usize],
            clock_enabled: self.rcc.apb2en() & clock_bit != 0,
        }
    }

    pub fn pins(&self) -> Vec<PinView> {
        let mut out = Vec::with_capacity(64);
        for port in 0..4 {
            for pin in 0..16 {
                out.push(self.pin(port, pin));
            }
        }
        out
    }

    pub fn set_external(&mut self, port: usize, pin: u8, drive: ExternalDrive, analog_v: f32) {
        if port < 4 && pin < 16 {
            self.gpio[port].external[pin as usize] = drive;
            self.gpio[port].analog_v[pin as usize] = analog_v;
            self.now = self.cpu.cycle;
            self.update_chip_selects();
            self.note_pin_edges();
        }
    }

    pub fn read_ram(&self, addr: u32, len: u32) -> Option<Vec<u8>> {
        let mut buf = vec![0u8; len as usize];
        for (i, b) in buf.iter_mut().enumerate() {
            *b = self.load_bus(addr.wrapping_add(i as u32)).ok()?;
        }
        Some(buf)
    }

    pub fn debug_line(&self) -> String {
        let t = &self.timers[0];
        let vector_addr = (self.cpu.mtvec & !3).wrapping_add(IRQ_TIM2 * 4);
        let vector = (0..4)
            .map(|i| self.load_bus(vector_addr.wrapping_add(i)).unwrap_or(0) as u32)
            .enumerate()
            .fold(0u32, |acc, (i, b)| acc | (b << (8 * i)));
        format!(
            "tim2_vector={:#010x} tim2 cen={} cnt={} psc={} arr={} dier={:#06x} sr={:#06x} ccr=[{}, {}] apb1en={:#x} mtvec={:#010x} mstatus={:#010x} intsyscr={:#x} pfic_en0={:#010x} pfic_en1={:#010x} pend1={:#010x} mie_steps={} irqs={} serving={} hw_frames={} pc={:#010x}",
            vector,
            t.cen() as u8,
            t.cnt,
            t.psc,
            t.arr,
            t.dier,
            t.sr,
            t.ccr[0],
            t.ccr[1],
            self.rcc.apb1en(),
            self.cpu.mtvec,
            self.cpu.mstatus,
            self.cpu.intsyscr,
            self.pfic.enable[0],
            self.pfic.enable[1],
            self.pfic.pending[1],
            self.cpu.mie_steps,
            self.cpu.irqs_taken,
            self.cpu.serving_len(),
            self.cpu.hw_frames(),
            self.cpu.pc,
        )
    }

    /// Assembly at `pc`, and what that instruction will do with the current registers.
    pub fn current_insn(&self) -> (String, String) {
        let pc = self.cpu.pc;
        let regs = self.cpu.x;
        let mem = |addr: u32, size: usize| self.peek_mem(addr, size);
        match self.load16_bus(pc) {
            Ok(half) if half & 0b11 != 0b11 => crate::cpu::explain_c(half, pc, &regs, &mem),
            Ok(half) => match self.load16_bus(pc.wrapping_add(2)) {
                Ok(hi) => {
                    let word = (half as u32) | ((hi as u32) << 16);
                    let csr = |id: u16| self.cpu.read_csr(id);
                    crate::cpu::explain32(word, pc, &regs, &csr, &mem)
                }
                Err(_) => (
                    format!(".half {half:04x}"),
                    "instruction fetch failed".into(),
                ),
            },
            Err(_) => ("???".into(), "instruction fetch failed".into()),
        }
    }

    fn peek_mem(&self, addr: u32, size: usize) -> Option<u32> {
        if !(1..=4).contains(&size) {
            return None;
        }
        let mut acc = 0u32;
        for i in 0..size {
            let byte = self.peek_byte(addr.wrapping_add(i as u32))?;
            acc |= (byte as u32) << (8 * i);
        }
        Some(acc)
    }

    fn peek_byte(&self, addr: u32) -> Option<u8> {
        if let Some(word) = self.peek_mmio_word(addr & !3) {
            return Some(word.to_le_bytes()[(addr & 3) as usize]);
        }
        self.load_bus(addr).ok()
    }

    /// Functional register value a load would see, without changing machine state.
    fn peek_mmio_word(&self, addr: u32) -> Option<u32> {
        let addr = addr & !3;
        for (i, &base) in GPIO_BASES.iter().enumerate() {
            if (base..base + 0x400).contains(&addr) {
                return Some(self.gpio[i].read(addr - base));
            }
        }
        for (i, &base) in TIM_BASES.iter().enumerate() {
            if (base..base + 0x400).contains(&addr) {
                return Some(self.timers[i].read(addr - base));
            }
        }
        if let Some(word) = self.afio_exti_read(addr) {
            return Some(word);
        }
        if (RCC_BASE..RCC_BASE + 0x40).contains(&addr) {
            let mut rcc = self.rcc;
            rcc.sync();
            return Some(rcc.regs[((addr - RCC_BASE) / 4) as usize]);
        }
        if (USART1_BASE..USART1_BASE + 0x20).contains(&addr) {
            return Some(self.usart1.read(addr - USART1_BASE));
        }
        for (i, &base) in SPI_BASES.iter().enumerate() {
            if (base..base + 0x400).contains(&addr) {
                let on = self.spi_clock(i);
                return Some(if on { self.spi[i].read(addr - base) } else { 0 });
            }
        }
        for (i, &base) in I2C_BASES.iter().enumerate() {
            if (base..base + 0x400).contains(&addr) {
                let on = self.i2c_clock(i);
                return Some(if on { self.i2c[i].read(addr - base) } else { 0 });
            }
        }
        if (SYSTICK_BASE..SYSTICK_BASE + 0x20).contains(&addr) {
            return Some(self.systick.read(addr - SYSTICK_BASE));
        }
        if (PFIC_BASE..PFIC_BASE + PFIC_SIZE as u32).contains(&addr) {
            return Some(self.pfic.read_word(addr - PFIC_BASE));
        }
        None
    }

    fn write_bytes(&mut self, addr: u32, data: &[u8]) -> Result<(), Fault> {
        for (i, b) in data.iter().enumerate() {
            self.store_bus(addr.wrapping_add(i as u32), *b)?;
        }
        Ok(())
    }

    fn load_bus(&self, addr: u32) -> Result<u8, Fault> {
        if addr < FLASH_SIZE as u32 {
            return Ok(self.flash[addr as usize]);
        }
        if (FLASH_BASE..FLASH_BASE + FLASH_SIZE as u32).contains(&addr) {
            return Ok(self.flash[(addr - FLASH_BASE) as usize]);
        }
        if (SRAM_BASE..SRAM_BASE + SRAM_SIZE as u32).contains(&addr) {
            return Ok(self.sram[(addr - SRAM_BASE) as usize]);
        }
        if (INFO_BASE..INFO_BASE + INFO_SIZE as u32).contains(&addr) {
            return Ok(self.info[(addr - INFO_BASE) as usize]);
        }
        if (MMIO_BASE..MMIO_BASE + MMIO_SIZE as u32).contains(&addr) {
            return Ok(self.mmio[(addr - MMIO_BASE) as usize]);
        }
        if (PFIC_BASE..PFIC_BASE + PFIC_SIZE as u32).contains(&addr) {
            let off = addr - PFIC_BASE;
            let word = self.pfic.read_word(off & !3);
            return Ok(word.to_le_bytes()[(off & 3) as usize]);
        }
        if (SYSTICK_BASE..SYSTICK_BASE + 0x20).contains(&addr) {
            let off = addr - SYSTICK_BASE;
            let word = self.systick.read(off & !3);
            return Ok(word.to_le_bytes()[(off & 3) as usize]);
        }
        Err(Fault::LoadFault)
    }

    fn store_bus(&mut self, addr: u32, val: u8) -> Result<(), Fault> {
        if addr < FLASH_SIZE as u32 || (FLASH_BASE..FLASH_BASE + FLASH_SIZE as u32).contains(&addr)
        {
            let off = if addr < FLASH_SIZE as u32 {
                addr as usize
            } else {
                (addr - FLASH_BASE) as usize
            };
            self.assign_byte(MemBank::Flash, off, val);
            return Ok(());
        }
        if (SRAM_BASE..SRAM_BASE + SRAM_SIZE as u32).contains(&addr) {
            self.assign_byte(MemBank::Sram, (addr - SRAM_BASE) as usize, val);
            return Ok(());
        }
        if (INFO_BASE..INFO_BASE + INFO_SIZE as u32).contains(&addr) {
            self.assign_byte(MemBank::Info, (addr - INFO_BASE) as usize, val);
            return Ok(());
        }
        if (MMIO_BASE..MMIO_BASE + MMIO_SIZE as u32).contains(&addr) {
            self.assign_byte(MemBank::Mmio, (addr - MMIO_BASE) as usize, val);
            return Ok(());
        }
        if (PFIC_BASE..PFIC_BASE + PFIC_SIZE as u32).contains(&addr) {
            let off = (addr - PFIC_BASE) as usize;
            self.assign_byte(MemBank::Pfic, off, val);
            if (0x400..0x500).contains(&off) {
                self.assign_byte(MemBank::Priority, off - 0x400, val);
            }
            return Ok(());
        }
        if (SYSTICK_BASE..SYSTICK_BASE + 0x20).contains(&addr) {
            // Byte writes are rare. Merge into a word write.
            let off = (addr - SYSTICK_BASE) & !3;
            let mut word = self.systick.read(off).to_le_bytes();
            word[(addr & 3) as usize] = val;
            self.systick.write(off, u32::from_le_bytes(word));
            return Ok(());
        }
        Err(Fault::StoreFault)
    }

    fn load16_bus(&self, addr: u32) -> Result<u16, Fault> {
        Ok(u16::from_le_bytes([
            self.load_bus(addr)?,
            self.load_bus(addr.wrapping_add(1))?,
        ]))
    }

    fn load32_bus(&mut self, addr: u32) -> Result<u32, Fault> {
        if let Some(word) = self.mmio_word(addr) {
            return Ok(word);
        }
        Ok(u32::from_le_bytes([
            self.load_bus(addr)?,
            self.load_bus(addr.wrapping_add(1))?,
            self.load_bus(addr.wrapping_add(2))?,
            self.load_bus(addr.wrapping_add(3))?,
        ]))
    }

    fn store32_bus(&mut self, addr: u32, val: u32) -> Result<(), Fault> {
        if self.mmio_store(addr, val) {
            return Ok(());
        }
        for (i, b) in val.to_le_bytes().into_iter().enumerate() {
            self.store_bus(addr.wrapping_add(i as u32), b)?;
        }
        Ok(())
    }

    fn mmio_word(&mut self, addr: u32) -> Option<u32> {
        let addr = addr & !3;
        for (i, &base) in GPIO_BASES.iter().enumerate() {
            if (base..base + 0x400).contains(&addr) {
                return Some(self.gpio[i].read(addr - base));
            }
        }
        for (i, &base) in TIM_BASES.iter().enumerate() {
            if (base..base + 0x400).contains(&addr) {
                return Some(self.timers[i].read(addr - base));
            }
        }
        if let Some(word) = self.afio_exti_read(addr) {
            return Some(word);
        }
        if (RCC_BASE..RCC_BASE + 0x40).contains(&addr) {
            self.rcc.sync();
            return Some(self.rcc.regs[((addr - RCC_BASE) / 4) as usize]);
        }
        if (USART1_BASE..USART1_BASE + 0x20).contains(&addr) {
            if addr - USART1_BASE == 0x04 {
                self.usart1.sr &= !USART_RXNE;
            }
            return Some(self.usart1.read(addr - USART1_BASE));
        }
        for (i, &base) in SPI_BASES.iter().enumerate() {
            if (base..base + 0x400).contains(&addr) {
                if !self.spi_clock(i) {
                    return Some(0);
                }
                let offset = addr - base;
                return Some(if offset == crate::serial::SPI_DATAR {
                    self.spi[i].read_data()
                } else {
                    self.spi[i].read(offset)
                });
            }
        }
        for (i, &base) in I2C_BASES.iter().enumerate() {
            if (base..base + 0x400).contains(&addr) {
                if !self.i2c_clock(i) {
                    return Some(0);
                }
                let (value, action) = self.i2c[i].load(addr - base);
                self.i2c_act(i, action);
                return Some(value);
            }
        }
        if (SYSTICK_BASE..SYSTICK_BASE + 0x20).contains(&addr) {
            return Some(self.systick.read(addr - SYSTICK_BASE));
        }
        if (PFIC_BASE..PFIC_BASE + PFIC_SIZE as u32).contains(&addr) {
            return Some(self.pfic.read_word(addr - PFIC_BASE));
        }
        None
    }

    fn mmio_store(&mut self, addr: u32, val: u32) -> bool {
        let addr = addr & !3;
        for (i, &base) in GPIO_BASES.iter().enumerate() {
            if (base..base + 0x400).contains(&addr) {
                self.gpio[i].write(addr - base, val);
                self.update_chip_selects();
                self.note_pin_edges();
                return true;
            }
        }
        if (AFIO_BASE..AFIO_BASE + 0x400).contains(&addr) {
            let offset = addr - AFIO_BASE;
            if (0x08..0x18).contains(&offset) {
                self.afio.exti_cr[((offset - 0x08) / 4) as usize] = val;
                return true;
            }
            return false;
        }
        if (EXTI_BASE..EXTI_BASE + 0x400).contains(&addr) {
            self.exti.write(addr - EXTI_BASE, val);
            return true;
        }
        for (i, &base) in SPI_BASES.iter().enumerate() {
            if (base..base + 0x400).contains(&addr) {
                if self.spi_clock(i) {
                    if let Some(frame) = self.spi[i].write(addr - base, val) {
                        let rx = self.spi_exchange(i, frame);
                        self.spi[i].complete(frame, rx);
                    }
                }
                return true;
            }
        }
        for (i, &base) in I2C_BASES.iter().enumerate() {
            if (base..base + 0x400).contains(&addr) {
                if self.i2c_clock(i) {
                    let action = self.i2c[i].write(addr - base, val);
                    self.i2c_act(i, action);
                }
                return true;
            }
        }
        for (i, &base) in TIM_BASES.iter().enumerate() {
            if (base..base + 0x400).contains(&addr) {
                self.timers[i].write(addr - base, val);
                return true;
            }
        }
        if (RCC_BASE..RCC_BASE + 0x40).contains(&addr) {
            let idx = ((addr - RCC_BASE) / 4) as usize;
            if idx == 4 {
                // APB1PRSTR. A set bit resets that timer.
                if val & 1 != 0 {
                    self.timers[0] = Timer::new(IRQ_TIM2);
                }
                if val & 2 != 0 {
                    self.timers[1] = Timer::new(IRQ_TIM3);
                }
                if val & 4 != 0 {
                    self.timers[2] = Timer::new(IRQ_TIM4);
                }
                if val & (1 << 14) != 0 {
                    self.spi[1] = Spi::new();
                }
                for bus in 0..2 {
                    if val & (1 << (21 + bus)) != 0 {
                        if let Some(address) = self.i2c[bus].target {
                            self.i2c_act(bus, I2cAction::Stop(address));
                        }
                        self.i2c[bus] = I2c::new();
                    }
                }
            }
            if idx == 3 {
                if val & (1 << 2) != 0 {
                    self.gpio[0] = GpioPort::new();
                }
                if val & (1 << 3) != 0 {
                    self.gpio[1] = GpioPort::new();
                }
                if val & (1 << 4) != 0 {
                    self.gpio[2] = GpioPort::new();
                }
                if val & (1 << 5) != 0 {
                    self.gpio[3] = GpioPort::new();
                }
                if val & (1 << 12) != 0 {
                    self.spi[0] = Spi::new();
                }
                if val & (1 << 14) != 0 {
                    let tx = std::mem::take(&mut self.usart1.tx);
                    self.usart1 = Usart::new();
                    self.usart1.tx = tx;
                }
                if val & 1 != 0 {
                    self.afio = Afio::new();
                }
                self.update_chip_selects();
                self.note_pin_edges();
            }
            self.rcc.regs[idx] = val;
            self.rcc.sync();
            return true;
        }
        if (USART1_BASE..USART1_BASE + 0x20).contains(&addr) {
            self.usart1.write(addr - USART1_BASE, val);
            if addr - USART1_BASE == 0x04 {
                self.uart_send(val as u8);
            }
            return true;
        }
        if (SYSTICK_BASE..SYSTICK_BASE + 0x20).contains(&addr) {
            self.systick.write(addr - SYSTICK_BASE, val);
            return true;
        }
        if (PFIC_BASE..PFIC_BASE + PFIC_SIZE as u32).contains(&addr) {
            let offset = addr - PFIC_BASE;
            if self.journaling && (0x400..0x500).contains(&offset) {
                let start = (offset - 0x400) as usize;
                let bytes = val.to_le_bytes();
                for i in 0..4 {
                    if start + i >= self.pfic.priority.len() {
                        break;
                    }
                    let old = self.pfic.priority[start + i];
                    if old != bytes[i] {
                        self.journal.push(ByteUndo {
                            bank: MemBank::Priority,
                            index: (start + i) as u32,
                            old,
                        });
                    }
                }
            } else if self.journaling && pfic_write_touches_other(offset) {
                let start = offset as usize;
                if start + 4 <= self.pfic.other.len() {
                    let bytes = val.to_le_bytes();
                    for i in 0..4 {
                        let old = self.pfic.other[start + i];
                        if old != bytes[i] {
                            self.journal.push(ByteUndo {
                                bank: MemBank::Pfic,
                                index: (start + i) as u32,
                                old,
                            });
                        }
                    }
                }
            }
            self.pfic.write_word(offset, val);
            return true;
        }
        false
    }

    /// Every modeled peripheral, in menu order.
    pub fn peripherals(&self) -> Vec<PeripheralView> {
        let mut out = Vec::with_capacity(11);
        for index in 0..3 {
            out.push(self.timer_view(index));
        }
        for port in 0..4 {
            out.push(self.gpio_view(port));
        }
        out.push(self.rcc_view());
        out.push(self.systick_view());
        out.push(self.usart_view());
        for bus in 0..2 {
            out.push(self.spi_view(bus));
        }
        for bus in 0..2 {
            out.push(self.i2c_view(bus));
        }
        out.push(self.exti_view());
        out.push(self.pfic_view());
        for attached in &self.devices {
            out.push(self.device_view(attached));
        }
        out
    }

    fn devices_on(&self, bus: &str) -> Vec<&Attached> {
        self.devices
            .iter()
            .filter(|d| d.bus_name() == bus)
            .collect()
    }

    fn spi_view(&self, bus: usize) -> PeripheralView {
        use crate::serial::{
            SPI_CTLR1, SPI_CTLR2, SPI_DATAR, SPI_OVR, SPI_RXNE, SPI_STATR, SPI_TXE,
        };
        let spi = &self.spi[bus];
        let base = SPI_BASES[bus];
        let id = format!("SPI{}", bus + 1);
        let clock = self.spi_clock(bus);
        let attached = self.devices_on(&id);
        let names: Vec<String> = attached
            .iter()
            .map(|d| match &d.link {
                Link::Spi { cs, selected, .. } => {
                    let cs = cs.map_or("no CS".to_string(), |(p, n)| {
                        format!("CS {}", pin_name(p, n))
                    });
                    let state = if *selected { "selected" } else { "idle" };
                    format!("{} ({cs}, {state})", d.id)
                }
                _ => d.id.clone(),
            })
            .collect();
        let selected: Vec<&str> = attached
            .iter()
            .filter(|d| matches!(d.link, Link::Spi { selected: true, .. }))
            .map(|d| d.id.as_str())
            .collect();
        let width = if spi.sixteen_bit() { 16 } else { 8 };
        let action = if !clock {
            format!("{id} clock is off (RCC). Register reads return 0 and writes are ignored.")
        } else if !spi.enabled_master() {
            format!("{id} is not enabled as master (SPE and MSTR in CTLR1). A store to DATAR does nothing.")
        } else if selected.is_empty() {
            format!(
                "Idle. A store to DATAR at {:#010x} clocks out one {width}-bit frame. No component is selected, so MISO reads 0xff.",
                base + SPI_DATAR
            )
        } else {
            format!(
                "Idle. A store to DATAR at {:#010x} clocks out one {width}-bit frame to {} and the reply lands in DATAR with RXNE set, in the same instruction.",
                base + SPI_DATAR,
                selected.join(", ")
            )
        };
        let flag = |mask: u32| u8::from(spi.statr & mask != 0);
        let inputs = vec![
            signal(
                "clock enable",
                Some(RCC_BASE + if bus == 0 { 0x18 } else { 0x1c }),
                format!("{}", u8::from(clock)),
            ),
            signal(
                "CTLR1",
                Some(base + SPI_CTLR1),
                format!(
                    "{:#06x} (SPE {}, MSTR {}, mode {}, {}-bit, {} first, BR /{})",
                    spi.ctlr1,
                    u8::from(spi.ctlr1 & (1 << 6) != 0),
                    u8::from(spi.ctlr1 & (1 << 2) != 0),
                    spi.mode(),
                    width,
                    if spi.lsb_first() { "LSB" } else { "MSB" },
                    2u32 << ((spi.ctlr1 >> 3) & 7)
                ),
            ),
            signal(
                "CTLR2",
                Some(base + SPI_CTLR2),
                format!("{:#06x}", spi.ctlr2),
            ),
            signal(
                "STATR",
                Some(base + SPI_STATR),
                format!(
                    "{:#06x} (RXNE {}, TXE {}, OVR {})",
                    spi.statr,
                    flag(SPI_RXNE),
                    flag(SPI_TXE),
                    flag(SPI_OVR)
                ),
            ),
            signal(
                "components",
                None,
                if names.is_empty() {
                    "(none attached)".into()
                } else {
                    names.join(", ")
                },
            ),
        ];
        let outputs = vec![
            signal(
                "DATAR (received)",
                Some(base + SPI_DATAR),
                format!("{:#06x}", spi.rx),
            ),
            signal(
                "last frame",
                None,
                match (spi.last_tx, spi.last_rx) {
                    (Some(tx), Some(rx)) => format!("MOSI {tx:#06x}, MISO {rx:#06x}"),
                    _ => "(none yet)".into(),
                },
            ),
            signal("frames", None, spi.frames.to_string()),
            signal(
                "IRQ line",
                None,
                format!(
                    "{} = {}",
                    irq_label(SPI_IRQS[bus]),
                    u8::from(spi.irq_asserted())
                ),
            ),
        ];
        PeripheralView {
            id,
            group: "SPI".into(),
            action,
            inputs,
            outputs,
            effects: Vec::new(),
        }
    }

    fn i2c_view(&self, bus: usize) -> PeripheralView {
        use crate::serial::{
            I2C_ACK, I2C_ADDR, I2C_AF, I2C_BTF, I2C_CTLR1, I2C_CTLR2, I2C_DATAR, I2C_RXNE, I2C_SB,
            I2C_STAR1, I2C_STAR2, I2C_TXE,
        };
        let i2c = &self.i2c[bus];
        let base = I2C_BASES[bus];
        let id = format!("I2C{}", bus + 1);
        let clock = self.i2c_clock(bus);
        let attached = self.devices_on(&id);
        let names: Vec<String> = attached
            .iter()
            .map(|d| match &d.link {
                Link::I2c { dev, .. } => format!("{} at {:#04x}", d.id, dev.address()),
                _ => d.id.clone(),
            })
            .collect();
        let target = i2c.target.map(|a| format!("{a:#04x}"));
        let action = if !clock {
            format!("{id} clock is off (RCC). Register reads return 0 and writes are ignored.")
        } else {
            match i2c.phase {
                I2cPhase::Idle => format!("Bus idle. Setting START in CTLR1 at {:#010x} begins a transaction.", base + I2C_CTLR1),
                I2cPhase::Address => format!(
                    "START sent (SB set). The next store to DATAR at {:#010x} is the address byte.",
                    base + I2C_DATAR
                ),
                I2cPhase::Nacked => "The target did not acknowledge (AF set). Firmware should send STOP.".into(),
                I2cPhase::Transmit => format!(
                    "Writing to {}. A store to DATAR sends one byte and sets TXE and BTF when it is acknowledged.",
                    target.clone().unwrap_or_default()
                ),
                I2cPhase::Receive => format!(
                    "Reading from {}. Each DATAR load takes a byte; with ACK set the next byte is fetched right away.",
                    target.clone().unwrap_or_default()
                ),
            }
        };
        let flag = |mask: u32| u8::from(i2c.star1 & mask != 0);
        let inputs = vec![
            signal(
                "clock enable",
                Some(RCC_BASE + 0x1c),
                format!("{}", u8::from(clock)),
            ),
            signal(
                "CTLR1",
                Some(base + I2C_CTLR1),
                format!(
                    "{:#06x} (PE {}, ACK {})",
                    i2c.ctlr1,
                    i2c.ctlr1 & 1,
                    u8::from(i2c.ctlr1 & I2C_ACK != 0)
                ),
            ),
            signal(
                "CTLR2",
                Some(base + I2C_CTLR2),
                format!("{:#06x}", i2c.ctlr2),
            ),
            signal(
                "components",
                None,
                if names.is_empty() {
                    "(none attached)".into()
                } else {
                    names.join(", ")
                },
            ),
        ];
        let outputs = vec![
            signal(
                "STAR1",
                Some(base + I2C_STAR1),
                format!(
                    "{:#06x} (SB {}, ADDR {}, BTF {}, RxNE {}, TxE {}, AF {})",
                    i2c.star1,
                    flag(I2C_SB),
                    flag(I2C_ADDR),
                    flag(I2C_BTF),
                    flag(I2C_RXNE),
                    flag(I2C_TXE),
                    flag(I2C_AF)
                ),
            ),
            signal(
                "STAR2",
                Some(base + I2C_STAR2),
                format!("{:#06x}", i2c.star2),
            ),
            signal(
                "DATAR",
                Some(base + I2C_DATAR),
                format!("{:#04x}", i2c.data),
            ),
            signal("addressed", None, target.unwrap_or_else(|| "(none)".into())),
            signal("bytes", None, i2c.bytes.to_string()),
        ];
        PeripheralView {
            id,
            group: "I2C".into(),
            action,
            inputs,
            outputs,
            effects: Vec::new(),
        }
    }

    fn device_view(&self, attached: &Attached) -> PeripheralView {
        let view = attached.view(&DeviceCtx {
            cycle: self.cpu.cycle,
        });
        let mut wiring = format!("On {}", attached.bus_name());
        if let Some(cs) = attached.cs_name() {
            wiring.push_str(&format!(", chip select {cs}"));
        }
        let mut inputs = vec![signal("bus", None, wiring.clone())];
        inputs.extend(
            view.inputs
                .into_iter()
                .map(|r| signal(r.name, None, r.value)),
        );
        PeripheralView {
            id: attached.id.clone(),
            group: "Devices".into(),
            action: format!("{wiring}. {}", view.action),
            inputs,
            outputs: view
                .outputs
                .into_iter()
                .map(|r| signal(r.name, None, r.value))
                .collect(),
            effects: Vec::new(),
        }
    }

    fn timer_view(&self, index: usize) -> PeripheralView {
        let timer = &self.timers[index];
        let base = TIM_BASES[index];
        let id = ["TIM2", "TIM3", "TIM4"][index];
        let clock = self.rcc.apb1en() & (1 << index) != 0;
        let cnt_addr = base + 0x24;
        let mut next = timer.clone();
        if clock {
            next.tick();
        }
        let mut effects = Vec::new();
        if timer.presc != next.presc {
            effects.push(MemEffect {
                name: "prescaler counter".into(),
                address: None,
                old: u64::from(timer.presc),
                new: u64::from(next.presc),
                bits: "internal, not memory-mapped".into(),
            });
        }
        push_effect(
            &mut effects,
            "CNT",
            Some(cnt_addr),
            u64::from(timer.cnt),
            u64::from(next.cnt),
            "[15:0]".into(),
        );
        push_effect(
            &mut effects,
            "INTFR",
            Some(base + 0x10),
            u64::from(timer.sr),
            u64::from(next.sr),
            changed_bits(timer.sr, next.sr, timer_flag_name),
        );
        let mut action = if !clock {
            format!("{id} clock is off (APB1PCENR bit {index} is 0), so this step leaves it alone.")
        } else if !timer.cen() {
            format!("{id} counter is stopped (CEN = 0).")
        } else if timer.presc >= timer.psc && timer.cnt == timer.arr {
            format!(
                "This step resets the counter at {cnt_addr:#010x} from {} to 0.",
                timer.cnt
            )
        } else if timer.presc >= timer.psc {
            format!(
                "This step increments the counter at {cnt_addr:#010x} from {} to {}.",
                timer.cnt,
                timer.cnt.wrapping_add(1)
            )
        } else {
            format!(
                "This step advances the prescaler from {} to {}. The counter at {cnt_addr:#010x} stays {}.",
                timer.presc,
                timer.presc.wrapping_add(1),
                timer.cnt
            )
        };
        if clock && timer.irq_asserted() != next.irq_asserted() {
            if let Some((addr, bit)) = pfic_pending_bit(timer.irq) {
                let going = if next.irq_asserted() { "1" } else { "0" };
                action.push_str(&format!(
                    " The {id} IRQ line becomes {going}; PFIC pending bit {bit} at {addr:#010x} is written at the start of the next step."
                ));
            }
        }
        let (pend_addr, pend_bit) = pfic_pending_bit(timer.irq).unwrap_or((0, 0));
        let pending = pfic_pending_bit(timer.irq)
            .map(|(_, bit)| self.pfic.pending[(timer.irq / 32) as usize] & (1 << bit) != 0)
            .unwrap_or(false);
        PeripheralView {
            id: id.into(),
            group: "TIM".into(),
            action,
            inputs: vec![
                signal(
                    "TIMEN",
                    Some(RCC_BASE + 0x1c),
                    format!("{} (APB1PCENR bit {index})", u8::from(clock)),
                ),
                signal("CTLR1", Some(base), format!("{:#010x}", timer.ctlr1)),
                signal("CEN", Some(base), u8::from(timer.cen()).to_string()),
                signal(
                    "URS",
                    Some(base),
                    u8::from(timer.urs_counter_only()).to_string(),
                ),
                signal("PSC", Some(base + 0x28), timer.psc.to_string()),
                signal("prescaler counter", None, timer.presc.to_string()),
                signal("ATRLR", Some(base + 0x2c), timer.arr.to_string()),
                signal("CNT", Some(cnt_addr), timer.cnt.to_string()),
                signal(
                    "DMAINTENR",
                    Some(base + 0x0c),
                    format!(
                        "{:#06x} ({})",
                        timer.dier,
                        set_bits(timer.dier, timer_ie_name)
                    ),
                ),
                signal(
                    "INTFR",
                    Some(base + 0x10),
                    format!(
                        "{:#06x} ({})",
                        timer.sr,
                        set_bits(timer.sr, timer_flag_name)
                    ),
                ),
                signal("CH1CVR", Some(base + 0x34), timer.ccr[0].to_string()),
                signal("CH2CVR", Some(base + 0x38), timer.ccr[1].to_string()),
                signal("CH3CVR", Some(base + 0x3c), timer.ccr[2].to_string()),
                signal("CH4CVR", Some(base + 0x40), timer.ccr[3].to_string()),
            ],
            outputs: vec![signal(
                "IRQ pending",
                Some(pend_addr),
                format!("bit {pend_bit} = {}", u8::from(pending)),
            )],
            effects,
        }
    }

    fn gpio_view(&self, port: usize) -> PeripheralView {
        let gpio = &self.gpio[port];
        let base = GPIO_BASES[port];
        let letter = b"ABCD"[port] as char;
        let id = format!("GPIO{letter}");
        let clock_bit = 2 + port;
        let clock = self.rcc.apb2en() & (1 << clock_bit) != 0;
        let mut inputs = vec![
            signal(
                "clock",
                Some(RCC_BASE + 0x18),
                format!("{} (APB2PCENR bit {clock_bit})", u8::from(clock)),
            ),
            signal("CFGLR", Some(base), format!("{:#010x}", gpio.cfglr)),
            signal("CFGHR", Some(base + 0x04), format!("{:#010x}", gpio.cfghr)),
            signal("OUTDR", Some(base + 0x0c), format!("{:#06x}", gpio.outdr)),
            signal("LCKR", Some(base + 0x18), format!("{:#010x}", gpio.lckr)),
        ];
        let mut outputs = vec![signal(
            "INDR",
            Some(base + 0x08),
            format!("{:#06x}", gpio.indr()),
        )];
        for pin in 0..16u8 {
            inputs.push(signal(
                format!("P{letter}{pin} external"),
                None,
                format!(
                    "{}, {:.2} V",
                    drive_name(gpio.external[pin as usize]),
                    gpio.analog_v[pin as usize]
                ),
            ));
            let odr = u8::from((gpio.outdr >> pin) & 1 == 1);
            outputs.push(signal(
                format!("P{letter}{pin}"),
                Some(base + 0x0c),
                format!(
                    "{} · {} · OUTDR bit {pin} = {odr}",
                    level_name(gpio.level_of(pin)),
                    mode_name(gpio.mode_of(pin))
                ),
            ));
        }
        PeripheralView {
            id,
            group: "GPIO".into(),
            action: format!(
                "This step does not change GPIO{letter} registers. Pin levels follow CFGLR, CFGHR, OUTDR, and the external drive. INDR at {:#010x} is {:#06x}.",
                base + 0x08,
                gpio.indr()
            ),
            inputs,
            outputs,
            effects: Vec::new(),
        }
    }

    fn rcc_view(&self) -> PeripheralView {
        let mut next = self.rcc.clone();
        next.sync();
        let mut effects = Vec::new();
        push_effect(
            &mut effects,
            "CTLR",
            Some(RCC_BASE),
            u64::from(self.rcc.regs[0]),
            u64::from(next.regs[0]),
            changed_bits(self.rcc.regs[0], next.regs[0], rcc_ctlr_name),
        );
        push_effect(
            &mut effects,
            "CFGR",
            Some(RCC_BASE + 0x04),
            u64::from(self.rcc.regs[1]),
            u64::from(next.regs[1]),
            changed_bits(self.rcc.regs[1], next.regs[1], rcc_cfgr_name),
        );
        let action = if effects.is_empty() {
            "Clock ready bits match the control registers. This step does not change RCC.".into()
        } else {
            "Ready bits are stale. The next read or write of RCC will update them; other instructions leave RCC alone.".into()
        };
        let names = [
            (0x00, "CTLR"),
            (0x04, "CFGR"),
            (0x08, "INTR"),
            (0x0c, "APB2PRSTR"),
            (0x10, "APB1PRSTR"),
            (0x14, "AHBPCENR"),
            (0x18, "APB2PCENR"),
            (0x1c, "APB1PCENR"),
        ];
        let inputs = names
            .iter()
            .map(|(off, name)| {
                let idx = (*off / 4) as usize;
                signal(
                    *name,
                    Some(RCC_BASE + off),
                    format!("{:#010x}", self.rcc.regs[idx]),
                )
            })
            .collect();
        PeripheralView {
            id: "RCC".into(),
            group: "RCC".into(),
            action,
            inputs,
            outputs: vec![
                signal(
                    "CTLR",
                    Some(RCC_BASE),
                    format!("{:#010x}", self.rcc.regs[0]),
                ),
                signal(
                    "CFGR",
                    Some(RCC_BASE + 0x04),
                    format!("{:#010x}", self.rcc.regs[1]),
                ),
            ],
            effects,
        }
    }

    fn systick_view(&self) -> PeripheralView {
        let current = &self.systick;
        let mut next = current.clone();
        next.tick();
        let mut effects = Vec::new();
        for (offset, name) in [(0x08, "CNTL"), (0x0c, "CNTH"), (0x04, "SR")] {
            push_effect(
                &mut effects,
                name,
                Some(SYSTICK_BASE + offset),
                u64::from(current.read(offset)),
                u64::from(next.read(offset)),
                if name == "SR" {
                    changed_bits(current.sr, next.sr, systick_sr_name)
                } else {
                    "[31:0]".into()
                },
            );
        }
        let enabled = current.ctlr & 1 != 0;
        let down = current.ctlr & (1 << 4) != 0;
        let reloading = enabled
            && ((down && current.cnt == 0)
                || (!down
                    && current.cmp != 0
                    && current.cnt.wrapping_add(1) >= current.cmp
                    && current.ctlr & (1 << 3) != 0));
        let mut action = if !enabled {
            "SysTick is stopped (STE = 0).".to_string()
        } else if reloading {
            format!(
                "This step reloads the counter (low {:#010x}, high {:#010x}).",
                SYSTICK_BASE + 0x08,
                SYSTICK_BASE + 0x0c
            )
        } else if effects
            .iter()
            .any(|effect| effect.name == "CNTL" || effect.name == "CNTH")
        {
            "This step updates the SysTick counter.".to_string()
        } else {
            "SysTick does not change registers this step.".to_string()
        };
        if current.irq_asserted() != next.irq_asserted() {
            if let Some((addr, bit)) = pfic_pending_bit(IRQ_SYSTICK) {
                let going = if next.irq_asserted() { "1" } else { "0" };
                action.push_str(&format!(
                    " The SysTick IRQ line becomes {going}; PFIC pending bit {bit} at {addr:#010x} is written at the start of the next step."
                ));
            }
        }
        let (pend_addr, pend_bit) = pfic_pending_bit(IRQ_SYSTICK).unwrap_or((0, 0));
        let pending = self.pfic.pending[(IRQ_SYSTICK / 32) as usize] & (1 << pend_bit) != 0;
        PeripheralView {
            id: "SysTick".into(),
            group: "SysTick".into(),
            action,
            inputs: vec![
                signal(
                    "CTLR",
                    Some(SYSTICK_BASE),
                    format!("{:#010x}", current.ctlr),
                ),
                signal("STE", Some(SYSTICK_BASE), u8::from(enabled).to_string()),
                signal(
                    "STIE",
                    Some(SYSTICK_BASE),
                    u8::from(current.ctlr & 2 != 0).to_string(),
                ),
                signal(
                    "STRE",
                    Some(SYSTICK_BASE),
                    u8::from(current.ctlr & (1 << 3) != 0).to_string(),
                ),
                signal("MODE", Some(SYSTICK_BASE), if down { "down" } else { "up" }),
                signal("CNT", Some(SYSTICK_BASE + 0x08), current.cnt.to_string()),
                signal("CMP", Some(SYSTICK_BASE + 0x10), current.cmp.to_string()),
                signal(
                    "SR",
                    Some(SYSTICK_BASE + 0x04),
                    format!("{:#x}", current.sr),
                ),
            ],
            outputs: vec![signal(
                "IRQ pending",
                Some(pend_addr),
                format!("bit {pend_bit} = {}", u8::from(pending)),
            )],
            effects,
        }
    }

    fn usart_view(&self) -> PeripheralView {
        let usart = &self.usart1;
        let mut effects = Vec::new();
        if !usart.tx.is_empty() {
            effects.push(MemEffect {
                name: "UART log".into(),
                address: None,
                old: self.uart_tx.len() as u64,
                new: (self.uart_tx.len() + usart.tx.len()) as u64,
                bits: format!(
                    "appends {} byte(s) from DATAR at {:#010x}",
                    usart.tx.len(),
                    USART1_BASE + 0x04
                ),
            });
        }
        let pending = if usart.tx.is_empty() {
            "USART1 does not transmit on its own. A store to DATAR at 0x40013804 appends a byte to the log."
                .to_string()
        } else {
            format!(
                "This step appends {} byte(s) to the UART log.",
                usart.tx.len()
            )
        };
        PeripheralView {
            id: "USART1".into(),
            group: "USART".into(),
            action: pending,
            inputs: vec![
                signal("STATR", Some(USART1_BASE), format!("{:#010x}", usart.sr)),
                signal("BRR", Some(USART1_BASE + 0x08), format!("{:#x}", usart.brr)),
                signal(
                    "CTLR1",
                    Some(USART1_BASE + 0x0c),
                    format!("{:#010x}", usart.cr1),
                ),
                signal("components", None, {
                    let names: Vec<&str> = self
                        .devices_on("USART1")
                        .iter()
                        .map(|d| d.id.as_str())
                        .collect();
                    if names.is_empty() {
                        "(none attached)".to_string()
                    } else {
                        names.join(", ")
                    }
                }),
            ],
            outputs: vec![
                signal(
                    "STATR",
                    Some(USART1_BASE),
                    format!(
                        "{:#010x} (RXNE {}, TC {}, TXE {})",
                        usart.sr,
                        u8::from(usart.sr & USART_RXNE != 0),
                        u8::from(usart.sr & USART_TC != 0),
                        u8::from(usart.sr & USART_TXE != 0)
                    ),
                ),
                signal(
                    "DATAR (received)",
                    Some(USART1_BASE + 0x04),
                    format!("{:#04x}", usart.rx),
                ),
                signal(
                    "log",
                    None,
                    if self.uart_tx.is_empty() {
                        "(quiet)".into()
                    } else {
                        self.uart_tx
                            .chars()
                            .rev()
                            .take(80)
                            .collect::<String>()
                            .chars()
                            .rev()
                            .collect::<String>()
                    },
                ),
            ],
            effects,
        }
    }

    fn exti_view(&self) -> PeripheralView {
        let mut inputs = vec![
            signal(
                "INTENR",
                Some(EXTI_BASE),
                format!("{:#010x}", self.exti.intenr),
            ),
            signal(
                "RTENR",
                Some(EXTI_BASE + 0x08),
                format!("{:#010x}", self.exti.rtenr),
            ),
            signal(
                "FTENR",
                Some(EXTI_BASE + 0x0c),
                format!("{:#010x}", self.exti.ftenr),
            ),
        ];
        let armed = self.exti.intenr | self.exti.rtenr | self.exti.ftenr | self.exti.intfr;
        for line in 0..16u8 {
            if armed & (1 << line) == 0 {
                continue;
            }
            let port = self.afio.port(line);
            let high = self
                .gpio
                .get(port as usize)
                .is_some_and(|g| g.input_bit(line));
            inputs.push(signal(
                format!("line {line}"),
                Some(AFIO_BASE + 0x08 + u32::from(line / 4) * 4),
                format!(
                    "{}{} is {}, rising {}, falling {}",
                    b"ABCD".get(port as usize).copied().unwrap_or(b'?') as char,
                    line,
                    if high { "high" } else { "low" },
                    u8::from(self.exti.rtenr & (1 << line) != 0),
                    u8::from(self.exti.ftenr & (1 << line) != 0)
                ),
            ));
        }
        let pending: Vec<String> = (0..16u8)
            .filter(|line| self.exti.intfr & (1 << line) != 0)
            .map(|line| format!("line {line}"))
            .collect();
        let action = if pending.is_empty() {
            "No EXTI line is pending. A rising or falling edge on the pin selected in AFIO EXTICR sets INTFR when that edge is enabled.".to_string()
        } else {
            format!(
                "Pending: {}. The interrupt handler clears INTFR by writing those bits as 1, and ch32-hal also clears INTENR for them.",
                pending.join(", ")
            )
        };
        PeripheralView {
            id: "EXTI".into(),
            group: "EXTI".into(),
            action,
            inputs,
            outputs: vec![signal(
                "INTFR",
                Some(EXTI_BASE + 0x14),
                format!("{:#010x}", self.exti.intfr),
            )],
            effects: Vec::new(),
        }
    }

    fn pfic_view(&self) -> PeripheralView {
        let mut next = self.pfic.clone();
        for (irq, asserted) in self.irq_lines() {
            next.set_pending(irq, asserted);
        }
        let mut effects = Vec::new();
        for word in 0..8 {
            let old = self.pfic.pending[word];
            let newv = next.pending[word];
            if old == newv {
                continue;
            }
            push_effect(
                &mut effects,
                &format!("IPR{word}"),
                Some(PFIC_BASE + 0x20 + word as u32 * 4),
                u64::from(old),
                u64::from(newv),
                changed_bits(old, newv, |bit| irq_label(word as u32 * 32 + bit)),
            );
        }
        let mut watch = vec![IRQ_SYSTICK, IRQ_TIM2, IRQ_TIM3, IRQ_TIM4];
        for irq in 0..256 {
            if let Some((word, mask)) = Pfic::bit(irq) {
                if self.pfic.enable[word] & mask != 0 || self.pfic.pending[word] & mask != 0 {
                    watch.push(irq);
                }
            }
        }
        watch.sort_unstable();
        watch.dedup();
        watch.truncate(24);
        let inputs = watch
            .iter()
            .filter_map(|&irq| {
                let (word, mask) = Pfic::bit(irq)?;
                let enable = self.pfic.enable[word] & mask != 0;
                let pending = self.pfic.pending[word] & mask != 0;
                let active = self.pfic.active[word] & mask != 0;
                Some(signal(
                    irq_label(irq),
                    Some(PFIC_BASE + word as u32 * 4),
                    format!(
                        "enable {} · pending {} · active {}",
                        u8::from(enable),
                        u8::from(pending),
                        u8::from(active)
                    ),
                ))
            })
            .collect();
        let taken = if self.cpu.interrupts_enabled() {
            next_irq(&next, &self.cpu)
        } else {
            None
        };
        let mut action = if effects.is_empty() {
            "No interrupt pending bits change at the start of this step.".to_string()
        } else {
            "This step latches peripheral IRQ lines into the PFIC pending registers.".to_string()
        };
        if let Some(irq) = taken {
            action.push_str(&format!(
                " If this instruction leaves interrupts enabled, the core takes {}.",
                irq_label(irq)
            ));
        }
        PeripheralView {
            id: "PFIC".into(),
            group: "PFIC".into(),
            action,
            inputs,
            outputs: Vec::new(),
            effects,
        }
    }
}

fn pfic_write_touches_other(offset: u32) -> bool {
    !matches!(
        offset,
        0x100..=0x11c | 0x180..=0x19c | 0x200..=0x21c | 0x280..=0x29c | 0x400..=0x4ff
    )
}

fn signal(name: impl Into<String>, address: Option<u32>, value: impl Into<String>) -> SignalView {
    SignalView {
        name: name.into(),
        address,
        value: value.into(),
    }
}

fn push_effect(
    effects: &mut Vec<MemEffect>,
    name: &str,
    address: Option<u32>,
    old: u64,
    newv: u64,
    bits: String,
) {
    if old != newv {
        effects.push(MemEffect {
            name: name.into(),
            address,
            old,
            new: newv,
            bits,
        });
    }
}

fn set_bits(value: u32, name_of: impl Fn(u32) -> String) -> String {
    if value == 0 {
        return "none".into();
    }
    let mut parts = Vec::new();
    for bit in 0..32 {
        if value & (1 << bit) != 0 {
            parts.push(name_of(bit));
        }
    }
    parts.join(", ")
}

fn changed_bits(old: u32, newv: u32, name_of: impl Fn(u32) -> String) -> String {
    let diff = old ^ newv;
    if diff == 0 {
        return "—".into();
    }
    let mut parts = Vec::new();
    for bit in 0..32 {
        if diff & (1 << bit) == 0 {
            continue;
        }
        let from = (old >> bit) & 1;
        let to = (newv >> bit) & 1;
        parts.push(format!("{} {from}→{to}", name_of(bit)));
    }
    parts.join(", ")
}

fn timer_flag_name(bit: u32) -> String {
    match bit {
        0 => "UIF".into(),
        1 => "CC1IF".into(),
        2 => "CC2IF".into(),
        3 => "CC3IF".into(),
        4 => "CC4IF".into(),
        other => format!("bit {other}"),
    }
}

fn timer_ie_name(bit: u32) -> String {
    match bit {
        0 => "UIE".into(),
        1 => "CC1IE".into(),
        2 => "CC2IE".into(),
        3 => "CC3IE".into(),
        4 => "CC4IE".into(),
        other => format!("bit {other}"),
    }
}

fn rcc_ctlr_name(bit: u32) -> String {
    match bit {
        1 => "HSIRDY".into(),
        17 => "HSERDY".into(),
        25 => "PLLRDY".into(),
        other => format!("bit {other}"),
    }
}

fn rcc_cfgr_name(bit: u32) -> String {
    match bit {
        2 | 3 => "SWS".into(),
        other => format!("bit {other}"),
    }
}

fn systick_sr_name(bit: u32) -> String {
    if bit == 0 {
        "CNTIF".into()
    } else {
        format!("bit {bit}")
    }
}

fn irq_label(irq: u32) -> String {
    match irq {
        IRQ_SYSTICK => "SysTick".into(),
        IRQ_TIM2 => "TIM2".into(),
        IRQ_TIM3 => "TIM3".into(),
        IRQ_TIM4 => "TIM4".into(),
        47 => "I2C1_EV".into(),
        48 => "I2C1_ER".into(),
        49 => "I2C2_EV".into(),
        50 => "I2C2_ER".into(),
        51 => "SPI1".into(),
        52 => "SPI2".into(),
        IRQ_USART1 => "USART1".into(),
        22 => "EXTI0".into(),
        23 => "EXTI1".into(),
        24 => "EXTI2".into(),
        25 => "EXTI3".into(),
        26 => "EXTI4".into(),
        39 => "EXTI9_5".into(),
        56 => "EXTI15_10".into(),
        other => format!("IRQ{other}"),
    }
}

fn pfic_pending_bit(irq: u32) -> Option<(u32, u32)> {
    let (word, mask) = Pfic::bit(irq)?;
    Some((PFIC_BASE + 0x20 + word as u32 * 4, mask.trailing_zeros()))
}

fn next_irq(pfic: &Pfic, cpu: &Cpu) -> Option<u32> {
    for irq in 0..256 {
        let (word, mask) = Pfic::bit(irq)?;
        if pfic.enable[word] & pfic.pending[word] & mask != 0 && !cpu.is_serving(irq) {
            return Some(irq);
        }
    }
    None
}

fn drive_name(drive: ExternalDrive) -> &'static str {
    match drive {
        ExternalDrive::High => "high",
        ExternalDrive::Low => "low",
        ExternalDrive::Float => "float",
    }
}

fn level_name(level: PinLevel) -> &'static str {
    match level {
        PinLevel::High => "high",
        PinLevel::Low => "low",
        PinLevel::Floating => "floating",
        PinLevel::Analog => "analog",
    }
}

fn mode_name(mode: PinMode) -> &'static str {
    match mode {
        PinMode::Analog => "analog",
        PinMode::Floating => "floating",
        PinMode::PullUp => "pull-up",
        PinMode::PullDown => "pull-down",
        PinMode::OutputPushPull => "output push-pull",
        PinMode::OutputOpenDrain => "output open-drain",
        PinMode::AlternatePushPull => "alternate push-pull",
        PinMode::AlternateOpenDrain => "alternate open-drain",
    }
}

impl Bus for Machine {
    // Peripheral registers are read once per access, so read side effects
    // (SPI RXNE, I2C ADDR, USART RXNE) happen once. Narrow stores merge into
    // the current register value and issue a single word write.
    fn load8(&mut self, addr: u32) -> Result<u8, Fault> {
        if let Some(word) = self.mmio_word(addr & !3) {
            return Ok(word.to_le_bytes()[(addr & 3) as usize]);
        }
        self.load_bus(addr).map_err(|_| Fault::LoadFault)
    }

    fn load16(&mut self, addr: u32) -> Result<u16, Fault> {
        if let Some(word) = self.mmio_word(addr & !3) {
            return Ok((word >> ((addr & 2) * 8)) as u16);
        }
        self.load16_bus(addr).map_err(|_| Fault::InstructionFault)
    }

    fn load32(&mut self, addr: u32) -> Result<u32, Fault> {
        self.load32_bus(addr).map_err(|_| Fault::LoadFault)
    }

    fn store8(&mut self, addr: u32, val: u8) -> Result<(), Fault> {
        if let Some(word) = self.peek_mmio_word(addr & !3) {
            let mut bytes = word.to_le_bytes();
            bytes[(addr & 3) as usize] = val;
            return self.store32_bus(addr & !3, u32::from_le_bytes(bytes));
        }
        self.store_bus(addr, val)
    }

    fn store16(&mut self, addr: u32, val: u16) -> Result<(), Fault> {
        if let Some(word) = self.peek_mmio_word(addr & !3) {
            let shift = (addr & 2) * 8;
            let merged = (word & !(0xffff << shift)) | ((val as u32) << shift);
            return self.store32_bus(addr & !3, merged);
        }
        self.store8(addr, val as u8)?;
        self.store8(addr.wrapping_add(1), (val >> 8) as u8)
    }

    fn store32(&mut self, addr: u32, val: u32) -> Result<(), Fault> {
        self.store32_bus(addr, val)
    }

    fn pending_irqs(&self) -> Vec<u32> {
        let mut out = Vec::new();
        for irq in 0..256 {
            let word = irq / 32;
            let mask = 1u32 << (irq % 32);
            if self.pfic.enable[word as usize] & self.pfic.pending[word as usize] & mask != 0 {
                out.push(irq);
            }
        }
        out
    }

    fn vector(&mut self, irq: u32, mtvec: u32) -> Result<u32, Fault> {
        let mode = mtvec & 0b11;
        let base = mtvec & !0b11;
        if mode == 3 {
            self.load32_bus(base.wrapping_add(irq.wrapping_mul(4)))
                .map_err(|_| Fault::InstructionFault)
        } else if mode == 1 {
            Ok(base.wrapping_add(irq.wrapping_mul(4)))
        } else {
            Ok(base)
        }
    }

    fn note_irq_active(&mut self, irq: u32, active: bool) {
        if let Some((word, mask)) = Pfic::bit(irq) {
            if active {
                self.pfic.active[word] |= mask;
            } else {
                self.pfic.active[word] &= !mask;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gpio_blink_program_toggles_pa0() {
        let mut m = Machine::new();
        let words = [
            0x4002_10b7u32, // lui x1, 0x40021
            0x0180_8093,    // addi x1, x1, 0x18   RCC_APB2PCENR
            0x0040_0113,    // addi x2, x0, 4
            0x0020_a023,    // sw x2, 0(x1)
            0x4001_10b7,    // lui x1, 0x40011
            0x8000_8093,    // addi x1, x1, -2048  -> 0x40010800 GPIOA
            0x0020_0113,    // addi x2, x0, 2      PA0 push-pull 2 MHz
            0x0020_a023,    // sw x2, 0(x1)
            0x0010_0113,    // addi x2, x0, 1
            0x0020_a823,    // sw x2, 16(x1)       BSHR set
            0x0001_0137,    // lui x2, 0x10        1 << 16
            0x0020_a823,    // sw x2, 16(x1)       BSHR reset
            0xff1f_f06f,    // jal x0, -16         back to the set
        ];
        let mut bytes = Vec::new();
        for w in words {
            bytes.extend_from_slice(&w.to_le_bytes());
        }
        m.load_image(&bytes, FLASH_BASE, &[(FLASH_BASE, bytes.as_slice())]);
        let mut saw_high = false;
        let mut saw_low = false;
        for _ in 0..40 {
            m.step().unwrap();
            let pin = m.pin(0, 0);
            if pin.mode == PinMode::OutputPushPull && pin.level == PinLevel::High {
                saw_high = true;
            }
            if pin.mode == PinMode::OutputPushPull && pin.level == PinLevel::Low {
                saw_low = true;
            }
        }
        assert!(saw_high, "PA0 never drove high: {:?}", m.faults);
        assert!(saw_low, "PA0 never drove low: {:?}", m.faults);
        assert!(m.pin(0, 0).clock_enabled);
    }

    #[test]
    fn tim2_preview_reports_counter_reset_address_and_values() {
        let mut m = Machine::new();
        m.flash[0] = 0x13;
        m.cpu.pc = FLASH_BASE;
        m.rcc.regs[7] = 1;
        m.timers[0].ctlr1 = 1;
        m.timers[0].psc = 0;
        m.timers[0].presc = 0;
        m.timers[0].arr = 4;
        m.timers[0].cnt = 4;
        m.timers[0].dier = 1;
        m.timers[0].ccr = [0xffff; 4];

        let view = m
            .peripherals()
            .into_iter()
            .find(|item| item.id == "TIM2")
            .unwrap();
        assert!(
            view.action.contains("resets the counter"),
            "{}",
            view.action
        );
        let cnt = view
            .effects
            .iter()
            .find(|effect| effect.name == "CNT")
            .unwrap();
        assert_eq!(cnt.address, Some(0x4000_0024));
        assert_eq!(cnt.old, 4);
        assert_eq!(cnt.new, 0);
        let status = view
            .effects
            .iter()
            .find(|effect| effect.name == "INTFR")
            .unwrap();
        assert_eq!(status.address, Some(0x4000_0010));
        assert_eq!(status.old, 0);
        assert_eq!(status.new, 1);
        assert!(status.bits.contains("UIF"));

        m.step().unwrap();
        assert_eq!(m.timers[0].cnt, 0);
        assert_eq!(m.timers[0].sr & 1, 1);
        assert_eq!(
            m.pfic.pending[1] & (1 << 12),
            0,
            "pending latches on the following step"
        );

        m.timers[0].psc = 10;
        m.timers[0].presc = 3;
        m.timers[0].cnt = 1;
        m.timers[0].arr = 100;
        let view = m
            .peripherals()
            .into_iter()
            .find(|item| item.id == "TIM2")
            .unwrap();
        assert!(view.effects.iter().all(|effect| effect.name != "CNT"));
        let presc = view
            .effects
            .iter()
            .find(|effect| effect.name == "prescaler counter")
            .unwrap();
        assert_eq!(presc.address, None);
        assert_eq!(presc.old, 3);
        assert_eq!(presc.new, 4);
    }

    #[test]
    fn reverse_step_restores_registers_and_sram() {
        let mut m = Machine::new();
        let words = [
            0x2000_0137u32, // lui x2, 0x20000
            0x0070_0093,    // addi x1, x0, 7
            0x0011_2023,    // sw x1, 0(x2)
            0x0010_8093,    // addi x1, x1, 1
        ];
        for (i, word) in words.iter().enumerate() {
            let off = i * 4;
            m.flash[off..off + 4].copy_from_slice(&word.to_le_bytes());
        }
        m.cpu.pc = FLASH_BASE;
        for _ in 0..4 {
            m.step().unwrap();
        }
        assert_eq!(m.cpu.x(1), 8);
        assert_eq!(u32::from_le_bytes(m.sram[0..4].try_into().unwrap()), 7);
        assert_eq!(m.undo_available(), 4);

        assert!(m.reverse_step());
        assert_eq!(m.cpu.x(1), 7);
        assert_eq!(m.cpu.pc, FLASH_BASE + 12);
        assert_eq!(u32::from_le_bytes(m.sram[0..4].try_into().unwrap()), 7);

        assert!(m.reverse_step());
        assert_eq!(m.cpu.x(1), 7);
        assert_eq!(u32::from_le_bytes(m.sram[0..4].try_into().unwrap()), 0);
        assert_eq!(m.cpu.pc, FLASH_BASE + 8);

        assert!(m.reverse_step());
        assert_eq!(m.cpu.x(1), 0);
        assert_eq!(m.cpu.x(2), 0x2000_0000);

        assert!(m.reverse_step());
        assert_eq!(m.cpu.pc, FLASH_BASE);
        assert_eq!(m.cpu.x(2), 0);
        assert!(!m.reverse_step());
    }

    #[test]
    fn reverse_keeps_ten_cycles_and_pin_drives() {
        let mut m = Machine::new();
        m.flash[0] = 0x6f; // jal x0, 0
        m.cpu.pc = FLASH_BASE;
        for _ in 0..12 {
            m.step().unwrap();
        }
        assert_eq!(m.undo_available(), 10);
        assert_eq!(m.cpu.cycle, 12);
        m.set_external(0, 0, ExternalDrive::High, 1.5);
        for _ in 0..10 {
            assert!(m.reverse_step());
        }
        assert!(!m.reverse_step());
        assert_eq!(m.cpu.cycle, 2);
        assert_eq!(m.gpio[0].external[0], ExternalDrive::High);
        assert_eq!(m.gpio[0].analog_v[0], 1.5);
    }

    #[test]
    fn rising_edge_on_pb1_interrupts_and_toggles_pa14() {
        let mut m = Machine::new();
        // IRQ 23 vector, a wait loop, and a handler that toggles PA14 then returns.
        let handler = FLASH_BASE + 0x100;
        let wait = FLASH_BASE + 0x80;
        m.store32(FLASH_BASE + 23 * 4, handler).unwrap();
        let wait_code = [
            0x1050_0073u32, // wfi
            0xffdf_f06f,    // jal x0, -4
        ];
        let handler_code = [
            0x4001_0237, // lui x4, 0x40010
            0x4142_0213, // addi x4, x4, 0x414 -> EXTI INTFR
            0x0020_0293, // addi x5, x0, 2
            0x0052_2023, // sw x5, 0(x4)      clear line 1
            0x4001_10b7, // lui x1, 0x40011
            0x8000_8093, // addi x1, x1, -2048 -> GPIOA
            0x00c0_a103, // lw x2, 12(x1)
            0x0000_41b7, // lui x3, 4 -> bit 14
            0x0031_4133, // xor x2, x2, x3
            0x0020_a623, // sw x2, 12(x1)
            0x3020_0073, // mret
        ];
        for (i, word) in wait_code.iter().enumerate() {
            m.store32(wait + i as u32 * 4, *word).unwrap();
        }
        for (i, word) in handler_code.iter().enumerate() {
            m.store32(handler + i as u32 * 4, *word).unwrap();
        }
        // PA14 push-pull output. PB1 input with pull-up, so it idles high.
        m.store32(0x4001_0804, (0x4444_4444 & !(0xf << 24)) | (0x2 << 24))
            .unwrap();
        m.store32(0x4001_0c00, (0x4444_4444 & !(0xf << 4)) | (0x8 << 4))
            .unwrap();
        m.store32(0x4001_0c0c, 1 << 1).unwrap();
        m.store32(AFIO_BASE + 0x08, 1 << 4).unwrap();
        m.store32(EXTI_BASE + 0x08, 1 << 1).unwrap();
        m.store32(EXTI_BASE, 1 << 1).unwrap();
        m.store32(PFIC_BASE + 0x100, 1 << 23).unwrap();
        m.cpu.mtvec = FLASH_BASE | 3;
        m.cpu.mstatus |= 0x8;
        m.cpu.pc = wait;

        m.set_external(1, 1, ExternalDrive::Low, 0.0);
        assert_eq!(
            m.exti.intfr & (1 << 1),
            0,
            "driving low is not a rising edge"
        );
        m.set_external(1, 1, ExternalDrive::High, 0.0);
        assert_ne!(m.exti.intfr & (1 << 1), 0, "INTFR line 1");
        for _ in 0..12 {
            m.step().unwrap();
        }
        assert_eq!(m.pin(0, 14).level, PinLevel::High);
        assert_eq!(m.cpu.irqs_taken, 1);
    }

    #[test]
    fn firmware_pb1_rising_edge_toggles_pa14() {
        let path = concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../firmware/embassy-blink/target/riscv32imc-unknown-none-elf/release/embassy-blink"
        );
        let Ok(bytes) = std::fs::read(path) else {
            return;
        };
        let loaded = crate::elf::load_elf(&bytes).expect("elf");
        let segs: Vec<(u32, &[u8])> = loaded
            .segments
            .iter()
            .map(|s| (s.address, s.data.as_slice()))
            .collect();
        let mut m = Machine::new();
        m.load_image(&bytes, loaded.entry, &segs);
        let mut armed = false;
        let mut steps = 0u32;
        for _ in 0..6_000_000 {
            m.step().unwrap();
            steps += 1;
            if m.exti.intenr & (1 << 1) != 0 {
                armed = true;
                break;
            }
        }
        assert!(armed, "EXTI line 1 was never armed after {steps} steps");
        assert_eq!(m.pin(0, 14).level, PinLevel::Low);
        m.set_external(1, 1, ExternalDrive::Low, 0.0);
        for _ in 0..1_000 {
            m.step().unwrap();
        }
        assert_eq!(m.pin(0, 14).level, PinLevel::Low);
        m.set_external(1, 1, ExternalDrive::High, 0.0);
        let mut toggled = false;
        for _ in 0..400_000 {
            m.step().unwrap();
            if m.pin(0, 14).level == PinLevel::High {
                toggled = true;
                break;
            }
        }
        assert!(toggled, "PA14 stayed low after a rising edge on PB1");
    }

    use crate::devices::{Component, DeviceCtx, DeviceView, I2cDevice, SimAdc, UartDevice};

    const SPI1_DATAR: u32 = 0x4001_300c;
    const SPI1_STATR: u32 = 0x4001_3008;
    const GPIOA_BSHR: u32 = 0x4001_0810;

    /// The register sequence ch32-hal uses: RCC clocks, PA4 push-pull, SPI1 master mode 0.
    fn spi_adc_machine() -> (Machine, String) {
        let mut m = Machine::new();
        let id = m.attach_kind("SPI1", "adc", Some("PA4")).unwrap();
        m.store32(RCC_BASE + 0x18, (1 << 12) | (1 << 2)).unwrap();
        m.store32(0x4001_0800, 0x4444_0444 | (0x3 << 16)).unwrap(); // PA4 output
        m.store32(GPIOA_BSHR, 1 << 4).unwrap(); // CS high
        m.store32(
            0x4001_3000,
            (1 << 2) | (3 << 3) | (1 << 6) | (1 << 8) | (1 << 9),
        )
        .unwrap();
        (m, id)
    }

    fn xfer(m: &mut Machine, byte: u8) -> u8 {
        assert_ne!(m.load32(SPI1_STATR).unwrap() & 2, 0, "TXE");
        m.store8(SPI1_DATAR, byte).unwrap();
        assert_ne!(m.load32(SPI1_STATR).unwrap() & 1, 0, "RXNE");
        m.load8(SPI1_DATAR).unwrap()
    }

    fn adc_read(m: &mut Machine, reg: u8, n: usize) -> Vec<u8> {
        m.store32(GPIOA_BSHR, 1 << (16 + 4)).unwrap(); // CS low
        xfer(m, 0x80 | reg);
        let out = (0..n).map(|_| xfer(m, 0)).collect();
        m.store32(GPIOA_BSHR, 1 << 4).unwrap();
        out
    }

    #[test]
    fn spi_adc_setup_then_conversion_reaches_the_firmware() {
        let (mut m, id) = spi_adc_machine();
        assert_eq!(id, "ADC0");
        assert_eq!(adc_read(&mut m, SimAdc::REG_ID, 1), vec![0xAD]);
        assert_eq!(adc_read(&mut m, SimAdc::REG_STATUS, 1), vec![0x00]);

        m.set_device("ADC0", "ch2", 2.0).unwrap();
        m.store32(GPIOA_BSHR, 1 << 20).unwrap();
        xfer(&mut m, SimAdc::REG_CONFIG);
        xfer(&mut m, SimAdc::CONFIG_PWR | (2 << 4));
        m.store32(GPIOA_BSHR, 1 << 4).unwrap();
        assert_eq!(adc_read(&mut m, SimAdc::REG_STATUS, 1), vec![0x01]);

        m.now += SimAdc::STARTUP_CYCLES;
        assert_eq!(adc_read(&mut m, SimAdc::REG_STATUS, 1), vec![0x81]);
        let data = adc_read(&mut m, SimAdc::REG_DATA, 2);
        let code = u16::from_be_bytes([data[0], data[1]]);
        assert_eq!(code, (2.0f64 / 3.3 * 4095.0).round() as u16);

        // With CS high the ADC ignores the bus and MISO idles high.
        assert_eq!(xfer(&mut m, 0x80), 0xff);
        let view = m
            .peripherals()
            .into_iter()
            .find(|p| p.id == "ADC0")
            .unwrap();
        assert!(view.action.contains("SPI1"), "{}", view.action);
    }

    #[test]
    fn reverse_step_undoes_an_spi_transfer() {
        let (mut m, _) = spi_adc_machine();
        m.store32(GPIOA_BSHR, 1 << 20).unwrap();
        m.flash[..4].copy_from_slice(&0x0011_2023u32.to_le_bytes()); // sw x1, 0(x2)
        m.cpu.pc = FLASH_BASE;
        m.cpu.x[1] = 0x80 | SimAdc::REG_ID as u32;
        m.cpu.x[2] = SPI1_DATAR;
        m.step().unwrap();
        assert_eq!(m.spi[0].frames, 1);
        assert!(m.reverse_step());
        assert_eq!(m.spi[0].frames, 0);
        // The ADC is back to waiting for a command, so the same store reads the ID again.
        m.step().unwrap();
        m.load8(SPI1_DATAR).unwrap();
        assert_eq!(xfer(&mut m, 0), 0xAD);
    }

    #[derive(Clone, Default)]
    struct Echo {
        heard: Vec<u8>,
        queue: Vec<u8>,
    }

    impl Component for Echo {
        fn kind(&self) -> &'static str {
            "echo"
        }
        fn view(&self, _: &DeviceCtx) -> DeviceView {
            DeviceView::default()
        }
    }

    impl UartDevice for Echo {
        fn receive(&mut self, byte: u8, _: &DeviceCtx) {
            self.heard.push(byte);
            self.queue.push(byte.to_ascii_uppercase());
        }
        fn has_byte(&self, _: &DeviceCtx) -> bool {
            !self.queue.is_empty()
        }
        fn take_byte(&mut self, _: &DeviceCtx) -> u8 {
            self.queue.remove(0)
        }
        fn clone_box(&self) -> Box<dyn UartDevice> {
            Box::new(self.clone())
        }
    }

    #[test]
    fn uart_component_hears_tx_and_answers_on_rx() {
        let mut m = Machine::new();
        m.attach_uart(Box::new(Echo::default()));
        m.store32(USART1_BASE + 0x0c, (1 << 13) | (1 << 3) | (1 << 2))
            .unwrap();
        m.store8(USART1_BASE + 0x04, b'a').unwrap();
        m.flash[0] = 0x6f; // jal x0, 0
        m.cpu.pc = FLASH_BASE;
        m.step().unwrap();
        assert_ne!(m.load32(USART1_BASE).unwrap() & USART_RXNE, 0);
        assert_eq!(m.load8(USART1_BASE + 0x04).unwrap(), b'A');
        assert_eq!(m.load32(USART1_BASE).unwrap() & USART_RXNE, 0);
        assert_eq!(m.uart_tx, "a");
    }

    #[derive(Clone, Default)]
    struct Reg8 {
        pointer: u8,
        regs: [u8; 4],
        stops: u32,
        first: bool,
    }

    impl Component for Reg8 {
        fn kind(&self) -> &'static str {
            "reg8"
        }
        fn view(&self, _: &DeviceCtx) -> DeviceView {
            DeviceView::default()
        }
    }

    impl I2cDevice for Reg8 {
        fn address(&self) -> u8 {
            0x48
        }
        fn start(&mut self, read: bool, _: &DeviceCtx) -> bool {
            self.first = !read;
            true
        }
        fn write(&mut self, byte: u8, _: &DeviceCtx) -> bool {
            if self.first {
                self.pointer = byte & 3;
                self.first = false;
            } else {
                self.regs[self.pointer as usize] = byte;
                self.pointer = (self.pointer + 1) & 3;
            }
            true
        }
        fn read(&mut self, _: &DeviceCtx) -> u8 {
            let value = self.regs[self.pointer as usize];
            self.pointer = (self.pointer + 1) & 3;
            value
        }
        fn stop(&mut self, _: &DeviceCtx) {
            self.stops += 1;
        }
        fn clone_box(&self) -> Box<dyn I2cDevice> {
            Box::new(self.clone())
        }
    }

    #[test]
    fn i2c_write_then_read_follows_the_hal_sequence() {
        let base = I2C_BASES[0];
        let star1 = base + 0x14;
        let star2 = base + 0x18;
        let data = base + 0x10;
        let ctlr1 = base;
        let mut m = Machine::new();
        m.attach_i2c(0, Box::new(Reg8::default()));
        m.store32(RCC_BASE + 0x1c, 1 << 21).unwrap();
        m.store32(ctlr1, 1).unwrap();

        // Write register 1 = 0x5a, register 2 = 0xa5.
        m.store32(ctlr1, 1 | (1 << 8)).unwrap();
        assert_ne!(m.load32(star1).unwrap() & 1, 0, "SB");
        m.store32(data, 0x48 << 1).unwrap();
        assert_ne!(m.load32(star1).unwrap() & 2, 0, "ADDR");
        m.load32(star2).unwrap();
        for byte in [1u32, 0x5a, 0xa5] {
            assert_ne!(m.load32(star1).unwrap() & (1 << 7), 0, "TxE");
            m.store32(data, byte).unwrap();
            assert_ne!(m.load32(star1).unwrap() & (1 << 2), 0, "BTF");
        }
        m.store32(ctlr1, 1 | (1 << 9)).unwrap();

        // Point at register 1, repeated START, read two bytes.
        m.store32(ctlr1, 1 | (1 << 8)).unwrap();
        m.store32(data, 0x48 << 1).unwrap();
        m.load32(star2).unwrap();
        m.store32(data, 1).unwrap();
        m.store32(ctlr1, 1 | (1 << 8) | (1 << 10)).unwrap();
        m.store32(data, (0x48 << 1) | 1).unwrap();
        assert_ne!(m.load32(star1).unwrap() & 2, 0, "ADDR");
        m.load32(star2).unwrap();
        assert_eq!(m.load32(data).unwrap(), 0x5a);
        m.store32(ctlr1, 1 | (1 << 9)).unwrap();
        assert_ne!(m.load32(star1).unwrap() & (1 << 6), 0, "RxNE");
        assert_eq!(m.load32(data).unwrap(), 0xa5);

        // Nobody answers at 0x50.
        m.store32(ctlr1, 1 | (1 << 8)).unwrap();
        m.store32(data, 0x50 << 1).unwrap();
        assert_ne!(m.load32(star1).unwrap() & (1 << 10), 0, "AF");
        let Link::I2c { dev, .. } = &m.devices[0].link else {
            panic!()
        };
        assert_eq!(dev.address(), 0x48);
    }
}
