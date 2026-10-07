//! CH32V203C8T6 machine: memory map plus the peripherals Embassy blink firmware touches.

use std::collections::VecDeque;

use crate::cpu::{Bus, Cpu, CpuHeap, CpuScalars, Fault};

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
    tx: Vec<u8>,
}

impl Usart {
    fn new() -> Self {
        Self {
            cr1: 0,
            brr: 0,
            sr: 0x00c0,
            tx: Vec::new(),
        }
    }

    fn read(&self, offset: u32) -> u32 {
        match offset {
            0x00 => self.sr,
            0x04 => 0,
            0x08 => self.brr,
            0x0c => self.cr1,
            _ => 0,
        }
    }

    fn write(&mut self, offset: u32, value: u32) {
        match offset {
            0x00 => self.sr = value,
            0x04 => {
                self.tx.push(value as u8);
                self.sr |= 0x00c0;
            }
            0x08 => self.brr = value,
            0x0c => self.cr1 = value,
            _ => {}
        }
    }
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
    usart_tx: Option<Vec<u8>>,
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
        self.uart_tx.clear();
        self.faults.clear();
        for (addr, data) in segments {
            let _ = self.write_bytes(*addr, data);
        }
        self.cpu.reset(self.entry);
    }

    pub fn step(&mut self) -> Result<(), Fault> {
        let mut undo = self.capture_undo();
        self.journaling = true;
        let result = self.step_inner();
        self.journaling = false;
        undo.mem = self.journal.clone();
        self.journal.clear();
        self.push_undo(undo);
        result
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
            usart_tx: if self.usart1.tx.is_empty() {
                None
            } else {
                Some(self.usart1.tx.clone())
            },
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
        for timer in &self.timers {
            self.pfic.set_pending(timer.irq, timer.irq_asserted());
        }
        self.pfic
            .set_pending(IRQ_SYSTICK, self.systick.irq_asserted());
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
        if (RCC_BASE..RCC_BASE + 0x40).contains(&addr) {
            let mut rcc = self.rcc;
            rcc.sync();
            return Some(rcc.regs[((addr - RCC_BASE) / 4) as usize]);
        }
        if (USART1_BASE..USART1_BASE + 0x20).contains(&addr) {
            return Some(self.usart1.read(addr - USART1_BASE));
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
        if (RCC_BASE..RCC_BASE + 0x40).contains(&addr) {
            self.rcc.sync();
            return Some(self.rcc.regs[((addr - RCC_BASE) / 4) as usize]);
        }
        if (USART1_BASE..USART1_BASE + 0x20).contains(&addr) {
            return Some(self.usart1.read(addr - USART1_BASE));
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
            }
            self.rcc.regs[idx] = val;
            self.rcc.sync();
            return true;
        }
        if (USART1_BASE..USART1_BASE + 0x20).contains(&addr) {
            self.usart1.write(addr - USART1_BASE, val);
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
        out.push(self.pfic_view());
        out
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
            ],
            outputs: vec![
                signal(
                    "STATR",
                    Some(USART1_BASE),
                    format!("{:#010x} (TXE and TC follow bit 6 and bit 7)", usart.sr),
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

    fn pfic_view(&self) -> PeripheralView {
        let mut next = self.pfic.clone();
        for timer in &self.timers {
            next.set_pending(timer.irq, timer.irq_asserted());
        }
        next.set_pending(IRQ_SYSTICK, self.systick.irq_asserted());
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
    fn load8(&mut self, addr: u32) -> Result<u8, Fault> {
        if self.mmio_word(addr & !3).is_some() {
            let word = self.mmio_word(addr & !3).unwrap();
            return Ok(word.to_le_bytes()[(addr & 3) as usize]);
        }
        self.load_bus(addr).map_err(|_| Fault::LoadFault)
    }

    fn load16(&mut self, addr: u32) -> Result<u16, Fault> {
        if self.mmio_word(addr & !3).is_some() && addr & 2 == 0 {
            let word = self.load32(addr & !3)?;
            return Ok(if addr & 2 == 0 {
                word as u16
            } else {
                (word >> 16) as u16
            });
        }
        self.load16_bus(addr).map_err(|_| Fault::InstructionFault)
    }

    fn load32(&mut self, addr: u32) -> Result<u32, Fault> {
        self.load32_bus(addr).map_err(|_| Fault::LoadFault)
    }

    fn store8(&mut self, addr: u32, val: u8) -> Result<(), Fault> {
        if self.mmio_word(addr & !3).is_some() {
            let aligned = addr & !3;
            let mut word = self.load32_bus(aligned).unwrap_or(0).to_le_bytes();
            word[(addr & 3) as usize] = val;
            return self.store32_bus(aligned, u32::from_le_bytes(word));
        }
        self.store_bus(addr, val)
    }

    fn store16(&mut self, addr: u32, val: u16) -> Result<(), Fault> {
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
}
