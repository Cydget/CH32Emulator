//! CH32V203C8T6 machine: memory map plus the peripherals Embassy blink firmware touches.

use crate::cpu::{Bus, Cpu, Fault};

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

#[derive(Clone, Debug)]
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

#[derive(Clone, Debug)]
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
            0x300..=0x31c => self.active.get((offset - 0x300) as usize / 4).copied().unwrap_or(0),
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

#[derive(Clone, Debug)]
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
            return;
        }
        if let Ok(info) = crate::elf::load_elf(&image) {
            let segs: Vec<(u32, Vec<u8>)> = info
                .segments
                .iter()
                .map(|s| (s.address, s.data.clone()))
                .collect();
            let borrowed: Vec<(u32, &[u8])> = segs.iter().map(|(a, d)| (*a, d.as_slice())).collect();
            self.entry = info.entry;
            self.reset_from_image(&borrowed);
        }
    }

    fn reset_from_image(&mut self, segments: &[(u32, &[u8])]) {
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
        self.pfic.set_pending(IRQ_SYSTICK, self.systick.irq_asserted());
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

    pub fn current_insn(&self) -> String {
        match self.load16_bus(self.cpu.pc) {
            Ok(half) if half & 0b11 != 0b11 => crate::cpu::disasm_c(half),
            Ok(half) => match self.load16_bus(self.cpu.pc.wrapping_add(2)) {
                Ok(hi) => crate::cpu::disasm((half as u32) | ((hi as u32) << 16)),
                Err(_) => format!(".half {half:04x}"),
            },
            Err(_) => "???".to_string(),
        }
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
        if addr < FLASH_SIZE as u32 || (FLASH_BASE..FLASH_BASE + FLASH_SIZE as u32).contains(&addr) {
            let off = if addr < FLASH_SIZE as u32 {
                addr as usize
            } else {
                (addr - FLASH_BASE) as usize
            };
            self.flash[off] = val;
            return Ok(());
        }
        if (SRAM_BASE..SRAM_BASE + SRAM_SIZE as u32).contains(&addr) {
            self.sram[(addr - SRAM_BASE) as usize] = val;
            return Ok(());
        }
        if (INFO_BASE..INFO_BASE + INFO_SIZE as u32).contains(&addr) {
            self.info[(addr - INFO_BASE) as usize] = val;
            return Ok(());
        }
        if (MMIO_BASE..MMIO_BASE + MMIO_SIZE as u32).contains(&addr) {
            self.mmio[(addr - MMIO_BASE) as usize] = val;
            return Ok(());
        }
        if (PFIC_BASE..PFIC_BASE + PFIC_SIZE as u32).contains(&addr) {
            let off = (addr - PFIC_BASE) as usize;
            self.pfic.other[off] = val;
            if (0x400..0x500).contains(&off) {
                self.pfic.priority[off - 0x400] = val;
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
            self.pfic.write_word(addr - PFIC_BASE, val);
            return true;
        }
        false
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
}
