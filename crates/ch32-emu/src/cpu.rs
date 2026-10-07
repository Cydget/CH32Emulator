//! RV32IMC interpreter plus the QingKe V4 CSR and interrupt behavior used by `qingke-rt`.

const HW_REGS: [usize; 16] = [1, 5, 6, 7, 10, 11, 12, 13, 14, 15, 16, 17, 28, 29, 30, 31];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Fault {
    InstructionMisaligned,
    InstructionFault,
    IllegalInstruction,
    Breakpoint,
    LoadFault,
    StoreFault,
    EnvCall,
}

impl Fault {
    fn code(self) -> u32 {
        match self {
            Fault::InstructionMisaligned => 0,
            Fault::InstructionFault => 1,
            Fault::IllegalInstruction => 2,
            Fault::Breakpoint => 3,
            Fault::LoadFault => 5,
            Fault::StoreFault => 7,
            Fault::EnvCall => 11,
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Fault::InstructionMisaligned => "instruction misaligned",
            Fault::InstructionFault => "instruction access fault",
            Fault::IllegalInstruction => "illegal instruction",
            Fault::Breakpoint => "breakpoint",
            Fault::LoadFault => "load access fault",
            Fault::StoreFault => "store access fault",
            Fault::EnvCall => "environment call",
        }
    }
}

pub trait Bus {
    fn load8(&mut self, addr: u32) -> Result<u8, Fault>;
    fn load16(&mut self, addr: u32) -> Result<u16, Fault>;
    fn load32(&mut self, addr: u32) -> Result<u32, Fault>;
    fn store8(&mut self, addr: u32, val: u8) -> Result<(), Fault>;
    fn store16(&mut self, addr: u32, val: u16) -> Result<(), Fault>;
    fn store32(&mut self, addr: u32, val: u32) -> Result<(), Fault>;
    /// IRQ numbers whose line is high and whose PFIC enable bit is set.
    fn pending_irqs(&self) -> Vec<u32>;
    fn vector(&mut self, irq: u32, mtvec: u32) -> Result<u32, Fault>;
    fn note_irq_active(&mut self, irq: u32, active: bool);
}

#[derive(Clone, Debug)]
pub struct Cpu {
    pub x: [u32; 32],
    pub pc: u32,
    pub mstatus: u32,
    pub mtvec: u32,
    pub mscratch: u32,
    pub mepc: u32,
    pub mcause: u32,
    pub mtval: u32,
    pub mie: u32,
    pub mip: u32,
    pub intsyscr: u32,
    pub corecfgr: u32,
    pub cycle: u64,
    pub instret: u64,
    custom_csr: std::collections::HashMap<u16, u32>,
    hw_frames: u32,
    /// Global-enable bits saved on trap entry. `mret` restores from here so a
    /// critical section inside the handler can clear `mstatus.MPIE` without
    /// losing the interrupted context. QingKe does this in hardware when
    /// `INTSYSCR.INESTEN` is set.
    mie_stack: Vec<bool>,
    serving: Vec<u32>,
    pub trace: bool,
    pub mie_steps: u64,
    pub irqs_taken: u64,
}

impl Default for Cpu {
    fn default() -> Self {
        Self {
            x: [0; 32],
            pc: 0,
            // MPP = machine. Interrupts stay off until firmware enables them.
            mstatus: 0x1800,
            mtvec: 0,
            mscratch: 0,
            mepc: 0,
            mcause: 0,
            mtval: 0,
            mie: 0,
            mip: 0,
            intsyscr: 0,
            corecfgr: 0,
            cycle: 0,
            instret: 0,
            custom_csr: std::collections::HashMap::new(),
            hw_frames: 0,
            mie_stack: Vec::new(),
            serving: Vec::new(),
            trace: false,
            mie_steps: 0,
            irqs_taken: 0,
        }
    }
}

impl Cpu {
    pub fn reset(&mut self, entry: u32) {
        let trace = self.trace;
        *self = Self::default();
        self.trace = trace;
        self.pc = entry;
    }

    pub fn x(&self, i: usize) -> u32 {
        if i == 0 { 0 } else { self.x[i] }
    }

    fn set_x(&mut self, i: usize, val: u32) {
        if i != 0 {
            self.x[i] = val;
        }
    }

    pub fn interrupts_enabled(&self) -> bool {
        self.mstatus & 0x8 != 0
    }

    pub fn serving_len(&self) -> usize {
        self.serving.len()
    }

    pub fn hw_frames(&self) -> u32 {
        self.hw_frames
    }

    pub fn hw_stack_enabled(&self) -> bool {
        self.intsyscr & 1 != 0
    }

    /// Execute one instruction, then take a pending interrupt if the core is enabled.
    ///
    /// `Ok(Some(fault))` means the instruction raised an exception and the core has
    /// already vectored to the handler. `Err` means vectoring itself could not complete.
    pub fn step(&mut self, bus: &mut impl Bus) -> Result<Option<Fault>, Fault> {
        self.x[0] = 0;
        if self.pc & 1 != 0 {
            self.enter_exception(bus, Fault::InstructionMisaligned, self.pc)?;
            return Ok(Some(Fault::InstructionMisaligned));
        }
        let pc = self.pc;
        let half = match bus.load16(pc) {
            Ok(h) => h,
            Err(f) => {
                self.enter_exception(bus, f, pc)?;
                return Ok(Some(f));
            }
        };
        let result = if half & 0b11 != 0b11 {
            if self.trace {
                eprintln!("pc={pc:08x} c {half:04x} {}", disasm_c(half));
            }
            self.exec_c(bus, pc, half)
        } else {
            let hi = match bus.load16(pc.wrapping_add(2)) {
                Ok(h) => h,
                Err(f) => {
                    self.enter_exception(bus, f, pc)?;
                    return Ok(Some(f));
                }
            };
            let word = (half as u32) | ((hi as u32) << 16);
            if self.trace {
                eprintln!("pc={pc:08x}   {word:08x} {}", disasm(word));
            }
            self.exec32(bus, pc, word)
        };
        if let Err(f) = result {
            self.enter_exception(bus, f, pc)?;
            return Ok(Some(f));
        }
        self.x[0] = 0;
        self.cycle = self.cycle.wrapping_add(1);
        self.instret = self.instret.wrapping_add(1);
        if self.interrupts_enabled() {
            self.mie_steps = self.mie_steps.wrapping_add(1);
        }
        self.poll_interrupt(bus)?;
        Ok(None)
    }

    fn poll_interrupt(&mut self, bus: &mut impl Bus) -> Result<(), Fault> {
        if !self.interrupts_enabled() {
            return Ok(());
        }
        let pending = bus.pending_irqs();
        for irq in pending {
            if self.serving.contains(&irq) {
                continue;
            }
            let handler = match bus.vector(irq, self.mtvec) {
                Ok(addr) if addr != 0 => addr,
                Ok(_) => continue,
                Err(f) => return self.enter_exception(bus, f, self.pc),
            };
            self.enter_irq(bus, irq, handler)?;
            return Ok(());
        }
        Ok(())
    }

    fn enter_irq(&mut self, bus: &mut impl Bus, irq: u32, handler: u32) -> Result<(), Fault> {
        let previous_mie = (self.mstatus >> 3) & 1 == 1;
        self.mie_stack.push(previous_mie);
        self.mepc = self.pc;
        self.mcause = 0x8000_0000 | irq;
        self.mtval = 0;
        self.mstatus = (self.mstatus & !0x88) | (u32::from(previous_mie) << 7);
        if self.hw_stack_enabled() {
            self.push_hw(bus)?;
            self.hw_frames = self.hw_frames.saturating_add(1);
        }
        self.serving.push(irq);
        self.irqs_taken = self.irqs_taken.wrapping_add(1);
        bus.note_irq_active(irq, true);
        self.pc = handler;
        Ok(())
    }

    fn enter_exception(&mut self, bus: &mut impl Bus, fault: Fault, pc: u32) -> Result<(), Fault> {
        self.mepc = pc;
        self.mcause = fault.code();
        self.mtval = pc;
        let previous_mie = (self.mstatus >> 3) & 1 == 1;
        self.mie_stack.push(previous_mie);
        self.mstatus = (self.mstatus & !0x88) | (u32::from(previous_mie) << 7);
        let mode = self.mtvec & 0b11;
        let base = self.mtvec & !0b11;
        // QingKe vectored-address mode sends every exception through vector slot 3
        // (`Exception` in qingke-rt), which then dispatches on `mcause`.
        let target = if mode == 3 {
            match bus.vector(3, self.mtvec) {
                Ok(addr) if addr != 0 => addr,
                _ => base,
            }
        } else {
            base
        };
        if self.hw_stack_enabled() {
            self.push_hw(bus)?;
            self.hw_frames = self.hw_frames.saturating_add(1);
        }
        self.pc = target;
        if self.trace {
            eprintln!("exception {} at {pc:08x} -> {target:08x}", fault.name());
        }
        Ok(())
    }

    fn push_hw(&mut self, bus: &mut impl Bus) -> Result<(), Fault> {
        for &reg in &HW_REGS {
            let sp = self.x(2).wrapping_sub(4);
            self.set_x(2, sp);
            bus.store32(sp, self.x(reg))?;
        }
        Ok(())
    }

    fn pop_hw(&mut self, bus: &mut impl Bus) -> Result<(), Fault> {
        for &reg in HW_REGS.iter().rev() {
            let sp = self.x(2);
            let val = bus.load32(sp)?;
            self.set_x(reg, val);
            self.set_x(2, sp.wrapping_add(4));
        }
        Ok(())
    }

    fn mret(&mut self, bus: &mut impl Bus) -> Result<(), Fault> {
        if self.hw_frames > 0 {
            self.pop_hw(bus)?;
            self.hw_frames -= 1;
        }
        if let Some(irq) = self.serving.pop() {
            bus.note_irq_active(irq, false);
        }
        let restore = self
            .mie_stack
            .pop()
            .unwrap_or((self.mstatus >> 7) & 1 == 1);
        self.mstatus = (self.mstatus & !0x8) | (u32::from(restore) << 3);
        self.mstatus |= 0x80;
        self.pc = self.mepc;
        Ok(())
    }

    fn exec32(&mut self, bus: &mut impl Bus, pc: u32, inst: u32) -> Result<(), Fault> {
        let opcode = inst & 0x7f;
        let rd = ((inst >> 7) & 0x1f) as usize;
        let funct3 = (inst >> 12) & 7;
        let rs1 = ((inst >> 15) & 0x1f) as usize;
        let rs2 = ((inst >> 20) & 0x1f) as usize;
        let funct7 = (inst >> 25) & 0x7f;
        let imm_i = sext(inst >> 20, 12);
        let imm_s = sext(((inst >> 25) << 5) | ((inst >> 7) & 0x1f), 12);
        let imm_b = sext(
            (((inst >> 31) & 1) << 12)
                | (((inst >> 7) & 1) << 11)
                | (((inst >> 25) & 0x3f) << 5)
                | (((inst >> 8) & 0xf) << 1),
            13,
        );
        let imm_u = inst & 0xffff_f000;
        let imm_j = sext(
            (((inst >> 31) & 1) << 20)
                | (((inst >> 12) & 0xff) << 12)
                | (((inst >> 20) & 1) << 11)
                | (((inst >> 21) & 0x3ff) << 1),
            21,
        );
        let next = pc.wrapping_add(4);
        match opcode {
            0x37 => {
                self.set_x(rd, imm_u);
                self.pc = next;
            }
            0x17 => {
                self.set_x(rd, pc.wrapping_add(imm_u));
                self.pc = next;
            }
            0x6f => {
                self.set_x(rd, next);
                self.pc = pc.wrapping_add(imm_j);
            }
            0x67 => {
                if funct3 != 0 {
                    return Err(Fault::IllegalInstruction);
                }
                let target = self.x(rs1).wrapping_add(imm_i) & !1;
                self.set_x(rd, next);
                self.pc = target;
            }
            0x63 => {
                let a = self.x(rs1);
                let b = self.x(rs2);
                let take = match funct3 {
                    0 => a == b,
                    1 => a != b,
                    4 => (a as i32) < (b as i32),
                    5 => (a as i32) >= (b as i32),
                    6 => a < b,
                    7 => a >= b,
                    _ => return Err(Fault::IllegalInstruction),
                };
                self.pc = if take { pc.wrapping_add(imm_b) } else { next };
            }
            0x03 => {
                let addr = self.x(rs1).wrapping_add(imm_i);
                let val = match funct3 {
                    0 => sext(bus.load8(addr)? as u32, 8),
                    1 => sext(bus.load16(addr)? as u32, 16),
                    2 => bus.load32(addr)?,
                    4 => bus.load8(addr)? as u32,
                    5 => bus.load16(addr)? as u32,
                    _ => return Err(Fault::IllegalInstruction),
                };
                self.set_x(rd, val);
                self.pc = next;
            }
            0x23 => {
                let addr = self.x(rs1).wrapping_add(imm_s);
                let val = self.x(rs2);
                match funct3 {
                    0 => bus.store8(addr, val as u8)?,
                    1 => bus.store16(addr, val as u16)?,
                    2 => bus.store32(addr, val)?,
                    _ => return Err(Fault::IllegalInstruction),
                }
                self.pc = next;
            }
            0x13 => {
                let a = self.x(rs1);
                let shamt = (imm_i & 0x1f) as u32;
                let val = match funct3 {
                    0 => a.wrapping_add(imm_i),
                    2 => ((a as i32) < (imm_i as i32)) as u32,
                    3 => (a < imm_i) as u32,
                    4 => a ^ imm_i,
                    6 => a | imm_i,
                    7 => a & imm_i,
                    1 if funct7 == 0 => a << shamt,
                    5 if funct7 == 0 => a >> shamt,
                    5 if funct7 == 0x20 => ((a as i32) >> shamt) as u32,
                    _ => return Err(Fault::IllegalInstruction),
                };
                self.set_x(rd, val);
                self.pc = next;
            }
            0x33 => {
                let a = self.x(rs1);
                let b = self.x(rs2);
                let val = match (funct3, funct7) {
                    (0, 0) => a.wrapping_add(b),
                    (0, 0x20) => a.wrapping_sub(b),
                    (0, 1) => a.wrapping_mul(b),
                    (1, 0) => a << (b & 0x1f),
                    (1, 1) => (((a as i32 as i64) * (b as i32 as i64)) >> 32) as u32,
                    (2, 0) => ((a as i32) < (b as i32)) as u32,
                    (2, 1) => {
                        let wide = (a as i32 as i64) * (b as u64 as i64);
                        (wide >> 32) as u32
                    }
                    (3, 0) => (a < b) as u32,
                    (3, 1) => (((a as u64) * (b as u64)) >> 32) as u32,
                    (4, 0) => a ^ b,
                    (4, 1) => div_s(a, b),
                    (5, 0) => a >> (b & 0x1f),
                    (5, 0x20) => ((a as i32) >> (b & 0x1f)) as u32,
                    (5, 1) => if b == 0 { 0xffff_ffff } else { a / b },
                    (6, 0) => a | b,
                    (6, 1) => rem_s(a, b),
                    (7, 0) => a & b,
                    (7, 1) => if b == 0 { a } else { a % b },
                    _ => return Err(Fault::IllegalInstruction),
                };
                self.set_x(rd, val);
                self.pc = next;
            }
            0x0f => {
                self.pc = next;
            }
            0x73 => self.system(bus, rd, funct3, rs1, imm_i, next)?,
            _ => return Err(Fault::IllegalInstruction),
        }
        Ok(())
    }

    fn system(
        &mut self,
        bus: &mut impl Bus,
        rd: usize,
        funct3: u32,
        rs1: usize,
        imm_i: u32,
        next: u32,
    ) -> Result<(), Fault> {
        if funct3 == 0 {
            match imm_i {
                0 => return Err(Fault::EnvCall),
                1 => return Err(Fault::Breakpoint),
                0x302 => {
                    self.mret(bus)?;
                    return Ok(());
                }
                0x105 => {
                    // WFI. Firmware also sets WFITOWFE. Keep retiring so a poll loop advances.
                    self.pc = next;
                    return Ok(());
                }
                _ => return Err(Fault::IllegalInstruction),
            }
        }
        let csr = (imm_i & 0xfff) as u16;
        let old = self.csr_read(csr);
        let new = match funct3 {
            1 => self.x(rs1),
            2 => {
                if rs1 != 0 {
                    old | self.x(rs1)
                } else {
                    old
                }
            }
            3 => {
                if rs1 != 0 {
                    old & !self.x(rs1)
                } else {
                    old
                }
            }
            5 => rs1 as u32,
            6 => old | (rs1 as u32),
            7 => old & !(rs1 as u32),
            _ => return Err(Fault::IllegalInstruction),
        };
        let write = !matches!(funct3, 2 | 3 | 6 | 7) || rs1 != 0;
        if write {
            self.csr_write(csr, new);
        }
        self.set_x(rd, old);
        self.pc = next;
        Ok(())
    }

    fn csr_read(&self, csr: u16) -> u32 {
        match csr {
            0x300 => self.mstatus,
            0x301 => 0x4000_1104, // RV32IMC
            0x304 => self.mie,
            0x305 => self.mtvec,
            0x320 => 0,
            0x340 => self.mscratch,
            0x341 => self.mepc,
            0x342 => self.mcause,
            0x343 => self.mtval,
            0x344 => self.mip,
            0x800 => self.mstatus & 0x88,
            0x804 => self.intsyscr,
            0xbc0 => self.corecfgr,
            0xb00 => self.cycle as u32,
            0xb80 => (self.cycle >> 32) as u32,
            0xb02 => self.instret as u32,
            0xb82 => (self.instret >> 32) as u32,
            0xf11 | 0xf12 | 0xf13 | 0xf14 => 0,
            other => self.custom_csr.get(&other).copied().unwrap_or(0),
        }
    }

    fn csr_write(&mut self, csr: u16, val: u32) {
        match csr {
            0x300 => self.mstatus = val,
            0x304 => self.mie = val,
            0x305 => self.mtvec = val,
            0x340 => self.mscratch = val,
            0x341 => self.mepc = val,
            0x342 => self.mcause = val,
            0x343 => self.mtval = val,
            0x344 => self.mip = val,
            0x800 => self.mstatus = (self.mstatus & !0x88) | (val & 0x88),
            0x804 => self.intsyscr = val,
            0xbc0 => self.corecfgr = val,
            0xb00 => self.cycle = (self.cycle & !0xffff_ffff) | val as u64,
            0xb80 => self.cycle = (self.cycle & 0xffff_ffff) | ((val as u64) << 32),
            0xb02 => self.instret = (self.instret & !0xffff_ffff) | val as u64,
            0xb82 => self.instret = (self.instret & 0xffff_ffff) | ((val as u64) << 32),
            0x301 | 0xf11 | 0xf12 | 0xf13 | 0xf14 => {}
            other => {
                self.custom_csr.insert(other, val);
            }
        }
    }

    fn exec_c(&mut self, bus: &mut impl Bus, pc: u32, inst: u16) -> Result<(), Fault> {
        let op = inst & 0b11;
        let funct3 = (inst >> 13) & 0b111;
        let next = pc.wrapping_add(2);
        let rdp = |i: u16| (((i >> 2) & 0x7) + 8) as usize;
        let rs1p = |i: u16| (((i >> 7) & 0x7) + 8) as usize;
        match (op, funct3) {
            (0, 0) => {
                // C.ADDI4SPN
                let rd = rdp(inst);
                let imm = ((((inst >> 11) & 0x3) as u32) << 4)
                    | ((((inst >> 7) & 0xf) as u32) << 6)
                    | ((((inst >> 6) & 1) as u32) << 2)
                    | ((((inst >> 5) & 1) as u32) << 3);
                if imm == 0 {
                    return Err(Fault::IllegalInstruction);
                }
                self.set_x(rd, self.x(2).wrapping_add(imm));
                self.pc = next;
            }
            (0, 2) => {
                // C.LW
                let rd = rdp(inst);
                let rs1 = rs1p(inst);
                let off = ((((inst >> 10) & 0x7) as u32) << 3)
                    | ((((inst >> 6) & 1) as u32) << 2)
                    | ((((inst >> 5) & 1) as u32) << 6);
                let addr = self.x(rs1).wrapping_add(off);
                let val = bus.load32(addr)?;
                self.set_x(rd, val);
                self.pc = next;
            }
            (0, 6) => {
                // C.SW
                let rs2 = rdp(inst);
                let rs1 = rs1p(inst);
                let off = ((((inst >> 10) & 0x7) as u32) << 3)
                    | ((((inst >> 6) & 1) as u32) << 2)
                    | ((((inst >> 5) & 1) as u32) << 6);
                let addr = self.x(rs1).wrapping_add(off);
                bus.store32(addr, self.x(rs2))?;
                self.pc = next;
            }
            (1, 0) => {
                // C.ADDI / C.NOP
                let rd = ((inst >> 7) & 0x1f) as usize;
                let imm = c_imm_i(inst);
                self.set_x(rd, self.x(rd).wrapping_add(imm));
                self.pc = next;
            }
            (1, 1) => {
                // C.JAL (RV32)
                let imm = c_j_imm(inst);
                self.set_x(1, next);
                self.pc = pc.wrapping_add(imm);
            }
            (1, 2) => {
                // C.LI
                let rd = ((inst >> 7) & 0x1f) as usize;
                self.set_x(rd, c_imm_i(inst));
                self.pc = next;
            }
            (1, 3) => {
                let rd = ((inst >> 7) & 0x1f) as usize;
                if rd == 2 {
                    // C.ADDI16SP
                    let imm = c_addi16sp_imm(inst);
                    if imm == 0 {
                        return Err(Fault::IllegalInstruction);
                    }
                    self.set_x(2, self.x(2).wrapping_add(imm));
                } else if rd != 0 {
                    // C.LUI
                    let imm = c_lui_imm(inst);
                    if imm == 0 {
                        return Err(Fault::IllegalInstruction);
                    }
                    self.set_x(rd, imm);
                } else {
                    return Err(Fault::IllegalInstruction);
                }
                self.pc = next;
            }
            (1, 4) => {
                let rd = rs1p(inst);
                let funct2 = (inst >> 10) & 0b11;
                match funct2 {
                    0 | 1 => {
                        let shamt = ((inst >> 2) & 0x1f) as u32;
                        if (inst & (1 << 12)) != 0 {
                            return Err(Fault::IllegalInstruction);
                        }
                        let val = if funct2 == 0 {
                            self.x(rd) >> shamt
                        } else {
                            ((self.x(rd) as i32) >> shamt) as u32
                        };
                        self.set_x(rd, val);
                    }
                    2 => {
                        let imm = c_imm_i(inst);
                        self.set_x(rd, self.x(rd) & imm);
                    }
                    3 => {
                        if inst & (1 << 12) != 0 {
                            return Err(Fault::IllegalInstruction);
                        }
                        let rs2 = rdp(inst);
                        let op = (inst >> 5) & 0b11;
                        let a = self.x(rd);
                        let b = self.x(rs2);
                        let val = match op {
                            0 => a.wrapping_sub(b),
                            1 => a ^ b,
                            2 => a | b,
                            _ => a & b,
                        };
                        self.set_x(rd, val);
                    }
                    _ => unreachable!(),
                }
                self.pc = next;
            }
            (1, 5) => {
                self.pc = pc.wrapping_add(c_j_imm(inst));
            }
            (1, 6) | (1, 7) => {
                let rs1 = rs1p(inst);
                let imm = c_b_imm(inst);
                let take = if funct3 == 6 {
                    self.x(rs1) == 0
                } else {
                    self.x(rs1) != 0
                };
                self.pc = if take { pc.wrapping_add(imm) } else { next };
            }
            (2, 0) => {
                // C.SLLI
                let rd = ((inst >> 7) & 0x1f) as usize;
                let shamt = (((inst >> 12) & 1) << 5 | ((inst >> 2) & 0x1f)) as u32;
                if shamt & 0x20 != 0 {
                    return Err(Fault::IllegalInstruction);
                }
                self.set_x(rd, self.x(rd) << shamt);
                self.pc = next;
            }
            (2, 2) => {
                // C.LWSP
                let rd = ((inst >> 7) & 0x1f) as usize;
                if rd == 0 {
                    return Err(Fault::IllegalInstruction);
                }
                let off = ((((inst >> 12) & 1) as u32) << 5)
                    | ((((inst >> 4) & 0x7) as u32) << 2)
                    | ((((inst >> 2) & 0x3) as u32) << 6);
                let val = bus.load32(self.x(2).wrapping_add(off))?;
                self.set_x(rd, val);
                self.pc = next;
            }
            (2, 4) => {
                let rd = ((inst >> 7) & 0x1f) as usize;
                let rs2 = ((inst >> 2) & 0x1f) as usize;
                let bit12 = (inst >> 12) & 1;
                if bit12 == 0 {
                    if rs2 == 0 {
                        if rd == 0 {
                            return Err(Fault::IllegalInstruction);
                        }
                        self.pc = self.x(rd) & !1;
                    } else {
                        self.set_x(rd, self.x(rs2));
                        self.pc = next;
                    }
                } else if rs2 == 0 {
                    if rd == 0 {
                        return Err(Fault::Breakpoint);
                    }
                    let target = self.x(rd) & !1;
                    self.set_x(1, next);
                    self.pc = target;
                } else {
                    self.set_x(rd, self.x(rd).wrapping_add(self.x(rs2)));
                    self.pc = next;
                }
            }
            (2, 6) => {
                // C.SWSP
                let rs2 = ((inst >> 2) & 0x1f) as usize;
                let off = ((((inst >> 9) & 0xf) as u32) << 2) | ((((inst >> 7) & 0x3) as u32) << 6);
                bus.store32(self.x(2).wrapping_add(off), self.x(rs2))?;
                self.pc = next;
            }
            _ => return Err(Fault::IllegalInstruction),
        }
        Ok(())
    }
}

fn div_s(a: u32, b: u32) -> u32 {
    if b == 0 {
        return 0xffff_ffff;
    }
    if a == 0x8000_0000 && b == 0xffff_ffff {
        return a;
    }
    ((a as i32) / (b as i32)) as u32
}

fn rem_s(a: u32, b: u32) -> u32 {
    if b == 0 {
        return a;
    }
    if a == 0x8000_0000 && b == 0xffff_ffff {
        return 0;
    }
    ((a as i32) % (b as i32)) as u32
}

fn sext(val: u32, bits: u32) -> u32 {
    let shift = 32 - bits;
    ((val << shift) as i32 >> shift) as u32
}

fn c_imm_i(inst: u16) -> u32 {
    let imm = (((inst >> 12) & 1) << 5) | ((inst >> 2) & 0x1f);
    sext(imm as u32, 6)
}

fn c_j_imm(inst: u16) -> u32 {
    let imm = (((inst >> 12) & 1) << 11)
        | (((inst >> 11) & 1) << 4)
        | (((inst >> 9) & 0x3) << 8)
        | (((inst >> 8) & 1) << 10)
        | (((inst >> 7) & 1) << 6)
        | (((inst >> 6) & 1) << 7)
        | (((inst >> 3) & 0x7) << 1)
        | (((inst >> 2) & 1) << 5);
    sext(imm as u32, 12)
}

fn c_b_imm(inst: u16) -> u32 {
    let imm = (((inst >> 12) & 1) << 8)
        | (((inst >> 10) & 0x3) << 3)
        | (((inst >> 5) & 0x3) << 6)
        | (((inst >> 3) & 0x3) << 1)
        | (((inst >> 2) & 1) << 5);
    sext(imm as u32, 9)
}

fn c_addi16sp_imm(inst: u16) -> u32 {
    let imm = (((inst >> 12) & 1) << 9)
        | (((inst >> 6) & 1) << 4)
        | (((inst >> 5) & 1) << 6)
        | (((inst >> 3) & 0x3) << 7)
        | (((inst >> 2) & 1) << 5);
    sext(imm as u32, 10)
}

fn c_lui_imm(inst: u16) -> u32 {
    let imm17_12 = (((inst >> 12) & 1) << 5) | ((inst >> 2) & 0x1f);
    sext((imm17_12 as u32) << 12, 18)
}

pub fn disasm(inst: u32) -> String {
    let opcode = inst & 0x7f;
    let rd = (inst >> 7) & 0x1f;
    let rs1 = (inst >> 15) & 0x1f;
    let rs2 = (inst >> 20) & 0x1f;
    let funct3 = (inst >> 12) & 7;
    match opcode {
        0x37 => format!("lui x{rd}, {:#x}", inst >> 12),
        0x17 => format!("auipc x{rd}, {:#x}", inst >> 12),
        0x6f => format!("jal x{rd}"),
        0x67 => format!("jalr x{rd}, x{rs1}"),
        0x63 => format!("b{funct3} x{rs1}, x{rs2}"),
        0x03 => format!("l{funct3} x{rd}, x{rs1}"),
        0x23 => format!("s{funct3} x{rs2}, x{rs1}"),
        0x13 => format!("opi{funct3} x{rd}, x{rs1}"),
        0x33 => format!("op{funct3} x{rd}, x{rs1}, x{rs2}"),
        0x73 => "system".to_string(),
        0x0f => "fence".to_string(),
        _ => format!(".word {inst:#010x}"),
    }
}

pub fn disasm_c(inst: u16) -> String {
    format!("c.{inst:04x}")
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Mem {
        data: [u8; 4096],
        base: u32,
    }

    impl Mem {
        fn new() -> Self {
            Self {
                data: [0; 4096],
                base: 0x0800_0000,
            }
        }
        fn store_words(&mut self, words: &[u32]) {
            for (i, w) in words.iter().enumerate() {
                self.data[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
            }
        }
        fn off(&self, addr: u32) -> Result<usize, Fault> {
            let off = addr.wrapping_sub(self.base) as usize;
            if off >= self.data.len() {
                Err(Fault::LoadFault)
            } else {
                Ok(off)
            }
        }
    }

    impl Bus for Mem {
        fn load8(&mut self, addr: u32) -> Result<u8, Fault> {
            Ok(self.data[self.off(addr)?])
        }
        fn load16(&mut self, addr: u32) -> Result<u16, Fault> {
            let o = self.off(addr)?;
            Ok(u16::from_le_bytes([self.data[o], self.data[o + 1]]))
        }
        fn load32(&mut self, addr: u32) -> Result<u32, Fault> {
            let o = self.off(addr)?;
            Ok(u32::from_le_bytes(self.data[o..o + 4].try_into().unwrap()))
        }
        fn store8(&mut self, addr: u32, val: u8) -> Result<(), Fault> {
            self.data[self.off(addr)?] = val;
            Ok(())
        }
        fn store16(&mut self, addr: u32, val: u16) -> Result<(), Fault> {
            let o = self.off(addr)?;
            self.data[o..o + 2].copy_from_slice(&val.to_le_bytes());
            Ok(())
        }
        fn store32(&mut self, addr: u32, val: u32) -> Result<(), Fault> {
            let o = self.off(addr)?;
            self.data[o..o + 4].copy_from_slice(&val.to_le_bytes());
            Ok(())
        }
        fn pending_irqs(&self) -> Vec<u32> {
            Vec::new()
        }
        fn vector(&mut self, _irq: u32, _mtvec: u32) -> Result<u32, Fault> {
            Ok(0)
        }
        fn note_irq_active(&mut self, _: u32, _: bool) {}
    }

    #[test]
    fn addi_sw_lw_and_branch() {
        let mut mem = Mem::new();
        // addi x1, x0, 5
        // sw x1, 0(x0) would be address 0, outside mem. Use x2 as base after lui.
        // lui x2, 0x08000 ; addi x2, x2, 0x100 ; addi x1, x0, 42 ; sw x1, 0(x2) ; lw x3, 0(x2)
        mem.store_words(&[
            0x0800_0137, // lui x2, 0x08000
            0x1001_0113, // addi x2, x2, 0x100
            0x02a0_0093, // addi x1, x0, 42
            0x0011_2023, // sw x1, 0(x2)
            0x0001_2183, // lw x3, 0(x2)
            0x0010_006f, // jal x0, +0? we'll check x3 before falling off
        ]);
        let mut cpu = Cpu::default();
        cpu.pc = mem.base;
        for _ in 0..5 {
            cpu.step(&mut mem).unwrap();
        }
        assert_eq!(cpu.x(1), 42);
        assert_eq!(cpu.x(3), 42);
    }

    #[test]
    fn compressed_li_and_add() {
        let mut mem = Mem::new();
        // c.li x1, 1  = 0x4085? let's use known encodings and check result via step
        // c.li a0, 1 : rd=10
        // bits 15:13=010, bit12=imm5=0, bits11:7=01010, bits6:2=00001, bits1:0=01
        // 010 0 01010 00001 01 = 0100 0101 0000 0101 = 0x4505
        // c.add a0, a0 : op=10, funct3=100, bit12=1, rd=10, rs2=10
        // 100 1 01010 0 01010 10 = 1001 0101 0001 01010 wait bits 6:2 = rs2 = 01010, bits 1:0 = 10
        // 1001 0101 0 01010 10 = 1001 0101 0010 1010 = 0x952a
        mem.data[0] = 0x05;
        mem.data[1] = 0x45;
        mem.data[2] = 0x2a;
        mem.data[3] = 0x95;
        let mut cpu = Cpu::default();
        cpu.pc = mem.base;
        cpu.step(&mut mem).unwrap();
        assert_eq!(cpu.x(10), 1);
        cpu.step(&mut mem).unwrap();
        assert_eq!(cpu.x(10), 2);
    }

    #[test]
    fn mul_and_div() {
        let mut mem = Mem::new();
        mem.store_words(&[
            0x0060_0093, // addi x1, x0, 6
            0x0070_0113, // addi x2, x0, 7
            0x0220_80b3, // mul x1, x1, x2
            0x0220_c1b3, // div x3, x1, x2   funct7=1 funct3=4 rd=3 rs1=1 rs2=2
        ]);
        let mut cpu = Cpu::default();
        cpu.pc = mem.base;
        for _ in 0..4 {
            cpu.step(&mut mem).unwrap();
        }
        assert_eq!(cpu.x(1), 42);
        assert_eq!(cpu.x(3), 6);
    }

    #[test]
    fn gintenr_maps_onto_mstatus_mie() {
        let mut mem = Mem::new();
        // csrrsi x1, 0x800, 8
        // csr = 0x800, zimm = 8, funct3 = 6, rd = 1, opcode = 0x73
        // imm[11:0]=0x800, rs1=8, funct3=6, rd=1
        // 0x800 << 20 | 8 << 15 | 6 << 12 | 1 << 7 | 0x73
        let csrrsi = (0x800u32 << 20) | (8 << 15) | (6 << 12) | (1 << 7) | 0x73;
        mem.store_words(&[csrrsi]);
        let mut cpu = Cpu::default();
        cpu.pc = mem.base;
        assert!(!cpu.interrupts_enabled());
        cpu.step(&mut mem).unwrap();
        assert!(cpu.interrupts_enabled());
    }
}
