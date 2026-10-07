//! SPI and I2C register models. The machine moves the bytes between these and attached devices.

pub(crate) const SPI_CTLR1: u32 = 0x00;
pub(crate) const SPI_CTLR2: u32 = 0x04;
pub(crate) const SPI_STATR: u32 = 0x08;
pub(crate) const SPI_DATAR: u32 = 0x0c;

pub(crate) const SPI_RXNE: u32 = 1 << 0;
pub(crate) const SPI_TXE: u32 = 1 << 1;
const SPI_CRCERR: u32 = 1 << 4;
const SPI_MODF: u32 = 1 << 5;
pub(crate) const SPI_OVR: u32 = 1 << 6;

const SPI_CPHA: u32 = 1 << 0;
const SPI_CPOL: u32 = 1 << 1;
const SPI_MSTR: u32 = 1 << 2;
const SPI_SPE: u32 = 1 << 6;
const SPI_LSBFIRST: u32 = 1 << 7;
const SPI_DFF: u32 = 1 << 11;

const SPI_ERRIE: u32 = 1 << 5;
const SPI_RXNEIE: u32 = 1 << 6;
const SPI_TXEIE: u32 = 1 << 7;

/// One SPI block in master mode. A frame finishes in the same instruction that writes DATAR.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Spi {
    pub ctlr1: u32,
    pub ctlr2: u32,
    pub statr: u32,
    pub crcpr: u32,
    pub rx: u16,
    pub frames: u64,
    pub last_tx: Option<u16>,
    pub last_rx: Option<u16>,
}

impl Spi {
    pub fn new() -> Self {
        Self {
            ctlr1: 0,
            ctlr2: 0,
            statr: SPI_TXE,
            crcpr: 7,
            rx: 0,
            frames: 0,
            last_tx: None,
            last_rx: None,
        }
    }

    pub fn enabled_master(&self) -> bool {
        self.ctlr1 & (SPI_SPE | SPI_MSTR) == SPI_SPE | SPI_MSTR
    }

    pub fn sixteen_bit(&self) -> bool {
        self.ctlr1 & SPI_DFF != 0
    }

    pub fn lsb_first(&self) -> bool {
        self.ctlr1 & SPI_LSBFIRST != 0
    }

    pub fn mode(&self) -> u8 {
        (u8::from(self.ctlr1 & SPI_CPOL != 0) << 1) | u8::from(self.ctlr1 & SPI_CPHA != 0)
    }

    /// Register value without read side effects.
    pub fn read(&self, offset: u32) -> u32 {
        match offset {
            SPI_CTLR1 => self.ctlr1,
            SPI_CTLR2 => self.ctlr2,
            SPI_STATR => self.statr,
            SPI_DATAR => self.rx as u32,
            0x10 => self.crcpr,
            _ => 0,
        }
    }

    /// A load from DATAR empties the receive buffer.
    pub fn read_data(&mut self) -> u32 {
        self.statr &= !(SPI_RXNE | SPI_OVR);
        self.rx as u32
    }

    /// Returns the frame to shift out when the write starts a transfer.
    pub fn write(&mut self, offset: u32, value: u32) -> Option<u16> {
        match offset {
            SPI_CTLR1 => self.ctlr1 = value & 0xffff,
            SPI_CTLR2 => self.ctlr2 = value & 0xff,
            SPI_STATR => {
                if value & SPI_CRCERR == 0 {
                    self.statr &= !SPI_CRCERR;
                }
            }
            SPI_DATAR => {
                let mask = if self.sixteen_bit() { 0xffff } else { 0xff };
                if self.enabled_master() {
                    return Some((value & mask) as u16);
                }
            }
            0x10 => self.crcpr = value & 0xffff,
            _ => {}
        }
        None
    }

    /// The frame shifted in while `tx` went out.
    pub fn complete(&mut self, tx: u16, rx: u16) {
        if self.statr & SPI_RXNE != 0 {
            self.statr |= SPI_OVR;
        } else {
            self.rx = rx;
        }
        self.statr |= SPI_RXNE | SPI_TXE;
        self.frames += 1;
        self.last_tx = Some(tx);
        self.last_rx = Some(rx);
    }

    pub fn irq_asserted(&self) -> bool {
        let s = self.statr;
        let c = self.ctlr2;
        (c & SPI_RXNEIE != 0 && s & SPI_RXNE != 0)
            || (c & SPI_TXEIE != 0 && s & SPI_TXE != 0)
            || (c & SPI_ERRIE != 0 && s & (SPI_OVR | SPI_MODF | SPI_CRCERR) != 0)
    }

    /// Bytes on the wire for one frame, in order. Each byte is MSB-first as devices expect.
    pub fn wire_bytes(&self, frame: u16) -> Vec<u8> {
        if self.sixteen_bit() {
            let bits = if self.lsb_first() {
                frame.reverse_bits()
            } else {
                frame
            };
            bits.to_be_bytes().to_vec()
        } else {
            let byte = frame as u8;
            vec![if self.lsb_first() {
                byte.reverse_bits()
            } else {
                byte
            }]
        }
    }

    /// Inverse of [`Spi::wire_bytes`] for the MISO side.
    pub fn frame_from_wire(&self, wire: &[u8]) -> u16 {
        if self.sixteen_bit() {
            let bits = u16::from_be_bytes([wire[0], wire[1]]);
            if self.lsb_first() {
                bits.reverse_bits()
            } else {
                bits
            }
        } else if self.lsb_first() {
            wire[0].reverse_bits() as u16
        } else {
            wire[0] as u16
        }
    }
}

pub(crate) const I2C_CTLR1: u32 = 0x00;
pub(crate) const I2C_CTLR2: u32 = 0x04;
pub(crate) const I2C_OADDR1: u32 = 0x08;
pub(crate) const I2C_OADDR2: u32 = 0x0c;
pub(crate) const I2C_DATAR: u32 = 0x10;
pub(crate) const I2C_STAR1: u32 = 0x14;
pub(crate) const I2C_STAR2: u32 = 0x18;
pub(crate) const I2C_CKCFGR: u32 = 0x1c;
pub(crate) const I2C_RTR: u32 = 0x20;

const I2C_PE: u32 = 1 << 0;
const I2C_START: u32 = 1 << 8;
const I2C_STOP: u32 = 1 << 9;
pub(crate) const I2C_ACK: u32 = 1 << 10;
const I2C_SWRST: u32 = 1 << 15;

pub(crate) const I2C_SB: u32 = 1 << 0;
pub(crate) const I2C_ADDR: u32 = 1 << 1;
pub(crate) const I2C_BTF: u32 = 1 << 2;
pub(crate) const I2C_RXNE: u32 = 1 << 6;
pub(crate) const I2C_TXE: u32 = 1 << 7;
pub(crate) const I2C_AF: u32 = 1 << 10;
const I2C_ERR_MASK: u32 = 0xdf00;

const I2C_MSL: u32 = 1 << 0;
const I2C_BUSY: u32 = 1 << 1;
const I2C_TRA: u32 = 1 << 2;

const I2C_ITERREN: u32 = 1 << 8;
const I2C_ITEVTEN: u32 = 1 << 9;
const I2C_ITBUFEN: u32 = 1 << 10;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum I2cPhase {
    Idle,
    /// START sent. The next DATAR write is the address byte.
    Address,
    /// The addressed target NACKed. Firmware must send STOP.
    Nacked,
    Transmit,
    Receive,
}

/// What the machine must do with the attached targets after a register access.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum I2cAction {
    None,
    /// Address byte: seven-bit address and the read bit.
    Address(u8, bool),
    Write(u8),
    Read,
    Stop(u8),
}

/// One I2C block in controller mode. Bytes move in the instruction that triggers them.
#[derive(Clone, Copy, Debug)]
pub(crate) struct I2c {
    pub ctlr1: u32,
    pub ctlr2: u32,
    pub oaddr1: u32,
    pub oaddr2: u32,
    pub data: u8,
    pub star1: u32,
    pub star2: u32,
    pub ckcfgr: u32,
    pub rtr: u32,
    pub phase: I2cPhase,
    pub target: Option<u8>,
    pub bytes: u64,
}

impl I2c {
    pub fn new() -> Self {
        Self {
            ctlr1: 0,
            ctlr2: 0,
            oaddr1: 0,
            oaddr2: 0,
            data: 0,
            star1: 0,
            star2: 0,
            ckcfgr: 0,
            rtr: 2,
            phase: I2cPhase::Idle,
            target: None,
            bytes: 0,
        }
    }

    pub fn read(&self, offset: u32) -> u32 {
        match offset {
            I2C_CTLR1 => self.ctlr1,
            I2C_CTLR2 => self.ctlr2,
            I2C_OADDR1 => self.oaddr1,
            I2C_OADDR2 => self.oaddr2,
            I2C_DATAR => self.data as u32,
            I2C_STAR1 => self.star1,
            I2C_STAR2 => self.star2,
            I2C_CKCFGR => self.ckcfgr,
            I2C_RTR => self.rtr,
            _ => 0,
        }
    }

    /// A load with side effects. STAR2 clears ADDR, DATAR empties the receive buffer.
    pub fn load(&mut self, offset: u32) -> (u32, I2cAction) {
        let value = self.read(offset);
        let mut action = I2cAction::None;
        match offset {
            I2C_STAR2 if self.star1 & I2C_ADDR != 0 => {
                self.star1 &= !I2C_ADDR;
                match self.phase {
                    I2cPhase::Transmit => self.star1 |= I2C_TXE,
                    I2cPhase::Receive => action = I2cAction::Read,
                    _ => {}
                }
            }
            I2C_DATAR if self.star1 & I2C_RXNE != 0 => {
                self.star1 &= !(I2C_RXNE | I2C_BTF);
                let more = self.phase == I2cPhase::Receive
                    && self.ctlr1 & I2C_ACK != 0
                    && self.star2 & I2C_MSL != 0;
                if more {
                    action = I2cAction::Read;
                }
            }
            _ => {}
        }
        (value, action)
    }

    pub fn write(&mut self, offset: u32, value: u32) -> I2cAction {
        match offset {
            I2C_CTLR1 => return self.write_ctlr1(value),
            I2C_CTLR2 => self.ctlr2 = value & 0xffff,
            I2C_OADDR1 => self.oaddr1 = value & 0xffff,
            I2C_OADDR2 => self.oaddr2 = value & 0xffff,
            I2C_DATAR => return self.write_data(value as u8),
            I2C_STAR1 => self.star1 &= value | !I2C_ERR_MASK,
            I2C_CKCFGR => self.ckcfgr = value & 0xffff,
            I2C_RTR => self.rtr = value & 0x3f,
            _ => {}
        }
        I2cAction::None
    }

    fn write_ctlr1(&mut self, value: u32) -> I2cAction {
        if value & I2C_SWRST != 0 {
            let target = self.target;
            *self = Self::new();
            self.ctlr1 = value & 0xffff;
            return target.map_or(I2cAction::None, I2cAction::Stop);
        }
        self.ctlr1 = value & 0xffff & !(I2C_START | I2C_STOP);
        if value & I2C_PE == 0 {
            let target = self.target.take();
            self.star1 = 0;
            self.star2 = 0;
            self.phase = I2cPhase::Idle;
            return target.map_or(I2cAction::None, I2cAction::Stop);
        }
        if value & I2C_START != 0 {
            self.star1 = (self.star1 & !(I2C_ADDR | I2C_BTF | I2C_TXE)) | I2C_SB;
            self.star2 |= I2C_MSL | I2C_BUSY;
            self.phase = I2cPhase::Address;
            return I2cAction::None;
        }
        if value & I2C_STOP != 0 && self.star2 & I2C_MSL != 0 {
            let target = self.target.take();
            self.star1 &= !(I2C_SB | I2C_ADDR | I2C_BTF | I2C_TXE);
            self.star2 &= !(I2C_MSL | I2C_BUSY | I2C_TRA);
            self.phase = I2cPhase::Idle;
            return target.map_or(I2cAction::None, I2cAction::Stop);
        }
        I2cAction::None
    }

    fn write_data(&mut self, byte: u8) -> I2cAction {
        self.data = byte;
        match self.phase {
            I2cPhase::Address => {
                self.star1 &= !I2C_SB;
                I2cAction::Address(byte >> 1, byte & 1 != 0)
            }
            I2cPhase::Transmit => {
                self.star1 &= !(I2C_TXE | I2C_BTF);
                I2cAction::Write(byte)
            }
            _ => I2cAction::None,
        }
    }

    /// Result of the address phase.
    pub fn addressed(&mut self, address: u8, read: bool, ack: bool) {
        if ack {
            self.star1 |= I2C_ADDR;
            self.target = Some(address);
            if read {
                self.star2 &= !I2C_TRA;
                self.phase = I2cPhase::Receive;
            } else {
                self.star2 |= I2C_TRA;
                self.phase = I2cPhase::Transmit;
            }
        } else {
            self.star1 |= I2C_AF;
            self.target = None;
            self.phase = I2cPhase::Nacked;
        }
    }

    pub fn written(&mut self, ack: bool) {
        self.bytes += 1;
        if ack {
            self.star1 |= I2C_TXE | I2C_BTF;
        } else {
            self.star1 |= I2C_AF;
            self.phase = I2cPhase::Nacked;
        }
    }

    pub fn received(&mut self, byte: u8) {
        self.bytes += 1;
        self.data = byte;
        self.star1 |= I2C_RXNE;
    }

    pub fn ev_irq(&self) -> bool {
        let s = self.star1;
        let c = self.ctlr2;
        c & I2C_ITEVTEN != 0
            && (s & (I2C_SB | I2C_ADDR | I2C_BTF) != 0
                || (c & I2C_ITBUFEN != 0 && s & (I2C_TXE | I2C_RXNE) != 0))
    }

    pub fn er_irq(&self) -> bool {
        self.ctlr2 & I2C_ITERREN != 0 && self.star1 & I2C_ERR_MASK != 0
    }
}
