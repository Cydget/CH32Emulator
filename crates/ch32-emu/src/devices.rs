//! Off-chip components wired to the emulated SPI, USART, and I2C peripherals.
//!
//! A component implements [`SpiDevice`], [`UartDevice`], or [`I2cDevice`] and is attached to a bus
//! with [`crate::machine::Machine::attach`]. The machine calls into the component when firmware
//! moves data on that bus, so the component sees the same byte stream a real part would.
//! Every call carries a [`DeviceCtx`] with the guest cycle, which is the component's only clock.

/// Bus timing seen by a component when the machine calls into it.
#[derive(Clone, Copy, Debug)]
pub struct DeviceCtx {
    /// Guest cycle at the start of the instruction that moved the data.
    pub cycle: u64,
}

/// A labeled value for the inspector.
#[derive(Clone, Debug, serde::Serialize)]
pub struct DeviceRow {
    pub name: String,
    pub value: String,
}

/// A number the user can change from the UI while the component is attached.
#[derive(Clone, Debug, serde::Serialize)]
pub struct DeviceSetting {
    pub key: String,
    pub label: String,
    pub value: f64,
}

/// What the inspector shows for a component.
#[derive(Clone, Debug, Default)]
pub struct DeviceView {
    /// One or two sentences about what the component will do next.
    pub action: String,
    /// Signals the component is sampling (analog inputs, chip select, clocks).
    pub inputs: Vec<DeviceRow>,
    /// Internal registers and the data it will drive back to the microcontroller.
    pub outputs: Vec<DeviceRow>,
}

/// Behavior shared by every component, independent of the bus it sits on.
pub trait Component: Send {
    /// Short kind name, for example `"adc"`.
    fn kind(&self) -> &'static str;
    fn view(&self, ctx: &DeviceCtx) -> DeviceView;
    fn settings(&self) -> Vec<DeviceSetting> {
        Vec::new()
    }
    fn set(&mut self, key: &str, _value: f64) -> Result<(), String> {
        Err(format!("{} has no setting named {key}", self.kind()))
    }
    /// Power-on state. Called when the machine resets. User settings should survive.
    fn reset(&mut self) {}
}

/// A component on an SPI bus. The microcontroller is always the master.
pub trait SpiDevice: Component {
    /// Chip select changed. `true` means selected (CS driven low).
    /// Devices attached without a CS pin are selected once, at attach time.
    fn select(&mut self, selected: bool, ctx: &DeviceCtx);
    /// One byte on the wire, most significant bit first. Returns the MISO byte.
    fn transfer(&mut self, mosi: u8, ctx: &DeviceCtx) -> u8;
    fn clone_box(&self) -> Box<dyn SpiDevice>;
}

/// A component on a USART. Bytes are delivered whole; baud rate timing is not modeled.
pub trait UartDevice: Component {
    /// A byte the microcontroller transmitted.
    fn receive(&mut self, byte: u8, ctx: &DeviceCtx);
    /// Whether the component has a byte for the microcontroller. Called every instruction.
    fn has_byte(&self, ctx: &DeviceCtx) -> bool;
    /// Take the next byte. Called only after `has_byte` returned true and the receiver is empty.
    fn take_byte(&mut self, ctx: &DeviceCtx) -> u8;
    fn clone_box(&self) -> Box<dyn UartDevice>;
}

/// A target on an I2C bus. The microcontroller is always the controller.
pub trait I2cDevice: Component {
    /// Seven-bit address the component answers to.
    fn address(&self) -> u8;
    /// A START or repeated START addressed this component. `read` is the R/W bit.
    /// Return `false` to NACK the address.
    fn start(&mut self, read: bool, ctx: &DeviceCtx) -> bool;
    /// A byte written by the controller. Return `false` to NACK it.
    fn write(&mut self, byte: u8, ctx: &DeviceCtx) -> bool;
    /// The next byte the controller clocks in.
    fn read(&mut self, ctx: &DeviceCtx) -> u8;
    /// STOP ended the transaction.
    fn stop(&mut self, ctx: &DeviceCtx);
    fn clone_box(&self) -> Box<dyn I2cDevice>;
}

/// Component kinds the UI and `--attach` can create, with the buses each one fits.
pub const KINDS: &[(&str, &str, &[&str])] = &[(
    "adc",
    "4-channel 12-bit SPI ADC (example)",
    &["SPI1", "SPI2"],
)];

pub enum Created {
    Spi(Box<dyn SpiDevice>),
    Uart(Box<dyn UartDevice>),
    I2c(Box<dyn I2cDevice>),
}

/// Build a component by kind name.
pub fn create(kind: &str) -> Option<Created> {
    match kind {
        "adc" => Some(Created::Spi(Box::new(SimAdc::new()))),
        _ => None,
    }
}

/// Example SPI ADC: four channels, 12 bits, SPI mode 0 or 3, chip select active low.
///
/// Every transaction starts with a command byte while CS is low. Bit 7 set reads a register,
/// clear writes one. Bits 3:0 select the register. A write sends one data byte after the command.
/// A read clocks the register out on the following bytes, most significant byte first.
///
/// | Reg | Name   | Access | Meaning |
/// |-----|--------|--------|---------|
/// | 0   | ID     | R      | Always `0xAD` |
/// | 1   | CONFIG | R/W    | Bit 0 powers the converter. Bits 5:4 pick the channel. |
/// | 2   | STATUS | R      | Bit 0 powered, bit 7 a conversion is ready (DRDY). |
/// | 3   | DATA   | R      | Two bytes, the 12-bit result right-aligned. Reading it starts the next conversion. |
///
/// After power-up the first conversion is ready [`SimAdc::STARTUP_CYCLES`] later. A channel change
/// or a DATA read schedules the next one [`SimAdc::CONVERSION_CYCLES`] later. Reading DATA before
/// DRDY returns the previous result.
#[derive(Clone, Debug)]
pub struct SimAdc {
    config: u8,
    ready_at: Option<u64>,
    result: u16,
    shift: [u8; 2],
    phase: AdcPhase,
    conversions: u64,
    last_command: Option<u8>,
    selected: bool,
    vref: f64,
    sine_offset: f64,
    sine_amplitude: f64,
    sine_period: f64,
    ramp_period: f64,
    ch2: f64,
    ch3: f64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum AdcPhase {
    Command,
    WriteData(u8),
    Read { reg: u8, index: usize },
}

impl SimAdc {
    pub const ID: u8 = 0xAD;
    pub const REG_ID: u8 = 0;
    pub const REG_CONFIG: u8 = 1;
    pub const REG_STATUS: u8 = 2;
    pub const REG_DATA: u8 = 3;
    pub const CONFIG_PWR: u8 = 1 << 0;
    pub const STATUS_DRDY: u8 = 1 << 7;
    pub const STARTUP_CYCLES: u64 = 2_000;
    pub const CONVERSION_CYCLES: u64 = 400;

    pub fn new() -> Self {
        Self {
            config: 0,
            ready_at: None,
            result: 0,
            shift: [0; 2],
            phase: AdcPhase::Command,
            conversions: 0,
            last_command: None,
            selected: false,
            vref: 3.3,
            sine_offset: 1.65,
            sine_amplitude: 1.2,
            sine_period: 800_000.0,
            ramp_period: 2_000_000.0,
            ch2: 1.0,
            ch3: 2.5,
        }
    }

    fn powered(&self) -> bool {
        self.config & Self::CONFIG_PWR != 0
    }

    fn channel(&self) -> u8 {
        (self.config >> 4) & 3
    }

    fn ready(&self, cycle: u64) -> bool {
        self.powered() && self.ready_at.is_some_and(|at| cycle >= at)
    }

    /// Voltage on a channel at a guest cycle.
    pub fn input_volts(&self, channel: u8, cycle: u64) -> f64 {
        let t = cycle as f64;
        match channel {
            0 => {
                let phase = std::f64::consts::TAU * t / self.sine_period.max(1.0);
                self.sine_offset + self.sine_amplitude * phase.sin()
            }
            1 => self.vref * (t % self.ramp_period.max(1.0)) / self.ramp_period.max(1.0),
            2 => self.ch2,
            _ => self.ch3,
        }
    }

    fn code_for(&self, volts: f64) -> u16 {
        let ratio = (volts / self.vref).clamp(0.0, 1.0);
        (ratio * 4095.0).round() as u16
    }

    fn status(&self, cycle: u64) -> u8 {
        let mut status = 0;
        if self.powered() {
            status |= 1;
        }
        if self.ready(cycle) {
            status |= Self::STATUS_DRDY;
        }
        status
    }

    fn write_reg(&mut self, reg: u8, value: u8, cycle: u64) {
        if reg != Self::REG_CONFIG {
            return;
        }
        let was_on = self.powered();
        let old_channel = self.channel();
        self.config = value & 0x31;
        if !self.powered() {
            self.ready_at = None;
        } else if !was_on {
            self.ready_at = Some(cycle + Self::STARTUP_CYCLES);
        } else if self.channel() != old_channel {
            self.ready_at = Some(cycle + Self::CONVERSION_CYCLES);
        }
    }

    /// Bytes a read of `reg` will shift out.
    fn read_reg(&mut self, reg: u8, cycle: u64) -> [u8; 2] {
        match reg {
            Self::REG_ID => [Self::ID, 0],
            Self::REG_CONFIG => [self.config, 0],
            Self::REG_STATUS => [self.status(cycle), 0],
            Self::REG_DATA => {
                if !self.powered() {
                    return [0, 0];
                }
                if self.ready(cycle) {
                    let volts = self.input_volts(self.channel(), cycle);
                    self.result = self.code_for(volts);
                    self.conversions += 1;
                    self.ready_at = Some(cycle + Self::CONVERSION_CYCLES);
                }
                self.result.to_be_bytes()
            }
            _ => [0, 0],
        }
    }
}

impl Default for SimAdc {
    fn default() -> Self {
        Self::new()
    }
}

impl Component for SimAdc {
    fn kind(&self) -> &'static str {
        "adc"
    }

    fn view(&self, ctx: &DeviceCtx) -> DeviceView {
        let cycle = ctx.cycle;
        let channel = self.channel();
        let action = if !self.powered() {
            "Powered down. Write CONFIG (register 1) with bit 0 set to start converting. DATA reads return 0 until then.".to_string()
        } else if self.ready(cycle) {
            let volts = self.input_volts(channel, cycle);
            format!(
                "Conversion ready. A DATA read now returns {:#05x} ({volts:.3} V on channel {channel}) and starts the next conversion.",
                self.code_for(volts)
            )
        } else {
            let at = self.ready_at.unwrap_or(cycle);
            format!(
                "Converting channel {channel}. DRDY sets at cycle {at}, {} cycles from now. A DATA read before then returns the previous result {:#05x}.",
                at.saturating_sub(cycle),
                self.result
            )
        };
        let mut inputs = vec![DeviceRow {
            name: "chip select".into(),
            value: if self.selected {
                "selected (low)".into()
            } else {
                "idle (high)".into()
            },
        }];
        for ch in 0..4u8 {
            let source = match ch {
                0 => "sine",
                1 => "ramp",
                _ => "constant",
            };
            inputs.push(DeviceRow {
                name: format!("CH{ch}"),
                value: format!("{:.3} V ({source})", self.input_volts(ch, cycle)),
            });
        }
        inputs.push(DeviceRow {
            name: "VREF".into(),
            value: format!("{:.2} V", self.vref),
        });
        let phase = match self.phase {
            AdcPhase::Command => "waiting for a command byte".to_string(),
            AdcPhase::WriteData(reg) => format!("next byte is written to register {reg}"),
            AdcPhase::Read { reg, index } => {
                format!("shifting out register {reg}, byte {index}")
            }
        };
        let outputs = vec![
            DeviceRow {
                name: "ID".into(),
                value: format!("{:#04x}", Self::ID),
            },
            DeviceRow {
                name: "CONFIG".into(),
                value: format!(
                    "{:#04x} (power {}, channel {channel})",
                    self.config,
                    if self.powered() { "on" } else { "off" }
                ),
            },
            DeviceRow {
                name: "STATUS".into(),
                value: format!("{:#04x}", self.status(cycle)),
            },
            DeviceRow {
                name: "DATA".into(),
                value: format!(
                    "{:#05x} ({:.3} V)",
                    self.result,
                    self.result as f64 * self.vref / 4095.0
                ),
            },
            DeviceRow {
                name: "transaction".into(),
                value: phase,
            },
            DeviceRow {
                name: "last command".into(),
                value: self
                    .last_command
                    .map(|c| format!("{c:#04x}"))
                    .unwrap_or_else(|| "(none)".into()),
            },
            DeviceRow {
                name: "conversions".into(),
                value: self.conversions.to_string(),
            },
        ];
        DeviceView {
            action,
            inputs,
            outputs,
        }
    }

    fn settings(&self) -> Vec<DeviceSetting> {
        let s = |key: &str, label: &str, value: f64| DeviceSetting {
            key: key.into(),
            label: label.into(),
            value,
        };
        vec![
            s("sine_offset", "CH0 sine offset (V)", self.sine_offset),
            s(
                "sine_amplitude",
                "CH0 sine amplitude (V)",
                self.sine_amplitude,
            ),
            s("sine_period", "CH0 sine period (cycles)", self.sine_period),
            s("ramp_period", "CH1 ramp period (cycles)", self.ramp_period),
            s("ch2", "CH2 (V)", self.ch2),
            s("ch3", "CH3 (V)", self.ch3),
            s("vref", "VREF (V)", self.vref),
        ]
    }

    fn set(&mut self, key: &str, value: f64) -> Result<(), String> {
        if !value.is_finite() {
            return Err(format!("{key} must be a number"));
        }
        let slot = match key {
            "sine_offset" => &mut self.sine_offset,
            "sine_amplitude" => &mut self.sine_amplitude,
            "sine_period" => &mut self.sine_period,
            "ramp_period" => &mut self.ramp_period,
            "ch2" => &mut self.ch2,
            "ch3" => &mut self.ch3,
            "vref" => &mut self.vref,
            _ => return Err(format!("adc has no setting named {key}")),
        };
        let positive = matches!(key, "sine_period" | "ramp_period" | "vref");
        if positive && value <= 0.0 {
            return Err(format!("{key} must be above zero"));
        }
        *slot = value;
        Ok(())
    }

    fn reset(&mut self) {
        self.config = 0;
        self.ready_at = None;
        self.result = 0;
        self.shift = [0; 2];
        self.phase = AdcPhase::Command;
        self.conversions = 0;
        self.last_command = None;
    }
}

impl SpiDevice for SimAdc {
    fn select(&mut self, selected: bool, _ctx: &DeviceCtx) {
        self.selected = selected;
        self.phase = AdcPhase::Command;
    }

    fn transfer(&mut self, mosi: u8, ctx: &DeviceCtx) -> u8 {
        if !self.selected {
            return 0xff;
        }
        match self.phase {
            AdcPhase::Command => {
                self.last_command = Some(mosi);
                let reg = mosi & 0x0f;
                if mosi & 0x80 != 0 {
                    self.shift = self.read_reg(reg, ctx.cycle);
                    self.phase = AdcPhase::Read { reg, index: 0 };
                } else {
                    self.phase = AdcPhase::WriteData(reg);
                }
                0x00
            }
            AdcPhase::WriteData(reg) => {
                self.write_reg(reg, mosi, ctx.cycle);
                self.phase = AdcPhase::Command;
                0x00
            }
            AdcPhase::Read { reg, index } => {
                let out = self.shift.get(index).copied().unwrap_or(0);
                self.phase = AdcPhase::Read {
                    reg,
                    index: index + 1,
                };
                out
            }
        }
    }

    fn clone_box(&self) -> Box<dyn SpiDevice> {
        Box::new(self.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ctx(cycle: u64) -> DeviceCtx {
        DeviceCtx { cycle }
    }

    fn read(adc: &mut SimAdc, reg: u8, n: usize, cycle: u64) -> Vec<u8> {
        adc.select(true, &ctx(cycle));
        adc.transfer(0x80 | reg, &ctx(cycle));
        let out = (0..n).map(|_| adc.transfer(0, &ctx(cycle))).collect();
        adc.select(false, &ctx(cycle));
        out
    }

    #[test]
    fn adc_needs_power_up_then_converts_the_selected_channel() {
        let mut adc = SimAdc::new();
        assert_eq!(read(&mut adc, SimAdc::REG_ID, 1, 0), vec![0xAD]);
        assert_eq!(read(&mut adc, SimAdc::REG_DATA, 2, 0), vec![0, 0]);

        adc.select(true, &ctx(10));
        adc.transfer(SimAdc::REG_CONFIG, &ctx(10));
        adc.transfer(SimAdc::CONFIG_PWR | (2 << 4), &ctx(10));
        adc.select(false, &ctx(10));
        assert_eq!(read(&mut adc, SimAdc::REG_STATUS, 1, 100), vec![0x01]);

        let ready = 10 + SimAdc::STARTUP_CYCLES;
        assert_eq!(read(&mut adc, SimAdc::REG_STATUS, 1, ready), vec![0x81]);
        let data = read(&mut adc, SimAdc::REG_DATA, 2, ready);
        let code = u16::from_be_bytes([data[0], data[1]]);
        assert_eq!(code, (1.0f64 / 3.3 * 4095.0).round() as u16);
        assert_eq!(read(&mut adc, SimAdc::REG_STATUS, 1, ready + 1), vec![0x01]);
    }
}
