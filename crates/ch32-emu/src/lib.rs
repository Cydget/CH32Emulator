//! Instruction-set emulator for WCH CH32V203 firmware.

pub mod cpu;
pub mod elf;
pub mod machine;

pub use elf::{load_elf, LoadedElf, StaticSymbol};
pub use machine::{ExternalDrive, Machine, PinLevel, PinMode, PinView};
