//! ELF loader and static-variable discovery.

use object::{Object, ObjectSection, ObjectSegment, ObjectSymbol, SymbolKind};
use serde::Serialize;

#[derive(Clone, Debug)]
pub struct LoadedSegment {
    pub address: u32,
    pub data: Vec<u8>,
}

#[derive(Clone, Debug, Serialize)]
pub struct StaticSymbol {
    pub name: String,
    pub demangled: String,
    pub address: u32,
    pub size: u32,
}

#[derive(Clone, Debug)]
pub struct LoadedElf {
    pub entry: u32,
    pub segments: Vec<LoadedSegment>,
    pub symbols: Vec<StaticSymbol>,
}

pub fn load_elf(bytes: &[u8]) -> Result<LoadedElf, String> {
    let file = object::File::parse(bytes).map_err(|e| e.to_string())?;
    let mut segments = Vec::new();
    for segment in file.segments() {
        let address = segment.address() as u32;
        let size = segment.size() as usize;
        if size == 0 {
            continue;
        }
        let mut data = vec![0u8; size];
        if let Ok(file_data) = segment.data() {
            let n = file_data.len().min(size);
            data[..n].copy_from_slice(&file_data[..n]);
        }
        segments.push(LoadedSegment { address, data });
    }
    // Some linker scripts only describe RAM contents as sections (NOBITS).
    if segments.is_empty() {
        for section in file.sections() {
            if section.size() == 0 {
                continue;
            }
            let address = section.address() as u32;
            let size = section.size() as usize;
            let mut data = vec![0u8; size];
            if let Ok(file_data) = section.data() {
                let n = file_data.len().min(size);
                data[..n].copy_from_slice(&file_data[..n]);
            }
            segments.push(LoadedSegment { address, data });
        }
    }

    let mut symbols = Vec::new();
    for symbol in file.symbols() {
        if symbol.kind() != SymbolKind::Data && symbol.kind() != SymbolKind::Unknown {
            continue;
        }
        let Ok(name) = symbol.name() else { continue };
        if name.is_empty() || name.starts_with('.') || name.starts_with("$") {
            continue;
        }
        let size = symbol.size() as u32;
        let address = symbol.address() as u32;
        if size == 0 || size > 256 {
            continue;
        }
        if !(0x2000_0000..0x2000_5000).contains(&address) {
            continue;
        }
        let demangled = rustc_demangle::demangle(name).to_string();
        symbols.push(StaticSymbol {
            name: name.to_string(),
            demangled,
            address,
            size,
        });
    }
    symbols.sort_by(|a, b| a.demangled.cmp(&b.demangled));
    symbols.dedup_by(|a, b| a.address == b.address && a.size == b.size);

    Ok(LoadedElf {
        entry: file.entry() as u32,
        segments,
        symbols,
    })
}

pub fn read_symbol(machine: &crate::machine::Machine, symbol: &StaticSymbol) -> Option<i64> {
    let bytes = machine.read_ram(symbol.address, symbol.size.min(8))?;
    let mut buf = [0u8; 8];
    buf[..bytes.len()].copy_from_slice(&bytes);
    Some(match symbol.size {
        1 => buf[0] as i64,
        2 => u16::from_le_bytes([buf[0], buf[1]]) as i64,
        4 => u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]) as i64,
        _ => i64::from_le_bytes(buf),
    })
}
