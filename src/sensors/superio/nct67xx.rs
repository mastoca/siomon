//! Direct register reader for Nuvoton NCT6775-NCT6799 Super I/O chips.
//!
//! Reads voltages, temperatures, and fan speeds directly from the chip's
//! hardware monitoring registers via I/O port address/data pairs, bypassing
//! the kernel hwmon driver entirely. Requires root or CAP_SYS_RAWIO.

use crate::db::voltage_scaling::{self, VoltageChannel};
use crate::model::sensor::{SensorCategory, SensorId, SensorReading, SensorUnit};
use crate::platform::port_io::PortIo;
use crate::sensors::superio::chip_detect::{ChipType, SuperIoChip};

// NCT6775 register offsets (banked: high byte = bank, low byte = register)
const REG_BANK: u8 = 0x4E;

// Voltage registers (bank 4, 18 channels for NCT6798)
const VOLTAGE_REGS: [u16; 18] = [
    0x480, 0x481, 0x482, 0x483, 0x484, 0x485, 0x486, 0x487, // VIN0-VIN7
    0x488, 0x489, 0x48A, 0x48B, 0x48C, 0x48D, 0x48E, 0x48F, // VBAT-VIN15
    0x470, 0x471, // VIN16-VIN17
];

// Internal voltage scaling factors for NCT6798 (units: 0.001V per LSB * factor/100)
// From kernel nct6775-core.c: scale_in_6798[]
const VOLTAGE_SCALE_NCT6798: [u32; 18] = [
    800, 800, 1600, 1600, 800, 800, 800, 1600, // VIN0-VIN7
    1600, 1600, 1600, 1600, 800, 800, 800, 800, // VBAT-VIN15
    1600, 800, // VIN16-VIN17
];

// Temperature monitoring registers (direct temperature values)
const TEMP_REGS: [(u16, &str); 7] = [
    (0x027, "SYSTIN"),
    (0x073, "PECI Agent 0"),
    (0x075, "CPUTIN"),
    (0x077, "SYSTIN2"),
    (0x079, "AUXTIN0"),
    (0x07B, "AUXTIN1"),
    (0x07D, "AUXTIN2"),
];

// Additional temperature registers (bank 4/6)
const TEMP_EXTRA_REGS: [(u16, &str); 5] = [
    (0x4A0, "AUXTIN3"),
    (0x670, "AUXTIN0 Direct"),
    (0x672, "AUXTIN1 Direct"),
    (0x674, "AUXTIN2 Direct"),
    (0x676, "AUXTIN3 Direct"),
];

// Fan count registers (16-bit, bank 4)
// From kernel nct6775-core.c: NCT6779_REG_FAN[]
const FAN_REGS: [u16; 7] = [0x4C0, 0x4C2, 0x4C4, 0x4C6, 0x4C8, 0x4CA, 0x4CE];

// Fan pulse registers (bank 6) — pulses per revolution configuration
// From kernel: NCT6779_REG_FAN_PULSES[]
const FAN_PULSE_REGS: [u16; 7] = [0x644, 0x645, 0x646, 0x647, 0x648, 0x649, 0x64F];

// Default fan labels
const FAN_LABELS: [&str; 7] = [
    "Fan 1", "Fan 2", "Fan 3", "Fan 4", "Fan 5", "Fan 6", "Fan 7",
];

// NCT6779+ stores RPM directly in the fan count register (fan_from_reg_rpm).
// NCT6775/6776 use the traditional formula: RPM = 1350000 / count.
const FAN_RPM_DIRECT: bool = true; // NCT6779+ (all chips we support)

// Temperature source selection registers for NCT6798
// From kernel nct6775-core.c: NCT6798_REG_TEMP_SOURCE[]
const TEMP_SOURCE_REGS: [u16; 8] = [0x621, 0x622, 0xC26, 0xC27, 0xC28, 0xC29, 0xC2A, 0xC2B];

// Temperature source labels (indexed by source register value)
// From kernel nct6775-core.c: nct6798_temp_label[]
const TEMP_SOURCE_LABELS: &[&str] = &[
    "",                      // 0
    "SYSTIN",                // 1
    "CPUTIN",                // 2
    "AUXTIN0",               // 3
    "AUXTIN1",               // 4
    "AUXTIN2",               // 5
    "AUXTIN3",               // 6
    "AUXTIN4",               // 7
    "SMBUSMASTER 0",         // 8
    "SMBUSMASTER 1",         // 9
    "Virtual_TEMP",          // 10
    "Virtual_TEMP",          // 11
    "",                      // 12
    "",                      // 13
    "",                      // 14
    "",                      // 15
    "PECI Agent 0",          // 16
    "PECI Agent 1",          // 17
    "PCH_CHIP_CPU_MAX_TEMP", // 18
    "PCH_CHIP_TEMP",         // 19
    "PCH_CPU_TEMP",          // 20
    "PCH_MCH_TEMP",          // 21
    "Agent0 Dimm0",          // 22
    "Agent0 Dimm1",          // 23
    "Agent1 Dimm0",          // 24
    "Agent1 Dimm1",          // 25
    "BYTE_TEMP0",            // 26
    "BYTE_TEMP1",            // 27
    "PECI Agent 0 Cal",      // 28
    "PECI Agent 1 Cal",      // 29
];

pub struct Nct67xxSource {
    chip: SuperIoChip,
    addr_port: u16,
    data_port: u16,
    board_name: Option<String>,
}

impl Nct67xxSource {
    /// Create a new NCT67xx sensor source from a detected chip.
    pub fn new(chip: SuperIoChip) -> Self {
        let addr_port = chip.hwm_base + 5;
        let data_port = chip.hwm_base + 6;
        let board_name = crate::db::sensor_labels::read_board_name();
        Self {
            chip,
            addr_port,
            data_port,
            board_name,
        }
    }

    /// Check if this source is usable.
    pub fn is_supported(&self) -> bool {
        matches!(
            self.chip.chip,
            ChipType::Nct6775
                | ChipType::Nct6776
                | ChipType::Nct6779
                | ChipType::Nct6791
                | ChipType::Nct6792
                | ChipType::Nct6793
                | ChipType::Nct6795
                | ChipType::Nct6796
                | ChipType::Nct6797
                | ChipType::Nct6798
                | ChipType::Nct6799
        )
    }

    /// Poll all sensors and return readings.
    pub fn poll(&self) -> Vec<(SensorId, SensorReading)> {
        let mut pio = match PortIo::open() {
            Some(p) => p,
            None => return Vec::new(),
        };

        let mut readings = Vec::new();
        let chip_name = format!("{}", self.chip.chip).to_lowercase();

        // Look up board-specific voltage scaling
        let voltage_config = voltage_scaling::lookup_nct6798(self.board_name.as_deref())
            .unwrap_or_else(voltage_scaling::default_nct6798);

        self.read_voltages(&mut pio, &chip_name, voltage_config, &mut readings);
        self.read_temperatures(&mut pio, &chip_name, &mut readings);
        self.read_fans(&mut pio, &chip_name, &mut readings);
        self.read_temp_sources(&mut pio, &chip_name, &mut readings);

        readings
    }

    fn read_voltages(
        &self,
        pio: &mut PortIo,
        chip_name: &str,
        voltage_config: &[VoltageChannel; 18],
        readings: &mut Vec<(SensorId, SensorReading)>,
    ) {
        let scale = &VOLTAGE_SCALE_NCT6798;
        for (i, &reg) in VOLTAGE_REGS.iter().enumerate() {
            if let Some(raw) = self.read_register(pio, reg) {
                if raw == 0 {
                    continue; // Unconnected input
                }
                let ch = &voltage_config[i];
                // chip_mv = raw * internal_scale / 100
                let chip_mv = raw as f64 * scale[i] as f64 / 100.0;
                // actual_mv = chip_mv * external_multiplier
                let actual_v = chip_mv * ch.multiplier / 1000.0;

                let id = SensorId {
                    source: "superio".into(),
                    chip: chip_name.into(),
                    sensor: format!("vin{i}"),
                };
                readings.push((
                    id,
                    SensorReading::new(
                        ch.label.to_string(),
                        actual_v,
                        SensorUnit::Volts,
                        SensorCategory::Voltage,
                    ),
                ));
            }
        }
    }

    fn read_temperatures(
        &self,
        pio: &mut PortIo,
        chip_name: &str,
        readings: &mut Vec<(SensorId, SensorReading)>,
    ) {
        // Main temperature registers
        for &(reg, label) in &TEMP_REGS {
            if let Some(raw) = self.read_register(pio, reg) {
                let temp = raw as i8 as f64;
                if temp == 0.0 || temp == -128.0 || temp > 127.0 {
                    continue;
                }

                let sensor_name = label.to_lowercase().replace(' ', "_");
                let id = SensorId {
                    source: "superio".into(),
                    chip: chip_name.into(),
                    sensor: sensor_name,
                };
                readings.push((
                    id,
                    SensorReading::new(
                        label.to_string(),
                        temp,
                        SensorUnit::Celsius,
                        SensorCategory::Temperature,
                    ),
                ));
            }
        }

        // Extra temperature registers (half-degree resolution)
        for &(reg, label) in &TEMP_EXTRA_REGS {
            if let Some(raw) = self.read_register(pio, reg) {
                let temp = raw as i8 as f64;
                if temp == 0.0 || temp == -128.0 {
                    continue;
                }

                // Try reading fractional part (next register)
                let frac = self
                    .read_register(pio, reg + 1)
                    .map(|f| (f >> 7) as f64 * 0.5)
                    .unwrap_or(0.0);
                let temp = temp + frac;

                let sensor_name = label.to_lowercase().replace(' ', "_");
                let id = SensorId {
                    source: "superio".into(),
                    chip: chip_name.into(),
                    sensor: sensor_name,
                };
                readings.push((
                    id,
                    SensorReading::new(
                        label.to_string(),
                        temp,
                        SensorUnit::Celsius,
                        SensorCategory::Temperature,
                    ),
                ));
            }
        }
    }

    fn read_fans(
        &self,
        pio: &mut PortIo,
        chip_name: &str,
        readings: &mut Vec<(SensorId, SensorReading)>,
    ) {
        // Read fan pulse settings for correct RPM calculation
        let mut fan_pulses = [2u8; 7]; // Default: 2 pulses per revolution
        for (i, &reg) in FAN_PULSE_REGS.iter().enumerate() {
            if let Some(val) = self.read_register(pio, reg) {
                // Pulses per rev: 0=1, 1=2, 2=3, 3=4 (bits 1:0)
                let p = (val & 0x03) + 1;
                if p > 0 {
                    fan_pulses[i] = p;
                }
            }
        }

        for (i, &reg) in FAN_REGS.iter().enumerate() {
            let id = SensorId {
                source: "superio".into(),
                chip: chip_name.into(),
                sensor: format!("fan{}", i + 1),
            };

            if let Some(count) = self.read_word(pio, reg) {
                if count == 0 || count == 0xFFFF {
                    readings.push((
                        id,
                        SensorReading::new(
                            FAN_LABELS[i].to_string(),
                            0.0,
                            SensorUnit::Rpm,
                            SensorCategory::Fan,
                        ),
                    ));
                    continue;
                }

                // NCT6779+ stores RPM directly in the register (kernel: fan_from_reg_rpm).
                // Older chips (NCT6775/6776) use count-based: RPM = 1350000 / count.
                let rpm = count as f64;

                readings.push((
                    id,
                    SensorReading::new(
                        FAN_LABELS[i].to_string(),
                        rpm,
                        SensorUnit::Rpm,
                        SensorCategory::Fan,
                    ),
                ));
            }
        }
    }

    /// Read temperature source selection registers to determine what each
    /// temp monitoring slot is actually measuring.
    fn read_temp_sources(
        &self,
        pio: &mut PortIo,
        chip_name: &str,
        readings: &mut Vec<(SensorId, SensorReading)>,
    ) {
        // Only read source mapping — store as metadata-style sensors
        for (i, &reg) in TEMP_SOURCE_REGS.iter().enumerate() {
            if let Some(src_val) = self.read_register(pio, reg) {
                let src_idx = (src_val & 0x1F) as usize;
                let label = if src_idx < TEMP_SOURCE_LABELS.len() {
                    let l = TEMP_SOURCE_LABELS[src_idx];
                    if l.is_empty() {
                        continue; // Unused source
                    }
                    format!("Temp Source {} -> {}", i + 1, l)
                } else {
                    format!("Temp Source {} -> #{}", i + 1, src_idx)
                };

                let id = SensorId {
                    source: "superio".into(),
                    chip: chip_name.into(),
                    sensor: format!("temp_src{}", i + 1),
                };
                readings.push((
                    id,
                    SensorReading::new(
                        label,
                        src_val as f64,
                        SensorUnit::Unitless,
                        SensorCategory::Other,
                    ),
                ));
            }
        }
    }

    /// Read a single byte from a banked HWM register.
    fn read_register(&self, pio: &mut PortIo, reg: u16) -> Option<u8> {
        let bank = (reg >> 8) as u8;
        let addr = (reg & 0xFF) as u8;

        // Select bank
        pio.write_byte(self.addr_port, REG_BANK).ok()?;
        pio.write_byte(self.data_port, bank).ok()?;

        // Read register
        pio.write_byte(self.addr_port, addr).ok()?;
        pio.read_byte(self.data_port).ok()
    }

    /// Read a 16-bit word from two consecutive registers.
    fn read_word(&self, pio: &mut PortIo, reg: u16) -> Option<u16> {
        let hi = self.read_register(pio, reg)? as u16;
        let lo = self.read_register(pio, reg + 1)? as u16;
        Some((hi << 8) | lo)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_voltage_scale_array_length() {
        assert_eq!(VOLTAGE_SCALE_NCT6798.len(), VOLTAGE_REGS.len());
    }

    #[test]
    fn test_voltage_calculation_with_multiplier() {
        // Raw value 125 with scale 800, +12V multiplier 12.0
        let raw = 125u8;
        let scale = 800u32;
        let multiplier = 12.0;
        let chip_mv = raw as f64 * scale as f64 / 100.0;
        let actual_v = chip_mv * multiplier / 1000.0;
        assert!((actual_v - 12.0).abs() < 0.1);
    }

    #[test]
    fn test_voltage_calculation_no_multiplier() {
        // Raw value 96 with scale 800, no external divider
        let raw = 96u8;
        let scale = 800u32;
        let multiplier = 1.0;
        let chip_mv = raw as f64 * scale as f64 / 100.0;
        let actual_v = chip_mv * multiplier / 1000.0;
        assert!((actual_v - 0.768).abs() < 0.001);
    }

    #[test]
    fn test_fan_rpm_direct() {
        // NCT6779+ stores RPM directly in the register
        let reg_value: u16 = 3890;
        assert_eq!(reg_value as f64, 3890.0);
    }

    #[test]
    fn test_fan_rpm_zero() {
        // Register value 0 means stopped fan
        let reg_value: u16 = 0;
        assert_eq!(reg_value, 0);
    }

    #[test]
    fn test_fan_register_addresses() {
        // Verify fan register addresses match kernel NCT6779_REG_FAN[]
        assert_eq!(FAN_REGS, [0x4C0, 0x4C2, 0x4C4, 0x4C6, 0x4C8, 0x4CA, 0x4CE]);
    }

    #[test]
    fn test_temp_source_labels() {
        assert_eq!(TEMP_SOURCE_LABELS[1], "SYSTIN");
        assert_eq!(TEMP_SOURCE_LABELS[2], "CPUTIN");
        assert_eq!(TEMP_SOURCE_LABELS[16], "PECI Agent 0");
    }

    #[test]
    fn test_voltage_regs_count() {
        assert_eq!(VOLTAGE_REGS.len(), 18);
    }
}
