//! IPMI BMC sensor source via `ipmitool sensor list` subprocess.
//!
//! Provides access to BMC-managed sensors including DIMM temperatures,
//! per-CCD voltages, PSU telemetry, and labeled fan RPMs. Works on
//! server and workstation boards with a BMC (requires /dev/ipmi0 + root).

use std::process::Command;
use std::time::{Duration, Instant};

use crate::model::sensor::{SensorCategory, SensorId, SensorReading, SensorUnit};

/// Minimum interval between `ipmitool sensor list` invocations.
const MIN_POLL_INTERVAL: Duration = Duration::from_secs(5);

pub struct IpmiSource {
    available: bool,
    cache: Vec<(SensorId, SensorReading)>,
    last_poll: Option<Instant>,
}

impl IpmiSource {
    pub fn discover() -> Self {
        let dev_exists = std::path::Path::new("/dev/ipmi0").exists();
        if !dev_exists {
            log::debug!("IPMI: /dev/ipmi0 not found");
            return Self {
                available: false,
                cache: Vec::new(),
                last_poll: None,
            };
        }

        let has_tool = Command::new("ipmitool")
            .arg("--help")
            .stdout(std::process::Stdio::null())
            .stderr(std::process::Stdio::null())
            .status()
            .is_ok();

        if !has_tool {
            log::debug!("IPMI: ipmitool not found in PATH");
            return Self {
                available: false,
                cache: Vec::new(),
                last_poll: None,
            };
        }

        log::info!("IPMI sensor source available (/dev/ipmi0 + ipmitool)");
        Self {
            available: true,
            cache: Vec::new(),
            last_poll: None,
        }
    }

    /// Poll sensors. Returns cached results if called within MIN_POLL_INTERVAL.
    pub fn poll(&mut self) -> Vec<(SensorId, SensorReading)> {
        if !self.available {
            return Vec::new();
        }

        if let Some(last) = self.last_poll {
            if last.elapsed() < MIN_POLL_INTERVAL {
                return self.cache.clone();
            }
        }

        match run_ipmitool() {
            Some(output) => {
                self.cache = parse_output(&output);
                self.last_poll = Some(Instant::now());
                self.cache.clone()
            }
            None => {
                log::warn!("IPMI: ipmitool sensor list failed");
                Vec::new()
            }
        }
    }

    pub fn is_available(&self) -> bool {
        self.available
    }
}

fn run_ipmitool() -> Option<String> {
    let output = Command::new("ipmitool")
        .args(["sensor", "list"])
        .stdout(std::process::Stdio::piped())
        .stderr(std::process::Stdio::null())
        .output()
        .ok()?;

    if !output.status.success() {
        return None;
    }

    String::from_utf8(output.stdout).ok()
}

fn parse_output(output: &str) -> Vec<(SensorId, SensorReading)> {
    output.lines().filter_map(parse_line).collect()
}

fn parse_line(line: &str) -> Option<(SensorId, SensorReading)> {
    let fields: Vec<&str> = line.split('|').collect();
    if fields.len() < 4 {
        return None;
    }

    let name = fields[0].trim();
    let value_str = fields[1].trim();
    let unit_str = fields[2].trim();
    let status = fields[3].trim();

    if value_str == "na" || status == "na" {
        return None;
    }

    let value: f64 = value_str.parse().ok()?;

    let (unit, category) = match unit_str {
        "degrees C" => (SensorUnit::Celsius, SensorCategory::Temperature),
        "Volts" => (SensorUnit::Volts, SensorCategory::Voltage),
        "RPM" => (SensorUnit::Rpm, SensorCategory::Fan),
        "Watts" => (SensorUnit::Watts, SensorCategory::Power),
        "Amps" => (SensorUnit::Amps, SensorCategory::Current),
        _ => return None,
    };

    let sensor_name = name
        .trim_start_matches(['+', '-'])
        .to_lowercase()
        .replace(' ', "_");

    let id = SensorId {
        source: "ipmi".into(),
        chip: "bmc".into(),
        sensor: sensor_name,
    };

    Some((
        id,
        SensorReading::new(name.to_string(), value, unit, category),
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parse_voltage() {
        let line = "+VCORE_0         | 0.896      | Volts      | ok    | na        | na        | na        | 1.648     | 1.728     | 1.800     ";
        let (id, r) = parse_line(line).unwrap();
        assert_eq!(id.sensor, "vcore_0");
        assert!((r.current - 0.896).abs() < 0.001);
        assert_eq!(r.unit, SensorUnit::Volts);
    }

    #[test]
    fn parse_temperature() {
        let line = "DIMMA1_Temp      | 45.000     | degrees C  | ok    | na        | na        | na        | 81.000    | 83.000    | 85.000    ";
        let (id, r) = parse_line(line).unwrap();
        assert_eq!(id.sensor, "dimma1_temp");
        assert!((r.current - 45.0).abs() < 0.001);
        assert_eq!(r.unit, SensorUnit::Celsius);
    }

    #[test]
    fn parse_fan() {
        let line = "CPU_FAN          | 840.000    | RPM        | ok    | 0.000     | 360.000   | 360.000   | na        | na        | na        ";
        let (id, r) = parse_line(line).unwrap();
        assert_eq!(id.sensor, "cpu_fan");
        assert!((r.current - 840.0).abs() < 0.001);
        assert_eq!(r.unit, SensorUnit::Rpm);
    }

    #[test]
    fn parse_power() {
        let line = "PSU1 Input       | 142.000    | Watts      | ok    | na        | na        | na        | na        | na        | na        ";
        let (id, _) = parse_line(line).unwrap();
        assert_eq!(id.sensor, "psu1_input");
    }

    #[test]
    fn skip_na_value() {
        let line = "PSU1 Power Out   | na         | Watts      | na    | na        | na        | na        | 1700.000  | 2000.000  | 2300.000  ";
        assert!(parse_line(line).is_none());
    }

    #[test]
    fn skip_discrete() {
        let line = "Chassis Intru    | 0x0        | discrete   | 0x0000| na        | na        | na        | na        | na        | na        ";
        assert!(parse_line(line).is_none());
    }

    #[test]
    fn skip_malformed() {
        assert!(parse_line("").is_none());
        assert!(parse_line("no pipes").is_none());
    }

    #[test]
    fn parse_full_output() {
        let output = "\
+VCORE_0         | 0.896      | Volts      | ok    | na        | na        | na        | 1.648     | 1.728     | 1.800
DIMMA1_Temp      | 45.000     | degrees C  | ok    | na        | na        | na        | 81.000    | 83.000    | 85.000
CPU_FAN          | 840.000    | RPM        | ok    | 0.000     | 360.000   | 360.000   | na        | na        | na
PSU1 Power Out   | na         | Watts      | na    | na        | na        | na        | 1700.000  | 2000.000  | 2300.000
";
        let readings = parse_output(output);
        assert_eq!(readings.len(), 3);
        assert_eq!(readings[0].0.sensor, "vcore_0");
        assert_eq!(readings[1].0.sensor, "dimma1_temp");
        assert_eq!(readings[2].0.sensor, "cpu_fan");
    }

    #[test]
    fn normalize_strips_plus() {
        let line = "+3.3V            | 3.280      | Volts      | ok    | 2.640     | 2.800     | 2.976     | 3.632     | 3.792     | 3.968     ";
        let (id, _) = parse_line(line).unwrap();
        assert_eq!(id.sensor, "3.3v");
    }
}
