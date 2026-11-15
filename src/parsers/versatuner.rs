//! VersaTuner (VersaTune) datalog parser — Mazda MZR DISI, SkyActiv, RX-8 Renesis, etc.
//!
//! Two formats:
//!
//! - **`.vtlog`** — VersaTuner's native log. XML:
//!   `<Datalog><Version/><Name/><RecordedOn/><Platform/><VIN/><Data><PID ID="n"><Record Time="s" Value="v"/>…`
//!   Channels carry only a numeric PID; each PID has its own timestamps. Names and units come from
//!   the platform's table ([`pid_table`]): PID numbers differ between platforms. Unknown PIDs load
//!   as `PID n` with no unit, so nothing is dropped.
//! - **CSV export** (VersaTuner log view, "1, 2, 3, 4" button): `Time (s),Engine RPM,Knock retard (°),…`,
//!   values in the user's chosen units.
//!
//! **Load-bearing behaviors**
//!
//! - The `<VIN>` element is never read into [`VersaTunerMeta`]: logs are shared on forums and the VIN
//!   would otherwise surface in the UI and the MCP server.
//! - `.vtlog` channels are merged onto the union of all PID timestamps. Each row carries the last
//!   reading of every channel (sample-and-hold); rows before a channel's first reading use that
//!   first reading, so a channel never starts with a false `0`.
//! - Times are written through as recorded (they match VersaTuner's own graph and CSV time column)
//!   and are monotonic by construction (sorted union).
//! - CSV units are normalised to the spellings `UnitPreferences` converts: psi and bar → kPa,
//!   mph → km/h, `AFR gas` → `AFR`. The `°F` / `°C` / `λ` units pass through unchanged.
//! - PID → name mapping was derived by correlating a `.vtlog` against the CSV export of the same
//!   log (r = 1.000 for every confirmed entry). `confirmed: false` marks entries inferred some other
//!   way; confirm before relying on them.

use regex::Regex;
use serde::Serialize;
use std::collections::BTreeMap;
use std::error::Error;
use std::sync::LazyLock;

use super::types::{Channel, Log, Meta, Parseable, Value};

/// Known VersaTuner PIDs. Raw `.vtlog` units (not the user's display units).
pub struct PidInfo {
    pub id: u32,
    pub name: &'static str,
    pub unit: &'static str,
    pub confirmed: bool,
}

pub const PID_TABLE: &[PidInfo] = &[
    PidInfo {
        id: 1,
        name: "Absolute load",
        unit: "",
        confirmed: true,
    },
    PidInfo {
        id: 2,
        name: "Accelerator pedal position",
        unit: "%",
        confirmed: true,
    },
    PidInfo {
        id: 3,
        name: "Actual equivalence/air to fuel ratio",
        unit: "λ",
        confirmed: true,
    },
    PidInfo {
        id: 8,
        name: "Boost air temperature",
        unit: "°C",
        confirmed: true,
    },
    PidInfo {
        id: 9,
        name: "Desired equivalence/air to fuel ratio",
        unit: "λ",
        confirmed: true,
    },
    PidInfo {
        id: 13,
        name: "Engine RPM",
        unit: "rpm",
        confirmed: true,
    },
    PidInfo {
        id: 15,
        name: "Fuel rail pressure",
        unit: "kPa",
        confirmed: true,
    },
    // Constant 0 in the reference log, so it could not be correlated; it is the only CSV column left.
    PidInfo {
        id: 17,
        name: "Knock retard",
        unit: "°",
        confirmed: false,
    },
    PidInfo {
        id: 19,
        name: "Intake manifold absolute pressure",
        unit: "kPa",
        confirmed: true,
    },
    PidInfo {
        id: 20,
        name: "Mass airflow",
        unit: "g/s",
        confirmed: true,
    },
    PidInfo {
        id: 21,
        name: "Mass airflow sensor voltage",
        unit: "V",
        confirmed: true,
    },
    PidInfo {
        id: 22,
        name: "Relative throttle position",
        unit: "%",
        confirmed: true,
    },
    PidInfo {
        id: 25,
        name: "Ignition timing advance",
        unit: "°",
        confirmed: true,
    },
    PidInfo {
        id: 27,
        name: "Wastegate control solenoid duty cycle",
        unit: "%",
        confirmed: true,
    },
    PidInfo {
        id: 40,
        name: "Injector duty cycle",
        unit: "%",
        confirmed: true,
    },
];

/// Gen2 (Series II) RX-8, platform `G2_MAZDA_RX8`. PID numbers are per platform: on the RX-8, 19 is
/// trailing timing and 40 an oil pump pressure, not MAP and injector duty as in [`PID_TABLE`].
/// Derived from a 48-channel RX-8 log and its CSV export. Throttle PIDs (4, 11, 22, 27) were told
/// apart by exact value fit, as they all correlate at r ≈ 1. The PIDs that stayed constant (17, 30,
/// 42, 47, 48, 49, 53 at 0) can't be correlated; 17 follows the Mazdaspeed3 guess.
pub const PID_TABLE_RX8_G2: &[PidInfo] = &[
    PidInfo {
        id: 1,
        name: "Absolute load",
        unit: "",
        confirmed: true,
    },
    PidInfo {
        id: 2,
        name: "Accelerator pedal position",
        unit: "%",
        confirmed: true,
    },
    PidInfo {
        id: 3,
        name: "Actual equivalence/air to fuel ratio",
        unit: "λ",
        confirmed: true,
    },
    PidInfo {
        id: 4,
        name: "Actual throttle angle",
        unit: "°",
        confirmed: true,
    },
    PidInfo {
        id: 5,
        name: "Desired metering oil pump flow (Center) BETA",
        unit: "",
        confirmed: true,
    },
    PidInfo {
        id: 6,
        name: "Barometric pressure",
        unit: "kPa",
        confirmed: true,
    },
    PidInfo {
        id: 7,
        name: "Control module voltage",
        unit: "V",
        confirmed: true,
    },
    PidInfo {
        id: 8,
        name: "Desired metering oil pump flow (Side) BETA",
        unit: "",
        confirmed: true,
    },
    PidInfo {
        id: 9,
        name: "Desired equivalence/air to fuel ratio",
        unit: "λ",
        confirmed: true,
    },
    PidInfo {
        id: 10,
        name: "Desired engine RPM",
        unit: "rpm",
        confirmed: true,
    },
    PidInfo {
        id: 11,
        name: "Desired throttle position",
        unit: "%",
        confirmed: true,
    },
    PidInfo {
        id: 12,
        name: "Engine coolant temperature",
        unit: "°C",
        confirmed: true,
    },
    PidInfo {
        id: 13,
        name: "Engine RPM",
        unit: "rpm",
        confirmed: true,
    },
    PidInfo {
        id: 14,
        name: "Fuel injection pulse width",
        unit: "ms",
        confirmed: true,
    },
    PidInfo {
        id: 16,
        name: "Intake air temperature",
        unit: "°C",
        confirmed: true,
    },
    PidInfo {
        id: 17,
        name: "Knock retard",
        unit: "°",
        confirmed: false,
    },
    PidInfo {
        id: 18,
        name: "Long term fuel trim",
        unit: "%",
        confirmed: true,
    },
    PidInfo {
        id: 19,
        name: "Ignition timing advance - trailing",
        unit: "°",
        confirmed: true,
    },
    PidInfo {
        id: 20,
        name: "Mass air flow",
        unit: "g/s",
        confirmed: true,
    },
    PidInfo {
        id: 21,
        name: "Mass airflow sensor voltage",
        unit: "V",
        confirmed: true,
    },
    PidInfo {
        id: 22,
        name: "Relative throttle position",
        unit: "%",
        confirmed: true,
    },
    PidInfo {
        id: 23,
        name: "Short term fuel trim (primary sensor)",
        unit: "%",
        confirmed: true,
    },
    PidInfo {
        id: 24,
        name: "Short term fuel trim (secondary sensor)",
        unit: "%",
        confirmed: false,
    },
    PidInfo {
        id: 25,
        name: "Ignition timing advance - leading",
        unit: "°",
        confirmed: true,
    },
    PidInfo {
        id: 26,
        name: "Vehicle speed",
        unit: "km/h",
        confirmed: true,
    },
    PidInfo {
        id: 27,
        name: "Desired throttle angle",
        unit: "°",
        confirmed: true,
    },
    PidInfo {
        id: 28,
        name: "Idle air control",
        unit: "%",
        confirmed: true,
    },
    // Constant 240 in the reference log, the same as its CSV column (the only one with that value).
    PidInfo {
        id: 29,
        name: "Fuel injection pulse width (Percent) BETA",
        unit: "%",
        confirmed: false,
    },
    PidInfo {
        id: 31,
        name: "Clutch pedal position",
        unit: "",
        confirmed: true,
    },
    PidInfo {
        id: 32,
        name: "Neutral gear status",
        unit: "",
        confirmed: true,
    },
    PidInfo {
        id: 33,
        name: "Auxiliary port valve 1 status",
        unit: "",
        confirmed: true,
    },
    PidInfo {
        id: 34,
        name: "Fan 1 and 2 status",
        unit: "",
        confirmed: true,
    },
    PidInfo {
        id: 36,
        name: "Fuel system status",
        unit: "",
        confirmed: true,
    },
    PidInfo {
        id: 37,
        name: "Secondary shutter valve status BETA",
        unit: "",
        confirmed: true,
    },
    PidInfo {
        id: 38,
        name: "Metering oil pump mode on/off (Center) BETA",
        unit: "",
        confirmed: true,
    },
    PidInfo {
        id: 39,
        name: "Metering oil pump mode on/off (Side) BETA",
        unit: "",
        confirmed: false,
    },
    PidInfo {
        id: 40,
        name: "Metering oil pump desired pressure BETA",
        unit: "kPa",
        confirmed: true,
    },
    PidInfo {
        id: 41,
        name: "Metering oil pump actual pressure BETA",
        unit: "kPa",
        confirmed: true,
    },
    PidInfo {
        id: 44,
        name: "Port air bypass control solenoid valve mode on/off BETA",
        unit: "",
        confirmed: true,
    },
    // Constant 121, the same as its CSV column (the only one with that value).
    PidInfo {
        id: 46,
        name: "Number of oil control valves open B BETA",
        unit: "",
        confirmed: false,
    },
    PidInfo {
        id: 52,
        name: "Auxiliary port valve 2 status",
        unit: "",
        confirmed: true,
    },
    PidInfo {
        id: 54,
        name: "Catalyst temperature",
        unit: "°C",
        confirmed: true,
    },
];

/// The PID table for a `.vtlog` `<Platform>`. Other platforms use [`PID_TABLE`] (matched on the
/// Mazdaspeed3), which is also the fallback for platforms not mapped yet.
pub fn pid_table(platform: &str) -> &'static [PidInfo] {
    match platform {
        "G2_MAZDA_RX8" => PID_TABLE_RX8_G2,
        _ => PID_TABLE,
    }
}

pub fn pid_info(platform: &str, id: u32) -> Option<&'static PidInfo> {
    pid_table(platform).iter().find(|p| p.id == id)
}

#[derive(Clone, Debug, Serialize)]
pub struct VersaTunerChannel {
    pub name: String,
    pub unit: String,
    /// The `.vtlog` PID, or `None` for a CSV column.
    pub pid: Option<u32>,
}

impl VersaTunerChannel {
    pub fn unit(&self) -> &str {
        &self.unit
    }
}

#[derive(Clone, Debug, Default, Serialize)]
pub struct VersaTunerMeta {
    /// "vtlog" or "csv"
    pub format: String,
    pub name: String,
    pub platform: String,
    pub recorded_on: String,
    /// PIDs present in the file but missing from [`PID_TABLE`] (shown as `PID n`).
    pub unmapped_pids: Vec<u32>,
}

pub struct VersaTuner;

static PID_RE: LazyLock<Regex> = LazyLock::new(|| Regex::new(r#"<PID\s+ID="(\d+)"\s*>"#).unwrap());
static RECORD_RE: LazyLock<Regex> =
    LazyLock::new(|| Regex::new(r#"<Record\s+Time="([^"]+)"\s+Value="([^"]+)"\s*/>"#).unwrap());

/// CSV column phrases only VersaTuner writes (Mazda/VersaTuner parameter naming).
const CSV_FINGERPRINTS: &[&str] = &[
    "equivalence/air to fuel ratio",
    "Absolute load",
    "Wastegate control solenoid duty cycle",
    "Short term fuel trim (primary sensor)",
    "Boost air temperature",
];

impl VersaTuner {
    /// True for a native `.vtlog` (XML with a `<Datalog>` root and `<PID ID=…>` blocks).
    pub fn detect_vtlog(contents: &str) -> bool {
        let head: String = contents
            .trim_start_matches('\u{FEFF}')
            .chars()
            .take(4096)
            .collect();
        head.trim_start().starts_with("<?xml")
            && head.contains("<Datalog>")
            && head.contains("<PID ID=")
    }

    /// True for a VersaTuner CSV export: `Time (s)` first plus a VersaTuner-specific column name.
    pub fn detect_csv(contents: &str) -> bool {
        let first = &contents[Self::csv_start(contents)..];
        let first = first.lines().next().unwrap_or("");
        first.starts_with("Time (s),") && CSV_FINGERPRINTS.iter().any(|f| first.contains(f))
    }

    /// Byte offset of the `Time (s),` header. Skips a BOM, including one an editor has
    /// re-encoded into a few junk characters (`ï»¿`, `o;?`) when re-saving the export.
    fn csv_start(contents: &str) -> usize {
        // Up to 8 junk characters plus the 9-character "Time (s)," marker, on char boundaries.
        let end = contents
            .char_indices()
            .nth(17)
            .map_or(contents.len(), |(i, _)| i);
        contents[..end].find("Time (s),").unwrap_or(0)
    }

    pub fn detect(contents: &str) -> bool {
        Self::detect_vtlog(contents) || Self::detect_csv(contents)
    }

    fn tag<'a>(xml: &'a str, name: &str) -> &'a str {
        let open = format!("<{name}>");
        let close = format!("</{name}>");
        xml.find(&open)
            .and_then(|s| {
                let from = s + open.len();
                xml[from..].find(&close).map(|e| xml[from..from + e].trim())
            })
            .unwrap_or("")
    }

    fn parse_vtlog(contents: &str) -> Result<Log, Box<dyn Error>> {
        let xml = contents.trim_start_matches('\u{FEFF}');
        let header_end = xml.find("<Data>").unwrap_or(xml.len());
        let header = &xml[..header_end];
        let mut meta = VersaTunerMeta {
            format: "vtlog".into(),
            name: Self::tag(header, "Name").to_string(),
            platform: Self::tag(header, "Platform").to_string(),
            recorded_on: Self::tag(header, "RecordedOn").to_string(),
            unmapped_pids: Vec::new(),
        };

        // PID blocks in file order; each block's records run until the next <PID> tag.
        let starts: Vec<(usize, u32)> = PID_RE
            .captures_iter(xml)
            .map(|c| (c.get(0).unwrap().end(), c[1].parse::<u32>().unwrap_or(0)))
            .collect();
        if starts.is_empty() {
            return Err("No <PID> channels found in .vtlog".into());
        }
        let mut series: Vec<(u32, Vec<(f64, f64)>)> = Vec::with_capacity(starts.len());
        for (i, &(from, id)) in starts.iter().enumerate() {
            let to = starts.get(i + 1).map(|s| s.0).unwrap_or(xml.len());
            let mut s: Vec<(f64, f64)> = RECORD_RE
                .captures_iter(&xml[from..to])
                .filter_map(|c| Some((c[1].parse::<f64>().ok()?, c[2].parse::<f64>().ok()?)))
                .filter(|(t, v)| t.is_finite() && v.is_finite())
                .collect();
            if s.is_empty() {
                continue;
            }
            s.sort_by(|a, b| a.0.total_cmp(&b.0));
            series.push((id, s));
        }
        if series.is_empty() {
            return Err("The .vtlog contains no readings".into());
        }

        // Union time axis.
        let mut times: Vec<f64> = series
            .iter()
            .flat_map(|(_, s)| s.iter().map(|p| p.0))
            .collect();
        times.sort_by(|a, b| a.total_cmp(b));
        times.dedup_by(|a, b| (*a - *b).abs() < 1e-9);

        let mut channels = Vec::with_capacity(series.len());
        let mut columns: Vec<Vec<f64>> = Vec::with_capacity(series.len());
        let mut names_seen: BTreeMap<String, u32> = BTreeMap::new();
        for (id, s) in &series {
            let (mut name, unit) = match pid_info(&meta.platform, *id) {
                Some(p) => (p.name.to_string(), p.unit.to_string()),
                None => {
                    meta.unmapped_pids.push(*id);
                    (format!("PID {id}"), String::new())
                }
            };
            if names_seen.insert(name.clone(), *id).is_some() {
                name = format!("{name} (PID {id})");
            }
            channels.push(Channel::VersaTuner(VersaTunerChannel {
                name,
                unit,
                pid: Some(*id),
            }));

            // Sample-and-hold onto the union axis; backfill with the first reading.
            let mut col = Vec::with_capacity(times.len());
            let mut k = 0usize;
            let mut current = s[0].1;
            for &t in &times {
                while k < s.len() && s[k].0 <= t + 1e-9 {
                    current = s[k].1;
                    k += 1;
                }
                col.push(current);
            }
            columns.push(col);
        }
        meta.unmapped_pids.sort_unstable();

        let data = (0..times.len())
            .map(|r| columns.iter().map(|c| Value::Float(c[r])).collect())
            .collect();
        Ok(Log {
            meta: Meta::VersaTuner(meta),
            channels,
            times,
            data,
        })
    }

    /// Split `Name (unit)` on the last parenthesis group and normalise the unit.
    /// Returns (name, unit, multiplier, offset) so values convert to the normalised unit.
    fn csv_column(header: &str) -> (String, String, f64, f64) {
        let h = header.trim();
        let (name, raw_unit) = match (h.rfind('('), h.ends_with(')')) {
            (Some(p), true) => (
                h[..p].trim().to_string(),
                h[p + 1..h.len() - 1].trim().to_string(),
            ),
            _ => (h.to_string(), String::new()),
        };
        // Exports re-saved by some editors mangle the degree sign to "B0" (UTF-8 C2 B0 read as Latin-1).
        let raw_unit = raw_unit.replace("B0", "°").replace("Â°", "°");
        match raw_unit.as_str() {
            "psi" => (name, "kPa".into(), 6.894_757, 0.0),
            "bar" => (name, "kPa".into(), 100.0, 0.0),
            "mph" => (name, "km/h".into(), 1.609_344, 0.0),
            "AFR gas" => (name, "AFR".into(), 1.0, 0.0),
            "λ" | "lambda" => (name, "λ".into(), 1.0, 0.0),
            // VersaTuner writes "Engine RPM" with no unit; give it one so specs and formulas match.
            "" if name.ends_with("RPM") => (name, "rpm".into(), 1.0, 0.0),
            _ => (name, raw_unit, 1.0, 0.0),
        }
    }

    fn parse_csv(contents: &str) -> Result<Log, Box<dyn Error>> {
        let text = &contents[Self::csv_start(contents)..];
        let mut lines = text.lines();
        let header = lines.next().ok_or("Empty VersaTuner CSV")?;
        let cols: Vec<(String, String, f64, f64)> =
            header.split(',').skip(1).map(Self::csv_column).collect();
        if cols.is_empty() {
            return Err("VersaTuner CSV has no channel columns".into());
        }
        let channels = cols
            .iter()
            .map(|(n, u, _, _)| {
                Channel::VersaTuner(VersaTunerChannel {
                    name: n.clone(),
                    unit: u.clone(),
                    pid: None,
                })
            })
            .collect();

        let mut times = Vec::new();
        let mut data = Vec::new();
        let mut last = vec![f64::NAN; cols.len()];
        let mut last_t = f64::NEG_INFINITY;
        for line in lines {
            let mut fields = line.split(',');
            let Some(Ok(t)) = fields.next().map(|f| f.trim().parse::<f64>()) else {
                continue;
            };
            if t <= last_t {
                continue; // keep times strictly increasing (time-shift lookups binary-search them)
            }
            let row: Vec<Value> = cols
                .iter()
                .enumerate()
                .map(|(i, (_, _, mul, off))| {
                    if let Some(Ok(v)) = fields.next().map(|f| f.trim().parse::<f64>()) {
                        last[i] = v * mul + off;
                    }
                    Value::Float(if last[i].is_nan() { 0.0 } else { last[i] })
                })
                .collect();
            last_t = t;
            times.push(t);
            data.push(row);
        }
        if times.is_empty() {
            return Err("VersaTuner CSV has a header but no data rows".into());
        }
        let meta = VersaTunerMeta {
            format: "csv".into(),
            ..Default::default()
        };
        Ok(Log {
            meta: Meta::VersaTuner(meta),
            channels,
            times,
            data,
        })
    }
}

impl Parseable for VersaTuner {
    fn parse(&self, data: &str) -> Result<Log, Box<dyn Error>> {
        if Self::detect_vtlog(data) {
            Self::parse_vtlog(data)
        } else {
            Self::parse_csv(data)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const VTLOG: &str = "\u{FEFF}<?xml version=\"1.0\" encoding=\"utf-8\"?>\r\n<Datalog>\r\n  <Version>1</Version>\r\n  <Name>Test</Name>\r\n  <Description />\r\n  <RecordedOn>2026-01-02T03:04:05</RecordedOn>\r\n  <Platform>RX8_SERIES1</Platform>\r\n  <VIN>JM1FAKEVIN0000000</VIN>\r\n  <Data>\r\n    <PID ID=\"13\">\r\n      <Record Time=\"10.0\" Value=\"800\" />\r\n      <Record Time=\"10.5\" Value=\"900\" />\r\n    </PID>\r\n    <PID ID=\"3\">\r\n      <Record Time=\"10.2\" Value=\"1.0\" />\r\n      <Record Time=\"10.7\" Value=\"0.85\" />\r\n    </PID>\r\n    <PID ID=\"999\">\r\n      <Record Time=\"10.1\" Value=\"5\" />\r\n    </PID>\r\n  </Data>\r\n</Datalog>\r\n";

    #[test]
    fn detects_vtlog_and_not_other_xml() {
        assert!(VersaTuner::detect_vtlog(VTLOG));
        assert!(VersaTuner::detect(VTLOG));
        assert!(!VersaTuner::detect_vtlog("<?xml version=\"1.0\"?><root/>"));
    }

    #[test]
    fn vtlog_merges_pids_onto_union_axis() {
        let log = VersaTuner.parse(VTLOG).unwrap();
        assert_eq!(log.times, vec![10.0, 10.1, 10.2, 10.5, 10.7]);
        assert_eq!(log.channels.len(), 3);
        assert_eq!(log.channels[0].name(), "Engine RPM");
        assert_eq!(
            log.channels[1].name(),
            "Actual equivalence/air to fuel ratio"
        );
        assert_eq!(log.channels[1].unit(), "λ");
        assert_eq!(log.channels[2].name(), "PID 999");
        let rpm = log.get_channel_data(0);
        assert_eq!(rpm, vec![800.0, 800.0, 800.0, 900.0, 900.0]);
        // Lambda before its first reading is backfilled, never 0.
        let lambda = log.get_channel_data(1);
        assert_eq!(lambda, vec![1.0, 1.0, 1.0, 1.0, 0.85]);
    }

    #[test]
    fn vtlog_meta_has_platform_but_never_the_vin() {
        let log = VersaTuner.parse(VTLOG).unwrap();
        let Meta::VersaTuner(m) = &log.meta else {
            panic!("wrong meta")
        };
        assert_eq!(m.platform, "RX8_SERIES1");
        assert_eq!(m.unmapped_pids, vec![999]);
        assert!(!serde_json::to_string(m).unwrap().contains("FAKEVIN"));
    }

    #[test]
    fn pid_names_depend_on_the_platform() {
        let rx8 = VTLOG
            .replace("RX8_SERIES1", "G2_MAZDA_RX8")
            .replace("PID ID=\"3\"", "PID ID=\"19\"");
        let names = |s: &str| -> Vec<String> {
            VersaTuner
                .parse(s)
                .unwrap()
                .channels
                .iter()
                .map(|c| c.name())
                .collect()
        };
        assert!(names(&rx8).contains(&"Ignition timing advance - trailing".to_string()));
        let other = VTLOG.replace("PID ID=\"3\"", "PID ID=\"19\"");
        assert!(names(&other).contains(&"Intake manifold absolute pressure".to_string()));
        assert_eq!(
            pid_info("G2_MAZDA_RX8", 40).unwrap().name,
            "Metering oil pump desired pressure BETA"
        );
        assert_eq!(
            pid_info("G2_MAZDASPEED3", 40).unwrap().name,
            "Injector duty cycle"
        );
    }

    const CSV: &str = "\u{FEFF}Time (s),Absolute load,Boost pressure (psi),Actual equivalence/air to fuel ratio (λ),Vehicle speed (mph),Engine coolant temperature (°F)\n72.354,0.765,1.0,0.974,10,156\n72.583,0.770,,0.970,11,157\n72.583,9,9,9,9,9\n";

    #[test]
    fn detects_csv_export_only_with_fingerprint() {
        assert!(VersaTuner::detect_csv(CSV));
        assert!(!VersaTuner::detect_csv("Time (s),RPM,MAP\n0,1,2\n"));
    }

    #[test]
    fn csv_normalises_units_and_holds_blanks() {
        let log = VersaTuner.parse(CSV).unwrap();
        assert_eq!(log.times, vec![72.354, 72.583]); // duplicate time dropped
        assert_eq!(log.channels[1].name(), "Boost pressure");
        assert_eq!(log.channels[1].unit(), "kPa");
        let boost = log.get_channel_data(1);
        assert!((boost[0] - 6.894757).abs() < 1e-6);
        assert!((boost[1] - 6.894757).abs() < 1e-6); // blank field holds last value
        assert_eq!(log.channels[2].unit(), "λ");
        assert_eq!(log.channels[3].unit(), "km/h");
        assert_eq!(log.channels[4].unit(), "°F");
    }

    #[test]
    fn csv_detects_export_with_mangled_bom() {
        // An editor re-saved the export and turned the BOM into "o;?" (seen on the forum).
        let mangled = "o;?Time (s),Absolute load,Engine RPM\n1.0,0.5,800\n";
        assert!(VersaTuner::detect_csv(mangled));
        let log = VersaTuner.parse(mangled).unwrap();
        assert_eq!(log.channels[0].name(), "Absolute load");
        assert_eq!(log.channels[1].unit(), "rpm");
        assert!(!VersaTuner::detect_csv("ééééééééééééééééééé"));
    }

    #[test]
    fn csv_repairs_mangled_degree_sign() {
        let (name, unit, _, _) = VersaTuner::csv_column("Boost air temperature (B0C)");
        assert_eq!(name, "Boost air temperature");
        assert_eq!(unit, "°C");
    }
}
