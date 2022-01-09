use chrono::NaiveDate;
use geomorph::{coord, utm};
use regex::Regex;
use std::str::FromStr;

/// Parses a string containing GPX data.
///
/// Retrieves `lat`, `lon`, and `time` for each point. The date part of `time` is
/// stripped and the timestamp is converted to milliseconds.
pub fn parse_gpx(gpx: String) -> Vec<Vec<[f64; 3]>> {
    let mut trjs: Vec<Vec<[f64; 3]>> = Vec::new();
    let mut trj: Vec<[f64; 3]> = Vec::new();
    let re = Regex::new(r"lat=\W(\d+[[:punct:]]\d+)\W\slon=\W(\d+[[:punct:]]\d+)\W{2}[[:space:]]*<ele>\d+[[:punct:]]\d+</ele>[[:space:]]*<time>(\d{4})-(\d{2})-(\d{2})T(\d{2}):(\d{2}):(\d{2})[[:punct:]](\d+)").unwrap();
    let (mut yr_, mut mn_, mut da_): (i32, u32, u32) = (0, 0, 0);
    for cap in re.captures_iter(&gpx) {
        let lat: f64 = cap[1].parse::<f64>().unwrap();
        let lon: f64 = cap[2].parse::<f64>().unwrap();
        let yr: i32 = cap[3].parse::<i32>().unwrap();
        let mn: u32 = cap[4].parse::<u32>().unwrap();
        let da: u32 = cap[5].parse::<u32>().unwrap();
        let h: u32 = cap[6].parse::<u32>().unwrap();
        let m: u32 = cap[7].parse::<u32>().unwrap();
        let s: u32 = cap[8].parse::<u32>().unwrap();
        let ms: u32 = cap[9].parse::<u32>().unwrap();
        let time = NaiveDate::from_ymd(1970, 1, 1)
            .and_hms_milli(h, m, s, ms)
            .timestamp_millis() as f64;
        if yr != yr_ || mn != mn_ || da != da_ {
            yr_ = yr;
            mn_ = mn;
            da_ = da;
            trjs.push(trj);
            trj = vec![];
        }
        let c = crate::Coord::from_gps(&[lon, lat, time]);
        trj.push([c.x, c.y, c.t]); // Note we have x=lon, y=lat, z=time(ms)
    }
    trjs
}

/// The fields can be set using the [config](parse_config) file.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// Number of points used in the [CH-filter](crate::utility::ch_filter)
    pub window_size: usize,
    /// If the object moves slower than this it is considered to be stopped. *Currently unused*.
    pub minimum_velocity: f64,
    /// If the object moves faster than `minimum_velocity` plus this then it is considered to be moving again. *Currently unused*.
    pub epsilon_velocity: f64,
    /// Maximal number of milliseconds between two measurements before the stream is cut into two.
    pub connection_timeout: f64,
    /// Maximal diagonal size of a geofenced region. If movement occurs within a region of this size it is eligible to be considered a stop.
    pub stop_diagonal_meters: f64,
    /// The least amount of time that movement must occur within a geofenced region before it is considered a stop.
    pub stop_duration_minutes: f64,
    /// When searching for matching stops allow them to be this many minutes apart.
    pub relax_bbox_minutes: f64,
    /// When searching for matching stops allow them to be this many meters apart.
    pub relax_bbox_meters: f64,
    /// If two trajectories belonging to the same edge have a Hausdorff distance of more than this, they will not be merged.
    pub max_hausdorff_meters: f64,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            window_size: 5,
            minimum_velocity: 2.5,
            epsilon_velocity: 0.5,
            stop_duration_minutes: 15.0,
            connection_timeout: 120000.0,
            stop_diagonal_meters: 50.0,
            relax_bbox_meters: 50.,
            relax_bbox_minutes: 30.,
            max_hausdorff_meters: 100.,
        }
    }
}

enum ConfigKeys {
    WindowSize,
    MinimumVelocity,
    EpsilonVelocity,
    StopDurationMinutes,
    ConnectionTimeout,
    StopDiagonalMeters,
    RelaxBboxMinutes,
    RelaxBboxMeters,
    MaxHausdorffMeters,
}

impl FromStr for ConfigKeys {
    type Err = ();

    fn from_str(s: &str) -> Result<ConfigKeys, ()> {
        match s {
            "window_size" => Ok(ConfigKeys::WindowSize),
            "minimum_velocity" => Ok(ConfigKeys::MinimumVelocity),
            "epsilon_velocity" => Ok(ConfigKeys::EpsilonVelocity),
            "stop_duration_minutes" => Ok(ConfigKeys::StopDurationMinutes),
            "connection_timeout" => Ok(ConfigKeys::ConnectionTimeout),
            "stop_diagonal_meters" => Ok(ConfigKeys::StopDiagonalMeters),
            "relax_bbox_minutes" => Ok(ConfigKeys::RelaxBboxMinutes),
            "relax_bbox_meters" => Ok(ConfigKeys::RelaxBboxMeters),
            "max_hausdorff_meters" => Ok(ConfigKeys::MaxHausdorffMeters),
            _ => Err(()),
        }
    }
}

/// Parses config.cfg in the root folder into a [Config](Config) struct
pub fn parse_config(config: String) -> Config {
    let mut default_config: Config = Config::default();

    fn handle_line(line: &str, config: &mut Config) {
        let key_val = line.split('=').collect::<Vec<&str>>();
        match ConfigKeys::from_str(key_val[0]) {
            Ok(ConfigKeys::WindowSize) => config.window_size = key_val[1].parse::<usize>().unwrap(),
            Ok(ConfigKeys::MinimumVelocity) => {
                config.minimum_velocity = key_val[1].parse::<f64>().unwrap()
            }
            Ok(ConfigKeys::EpsilonVelocity) => {
                config.epsilon_velocity = key_val[1].parse::<f64>().unwrap()
            }
            Ok(ConfigKeys::StopDurationMinutes) => {
                config.stop_duration_minutes = key_val[1].parse::<f64>().unwrap()
            }
            Ok(ConfigKeys::ConnectionTimeout) => {
                config.connection_timeout = key_val[1].parse::<f64>().unwrap()
            }
            Ok(ConfigKeys::StopDiagonalMeters) => {
                config.stop_diagonal_meters = key_val[1].parse::<f64>().unwrap()
            }
            Ok(ConfigKeys::RelaxBboxMinutes) => {
                config.relax_bbox_minutes = key_val[1].parse::<f64>().unwrap()
            }
            Ok(ConfigKeys::RelaxBboxMeters) => {
                config.relax_bbox_meters = key_val[1].parse::<f64>().unwrap()
            }
            Ok(ConfigKeys::MaxHausdorffMeters) => {
                config.max_hausdorff_meters = key_val[1].parse::<f64>().unwrap()
            }
            Err(_) => {
                panic!("Mismatched config key: {}", key_val[0])
            }
        }
    }
    config
        .lines()
        .for_each(|line: &str| handle_line(line, &mut default_config));
    default_config
}
