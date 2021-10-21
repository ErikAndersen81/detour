use std::str::FromStr;

use chrono::NaiveDate;
use regex::Regex;

pub fn parse_gpx(gpx: String) -> Vec<[f64; 3]> {
    let mut trj: Vec<[f64; 3]> = Vec::new();
    let re = Regex::new(r"lat=\W(\d+[[:punct:]]\d+)\W\slon=\W(\d+[[:punct:]]\d+)\W{2}[[:space:]]*<ele>\d+[[:punct:]]\d+</ele>[[:space:]]*<time>(\d{4})-(\d{2})-(\d{2})T(\d{2}):(\d{2}):(\d{2})[[:punct:]](\d+)").unwrap();
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
        let time = NaiveDate::from_ymd(yr, mn, da)
            .and_hms_milli(h, m, s, ms)
            .timestamp_millis() as f64;
        trj.push([lon, lat, time]); // Note we have x=lon, y=lat, z=time(ms)
    }
    println!("parsed {} coords", trj.len());
    trj
}

pub struct Config {
    pub window_size: usize,
    pub minimum_velocity: f64,
    pub epsilon_velocity: f64,
    pub timespan: f64,
}

enum ConfigKeys {
    WindowSize,
    MinimumVelocity,
    EpsilonVelocity,
    Timespan,
}

impl FromStr for ConfigKeys {
    type Err = ();

    fn from_str(s: &str) -> Result<ConfigKeys, ()> {
        match s {
            "window_size" => Ok(ConfigKeys::WindowSize),
            "minimum_velocity" => Ok(ConfigKeys::MinimumVelocity),
            "epsilon_velocity" => Ok(ConfigKeys::EpsilonVelocity),
            "timespan" => Ok(ConfigKeys::Timespan),
            _ => Err(()),
        }
    }
}

pub fn parse_config(config: String) -> Config {
    let mut default_config: Config = Config {
        window_size: 5,
        minimum_velocity: 2.5,
        epsilon_velocity: 0.5,
        timespan: 120000.0,
    };

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
            Ok(ConfigKeys::Timespan) => config.timespan = key_val[1].parse::<f64>().unwrap(),
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
