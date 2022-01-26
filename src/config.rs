use std::{fmt::Display, str::FromStr};

#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// Number of points used in the CH-filter.
    pub window_size: usize,
    /// If the object moves slower than this it is considered to be stopped.
    pub minimum_velocity: f64,
    /// If the object moves faster than `minimum_velocity` plus this then it is considered to be moving again.
    pub epsilon_velocity: f64,
    /// The timespan used by the `MotionDetector` to calculate average velocity.
    pub motion_detector_timespan: f64,
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
    /// Threshold for Visvalingam algorithm.
    pub visvalingam_threshold: f64,
    /// UTM zone used for projection. This zone will be extended if needed.
    pub utm_zone: i32,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            window_size: 5,
            minimum_velocity: 2.5,
            epsilon_velocity: 1.5,
            motion_detector_timespan: 60000.0,
            connection_timeout: 120000.0,
            stop_diagonal_meters: 50.0,
            stop_duration_minutes: 15.0,
            relax_bbox_minutes: 30.,
            relax_bbox_meters: 50.,
            max_hausdorff_meters: 100.,
            visvalingam_threshold: 0.5,
            utm_zone: 32, // DK
        }
    }
}

impl Display for Config {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "window_size={}", self.window_size)?;
        writeln!(f, "minimum_velocity={}", self.minimum_velocity)?;
        writeln!(f, "epsilon_velocity={}", self.epsilon_velocity)?;
        writeln!(
            f,
            "motion_detector_timespan={}",
            self.motion_detector_timespan
        )?;
        writeln!(f, "connection_timeout={}", self.connection_timeout)?;
        writeln!(f, "stop_diagonal_meters={}", self.stop_diagonal_meters)?;
        writeln!(f, "stop_duration_minutes={}", self.stop_duration_minutes)?;
        writeln!(f, "relax_bbox_minutes={}", self.relax_bbox_minutes)?;
        writeln!(f, "relax_bbox_meters={}", self.relax_bbox_meters)?;
        writeln!(f, "max_hausdorff_meters={}", self.max_hausdorff_meters)?;
        writeln!(f, "visvalingam_threshold={}", self.visvalingam_threshold)?;
        writeln!(f, "UTM Zone={}", self.utm_zone)?;
        Ok(())
    }
}

enum ConfigKeys {
    WindowSize,
    MinimumVelocity,
    EpsilonVelocity,
    MotionDetectorTimespan,
    StopDurationMinutes,
    ConnectionTimeout,
    StopDiagonalMeters,
    RelaxBboxMinutes,
    RelaxBboxMeters,
    MaxHausdorffMeters,
    VisvalingamThreshold,
    UTMZone,
}

impl FromStr for ConfigKeys {
    type Err = ();

    fn from_str(s: &str) -> Result<ConfigKeys, ()> {
        match s {
            "window_size" => Ok(ConfigKeys::WindowSize),
            "minimum_velocity" => Ok(ConfigKeys::MinimumVelocity),
            "epsilon_velocity" => Ok(ConfigKeys::EpsilonVelocity),
            "motion_detector_timespan" => Ok(ConfigKeys::MotionDetectorTimespan),
            "stop_duration_minutes" => Ok(ConfigKeys::StopDurationMinutes),
            "connection_timeout" => Ok(ConfigKeys::ConnectionTimeout),
            "stop_diagonal_meters" => Ok(ConfigKeys::StopDiagonalMeters),
            "relax_bbox_minutes" => Ok(ConfigKeys::RelaxBboxMinutes),
            "relax_bbox_meters" => Ok(ConfigKeys::RelaxBboxMeters),
            "max_hausdorff_meters" => Ok(ConfigKeys::MaxHausdorffMeters),
            "visvalingam_threshold" => Ok(ConfigKeys::VisvalingamThreshold),
            "utm_zone" => Ok(ConfigKeys::UTMZone),
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
            Ok(ConfigKeys::WindowSize) => {
                config.window_size = key_val[1].trim().parse::<usize>().expect("window_size")
            }
            Ok(ConfigKeys::MinimumVelocity) => {
                config.minimum_velocity =
                    key_val[1].trim().parse::<f64>().expect("minimum_velocity")
            }
            Ok(ConfigKeys::EpsilonVelocity) => {
                config.epsilon_velocity =
                    key_val[1].trim().parse::<f64>().expect("epsilon_velocity")
            }
            Ok(ConfigKeys::MotionDetectorTimespan) => {
                config.motion_detector_timespan = key_val[1]
                    .trim()
                    .parse::<f64>()
                    .expect("motion_detector_timespan")
            }
            Ok(ConfigKeys::StopDurationMinutes) => {
                config.stop_duration_minutes = key_val[1]
                    .trim()
                    .parse::<f64>()
                    .expect("stop_duration_minutes")
            }
            Ok(ConfigKeys::ConnectionTimeout) => {
                config.connection_timeout = key_val[1]
                    .trim()
                    .parse::<f64>()
                    .expect("connection_timeout")
            }
            Ok(ConfigKeys::StopDiagonalMeters) => {
                config.stop_diagonal_meters = key_val[1]
                    .trim()
                    .parse::<f64>()
                    .expect("stop_diagonal_meters")
            }
            Ok(ConfigKeys::RelaxBboxMinutes) => {
                config.relax_bbox_minutes = key_val[1]
                    .trim()
                    .parse::<f64>()
                    .expect("relax_bbox_minutes")
            }
            Ok(ConfigKeys::RelaxBboxMeters) => {
                config.relax_bbox_meters =
                    key_val[1].trim().parse::<f64>().expect("relax_bbox_meters")
            }
            Ok(ConfigKeys::MaxHausdorffMeters) => {
                config.max_hausdorff_meters = key_val[1]
                    .trim()
                    .parse::<f64>()
                    .expect("max_hausdorff_meters")
            }
            Ok(ConfigKeys::VisvalingamThreshold) => {
                config.visvalingam_threshold = key_val[1]
                    .trim()
                    .parse::<f64>()
                    .expect("visvalingam_threshold")
            }
            Ok(ConfigKeys::UTMZone) => {
                config.utm_zone = key_val[1].trim().parse::<i32>().expect("utm_zone")
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
