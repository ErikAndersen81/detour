#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// Number of points used in the CH-filter.
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
    /// Threshhold for Visvalingam algorithm.
    pub visvalingam_threshold: f64,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            window_size: 5,
            minimum_velocity: 2.5,
            epsilon_velocity: 0.5,
            connection_timeout: 120000.0,
            stop_diagonal_meters: 50.0,
            stop_duration_minutes: 15.0,
            relax_bbox_minutes: 30.,
            relax_bbox_meters: 50.,
            max_hausdorff_meters: 100.,
            visvalingam_threshold: 0.5,
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
    VisvalingamThreshold,
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
            "visvalingam_threshold" => Ok(ConfigKeys::VisvalingamThreshold),
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
            Ok(ConfigKeys::VisvalingamThreshold) => {
                config.visvalingam_threshold = key_val[1].parse::<f64>().unwrap()
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
