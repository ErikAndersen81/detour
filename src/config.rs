use serde::{Deserialize, Serialize};
use std::fmt::Display;

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
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
    /// Maximal length of any side of a `Bbox`. If movement occurs within a region of this size it is eligible to be considered a stop.
    pub bbox_max_meters: f64,
    /// The least amount of time that movement must occur within a `Bbox` before it is considered a stop.
    pub bbox_min_minutes: f64,
    /// If two trajectories belonging to the same edge have a Hausdorff distance of more than this, they will not be merged.
    pub max_hausdorff_meters: f64,
    /// Threshold for Visvalingam algorithm.
    pub visvalingam_threshold: f64,
}

impl Default for Config {
    fn default() -> Self {
        Config {
            window_size: 5,
            minimum_velocity: 2.5,
            epsilon_velocity: 1.5,
            motion_detector_timespan: 60000.0,
            connection_timeout: 120000.0,
            bbox_max_meters: 50.0,
            bbox_min_minutes: 15.0,
            max_hausdorff_meters: 100.,
            visvalingam_threshold: 0.5,
        }
    }
}

impl Display for Config {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let json = serde_json::to_string(self).unwrap();
        writeln!(f, "{}", json)?;
        Ok(())
    }
}

/// Parses config.json in the root folder into a [Config](Config) struct
pub fn parse_config(config: String) -> Config {
    serde_json::from_str(&config).unwrap()
}
