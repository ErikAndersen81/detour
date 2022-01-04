use super::Bbox;
use crate::parser::Config;

/// Detect if the object is stopped.
/// If the movements of the object within a time frame of `duration_ms` is limited to a geofenced location with a diagonal of `diagonal_meters` the object is considered to be stopped.
///
/// # Example
/// ``` rust
/// fn object_is_stopped(stream:Vec<[f64;3]>, config:Config) -> Vec<([f64;3],bool)> {
///     let mut sd = StopDetector::new(&config);
///     stream
///         .into_iter()
///         .map(|point| (point,sd.is_stopped(point)))
///}
/// // Returns a list of tuples containing a coordinate and a boolean
/// // indicating whether the object is stopped at the given point.
/// ```
pub struct StopDetector {
    duration_ms: f64,
    diagonal_meters: f64,  // Maximal length of Bbox diagonal (spatially)
    points: Vec<[f64; 3]>, // last read points spanning no more than timespan
}

impl StopDetector {
    pub fn new(config: &Config) -> StopDetector {
        StopDetector {
            duration_ms: config.stop_duration_minutes * 60. * 1000.0,
            diagonal_meters: config.stop_diagonal_meters,
            points: vec![],
        }
    }

    pub fn is_stopped(&mut self, point: [f64; 3]) -> bool {
        self.points.push(point);
        self.fit_to_timespan();
        let bbox = Bbox::new(&self.points);
        bbox.get_diameter() < self.diagonal_meters
    }

    fn fit_to_timespan(&mut self) {
        while self.points.len() > 1
            && self.points[self.points.len() - 1][2] - self.points[0][2] > self.duration_ms
        {
            self.points.remove(0);
        }
    }
}
