use super::Bbox;
use crate::CONFIG;

/// Detect if the object is stopped.
/// If the movements of the object within a time frame of `min_duration_ms` is limited to a geofenced location with a diagonal of `max_diagonal_meters` the object is considered to be stopped.
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
#[derive(Debug)]
pub struct StopDetector {
    min_duration_ms: f64,     // Minimum time a stop must last
    max_diagonal_meters: f64, // Maximum width/length of a stop.
    current_bbox: Option<Bbox>,
}

impl StopDetector {
    pub fn new() -> StopDetector {
        StopDetector {
            min_duration_ms: CONFIG.bbox_min_minutes * 60. * 1000.0,
            max_diagonal_meters: CONFIG.bbox_max_meters,
            current_bbox: None,
        }
    }

    pub fn is_stopped(&mut self, point: &[f64; 3]) -> IsStopped {
        if let Some(mut bbox) = self.current_bbox {
            bbox.insert_point(point);
            self.current_bbox = Some(bbox);
        } else {
            // `reset()` was called, i.e. motion detector returned `IsStopped::No`
            // And so should this function.
            // (If it's the first point in the stream this return value
            // is ignored by `PathBuilder.add_point()` anyway.)
            self.current_bbox = Some(Bbox::new(&[*point]));
            return IsStopped::No;
        }
        let spatial_fit = self.current_bbox.unwrap().verify_spatial();
        let temporal_fit = self.current_bbox.unwrap().verify_temporal();
        match (spatial_fit, temporal_fit) {
            (true, true) => IsStopped::Yes,
            (true, false) => IsStopped::Maybe,
            (false, true) | (false, false) => IsStopped::No,
        }
    }

    /// Clear the bbox used for detection.
    pub fn reset(&mut self) {
        self.current_bbox = None;
    }
}

#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum IsStopped {
    Maybe,
    Yes,
    No,
}

#[cfg(test)]
mod test {
    use super::*;
    #[test]
    fn too_litle_time() {
        let config = Config::default();
        // Default time limit is 15 minutes
        let mut sd = StopDetector::new(&config);
        let points = vec![[0., 0., 0.], [0., 0., 5. * 60000.], [0., 0., 10. * 60000.]];
        for point in points {
            assert_eq!(sd.is_stopped(&point), IsStopped::Maybe, "t:{}", point[2]);
        }
    }

    #[test]
    fn too_large_area() {
        let config = Config::default();
        // Default size limit is 50 meters
        let mut sd = StopDetector::new(&config);
        let points = vec![[0., 0., 0.], [0., 51., 5. * 60000.]];
        assert_eq!(sd.is_stopped(&points[0]), IsStopped::Maybe);
        assert_eq!(sd.is_stopped(&points[1]), IsStopped::No);
    }

    #[test]
    fn perfect_fit() {
        let config = Config::default();
        // Default time limit is 15 minutes
        // Default size limit is 50 meters
        let mut sd = StopDetector::new(&config);
        let points = vec![
            [0., 0., 0.],
            [0., 10., 5. * 60000.],
            [10., 10., 10. * 60000.],
            [15., 15., 16. * 60000.],
            [20., 35., 20. * 60000.],
        ];
        assert_eq!(sd.is_stopped(&points[0]), IsStopped::Maybe);
        assert_eq!(sd.is_stopped(&points[1]), IsStopped::Maybe);
        assert_eq!(sd.is_stopped(&points[2]), IsStopped::Maybe);
        assert_eq!(sd.is_stopped(&points[3]), IsStopped::Yes);
        assert_eq!(sd.is_stopped(&points[4]), IsStopped::Yes);
    }

    #[test]
    fn reset_after_start() {
        // Once an object started moving, reset the Bbox
        let config = Config::default();
        // Default time limit is 15 minutes
        // Default size limit is 50 meters
        let mut sd = StopDetector::new(&config);
        let points = vec![
            [0., 0., 0.],             // Maybe
            [0., 10., 20. * 60000.],  // Yes
            [0., 51., 25. * 60000.],  // No
            [15., 51., 30. * 60000.], // Maybe
            [25., 25., 41. * 60000.], // Yes
        ];
        assert_eq!(sd.is_stopped(&points[0]), IsStopped::Maybe);
        assert_eq!(sd.is_stopped(&points[1]), IsStopped::Yes);
        assert_eq!(sd.is_stopped(&points[2]), IsStopped::No);
        assert_eq!(sd.is_stopped(&points[3]), IsStopped::Maybe);
        assert_eq!(sd.is_stopped(&points[4]), IsStopped::Yes, "{:?}", sd);
    }
}
