pub use ch_filter::CHFilter;
use geo::prelude::HaversineDistance;
pub use motion_detector::MotionDetector;
pub use time_guard::clean_stream;
pub use timeout_handler::TimeoutHandler;
pub mod ch_filter;
pub mod motion_detector;
pub mod time_guard;
pub mod timeout_handler;
pub use bounding_box::Bbox;
pub mod bounding_box;

fn get_distance(from: &[f64; 3], to: &[f64; 3]) -> f64 {
    // Returns haversine distance in meters
    let start = geo::point!(x: from[0],y: from[1]);
    let end = geo::point!(x:to[0], y:to[1]);
    start.haversine_distance(&end)
}

#[cfg(test)]
mod utility_test {
    use super::*;
    #[test]
    fn distance_test() {
        // According to google these two points are approximately 2 km apart
        let from = &[10.128126551731393, 55.39057912238903, 0.];
        let to = &[10.159840991123847, 55.386813002794774, 1.];
        let google_distance = 2000.;
        assert!((get_distance(from, to) - google_distance).abs() < 50.);
    }
}
