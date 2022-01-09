//! Various utilities
//!
//! Contains several functions that may be useful in other scenarios as well.

pub mod bounding_box;
pub mod ch_filter;
pub mod clustering;
pub mod line;
pub mod motion_detector;
mod stop_detector;
pub mod time_guard;
pub mod timeout_handler;
pub mod trajectory;
pub mod visvalingam;
#[doc(inline)]
pub use self::stop_detector::StopDetector;
pub use bounding_box::Bbox;
#[doc(inline)]
pub use ch_filter::CHFilter;
pub use clustering::Clustering;
use geo::prelude::HaversineDistance;
pub use motion_detector::MotionDetector;
pub use time_guard::clean_stream;
pub use timeout_handler::TimeoutHandler;

pub use line::Line;
pub use visvalingam::visvalingam;

/// Returns Euclidean distance in meters
///
/// # Examples
/// todo!
fn get_distance(from: &[f64; 3], to: &[f64; 3]) -> f64 {
    // This is the old distance, before we decided to project to UTM
    //let start = geo::point!(x: from[0],y: from[1]);
    //let end = geo::point!(x:to[0], y:to[1]);
    //start.haversine_distance(&end)
    ((from[0] - to[0]).powi(2) + (from[1] - to[1]).powi(2)).sqrt()
}

#[cfg(test)]
mod test {
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
