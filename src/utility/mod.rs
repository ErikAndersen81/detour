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
pub use motion_detector::MotionDetector;
pub use time_guard::clean_stream;
pub use timeout_handler::TimeoutHandler;

pub use line::Line;
pub use stop_detector::IsStopped;
pub use visvalingam::visvalingam;

/// Returns Euclidean distance in meters
///
/// # Examples
/// todo!
pub fn get_distance(from: &[f64; 3], to: &[f64; 3]) -> f64 {
    ((from[0] - to[0]).powi(2) + (from[1] - to[1]).powi(2)).sqrt()
}
