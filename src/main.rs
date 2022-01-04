//! Constructs Detour Graph
//!
//! Command line utility for constructing an acyclic graph that represents
//! condensed movement patterns of the GPX files given as input.
//! Streams are cleaned using a [temporal filter](time_guard::clean_stream)
//! which ensures temporal monotonicity.
//!
//! ## Example
//! Say you have a bunch of GPX files located in /home/user/gpx you want to use as input. Pipe the output to the program and specify an output folder like so:
//! ``` shell
//! $ cat /home/user/gpx/* | .detour /home/user/output
//! ```
//! ## Configuration
//! Various settings can be adjusted by modifying config.cfg located in
//! the root folder:
//! - `window_size` - Number of points used in the [CH-filter](CHFilter)
//! - To determine if an object is stopped we use a [stop detector](StopDetector) which has two configuration options:
//!   - `stop_diagonal_meters` - Maximal diagonal size of a geofenced region. If movement occurs within a region of this size it is eligible to be considered a stop.
//!   - `stop_duration_minutes` - The least amount of time that movement must occur within a geofenced region before it is considered a stop.
//! - `connection_timeout` - Maximal number of milliseconds between two measurements before the stream is cut into two.
//! - When we merge stops (bounding boxes) we can temporarily expand them when searching for mathes:
//!   - `relax_bbox_minutes` - Allow stops to be this many minutes apart.
//!   - `relax_bbox_meters` - Allow stops to be this many meters apart.

#![feature(slice_group_by)]
use std::{
    fs,
    io::{BufReader, Read},
};
pub use utility::{time_guard, CHFilter, StopDetector};
mod data_structures;
mod parser;
mod utility;
use std::{env, path::Path};

use crate::data_structures::get_graph;

fn main() {
    let config = parser::parse_config(std::fs::read_to_string("config.cfg").unwrap());
    let out_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "output".to_owned());
    fs::create_dir_all(&out_path).expect("Cant write to specified output folder");
    println!("Writing output to: {}", &out_path);
    let out_path = Path::new(&out_path);
    assert!(env::set_current_dir(&out_path).is_ok());
    let mut buf_reader = BufReader::new(std::io::stdin());
    let mut contents = String::new();
    buf_reader
        .read_to_string(&mut contents)
        .expect("can't read from stdin");
    let daily_streams: Vec<Vec<[f64; 3]>> = parser::parse_gpx(contents)
        .into_iter()
        .filter(|day| !day.is_empty())
        .map(time_guard::clean_stream)
        .map(|stream| {
            CHFilter::new(config.window_size, stream.into_iter()).collect::<Vec<[f64; 3]>>()
        })
        .collect();
    println!("parsed {} days", daily_streams.len());
    let graph = get_graph(daily_streams, config);
    graph.to_csv().expect("Could not write output.");
}
