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
//! the root folder. Read more about [Config](Config) here.
#![feature(slice_group_by)]
pub use parser::Config;
use std::{
    fs,
    io::{BufReader, Read},
};
pub use utility::{time_guard, CHFilter, StopDetector};
mod data_structures;
mod parser;
mod utility;
use crate::{data_structures::get_graph, utility::visvalingam};
pub use data_structures::coord::Coord;
use std::{env, path::Path};

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
            let stream =
                CHFilter::new(config.window_size, stream.into_iter()).collect::<Vec<[f64; 3]>>();
            visvalingam(&stream, config.visvalingam_threshold)
        })
        .collect();
    println!("parsed {} days", daily_streams.len());
    let graph = get_graph(daily_streams, config);
    graph.to_csv().expect("Could not write output.");
}
