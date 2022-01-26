//! Constructs Detour Graph
//!
//! Command line utility for constructing an acyclic graph that represents
//! condensed movement patterns of the GPX files given as input.
//! Streams are cleaned using a [temporal filter](time_guard::clean_stream)
//! which ensures temporal monotonicity.
//!
//! ## Example
//! Say you have a bunch of GPX files located in /home/user/gpx you want to use as input. Pipe the output to the program like:
//! ``` shell
//! $ cat /home/user/gpx/* | .detour -o /home/user/output
//! ```
//!
//! To use a specific configuration file use the `-c` or `--config` option followed by desired configuration file.
//! By default output is written to a folder called `Output` unless otherwise is specified by using the `-o` or `--output` option.
//! ## Configuration
//! Various settings can be adjusted by modifying config.cfg located in
//! the root folder. Read more about [Config](Config) here.
#[macro_use]
extern crate lazy_static;

pub mod arguments;
pub mod config;
pub use config::Config;
use std::io::{BufReader, Read};
pub use utility::{time_guard, CHFilter, StopDetector};
mod coord;
mod graph;
mod parser;
mod utility;
use crate::{graph::get_graph, utility::visvalingam};
pub use coord::Coord;

lazy_static! {
    pub static ref CONFIG: Config = arguments::parse_arguments();
}

fn main() {
    let mut buf_reader = BufReader::new(std::io::stdin());
    let mut contents = String::new();
    buf_reader
        .read_to_string(&mut contents)
        .expect("can't read from stdin");
    println!("Parsing input...");
    let daily_streams: Vec<Vec<[f64; 3]>> = parser::parse_plt(contents)
        .into_iter()
        .filter(|day| !day.is_empty())
        .map(time_guard::clean_stream)
        .map(|stream| {
            CHFilter::new(CONFIG.window_size, stream.into_iter()).collect::<Vec<[f64; 3]>>()
        })
        .collect();
    println!("Constructing graph...");
    let graph = get_graph(daily_streams);
    graph.to_csv().expect("Could not write output.");
}
