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
//! Various settings can be adjusted by modifying config.json located in
//! the root folder. Read more about [Config](Config) here.
#[macro_use]
extern crate lazy_static;

pub mod arguments;
pub mod config;
use arguments::{ClusteringArgs, Output};
pub use config::Config;
use graph::{get_graph_v2, Writable};
use std::{
    io::{BufReader, Read},
    sync::Mutex,
};
pub use utility::{time_guard, CHFilter, StopDetector};
mod coord;
mod graph;
mod parser;
mod utility;
use crate::{graph::get_graph, utility::visvalingam};
pub use coord::{from_epsg_3857_to_4326, from_epsg_4326_to_3857};
lazy_static! {
    pub static ref CONFIG: Config = arguments::parse_arguments();
}

lazy_static! {
    pub static ref OUTPUT: Mutex<Output> = Mutex::new(Output::default());
}

lazy_static! {
    pub static ref CLUSTERINGARGS: Mutex<ClusteringArgs> = Mutex::new(ClusteringArgs::default());
}

#[derive(Debug, Default, Clone)]
pub struct Statistics {
    pub node_merges: usize,
    pub edge_merges: usize,
    pub node_splits: usize,
    pub redundant_trj_removals: usize,
    pub redundant_node_removals: usize,
    pub outlier_node_removals: usize,
    pub spatial_clusters: usize,
}

lazy_static! {
    pub static ref STATS: Mutex<Statistics> = Mutex::new(Statistics::default());
}

fn main() {
    if (CONFIG.window_size == 0_usize) | (CONFIG.window_size != 0_usize) {
        // This bogus test ensures we parse arguments before trying to read from stdin
        // s.t. the user can get a helpful message
    }
    let mut buf_reader = BufReader::new(std::io::stdin());
    let mut contents = String::new();
    buf_reader
        .read_to_string(&mut contents)
        .expect("can't read from stdin");
    //println!("Parsing input...");
    let daily_streams: Vec<Vec<[f64; 3]>> = parser::parse(contents)
        .into_iter()
        .filter(|day| !day.is_empty())
        .map(time_guard::clean_stream)
        //.map(|stream| {
        //    CHFilter::new(CONFIG.window_size, stream.into_iter()).collect::<Vec<[f64; 3]>>()
        //})
        .collect();
    //println!("Constructing graph...");
    let graph = get_graph_v2(daily_streams);
    graph.to_csv().expect("Could not write output.");
}
