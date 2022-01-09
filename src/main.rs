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
use geomorph::{coord, utm};
pub use parser::Config;
use std::{
    fs,
    io::{BufReader, Read},
};
pub use utility::{time_guard, CHFilter, StopDetector};
mod data_structures;
mod parser;
mod utility;
use std::{env, path::Path};

use crate::{data_structures::get_graph, utility::visvalingam};

pub struct Coord {
    pub x: f64,
    pub y: f64,
    pub t: f64,
}

impl Coord {
    pub fn from_gps(pt: &[f64; 3]) -> Self {
        let c: utm::Utm = coord::Coord::new(pt[1], pt[0]).into();
        Coord {
            x: c.easting,
            y: c.northing,
            t: pt[2],
        }
    }
    /// Converts to GPS assuming UTM coordinates belong to zone 32 (Denmark)
    pub fn to_gps(&self) -> [f64; 3] {
        let c: coord::Coord = utm::Utm::new(self.x, self.y, true, 32, 'U', false).into();
        [c.lon, c.lat, self.t]
    }
}

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

    /////////////////////////////////////////////////////////////////////
    // // This is for 'testing' the simplification procedure	       //
    // for stream in daily_streams.iter() {			       //
    //     let a = stream.len();				       //
    //     let b = visvalingam(stream, config.epsilon_velocity).len(); //
    //     println!("a:{}, b:{}", a, b);			       //
    // }							       //
    /////////////////////////////////////////////////////////////////////

    let graph = get_graph(daily_streams, config);
    graph.to_csv().expect("Could not write output.");
}
