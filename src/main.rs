#![feature(slice_group_by)]
use std::io::{BufReader, Read};
use utility::{time_guard, CHFilter};
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
    println!("Writing output to: {}", out_path);
    let out_path = Path::new(&out_path);
    assert!(env::set_current_dir(&out_path).is_ok());
    // The buffered reader could be read from something other than stdin e.g. a tcp-socket.
    let mut buf_reader = BufReader::new(std::io::stdin());
    let mut contents = String::new();
    buf_reader
        .read_to_string(&mut contents)
        .expect("can't read from stdin");
    // More parsers are probably needed, for now we only accept gpx
    let daily_streams: Vec<Vec<[f64; 3]>> = parser::parse_gpx(contents)
        .into_iter()
        .filter(|day| !day.is_empty())
        .map(time_guard::clean_stream)
        .map(|stream| {
            CHFilter::new(config.window_size, stream.into_iter()).collect::<Vec<[f64; 3]>>()
        })
        .collect();
    println!("parsed {} days", daily_streams.len());
    let _graph = get_graph(daily_streams, config);
}
