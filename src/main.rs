use datastructures::Graph;
use std::io::{BufReader, Read};
use utility::{CHFilter, MotionDetector};
mod datastructures;
mod parser;
mod utility;
// const MIN_VELOCITY: f64 = 2.5; // If the average velocity of WINDOWS_SIZE points is less than MIN_VELOCITY km/h the trj is cut.

fn main() -> std::io::Result<()> {
    // WINDOW_SIZE is the number of points to consider when filtering spike noise
    let window_size: usize = std::env::var("WINDOW_SIZE")
        .expect("Set WINDOW_SIZE as path variable.")
        .parse::<usize>()
        .unwrap();
    // Lets us specify output directory, for debugging.
    // ideally output should be to a ReTraTree-like database, not separate csv-files.
    // let args: Vec<String> = env::args().collect();
    // The buffered reader could be read from something other than stdin e.g. a tcp-socket.
    let mut buf_reader = BufReader::new(std::io::stdin());
    let mut contents = String::new();
    buf_reader.read_to_string(&mut contents)?;
    // More parsers are probably needed, for now we only accept gpx
    let stream: Vec<[f64; 3]> = parser::parse_gpx(contents);
    let stream: CHFilter<std::vec::IntoIter<[f64; 3]>> =
        CHFilter::new(window_size, stream.into_iter());
    // build the motion detector
    let timespan: f64 = 120000.; // set span to 2 minutes
    let min_velocity: f64 = 2.5; // stopped moving at 2.5 km/h
    let eps: f64 = 0.5; // set epsilon to .5 km/h, i.e., moving at 2.5 + 0.5 = 3 km/h
    let md: MotionDetector = MotionDetector::new(timespan, min_velocity, eps);
    let stream: Vec<[f64; 3]> = stream.collect::<Vec<[f64; 3]>>();
    let graph = Graph::new(stream, md);
    graph.show_vertices();
    Ok(())
}
