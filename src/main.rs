use datastructures::Graph;
use std::io::{BufReader, Read};
use utility::{CHFilter, MotionDetector};
mod datastructures;
mod parser;
mod utility;

fn main() -> std::io::Result<()> {
    let config = std::fs::read_to_string("config.cfg");
    let config = parser::parse_config(config.unwrap());

    // The buffered reader could be read from something other than stdin e.g. a tcp-socket.
    let mut buf_reader = BufReader::new(std::io::stdin());
    let mut contents = String::new();
    buf_reader.read_to_string(&mut contents)?;
    // More parsers are probably needed, for now we only accept gpx
    let stream: Vec<[f64; 3]> = parser::parse_gpx(contents);
    let stream: CHFilter<std::vec::IntoIter<[f64; 3]>> =
        CHFilter::new(config.window_size, stream.into_iter());
    // build the motion detector
    let md: MotionDetector = MotionDetector::new(
        config.timespan,
        config.minimum_velocity,
        config.epsilon_velocity,
    );
    let stream: Vec<[f64; 3]> = stream.collect::<Vec<[f64; 3]>>();
    let graph = Graph::new(stream, md);
    graph.show_vertices();
    Ok(())
}
