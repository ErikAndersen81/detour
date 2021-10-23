use datastructures::Graph;
use std::io::{BufReader, Read};
use utility::{CHFilter, MotionDetector};
mod datastructures;
mod parser;
mod utility;
use std::env;
use std::path::Path;

fn main() {
    let config = std::fs::read_to_string("config.cfg");
    let config = parser::parse_config(config.unwrap());
    let out_path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "output".to_owned());
    let out_path = Path::new(&out_path);
    assert!(env::set_current_dir(&out_path).is_ok());
    println!(
        "Successfully changed working directory to {}!",
        out_path.display()
    );

    // The buffered reader could be read from something other than stdin e.g. a tcp-socket.
    let mut buf_reader = BufReader::new(std::io::stdin());
    let mut contents = String::new();
    buf_reader
        .read_to_string(&mut contents)
        .expect("can't read from stdin");
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
    graph
        .to_csv(String::from("vertices.csv"))
        .expect("Could not write to csv!");
}
