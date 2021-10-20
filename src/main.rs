use ch_filter::CHFilter;
use chrono::NaiveDate;
use kd::kd::build_graph;
use motion_detector::MotionDetector;
use regex::Regex;
use std::io::{BufReader, Read};
mod ch_filter;
mod motion_detector;
mod tradis;
pub mod trajectory;
mod kd {
    pub mod kd;
}
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
    let stream: Vec<[f64; 3]> = parse_gpx(contents);
    let stream: CHFilter<std::vec::IntoIter<[f64; 3]>> =
        CHFilter::new(window_size, stream.into_iter());
    // build the motion detector
    let timespan: f64 = 120000.; // set span to 2 minutes
    let min_velocity: f64 = 2.5; // stopped moving at 2.5 km/h
    let eps: f64 = 0.5; // set epsilon to .5 km/h, i.e., moving at 2.5 + 0.5 = 3 km/h
    let md: MotionDetector = MotionDetector::new(timespan, min_velocity, eps);
    let stream: Vec<[f64; 3]> = stream.collect::<Vec<[f64; 3]>>();
    let graph: kd::kd::Graph = build_graph(stream, md);
    graph.show_vertices();
    Ok(())
}

fn parse_gpx(gpx: String) -> Vec<[f64; 3]> {
    let mut trj: Vec<[f64; 3]> = Vec::new();
    let re = Regex::new(r"lat=\W(\d+[[:punct:]]\d+)\W\slon=\W(\d+[[:punct:]]\d+)\W{2}[[:space:]]*<ele>\d+[[:punct:]]\d+</ele>[[:space:]]*<time>(\d{4})-(\d{2})-(\d{2})T(\d{2}):(\d{2}):(\d{2})[[:punct:]](\d+)").unwrap();
    for cap in re.captures_iter(&gpx) {
        let lat: f64 = cap[1].parse::<f64>().unwrap();
        let lon: f64 = cap[2].parse::<f64>().unwrap();
        let yr: i32 = cap[3].parse::<i32>().unwrap();
        let mn: u32 = cap[4].parse::<u32>().unwrap();
        let da: u32 = cap[5].parse::<u32>().unwrap();
        let h: u32 = cap[6].parse::<u32>().unwrap();
        let m: u32 = cap[7].parse::<u32>().unwrap();
        let s: u32 = cap[8].parse::<u32>().unwrap();
        let ms: u32 = cap[9].parse::<u32>().unwrap();
        let time = NaiveDate::from_ymd(yr, mn, da)
            .and_hms_milli(h, m, s, ms)
            .timestamp_millis() as f64;
        trj.push([lon, lat, time]); // Note we have x=lon, y=lat, z=time(ms)
    }
    println!("parsed {} coords", trj.len());
    trj
}
