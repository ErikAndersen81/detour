use chrono::NaiveDate;
use geo::prelude::HaversineDistance;
use regex::Regex;
use std::io::{BufReader, Read};
use std::iter::FromIterator;
use trajectory::Trajectory;
mod trajectory;
use std::env;

const WINDOW_SIZE: usize = 6; // number of points to consider when filtering spike noise
                              // If the average velocity of WINDOWS_SIZE points is less than MIN_VELOCITY km/h the trj is cut.
const MIN_VELOCITY: f64 = 2.5;

fn main() -> std::io::Result<()> {
    // Lets us specify output directory, for debugging.
    // ideally output should be to a ReTraTree-like database, not separate csv-files.
    let args: Vec<String> = env::args().collect();
    // The buffered reader could be read from something other than stdin e.g. a tcp-socket.
    let mut buf_reader = BufReader::new(std::io::stdin());
    let mut contents = String::new();
    buf_reader.read_to_string(&mut contents)?;
    // More parsers are probably needed, for now we only accept gpx
    let trj_stream: Vec<[f64; 3]> = parse_gpx(contents);
    let mut builder = TrajectoryBuilder::new();
    let mut count = 0;
    for coord in trj_stream {
        if let Some(trj) = builder.handle_next(coord) {
            if trj.len() > 1 {
                count += 1;
                trj.to_csv(format!("{}/trj{}_win{}.csv", args[1], count, WINDOW_SIZE));
                println!("wrote trj with len: {}", trj.to_array().len());
            }
        };
    }
    Ok(())
}

fn parse_gpx(gpx: String) -> Vec<[f64; 3]> {
    let mut trj: Vec<[f64; 3]> = Vec::new();
    let re_lat = Regex::new(r"lat=\W(\d+[[:punct:]]\d+)\W\slon=\W(\d+[[:punct:]]\d+)\W{2}<ele>\d+[[:punct:]]\d+</ele><time>(\d{4})-(\d{2})-(\d{2})T(\d{2}):(\d{2}):(\d{2})[[:punct:]](\d+)").unwrap();
    for cap in re_lat.captures_iter(&gpx) {
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
        trj.push([lon, lat, time]); // Note we have x=lon, y=lat, z=time
    }
    trj
}

fn get_velocity(from: &[f64; 3], to: &[f64; 3]) -> f64 {
    let km = get_distance(from, to) / 1000.0;
    let h = (to[2] - from[2]) / (3600. * 1000.);
    km / h
}

fn get_distance(from: &[f64; 3], to: &[f64; 3]) -> f64 {
    // Returns haversine distance in meters
    let start = geo::point!(x: from[0],y: from[1]);
    let end = geo::point!(x:to[0], y:to[1]);
    start.haversine_distance(&end)
}

fn avg_velocity(window: Vec<[f64; 3]>) -> f64 {
    let mut vels: Vec<f64> = Vec::new();
    for i in 1..(window.len() - 1) {
        let v = get_velocity(&window[i - 1], &window[i]);
        vels.push(v);
    }
    vels.iter().sum::<f64>() / (vels.len() as f64)
}

struct TrajectoryBuilder {
    trj: Vec<[f64; 3]>,
    window: Vec<[f64; 3]>,
}

impl TrajectoryBuilder {
    fn new() -> TrajectoryBuilder {
        TrajectoryBuilder {
            trj: Vec::new(),
            window: Vec::new(),
        }
    }

    fn get_trajectory(&self) -> Trajectory {
        Trajectory::from_array(self.trj.clone())
    }

    fn handle_next(&mut self, next: [f64; 3]) -> Option<Trajectory> {
        self.window.push(next);
        if self.window.len() < WINDOW_SIZE {
            return None;
        }
        let trj: Vec<[f64; 3]> = get_convex_hull_trj(self.window.clone());
        let trj: Vec<[f64; 3]> = remove_spikes(trj);
        if ((&trj).len() >= 3) && (avg_velocity((&trj).clone()) < MIN_VELOCITY) {
            let trj = self.get_trajectory();
            self.window = Vec::new();
            self.trj = Vec::new();
            return Some(trj);
        }
        if trj.len() == WINDOW_SIZE {
            let coord = trj[0];
            let window = Vec::from_iter(trj[1..((&trj).len() - 1)].iter().cloned());
            self.trj.push(coord);
            self.window = window;
        } else {
            self.window = trj;
        }
        None
    }
}

fn get_convex_hull_trj(points: Vec<[f64; 3]>) -> Vec<[f64; 3]> {
    let mut coords2d: Vec<geo::Coordinate<f64>> = points
        .iter()
        .map(|c| geo::Coordinate { x: c[0], y: c[1] })
        .collect::<Vec<geo::Coordinate<f64>>>();
    let extreme_pts: geo::LineString<f64> =
        geo::algorithm::convex_hull::quick_hull(coords2d.as_mut_slice());
    fn same_point(c3: &[f64; 3], c2: &geo::Point<f64>) -> bool {
        (c3[0] - c2.x()).abs() + (c3[1] - c2.y()).abs() < 0.00000001
    }
    fn is_extreme(c3: &[f64; 3], extreme_pts: &geo::LineString<f64>) -> bool {
        extreme_pts
            .clone()
            .into_points()
            .into_iter()
            .any(|p| same_point(c3, &p))
    }
    points
        .into_iter()
        .filter(|p| is_extreme(p, &extreme_pts))
        .collect::<Vec<[f64; 3]>>()
}

fn remove_spikes(trj: Vec<[f64; 3]>) -> Vec<[f64; 3]> {
    let mut spikeless_trj: Vec<[f64; 3]> = Vec::new();
    fn is_spike(p: &[f64; 3], q: &[f64; 3], r: &[f64; 3]) -> bool {
        (get_distance(p, q) > get_distance(p, r)) | (get_distance(q, r) > get_distance(p, r))
    }
    spikeless_trj.push(trj[0]);
    for i in 1..trj.len() - 1 {
        if !is_spike(&trj[i - 1], &trj[i], &trj[i + 1]) {
            spikeless_trj.push(trj[i])
        }
    }
    spikeless_trj.push(trj[trj.len() - 1]);
    spikeless_trj
}
