use ch_filter::CHFilter;
use chrono::NaiveDate;
use regex::Regex;
use std::io::{BufReader, Read};
mod ch_filter;
mod tradis;
mod trajectory;
use std::env;

const MIN_VELOCITY: f64 = 2.5; // If the average velocity of WINDOWS_SIZE points is less than MIN_VELOCITY km/h the trj is cut.

fn main() -> std::io::Result<()> {
    // WINDOW_SIZE is the number of points to consider when filtering spike noise
    let window_size: usize = std::env::var("WINDOW_SIZE")
        .expect("Set WINDOW_SIZE as path variable.")
        .parse::<usize>()
        .unwrap();
    // Lets us specify output directory, for debugging.
    // ideally output should be to a ReTraTree-like database, not separate csv-files.
    let args: Vec<String> = env::args().collect();
    // The buffered reader could be read from something other than stdin e.g. a tcp-socket.
    let mut buf_reader = BufReader::new(std::io::stdin());
    let mut contents = String::new();
    buf_reader.read_to_string(&mut contents)?;
    println!("file length {}", contents.len());
    // More parsers are probably needed, for now we only accept gpx
    let trj_stream: Vec<[f64; 3]> = parse_gpx(contents);
    let mut filtered = CHFilter::new(window_size, trj_stream.into_iter());
    let mut count = 0;
    while let Some(pt) = filtered.next() {
        count += 1;
        print!("0 {:?}", pt);
    }
    println!("\ncount:{}", count);
    Ok(())
}

fn parse_gpx(gpx: String) -> Vec<[f64; 3]> {
    let mut trj: Vec<[f64; 3]> = Vec::new();
    let re_lat = Regex::new(r"lat=\W(\d+[[:punct:]]\d+)\W\slon=\W(\d+[[:punct:]]\d+)\W{2}[[:space:]]*<ele>\d+[[:punct:]]\d+</ele>[[:space:]]*<time>(\d{4})-(\d{2})-(\d{2})T(\d{2}):(\d{2}):(\d{2})[[:punct:]](\d+)").unwrap();
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
        trj.push([lon, lat, time]); // Note we have x=lon, y=lat, z=time(ms)
    }
    println!("parsed {} coords", trj.len());
    trj
}
