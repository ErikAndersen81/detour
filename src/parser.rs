use chrono::NaiveDate;
use regex::Regex;
use std::str::FromStr;

/// Parses a string containing GPX data.
///
/// Creates an array with UTM projected `[easting, northing, time]` for each `<trkpt>`.
/// See [Coord](crate::Coord) for details on projection.
/// The date part of `time` is
/// stripped and the timestamp is converted to milliseconds.
pub fn parse_gpx(gpx: String) -> Vec<Vec<[f64; 3]>> {
    let mut trjs: Vec<Vec<[f64; 3]>> = Vec::new();
    let mut trj: Vec<[f64; 3]> = Vec::new();
    let re = Regex::new(r"lat=\W(\d+[[:punct:]]\d+)\W\slon=\W(\d+[[:punct:]]\d+)\W{2}[[:space:]]*<ele>\d+[[:punct:]]\d+</ele>[[:space:]]*<time>(\d{4})-(\d{2})-(\d{2})T(\d{2}):(\d{2}):(\d{2})[[:punct:]](\d+)").unwrap();
    let (mut yr_, mut mn_, mut da_): (i32, u32, u32) = (0, 0, 0);
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
        let time = NaiveDate::from_ymd(1970, 1, 1)
            .and_hms_milli(h, m, s, ms)
            .timestamp_millis() as f64;
        if yr != yr_ || mn != mn_ || da != da_ {
            yr_ = yr;
            mn_ = mn;
            da_ = da;
            trjs.push(trj);
            trj = vec![];
        }
        let c = crate::Coord::from_gps(&[lon, lat, time]);
        trj.push([c.x, c.y, c.t]); // Note we have x=lon, y=lat, z=time(ms)
    }
    trjs
}

