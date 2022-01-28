use chrono::NaiveDate;
use regex::Regex;

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
            if !trj.is_empty() {
                trjs.push(trj);
                trj = vec![];
            }
        }
        let c = crate::Coord::from_gps(&[lon, lat, time]);
        trj.push([c.x, c.y, c.t]); // Note we have x=lon, y=lat, z=time(ms)
    }
    trjs
}

/// Parses a string containing PLT data.
///
/// Specifically designed to data from Geolife Trajectories 1.3.
/// Creates an array with UTM projected `[easting, northing, time]` coordinates and time.
/// See [Coord](crate::Coord) for details on projection.
/// We only use fields 1 (latitude), 2(longitude), 5(days) and 7(time).
/// Each day is put in a separate trajectory(`Vec`)
pub fn parse_plt(plt: String) -> Vec<Vec<[f64; 3]>> {
    let mut trjs: Vec<Vec<[f64; 3]>> = Vec::new();
    let mut trj: Vec<[f64; 3]> = Vec::new();
    let mut last_day = -1;
    let lines = plt.lines();
    for line in lines {
        let mut fields = line.split(',');
        let mut coord: [f64; 3] = [f64::NAN, f64::NAN, f64::NAN];

        if let Some(lat) = fields.next() {
            if let Ok(lat) = lat.parse::<f64>() {
                coord[1] = lat;
            }
        };
        if let Some(lon) = fields.next() {
            if let Ok(lon) = lon.parse::<f64>() {
                coord[0] = lon;
            }
        };
        if let Some(day) = fields.nth(2) {
            if let Ok(day) = day.parse::<f64>() {
                if day as i32 > last_day {
                    last_day = day as i32;
                    if !trj.is_empty() {
                        trjs.push(trj);
                        trj = vec![];
                    }
                }
            }
        }
        if let Some(time) = fields.nth(1) {
            let mut time = time.split(':');
            let ms: f64 = match [time.next(), time.next(), time.next()] {
                [Some(h), Some(m), Some(s)] => {
                    match [h.parse::<f64>(), m.parse::<f64>(), s.parse::<f64>()] {
                        [Ok(h), Ok(m), Ok(s)] => {
                            (h * 60.0 * 60.0 * 1000.0) + (m * 60.0 * 1000.0) + (s * 1000.0)
                        }
                        _ => f64::NAN,
                    }
                }
                _ => f64::NAN,
            };
            coord[2] = ms
        }
        if coord[0].is_finite() & coord[1].is_finite() & coord[2].is_finite() {
            let c = crate::Coord::from_gps(&coord);
            trj.push([c.x, c.y, c.t]); // Note we have x=lon, y=lat, z=time(ms)
        }
    }
    trjs
}

/// Determines if content is GPX or PLT and parses content.
/// Specifically, if the first line is 'Geolife trajectory'
/// (casing ignored), it is assumed to be PLT otherwise
/// it's considered to be GPX.
pub fn parse(content: String) -> Vec<Vec<[f64; 3]>> {
    let line = content.lines().next();
    if let Some(line) = line {
        let line = line.trim().to_ascii_lowercase();
        match line.as_str() {
            "geolife trajectory" => parse_plt(content),
            _ => parse_gpx(content),
        }
    } else {
        panic!("Nothing to read from stdin!")
    }
}
