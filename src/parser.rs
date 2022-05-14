use chrono::{Datelike, NaiveDate, NaiveDateTime, Timelike};
use regex::Regex;

/// Parses a string containing GPX data.
///
/// Creates an array with EPSG 3857 projected `[easting, northing, time]` for each `<trkpt>`.
/// The date part of `time` is
/// stripped and the timestamp is converted to milliseconds.
pub fn parse_gpx(gpx: String) -> Vec<Vec<[f64; 3]>> {
    println!("Parsing GPX");
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
        let c = crate::from_epsg_4326_to_3857(&[lat, lon, time]);
        trj.push(c);
    }
    if !trj.is_empty() {
        trjs.push(trj);
    }
    trjs
}

/// Parses a string containing PLT data.
///
/// Specifically designed to data from Geolife Trajectories 1.3.
/// Creates an array with EPSG 3857 `[easting, northing, time]` coordinates and time.
/// We only use fields 1 (latitude), 2(longitude), 5(days) and 7(time).
/// Each day is put in a separate trajectory(`Vec`)
pub fn parse_plt(plt: String) -> Vec<Vec<[f64; 3]>> {
    println!("Parsing PLT");
    let mut trjs: Vec<Vec<[f64; 3]>> = Vec::new();
    let mut trj: Vec<[f64; 3]> = Vec::new();
    let mut last_day = -1;
    let lines = plt.lines();
    for line in lines {
        let mut fields = line.split(',');
        let mut coord: [f64; 3] = [f64::NAN, f64::NAN, f64::NAN];

        if let Some(lat) = fields.next() {
            if let Ok(lat) = lat.parse::<f64>() {
                coord[0] = lat;
            }
        };
        if let Some(lon) = fields.next() {
            if let Ok(lon) = lon.parse::<f64>() {
                coord[1] = lon;
            }
        };
        if let Some(day) = fields.nth(2) {
            if let Ok(day) = day.parse::<f64>() {
                if day as i32 != last_day {
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
            let coord = crate::from_epsg_4326_to_3857(&coord);
            trj.push(coord);
        }
    }
    if !trj.is_empty() {
        trjs.push(trj);
    }
    trjs
}

/// Parses a string containing AIS data.
///
/// Specifically designed to the AIS Brest 2009 [dataset](https://chorochronos.datastories.org/).
/// Creates an array with EPSG 3857 `[easting, northing, time]` coordinates and time.
/// We only use fields 3 (latitude), 4(longitude), and 2(time).
/// Each day is put in a separate trajectory(`Vec`)
pub fn parse_ais(content: String) -> Vec<Vec<[f64; 3]>> {
    println!("Parsing AIS");
    let mut trjs: Vec<Vec<[f64; 3]>> = Vec::new();
    let mut trj: Vec<[f64; 3]> = Vec::new();
    let mut last_day = -1;
    let lines = content.lines();
    for line in lines {
        let mut fields = line.split(',');
        let mut coord: [f64; 3] = [f64::NAN, f64::NAN, f64::NAN];

        if let Some(time) = fields.nth(1) {
            if let Ok(time) = NaiveDateTime::parse_from_str(time, "%Y-%m-%d %H:%M:%S") {
                let day = time.num_days_from_ce();
                if day != last_day {
                    last_day = day;
                    if !trj.is_empty() {
                        trjs.push(trj);
                        trj = vec![];
                    }
                }
                coord[2] = (time.num_seconds_from_midnight() as f64) * 1000.0;
            }
        };
        if let Some(lon) = fields.next() {
            if let Ok(lon) = lon.parse::<f64>() {
                coord[1] = lon;
            }
        };

        if let Some(lat) = fields.next() {
            if let Ok(lat) = lat.parse::<f64>() {
                coord[0] = lat;
            }
        };

        if coord[0].is_finite() & coord[1].is_finite() & coord[2].is_finite() {
            let coord = crate::from_epsg_4326_to_3857(&coord);
            trj.push(coord);
        }
    }
    if !trj.is_empty() {
        trjs.push(trj);
    }
    trjs
}

/// Parses synthetic data from [https://github.com/NicklasXYZ/rtdm]
/// The original data is located in `RealtimeTrajectoryDataMining/rtdm/scripts/data`
/// It is originally in json, but we have extracted trajectories into separate csv-files before parsing.
pub fn parse_synthetic(content: String) -> Vec<Vec<[f64; 3]>> {
    //println!("Parsing Synthetic Data");
    let mut trjs: Vec<Vec<[f64; 3]>> = Vec::new();
    let mut trj: Vec<[f64; 3]> = Vec::new();
    let mut last_day = -1;
    let lines = content.lines();
    for line in lines {
        let mut fields = line.split(',');
        let mut coord: [f64; 3] = [f64::NAN, f64::NAN, f64::NAN];

        if let Some(time) = fields.next() {
            if let Ok(time) = NaiveDateTime::parse_from_str(time, "%Y-%m-%d %H:%M:%S%.f") {
                let day = time.num_days_from_ce();
                if day != last_day {
                    last_day = day;
                    if !trj.is_empty() {
                        trjs.push(trj);
                        trj = vec![];
                    }
                }
                coord[2] = (time.num_seconds_from_midnight() as f64) * 1000.0
                    + (time.timestamp_subsec_millis() as f64);
            }
        };
        if let Some(lat) = fields.next() {
            if let Ok(lat) = lat.parse::<f64>() {
                coord[0] = lat;
            }
        };

        if let Some(lon) = fields.next() {
            if let Ok(lon) = lon.parse::<f64>() {
                coord[1] = lon;
            }
        };
        if coord[0].is_finite() & coord[1].is_finite() & coord[2].is_finite() {
            let coord = crate::from_epsg_4326_to_3857(&coord);
            trj.push(coord);
        }
    }
    if !trj.is_empty() {
        trjs.push(trj);
    }
    trjs
}

/// Determines if content type and parses accordingly.
/// Specifically, the first line(casing ignored) determines content type:
/// - "geolife trajectory" => PLT
/// - "mmsi_number,time,longitude,latitude,heading,speed,cog,rot,shipcode" => AIS
/// - ",latitude,longitude,uid,anom_start" => synthetic data [info](parse_synthetic)
/// - Otherwise => GPX
pub fn parse(content: String) -> Vec<Vec<[f64; 3]>> {
    let line = content.lines().next();
    if let Some(line) = line {
        let line = line.trim().to_ascii_lowercase();
        match line.as_str() {
            "geolife trajectory" => parse_plt(content),
            "mmsi_number,time,longitude,latitude,heading,speed,cog,rot,shipcode" => {
                parse_ais(content)
            }
            ",latitude,longitude,uid,anom_start" => parse_synthetic(content),
            _ => parse_gpx(content),
        }
    } else {
        panic!("Nothing to read from stdin!")
    }
}
