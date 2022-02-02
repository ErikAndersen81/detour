use std::f64::consts::{E, PI};

/// Constants for WSG 84 / Pseudo-Mercator (EPSG CRS Code 3857)
const RADIUS: f64 = 6378137.0;
const RAD: f64 = PI / 180.0;
const DEG: f64 = 180.0 / PI;

/// Convert a coordinate of `[lat, lon, time]` in EPSG 4326 to EPSG 3857 `[easting, northing, time]`
///
/// Panics if the latitude is above 88 degrees.
pub fn from_epsg_4326_to_3857(pt: &[f64; 3]) -> [f64; 3] {
    if pt[0] > 88.0 {
        panic!("Cannot convert latitude above 88 deg to EPSG:3857")
    }
    let lat = pt[0] * RAD;
    let lon = pt[1] * RAD;
    let easting = RADIUS * lon;
    let northing = RADIUS * ((PI / 4.0 + lat / 2.0).tan().abs()).ln();
    [easting, northing, pt[2]]
}

/// Convert a coordinate of EPSG 3857 `[easting, northing, time]` to  `[lat, lon, time]` in EPSG 4326
pub fn from_epsg_3857_to_4326(pt: &[f64; 3]) -> [f64; 3] {
    let d = -pt[1] / RADIUS;
    let lat = PI / 2.0 - 2.0 * (E.powf(d)).atan();
    let lon = pt[0] / RADIUS;
    [lat * DEG, lon * DEG, pt[2]]
}
