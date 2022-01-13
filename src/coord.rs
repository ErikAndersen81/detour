use geomorph::{coord, utm};

/// Holds coordinates in UTM projection.
///
/// Currently, information about zone is not stored and is assumed to belong to 32, i.e. the zone covering the majority of Denmark. Generally, we assume any coordinate is from Denmark.
pub struct Coord {
    pub x: f64,
    pub y: f64,
    pub t: f64,
}

impl Coord {
    /// Create a UTM projected coordinate from GPS-Coordinates '(lon, lat, time)'.
    pub fn from_gps(pt: &[f64; 3]) -> Self {
        let c: utm::Utm = coord::Coord::new(pt[1], pt[0]).into();
        Coord {
            x: c.easting,
            y: c.northing,
            t: pt[2],
        }
    }
    /// Converts to GPS '(lon, lat, time)' assuming UTM coordinates belong to zone 32 (Denmark)
    pub fn to_gps(&self) -> [f64; 3] {
        let c: coord::Coord = utm::Utm::new(self.x, self.y, true, 32, 'U', false).into();
        [c.lon, c.lat, self.t]
    }
}
