use geomorph::{coord, utm};

use crate::CONFIG;

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
    /// Projects using zone specified as command line argument or in the config file.
    pub fn from_gps(pt: &[f64; 3]) -> Self {
        let coord = coord::Coord::new(pt[1], pt[0]);
        let c: utm::Utm = utm::Utm::from_coord_use_fixed_zone(coord, CONFIG.utm_zone);
        Coord {
            x: c.easting,
            y: c.northing,
            t: pt[2],
        }
    }
    /// Converts to GPS '(lon, lat, time)'
    /// Assumes UTM coordinates belong to a pre specified UTM zone
    pub fn to_gps(&self) -> [f64; 3] {
        let c: coord::Coord = utm::Utm::new(
            self.x,
            self.y,
            true,
            CONFIG.utm_zone,
            CONFIG.utm_band,
            false,
        )
        .into();
        [c.lon, c.lat, self.t]
    }
}

enum Band {
    X,
    W,
    V,
    U,
    T,
    S,
    R,
    Q,
    P,
    N,
    M,
    L,
    K,
    J,
    H,
    G,
    F,
    E,
    D,
    C,
}

impl From<char> for Band {
    fn from(c: char) -> Band {
        match c {
            'X' => Band::X,
            'W' => Band::W,
            'V' => Band::V,
            'U' => Band::U,
            'T' => Band::T,
            'S' => Band::S,
            'R' => Band::R,
            'Q' => Band::Q,
            'P' => Band::P,
            'N' => Band::N,
            'M' => Band::M,
            'L' => Band::L,
            'K' => Band::K,
            'J' => Band::J,
            'H' => Band::H,
            'G' => Band::G,
            'F' => Band::F,
            'E' => Band::E,
            'D' => Band::D,
            'C' => Band::C,
            _ => panic!("Invalid Band: {}", c),
        }
    }
}

/// Return approximate width of a band in meters
fn band_width(band: Band) -> f64 {
    match band {
        Band::X => 115800.0,
        Band::C | Band::W => 206100.0,
        Band::D | Band::V => 292400.0,
        Band::E | Band::U => 373000.0,
        Band::F | Band::T => 464300.0,
        Band::G | Band::S => 511000.0,
        Band::H | Band::R => 565700.0,
        Band::J | Band::Q => 609400.0,
        Band::K | Band::P => 641300.0,
        Band::L | Band::N => 660700.0,
        Band::M => 667200.0,
    }
}
