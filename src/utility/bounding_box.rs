use std::fmt::Display;

use super::{get_distance, Line};
#[derive(Clone, Copy, Debug)]
pub struct Bbox {
    x1: f64,
    x2: f64,
    y1: f64,
    y2: f64,
    pub t1: f64,
    pub t2: f64,
}

impl Eq for Bbox {}
impl PartialOrd for Bbox {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        match self.t1.partial_cmp(&other.t1) {
            Some(core::cmp::Ordering::Equal) => self.t2.partial_cmp(&other.t2),
            _ => self.t1.partial_cmp(&other.t1),
        }
    }
}

impl PartialEq for Bbox {
    fn eq(&self, other: &Self) -> bool {
        self.x1 == other.x1
            && self.x2 == other.x2
            && self.y1 == other.y1
            && self.y2 == other.y2
            && self.t1 == other.t1
            && self.t2 == other.t2
    }
}

impl Ord for Bbox {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        if let (Some(t1_order), Some(t2_order)) = (
            self.t1.partial_cmp(&other.t1),
            self.t2.partial_cmp(&other.t2),
        ) {
            if matches!(t1_order, std::cmp::Ordering::Equal) {
                t2_order
            } else {
                t1_order
            }
        } else {
            std::cmp::Ordering::Equal
        }
    }
}

impl Bbox {
    pub fn new(pts: Vec<[f64; 3]>) -> Bbox {
        assert!(!pts.is_empty(), "Need points to instantiate bounding box!");
        let mut iter = pts.into_iter();
        let pt = iter.next().unwrap();
        let mut x1: f64 = pt[0];
        let mut x2: f64 = pt[0];
        let mut y1: f64 = pt[1];
        let mut y2: f64 = pt[1];
        let mut t1: f64 = pt[2];
        let mut t2: f64 = pt[2];
        iter.for_each(|pt| {
            let [x, y, t] = pt;
            x1 = if x < x1 { x } else { x1 };
            x2 = if x > x2 { x } else { x2 };
            y1 = if y < y1 { y } else { y1 };
            y2 = if y > y2 { y } else { y2 };
            t1 = if t < t1 { t } else { t1 };
            t2 = if t > t2 { t } else { t2 };
        });
        Bbox {
            x1,
            x2,
            y1,
            y2,
            t1,
            t2,
        }
    }

    pub fn center(&self) -> [f64; 3] {
        let x = self.x1 + (self.x2 - self.x1) / 2.;
        let y = self.y1 + (self.y2 - self.y1) / 2.;
        let t = self.t1 + (self.t2 - self.t1) / 2.;
        [x, y, t]
    }

    pub fn overlaps(&self, other: &Self) -> bool {
        let vertical = (self.x1..self.x2).contains(&other.x1)
            || (self.x1..self.x2).contains(&other.x2)
            || (other.x1..other.x2).contains(&self.x1)
            || (other.x1..other.x2).contains(&self.x2);
        let horizontal = (self.y1..self.y2).contains(&other.y1)
            || (self.y1..self.y2).contains(&other.y2)
            || (other.y1..other.y2).contains(&self.y1)
            || (other.y1..other.y2).contains(&self.y2);
        let temporal = (self.t1..self.t2).contains(&other.t1)
            || (self.t1..self.t2).contains(&other.t2)
            || (other.t1..other.t2).contains(&self.t1)
            || (other.t1..other.t2).contains(&self.t2);
        vertical && horizontal && temporal
    }

    #[allow(dead_code)]
    pub fn get_bounding_lines(&self) -> [Line; 4] {
        let line_a = Line {
            start: [self.x1, self.y1],
            end: [self.x1, self.y2],
        };
        let line_b = Line {
            start: [self.x1, self.y2],
            end: [self.x2, self.y2],
        };
        let line_c = Line {
            start: [self.x2, self.y2],
            end: [self.x2, self.y1],
        };
        let line_d = Line {
            start: [self.x2, self.y1],
            end: [self.x1, self.y1],
        };
        [line_a, line_b, line_c, line_d]
    }

    #[allow(dead_code)]
    pub fn is_in(&self, pt: &[f64; 3]) -> bool {
        let in_t = (self.t1..=self.t2).contains(&pt[2]);
        self.is_in_spatial(pt) && in_t
    }

    #[allow(dead_code)]
    pub fn is_in_spatial(&self, pt: &[f64; 3]) -> bool {
        let in_x = (self.x1..=self.x2).contains(&pt[0]);
        let in_y = (self.y1..=self.y2).contains(&pt[1]);
        in_x && in_y
    }

    pub fn expand_bbox(&self, meters: f64, minutes: f64) -> Self {
        let latitude_min: f64 = self.y1;
        let latitude_max: f64 = self.y2;
        let x1 = self.x1 - meters_to_degrees(latitude_min, meters);
        let x2 = self.x2 + meters_to_degrees(latitude_max, meters);
        let latitude_degree_length = 360. / 40075000.0;
        let y1 = self.y1 - meters * latitude_degree_length;
        let y2 = self.y2 + meters * latitude_degree_length;
        let t1 = self.t1 - minutes * 60000.;
        let t2 = self.t2 + minutes * 60000.;
        Bbox {
            x1,
            x2,
            y1,
            y2,
            t1,
            t2,
        }
    }

    pub fn get_diameter(&self) -> f64 {
        let a: [f64; 3] = [self.x1, self.y1, self.t1];
        let b: [f64; 3] = [self.x2, self.y2, self.t2];
        get_distance(&a, &b)
    }

    #[allow(dead_code)]
    pub fn has_temporal_overlap(&self, other: &Bbox) -> bool {
        (self.t1..=self.t2).contains(&other.t1) || (self.t1..=self.t2).contains(&other.t2)
    }

    #[allow(dead_code)]
    pub fn merge(&mut self, other: &Self) {
        *self = self.union(other);
    }

    pub fn union(&self, other: &Self) -> Self {
        let x1 = if other.x1 < self.x1 {
            other.x1
        } else {
            self.x1
        };
        let x2 = if other.x2 > self.x2 {
            other.x2
        } else {
            self.x2
        };
        let y1 = if other.y1 < self.y1 {
            other.y1
        } else {
            self.y1
        };
        let y2 = if other.y2 > self.y2 {
            other.y2
        } else {
            self.y2
        };
        let t1 = if other.t1 < self.t1 {
            other.t1
        } else {
            self.t1
        };
        let t2 = if other.t2 > self.t2 {
            other.t2
        } else {
            self.t2
        };
        Bbox {
            x1,
            x2,
            y1,
            y2,
            t1,
            t2,
        }
    }

    pub fn add_point(&mut self, point: [f64; 3]) {
        let [x, y, t] = point;
        self.x1 = x.min(self.x1);
        self.x2 = x.max(self.x2);
        self.y1 = y.min(self.y1);
        self.y2 = y.max(self.y2);
        self.t1 = t.min(self.t1);
        self.t2 = t.max(self.t2);
    }
}

fn meters_to_degrees(latitude: f64, meters: f64) -> f64 {
    meters * 360. / get_longitude_degree_length(latitude)
}

fn get_longitude_degree_length(latitude: f64) -> f64 {
    let rads: f64 = latitude * 180. / std::f64::consts::PI;
    rads.cos() * 40075000.0
}

impl Display for Bbox {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(
            f,
            "{},{},{},{},{},{}",
            self.x1, self.y1, self.t1, self.x2, self.y2, self.t2
        )
    }
}
