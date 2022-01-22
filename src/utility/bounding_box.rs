use super::{get_distance, Line};
use std::fmt::Display;

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
    pub fn new(pts: &[[f64; 3]]) -> Bbox {
        assert!(!pts.is_empty(), "Need points to instantiate bounding box!");
        let mut iter = pts.iter();
        let pt = iter.next().unwrap();
        let mut x1: f64 = pt[0];
        let mut x2: f64 = pt[0];
        let mut y1: f64 = pt[1];
        let mut y2: f64 = pt[1];
        let mut t1: f64 = pt[2];
        let mut t2: f64 = pt[2];
        for pt in iter {
            let [x, y, t] = *pt;
            x1 = x.min(x1);
            x2 = x.max(x2);
            y1 = y.min(y1);
            y2 = y.max(y2);
            t1 = t.min(t1);
            t2 = t.max(t2);
        }
        Bbox {
            x1,
            x2,
            y1,
            y2,
            t1,
            t2,
        }
    }

    pub fn overlaps(&self, other: &Self) -> bool {
        let vertical = (self.x1..=self.x2).contains(&other.x1)
            || (self.x1..=self.x2).contains(&other.x2)
            || (other.x1..=other.x2).contains(&self.x1)
            || (other.x1..=other.x2).contains(&self.x2);
        let horizontal = (self.y1..=self.y2).contains(&other.y1)
            || (self.y1..=self.y2).contains(&other.y2)
            || (other.y1..=other.y2).contains(&self.y1)
            || (other.y1..=other.y2).contains(&self.y2);
        let temporal = (self.t1..=self.t2).contains(&other.t1)
            || (self.t1..=self.t2).contains(&other.t2)
            || (other.t1..=other.t2).contains(&self.t1)
            || (other.t1..=other.t2).contains(&self.t2);
        vertical & horizontal & temporal
    }

    /// Inserts `point` and expands `Bbox` if neccesary.
    pub fn insert_point(&mut self, point: &[f64; 3]) {
        if !self.contains_point(point) {
            self.x1 = self.x1.min(point[0]);
            self.x2 = self.x2.max(point[0]);
            self.y1 = self.y1.min(point[1]);
            self.y2 = self.y2.max(point[1]);
            self.t1 = self.t1.min(point[2]);
            self.t2 = self.t2.max(point[2]);
        }
    }

    /// Determine if other 'Bbox' is temporally before 'self'.
    ///
    /// Specifically, for Bboxes x and y; if x ends before y starts, return true
    ///
    /// # Example:
    ///
    /// ``` rust
    /// let a = Bbox::new(&[[0., 0., 0.], [2., 2., 2.]]);
    /// let b = Bbox::new(&[[0., 0., 1.], [2., 2., 3.]]);
    /// let c = Bbox::new(&[[0., 0., 2.5], [2., 2., 4.]]);
    /// assert!(a.is_before(&c));
    /// assert!(!b.is_before(&c));
    pub fn is_before(&self, other: &Self) -> bool {
        self.t2 < other.t1
    }

    pub fn contains_point(&self, pt: &[f64; 3]) -> bool {
        self.is_in_spatial(pt) & self.is_in_temporal(pt)
    }

    pub fn is_in_temporal(&self, pt: &[f64; 3]) -> bool {
        (self.t1..=self.t2).contains(&pt[2])
    }

    pub fn is_in_spatial(&self, pt: &[f64; 3]) -> bool {
        let in_x = (self.x1..=self.x2).contains(&pt[0]);
        let in_y = (self.y1..=self.y2).contains(&pt[1]);
        in_x & in_y
    }

    /// Splits Bbox temporally at `t`.
    ///
    /// Note that we assume a granularity of whole ms. Thus, the first Bbox will contain t and the second will contain t+1ms
    pub fn temporal_split(&self, t: f64) -> (Self, Self) {
        let bbox1 = Bbox::new(&[[self.x1, self.y1, self.t1], [self.x2, self.y2, t]]);
        let bbox2 = Bbox::new(&[[self.x1, self.y1, t + 1.0], [self.x2, self.y2, self.t2]]);
        (bbox1, bbox2)
    }

    pub fn expand(&mut self, meters: f64, minutes: f64) {
        self.x1 -= meters;
        self.x2 += meters;
        self.y1 -= meters;
        self.y2 += meters;
        self.t1 -= minutes * 60000.;
        self.t2 += minutes * 60000.;
    }

    /// Returns the longer of width and height
    pub fn get_spatialspan(&self) -> f64 {
        (self.x2 - self.x1).max(self.y2 - self.y1)
    }

    /// Returns timespan in ms
    pub fn get_timespan(&self) -> f64 {
        self.t2 - self.t1
    }

    pub fn union(&self, other: &Self) -> Self {
        let x1 = self.x1.min(other.x1);
        let y1 = self.y1.min(other.y1);
        let t1 = self.t1.min(other.t1);
        let x2 = self.x2.max(other.x2);
        let y2 = self.y2.max(other.y2);
        let t2 = self.t2.max(other.t2);
        Bbox {
            x1,
            x2,
            y1,
            y2,
            t1,
            t2,
        }
    }
}

impl Display for Bbox {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let a = crate::Coord {
            x: self.x1,
            y: self.y1,
            t: self.t1,
        };
        let [x1, y1, t1] = a.to_gps();
        let a = crate::Coord {
            x: self.x2,
            y: self.y2,
            t: self.t2,
        };
        let [x2, y2, t2] = a.to_gps();
        writeln!(f, "{},{},{},{},{},{}", x1, y1, t1, x2, y2, t2)
    }
}
