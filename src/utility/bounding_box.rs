use crate::CONFIG;
use serde::{Deserialize, Serialize};
use std::fmt::Display;

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Bbox {
    pub x1: f64,
    pub x2: f64,
    pub y1: f64,
    pub y2: f64,
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

    pub fn overlaps_spatially_by(&self, other: &Self) -> bool {
        let pct = 0.5;
        let area = self.area().min(other.area());
        let x_overlap = Bbox::interval_overlap(self.x1, self.x2, other.x1, other.x2);
        let y_overlap = Bbox::interval_overlap(self.y1, self.y2, other.y1, other.y2);
        let overlap = x_overlap * y_overlap;
        overlap / area >= pct
    }

    /// Determine overlap of intervals
    fn interval_overlap(a1: f64, a2: f64, b1: f64, b2: f64) -> f64 {
        let mut pts = vec![a1, a2, b1, b2];
        pts.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let inner = pts[2] - pts[1];
        let outer = pts[3] - pts[0];
        let (len_a, len_b) = (a2 - a1, b2 - b1);
        if outer >= len_a + len_b {
            // intervals do not overlap
            0.0
        } else if inner >= len_a.min(len_b) {
            // one interval contained in other interval
            len_a.min(len_b)
        } else {
            inner
        }
    }

    /// Spatial area in m²
    fn area(&self) -> f64 {
        let dx = self.x2 - self.x1;
        let dy = self.y2 - self.y1;
        dx * dy
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

    /// Returns the squared diameter of the bbox.
    pub fn get_diameter(&self) -> f64 {
        (self.x1 - self.x2).powi(2) + (self.y1 - self.y2).powi(2)
    }

    /// Splits Bbox temporally at `t`.
    ///
    /// Note that we assume a granularity of whole ms. Thus, the first Bbox will contain t and the second will contain t+1ms
    pub fn temporal_split(&self, t: f64) -> (Self, Self) {
        let bbox1 = Bbox::new(&[[self.x1, self.y1, self.t1], [self.x2, self.y2, t]]);
        let bbox2 = Bbox::new(&[[self.x1, self.y1, t + 1.0], [self.x2, self.y2, self.t2]]);
        (bbox1, bbox2)
    }

    /// Verifies if bbox satisfies the spatial constraints given in config
    pub fn verify_spatial(&self) -> bool {
        let span = (self.x2 - self.x1).max(self.y2 - self.y1);
        span < CONFIG.bbox_max_meters
    }

    /// Verifies if bbox satisfies the temporal constraints given in config
    pub fn verify_temporal(&self) -> bool {
        // Convert ms to minutes
        let span = (self.t2 - self.t1) / (1000.0 * 60.0);
        span > CONFIG.bbox_min_minutes
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

    pub fn is_single_point(&self) -> bool {
        self.t1.partial_cmp(&self.t2).unwrap().is_eq()
    }

    pub fn can_contain_trj(&self, trj: &[[f64; 3]]) -> bool {
        let mut bbox = *self;
        for point in trj {
            bbox.insert_point(point);
            if !bbox.verify_spatial() {
                return false;
            }
        }
        true
    }

    /// Expands the bbox along the trajectories.
    /// TODO: When building paths, we probably want to merge two nodes if they both can contain
    /// the entire trajectory.
    pub fn expand_along_trjs(
        &self,
        mut trjs: Vec<Vec<[f64; 3]>>,
        t1: Option<f64>,
        t2: Option<f64>,
    ) -> Bbox {
        // Determine the minimal and maximal values for t1 and t2
        // s.t. we don't break temporal monotonicity
        let mut bbox = *self;
        let mut tmp_bbox = *self;
        while tmp_bbox.verify_spatial() {
            if let Some(t1) = t1 {
                if t1 >= tmp_bbox.t1 {
                    break;
                }
            }
            if let Some(t2) = t2 {
                if t2 <= tmp_bbox.t2 {
                    break;
                }
            }
            bbox = tmp_bbox;
            if let Some((min_idx, min_bbox)) = trjs
                .iter()
                .enumerate()
                .map(|(idx, trj)| {
                    let point = trj[0];
                    let mut expanded = tmp_bbox;
                    expanded.insert_point(&point);
                    (idx, expanded)
                })
                .min_by(|a, b| {
                    a.1.get_diameter()
                        .partial_cmp(&(b.1.get_diameter()))
                        .unwrap()
                })
            {
                trjs[min_idx].remove(0);
                if trjs[min_idx].is_empty() {
                    trjs.remove(min_idx);
                }
                tmp_bbox = min_bbox;
            } else {
                break;
            }
        }
        bbox
    }
}

impl Display for Bbox {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let [x1, y1, t1] = crate::from_epsg_3857_to_4326(&[self.x1, self.y1, self.t1]);
        let [x2, y2, t2] = crate::from_epsg_3857_to_4326(&[self.x2, self.y2, self.t2]);
        writeln!(f, "{},{},{},{},{},{}", x1, y1, t1, x2, y2, t2)
    }
}
