use std::fmt::Display;

use super::PathElement;
use crate::utility::Bbox;

#[derive(Debug, Clone)]
pub struct Path {
    path: Vec<PathElement>,
}

impl Display for Path {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        for elm in self.path.iter() {
            write!(f, "{}", elm)?;
        }
        Ok(())
    }
}

impl Path {
    pub fn new() -> Self {
        Path { path: vec![] }
    }

    pub fn verify(&self) -> bool {
        for i in 0..self.path.len() {
            if i % 2 == 1 {
                assert!(!self.path[i].is_stop());
                let trj = self.path[i].copy_trj().unwrap();
                let a = self.path[i - 1].copy_bbox().unwrap();
                let b = self.path[i + 1].copy_bbox().unwrap();
                let start = trj[0];
                let end = trj[trj.len() - 1];
                assert!(a.contains_point(&start), "start not in Stop:\n{}", self);
                assert!(b.contains_point(&end), "end not in Stop:\n{}", self);
            } else {
                assert!(self.path[i].is_stop());
                if i != self.path.len() - 1 {
                    let a = self.path[i].copy_bbox().unwrap();
                    let b = self.path[i + 2].copy_bbox().unwrap();
                    assert!(a.t2 < b.t1, "Invalid path:\n{}", self);
                }
            }
        }
        true
    }

    /// Stops are expanded along the routes they are connected to.
    /// Stops are always expanded as spatially little as possible,
    /// i.e. we select the point from either the ingoing route or the outgoing route
    /// which minimizes the expansion.
    /// A stop cannot expand `backwards` to the extend that it overlaps the previous stop.
    pub fn expand_stops(&mut self) {
        let trjs: Vec<Vec<[f64; 3]>> = self
            .path
            .iter()
            .filter(|elm| !elm.is_stop())
            .map(|elm| elm.copy_trj().unwrap())
            .collect();
        if trjs.is_empty() {
            panic!("No trajectories in path: {}", self);
        }
        let stops: Vec<Bbox> = self
            .path
            .iter()
            .filter(|elm| elm.is_stop())
            .map(|elm| elm.copy_bbox().unwrap())
            .collect();
        let last_idx = stops.len() - 1;
        for (i, bbox) in stops.into_iter().enumerate() {
            let mut tmp_bbox = bbox;
            if (i > 0) & (i < last_idx) {
                let prev_bbox = self.path[(i - 1) * 2].copy_bbox().unwrap();
                // The first bbox should already be expanded by PathBuilder.insert_point()
                // and we handle the last bbox separately after this, since it only has an ingoing trj.
                // Insert the point from ingoing or outgoing trajectory
                // that expands the bbox as spatially little as possible.
                let mut trj_in = trjs[i - 1].clone();
                trj_in.reverse();
                let mut trj_in = trj_in.into_iter().peekable();
                let mut trj_out = trjs[i].clone().into_iter().peekable();
                let [mut in_pts, mut out_pts] = [true, true];
                while in_pts | out_pts {
                    let mut bbox_in = tmp_bbox;
                    let mut bbox_out = tmp_bbox;
                    if let Some(pt) = trj_in.peek() {
                        bbox_in.insert_point(pt);
                    } else {
                        in_pts = false;
                    }
                    if let Some(pt) = trj_out.peek() {
                        bbox_out.insert_point(pt);
                    } else {
                        out_pts = false
                    }
                    let bbox_min = if bbox_in.get_diameter() < bbox_out.get_diameter() {
                        bbox_in
                    } else {
                        bbox_out
                    };
                    if bbox_min.verify_spatial() & prev_bbox.is_before(&bbox_min) {
                        tmp_bbox = bbox_min;
                    } else {
                        break;
                    }
                }
            } else if i == last_idx {
                let mut trj = trjs[i - 1].clone();
                trj.reverse();
                let prev_bbox = self.path[(i - 1) * 2].copy_bbox().unwrap();
                for pt in trj {
                    let mut expanded_bbox = tmp_bbox;
                    expanded_bbox.insert_point(&pt);
                    if expanded_bbox.verify_spatial() & prev_bbox.is_before(&expanded_bbox) {
                        tmp_bbox = expanded_bbox;
                    } else {
                        break;
                    }
                }
            }
            self.path[i * 2] = PathElement::Stop(tmp_bbox);
        }
    }

    pub fn add_points(&mut self, points: &[[f64; 3]]) {
        let last = self.path.len() - 1;
        self.path[last] = self.path[last].update_element(points);
    }

    pub fn push(&mut self, path_element: PathElement) {
        self.path.push(path_element);
    }

    pub fn is_empty(&self) -> bool {
        self.path.is_empty()
    }

    pub fn last_element(&self) -> Option<PathElement> {
        if !self.is_empty() {
            let last = self.path.len() - 1;
            Some(self.path[last].clone())
        } else {
            None
        }
    }

    /// Replace the `Bbox` last in the path.
    pub fn replace_last_bbox(&mut self, bbox: Bbox) {
        let last = self.path.len() - 1;
        assert!(
            matches!(self.path[last], PathElement::Stop(..)),
            "Cant replace Route with Stop!"
        );
        self.path[last] = PathElement::Stop(bbox);
    }

    pub fn len(&self) -> usize {
        self.path.len()
    }

    pub fn remove_first(&mut self) -> PathElement {
        self.path.remove(0)
    }

    pub fn remove_last(&mut self) -> PathElement {
        self.path.pop().unwrap()
    }

    pub fn next_trj_stop(&mut self) -> Option<(Vec<[f64; 3]>, Bbox)> {
        if self.is_empty() {
            None
        } else {
            let trj = self.path.remove(0).copy_trj().unwrap();
            let bbox = self.path.remove(0).copy_bbox().unwrap();
            Some((trj, bbox))
        }
    }
}
