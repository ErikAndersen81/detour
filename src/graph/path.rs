use super::PathElement;
use std::fmt::Display;

use crate::{utility::Bbox, STATS};

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
        let last_idx = self.path.len() - 1;
        for (i, bbox) in self.path.clone().into_iter().enumerate() {
            if i == 0 {
                let trjs = vec![self.path[1].copy_trj().unwrap()];
                let t2 = self.path[2].copy_bbox().unwrap().t1;
                let bbox = bbox
                    .copy_bbox()
                    .unwrap()
                    .expand_along_trjs(trjs, None, Some(t2));
                self.path[i] = PathElement::Stop(bbox);
            } else if i == last_idx {
                let mut trj = self.path[i - 1].copy_trj().unwrap();
                trj.reverse();
                let trjs = vec![trj];
                let t1 = self.path[i - 2].copy_bbox().unwrap().t2;
                let bbox = bbox
                    .copy_bbox()
                    .unwrap()
                    .expand_along_trjs(trjs, Some(t1), None);
                self.path[i] = PathElement::Stop(bbox);
            } else if i % 2 == 0 {
                let mut trj = self.path[i - 1].copy_trj().unwrap();
                trj.reverse();
                let trj_2 = self.path[i + 1].copy_trj().unwrap();
                let trjs = vec![trj, trj_2];
                let t1 = self.path[i - 2].copy_bbox().unwrap().t2;
                let t2 = self.path[i + 2].copy_bbox().unwrap().t1;
                let bbox = bbox
                    .copy_bbox()
                    .unwrap()
                    .expand_along_trjs(trjs, Some(t1), Some(t2));
                self.path[i] = PathElement::Stop(bbox);
            }
        }
    }

    /// Tests if the trj between two consecutive bboxs can be contained in
    /// a single bbox. If it can, remove the trj and replace the
    /// two bboxs and the trj with a single bbox
    pub fn merge_nodes(&mut self) {
        let mut rm_trj_idxs = vec![];
        let last_idx = self.path.len() - 1;
        for (idx, elm) in self.path.iter().enumerate() {
            if (idx != last_idx) & (idx % 2 == 0) {
                let bbox = elm.copy_bbox().unwrap();
                let trj = self.path[idx + 1].copy_trj().unwrap();
                if bbox.can_contain_trj(&trj) {
                    rm_trj_idxs.push(idx + 1);
                }
            }
        }
        rm_trj_idxs.reverse();
        for idx in rm_trj_idxs {
            STATS.lock().unwrap().redundant_trj_removals += 1;
            let mut bbox = self.path[idx - 1].copy_bbox().unwrap();
            let trj = self.path[idx].copy_trj().unwrap();
            for point in trj {
                bbox.insert_point(&point);
            }
            self.path.remove(idx + 1);
            self.path.remove(idx);
            self.path[idx - 1] = PathElement::Stop(bbox);
        }
    }

    /// Remove consecutive single point nodes
    /// If the granularity of the measurements is too low, we assume it's noisy
    pub fn rm_single_points(&mut self) {
        let mut rm_bbox_idxs = vec![];
        let last_idx = self.path.len() - 1;
        for (idx, elm) in self.path.iter().enumerate() {
            if (idx != last_idx) & (idx % 2 == 0) {
                let bbox_1 = elm.copy_bbox().unwrap();
                let bbox_2 = self.path[idx + 2].copy_bbox().unwrap();
                if bbox_1.is_single_point() & bbox_2.is_single_point() {
                    rm_bbox_idxs.push(idx);
                }
            }
        }
        rm_bbox_idxs.reverse();
        for idx in rm_bbox_idxs {
            STATS.lock().unwrap().redundant_node_removals += 1;
            self.path.remove(idx);
            self.path.remove(idx);
            self.path.remove(idx);
        }
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
