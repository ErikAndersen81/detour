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
                let trj = self.path[i].get_trj().unwrap();
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

    pub fn next_trj_stop(&mut self) -> Option<(Vec<[f64; 3]>, Bbox)> {
        if self.is_empty() {
            None
        } else {
            let trj = self.path.remove(0).get_trj().unwrap();
            let bbox = self.path.remove(0).copy_bbox().unwrap();
            Some((trj, bbox))
        }
    }
}
