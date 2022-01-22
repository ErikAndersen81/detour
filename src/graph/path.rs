use super::PathElement;
use crate::utility::Bbox;

#[derive(Debug, Clone)]
pub struct Path {
    path: Vec<PathElement>,
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
                let a = self.path[i - 1].get_bbox().unwrap();
                let b = self.path[i + 1].get_bbox().unwrap();
                let start = trj[0];
                let end = trj[trj.len() - 1];
                assert!(a.contains_point(&start), "box does not contain start");
                assert!(b.contains_point(&end), "box does not contain end");
            } else {
                assert!(self.path[i].is_stop());
                if i != self.path.len() - 1 {
                    let a = self.path[i].get_bbox().unwrap();
                    let b = self.path[i + 2].get_bbox().unwrap();
                    assert!(
                        a.t2 < b.t1,
                        "Invalid: Path bbox a.t2 {} b.t1 {}, trj:{:?}",
                        a.t2,
                        b.t1,
                        self.path[i + 1].get_trj().unwrap()
                    );
                }
            }
        }
        true
    }

    pub fn add_points(&mut self, points: &[[f64; 3]]) {
        let last = self.path.len() - 1;
        for point in points {
            self.path[last].push(point);
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

    pub fn next_trj_stop(&mut self) -> Option<(Vec<[f64; 3]>, Bbox)> {
        if self.is_empty() {
            None
        } else {
            let trj = self.path.remove(0).get_trj().unwrap();
            let bbox = self.path.remove(0).get_bbox().unwrap();
            Some((trj, bbox))
        }
    }
}
