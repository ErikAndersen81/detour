use std::fmt::Display;

use crate::utility::Bbox;

#[derive(Clone, Debug)]
pub enum PathElement {
    Stop(Bbox),
    Route(Vec<[f64; 3]>),
}

impl Display for PathElement {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PathElement::Stop(bbox) => writeln!(f, "{:?}", bbox),
            PathElement::Route(trj) => {
                if !trj.is_empty() {
                    writeln!(
                        f,
                        "{:?} -> {:?} ({})",
                        trj[0],
                        trj[trj.len() - 1],
                        trj.len()
                    )
                } else {
                    writeln!(f, "empty route")
                }
            }
        }
    }
}

impl PathElement {
    pub fn is_stop(&self) -> bool {
        matches!(self, &PathElement::Stop(_))
    }

    pub fn copy_bbox(&self) -> Option<Bbox> {
        if let PathElement::Stop(bbox) = self {
            Some(*bbox)
        } else {
            None
        }
    }

    pub fn copy_trj(&self) -> Option<Vec<[f64; 3]>> {
        if let PathElement::Route(trj) = self {
            Some(trj.clone())
        } else {
            None
        }
    }

    pub fn set_bbox(&mut self, new_bbox: Bbox) {
        if let PathElement::Stop(bbox) = self {
            *bbox = new_bbox;
        } else {
            panic!("Can't set bbox on `PathElement::Route`.")
        }
    }

    pub fn update_element(&self, points: &[[f64; 3]]) -> Self {
        match self {
            PathElement::Stop(bbox) => {
                let new_bbox = Bbox::new(points);
                let new_bbox = new_bbox.union(bbox);
                PathElement::Stop(new_bbox)
            }
            PathElement::Route(trj) => {
                let mut trj = trj.clone();
                let new_trj = points.to_vec();
                trj.extend(new_trj);
                PathElement::Route(trj)
            }
        }
    }

    pub fn new_stop(points: &[[f64; 3]]) -> Self {
        PathElement::Stop(Bbox::new(points))
    }

    pub fn new_route(points: &[[f64; 3]]) -> Self {
        PathElement::Route(points.to_vec())
    }
}
