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
}
