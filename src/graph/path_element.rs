use crate::utility::Bbox;

#[derive(Clone, Debug)]
pub enum PathElement {
    Stop(Bbox),
    Route(Vec<[f64; 3]>),
}

impl PathElement {
    pub fn is_stop(&self) -> bool {
        matches!(self, &PathElement::Stop(_))
    }

    pub fn get_bbox(&self) -> Option<Bbox> {
        if let PathElement::Stop(bbox) = self {
            Some(*bbox)
        } else {
            None
        }
    }

    pub fn get_trj(&self) -> Option<Vec<[f64; 3]>> {
        if let PathElement::Route(trj) = self {
            Some(trj.clone())
        } else {
            None
        }
    }

    pub fn push(&mut self, point: &[f64; 3]) {
        match self {
            PathElement::Stop(mut bbox) => bbox.insert_point(point),
            PathElement::Route(trj) => trj.push(*point),
        }
    }

    pub fn new_stop(points: &[[f64; 3]]) -> Self {
        PathElement::Stop(Bbox::new(points))
    }

    pub fn new_route(points: &[[f64; 3]]) -> Self {
        PathElement::Route(points.to_vec())
    }
}
