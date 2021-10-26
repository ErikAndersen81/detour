use super::Edge;
use crate::utility::Bbox;

#[derive(Clone)]
pub(crate) struct Vertex {
    bbox: Bbox,
    pub edges: Vec<Edge>,
}

impl Vertex {
    pub fn new(points: Vec<[f64; 3]>) -> Vertex {
        Vertex {
            bbox: Bbox::new(points),
            edges: Vec::new(),
        }
    }

    #[allow(dead_code)]
    pub fn is_in(&self, pt: &[f64; 3]) -> bool {
        self.bbox.is_in(pt)
    }

    #[allow(dead_code)]
    pub fn get_children(&self) -> Vec<Vertex> {
        self.edges
            .clone()
            .into_iter()
            .map(|e| *e.to)
            .collect::<Vec<Vertex>>()
    }

    pub fn recursive_get_children(&self) -> Vec<Vertex> {
        let mut vertices = self
            .edges
            .clone()
            .into_iter()
            .flat_map(|e| e.to.recursive_get_children())
            .collect::<Vec<Vertex>>();
        vertices.push(self.clone());
        vertices
    }

    pub fn edges_to_csv(&self, vertex_id: usize) {
        for (idx, edge) in self.edges.clone().into_iter().enumerate() {
            let filename: String = format!("{}-{}.csv", vertex_id, idx);
            edge.to_csv(filename).expect("Can't write trajectory");
        }
    }

    pub fn get_bbox(&self) -> Bbox {
        self.bbox
    }
}
