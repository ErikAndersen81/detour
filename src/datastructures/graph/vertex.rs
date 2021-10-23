use super::Edge;

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

    pub fn edges_to_csv(&self, filename: String, vertex_id: usize) {
        for (idx, edge) in self.edges.clone().into_iter().enumerate() {
            let mut prefix: String = format!("{}-{}-", vertex_id, idx);
            prefix.push_str(&filename);
            edge.to_csv(prefix).expect("Can't write trajectory");
        }
    }

    #[allow(dead_code)]
    pub fn print_bbox(&self) {
        println!(
            "{} {}\t{} {}\t{} {}",
            self.bbox.x1, self.bbox.x2, self.bbox.y1, self.bbox.y2, self.bbox.t1, self.bbox.t2
        );
    }

    pub fn get_bbox(&self) -> [f64; 6] {
        [
            self.bbox.x1,
            self.bbox.y1,
            self.bbox.t1,
            self.bbox.x2,
            self.bbox.y2,
            self.bbox.t2,
        ]
    }
}

#[derive(Clone, Copy)]
struct Bbox {
    x1: f64,
    x2: f64,
    y1: f64,
    y2: f64,
    t1: f64,
    t2: f64,
}

impl Bbox {
    fn new(pts: Vec<[f64; 3]>) -> Bbox {
        let mut iter = pts.into_iter();
        let pt = iter.next().unwrap();
        let mut x1: f64 = pt[0];
        let mut x2: f64 = pt[0];
        let mut y1: f64 = pt[1];
        let mut y2: f64 = pt[1];
        let mut t1: f64 = pt[2];
        let mut t2: f64 = pt[2];
        iter.for_each(|pt| {
            let [x, y, t] = pt;
            x1 = if x < x1 { x } else { x1 };
            x2 = if x > x2 { x } else { x2 };
            y1 = if y < y1 { y } else { y1 };
            y2 = if y > y2 { y } else { y2 };
            t1 = if t < t1 { t } else { t1 };
            t2 = if t > t2 { t } else { t2 };
        });
        Bbox {
            x1,
            x2,
            y1,
            y2,
            t1,
            t2,
        }
    }

    fn is_in(&self, pt: &[f64; 3]) -> bool {
        let in_x = (self.x1..self.x2).contains(&pt[0]);
        let in_y = (self.y1..self.y2).contains(&pt[1]);
        let in_t = (self.t1..self.t2).contains(&pt[2]);
        in_x && in_y && in_t
    }
}
