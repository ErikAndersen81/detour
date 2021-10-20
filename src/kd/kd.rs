use crate::{motion_detector::MotionDetector, trajectory::Trajectory};

pub fn build_graph(stream: Vec<[f64; 3]>, mut md: MotionDetector) -> Graph {
    let mut pts: Vec<[f64; 3]> = Vec::new();
    let mut stream = stream.into_iter();
    let mut pt: [f64; 3] = stream.next().unwrap();
    // Write the initial part of the stream into pts, until
    // we know whether it belongs to a trajectory or a stop
    while matches!(md.is_moving(pt), None) {
        pts.push(pt);
        pt = stream.next().unwrap();
    }
    let mut builder = PathBuilder::new(pts);
    stream.for_each(|pt| builder.add_pt(pt, md.is_moving(pt).unwrap()));
    builder.get_path()
}

pub struct Graph {
    root: Vec<Vertex>,
}

impl Graph {
    pub fn show_vertices(&self) {
        // For now, we only show a single path of the graph
        let mut vertex = &self.root[0];
        println!(
            "{} {}\t{} {}\t{} {}",
            vertex.bbox.x1,
            vertex.bbox.x2,
            vertex.bbox.y1,
            vertex.bbox.y2,
            vertex.bbox.t1,
            vertex.bbox.t2
        );
        while !(vertex.edges.is_empty()) {
            vertex = &vertex.edges[0].to;
            println!(
                "{} {}\t{} {}\t{} {}",
                vertex.bbox.x1,
                vertex.bbox.x2,
                vertex.bbox.y1,
                vertex.bbox.y2,
                vertex.bbox.t1,
                vertex.bbox.t2
            );
        }
    }
}

#[derive(Clone)]
struct Vertex {
    bbox: Bbox,
    edges: Vec<Edge>,
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

    pub fn is_in(&self, pt: &[f64; 3]) -> bool {
        let in_x = (self.x1..self.x2).contains(&pt[0]);
        let in_y = (self.y1..self.y2).contains(&pt[1]);
        let in_t = (self.t1..self.t2).contains(&pt[2]);
        in_x && in_y && in_t
    }
}

#[derive(Clone)]
struct Edge {
    to: Vertex,
    trj: Trajectory,
}

struct PathBuilder {
    vertices: Vec<Vertex>,
    trjs: Vec<Trajectory>,
    building_vtx: bool,
    pts: Vec<[f64; 3]>,
}

impl PathBuilder {
    pub fn new(pts: Vec<[f64; 3]>) -> PathBuilder {
        let vertices: Vec<Vertex> = Vec::new();
        let trjs: Vec<Trajectory> = Vec::new();
        PathBuilder {
            vertices,
            trjs,
            building_vtx: true,
            pts,
        }
    }

    pub fn add_pt(&mut self, pt: [f64; 3], is_moving: bool) {
        if is_moving {
            self.add_to_trj(pt);
        } else {
            self.add_to_vertex(pt);
        }
    }

    fn add_to_vertex(&mut self, pt: [f64; 3]) {
        if self.building_vtx {
            self.pts.push(pt);
        } else {
            // Finish building trajectory
            self.trjs.push(Trajectory::from_array(self.pts.clone()));
            self.pts = vec![pt];
            self.building_vtx = true;
        }
    }

    fn add_to_trj(&mut self, pt: [f64; 3]) {
        if !self.building_vtx {
            self.pts.push(pt);
        } else {
            // Finish building vertex
            let bbox: Bbox = Bbox::new(self.pts.clone());
            self.vertices.push(Vertex {
                bbox,
                edges: Vec::new(),
            });
            self.pts = vec![pt];
            self.building_vtx = false;
        }
    }

    pub fn get_path(&self) -> Graph {
        let mut graph = Graph { root: Vec::new() };
        for i in 0..self.trjs.len() {
            let mut from: Vertex = self.vertices[i].clone();
            let to: Vertex = self.vertices[i + 1].clone();
            let trj: Trajectory = self.trjs[i].clone();
            let edge = Edge { to, trj };
            from.edges.push(edge);
            if i == 0 {
                graph.root.push(from)
            }
        }
        graph
    }
}
