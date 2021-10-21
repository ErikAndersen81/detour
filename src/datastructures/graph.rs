use super::Trajectory;
use crate::utility::MotionDetector;
use edge::Edge;
use vertex::Vertex;
mod edge;
mod vertex;

pub struct Graph {
    root: Vec<Vertex>,
}

impl Graph {
    pub fn new(stream: Vec<[f64; 3]>, mut md: MotionDetector) -> Graph {
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

    pub fn show_vertices(&self) {
        // For now, we only show a single path of the graph
        let mut vertex = &self.root[0];
        vertex.print_bbox();
        while !(vertex.edges.is_empty()) {
            vertex = &(vertex.edges[0].to);
            vertex.print_bbox();
        }
    }
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
            let vertex = Vertex::new(self.pts.clone());
            self.vertices.push(vertex);
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
