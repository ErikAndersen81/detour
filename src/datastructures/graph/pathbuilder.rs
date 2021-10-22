use super::*;

pub struct PathBuilder {
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
