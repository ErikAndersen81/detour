use std::fmt;

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

    fn finalize_path(&mut self) -> Result<(), PathBuilderError> {
        if self.building_vtx {
            // Finish building vertex
            let vertex = Vertex::new(self.pts.clone());
            self.vertices.push(vertex);
            Ok(())
        } else {
            Err(PathBuilderError)
        }
    }

    pub fn get_path(&mut self) -> Result<Graph, PathBuilderError> {
        self.finalize_path()?;
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
        Ok(graph)
    }
}

#[derive(Debug, Clone)]
pub struct PathBuilderError;

impl fmt::Display for PathBuilderError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Paths must end with a stop.")
    }
}

#[cfg(test)]
mod pathbuilder_test {
    use super::*;

    #[test]
    fn new_test() {
        let points = vec![[0.0, 0.0, 0.0], [0.0, 1.0, 1.0]];
        let pb = PathBuilder::new(points.clone());
        assert!(pb.pts == points)
    }

    #[test]
    fn add_point_moving_test() {
        let points = vec![[0.0, 0.0, 0.0], [0.0, 1.0, 1.0]];
        let mut pb = PathBuilder::new(points);
        pb.add_pt([1., 1., 2.], true);
        assert!(pb.pts == vec!([1., 1., 2.]));
        assert!(!pb.building_vtx);
        assert!(pb.vertices.len() == 1);
    }

    #[test]
    fn add_point_alternate_moving_test() {
        let points = vec![[0.0, 0.0, 0.0], [0.0, 1.0, 1.0]];
        let mut pb = PathBuilder::new(points);
        pb.add_pt([1., 1., 2.], true);
        pb.add_pt([2., 1., 3.], false);
        pb.add_pt([3., 1., 4.], true);
        pb.add_pt([2., 1., 5.], false);
        pb.add_pt([1., 1., 6.], true);
        pb.add_pt([1., 1., 7.], false);
        assert!(pb.vertices.len() == 3);
    }

    #[test]
    fn failing_finalize_path_test() {
        let points = vec![[0.0, 0.0, 0.0], [0.0, 1.0, 1.0]];
        let mut pb = PathBuilder::new(points);
        pb.add_pt([1., 1., 2.], true);
        pb.add_pt([2., 1., 3.], false);
        pb.add_pt([3., 1., 4.], true);
        pb.add_pt([2., 1., 5.], false);
        pb.add_pt([1., 1., 6.], true);
        let result = pb.finalize_path();
        assert!(result.is_err())
    }

    #[test]
    fn finalize_path_test() {
        let points = vec![[0.0, 0.0, 0.0], [0.0, 1.0, 1.0]];
        let mut pb = PathBuilder::new(points);
        pb.add_pt([1., 1., 2.], true);
        pb.add_pt([2., 1., 3.], false);
        pb.add_pt([3., 1., 4.], true);
        pb.add_pt([2., 1., 5.], false);
        pb.add_pt([1., 1., 6.], true);
        pb.add_pt([1., 1., 7.], false);
        let result = pb.finalize_path();
        assert!(result.is_ok());
        assert!(pb.trjs.len() == pb.vertices.len() - 1);
    }

    #[test]
    fn get_path_test() {
        let points = vec![[0.0, 0.0, 0.0], [0.0, 1.0, 1.0]];
        let mut pb = PathBuilder::new(points);
        pb.add_pt([1., 1., 2.], true);
        pb.add_pt([2., 1., 3.], false);
        pb.add_pt([3., 1., 4.], true);
        pb.add_pt([2., 1., 5.], false);
        pb.add_pt([1., 1., 6.], true);
        pb.add_pt([1., 1., 7.], false);
        let result = pb.get_path();
        assert!(result.is_ok());
        assert!(result.unwrap().root.len() == 1)
    }
}
