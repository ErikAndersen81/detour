use std::fmt;

use super::*;

pub struct PathBuilder {
    vertices: Vec<Vertex>,
    trjs: Vec<Trajectory>,
    building_trajectory: bool,
    pts: Vec<[f64; 3]>,
}

impl PathBuilder {
    pub fn new(pts: Vec<[f64; 3]>, building_trajectory: bool) -> PathBuilder {
        let mut vertices: Vec<Vertex> = Vec::new();
        let trjs: Vec<Trajectory> = Vec::new();
        if building_trajectory {
            println!("Warning: Path does not start with a vertex. Inserting degenerated vertex.");
            let point = pts[0];
            let vertex = Vertex::new(vec![point]);
            vertices.push(vertex);
        }
        PathBuilder {
            vertices,
            trjs,
            building_trajectory,
            pts,
        }
    }

    pub fn add_pt(&mut self, pt: [f64; 3], is_moving: bool) {
        self.pts.push(pt);
        if is_moving {
            self.add_to_trj(pt);
        } else {
            self.add_to_vertex(pt);
        }
    }

    fn add_to_vertex(&mut self, pt: [f64; 3]) {
        if self.building_trajectory {
            // Finish building trajectory
            self.trjs.push(Trajectory::from_array(self.pts.clone()));
            self.pts = vec![pt];
            self.building_trajectory = false;
        }
    }

    fn add_to_trj(&mut self, pt: [f64; 3]) {
        if !self.building_trajectory {
            // Finish building vertex
            self.vertices.push(Vertex::new(self.pts.clone()));
            self.pts = vec![pt];
            self.building_trajectory = true;
        }
    }

    fn finalize_path(&mut self) -> Result<(), PathBuilderError> {
        if !self.building_trajectory {
            // Finish building vertex
            self.vertices.push(Vertex::new(self.pts.clone()));
            Ok(())
        } else {
            // We shouldn't end in the middle of a trajectory
            // so we construct a degenerate Bbox of the
            // last point, use the rest for a trajectory
            // and return an Error
            if self.pts.len() > 2 {
                let point = self.pts[self.pts.len() - 1];
                let vertex = Vertex::new(vec![point]);
                self.vertices.push(vertex);
                self.trjs.push(Trajectory::from_array(self.pts.clone()));
            }
            Err(PathBuilderError)
        }
    }

    pub fn get_path(&mut self) -> Graph {
        self.finalize_path()
            .unwrap_or_else(|err| println!("{}", err));
        let mut graph: Graph = Graph { root: Vec::new() };
        let root = self.vertices[0].clone();
        graph.root.push(root);
        let mut edges: Vec<Edge> = Vec::new();
        for i in 0..self.trjs.len() {
            let to = Box::new(self.vertices[i + 1].clone());
            let trj: Trajectory = self.trjs[i].to_owned();
            let edge: Edge = Edge { to, trj };
            edges.push(edge);
        }
        let mut curr_vertex = &mut graph.root[0];
        for edge in edges {
            curr_vertex.edges = vec![edge.clone()];
            curr_vertex = &mut curr_vertex.edges[0].to;
        }
        graph
    }
}

#[derive(Debug, Clone)]
pub struct PathBuilderError;

impl fmt::Display for PathBuilderError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Warning: Path ended during the construction of a trajectory. Degenerate vertex inserted.")
    }
}

#[cfg(test)]
mod pathbuilder_test {
    use super::*;

    #[test]
    fn new_test() {
        let points = vec![[0.0, 0.0, 0.0], [0.0, 1.0, 1.0]];
        let pb = PathBuilder::new(points.clone(), true);
        assert!(pb.pts == points)
    }

    #[test]
    fn add_point_moving_test() {
        let points = vec![[0.0, 0.0, 0.0], [0.0, 1.0, 1.0]];
        let mut pb = PathBuilder::new(points, false);
        pb.add_pt([1., 1., 2.], true);
        assert!(pb.pts == vec!([1., 1., 2.]));
        assert!(pb.building_trajectory);
        assert!(pb.vertices.len() == 1);
    }

    #[test]
    fn add_point_alternate_moving_test() {
        let points = vec![[0.0, 0.0, 0.0], [0.0, 1.0, 1.0]];
        let mut pb = PathBuilder::new(points, true);
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
        let mut pb = PathBuilder::new(points, true);
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
        let mut pb = PathBuilder::new(points, true);
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
        let mut pb = PathBuilder::new(points, true);
        pb.add_pt([1., 1., 2.], true);
        pb.add_pt([2., 1., 3.], false);
        pb.add_pt([3., 1., 4.], true);
        pb.add_pt([2., 1., 5.], false);
        pb.add_pt([1., 1., 6.], true);
        pb.add_pt([1., 1., 7.], false);
        let path = pb.get_path();
        assert_eq!(path.root.len(), 1);
        assert_eq!(path.root[0].edges.len(), 1);
        assert_eq!(path.root[0].edges[0].to.edges.len(), 1);
        assert_eq!(path.get_vertices().len(), 4);
    }
}
