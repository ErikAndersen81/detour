use std::{
    fs::File,
    io::{BufWriter, Write},
};

use super::Trajectory;
use crate::utility::MotionDetector;
use edge::Edge;
use pathbuilder::PathBuilder;
use vertex::Vertex;
mod edge;
mod pathbuilder;
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

    fn get_vertices(&self) -> Vec<Vertex> {
        let mut vertices: Vec<Vertex> = self.root.clone();
        let mut n_vertices: usize = vertices.len();
        loop {
            vertices = vertices
                .iter()
                .flat_map(|vertex| vertex.get_children())
                .collect();
            if vertices.len() == n_vertices {
                break;
            }
            n_vertices = vertices.len();
        }
        vertices
    }

    pub fn to_csv(&self, filename: String) -> std::io::Result<()> {
        let f = File::create(filename.clone())?;
        let mut f: BufWriter<File> = BufWriter::new(f);
        writeln!(f, "id,x1,y1,t1,x2,y2,t2")?;
        for (idx, vertex) in self.get_vertices().into_iter().enumerate() {
            let [x1, y1, t1, x2, y2, t2] = vertex.get_bbox();
            writeln!(f, "{},{},{},{},{},{},{}", idx, x1, y1, t1, x2, y2, t2)
                .expect("Unable to write!");
            vertex.edges_to_csv(filename.clone(), idx);
        }
        Ok(())
    }
}
