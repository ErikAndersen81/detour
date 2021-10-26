use std::{
    fs::File,
    io::{BufWriter, Write},
};

use super::Trajectory;
use crate::{
    parser::Config,
    utility::{MotionDetector, TimeoutHandler},
};
use edge::Edge;
use pathbuilder::PathBuilder;
use vertex::Vertex;
mod edge;
mod pathbuilder;
mod vertex;

#[derive(Clone)]
pub struct Graph {
    root: Vec<Vertex>,
}

impl Graph {
    pub fn new(stream: Vec<[f64; 3]>, config: Config) -> Vec<Graph> {
        let splitted_streams = split_stream(stream, config.connection_timeout);
        let paths = splitted_streams
            .into_iter()
            .map(|stream| get_path(stream, config.clone()))
            .flatten()
            .collect::<Vec<Graph>>();
        assert!(
            !paths.is_empty(),
            "No paths could be created. Maybe connection timeout is too low."
        );
        paths
    }

    #[allow(dead_code)]
    pub fn show_vertices(&self) {
        todo!();
    }

    fn get_vertices(&self) -> Vec<Vertex> {
        // This function has an issue:
        // A vertex is returned once for every edge pointing to it. (except root vertices)
        self.root
            .clone()
            .into_iter()
            .flat_map(|v| v.recursive_get_children())
            .collect::<Vec<Vertex>>()
    }

    pub fn to_csv(&self, filename: String) -> std::io::Result<()> {
        let f = File::create(filename)?;
        let mut f: BufWriter<File> = BufWriter::new(f);
        writeln!(f, "id,x1,y1,t1,x2,y2,t2")?;
        for (idx, vertex) in self.get_vertices().into_iter().enumerate() {
            let [x1, y1, t1, x2, y2, t2] = vertex.get_bbox();
            writeln!(f, "{},{},{},{},{},{},{}", idx, x1, y1, t1, x2, y2, t2)
                .expect("Unable to write!");
            vertex.edges_to_csv(idx);
        }
        Ok(())
    }
}

fn get_path(stream: Vec<[f64; 3]>, config: Config) -> Option<Graph> {
    // This function should be called after split_stream
    let mut md = MotionDetector::new(config);
    let mut pts: Vec<[f64; 3]> = Vec::new();
    let mut stream = stream.into_iter();
    let mut builder: PathBuilder;
    // Write the initial part of the stream into pts, until
    // we know whether it belongs to a trajectory or a stop
    while let Some(point) = stream.next() {
        let is_moving = md.is_moving(point);
        pts.push(point);
        if !matches!(is_moving, None) {
            builder = PathBuilder::new(pts, !is_moving.unwrap());
            stream.for_each(|pt| builder.add_pt(pt, md.is_moving(pt).unwrap()));
            return Some(builder.get_path());
        }
    }
    None
}

fn split_stream(mut stream: Vec<[f64; 3]>, connection_timeout: f64) -> Vec<Vec<[f64; 3]>> {
    // When we build the graph we use data collected over a given period of time
    // Still, we need to handle parts separated by a connection timeout separately.
    let org_len: usize = stream.len();
    let initial_point = stream.remove(0);
    let mut timeout_handler: TimeoutHandler =
        TimeoutHandler::new(connection_timeout, &initial_point);
    let mut result: Vec<Vec<[f64; 3]>> = Vec::new();
    let mut stream_part: Vec<[f64; 3]> = vec![initial_point];
    for point in stream.into_iter() {
        if timeout_handler.is_alive(&point) {
            stream_part.push(point)
        } else {
            result.push(stream_part);
            stream_part = vec![point];
        }
    }
    result.push(stream_part);
    println!(
        "Split stream of {} points into {} streams.",
        org_len,
        result.len()
    );
    result
}

#[cfg(test)]
mod test_graph {
    use super::*;

    #[test]
    fn split_stream_test() {
        let stream = vec![
            [0., 0., 1.],
            [0., 0., 2.],
            [0., 0., 3.],
            [0., 0., 4.],
            [0., 0., 9.],
            [0., 0., 10.],
            [0., 0., 11.],
            [0., 0., 12.],
            [0., 0., 13.],
            [0., 0., 14.],
        ];
        let connection_timeout = 3.0;
        let streams = split_stream(stream, connection_timeout);
        assert_eq!(streams.len(), 2);
        assert_eq!(streams[0].len(), 4);
        assert_eq!(streams[1].len(), 6);
    }
}
