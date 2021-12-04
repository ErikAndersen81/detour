use super::{pathbuilder::get_paths, DetourGraph};
use crate::parser::Config;

pub fn get_graph(streams: Vec<Vec<[f64; 3]>>, config: Config) -> DetourGraph {
    let mut graph = DetourGraph::new(config);
    streams
        .into_iter()
        .flat_map(|stream| get_paths(stream, &config))
        .filter(|path| path.len() > 1)
        .for_each(|path| graph.add_path(path));
    graph.merge_nodes();
    //graph.merge_edges();
    graph
}
