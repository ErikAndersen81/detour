use super::pathbuilder::Path;
use super::{pathbuilder::get_paths, DetourGraph};
use crate::parser::Config;

pub fn get_graph(streams: Vec<Vec<[f64; 3]>>, config: Config) -> DetourGraph {
    let paths: Vec<Path> = streams
        .into_iter()
        .flat_map(|stream| get_paths(stream, &config))
        .collect();
    assert!(!paths.is_empty(), "No paths provided!");
    let mut graph = DetourGraph::new(config);
    paths.into_iter().for_each(|path| graph.add_path(path));
    graph.to_csv().expect("Could not write output.");
    graph.merge_edges();
    graph
}
