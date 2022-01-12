use super::{pathbuilder::get_paths, DetourGraph};
use crate::config::Config;

/// Returns a detour graph when given a vector of preprocessed streams.
///
/// First, streams are converted into paths using [get_paths](get_paths).
/// Paths that does not contain at least one trajectory are removed.
/// Then, each path is inserted into a detour graph and the nodes are
/// merged. The graph is then made acyclic and finally the edges are merged.
pub fn get_graph(streams: Vec<Vec<[f64; 3]>>, config: &Config) -> DetourGraph {
    let mut graph = DetourGraph::new(config);
    streams
        .into_iter()
        .flat_map(|stream| get_paths(stream, config))
        .filter(|path| path.len() > 1)
        .for_each(|path| graph.add_path(path));
    graph.merge_nodes();
    graph.make_acyclic();
    graph.merge_edges();
    graph
}
