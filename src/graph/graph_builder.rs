use crate::graph::spatially_cluster_nodes;

use super::{merge_edges, path_builder::get_paths, DetourGraph, PathBuilderStats};

/// Returns a detour graph when given a vector of preprocessed streams.
///
/// First, streams are converted into paths using [get_paths](get_paths).
/// Paths that does not contain at least one trajectory are removed.
/// Then, each path is inserted into a detour graph and the nodes are
/// merged. The graph is then made acyclic and finally the edges are merged.
pub fn get_graph(streams: Vec<Vec<[f64; 3]>>) -> DetourGraph {
    let mut graph = DetourGraph::new();
    let mut path_stats = PathBuilderStats::default();
    streams
        .into_iter()
        .flat_map(|stream| get_paths(stream, &mut path_stats))
        .filter(|path| path.len() > 1)
        .for_each(|path| graph.add_path(path));
    println!("\tremoving sparsely visited places");
    spatially_cluster_nodes(&mut graph);
    println!("\tmerging nodes");
    graph.merge_nodes();
    println!("\tmerging edges");
    merge_edges(graph.get_mut_graph());
    println!("Path builder stats:\n{}", path_stats);
    graph
}
