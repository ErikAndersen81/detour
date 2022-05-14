use std::collections::{HashMap, HashSet};

use itertools::Itertools;
use petgraph::graph::NodeIndex;

use crate::arguments::EdgeRepresentative;
use crate::graph::path_builder::split_stream_on_timeout;
use crate::utility::trajectory::Trajectory;
use crate::utility::{Bbox, Clustering};
use crate::CLUSTERINGARGS;

use super::temporal_splitting::make_temporally_monotone;
use super::{set_edges_centroid_trjs, set_edges_mediod_trjs, Graph, PathBuilderStats};

/// Constructs the graph
/// Handles spatio-temporal clustering of paths + edge clustering.
pub fn get_graph_v2(streams: Vec<Vec<[f64; 3]>>) -> Graph {
    let mut graph: Graph = Graph::new();
    let mut path_stats = PathBuilderStats::default();
    let trjs = streams
        .into_iter()
        .flat_map(|stream| split_stream_on_timeout(&stream, &mut path_stats))
        .collect::<Vec<Trajectory>>();

    // Collect endpoints and store them in a tuple with their trj index
    let endpoints: Vec<[f64; 3]> = trjs
        .iter()
        .flat_map(|trj| vec![trj[0], trj[trj.len() - 1]])
        .collect();
    let clusters = cluster_endpoints(&endpoints);
    let bboxs: Vec<Bbox> = clusters
        .iter()
        .map(|c| Bbox::new(&c.iter().map(|&idx| endpoints[idx]).collect_vec()))
        .collect();
    let mut cx_to_nx = HashMap::<usize, NodeIndex>::new();
    for (cx, bbox) in bboxs.iter().enumerate() {
        let cluster_size = clusters[cx].len() as u32;
        let nx = graph.add_node((cluster_size, *bbox));
        cx_to_nx.insert(cx, nx);
    }
    let mut nodes_to_split = vec![];
    for trj in trjs {
        let start = trj[0];
        let end = trj[trj.len() - 1];
        let mut start_nx: Option<NodeIndex> = None;
        let mut end_nx: Option<NodeIndex> = None;
        for nx in graph.node_indices() {
            if graph[nx].1.contains_point(&start) {
                start_nx = Some(nx);
            }
            if graph[nx].1.contains_point(&end) {
                end_nx = Some(nx);
            }
            if start_nx.is_some() & end_nx.is_some() {
                break;
            }
        }
        if let (Some(start_nx), Some(end_nx)) = (start_nx, end_nx) {
            if start_nx == end_nx {
                nodes_to_split.push((start_nx, start, end));
            }
            graph.add_edge(start_nx, end_nx, (1, trj));
        } else {
            panic!("Endpoints of trajectory not contained in any node!");
        }
    }
    make_temporally_monotone(&mut graph);
    match CLUSTERINGARGS.lock().unwrap().cluster_strategy {
        EdgeRepresentative::Medoid => set_edges_mediod_trjs(&mut graph),
        EdgeRepresentative::Centroid => set_edges_centroid_trjs(&mut graph),
    }

    graph
}

const MS_IN_24H: f64 = 24.0 * 60.0 * 60.0 * 1000.0;

/// Returns spatiotemporal distance
/// The distance is a weighted Euclidean distance based on `temporal_slack` which ranges from zero to `T`
/// where `T` is the number of ms in a 24 hour period.
/// The Euclidean distance between the 3D points `p` and `q` has its temporal
/// dimension weighted by `T`-`temporal_slack`.
fn spatiotemporal_distance(q: &[f64; 3], p: &[f64; 3], temporal_slack: f64) -> f64 {
    let weight = (MS_IN_24H - temporal_slack) / 1000.0;
    ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + weight * (p[2] - q[2]).powi(2)).sqrt()
}

/// Returns an agglomerative clustering of the endpoints
fn cluster_endpoints(endpoints: &[[f64; 3]]) -> Vec<HashSet<usize>> {
    let temporal_slack: f64;
    let threshold: f64;
    {
        let args = CLUSTERINGARGS.lock().unwrap();
        temporal_slack = args.temporal_slack;
        threshold = args.threshold;
    }
    let n = endpoints.len();
    let mut dists = vec![vec![0.0; n]; n];
    for i in 0..n {
        for j in i + 1..n {
            dists[i][j] = spatiotemporal_distance(&endpoints[i], &endpoints[j], temporal_slack);
            dists[j][i] = dists[i][j];
        }
    }
    let clustering = Clustering::new(dists, threshold);
    clustering.clusters
}
