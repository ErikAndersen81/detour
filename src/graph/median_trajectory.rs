use crate::utility::trajectory::Timespan;

use super::Graph;
use petgraph::graph::EdgeIndex;
use trajectory_similarity::dissim::similarity;

pub fn get_mediod_trj(graph: &Graph, edges: &[EdgeIndex]) -> EdgeIndex {
    let mut dists = vec![vec![0.0; edges.len()]; edges.len()];
    for i in 0..edges.len() {
        for j in (i + 1)..edges.len() {
            let trj_i = graph.edge_weight(edges[i]).unwrap();
            let trj_j = graph.edge_weight(edges[j]).unwrap();
            let timespan = trj_i.1.common_timespan(&trj_j.1);
            let trj_i = trj_i.1.trim_to_timespan(timespan);
            let trj_j = trj_j.1.trim_to_timespan(timespan);
            dists[i][j] = similarity(&trj_i, &trj_j);
            dists[j][i] = dists[i][j];
        }
    }
    let idx_min = dists
        .iter()
        .map(|d| d.iter().sum::<f64>())
        .enumerate()
        .min_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(index, _)| index)
        .unwrap();
    edges[idx_min]
}
