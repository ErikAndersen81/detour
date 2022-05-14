use crate::utility::trajectory::merge;
use crate::utility::{clustering, Bbox};
use crate::{CONFIG, STATS};
use clustering::Clustering;
use petgraph::graph::NodeIndex;
use petgraph::prelude::EdgeIndex;
use petgraph::stable_graph::StableDiGraph;
use petgraph::visit::EdgeRef;
use petgraph::EdgeDirection;
use std::collections::HashMap;
use trajectory_similarity::hausdorff;

use super::{get_mediod_trj, Graph};

type EdgeClusters = Vec<Vec<EdgeIndex>>;

/// Cluster the edges using Hausdorff similarity and insert a representative(mediod)
/// trajectory for each cluster instead of the all of the original trajectories.
pub fn set_edges_mediod_trjs(graph: &mut Graph) {
    let groups: Vec<((NodeIndex, NodeIndex), EdgeClusters)> = get_edge_groups(graph)
        .iter()
        .map(|((source, target), group)| {
            ((*source, *target), get_edge_group_clusters(graph, group))
        })
        .collect();
    for ((source, target), clustering) in groups {
        for cluster in clustering {
            let weight = cluster.len() as u32;
            let trj: Vec<[f64; 3]>;
            {
                let graph: &Graph = graph;
                let mediod = get_mediod_trj(graph, &cluster);
                let (_, mediod) = graph.edge_weight(mediod).unwrap();
                trj = mediod.clone();
            }
            replace_edges(graph, source, target, &cluster, (weight, trj));
        }
    }
}

/// Merges the edges using interpolation.
/// I.e. the average/mean position of the moving object at a given time
/// according to the two trajectories.
pub fn set_edges_centroid_trjs(graph: &mut Graph) {
    let groups: Vec<((NodeIndex, NodeIndex), EdgeClusters)> = get_edge_groups(graph)
        .iter()
        .map(|((source, target), group)| {
            ((*source, *target), get_edge_group_clusters(graph, group))
        })
        .collect();
    for ((source, target), clustering) in groups {
        for cluster in clustering {
            let mut trjs: Vec<(u32, Vec<[f64; 3]>)> = cluster
                .iter()
                .map(|ex| graph.edge_weight(*ex).unwrap().clone())
                .collect();
            STATS.lock().unwrap().edge_merges += if trjs.is_empty() { 0 } else { trjs.len() - 1 };
            let trj = trjs.pop().unwrap();
            let trj: (u32, Vec<[f64; 3]>) = trjs.into_iter().fold(trj, |trj_a, trj_b| {
                (trj_a.0 + trj_b.0, merge(&trj_a.1, &trj_b.1))
            });
            replace_edges(graph, source, target, &cluster, trj);
        }
    }
}

fn get_edge_group_clusters(
    graph: &StableDiGraph<(u32, Bbox), (u32, Vec<[f64; 3]>)>,
    group: &[EdgeIndex],
) -> EdgeClusters {
    let n = group.len();
    let mut dists = vec![vec![0f64; n]; n];
    for i in 0..group.len() {
        for j in (i + 1)..group.len() {
            let trj_a = graph.edge_weight(group[i]).unwrap();
            let trj_b = graph.edge_weight(group[j]).unwrap();
            dists[i][j] = hausdorff::similarity(&trj_a.1, &trj_b.1);
            dists[j][i] = dists[i][j];
        }
    }
    let clusters = Clustering::new(dists, CONFIG.max_hausdorff_meters).clusters;
    let clusters = clusters
        .iter()
        .map(|c| c.iter().map(|idx| group[*idx]).collect::<Vec<EdgeIndex>>())
        .collect::<EdgeClusters>();
    clusters
}

fn get_edge_groups(
    graph: &StableDiGraph<(u32, Bbox), (u32, Vec<[f64; 3]>)>,
) -> HashMap<(NodeIndex, NodeIndex), Vec<EdgeIndex>> {
    let mut groups: HashMap<(NodeIndex, NodeIndex), Vec<EdgeIndex>> = HashMap::new();
    for source in graph.node_indices() {
        graph
            .edges_directed(source, EdgeDirection::Outgoing)
            .map(|ex| (ex.id(), ex.target()))
            .for_each(|(ex, target)| {
                if let Some(lst) = groups.get_mut(&(source, target)) {
                    lst.push(ex);
                } else {
                    groups.insert((source, target), vec![ex]);
                }
            });
    }
    let mut total = 0;
    for group in groups.clone() {
        total += group.1.len() as u32;
    }
    groups
}

fn replace_edges(
    graph: &mut StableDiGraph<(u32, Bbox), (u32, Vec<[f64; 3]>)>,
    source: NodeIndex,
    target: NodeIndex,
    group: &[EdgeIndex],
    trj: (u32, Vec<[f64; 3]>),
) {
    group.iter().for_each(|ex| {
        graph.remove_edge(*ex);
    });
    // Simplify the trajectory to avoid an excessive amount of points.
    let simplified = crate::visvalingam(&trj.1, CONFIG.visvalingam_threshold);
    graph.add_edge(source, target, (trj.0, simplified));
}
