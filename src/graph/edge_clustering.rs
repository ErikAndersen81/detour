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

type EdgeClusters = Vec<Vec<EdgeIndex>>;

pub fn merge_edges(graph: &mut StableDiGraph<Bbox, Vec<[f64; 3]>>) {
    let groups: Vec<((NodeIndex, NodeIndex), EdgeClusters)> = get_edge_groups(graph)
        .iter()
        .map(|((source, target), group)| {
            ((*source, *target), get_edge_group_clusters(graph, group))
        })
        .filter(|(_, clustering)| clustering.len() > 1)
        .collect();
    for ((source, target), clustering) in groups {
        for cluster in clustering {
            if cluster.len() < 2 {
                continue;
            }
            let mut trjs: Vec<Vec<[f64; 3]>> = cluster
                .iter()
                .map(|ex| graph.edge_weight(*ex).unwrap().clone())
                .collect();
            STATS.lock().unwrap().edge_merges += if trjs.is_empty() { 0 } else { trjs.len() - 1 };
            let trj = trjs.pop().unwrap();
            let trj: Vec<[f64; 3]> = trjs
                .into_iter()
                .fold(trj, |trj_a, trj_b| merge(&trj_a, &trj_b));
            replace_edges(graph, source, target, &cluster, trj);
        }
    }
}

fn get_edge_group_clusters(
    graph: &StableDiGraph<Bbox, Vec<[f64; 3]>>,
    group: &[EdgeIndex],
) -> EdgeClusters {
    let n = group.len();
    let mut dists = vec![vec![0f64; n]; n];
    for i in 0..group.len() {
        for j in (i + 1)..group.len() {
            let trj_a = graph.edge_weight(group[i]).unwrap();
            let trj_b = graph.edge_weight(group[j]).unwrap();
            dists[i][j] = hausdorff::similarity(trj_a, trj_b);
            dists[j][i] = dists[i][j];
        }
    }
    let clusters = Clustering::new(dists, CONFIG.max_hausdorff_meters).clusters;
    clusters
        .iter()
        .map(|c| c.iter().map(|idx| group[*idx]).collect::<Vec<EdgeIndex>>())
        .collect::<EdgeClusters>()
}

fn get_edge_groups(
    graph: &StableDiGraph<Bbox, Vec<[f64; 3]>>,
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
    groups
}

fn replace_edges(
    graph: &mut StableDiGraph<Bbox, Vec<[f64; 3]>>,
    source: NodeIndex,
    target: NodeIndex,
    group: &[EdgeIndex],
    trj: Vec<[f64; 3]>,
) {
    group.iter().for_each(|ex| {
        graph.remove_edge(*ex);
    });
    // Simplify the trajectory to avoid an excessive amount of points.
    let trj = crate::visvalingam(&trj, CONFIG.visvalingam_threshold);
    STATS.lock().unwrap().edge_merges += 1;
    graph.add_edge(source, target, trj);
}
