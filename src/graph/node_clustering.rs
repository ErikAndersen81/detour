use super::DetourGraph;
use crate::utility::Bbox;
use crate::STATS;

use petgraph::stable_graph::NodeIndex;
use petgraph::EdgeDirection;
use std::collections::HashSet;

pub fn spatially_cluster_nodes(graph: &mut DetourGraph) {
    let mut clustering = get_spatial_clustering(graph);
    clustering = rm_small_clusters(graph, clustering);
    let bboxs: Vec<(usize, Bbox)> = clustering
        .iter()
        .map(|cluster| calculate_mean_bbox(graph, cluster))
        .enumerate()
        .collect();
    for (idx, bbox) in bboxs {
        let start_h: i32 = (bbox.t1 / (1000.0 * 60.0 * 60.0)).floor() as i32;
        let start_m: i32 = ((bbox.t1 % (1000.0 * 60.0 * 60.0)) / (1000.0 * 60.0)).floor() as i32;
        let start_s: i32 = ((bbox.t1 % (1000.0 * 60.0)) / (1000.0)).floor() as i32;
        let end_h: i32 = (bbox.t2 / (1000.0 * 60.0 * 60.0)).floor() as i32;
        let end_m: i32 = ((bbox.t2 % (1000.0 * 60.0 * 60.0)) / (1000.0 * 60.0)).floor() as i32;
        let end_s: i32 = ((bbox.t2 % (1000.0 * 60.0)) / (1000.0)).floor() as i32;
        println!(
            "Cluster {} temporal span {}h{}m{}s - {}h{}m{}s \t representative node {:?}",
            idx, start_h, start_m, start_s, end_h, end_m, end_s, clustering[idx][0]
        );
        fit_edges_to_cluster(graph, bbox, &clustering[idx]);
        resize_bboxs(graph, bbox, &clustering[idx]);
    }
}

fn resize_bboxs(graph: &mut DetourGraph, bbox: Bbox, cluster: &[NodeIndex]) {
    for nx in cluster {
        let mut graph = graph.get_mut_graph();
        let mut old_bbox = graph[*nx];
        old_bbox.x1 = bbox.x1;
        old_bbox.y1 = bbox.y1;
        old_bbox.x2 = bbox.x2;
        old_bbox.y2 = bbox.y2;
        graph[*nx] = old_bbox;
    }
}

fn fit_edges_to_cluster(graph: &mut DetourGraph, bbox: Bbox, cluster: &[NodeIndex]) {
    let mut graph = graph.get_mut_graph();
    let mut ingoing: Vec<Vec<[f64; 3]>> = cluster
        .iter()
        .map(|nx| graph.edges_directed(*nx, EdgeDirection::Incoming))
        .flatten()
        .map(|edge| edge.weight().clone())
        .collect();
    for trj in ingoing.iter_mut() {
        let last_idx = trj.len() - 1;
        let last_point = trj[last_idx];
        if !bbox.contains_point(&last_point) {
            // Connect to the closest corner of the bbox
            let mut connect_point = bbox.nearest_corner(&last_point);
            // adjust the time slightly.
            // Perhaps we need to adjust for estimated speed here instead.
            connect_point[2] += 1.0;
            // WARNING: This point may be temporally after the bbox!!
            trj.push(connect_point);
        }
    }

    let mut outgoing: Vec<Vec<[f64; 3]>> = cluster
        .iter()
        .map(|nx| graph.edges_directed(*nx, EdgeDirection::Outgoing))
        .flatten()
        .map(|edge| edge.weight().clone())
        .collect();
    for trj in outgoing.iter_mut() {
        let first_point = trj[0];
        if !bbox.contains_point(&first_point) {
            // Connect to the closest corner of the bbox
            let mut connect_point = bbox.nearest_corner(&first_point);
            // adjust the time slightly.
            // Perhaps we need to adjust for estimated speed here instead.
            connect_point[2] -= 1.0;
            // WARNING: This point may be temporally before the bbox!!
            trj.insert(0, connect_point);
        }
    }
}

fn calculate_mean_bbox(graph: &DetourGraph, cluster: &[NodeIndex]) -> Bbox {
    let graph = graph.get_graph();
    let mut bboxs: Vec<Bbox> = cluster.iter().map(|nx| graph[*nx]).collect();
    let initial_bbox = bboxs.pop().unwrap();
    bboxs.iter().fold(initial_bbox, |acc, bbox| {
        let x1 = (acc.x1 + bbox.x1) / 2.0;
        let x2 = (acc.x2 + bbox.x2) / 2.0;
        let y1 = (acc.y1 + bbox.y1) / 2.0;
        let y2 = (acc.y2 + bbox.y2) / 2.0;
        let t1 = acc.t1.min(bbox.t1);
        let t2 = acc.t2.max(bbox.t2);
        Bbox {
            x1,
            x2,
            y1,
            y2,
            t1,
            t2,
        }
    })
}

fn rm_small_clusters(
    graph: &mut DetourGraph,
    mut clustering: Vec<Vec<NodeIndex>>,
) -> Vec<Vec<NodeIndex>> {
    let mut rm_clusters = vec![];
    for (cluster_idx, cluster) in clustering.iter().enumerate() {
        if cluster.len() < 3 {
            rm_clusters.push(cluster_idx);
            for nx in cluster.iter() {
                graph.remove_node(*nx);
            }
        }
    }
    rm_clusters.reverse();
    for cluster_idx in rm_clusters {
        clustering.remove(cluster_idx);
    }
    STATS.lock().unwrap().spatial_clusters = clustering.len();
    clustering
}

/// Cluster nodes spatially
/// Clustering criteria: If two nodes overlap spatially they belong to the same cluster.
fn get_spatial_clustering(graph: &DetourGraph) -> Vec<Vec<NodeIndex>> {
    // Create initial clustering
    let mut clusters: Vec<HashSet<NodeIndex>> = vec![];
    for nx_a in graph.node_indices() {
        for nx_b in graph.node_indices() {
            if nx_a.index() < nx_b.index() {
                let bbox_a = graph.get_node_weight(nx_a);
                let bbox_b = graph.get_node_weight(nx_b);
                if bbox_a.overlaps_spatially(&bbox_b) {
                    let mut cluster = HashSet::new();
                    cluster.insert(nx_a);
                    cluster.insert(nx_b);
                    let mut not_in_cluster = true;
                    for c in clusters.iter_mut() {
                        if !c.is_disjoint(&cluster) {
                            let union: HashSet<_> = c.union(&cluster).copied().collect();
                            *c = union;
                            not_in_cluster = false;
                        }
                    }
                    if not_in_cluster {
                        clusters.push(cluster);
                    }
                }
            }
        }
    }
    // merge clusters s.t. a single node belongs to only one cluster.
    loop {
        let mut merge: Option<(usize, usize)> = None;
        for (idx_a, cluster_a) in clusters.iter().enumerate() {
            for (idx_b, cluster_b) in clusters.iter().enumerate() {
                if idx_a >= idx_b {
                    continue;
                }
                if !cluster_a.is_disjoint(cluster_b) {
                    merge = Some((idx_a, idx_b));
                    break;
                }
            }
            if merge.is_some() {
                break;
            }
        }
        if let Some((a, b)) = merge {
            clusters[a] = clusters[a].union(&clusters[b]).copied().collect();
            clusters.remove(b);
        } else {
            break;
        }
    }
    clusters
        .into_iter()
        .map(|hs| hs.into_iter().collect::<Vec<NodeIndex>>())
        .collect::<Vec<Vec<NodeIndex>>>()
}
