use super::DetourGraph;
use crate::utility::Bbox;
use crate::{from_epsg_3857_to_4326, STATS};
use petgraph::stable_graph::{NodeIndex, StableDiGraph};
use petgraph::visit::EdgeRef;
use petgraph::EdgeDirection;
use std::collections::HashMap;

/// Cluster nodes if they are spatially overlapping.
/// Returns clustering
pub fn spatially_cluster_nodes(graph: &mut DetourGraph) -> Vec<Vec<NodeIndex>> {
    let clustering = get_spatial_clustering(graph);
    let bboxs: Vec<(usize, Bbox)> = clustering
        .iter()
        .map(|cluster| get_minimal_bbox(graph, cluster))
        .enumerate()
        .collect();
    fn format_cluster(
        cluster: &str,
        start: &str,
        end: &str,
        dim: &str,
        coord: &str,
        size: &str,
    ) -> String {
        format!(
            "{:^9} {:<9}- {:<9} {:<20} {:<25} {:>5}",
            cluster, start, end, dim, coord, size,
        )
    }
    println!(
        "{}",
        format_cluster(
            "Cluster",
            "Start",
            "End",
            "Dimensions",
            "Coordinate",
            "Size"
        )
    );
    for (idx, bbox) in bboxs {
        let start_h: i32 = (bbox.t1 / (1000.0 * 60.0 * 60.0)).floor() as i32;
        let start_m: i32 = ((bbox.t1 % (1000.0 * 60.0 * 60.0)) / (1000.0 * 60.0)).floor() as i32;
        let start_s: i32 = ((bbox.t1 % (1000.0 * 60.0)) / (1000.0)).floor() as i32;
        let end_h: i32 = (bbox.t2 / (1000.0 * 60.0 * 60.0)).floor() as i32;
        let end_m: i32 = ((bbox.t2 % (1000.0 * 60.0 * 60.0)) / (1000.0 * 60.0)).floor() as i32;
        let end_s: i32 = ((bbox.t2 % (1000.0 * 60.0)) / (1000.0)).floor() as i32;
        let coord = from_epsg_3857_to_4326(&[bbox.x1, bbox.y1, bbox.t1]);
        let starttime = format!("{:02}:{:02}:{:02}", start_h, start_m, start_s);
        let endtime = format!("{:02}:{:02}:{:02}", end_h, end_m, end_s);
        let dimensions = format!("{:.2}m x {:.2}m", bbox.x2 - bbox.x1, bbox.y2 - bbox.y1);
        let coord = format!("{:.7},{:.7}", coord[0], coord[1]);
        println!(
            "{}",
            format_cluster(
                (idx + 1).to_string().as_str(),
                &starttime,
                &endtime,
                &dimensions,
                &coord,
                (&clustering[idx].len()).to_string().as_str()
            )
        );
        resize_bboxs(graph, bbox, &clustering[idx]);
    }
    clustering
}

/// Determines the minimal bbox s.t. endpoints of all connected trjs fits inside
fn get_minimal_bbox(graph: &mut DetourGraph, cluster: &[NodeIndex]) -> Bbox {
    let bbox = graph.get_node_bbox(cluster[0]);
    // Handle ingoing edges
    let bbox = cluster.iter().fold(bbox, |mut bbox, nx| {
        let edges = graph.edges_directed(*nx, EdgeDirection::Incoming);
        for ex in edges {
            let trj = graph.edge_trj_mut(ex);
            let last_idx = trj.len() - 1;
            let point = trj[last_idx];
            bbox.insert_point(&point);
        }
        bbox
    });
    // Handle outgoing edges
    let bbox = cluster.iter().fold(bbox, |mut bbox, nx| {
        let edges = graph.edges_directed(*nx, EdgeDirection::Outgoing);
        for ex in edges {
            let trj = graph.edge_trj_mut(ex);
            let point = trj[0];
            bbox.insert_point(&point);
        }
        bbox
    });
    bbox
}

fn resize_bboxs(graph: &mut DetourGraph, bbox: Bbox, cluster: &[NodeIndex]) {
    for nx in cluster {
        let mut bbox_new = graph.get_node_bbox(*nx);
        bbox_new.x1 = bbox.x1;
        bbox_new.y1 = bbox.y1;
        bbox_new.x2 = bbox.x2;
        bbox_new.y2 = bbox.y2;
        graph.set_node_bbox(*nx, bbox_new);
    }
}

/// Cluster nodes spatially
/// Clustering criteria: If two nodes overlap spatially they belong to the same cluster.
pub fn get_spatial_clustering(graph: &DetourGraph) -> Vec<Vec<NodeIndex>> {
    // We start by assigning each node to its own cluster
    // Each cluster is represented by a bounding box
    // If two boxes overlaps their union form the representative for the new cluster.
    let mut clustering: Vec<(Vec<NodeIndex>, Bbox)> = graph
        .node_indices()
        .map(|nx| (vec![nx], graph.get_node_bbox(nx)))
        .collect();
    let mut clusters_merged = true;
    while clusters_merged {
        clusters_merged = false;
        let mut merge = None;
        'outer: for (a, (_, bbox_a)) in clustering.iter().enumerate() {
            for (b, (_, bbox_b)) in clustering.iter().enumerate() {
                if (a < b) & (bbox_a.overlaps_spatially_by(bbox_b)) {
                    merge = Some((a, b));
                    break 'outer;
                }
            }
        }
        if let Some((a, b)) = merge {
            let (cluster_a, bbox_a) = &clustering[a];
            let (cluster_b, bbox_b) = &clustering[b];
            let bbox = bbox_a.union(bbox_b);
            let mut cluster = cluster_a.clone();
            cluster.extend(cluster_b);
            clustering.remove(b);
            clustering.remove(a);
            clustering.push((cluster, bbox));
            clusters_merged = true;
        }
    }
    clustering.iter().map(|cluster| cluster.0.clone()).collect()
}
