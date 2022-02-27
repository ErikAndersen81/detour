use super::DetourGraph;
use crate::utility::Bbox;
use crate::{from_epsg_3857_to_4326, STATS};
use petgraph::stable_graph::{NodeIndex, StableDiGraph};
use petgraph::visit::EdgeRef;
use petgraph::EdgeDirection;
use std::collections::HashMap;

/// Cluster nodes if they are spatially overlapping.
/// Clusters of size < 3 are put in an `outlier` graph
/// Outlier nodes and related edges are removed from `graph`
/// Returns clustering and outlier graph
pub fn spatially_cluster_nodes(
    graph: &mut DetourGraph,
) -> (
    Vec<Vec<NodeIndex>>,
    StableDiGraph<(usize, Bbox), Vec<[f64; 3]>>,
) {
    let mut clustering = get_spatial_clustering(graph);
    let outliers = remove_outliers(graph, &mut clustering);
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
    (clustering, outliers)
}

/// Determines the minimal bbox s.t. endpoints of all connected trjs fits inside
fn get_minimal_bbox(graph: &mut DetourGraph, cluster: &[NodeIndex]) -> Bbox {
    println!("endpoints\n");
    let bbox = graph.get_node_weight(cluster[0]);
    // Handle ingoing edges
    let bbox = cluster.iter().fold(bbox, |mut bbox, nx| {
        let edges = graph.edges_directed(*nx, EdgeDirection::Incoming);
        for ex in edges {
            let trj = graph.edge_weight_mut(ex);
            let last_idx = trj.len() - 1;
            let point = trj[last_idx];
            println!(
                "nan,{},{},nan,{},{},nan",
                point[0], point[1], point[0], point[1]
            );
            bbox.insert_point(&point);
        }
        bbox
    });
    println!();
    // Handle outgoing edges
    let bbox = cluster.iter().fold(bbox, |mut bbox, nx| {
        let edges = graph.edges_directed(*nx, EdgeDirection::Outgoing);
        for ex in edges {
            let trj = graph.edge_weight_mut(ex);
            let point = trj[0];
            println!(
                "nan,{},{},nan,{},{},nan",
                point[0], point[1], point[0], point[1]
            );
            bbox.insert_point(&point);
        }
        bbox
    });
    bbox
}

fn resize_bboxs(graph: &mut DetourGraph, bbox: Bbox, cluster: &[NodeIndex]) {
    for nx in cluster {
        let mut bbox_new = graph.get_node_weight(*nx);
        bbox_new.x1 = bbox.x1;
        bbox_new.y1 = bbox.y1;
        bbox_new.x2 = bbox.x2;
        bbox_new.y2 = bbox.y2;
        graph.set_node_weight(*nx, bbox_new);
    }
}

/// Constructs a graph of non-frequently visitied nodes, i.e. small clusters of size < 3.
/// Removes the nodes in the small clusters from `graph` and stores them and their edges
/// in an outlier list.
/// Removes the small clusters from `clustering`.
/// Returns a list of 'outlier'-nodes and edges.
fn remove_outliers(
    graph: &mut DetourGraph,
    clustering: &mut Vec<Vec<NodeIndex>>,
) -> StableDiGraph<(usize, Bbox), Vec<[f64; 3]>> {
    // Initially, identify less frequently visited nodes and remove them from `clustering`
    let mut rm_clusters = vec![];
    for (cluster_idx, cluster) in clustering.iter().enumerate() {
        if cluster.len() < 3 {
            rm_clusters.push(cluster_idx);
        }
    }
    rm_clusters.reverse();
    let mut outlier_nodes = vec![];
    for cluster_idx in rm_clusters {
        for nx in clustering[cluster_idx].iter() {
            outlier_nodes.push(*nx);
        }
        clustering.remove(cluster_idx);
    }

    // Construct the outlier graph
    // we include some nodes that are not outliers
    // s.t. we can store the edges leading to/from the outlier nodes.
    let mut outlier_graph: StableDiGraph<(usize, Bbox), Vec<[f64; 3]>> = StableDiGraph::new();
    // We use a mapping to keep track of inserted nodes and avoid duplicate inserts.
    let mut nx_map: HashMap<NodeIndex, NodeIndex> = HashMap::new();
    let old_graph = graph.get_graph();
    for nx in outlier_nodes.iter() {
        let nx = if !nx_map.contains_key(nx) {
            let bbox = old_graph[*nx];
            let new_nx = outlier_graph.add_node((0, bbox));
            nx_map.insert(*nx, new_nx);
            new_nx
        } else {
            *nx_map.get(nx).unwrap()
        };
        // Insert incoming edges
        for edge in old_graph.edges_directed(nx, EdgeDirection::Incoming) {
            let trj = edge.weight().clone();
            let source = if !nx_map.contains_key(&edge.source()) {
                let bbox = old_graph[edge.source()];
                let new_nx = outlier_graph.add_node((1, bbox));
                nx_map.insert(edge.source(), new_nx);
                new_nx
            } else {
                *nx_map.get(&edge.source()).unwrap()
            };
            outlier_graph.add_edge(source, nx, trj);
        }
        // Insert outgoing edges
        for edge in old_graph.edges_directed(nx, EdgeDirection::Outgoing) {
            let trj = edge.weight().clone();
            let target = if !nx_map.contains_key(&edge.target()) {
                let bbox = old_graph[edge.target()];
                let new_nx = outlier_graph.add_node((1, bbox));
                nx_map.insert(edge.target(), new_nx);
                new_nx
            } else {
                *nx_map.get(&edge.target()).unwrap()
            };
            outlier_graph.add_edge(nx, target, trj);
        }
    }

    // Remove outliers from the old graph.
    for nx in outlier_nodes {
        graph.remove_node(nx);
    }
    // Update STATS. TODO: Should also include amount of outliers.
    STATS.lock().unwrap().spatial_clusters = clustering.len();
    outlier_graph
}

/// Cluster nodes spatially
/// Clustering criteria: If two nodes overlap spatially they belong to the same cluster.
pub fn get_spatial_clustering(graph: &DetourGraph) -> Vec<Vec<NodeIndex>> {
    // We start by assigning each node to its own cluster
    // Each cluster is represented by a bounding box
    // If two boxes overlaps their union form the representative for the new cluster.
    let mut clustering: Vec<(Vec<NodeIndex>, Bbox)> = graph
        .node_indices()
        .map(|nx| (vec![nx], graph.get_node_weight(nx)))
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
