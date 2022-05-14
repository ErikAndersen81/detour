use super::set_edges_mediod_trjs;
use super::{path_builder::get_paths, set_edges_centroid_trjs, DetourGraph, PathBuilderStats};
use crate::graph::node_clustering::spatially_cluster_nodes;
use crate::utility::Bbox;
use itertools::Itertools;
use petgraph::graph::NodeIndex;
use petgraph::stable_graph::StableDiGraph;
use petgraph::visit::EdgeRef;
use petgraph::EdgeDirection;
use std::collections::HashSet;

type Graph = StableDiGraph<(u32, Bbox), (u32, Vec<[f64; 3]>)>;

/// Constructs the graph
/// Handles spatial node clustering, edge clustering and detecting less frequently visited places.
pub fn get_graph(streams: Vec<Vec<[f64; 3]>>) -> DetourGraph {
    let mut graph = DetourGraph::new();
    let mut path_stats = PathBuilderStats::default();
    streams
        .into_iter()
        .flat_map(|stream| get_paths(stream, &mut path_stats))
        .filter(|path| path.len() > 1)
        .for_each(|path| graph.add_path(path));
    merge_nodes(&mut graph);
    //merge_edges(graph.get_mut_graph());
    set_edges_mediod_trjs(graph.get_mut_graph());
    graph
}

fn merge_nodes(graph: &mut DetourGraph) {
    let node_clustering = spatially_cluster_nodes(graph);
    // calculate a cluster representative for each cluster
    let representatives: Vec<(u32, Bbox)> = node_clustering
        .iter()
        .map(|cluster| {
            let (weight, bbox) = (cluster.len() as u32, graph.get_graph()[cluster[0]].1);
            let bbox = Bbox {
                x1: bbox.x1,
                x2: bbox.x2,
                y1: bbox.y1,
                y2: bbox.y2,
                t1: 0.0,                          // Time starts at 0.0
                t2: (24 * 60 * 60 * 1000) as f64, // We always have 24*60*60*1000 ms in a 24 hour span
            };
            (weight, bbox)
        })
        .collect();

    // Construct a new graph with cluster representatives.
    let mut new_graph: Graph = StableDiGraph::new();
    let representatives: Vec<NodeIndex> = representatives
        .iter()
        .map(|bbox| new_graph.add_node(*bbox))
        .collect_vec();

    // Add edges to the cluster representatives
    graph.edge_weights().into_iter().for_each(|edge| {
        let start_point = edge.1[0];
        let end_point = edge.1[edge.1.len() - 1];
        let mut start_node = None;
        let mut end_node = None;
        for nx in representatives.iter() {
            if new_graph[*nx].1.contains_point(&start_point) {
                start_node = Some(*nx);
                break;
            }
        }
        for nx in representatives.iter() {
            if new_graph[*nx].1.contains_point(&end_point) {
                end_node = Some(*nx);
                break;
            }
        }
        if let (Some(a), Some(b)) = (start_node, end_node) {
            new_graph.add_edge(a, b, (1, edge.1.clone()));
        }
    });

    // We could try to move merge edges here instead of after merge_node
    //merge_edges(&mut new_graph);

    // Try to find suitable temporal split values for each node cluster
    // based on start and end times of trajectories connecting them.
    let mut required_splits = vec![];
    for nx in new_graph.node_indices() {
        // calculate required splits
        let mut edges: Vec<(u32, Vec<[f64; 3]>)> = new_graph
            .edges_directed(nx, EdgeDirection::Incoming)
            .map(|edge| edge.weight().clone())
            .collect();
        let mut outgoing: Vec<(u32, Vec<[f64; 3]>)> = new_graph
            .edges_directed(nx, EdgeDirection::Outgoing)
            .map(|edge| edge.weight().clone())
            .collect();
        edges.append(&mut outgoing);
        let splits = get_temporal_splits(edges);
        required_splits.push((nx, splits));
    }
    for (split_node, splits) in required_splits {
        // Split bbox at splits and add the new nodes
        let nodes: Vec<NodeIndex> = split_bbox(new_graph[split_node].1, &splits)
            .into_iter()
            .map(|bbox| new_graph.add_node((new_graph[split_node].0, bbox)))
            .collect();
        reassign_edges(&mut new_graph, split_node, &nodes, EdgeDirection::Outgoing);
        reassign_edges(&mut new_graph, split_node, &nodes, EdgeDirection::Incoming);
        // remove new nodes that has no edges
        let no_edge_nodes: Vec<&NodeIndex> = nodes
            .iter()
            .filter(|nx| {
                (new_graph
                    .edges_directed(**nx, EdgeDirection::Incoming)
                    .count()
                    == 0)
                    & (new_graph
                        .edges_directed(**nx, EdgeDirection::Outgoing)
                        .count()
                        == 0)
            })
            .collect();
        for nx in no_edge_nodes.iter() {
            new_graph.remove_node(**nx);
        }
    }

    // remove the representative nodes
    for nx in representatives {
        new_graph.remove_node(nx);
    }

    // Find the root nodes
    let mut root_nodes = vec![];
    for nx in new_graph.node_indices() {
        if new_graph
            .edges_directed(nx, EdgeDirection::Incoming)
            .count()
            == 0
        {
            root_nodes.push(nx);
        }
    }

    graph.set_graph(new_graph, root_nodes);
}

fn reassign_edges(
    graph: &mut Graph,
    split_node: NodeIndex,
    nodes: &[NodeIndex],
    direction: EdgeDirection,
) {
    let mut rm_edge_idx = vec![];
    let edges = match direction {
        EdgeDirection::Outgoing => {
            let mut edges = vec![];
            for edge in graph.edges_directed(split_node, direction) {
                rm_edge_idx.push(edge.id());
                let trj = edge.weight().clone();
                let start_point = trj.1[0];
                let target = if edge.target() != split_node {
                    edge.target()
                } else {
                    // The edge goes from split_node to split node, so we need to handle this
                    let mut target = None;
                    let end_point = trj.1[trj.1.len() - 1];
                    for node in nodes {
                        if graph[*node].1.is_in_temporal(&end_point) {
                            target = Some(*node);
                        }
                    }
                    if let Some(target) = target {
                        target
                    } else {
                        panic!("No matching target node for trj!");
                    }
                };
                for node in nodes {
                    if graph[*node].1.is_in_temporal(&start_point) {
                        edges.push((*node, target, trj));
                        break;
                    }
                }
            }
            edges
        }
        EdgeDirection::Incoming => {
            let mut edges = vec![];
            for edge in graph.edges_directed(split_node, direction) {
                rm_edge_idx.push(edge.id());
                let trj = edge.weight().clone();
                let end_point = trj.1[trj.1.len() - 1];
                let source = if edge.source() != split_node {
                    edge.source()
                } else {
                    // edge from split_node to split_node
                    let mut source = None;
                    let start_point = trj.1[0];
                    for node in nodes {
                        if graph[*node].1.is_in_temporal(&start_point) {
                            source = Some(*node);
                        }
                    }
                    if let Some(source) = source {
                        source
                    } else {
                        panic!("No matching source node for trj!");
                    }
                };
                for node in nodes {
                    if graph[*node].1.is_in_temporal(&end_point) {
                        edges.push((source, *node, trj));
                        break;
                    }
                }
            }
            edges
        }
    };
    for (source, target, trj) in edges.into_iter() {
        graph.add_edge(source, target, trj);
    }
    for ex in rm_edge_idx {
        graph.remove_edge(ex);
    }
}

fn split_bbox(bbox: Bbox, splits: &[f64]) -> Vec<Bbox> {
    let mut boxes = vec![];
    let mut bbox = bbox;
    for t in splits {
        let (box1, box2) = bbox.temporal_split(*t);
        boxes.push(box1);
        bbox = box2;
    }
    boxes.push(bbox);
    boxes
}

/// Determines where the cluster representative bbox should split
fn get_temporal_splits(trjs: Vec<(u32, Vec<[f64; 3]>)>) -> Vec<f64> {
    let mut splits = vec![];
    let mut timestamps = vec![];
    for (idx, trj) in trjs.iter().map(|trj| trj.1.clone()).enumerate() {
        let (t1, t2) = (trj[0][2], trj[trj.len() - 1][2]);
        timestamps.push((idx, t1));
        timestamps.push((idx, t2));
    }
    timestamps.sort_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap());
    let mut visited: HashSet<usize> = HashSet::new();
    let mut last_visited = None;
    for (idx, t) in timestamps.into_iter() {
        // mark index as visited
        if !visited.insert(idx) {
            // if we already visited the edge we can infer
            // that this must be a split.
            // (see paper on detour, section on 'Perserving temporal monotonicity')
            // we add -1ms s.t. the split is just before the end
            // of the edge
            if let Some(last_visited) = last_visited {
                splits.push(last_visited);
                splits.push(t - 1.0);
                visited = HashSet::new();
            } else {
                panic!("This can't happen since we must've visited the idx.")
            }
        }
        last_visited = Some(t + 1.0);
    }
    splits
}
