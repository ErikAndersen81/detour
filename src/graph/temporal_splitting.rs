use std::collections::HashSet;

use petgraph::graph::NodeIndex;
use petgraph::visit::EdgeRef;
use petgraph::EdgeDirection;

use crate::utility::{trajectory::Trajectory, Bbox};

use super::Graph;

pub fn make_temporally_monotone(graph: &mut Graph) {
    // Try to find suitable temporal split values for each node
    // based on start and end times of trajectories connecting them.
    let mut required_splits = vec![];
    for nx in graph.node_indices() {
        // calculate required splits
        let mut edges: Vec<Trajectory> = graph
            .edges_directed(nx, EdgeDirection::Incoming)
            .map(|edge| edge.weight().1.clone())
            .collect();
        let mut outgoing: Vec<Trajectory> = graph
            .edges_directed(nx, EdgeDirection::Outgoing)
            .map(|edge| edge.weight().1.clone())
            .collect();
        edges.append(&mut outgoing);
        let splits = get_temporal_splits(edges);
        required_splits.push((nx, splits));
    }
    for (split_node, splits) in required_splits {
        // Split bbox at splits and add the new nodes
        let nodes: Vec<NodeIndex> = split_bbox(graph[split_node].1, &splits)
            .into_iter()
            .map(|bbox| graph.add_node((graph[split_node].0, bbox)))
            .collect();
        reassign_edges(graph, split_node, &nodes, EdgeDirection::Outgoing);
        reassign_edges(graph, split_node, &nodes, EdgeDirection::Incoming);
        // remove new nodes that has no edges
        let no_edge_nodes: Vec<&NodeIndex> = nodes
            .iter()
            .filter(|nx| {
                (graph.edges_directed(**nx, EdgeDirection::Incoming).count() == 0)
                    & (graph.edges_directed(**nx, EdgeDirection::Outgoing).count() == 0)
            })
            .collect();
        for nx in no_edge_nodes.iter() {
            graph.remove_node(**nx);
        }
        // Finally remove the split node
        graph.remove_node(split_node);
    }
}

/// Returns list of timestamps where the trajectories should split to avoid breaking temporal monotonicity
fn get_temporal_splits(trjs: Vec<Trajectory>) -> Vec<f64> {
    let mut splits = vec![];
    let mut timestamps = vec![];
    for (idx, trj) in trjs.iter().enumerate() {
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
