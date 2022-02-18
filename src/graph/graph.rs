use super::Path;
use crate::utility::Bbox;
use crate::{CONFIG, STATS};
use itertools::Itertools;
use petgraph::dot::Dot;
use petgraph::graph::NodeIndex;
use petgraph::prelude::EdgeIndex;
use petgraph::stable_graph::StableDiGraph;
use petgraph::visit::{depth_first_search, Bfs, Control, DfsEvent, EdgeRef, IntoEdgeReferences};
use petgraph::EdgeDirection;
use serde::{Deserialize, Serialize};
use std::collections::HashSet;
use std::fs::File;
use std::io::{BufWriter, Result, Write};

enum RootCase {
    /// None of the nodes are root nodes
    NonRoots,
    /// One of the nodes has this idx in the root list. The other can be reached from a different node in the root list.
    DifferentRoot(usize),
    /// One of the nodes has this idx in the root list. The other can be reached only from this node.
    SameRoot(usize),
    /// Both nodes are in the root list.
    DoubleRoot((usize, usize)),
}

#[derive(Eq, PartialEq, Hash, Clone)]
struct TimePairs {
    a: usize,
    b: bool,
}

#[derive(Clone)]
pub struct DetourGraph {
    graph: StableDiGraph<Bbox, Vec<[f64; 3]>>,
    roots: Vec<NodeIndex>,
    outliers: StableDiGraph<Bbox, Vec<[f64; 3]>>,
}

impl DetourGraph {
    pub fn new() -> DetourGraph {
        let graph: StableDiGraph<Bbox, Vec<[f64; 3]>> = StableDiGraph::new();
        let outliers: StableDiGraph<Bbox, Vec<[f64; 3]>> = StableDiGraph::new();
        DetourGraph {
            graph,
            roots: vec![],
            outliers,
        }
    }

    pub fn set_graph(
        &mut self,
        new_graph: StableDiGraph<Bbox, Vec<[f64; 3]>>,
        root_nodes: Vec<NodeIndex>,
    ) {
        self.roots = root_nodes;
        self.graph = new_graph;
    }

    pub fn get_mut_graph(&mut self) -> &mut StableDiGraph<Bbox, Vec<[f64; 3]>> {
        &mut self.graph
    }

    pub fn get_graph(&self) -> &StableDiGraph<Bbox, Vec<[f64; 3]>> {
        &self.graph
    }

    pub fn set_outlier_graph(&mut self, outlier_graph: StableDiGraph<Bbox, Vec<[f64; 3]>>) {
        self.outliers = outlier_graph;
    }

    /// Writes the graph to the output folder.

    /// The coordinates of node and edge weights, i.e., bounding boxes and trajectories respectively,
    /// are stored in Web Mercator format.
    ///
    /// - A Visual representation are stored in `graph.dot` and `outlier_graph.dot` for the *regular graph* and the *outlier graph*, respectively.
    /// - The regular graph is stored in `graph.json` and the outlier graph is stored in `outlier_graph.json`
    /// - Information about splits and merges of nodes/edges are stored in `STATS`.
    /// - Configuration is written to `config` for reference purposes.
    pub fn to_csv(&self) -> Result<()> {
        println!("Writing data...");
        // Store the graph in graphviz format
        let dot = Dot::with_config(
            &self.graph,
            &[
                petgraph::dot::Config::NodeIndexLabel,
                petgraph::dot::Config::EdgeIndexLabel,
            ],
        );
        let f = File::create("graph.dot")?;
        let mut f = BufWriter::new(f);
        write!(f, "{:?}", dot)?;

        // Store the graph in json format
        let serialized = serde_json::to_string(&self.graph)?;
        let f = File::create("graph.json")?;
        let mut f = BufWriter::new(f);
        write!(f, "{:?}", serialized)?;

        // Store the outlier graph in graphviz format
        let dot = Dot::with_config(
            &self.outliers,
            &[
                petgraph::dot::Config::NodeIndexLabel,
                petgraph::dot::Config::EdgeIndexLabel,
            ],
        );
        let f = File::create("outlier_graph.dot")?;
        let mut f = BufWriter::new(f);
        write!(f, "{:?}", dot)?;

        // Write the outlier graph in json format
        let serialized = serde_json::to_string(&self.outliers)?;
        let f = File::create("outlier_graph.json")?;
        let mut f = BufWriter::new(f);
        write!(f, "{:?}", serialized)?;

        // Write a single csv file with bounding boxes
        let nodes = self
            .graph
            .node_indices()
            .map(|nx| format!("{},{}", nx.index(), self.graph[nx]))
            .join("");
        let nodes = format!("label,x1,y1,t1,x2,y2,t2\n{}", nodes);
        let f = File::create("nodes.csv")?;
        let mut f = BufWriter::new(f);
        write!(f, "{}", nodes)?;

        // Write each trajectory to a separate csv file.
        for (i, edge) in self.graph.edge_references().enumerate() {
            let f = File::create(format!("edge_{}.csv", i))?;
            let trj = edge
                .weight()
                .iter()
                .map(|[x, y, t]| {
                    let [x, y, t] = crate::from_epsg_3857_to_4326(&[*x, *y, *t]);
                    format!("{},{},{}", x, y, t)
                })
                .join("\n");
            let mut f = BufWriter::new(f);
            write!(f, "x,y,t\n{}", trj)?;
        }

        // Write a single csv file with outlier bounding boxes
        let nodes = self
            .outliers
            .node_indices()
            .map(|nx| format!("{},{}", nx.index(), self.outliers[nx]))
            .join("");
        let nodes = format!("label,x1,y1,t1,x2,y2,t2\n{}", nodes);
        let f = File::create("outlier_nodes.csv")?;
        let mut f = BufWriter::new(f);
        write!(f, "{}", nodes)?;

        // Write each outlier trajectory to a separate csv file.
        for (i, edge) in self.outliers.edge_references().enumerate() {
            let f = File::create(format!("outlier_edge_{}.csv", i))?;
            let trj = edge
                .weight()
                .iter()
                .map(|[x, y, t]| {
                    let [x, y, t] = crate::from_epsg_3857_to_4326(&[*x, *y, *t]);
                    format!("{},{},{}", x, y, t)
                })
                .join("\n");
            let mut f = BufWriter::new(f);
            write!(f, "x,y,t\n{}", trj)?;
        }

        // Write indices of root nodes.
        let roots = self
            .roots
            .iter()
            .map(|nx| format!("{}", nx.index()))
            .join(",");
        let f = File::create("roots.csv")?;
        let mut f = BufWriter::new(f);
        write!(f, "{}", roots)?;

        // Write Statistics
        println!("Storing stats:\n{:?}", *STATS.lock().unwrap());
        let f = File::create("stats")?;
        let mut f = BufWriter::new(f);
        write!(f, "{:?}", *STATS)?;

        // Write Configuration
        let f = File::create("config")?;
        let mut f = BufWriter::new(f);
        write!(f, "{}", *CONFIG)?;
        Ok(())
    }

    /// Merges a pair of nodes and returns the new node index
    fn merge_node_pair(&mut self, a: NodeIndex, b: NodeIndex) -> NodeIndex {
        let root_case = self.get_root_case(a, b);
        match root_case {
            RootCase::DifferentRoot(root_idx) => {
                self.roots.remove(root_idx);
                self.graph[a] = self.graph[a].union(&self.graph[b]);
                self.copy_edges(a, b);
                self.graph.remove_node(b);
            }
            RootCase::SameRoot(root_idx) => {
                self.roots[root_idx] = a;
                self.graph[a] = self.graph[a].union(&self.graph[b]);
                self.copy_edges(a, b);
                self.graph.remove_node(b);
            }
            RootCase::NonRoots => {
                self.graph[a] = self.graph[a].union(&self.graph[b]);
                self.copy_edges(a, b);
                self.graph.remove_node(b);
            }
            RootCase::DoubleRoot((root_a, root_b)) => {
                self.roots[root_a] = a;
                self.roots.remove(root_b);
                self.graph[a] = self.graph[a].union(&self.graph[b]);
                self.copy_edges(a, b);
                self.graph.remove_node(b);
            }
        }
        a
    }

    pub fn merge_nodes(&mut self) {
        while let Some((a, b)) = self.find_matching_nodes() {
            assert!(
                self.verify_constraints(),
                "Failed to verify constraints! {:?}",
                *STATS
            );
            STATS.lock().unwrap().node_merges += 1;
            let a = self.merge_node_pair(a, b);
            if !self.graph[a].verify_spatial() {
                // Stop exceeds spatial bounds, try to shrink it
                self.shrink_node(a);
            }
            if self.should_split(a) {
                self.split_node(a);
            }
            while !self.verify_temporal_monotonicity() {
                self.fix_temporal_monotonicity();
            }
        }
        if !self.verify_constraints() {
            let res = self.to_csv();
        }
        assert!(self.verify_constraints(), "Invalid graph structure!");
    }

    /// Shrink bbox based on points of connected trajectories. (not just endpoints)
    fn shrink_node(&mut self, nx: NodeIndex) {
        // Shrink bbox to fit all endpoints
        let in_points: Vec<[f64; 3]> = self
            .graph
            .edges_directed(nx, EdgeDirection::Incoming)
            .map(|edge| edge.weight().clone().pop().unwrap())
            .collect();
        let mut points: Vec<[f64; 3]> = self
            .graph
            .edges_directed(nx, EdgeDirection::Outgoing)
            .map(|edge| edge.weight().clone()[0])
            .collect();
        points.extend(in_points);
        let bbox = Bbox::new(&points);
        if bbox.verify_spatial() {
            self.graph[nx] = bbox;
        } else {
            panic!("Nodes should not have been merged!!");
        }

        // Expand bbox as much as possible:
        // First get trjs ending/starting in bbox
        let in_trjs: Vec<Vec<[f64; 3]>> = self
            .graph
            .edges_directed(nx, EdgeDirection::Incoming)
            .map(|edge| {
                let mut trj = edge.weight().clone();
                trj.reverse();
                trj
            })
            .collect();
        let mut trjs: Vec<Vec<[f64; 3]>> = self
            .graph
            .edges_directed(nx, EdgeDirection::Outgoing)
            .map(|edge| edge.weight().clone())
            .collect();
        trjs.extend(in_trjs);

        // Determine the minimal and maximal values for t1 and t2
        // s.t. we don't break temporal monotonicity
        let t1 = self
            .graph
            .edges_directed(nx, EdgeDirection::Incoming)
            .map(|edge| {
                let node = edge.source();
                self.graph[node].t2
            })
            .max_by(|a, b| a.partial_cmp(b).unwrap());
        let t2 = self
            .graph
            .edges_directed(nx, EdgeDirection::Outgoing)
            .map(|edge| {
                let node = edge.source();
                self.graph[node].t2
            })
            .min_by(|a, b| a.partial_cmp(b).unwrap());

        let bbox = bbox.expand_along_trjs(trjs, t1, t2);
        self.graph[nx] = bbox;
    }

    fn should_split(&self, nx: NodeIndex) -> bool {
        let bbox = self.graph[nx];
        for edge in self.graph.edges_directed(nx, EdgeDirection::Incoming) {
            let source = self.graph[edge.source()];
            if source.t2 >= bbox.t1 {
                return true;
            }
        }
        for edge in self.graph.edges_directed(nx, EdgeDirection::Outgoing) {
            let target = self.graph[edge.target()];
            if bbox.t2 >= target.t1 {
                return true;
            }
        }
        false
    }

    fn split_node_at(&mut self, split_node: NodeIndex, splits: &[f64]) {
        STATS.lock().unwrap().node_splits += splits.len();
        let nodes: Vec<NodeIndex> = DetourGraph::split_bbox(self.graph[split_node], splits)
            .into_iter()
            .map(|bbox| self.graph.add_node(bbox))
            .collect();
        self.reassign_edges(split_node, &nodes, EdgeDirection::Outgoing);
        self.reassign_edges(split_node, &nodes, EdgeDirection::Incoming);
        // remove new nodes that has no edges
        let no_edge_nodes: Vec<&NodeIndex> = nodes
            .iter()
            .filter(|nx| {
                (self
                    .graph
                    .edges_directed(**nx, EdgeDirection::Incoming)
                    .count()
                    == 0)
                    & (self
                        .graph
                        .edges_directed(**nx, EdgeDirection::Outgoing)
                        .count()
                        == 0)
            })
            .collect();
        for nx in no_edge_nodes.iter() {
            self.graph.remove_node(**nx);
        }
        // If the split node was in root, remove its reference from the root list
        if let Some(root_idx) = self.get_root_index(split_node) {
            self.roots.remove(root_idx);
        }
        // remove the now unused split node
        self.graph.remove_node(split_node);
        // Add orphan nodes to the root list
        let orphan_nodes: Vec<&NodeIndex> = nodes
            .iter()
            .filter(|nx| {
                (self
                    .graph
                    .edges_directed(**nx, EdgeDirection::Incoming)
                    .count()
                    == 0)
                    & (self
                        .graph
                        .edges_directed(**nx, EdgeDirection::Outgoing)
                        .count()
                        > 0)
            })
            .collect();
        for nx in orphan_nodes {
            self.roots.push(*nx);
        }
    }

    fn split_node(&mut self, split_node: NodeIndex) {
        let splits = self.get_temporal_splits(split_node);
        if !splits.is_empty() {
            self.split_node_at(split_node, &splits);
        }
    }

    fn get_temporal_splits(&self, split_node: NodeIndex) -> Vec<f64> {
        let edge_idxs = self.get_trjs_in_timeframe(split_node);
        let mut splits = vec![];
        let mut timestamps = vec![];
        for idx in edge_idxs {
            let trj = self.graph.edge_weight(idx).unwrap();
            let (t1, t2) = (trj[0][2], trj[trj.len() - 1][2]);
            timestamps.push((idx, t1));
            timestamps.push((idx, t2));
        }
        timestamps.sort_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap());
        let mut visited: HashSet<EdgeIndex> = HashSet::new();
        for (idx, t) in timestamps.into_iter() {
            // mark index as visited
            if !visited.insert(idx) {
                // if we already visited the edge we can infer
                // that this must be a split.
                // (see paper on detour, section on 'Perserving temporal monotonicity')
                // we add -1ms s.t. the split is just before the end
                // of the edge
                splits.push(t - 1.0);
                visited = HashSet::new();
            }
        }
        splits
    }

    /// Return edge indices connected to node that potentially violates temporal monotonicity
    /// i.e. their trajectories' temporal start and end lie within bbox of the node.
    fn get_trjs_in_timeframe(&self, node: NodeIndex) -> Vec<EdgeIndex> {
        let (box_t1, box_t2) = (self.graph[node].t1, self.graph[node].t2);
        let mut incoming: Vec<EdgeIndex> = self
            .graph
            .edges_directed(node, EdgeDirection::Incoming)
            .filter(|edge| {
                let trj = edge.weight();
                let [t1, t2] = [trj[0][2], trj[trj.len() - 1][2]];
                (box_t1..=box_t2).contains(&t1) & (box_t1..=box_t2).contains(&t2)
            })
            .map(|edge| edge.id())
            .collect();
        let outgoing: Vec<EdgeIndex> = self
            .graph
            .edges_directed(node, EdgeDirection::Outgoing)
            .filter(|edge| {
                let trj = edge.weight();
                let [t1, t2] = [trj[0][2], trj[trj.len() - 1][2]];
                (box_t1..=box_t2).contains(&t1) & (box_t1..=box_t2).contains(&t2)
            })
            .map(|edge| edge.id())
            .collect();
        incoming.extend(outgoing);
        incoming
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
        &mut self,
        split_node: NodeIndex,
        nodes: &[NodeIndex],
        direction: EdgeDirection,
    ) {
        let edges = match direction {
            EdgeDirection::Outgoing => {
                let mut edges = vec![];
                for edge in self.graph.edges_directed(split_node, direction) {
                    let trj = edge.weight().clone();
                    let pt = trj[0];
                    let target = if edge.target() != split_node {
                        edge.target()
                    } else {
                        nodes[nodes.len() - 1]
                    };
                    for node in nodes {
                        if self.graph[*node].is_in_temporal(&pt) {
                            edges.push((*node, target, trj));
                            break;
                        }
                    }
                }
                edges
            }
            EdgeDirection::Incoming => {
                let mut edges = vec![];
                for edge in self.graph.edges_directed(split_node, direction) {
                    let trj = edge.weight().clone();
                    let pt = trj[trj.len() - 1];
                    let source = if edge.source() != split_node {
                        edge.source()
                    } else {
                        nodes[0]
                    };
                    for node in nodes {
                        if self.graph[*node].is_in_temporal(&pt) {
                            edges.push((source, *node, trj));
                            break;
                        }
                    }
                }
                edges
            }
        };
        for (source, target, trj) in edges.into_iter() {
            self.graph.add_edge(source, target, trj);
        }
    }

    fn get_root_case(&mut self, nx_a: NodeIndex, nx_b: NodeIndex) -> RootCase {
        let a = self.get_root_index(nx_a);
        let b = self.get_root_index(nx_b);
        match (a, b) {
            (None, None) => RootCase::NonRoots,
            (Some(idx), None) => {
                let roots = self.get_dominant_roots(nx_b);
                for root in roots.iter() {
                    if *root != idx {
                        return RootCase::DifferentRoot(idx);
                    }
                }
                RootCase::SameRoot(idx)
            }
            (None, Some(idx)) => {
                let roots = self.get_dominant_roots(nx_a);
                for root in roots.iter() {
                    if *root != idx {
                        return RootCase::DifferentRoot(idx);
                    }
                }
                RootCase::SameRoot(idx)
            }
            (Some(idx_a), Some(idx_b)) => RootCase::DoubleRoot((idx_a, idx_b)),
        }
    }

    /// Removes node from the graph and the root list.
    pub fn remove_node(&mut self, nx: NodeIndex) {
        STATS.lock().unwrap().outlier_node_removals += 1;
        if let Some(root_idx) = self.get_root_index(nx) {
            self.roots.remove(root_idx);
            for edge in self.graph.edges_directed(nx, EdgeDirection::Outgoing) {
                self.roots.push(edge.target());
            }
        }
        self.graph.remove_node(nx);
    }

    /// Returns Some(index) of `nx` in the roots list or None
    fn get_root_index(&self, nx: NodeIndex) -> Option<usize> {
        self.roots.iter().position(|x| *x == nx)
    }

    /// Allows iteration over nodes.
    pub fn node_indices(&self) -> petgraph::stable_graph::NodeIndices<Bbox> {
        self.graph.node_indices()
    }

    /// Allows iteration over edge weights.
    pub fn edge_weights(&self) -> Vec<&Vec<[f64; 3]>> {
        self.graph.edge_weights().collect::<Vec<&Vec<[f64; 3]>>>()
    }

    pub fn get_node_weight(&self, nx: NodeIndex) -> Bbox {
        self.graph[nx]
    }

    pub fn set_node_weight(&mut self, nx: NodeIndex, bbox: Bbox) {
        self.graph[nx] = bbox;
    }

    pub fn edges_directed(&self, nx: NodeIndex, dir: EdgeDirection) -> Vec<EdgeIndex> {
        self.graph
            .edges_directed(nx, dir)
            .map(|e| e.id())
            .collect_vec()
    }

    pub fn edge_weight_mut(&mut self, ex: EdgeIndex) -> &mut Vec<[f64; 3]> {
        self.graph.edge_weight_mut(ex).unwrap()
    }

    /// Returns indices of nodes in self.roots from which nx can be reached
    fn get_dominant_roots(&self, nx: NodeIndex) -> Vec<usize> {
        let mut indices: Vec<usize> = vec![];
        for (idx, root) in self.roots.iter().enumerate() {
            let mut bfs = Bfs::new(&self.graph, *root);
            while let Some(node) = bfs.next(&self.graph) {
                if node == nx {
                    indices.push(idx);
                    break;
                }
            }
        }
        indices
    }

    fn copy_edges(&mut self, copyer: NodeIndex, copyed: NodeIndex) {
        let mut edges: Vec<(NodeIndex, NodeIndex, Vec<[f64; 3]>)> = vec![];
        // retreive the edges of 'copyed'
        for edge in self.graph.edges_directed(copyed, EdgeDirection::Incoming) {
            let (s, t) = (edge.source(), copyer);
            let trj = edge.weight().clone();
            edges.push((s, t, trj));
        }
        for edge in self.graph.edges_directed(copyed, EdgeDirection::Outgoing) {
            let (s, t) = (copyer, edge.target());
            let trj = edge.weight().clone();
            edges.push((s, t, trj));
        }
        for (s, t, trj) in edges {
            self.graph.add_edge(s, t, trj);
        }
    }

    fn verify_constraints(&self) -> bool {
        let mut valid = true;
        for nx in self.graph.node_indices() {
            match self.verify_node(nx) {
                (true, true, true) => (),
                (true, true, false) => {
                    println!("{:?} is not reachable (ttf)", nx);
                    valid = false;
                }
                (true, false, true) => {
                    println!("{:?} is root but not orphan", nx);
                    valid = false;
                }
                (true, false, false) => {
                    println!("{:?} is root but not orpan and not reachable", nx);
                    valid = false;
                }
                (false, true, true) => {
                    println!("{:?} is orphan but not root (still reachable!?)", nx);
                    valid = false;
                }
                (false, true, false) => {
                    println!("{:?} is orphan but not root and is not reachable", nx);
                    valid = false;
                }
                (false, false, true) => (),
                (false, false, false) => {
                    println!("{:?} is not reachable (fff)", nx);
                    println!("{:?}", self.roots);
                    valid = false;
                }
            }
        }
        valid & self.verify_temporal_monotonicity()
    }

    /// Checks and returns a tuple with the following for the node, nx:
    /// (is_root, is_orphan, is_root_reachable)
    fn verify_node(&self, nx: NodeIndex) -> (bool, bool, bool) {
        let is_root = self.get_root_index(nx).is_some();
        let is_orphan = self
            .graph
            .edges_directed(nx, EdgeDirection::Incoming)
            .count()
            == 0;
        let root_reachable = self.root_reachable(nx);
        (is_root, is_orphan, root_reachable)
    }

    /// If any edge (a,b) breaks temporal monotonicity -> false.
    fn verify_temporal_monotonicity_old(&self) -> bool {
        depth_first_search(&self.graph, self.roots.clone(), |event| match event {
            DfsEvent::TreeEdge(a, b) => {
                let a_time = self.graph[a].t2;
                let b_time = self.graph[b].t1;
                if a_time >= b_time {
                    println!("Temporal monotonicity not satisfied!");
                    println!("{:?} -> {:?}", a, b);
                    Control::Break(a)
                } else {
                    Control::Continue
                }
            }
            _ => Control::Continue,
        })
        .break_value()
        .is_none()
    }

    /// If any edge (a,b) breaks temporal monotonicity -> false.
    fn verify_temporal_monotonicity(&self) -> bool {
        depth_first_search(&self.graph, self.roots.clone(), |event| match event {
            DfsEvent::TreeEdge(a, b) => {
                let b_start_time = self.graph[b].t1;
                let edges_starting_after_b = self
                    .graph
                    .edges_directed(a, EdgeDirection::Outgoing)
                    .filter(|edge| edge.target() == b)
                    .map(|edge| edge.weight()[0][2])
                    .filter(|t| *t >= b_start_time)
                    .count();

                if edges_starting_after_b > 0 {
                    println!("Temporal monotonicity not satisfied!");
                    println!("{:?} -> {:?}", a, b);
                    Control::Break(a)
                } else {
                    Control::Continue
                }
            }
            _ => Control::Continue,
        })
        .break_value()
        .is_none()
    }

    /// finds and fixes temporal monotonicity by splitting nodes.
    fn fix_temporal_monotonicity(&mut self) {
        let broken_pair =
            depth_first_search(&self.graph, self.roots.clone(), |event| match event {
                DfsEvent::TreeEdge(a, b) => {
                    let a_time = self.graph[a].t2;
                    let b_time = self.graph[b].t1;
                    if a_time >= b_time {
                        Control::Break((a, b))
                    } else {
                        Control::Continue
                    }
                }
                _ => Control::Continue,
            })
            .break_value();
        if let Some((a, b)) = broken_pair {
            let split_b = self.graph[a].t2;
            let split_a = self.graph[b].t1;
            self.split_node_at(b, &[split_b]);
            self.split_node_at(a, &[split_a]);
        }
    }

    fn find_matching_nodes(&self) -> Option<(NodeIndex, NodeIndex)> {
        for match_nx in self.graph.node_indices() {
            let bbox: Bbox = self.graph[match_nx];
            let roots = self.roots.clone();
            let result = depth_first_search(&self.graph, roots, |event| match event {
                DfsEvent::Discover(nx, _) => {
                    if self.graph[nx].overlaps(&bbox) && match_nx != nx {
                        // Make sure that merging the nodes keeps the spatial constraints
                        let in_points_a: Vec<[f64; 3]> = self
                            .graph
                            .edges_directed(nx, EdgeDirection::Incoming)
                            .map(|edge| edge.weight().clone().pop().unwrap())
                            .collect();
                        let in_points: Vec<[f64; 3]> = self
                            .graph
                            .edges_directed(match_nx, EdgeDirection::Incoming)
                            .map(|edge| edge.weight().clone().pop().unwrap())
                            .collect();
                        let out_points_a: Vec<[f64; 3]> = self
                            .graph
                            .edges_directed(nx, EdgeDirection::Outgoing)
                            .map(|edge| edge.weight().clone()[0])
                            .collect();
                        let mut points: Vec<[f64; 3]> = self
                            .graph
                            .edges_directed(match_nx, EdgeDirection::Outgoing)
                            .map(|edge| edge.weight().clone()[0])
                            .collect();
                        points.extend(in_points);
                        points.extend(in_points_a);
                        points.extend(out_points_a);
                        let bbox = Bbox::new(&points);
                        if bbox.verify_spatial() {
                            Control::Break(nx)
                        } else {
                            Control::Continue
                        }
                    } else {
                        Control::Continue
                    }
                }
                DfsEvent::TreeEdge(_, nx) => {
                    if bbox.is_before(&self.graph[nx]) {
                        Control::Prune
                    } else {
                        Control::Continue
                    }
                }
                _ => Control::Continue,
            });
            if let Some(nx) = result.break_value() {
                return Some((match_nx, nx));
            }
        }
        None
    }

    fn root_reachable(&self, target: NodeIndex) -> bool {
        let roots = self.roots.clone();
        let bbox = &self.graph[target];
        let result = depth_first_search(&self.graph, roots, |event| match event {
            DfsEvent::Discover(nx, _) => {
                if target == nx {
                    Control::Break(nx)
                } else {
                    Control::Continue
                }
            }
            DfsEvent::TreeEdge(_, nx) => {
                if bbox.is_before(&self.graph[nx]) {
                    Control::Prune
                } else {
                    Control::Continue
                }
            }
            _ => Control::Continue,
        });
        result.break_value().is_some()
    }

    pub fn add_path(&mut self, mut path: Path) {
        let bbox = path.remove_first().copy_bbox().unwrap();
        let mut a: NodeIndex = self.graph.add_node(bbox);
        self.roots.push(a);
        while let Some((trj, bbox)) = path.next_trj_stop() {
            let b = self.graph.add_node(bbox);
            self.graph.add_edge(a, b, trj);
            a = b;
        }
        assert!(self.verify_constraints(), "Invalid graph structure!");
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::{config::Config, graph::path_builder::PathElement};
    #[test]
    fn merge_twins() {
        let config = Config::default();
        let mut graph = DetourGraph::new(&config);
        let bbox1 = Bbox::new(&[[0., 0., 0.], [1., 1., 1.]]);
        let bbox2 = Bbox::new(&[[2., 2., 2.], [3., 3., 3.]]);
        let bbox3 = Bbox::new(&[[4., 4., 4.], [5., 5., 5.]]);
        let trj = vec![[0.5, 0.5, 0.5], [2.5, 2.5, 2.5]];
        let trj2 = vec![[2.5, 2.5, 2.5], [4.5, 4.5, 4.5]];
        let path: Path = vec![
            PathElement::Stop(bbox1),
            PathElement::Route(trj),
            PathElement::Stop(bbox2),
            PathElement::Route(trj2),
            PathElement::Stop(bbox3),
        ];
        graph.add_path(path.clone());
        graph.add_path(path);
        let a = graph.roots[0];
        let b = graph.roots[1];
        graph.merge_node_pair(a, b);
        assert!(graph.verify_constraints());
    }

    #[test]
    fn merge_quadruplets() {
        let config = Config::default();
        let mut graph = DetourGraph::new(&config);
        let bbox1 = Bbox::new(&[[0., 0., 0.], [1., 1., 1.]]);
        let bbox2 = Bbox::new(&[[2., 2., 2.], [3., 3., 3.]]);
        let bbox3 = Bbox::new(&[[4., 4., 4.], [5., 5., 5.]]);
        let trj = vec![[0.5, 0.5, 0.5], [2.5, 2.5, 2.5]];
        let trj2 = vec![[2.5, 2.5, 2.5], [4.5, 4.5, 4.5]];
        let path1: Path = vec![
            PathElement::Stop(bbox1),
            PathElement::Route(trj),
            PathElement::Stop(bbox2),
            PathElement::Route(trj2),
            PathElement::Stop(bbox3),
        ];
        graph.add_path(path1.clone());
        graph.add_path(path1.clone());
        graph.add_path(path1.clone());
        graph.add_path(path1);
        assert!(graph.verify_constraints());
    }

    #[test]
    fn merge_root_non_root() {
        let config = Config::default();
        let mut graph = DetourGraph::new(&config);
        let bbox1 = Bbox::new(&[[0., 0., 0.], [1., 1., 1.]]);
        let bbox2 = Bbox::new(&[[2., 2., 2.], [3., 3., 3.]]);
        let bbox3 = Bbox::new(&[[4., 4., 4.], [5., 5., 5.]]);
        let bbox4 = Bbox::new(&[[1.2, 1.2, 1.2], [1.7, 1.7, 1.7]]);
        let trj = vec![[0.5, 0.5, 0.5], [2.5, 2.5, 2.5]];
        let trj2 = vec![[2.5, 2.5, 2.5], [4.5, 4.5, 4.5]];
        let trj3 = vec![[1.5, 1.5, 1.5], [2.5, 2.5, 2.5]];
        let path1: Path = vec![
            PathElement::Stop(bbox1),
            PathElement::Route(trj),
            PathElement::Stop(bbox2),
            PathElement::Route(trj2.clone()),
            PathElement::Stop(bbox3),
        ];
        let path2: Path = vec![
            PathElement::Stop(bbox4),
            PathElement::Route(trj3),
            PathElement::Stop(bbox2),
            PathElement::Route(trj2),
            PathElement::Stop(bbox3),
        ];
        graph.add_path(path1);
        graph.add_path(path2);
        assert!(graph.verify_constraints());
    }

    #[test]
    fn merge_skip_one() {
        let config = Config::default();
        let mut graph = DetourGraph::new(&config);
        let bbox1 = Bbox::new(&[[0., 0., 0.], [1., 1., 1.]]);
        let bbox2 = Bbox::new(&[[2., 2., 2.], [3., 3., 3.]]);
        let bbox3 = Bbox::new(&[[4., 4., 4.], [5., 5., 5.]]);
        let trj = vec![[0.5, 0.5, 0.5], [2.5, 2.5, 2.5]];
        let trj2 = vec![[2.5, 2.5, 2.5], [4.5, 4.5, 4.5]];
        let trj3 = vec![[1.5, 1.5, 1.5], [2.5, 2.5, 2.5]];
        let path1: Path = vec![
            PathElement::Stop(bbox1),
            PathElement::Route(trj),
            PathElement::Stop(bbox2),
            PathElement::Route(trj2),
            PathElement::Stop(bbox3),
        ];
        let path2: Path = vec![
            PathElement::Stop(bbox1),
            PathElement::Route(trj3),
            PathElement::Stop(bbox3),
        ];
        graph.add_path(path2);
        graph.add_path(path1.clone());
        graph.add_path(path1);
        assert!(graph.verify_constraints());
    }
}
