use std::collections::HashMap;
use std::fs::File;
use std::io::{BufWriter, Result, Write};

use crate::parser::Config;
use crate::utility::trajectory::{merge, Monotone};
use crate::utility::{clustering, Bbox, StopDetector};
mod pathbuilder;

mod graph_builder;
use clustering::Clustering;
pub use graph_builder::get_graph;

use itertools::Itertools;
use petgraph::dot::Dot;
use petgraph::graph::EdgeIndex;
use petgraph::graph::NodeIndex;
use petgraph::stable_graph::StableDiGraph;
use petgraph::visit::{depth_first_search, Bfs, Control, DfsEvent, EdgeRef, IntoEdgeReferences};
use petgraph::EdgeDirection;
use trajectory_similarity::hausdorff;

use self::pathbuilder::Path;

enum RootCase {
    MultiRoot((NodeIndex, NodeIndex)),
    SingleRoot((NodeIndex, NodeIndex)),
}

#[derive(Clone)]
pub struct DetourGraph {
    graph: StableDiGraph<Bbox, Vec<[f64; 3]>>,
    roots: Vec<NodeIndex>,
    config: Config,
}

impl DetourGraph {
    pub fn new(config: Config) -> DetourGraph {
        let graph: StableDiGraph<Bbox, Vec<[f64; 3]>> = StableDiGraph::new();
        DetourGraph {
            graph,
            roots: vec![],
            config,
        }
    }

    pub fn to_csv(&self) -> Result<()> {
        let dot = Dot::with_config(
            &self.graph,
            &[
                petgraph::dot::Config::NodeIndexLabel,
                petgraph::dot::Config::EdgeIndexLabel,
            ],
        );
        let nodes = self
            .graph
            .node_indices()
            .map(|nx| format!("{},{}", nx.index(), self.graph[nx]))
            .join("");
        let nodes = format!("label,x1,y1,t1,x2,y2,t2\n{}", nodes);

        let f = File::create("nodes.csv")?;
        let mut f = BufWriter::new(f);
        write!(f, "{}", nodes)?;

        let f = File::create("graph.dot")?;
        let mut f = BufWriter::new(f);
        write!(f, "{:?}", dot)?;

        for (i, edge) in self.graph.edge_references().enumerate() {
            let f = File::create(format!("edge_{}.csv", i))?;
            let trj = edge
                .weight()
                .iter()
                .map(|[x, y, t]| format!("{},{},{}", *x, *y, *t))
                .join("\n");

            let mut f = BufWriter::new(f);
            write!(f, "x,y,t\n{}", trj)?;
        }

        Ok(())
    }

    pub fn merge_nodes(&mut self) {
        while let Some((a, b)) = self.find_matching_nodes() {
            assert!(self.verify_constraints(), "Not root reachable!");
            let root_case = self.get_root_case(a, b);
            match root_case {
                RootCase::MultiRoot((a, b)) => {
                    self.graph[a] = self.graph[a].union(&self.graph[b]);
                    self.copy_edges(a, b);
                    self.graph.remove_node(b);
                }
                RootCase::SingleRoot((a, b)) => {
                    self.graph[a] = self.graph[a].union(&self.graph[b]);
                    self.copy_edges(a, b);
                    self.graph.remove_node(b);
                    self.split_node(a);
                }
            }
        }
        assert!(self.verify_constraints(), "Invalid graph structure!");
        println!("Roots after merge:{:?}", self.roots);
    }

    fn get_temporal_splits(&self, split_node: NodeIndex) -> (f64, f64) {
        let t1 = self
            .graph
            .edges_directed(split_node, EdgeDirection::Incoming)
            .fold(f64::NEG_INFINITY, |acc, x| acc.max(x.weight()[0][2]));
        let t2 = self
            .graph
            .edges_directed(split_node, EdgeDirection::Incoming)
            .fold(f64::INFINITY, |acc, x| {
                acc.min(x.weight()[x.weight().len() - 1][2])
            });
        (t1, t2)
    }

    fn split_bbox(bbox: Bbox, t1: f64, t2: f64) -> (Bbox, Bbox, Bbox) {
        let (box1, box2) = bbox.temporal_split(t1, true);
        let (box2, box3) = box2.temporal_split(t2, false);
        (box1, box2, box3)
    }

    fn split_node(&mut self, split_node: NodeIndex) {
        let (t1, t2) = self.get_temporal_splits(split_node);
        let (box1, box2, box3) = DetourGraph::split_bbox(self.graph[split_node], t1, t2);
        let a = self.graph.add_node(box1);
        let b = self.graph.add_node(box2);
        let c = self.graph.add_node(box3);
        let mut a_edges: Vec<(NodeIndex, Vec<[f64; 3]>)> = vec![];
        let mut b_edges: Vec<(NodeIndex, Vec<[f64; 3]>)> = vec![];
        let mut c_edges: Vec<(NodeIndex, Vec<[f64; 3]>)> = vec![];
        for edge in self
            .graph
            .edges_directed(split_node, EdgeDirection::Outgoing)
        {
            let trj = edge.weight().clone();
            if box1.contains_point(&trj[0]) {
                a_edges.push((edge.target(), trj));
            } else if box2.contains_point(&trj[0]) {
                b_edges.push((edge.target(), trj));
            } else {
                c_edges.push((edge.target(), trj));
            }
        }
        for (target, trj) in a_edges.into_iter() {
            self.graph.add_edge(a, target, trj);
        }
        for (target, trj) in b_edges.into_iter() {
            self.graph.add_edge(b, target, trj);
        }
        for (target, trj) in c_edges.into_iter() {
            self.graph.add_edge(c, target, trj);
        }

        let mut c_edges: Vec<(NodeIndex, Vec<[f64; 3]>)> = vec![];

        for edge in self
            .graph
            .edges_directed(split_node, EdgeDirection::Incoming)
        {
            let trj = edge.weight().clone();
            if edge.source() == split_node {
                c_edges.push((a, trj));
            } else {
                c_edges.push((edge.source(), trj));
            }
        }

        for (source, trj) in c_edges.into_iter() {
            self.graph.add_edge(source, c, trj);
        }
        self.roots.push(a);
        if self.graph.edges(b).count() == 0 {
            self.graph.remove_node(b);
        } else {
            self.roots.push(b);
        }
        self.graph.remove_node(split_node);
    }

    fn get_root_case(&mut self, nx_a: NodeIndex, nx_b: NodeIndex) -> RootCase {
        // Returns node indices: (keep, remove);
        let a = self.in_root(nx_a);
        let b = self.in_root(nx_b);
        match (a, b) {
            (None, None) => RootCase::MultiRoot((nx_a, nx_b)),
            (Some(idx), None) => {
                let roots = self.get_root_indices(nx_b);
                self.roots.remove(idx);
                for root in roots.iter() {
                    if *root != idx {
                        return RootCase::MultiRoot((nx_b, nx_a));
                    }
                }
                RootCase::SingleRoot((nx_b, nx_a))
            }
            (None, Some(idx)) => {
                let roots = self.get_root_indices(nx_a);
                self.roots.remove(idx);
                for root in roots.iter() {
                    if *root != idx {
                        return RootCase::MultiRoot((nx_a, nx_b));
                    }
                }
                RootCase::SingleRoot((nx_a, nx_b))
            }
            (Some(_), Some(idx)) => {
                self.roots.remove(idx);
                RootCase::MultiRoot((nx_a, nx_b))
            }
        }
    }

    fn get_root_indices(&self, nx: NodeIndex) -> Vec<usize> {
        // Returns indices of nodes in self.roots from which nx can be reached
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

    fn copy_edges(&mut self, to: NodeIndex, from: NodeIndex) {
        let mut edges: Vec<(NodeIndex, NodeIndex, Vec<[f64; 3]>)> = vec![];
        // retreive the edges of 'from'
        for edge in self.graph.edges_directed(from, EdgeDirection::Incoming) {
            let (s, t) = (edge.source(), to);
            let trj = edge.weight().clone();
            edges.push((s, t, trj));
        }
        for edge in self.graph.edges_directed(from, EdgeDirection::Outgoing) {
            let (s, t) = (to, edge.target());
            let trj = edge.weight().clone();
            edges.push((s, t, trj));
        }
        for (s, t, trj) in edges {
            self.graph.add_edge(s, t, trj);
        }
    }

    fn in_root(&self, nx: NodeIndex) -> Option<usize> {
        self.roots.iter().position(|x| *x == nx)
    }

    fn verify_constraints(&self) -> bool {
        let mut valid = true;
        for nx in self.graph.node_indices() {
            match self.verify_node(nx) {
                (true, true, true) => (),
                (true, true, false) => {
                    println!("{:?} is not reachable", nx);
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
                    println!("{:?} is not reachable", nx);
                    valid = false;
                }
            }
        }
        valid
    }

    fn verify_node(&self, nx: NodeIndex) -> (bool, bool, bool) {
        // If a node is root it must be orphan.
        // A node that is not root cannot be orphan
        // Every node must be reachable from a root vertex.
        let is_root = self.in_root(nx).is_some();
        let is_orphan = self
            .graph
            .edges_directed(nx, EdgeDirection::Incoming)
            .count()
            == 0;
        let root_reachable = self.find_match(nx).is_some();
        (is_root, is_orphan, root_reachable)
    }

    fn find_matching_nodes(&self) -> Option<(NodeIndex, NodeIndex)> {
        for match_nx in self.graph.node_indices() {
            let bbox = self.graph[match_nx];
            let roots = self.roots.clone();
            let result = depth_first_search(&self.graph, roots, |event| {
                if let DfsEvent::Discover(nx, _) = event {
                    if self.graph[nx].overlaps(&bbox) && match_nx != nx {
                        Control::Break(nx)
                    } else {
                        Control::Continue
                    }
                } else if let DfsEvent::TreeEdge(_, nx) = event {
                    if bbox.is_before(&self.graph[nx]) {
                        Control::Prune
                    } else {
                        Control::Continue
                    }
                } else {
                    Control::Continue
                }
            });
            if let Some(nx) = result.break_value() {
                return Some((match_nx, nx));
            }
        }
        None
    }

    fn find_match(&self, target: NodeIndex) -> Option<NodeIndex> {
        let roots = self.roots.clone();
        let bbox = &self.graph[target];
        let result = depth_first_search(&self.graph, roots, |event| {
            if let DfsEvent::Discover(nx, _) = event {
                if target == nx {
                    Control::Break(nx)
                } else {
                    Control::Continue
                }
            } else if let DfsEvent::TreeEdge(_, nx) = event {
                if bbox.is_before(&self.graph[nx]) {
                    Control::Prune
                } else {
                    Control::Continue
                }
            } else {
                Control::Continue
            }
        });
        result.break_value()
    }

    pub fn merge_edges(&mut self) {
        let groups: Vec<((NodeIndex, NodeIndex), Vec<Vec<EdgeIndex>>)> = self
            .get_edge_groups()
            .iter()
            .map(|((source, target), group)| {
                ((*source, *target), self.get_edge_group_clusters(group))
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
                    .map(|ex| self.graph.edge_weight(*ex).unwrap().clone())
                    .collect();
                let trj = trjs.pop().unwrap();
                let trj: Vec<[f64; 3]> = trjs
                    .into_iter()
                    .fold(trj, |trj_a, trj_b| merge(&trj_a, &trj_b));
                self.replace_edges(source, target, &cluster, trj);
            }
        }
    }

    fn get_edge_group_clusters(&self, group: &[EdgeIndex]) -> Vec<Vec<EdgeIndex>> {
        let n = group.len();
        let mut dists = vec![vec![0f64; n]; n];
        for i in 0..group.len() {
            for j in (i + 1)..group.len() {
                let trj_a = self.graph.edge_weight(group[i]).unwrap();
                let trj_b = self.graph.edge_weight(group[j]).unwrap();
                dists[i][j] = hausdorff::similarity(trj_a, trj_b);
                dists[j][i] = dists[i][j];
            }
        }
        let clusters = Clustering::new(dists, self.config.max_hausdorff_meters).clusters;
        clusters
            .iter()
            .map(|c| c.iter().map(|idx| group[*idx]).collect::<Vec<EdgeIndex>>())
            .collect::<Vec<Vec<EdgeIndex>>>()
    }

    fn get_edge_groups(&self) -> HashMap<(NodeIndex, NodeIndex), Vec<EdgeIndex>> {
        let mut groups: HashMap<(NodeIndex, NodeIndex), Vec<EdgeIndex>> = HashMap::new();
        for source in self.graph.node_indices() {
            self.graph
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
        &mut self,
        source: NodeIndex,
        target: NodeIndex,
        group: &[EdgeIndex],
        trj: Vec<[f64; 3]>,
    ) {
        group.iter().for_each(|ex| {
            self.graph.remove_edge(*ex);
        });
        let replacement = self.graph.add_edge(source, target, trj);
    }

    #[allow(dead_code)]
    fn add_endpoints(&self, group: &[EdgeIndex]) -> Vec<Vec<[f64; 3]>> {
        // This might produce better clusters when we use hausdorff distance
        // however, if the bboxes are spatially small it's probably not needed.
        group
            .iter()
            .map(|ex| {
                let (a, b) = self.graph.edge_endpoints(*ex).unwrap();
                let (ex, (a, b)) = (*ex, (self.graph[a].center(), self.graph[b].center()));
                let mut trj = self.graph.edge_weight(ex).unwrap().clone();
                trj.insert(0, a);
                trj.push(b);
                trj
            })
            .collect::<Vec<Vec<[f64; 3]>>>()
    }

    pub fn add_path(&mut self, mut path: Path) {
        let bbox = path.remove(0).get_bbox().unwrap();
        let mut a: NodeIndex = self.graph.add_node(bbox);
        self.roots.push(a);
        path.iter().tuples().for_each(|(route, stop)| {
            let mut trj = route.get_trj().unwrap();
            let bbox = stop.get_bbox().unwrap();
            let b = self.graph.add_node(bbox);
            // adjust the trj to start and end in the center of the bboxes
            // at the earliest and latest time, respectively.
            let mut center_start = self.graph[a].center();
            center_start[2] = self.graph[a].t1;
            let mut center_end = self.graph[b].center();
            center_end[2] = self.graph[b].t2;
            trj.insert(0, center_start);
            trj.push(center_end);
            self.graph.add_edge(a, b, trj);
            a = b;
        });
        assert!(self.verify_constraints(), "Invalid graph structure!");
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::{data_structures::graph::pathbuilder::PathElement, parser::Config};
    #[test]
    fn merge_twins() {
        let mut graph = DetourGraph::new(Config::default());
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
        let mut found_n: usize = 0;
        while let Some((a, b)) = graph.find_matching_nodes() {
            found_n += 1;
            graph.get_root_case(a, b);
            graph.copy_edges(a, b);
            graph.graph.remove_node(b);
        }
        assert!(graph.verify_constraints());
        assert_eq!(graph.roots.len(), 1);
        assert_eq!(found_n, 3);
    }

    #[test]
    fn merge_quadruplets() {
        let mut graph = DetourGraph::new(Config::default());
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
        let mut found_n: usize = 0;
        while let Some((a, b)) = graph.find_matching_nodes() {
            found_n += 1;
            graph.get_root_case(a, b);
            graph.copy_edges(a, b);
            graph.graph.remove_node(b);
        }
        assert!(graph.verify_constraints());
        assert_eq!(graph.roots.len(), 1);
        assert_eq!(found_n, 9);
    }

    #[test]
    fn merge_root_non_root() {
        let mut graph = DetourGraph::new(Config::default());
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
        let mut found_n: usize = 0;
        while let Some((a, b)) = graph.find_matching_nodes() {
            found_n += 1;
            graph.get_root_case(a, b);
            graph.copy_edges(a, b);
            graph.graph.remove_node(b);
        }
        assert_eq!(graph.roots.len(), 2);
        assert!(graph.verify_constraints());
        assert_eq!(found_n, 2);
    }

    #[test]
    fn merge_skip_one() {
        let mut graph = DetourGraph::new(Config::default());
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
        let mut found_n: usize = 0;
        while let Some((a, b)) = graph.find_matching_nodes() {
            found_n += 1;
            graph.get_root_case(a, b);
            graph.copy_edges(a, b);
            graph.graph.remove_node(b);
        }
        assert_eq!(graph.roots.len(), 1);
        assert_eq!(found_n, 5);
        assert!(graph.verify_constraints());
    }
}
