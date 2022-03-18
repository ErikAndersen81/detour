use super::Path;
use crate::utility::trajectory::Monotone;
use crate::utility::Bbox;
use crate::{CONFIG, OUTPUT, STATS};
use itertools::Itertools;
use petgraph::dot::Dot;
use petgraph::graph::NodeIndex;
use petgraph::prelude::EdgeIndex;
use petgraph::stable_graph::StableDiGraph;
use petgraph::visit::{depth_first_search, Control, DfsEvent, EdgeRef, IntoEdgeReferences};
use petgraph::EdgeDirection;
use std::fs::File;
use std::io::{BufWriter, Result, Write};

#[derive(Eq, PartialEq, Hash, Clone)]
struct TimePairs {
    a: usize,
    b: bool,
}

pub type Graph = StableDiGraph<(u32, Bbox), (u32, Vec<[f64; 3]>)>;

#[derive(Clone)]
pub struct DetourGraph {
    graph: Graph,
    roots: Vec<NodeIndex>,
}

pub trait Writable {
    fn to_csv(&self) -> Result<()>;
}

impl Writable for Graph {
    fn to_csv(&self) -> Result<()> {
        let output = OUTPUT.lock().unwrap();

        if output.graph_dot {
            // Store the graph in graphviz format
            let dot = Dot::with_config(
                &self,
                &[
                    petgraph::dot::Config::NodeIndexLabel,
                    petgraph::dot::Config::EdgeIndexLabel,
                ],
            );
            let f = File::create("graph.dot")?;
            let mut f = BufWriter::new(f);
            writeln!(f, "{:?}", dot)?;
        }

        if output.graph_json {
            // Store the graph in json format
            let serialized = serde_json::to_string(self)?;
            let mut f = File::create("graph.json")?;
            f.write_all(serialized.as_bytes())?;
        }

        if output.nodes_csv {
            // Write a single csv file with bounding boxes
            let nodes = self
                .node_indices()
                .map(|nx| format!("{},{},{}", nx.index(), self[nx].0, self[nx].1))
                .join("");
            let nodes = format!("label,weight,x1,y1,t1,x2,y2,t2\n{}", nodes);
            let f = File::create("nodes.csv")?;
            let mut f = BufWriter::new(f);
            writeln!(f, "{}", nodes)?;
        }

        if output.edges_csv {
            println!("Writing {} edges", self.edge_count());
            // Write each trajectory to a separate csv file.
            for (i, edge) in self.edge_references().enumerate() {
                let f = File::create(format!("edge_{}_{}.csv", i, edge.weight().0))?;
                let trj = edge
                    .weight()
                    .1
                    .iter()
                    .map(|[x, y, t]| format!("{},{},{}", x, y, t))
                    .join("\n");
                let mut f = BufWriter::new(f);
                write!(f, "x,y,t\n{}", trj)?;
            }
        }
        Ok(())
    }
}

impl DetourGraph {
    pub fn new() -> DetourGraph {
        let graph: Graph = StableDiGraph::new();
        DetourGraph {
            graph,
            roots: vec![],
        }
    }

    pub fn set_graph(&mut self, new_graph: Graph, root_nodes: Vec<NodeIndex>) {
        self.roots = root_nodes;
        self.graph = new_graph;
    }

    pub fn get_mut_graph(&mut self) -> &mut Graph {
        &mut self.graph
    }

    pub fn get_graph(&self) -> &Graph {
        &self.graph
    }

    /// Writes the graph to the output folder.
    /// Command line arguments specifies the type of output.
    /// Coordinates are written in Web Mercator (EPSG 3857) Projection.
    pub fn to_csv(&self) -> Result<()> {
        // //println!("Writing data...");
        let output = OUTPUT.lock().unwrap();

        if output.graph_dot {
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
            writeln!(f, "{:?}", dot)?;
        }

        if output.graph_json {
            // Store the graph in json format
            let serialized = serde_json::to_string(&self.graph)?;
            let mut f = File::create("graph.json")?;
            f.write_all(serialized.as_bytes())?;
        }

        if output.nodes_csv {
            // Write a single csv file with bounding boxes
            let nodes = self
                .graph
                .node_indices()
                .map(|nx| format!("{},{},{}", nx.index(), self.graph[nx].0, self.graph[nx].1))
                .join("");
            let nodes = format!("label,weight,x1,y1,t1,x2,y2,t2\n{}", nodes);
            let f = File::create("nodes.csv")?;
            let mut f = BufWriter::new(f);
            writeln!(f, "{}", nodes)?;
        }

        if output.edges_csv {
            println!("Writing {} edges", self.graph.edge_count());
            // Write each trajectory to a separate csv file.
            for (i, edge) in self.graph.edge_references().enumerate() {
                let f = File::create(format!("edge_{}_{}.csv", i, edge.weight().0))?;
                let trj = edge
                    .weight()
                    .1
                    .iter()
                    .map(|[x, y, t]| format!("{},{},{}", x, y, t))
                    .join("\n");
                let mut f = BufWriter::new(f);
                write!(f, "x,y,t\n{}", trj)?;
            }
        }

        // // Write Statistics
        // println!("Storing stats:\n{:?}", *STATS.lock().unwrap());
        // let f = File::create("stats")?;
        // let mut f = BufWriter::new(f);
        // write!(f, "{:?}", *STATS)?;

        // // Write Configuration
        // let f = File::create("config")?;
        // let mut f = BufWriter::new(f);
        // write!(f, "{}", *CONFIG)?;
        Ok(())
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
    pub fn node_indices(&self) -> petgraph::stable_graph::NodeIndices<(u32, Bbox)> {
        self.graph.node_indices()
    }

    /// Allows iteration over edge weights.
    pub fn edge_weights(&self) -> Vec<&(u32, Vec<[f64; 3]>)> {
        self.graph
            .edge_weights()
            .collect::<Vec<&(u32, Vec<[f64; 3]>)>>()
    }

    pub fn get_node_bbox(&self, nx: NodeIndex) -> Bbox {
        self.graph[nx].1
    }

    pub fn get_node_weight(&self, nx: NodeIndex) -> u32 {
        self.graph[nx].0
    }

    pub fn set_node_bbox(&mut self, nx: NodeIndex, bbox: Bbox) {
        self.graph[nx].1 = bbox;
    }

    pub fn set_node_weight(&mut self, nx: NodeIndex, weight: u32) {
        self.graph[nx].0 = weight;
    }

    pub fn edges_directed(&self, nx: NodeIndex, dir: EdgeDirection) -> Vec<EdgeIndex> {
        self.graph
            .edges_directed(nx, dir)
            .map(|e| e.id())
            .collect_vec()
    }

    pub fn edge_trj_mut(&mut self, ex: EdgeIndex) -> &mut Vec<[f64; 3]> {
        &mut self.graph.edge_weight_mut(ex).unwrap().1
    }

    pub fn set_edge_weight(&mut self, ex: EdgeIndex, weight: u32) {
        self.graph.edge_weight_mut(ex).unwrap().0 = weight;
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
    fn verify_temporal_monotonicity(&self) -> bool {
        depth_first_search(&self.graph, self.roots.clone(), |event| match event {
            DfsEvent::TreeEdge(a, b) => {
                let b_start_time = self.graph[b].1.t1;
                let edges_starting_after_b = self
                    .graph
                    .edges_directed(a, EdgeDirection::Outgoing)
                    .filter(|edge| edge.target() == b)
                    .map(|edge| edge.weight().1[0][2])
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

    fn root_reachable(&self, target: NodeIndex) -> bool {
        let roots = self.roots.clone();
        let bbox = &self.graph[target].1;
        let result = depth_first_search(&self.graph, roots, |event| match event {
            DfsEvent::Discover(nx, _) => {
                if target == nx {
                    Control::Break(nx)
                } else {
                    Control::Continue
                }
            }
            DfsEvent::TreeEdge(_, nx) => {
                if bbox.is_before(&self.graph[nx].1) {
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
        let mut a: NodeIndex = self.graph.add_node((1, bbox));
        self.roots.push(a);
        while let Some((trj, bbox)) = path.next_trj_stop() {
            let b = self.graph.add_node((1, bbox));
            self.graph.add_edge(a, b, (1, trj));
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
