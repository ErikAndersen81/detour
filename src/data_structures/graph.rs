use std::collections::HashSet;
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
use petgraph::visit::{Bfs, EdgeRef};
use petgraph::EdgeDirection;
use trajectory_similarity::hausdorff;

use self::pathbuilder::Path;

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
            config,
            roots: vec![],
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

        for ex in self.graph.edge_indices() {
            let f = File::create(format!("edge_{}.csv", ex.index()))?;
            let trj = self
                .graph
                .edge_weight(ex)
                .unwrap()
                .iter()
                .map(|[x, y, t]| format!("{},{},{}", *x, *y, *t))
                .join("\n");

            let mut f = BufWriter::new(f);
            write!(f, "x,y,t\n{}", trj)?;
        }
        Ok(())
    }

    pub fn merge_edges(&mut self) {
        let mut handled_nodes: HashSet<NodeIndex> = HashSet::new();
        for root in self.roots.clone().iter() {
            let mut bfs = Bfs::new(&self.graph, *root);
            while let Some(nx) = bfs.next(&self.graph) {
                if !handled_nodes.contains(&nx) {
                    handled_nodes.insert(nx);
                    let groups = self
                        .graph
                        .neighbors_directed(nx, EdgeDirection::Outgoing)
                        .collect::<Vec<_>>();
                    let groups = groups.as_slice().group_by(|a, b| *a == *b);
                    for group in groups {
                        let group = self
                            .graph
                            .edges(nx)
                            .filter(|edge| edge.target() == group[0])
                            .map(|edge| edge.id())
                            .collect::<Vec<EdgeIndex>>();
                        let trjs = self.add_endpoints(&group);
                        let n = trjs.len();
                        let mut dists = vec![vec![0f64; n]; n];
                        for i in 0..trjs.len() {
                            for j in (i + 1)..trjs.len() {
                                dists[i][j] = hausdorff::similarity(&trjs[i], &trjs[j]);
                                dists[j][i] = dists[i][j];
                            }
                        }
                        let clustering = Clustering::new(dists, 100f64);
                        for cluster in clustering.clusters {
                            if cluster.len() < 2 {
                                continue;
                            }
                            let mut trajectories: Vec<Vec<[f64; 3]>> = cluster
                                .iter()
                                .map(|e| self.graph.edge_weight(group[*e]).unwrap().clone())
                                .collect();
                            let mut trj_a = trajectories.pop().unwrap();
                            for trj_b in trajectories {
                                trj_a = merge(&trj_a, &trj_b);
                            }
                            let edges_to_remove = cluster
                                .iter()
                                .map(|idx| group[*idx])
                                .collect::<Vec<EdgeIndex>>();
                            self.replace_edges(&edges_to_remove, trj_a);
                        }
                    }
                }
            }
        }
    }

    fn replace_edges(&mut self, group: &[EdgeIndex], trj: Vec<[f64; 3]>) {
        let (a, b) = self.graph.edge_endpoints(group[0]).unwrap();
        group.iter().for_each(|ex| {
            self.graph.remove_edge(*ex);
        });
        let replacement = self.graph.add_edge(a, b, trj);
        println!("replacing {:?} with {:?}", group, replacement);
    }

    fn add_endpoints(&self, group: &[EdgeIndex]) -> Vec<Vec<[f64; 3]>> {
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

    fn add_primary_path(&mut self, mut path: Path) {
        let bbox = path.remove(0).get_bbox().unwrap();
        let mut a: NodeIndex = self.graph.add_node(bbox);
        self.roots.push(a);
        path.iter().tuples().for_each(|(route, stop)| {
            let trj = route.get_trj().unwrap();
            let bbox = stop.get_bbox().unwrap();
            let b = self.graph.add_node(bbox);
            self.graph.add_edge(a, b, trj);
            a = b;
        });
    }

    pub fn add_path(&mut self, mut path: Path) {
        if self.roots.is_empty() {
            self.add_primary_path(path);
        } else {
            let bbox = path.remove(0).get_bbox().unwrap();
            let mut a = if let Some(nx) = self.find_matching_node(&bbox) {
                self.expand_node(nx, bbox);
                nx
            } else {
                let nx = self.graph.add_node(bbox);
                self.roots.push(nx);
                nx
            };
            path.iter().tuples().for_each(|(route, stop)| {
                let trj = route.get_trj().unwrap();
                let bbox = stop.get_bbox().unwrap();
                let b = if let Some(nx) = self.find_matching_node(&bbox) {
                    self.expand_node(nx, bbox);
                    nx
                } else {
                    self.graph.add_node(bbox)
                };
                self.graph.add_edge(a, b, trj);
                a = b;
            });
        }
    }

    fn expand_node(&mut self, nx: NodeIndex, other: Bbox) {
        let bbox = self.graph[nx];
        self.graph[nx] = bbox.union(&other);
    }

    fn find_matching_node(&self, bbox: &Bbox) -> Option<NodeIndex> {
        for root in self.roots.clone().into_iter() {
            let mut bfs = Bfs::new(&self.graph, root);
            let bbox = bbox.expand_bbox(
                self.config.relax_bbox_meters,
                self.config.relax_bbox_minutes,
            );
            while let Some(nx) = bfs.next(&self.graph) {
                let other = self.graph[nx];
                if bbox.overlaps(&other) {
                    return Some(nx);
                }
            }
        }
        None
    }
}
