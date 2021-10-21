use super::{Trajectory, Vertex};

#[derive(Clone)]
pub(crate) struct Edge {
    pub to: Vertex,
    pub trj: Trajectory,
}
