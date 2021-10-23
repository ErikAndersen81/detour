use super::{Trajectory, Vertex};

#[derive(Clone)]
pub(crate) struct Edge {
    pub to: Box<Vertex>,
    pub trj: Trajectory,
}

impl Edge {
    pub fn to_csv(&self, filename: String) -> std::io::Result<()> {
        self.trj.to_csv(filename)?;
        Ok(())
    }
}
