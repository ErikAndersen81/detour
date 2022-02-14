use std::fmt;

#[derive(Default, Debug)]
pub struct PathBuilderStats {
    pub streams_handled: usize,
    pub timeouts: Vec<f64>,
    pub path_lens: Vec<usize>,
}

impl fmt::Display for PathBuilderStats {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "Streams handled: {}", self.streams_handled)?;
        writeln!(f, "Timeouts: {}", self.timeouts.len())?;
        writeln!(f, "Paths created: {}", self.path_lens.len())?;
        let mut path_lens = self.path_lens.clone();
        path_lens.sort_unstable();
        if !path_lens.is_empty() {
            let min = path_lens[0];
            let median = path_lens[path_lens.len() / 2];
            let max = path_lens[path_lens.len() - 1];
            write!(
                f,
                "Path lengths:\n\tmin: {}\n\tmedian: {}\n\tmax: {}",
                min, median, max
            )?;
        }
        Ok(())
    }
}
