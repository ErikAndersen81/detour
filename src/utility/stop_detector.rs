use super::Bbox;
use crate::parser::Config;

pub struct StopDetector {
    duration_ms: f64,
    diagonal_meters: f64,  // Maximal length of Bbox diagonal (spatially)
    points: Vec<[f64; 3]>, // last read points spanning no more than timespan
}

impl StopDetector {
    pub fn new(config: &Config) -> StopDetector {
        StopDetector {
            duration_ms: config.stop_duration_minutes * 60. * 1000.0,
            diagonal_meters: config.stop_diagonal_meters,
            points: vec![],
        }
    }

    pub fn is_stopped(&mut self, point: [f64; 3]) -> bool {
        self.points.push(point);
        self.fit_to_timespan();
        let bbox = Bbox::new(&self.points);
        bbox.get_diameter() < self.diagonal_meters
    }

    fn fit_to_timespan(&mut self) {
        while self.points.len() > 1
            && self.points[self.points.len() - 1][2] - self.points[0][2] > self.duration_ms
        {
            self.points.remove(0);
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn alternating() {
        let sd = StopDetector {
            duration_ms: 300000.0,
            diagonal_meters: 75.0,
            points: vec![],
        };
        let points = vec![];
    }
}
