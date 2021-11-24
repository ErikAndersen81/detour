use super::Bbox;
use crate::parser::Config;

pub struct StopDetector {
    timespan: f64,
    maximal_distance: f64, // Maximal length of Bbox diagonal (spatially)
    points: Vec<[f64; 3]>, // last read points spanning no more than timespan
}

impl StopDetector {
    pub fn new(config: &Config) -> StopDetector {
        StopDetector {
            timespan: config.timespan,
            maximal_distance: config.maximal_distance,
            points: vec![],
        }
    }

    pub fn is_stopped(&mut self, point: [f64; 3]) -> bool {
        self.points.push(point);
        self.fit_to_timespan();
        let bbox = Bbox::new(&self.points);
        bbox.get_diameter() < self.maximal_distance
    }

    fn fit_to_timespan(&mut self) {
        while self.points.len() > 1
            && self.points[self.points.len() - 1][2] - self.points[0][2] > self.timespan
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
            timespan: 300000.0,
            maximal_distance: 75.0,
            points: vec![],
        };
        let points = vec![];
    }
}
