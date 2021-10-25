use super::get_distance;
use crate::parser::Config;

pub struct MotionDetector {
    timespan: f64,
    min_velocity: f64,
    eps: f64,
    tmp_ivls: Vec<f64>,       // Temporal intervals in timespan
    spt_ivls: Vec<f64>,       // Spatial intervals (in meters) in timespan
    ref_pt: Option<[f64; 3]>, // Reference point for calculating intervals
    is_moving: Option<bool>,  // Current motion state.
}

impl MotionDetector {
    pub fn new(config: Config) -> MotionDetector {
        MotionDetector {
            timespan: config.timespan,
            min_velocity: config.minimum_velocity,
            eps: config.epsilon_velocity,
            tmp_ivls: Vec::new(),
            spt_ivls: Vec::new(),
            ref_pt: None,
            is_moving: None,
        }
    }

    fn get_avg_velocity(&self) -> f64 {
        // calculate velocity of points in tmp_ivls and spt_ivls
        let dist: f64 = self.spt_ivls.clone().into_iter().sum();
        let span: f64 = self.tmp_ivls.clone().into_iter().sum();
        let h: f64 = span / 3600000.0;
        let km: f64 = dist / 1000.0;
        km / h
    }

    pub fn is_moving(&mut self, point: [f64; 3]) -> Option<bool> {
        /*
        Returns None when timespan is not filled
        */
        if let Some(from) = self.ref_pt {
            let dist: f64 = get_distance(&from, &point);
            self.ref_pt = Some(point);
            let span: f64 = point[2] - from[2];
            self.spt_ivls.push(dist);
            self.tmp_ivls.push(span);
            let span: f64 = self.tmp_ivls.clone().into_iter().sum();
            if span >= self.timespan {
                // The window is full
                let mut excess: f64 = self.tmp_ivls[0];
                // Throw away excess intervals
                while (span - excess) > self.timespan {
                    self.spt_ivls.remove(0);
                    self.tmp_ivls.remove(0);
                    excess += self.tmp_ivls[0];
                }
                let km_h = self.get_avg_velocity();
                match self.is_moving {
                    None => {
                        if self.min_velocity > km_h {
                            self.is_moving = Some(false);
                        } else {
                            self.is_moving = Some(true);
                        }
                    }
                    Some(true) if self.min_velocity > km_h => {
                        self.is_moving = Some(false);
                    }
                    Some(false) if (self.min_velocity + self.eps) < km_h => {
                        self.is_moving = Some(true);
                    }
                    _ => {}
                }
            }
        } else {
            self.ref_pt = Some(point);
        }
        self.is_moving
    }
}

#[cfg(test)]
mod trajectory_builder_test {
    use super::*;
    const CONFIG: Config = Config {
        timespan: 300000.0,
        window_size: 5,
        minimum_velocity: 1.5,
        epsilon_velocity: 0.3,
        connection_timeout: 180000.0,
    };
    #[test]
    fn unfilled_window_test() {
        // According to google these two points are approximately 2 km apart
        let from = [10.128126551731393, 55.39057912238903, 0.];
        let to = [10.159840991123847, 55.386813002794774, 1.];
        let mut md = MotionDetector::new(CONFIG);
        let mut is_moving = md.is_moving(from);
        assert_eq!(None, is_moving);
        is_moving = md.is_moving(to);
        assert_eq!(None, is_moving);
    }

    #[test]
    fn is_moving_test() {
        // According to google these two points are approximately 2 km apart
        // If travelling from a -> b takes 15 min = 900 000 ms
        // then the travel speed is 8 km/h which should be higher than 2.5 km/h
        // i.e. is_moving is true
        let speed: f64 = 900000.0;
        let a = [10.128126551731393, 55.39057912238903, 0.];
        let b = [10.159840991123847, 55.386813002794774, speed];
        let mut md = MotionDetector::new(CONFIG);
        let mut is_moving = md.is_moving(a);
        assert_eq!(is_moving, None);
        is_moving = md.is_moving(b);
        assert_eq!(is_moving, Some(true));
        // Staying at point b for slightly more than 15 minutes
        // s.t. the first value is tossed
        let c = [b[0], b[1], speed * 2.1];
        is_moving = md.is_moving(c);
        assert_eq!(is_moving, Some(false));
    }
}
