use super::get_distance;
use crate::config::Config;

pub struct MotionDetector {
    /// Milliseconds of measurements to consider
    timespan: f64,
    /// Minimum velocity in km/h
    min_velocity: f64,
    /// Was the last point a 'stop'
    was_stopped: bool,
    /// If (min_velocity + eps > average velocity) then it is moving
    eps: f64,
    /// Temporal intervals in timespan
    tmp_ivls: Vec<f64>,
    /// Spatial intervals (in meters) in timespan
    spt_ivls: Vec<f64>,
    /// Reference point for calculating intervals
    ref_pt: Option<[f64; 3]>,
}

impl MotionDetector {
    pub fn new(config: &Config) -> MotionDetector {
        MotionDetector {
            timespan: config.stop_duration_minutes,
            min_velocity: config.minimum_velocity,
            was_stopped: true,
            eps: config.epsilon_velocity,
            tmp_ivls: Vec::new(),
            spt_ivls: Vec::new(),
            ref_pt: None,
        }
    }

    /// calculate average velocity of points in tmp_ivls and spt_ivls
    fn get_avg_velocity(&self) -> f64 {
        let dist: f64 = self.spt_ivls.clone().into_iter().sum();
        let span: f64 = self.tmp_ivls.clone().into_iter().sum();
        let h: f64 = span / 3600000.0;
        let km: f64 = dist / 1000.0;
        km / h
    }

    /// Returns true if the object is stopped.
    pub fn is_stopped(&mut self, point: [f64; 3]) -> bool {
        if let Some(from) = self.ref_pt {
            let dist: f64 = get_distance(&from, &point);
            self.ref_pt = Some(point);
            let span: f64 = point[2] - from[2];
            self.spt_ivls.push(dist);
            self.tmp_ivls.push(span);
            let span: f64 = self.tmp_ivls.iter().sum();
            if span >= self.timespan {
                // The window is full
                let mut excess: f64 = self.tmp_ivls[0];
                // Throw away excess intervals
                while (span - excess) > self.timespan {
                    self.spt_ivls.remove(0);
                    self.tmp_ivls.remove(0);
                    excess += self.tmp_ivls[0];
                }
            }
            let km_h = self.get_avg_velocity();
            if self.was_stopped && ((self.min_velocity + self.eps) < km_h) {
                self.was_stopped = false;
            } else if !self.was_stopped && (self.min_velocity > km_h) {
                self.was_stopped = true;
            }
        } else {
            self.ref_pt = Some(point);
            self.was_stopped = true;
        }
        self.was_stopped
    }
}
