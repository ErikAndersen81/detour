pub struct TimeoutHandler {
    connection_timeout: f64,
    last_read: [f64; 3],
}

impl TimeoutHandler {
    pub fn new(connection_timeout: f64, initial_point: &[f64; 3]) -> TimeoutHandler {
        TimeoutHandler {
            connection_timeout,
            last_read: *initial_point,
        }
    }

    pub fn is_alive(&mut self, point: &[f64; 3]) -> bool {
        let delta_time = point[2] - self.last_read[2];
        self.last_read = *point;
        delta_time < self.connection_timeout
    }
}

#[cfg(test)]
mod test {
    use super::*;
    #[test]
    fn test_is_alive() {
        let connection_timeout = 3.0;
        let points: Vec<[f64; 3]> = vec![
            [0., 0., 1.],
            [0., 0., 2.],
            [0., 0., 3.],
            [0., 0., 4.],
            [0., 0., 5.],
            [0., 0., 9.],
            [0., 0., 10.],
            [0., 0., 11.],
            [0., 0., 12.],
            [0., 0., 13.],
            [0., 0., 14.],
        ];
        let mut th = TimeoutHandler::new(connection_timeout, &points[0]);
        assert!(th.is_alive(&points[1]));
        assert!(th.is_alive(&points[2]));
        assert!(th.is_alive(&points[3]));
        assert!(th.is_alive(&points[4]));
        assert!(!th.is_alive(&points[5]));
        assert!(th.is_alive(&points[6]));
        assert!(th.is_alive(&points[7]));
        assert!(th.is_alive(&points[8]));
        assert!(th.is_alive(&points[9]));
        assert!(th.is_alive(&points[10]));
    }

    #[test]
    fn short_timeout_test() {
        let connection_timeout = 3.0;
        let points: Vec<[f64; 3]> = vec![
            [0., 0., 1.],
            [0., 0., 6.],
            [0., 0., 7.],
            [0., 0., 12.],
            [0., 0., 16.],
        ];
        let mut th = TimeoutHandler::new(connection_timeout, &points[0]);
        assert!(!th.is_alive(&points[1]));
        assert!(th.is_alive(&points[2]));
        assert!(!th.is_alive(&points[3]));
        assert!(!th.is_alive(&points[4]));
    }
}
