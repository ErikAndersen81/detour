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
