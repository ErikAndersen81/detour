struct TimeGuard {
    last_time: f64,
}

impl TimeGuard {
    pub fn new(point: &[f64; 3]) -> TimeGuard {
        TimeGuard {
            last_time: point[2],
        }
    }
    pub fn is_ok(&mut self, point: &[f64; 3]) -> bool {
        let new_time = point[2];
        if new_time < self.last_time {
            return false;
        }
        self.last_time = new_time;
        true
    }
}

pub fn clean_stream(mut stream: Vec<[f64; 3]>) -> Vec<[f64; 3]> {
    assert!(!stream.is_empty(), "Cannot clean empty stream");
    let init_point = stream.remove(0);
    let mut tg = TimeGuard::new(&init_point);
    stream
        .into_iter()
        .filter(|point| !tg.is_ok(point))
        .collect::<Vec<[f64; 3]>>()
}
