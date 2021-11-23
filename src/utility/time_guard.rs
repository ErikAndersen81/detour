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

pub fn clean_stream(stream: Vec<[f64; 3]>) -> Vec<[f64; 3]> {
    assert!(!stream.is_empty(), "Cannot clean empty stream");
    let mut tg = TimeGuard::new(&stream[0]);
    stream
        .into_iter()
        .enumerate()
        .filter(|(i, point)| (*i == 0) || tg.is_ok(point))
        .map(|(_, x)| x)
        .collect::<Vec<[f64; 3]>>()
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn clean_stream_test() {
        let stream = vec![
            [0., 0., 0.],
            [0., 0., 1.],
            [0., 0., 2.],
            [0., 0., 1.],
            [0., 0., 3.],
            [0., 0., 4.],
            [0., 0., 5.],
            [0., 0., 6.],
        ];
        let clean = clean_stream(stream.clone());
        assert_eq!(clean.len(), 7);
        assert_eq!(clean[0][2] as i32, stream[0][2] as i32);
        assert_eq!(clean[2][2] as i32, stream[2][2] as i32);
        assert_ne!(clean[3][2] as i32, stream[3][2] as i32);
        assert_eq!(clean[3][2] as i32, stream[4][2] as i32);
    }
}
