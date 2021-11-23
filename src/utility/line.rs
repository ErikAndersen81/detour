pub struct Line {
    pub start: [f64; 2],
    pub end: [f64; 2],
}
#[allow(dead_code)]
impl Line {
    pub fn intersection(&self, other: &Line) -> Option<[f64; 2]> {
        let (x1, y1, x2, y2) = (self.start[0], self.start[1], self.end[0], self.end[1]);
        let (x3, y3, x4, y4) = (other.start[0], other.start[1], other.end[0], other.end[1]);
        let denom: f64 = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        let t: f64 = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
        let u: f64 = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom;
        if (0.0..=1.0).contains(&t) {
            return Some([x1 + t * (x2 - x1), y1 + t * (y2 - y1)]);
        } else if (0.0..=1.0).contains(&u) {
            return Some([x3 + u * (x4 - x3), y3 + u * (y4 - y3)]);
        };
        None
    }

    pub fn advance(&mut self, next: &[f64; 2]) {
        self.start = self.end;
        self.end = *next;
    }

    pub fn length(&self) -> f64 {
        let [x1, y1] = self.start;
        let [x2, y2] = self.end;
        ((x1 - x2).powi(2) + (y1 - y2).powi(2)).sqrt()
    }
}
