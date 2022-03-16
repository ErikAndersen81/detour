use itertools::Itertools;

use crate::visvalingam;
use crate::CONFIG;

pub type Trajectory = Vec<[f64; 3]>;

pub trait Monotone {
    fn is_monotone(&self) -> bool;
    fn make_monotone(&self) -> Self;
}

impl Monotone for Trajectory {
    fn is_monotone(&self) -> bool {
        self.iter()
            .enumerate()
            .all(|(idx, [_, _, t])| idx == 0 || self[idx - 1][2] < *t)
    }

    fn make_monotone(&self) -> Self {
        let mut mono = vec![self[0]];
        let mut prev = self[0][2];
        for ts in self {
            if ts[2] > prev {
                mono.push(*ts);
                prev = ts[2];
            }
        }
        mono
    }
}

impl Monotone for Vec<f64> {
    fn is_monotone(&self) -> bool {
        self.iter()
            .enumerate()
            .all(|(idx, val)| idx == 0 || self[idx - 1] < *val)
    }

    fn make_monotone(&self) -> Self {
        let mut mono = vec![self[0]];
        let mut prev = self[0];
        for ts in self.iter() {
            if *ts > prev {
                mono.push(*ts);
                prev = *ts;
            }
        }
        mono
    }
}

pub trait Timespan {
    fn common_timespan(&self, other: &Self) -> (f64, f64);
    fn trim_to_timespan(&self, timespan: (f64, f64)) -> Self;
}

impl Timespan for Trajectory {
    fn common_timespan(&self, other: &Self) -> (f64, f64) {
        let t1 = self[0][2].max(other[0][2]);
        let t2 = self[self.len() - 1][2].min(other[other.len() - 1][2]);
        (t1, t2)
    }

    fn trim_to_timespan(&self, timespan: (f64, f64)) -> Self {
        let (start, end) = (timespan.0, timespan.1);
        let mut trj = self
            .iter()
            .enumerate()
            .filter(|(_, &pt)| (start..=end).contains(&pt[2]))
            .map(|(i, pt)| (i, *pt))
            .collect_vec();
        if !self[0][2].eq(&start) {
            let start_idx = trj[0].0 - 1_usize;
            let p = self[start_idx];
            let q = self[start_idx + 1];
            let start_pt = interpolate(&start, &p, &q);
            trj[0].1 = start_pt;
        }
        let trj_last_idx = trj.len() - 1;
        if !self[self.len() - 1][2].eq(&end) {
            let end_idx = trj[trj_last_idx].0;
            let p = self[end_idx];
            let q = self[end_idx + 1];
            let end_pt = interpolate(&end, &p, &q);
            trj[trj_last_idx].1 = end_pt;
        }
        trj.iter().map(|(_, pt)| *pt).collect_vec()
    }
}

pub fn merge(trj_a: &[[f64; 3]], trj_b: &[[f64; 3]]) -> Vec<[f64; 3]> {
    let (trj_a, trj_b) = align_start_time(trj_a, trj_b);
    let (trj_a, trj_b) = morph_to_fit(&trj_a, &trj_b);
    let trj = average(&trj_a, &trj_b);
    assert!(trj.is_monotone());
    visvalingam(&trj, CONFIG.visvalingam_threshold)
}

fn average(trj_a: &[[f64; 3]], trj_b: &[[f64; 3]]) -> Vec<[f64; 3]> {
    let mut timestamps: Vec<f64> = trj_a.iter().map(|[_, _, t]| *t).collect();
    timestamps.extend(trj_b.iter().map(|[_, _, t]| *t));
    timestamps.sort_by(|a, b| a.partial_cmp(b).unwrap());
    timestamps = timestamps.make_monotone();
    let mut trj: Vec<[f64; 3]> = vec![];
    let mut trj_a = Interpolator::from(trj_a.to_vec());
    let mut trj_b = Interpolator::from(trj_b.to_vec());
    for t in timestamps {
        let point_a = trj_a.get_point(t);
        let point_b = trj_b.get_point(t);
        trj.push(mean_point(&point_a, &point_b));
    }
    trj
}

struct Interpolator {
    trj: Vec<[f64; 3]>,
    idx: usize,
}

impl Interpolator {
    fn from(trj: Vec<[f64; 3]>) -> Self {
        let trj = trj.make_monotone();
        assert!(
            trj.is_monotone(),
            "Not monotone: {:?}",
            trj.iter().map(|[_, _, t]| t).collect::<Vec<&f64>>()
        );
        Interpolator { trj, idx: 0 }
    }

    fn get_point(&mut self, t: f64) -> [f64; 3] {
        while self.idx < self.trj.len() - 1 {
            let p = self.trj[self.idx];
            let q = self.trj[self.idx + 1];
            if (p[2]..=q[2]).contains(&t) {
                return interpolate(&t, &p, &q);
            } else {
                self.idx += 1;
            }
        }
        panic!(
            "{} not in {:?}",
            t,
            self.trj.iter().map(|[_, _, t]| *t).collect::<Vec<f64>>()
        );
    }
}

fn interpolate(t: &f64, p: &[f64; 3], q: &[f64; 3]) -> [f64; 3] {
    assert!((p[2]..=q[2]).contains(t), "({}..={}), {}", p[2], q[2], t);
    if p[2].eq(t) {
        *p
    } else if q[2].eq(t) {
        *q
    } else {
        let dx = q[0] - p[0];
        let dy = q[1] - p[1];
        let s = (t - p[2]) / (q[2] - p[2]);
        [p[0] + dx * s, p[1] + dy * s, *t]
    }
}

fn mean_point(a: &[f64; 3], b: &[f64; 3]) -> [f64; 3] {
    let x: f64 = (a[0] + b[0]) / 2.;
    let y: f64 = (a[1] + b[1]) / 2.;
    let t: f64 = a[2];
    [x, y, t]
}

fn align_start_time(trj_a: &[[f64; 3]], trj_b: &[[f64; 3]]) -> (Vec<[f64; 3]>, Vec<[f64; 3]>) {
    let mut switch: bool = false;
    let (trj_a, trj_b) = if trj_a[0][2] < trj_b[0][2] {
        (trj_a, trj_b)
    } else {
        switch = true;
        (trj_b, trj_a)
    };
    let dt: f64 = (trj_b[0][2] - trj_a[0][2]) / 2.;
    let trj_a = trj_a
        .iter()
        .map(|c| [c[0], c[1], c[2] + dt])
        .collect::<Vec<[f64; 3]>>();
    let trj_b = trj_b
        .iter()
        .map(|c| [c[0], c[1], c[2] - dt])
        .collect::<Vec<[f64; 3]>>();
    if switch {
        (trj_b, trj_a)
    } else {
        (trj_a, trj_b)
    }
}

fn morph_to_fit(trj_a: &[[f64; 3]], trj_b: &[[f64; 3]]) -> (Vec<[f64; 3]>, Vec<[f64; 3]>) {
    let mut switch: bool = false;
    let (trj_a, trj_b) = if trj_a[trj_a.len() - 1][2] < trj_b[trj_b.len() - 1][2] {
        (trj_a, trj_b)
    } else {
        switch = true;
        (trj_b, trj_a)
    };
    let delta_a = trj_a[trj_a.len() - 1][2] - trj_a[0][2];
    let delta_b = trj_b[trj_b.len() - 1][2] - trj_b[0][2];
    let target: f64 = delta_a + ((delta_b - delta_a) / 2.0);
    let factor_a = delta_a / target;
    let factor_b = delta_b / target;
    let trj_a = trj_a
        .iter()
        .map(|[x, y, t]| [*x, *y, ((*t - trj_a[0][2]) / factor_a) + trj_a[0][2]])
        .collect::<Vec<[f64; 3]>>();
    let trj_b = trj_b
        .iter()
        .map(|[x, y, t]| [*x, *y, ((*t - trj_b[0][2]) / factor_b) + trj_b[0][2]])
        .collect::<Vec<[f64; 3]>>();
    // Ensure timespans starts and ends at exactly the same timestamp
    // (avoid floating point errors)
    let (idx_a, idx_b) = (trj_a.len() - 1, trj_b.len() - 1);
    let mut trj_a = trj_a;
    let mut trj_b = trj_b;
    trj_a[0][2] = trj_b[0][2].min(trj_a[0][2]);
    trj_b[0][2] = trj_a[0][2];
    trj_a[idx_a][2] = trj_b[idx_b][2].max(trj_a[idx_a][2]);
    trj_b[idx_b][2] = trj_a[idx_a][2];
    let trj_a = trj_a;
    if !switch {
        (trj_a, trj_b)
    } else {
        (trj_b, trj_a)
    }
}

#[cfg(test)]
mod test {
    use super::*;
    #[test]
    fn average_test() {}

    #[test]
    fn morph_25_16_test() {
        // trj_a should be stretched by 25% and trj_b should be
        // shrunk by (approx.) 16.66% s.t. they end at the same time.
        let trj_a = [[0., 0., 0.], [1., 0., 1.], [2., 0., 2.], [4., 0., 4.]];
        let trj_b = [[0., 0., 0.], [3., 0., 3.], [6., 0., 6.]];
        let (trj_a, trj_b) = morph_to_fit(&trj_a, &trj_b);
        assert_eq!([0., 0., 0.0], trj_a[0]);
        assert_eq!([1., 0., 1.25], trj_a[1]);
        assert_eq!([2., 0., 2.5], trj_a[2]);
        assert_eq!([4., 0., 5.0], trj_a[3]);

        assert_eq!([0., 0., 0.0], trj_b[0]);
        assert_eq!([3., 0., 2.5], trj_b[1]);
        assert_eq!([6., 0., 5.0], trj_b[2]);
    }

    #[test]
    fn align_start_time_test() {
        let trj_a = [[0., 0., 0.5], [1., 0., 1.], [2., 0., 2.], [3., 0., 3.]];
        let trj_b = [[0., 0., 0.], [0., 1., 1.], [0., 2., 2.], [0., 3., 3.]];
        let (trj_a, _trj_b) = align_start_time(&trj_a, &trj_b);
        assert_eq!([0., 0., 0.25], trj_a[0]);
        assert_eq!([1., 0., 0.75], trj_a[1]);
        assert_eq!([2., 0., 1.75], trj_a[2]);
        assert_eq!([3., 0., 2.75], trj_a[3]);
    }
}
