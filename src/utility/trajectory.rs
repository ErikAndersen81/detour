use crate::visvalingam;
use crate::Config;
use std::iter::Peekable;

pub trait Monotone {
    fn is_monotone(&self) -> bool;
    fn make_monotone(&self) -> Self;
}

impl Monotone for Vec<[f64; 3]> {
    fn is_monotone(&self) -> bool {
        self.iter()
            .enumerate()
            .all(|(idx, [_, _, t])| idx == 0 || self[idx - 1][2] < *t)
    }

    fn make_monotone(&self) -> Self {
        let mut mono = vec![];
        let mut prev = self[0][2] - 1.;
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
        let mut mono = vec![];
        let mut prev = self[0] - 1.;
        for ts in self.iter() {
            if *ts > prev {
                mono.push(*ts);
                prev = *ts;
            }
        }
        mono
    }
}

pub fn get_common_time_span(trj_a: &[[f64; 3]], trj_b: &[[f64; 3]]) -> Option<(f64, f64)> {
    let a_len = trj_a.len() - 1;
    let b_len = trj_b.len() - 1;
    assert!(a_len > 0);
    assert!(trj_a[a_len][2] - trj_a[0][2] > 0.);
    assert!(b_len > 0);
    assert!(trj_b[b_len][2] - trj_b[0][2] > 0.);
    let start: f64 = if trj_a[0][2] > trj_b[0][2] {
        trj_a[0][2]
    } else {
        trj_b[0][2]
    };
    let end: f64 = if trj_a[a_len][2] < trj_b[b_len][2] {
        trj_a[a_len][2]
    } else {
        trj_b[b_len][2]
    };
    if end - start < 0.0 {
        None
    } else {
        Some((start, end))
    }
}

pub fn trim(trj: &[[f64; 3]], start: &f64, end: &f64) -> Vec<[f64; 3]> {
    // Adjust the trajectory to start and end at the given times
    let mut start_idx: usize = 0;
    let mut end_idx: usize = trj.len() - 1;
    let mut new_start: [f64; 3] = trj[start_idx];
    let mut new_end: [f64; 3] = trj[end_idx];
    for i in 0..(trj.len() - 1) {
        let point_p: [f64; 3] = trj[i];
        let point_q: [f64; 3] = trj[i + 1];
        if (point_p[2]..point_q[2]).contains(start) {
            new_start = Interpolator::interpolate(start, &point_p, &point_q);
            start_idx = i;
        };
        if (point_p[2]..point_q[2]).contains(end) {
            new_end = Interpolator::interpolate(end, &point_p, &point_q);
            end_idx = i + 1;
        };
    }
    let mut coords: Vec<[f64; 3]> = trj[start_idx..(end_idx + 1)].to_vec();
    coords[0] = new_start;
    coords[end_idx - start_idx] = new_end;
    coords
}

pub fn align_time_to_zero(trj: Vec<[f64; 3]>) -> Vec<[f64; 3]> {
    let offset: f64 = trj[0][2];
    trj.into_iter()
        .map(|point| [point[0], point[1], point[2] - offset])
        .collect::<Vec<[f64; 3]>>()
}

pub fn merge(trj_a: &[[f64; 3]], trj_b: &[[f64; 3]], config: &Config) -> Vec<[f64; 3]> {
    let (trj_a, trj_b) = align_start_time(trj_a, trj_b);
    let (trj_a, trj_b) = morph_to_fit(&trj_a, &trj_b);
    let trj = average(&trj_a, &trj_b);
    visvalingam(&trj, config.visvalingam_threshold)
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
                return Interpolator::interpolate(&t, &p, &q);
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

    fn interpolate(t: &f64, p: &[f64; 3], q: &[f64; 3]) -> [f64; 3] {
        assert!((p[2]..=q[2]).contains(t));
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

fn get_min(
    mut trj_a: Peekable<std::slice::Iter<[f64; 3]>>,
    mut trj_b: Peekable<std::slice::Iter<[f64; 3]>>,
) -> [f64; 3] {
    let a = *trj_a.peek().unwrap();
    let b = *trj_b.peek().unwrap();
    if a[2] < b[2] {
        *trj_a.next().unwrap()
    } else {
        *trj_b.next().unwrap()
    }
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
    let dt: f64 = (delta_b - delta_a) / 2.;
    let factor_a = delta_a / (delta_a + dt);
    let factor_b = delta_b / (delta_a + dt);
    let trj_a = trj_a
        .iter()
        .map(|[x, y, t]| [*x, *y, (*t - trj_a[0][2]) / factor_a])
        .collect::<Vec<[f64; 3]>>();
    let trj_b = trj_b
        .iter()
        .map(|[x, y, t]| [*x, *y, (*t - trj_b[0][2]) / factor_b])
        .collect::<Vec<[f64; 3]>>();
    // Ensure timespans end at exactly the same timestamp
    let (idx_a, idx_b) = (trj_a.len() - 1, trj_b.len() - 1);
    let mut trj_a = trj_a;
    trj_a[idx_a][2] = trj_b[idx_b][2];
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
    const CONFIG: Config = Config::default();
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

    #[test]
    fn merge_test() {
        let trj_a = [[0., 0., 0.], [1., 0., 1.], [2., 0., 2.], [3., 0., 3.]];
        let trj_b = [[0., 0., 0.], [0., 2., 2.], [0., 3., 3.]];
        let trj = merge(&trj_a, &trj_b, &CONFIG);
        assert_eq!([0., 0., 0.], trj[0]);
        assert_eq!([0.5, 0.5, 1.], trj[1]);
        assert_eq!([1., 1., 2.], trj[2]);
        assert_eq!([1.5, 1.5, 3.], trj[3]);
        assert_eq!(trj.len(), 4);
    }
}
