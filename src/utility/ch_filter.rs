use geo::prelude::HaversineDistance;

pub struct CHFilter<I: Iterator<Item = [f64; 3]>> {
    stream: I,
    window: Vec<[f64; 3]>,
    window_size: usize,
}

impl<I: Iterator<Item = [f64; 3]>> Iterator for CHFilter<I> {
    type Item = [f64; 3];
    fn next(&mut self) -> std::option::Option<[f64; 3]> {
        while self.window.len() < self.window_size {
            let point = self.stream.next();
            if let Some(point) = point {
                self.window.push(point);
            } else {
                // Nothing to read from stream. Reflect this in return
                return None;
            }
            let trj = get_convex_hull_trj(self.window.clone());
            let spikes: Vec<[f64; 3]> = get_spikes(trj);
            self.window = remove_spikes(self.window.clone(), spikes);
        }
        Some(self.window.remove(0))
    }
}

impl<I: Iterator<Item = [f64; 3]>> CHFilter<I> {
    pub fn new(window_size: usize, stream: I) -> CHFilter<I> {
        CHFilter {
            stream,
            window: Vec::new(),
            window_size,
        }
    }
}

fn get_convex_hull_trj(points: Vec<[f64; 3]>) -> Vec<[f64; 3]> {
    let mut coords2d: Vec<geo::Coordinate<f64>> = points
        .iter()
        .map(|c| geo::Coordinate { x: c[0], y: c[1] })
        .collect::<Vec<geo::Coordinate<f64>>>();
    let extreme_pts: geo::LineString<f64> =
        geo::algorithm::convex_hull::quick_hull(coords2d.as_mut_slice());
    fn same_point(c3: &[f64; 3], c2: &geo::Point<f64>) -> bool {
        (c3[0] - c2.x()).abs() + (c3[1] - c2.y()).abs() < 0.00000001
    }
    fn is_extreme(c3: &[f64; 3], extreme_pts: &geo::LineString<f64>) -> bool {
        extreme_pts
            .clone()
            .into_points()
            .into_iter()
            .any(|p| same_point(c3, &p))
    }
    points
        .into_iter()
        .filter(|p: &[f64; 3]| is_extreme(p, &extreme_pts))
        .collect::<Vec<[f64; 3]>>()
}

fn remove_spikes(trj: Vec<[f64; 3]>, spikes: Vec<[f64; 3]>) -> Vec<[f64; 3]> {
    fn same_point(p: &[f64; 3], q: &[f64; 3]) -> bool {
        // Equality of floats is intentional: they must be copies!
        (p[0] == q[0]) && (p[1] == q[1])
    }
    let mut spikeless: Vec<[f64; 3]> = Vec::new();
    let mut i: usize = 0;
    let mut j: usize = 0;
    while i < spikes.len() {
        let p = spikes[i];
        let q = trj[j];
        if same_point(&p, &q) {
            i += 1;
            j += 1;
        } else {
            j += 1;
            spikeless.push(q);
        }
    }
    while j < trj.len() {
        spikeless.push(trj[j]);
        j += 1;
    }
    spikeless
}

fn get_spikes(trj: Vec<[f64; 3]>) -> Vec<[f64; 3]> {
    let mut spikes: Vec<[f64; 3]> = Vec::new();
    fn is_spike(p: &[f64; 3], q: &[f64; 3], r: &[f64; 3]) -> bool {
        (get_distance(p, q) > get_distance(p, r)) | (get_distance(q, r) > get_distance(p, r))
    }
    for i in 1..trj.len() - 1 {
        if is_spike(&trj[i - 1], &trj[i], &trj[i + 1]) {
            spikes.push(trj[i])
        }
    }
    spikes
}

fn get_distance(from: &[f64; 3], to: &[f64; 3]) -> f64 {
    // Returns haversine distance in meters
    let start = geo::point!(x: from[0],y: from[1]);
    let end = geo::point!(x:to[0], y:to[1]);
    start.haversine_distance(&end)
}

#[cfg(test)]
mod trajectory_builder_test {
    use super::*;
    #[test]
    fn distance_test() {
        // According to google these two points are approximately 2 km apart
        let from = &[10.128126551731393, 55.39057912238903, 0.];
        let to = &[10.159840991123847, 55.386813002794774, 1.];
        let google_distance = 2000.;
        assert!((get_distance(from, to) - google_distance).abs() < 50.);
    }

    #[test]
    fn test_convexhull_track() {
        // Colinear points are also removed, as they should be!
        let trj = vec![
            [0., 0., 0.],
            [1.1, 0., 1.],
            [2., 0., 2.],
            [2., 1.1, 3.],
            [2., 2., 4.],
            [1., 2., 5.],
            [0., 2., 6.],
            [0., 1., 7.],
            [0., 0., 8.],
        ];
        let hulltrack = vec![
            [0., 0., 0.],
            [2., 0., 2.],
            [2., 2., 4.],
            [0., 2., 6.],
            [0., 0., 8.],
        ];
        let res = get_convex_hull_trj(trj);
        assert_eq!(res, hulltrack);
    }
}
