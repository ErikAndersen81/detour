use crate::trajectory::Trajectory;
use crate::{MIN_VELOCITY, WINDOW_SIZE};
use geo::prelude::HaversineDistance;
use std::iter::FromIterator;

pub struct TrajectoryBuilder {
    trj: Vec<[f64; 3]>,
    window: Vec<[f64; 3]>,
}

impl TrajectoryBuilder {
    pub fn new() -> TrajectoryBuilder {
        TrajectoryBuilder {
            trj: Vec::new(),
            window: Vec::new(),
        }
    }

    pub fn get_trajectory(&self) -> Trajectory {
        Trajectory::from_array(self.trj.clone())
    }

    pub fn handle_next(&mut self, next: [f64; 3]) -> Option<Trajectory> {
        self.window.push(next);
        if self.window.len() < WINDOW_SIZE {
            return None;
        }
        let trj: Vec<[f64; 3]> = get_convex_hull_trj(self.window.clone());
        let trj: Vec<[f64; 3]> = remove_spikes(trj);
        if ((&trj).len() >= 3) && (avg_velocity((&trj).clone()) < MIN_VELOCITY) {
            let trj = self.get_trajectory();
            self.window = Vec::new();
            self.trj = Vec::new();
            return Some(trj);
        }
        if trj.len() == WINDOW_SIZE {
            let coord = trj[0];
            let window = Vec::from_iter(trj[1..((&trj).len() - 1)].iter().cloned());
            self.trj.push(coord);
            self.window = window;
        } else {
            self.window = trj;
        }
        None
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
        .filter(|p| is_extreme(p, &extreme_pts))
        .collect::<Vec<[f64; 3]>>()
}

fn remove_spikes(trj: Vec<[f64; 3]>) -> Vec<[f64; 3]> {
    let mut spikeless_trj: Vec<[f64; 3]> = Vec::new();
    fn is_spike(p: &[f64; 3], q: &[f64; 3], r: &[f64; 3]) -> bool {
        (get_distance(p, q) > get_distance(p, r)) | (get_distance(q, r) > get_distance(p, r))
    }
    spikeless_trj.push(trj[0]);
    for i in 1..trj.len() - 1 {
        if !is_spike(&trj[i - 1], &trj[i], &trj[i + 1]) {
            spikeless_trj.push(trj[i])
        }
    }
    spikeless_trj.push(trj[trj.len() - 1]);
    spikeless_trj
}

fn get_velocity(from: &[f64; 3], to: &[f64; 3]) -> f64 {
    let km = get_distance(from, to) / 1000.0;
    let h = (to[2] - from[2]) / (3600. * 1000.);
    km / h
}

fn get_distance(from: &[f64; 3], to: &[f64; 3]) -> f64 {
    // Returns haversine distance in meters
    let start = geo::point!(x: from[0],y: from[1]);
    let end = geo::point!(x:to[0], y:to[1]);
    start.haversine_distance(&end)
}

fn avg_velocity(window: Vec<[f64; 3]>) -> f64 {
    let mut vels: Vec<f64> = Vec::new();
    for i in 1..(window.len() - 1) {
        let v = get_velocity(&window[i - 1], &window[i]);
        vels.push(v);
    }
    vels.iter().sum::<f64>() / (vels.len() as f64)
}
