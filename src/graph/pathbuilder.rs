use crate::utility::{trajectory::Monotone, Bbox, MotionDetector, StopDetector};
use crate::Config;
use std::fmt;

#[derive(Clone, Debug)]
pub enum PathElement {
    Stop(Bbox),
    Route(Vec<[f64; 3]>),
}

impl PathElement {
    fn is_stop(&self) -> bool {
        matches!(self, &PathElement::Stop(_))
    }

    pub fn get_bbox(&self) -> Option<Bbox> {
        if let PathElement::Stop(bbox) = self {
            Some(*bbox)
        } else {
            None
        }
    }

    pub fn get_trj(&self) -> Option<Vec<[f64; 3]>> {
        if let PathElement::Route(trj) = self {
            Some(trj.clone())
        } else {
            None
        }
    }

    pub fn stop_from(points: &[[f64; 3]]) -> PathElement {
        let (x, y, t1, t2) = points.iter().map(|[x, y, t]| (*x, *y, *t)).fold(
            (0., 0., f64::INFINITY, f64::NEG_INFINITY),
            |(x_acc, y_acc, t1_acc, t2_acc), (x, y, t)| {
                (x_acc + x, y_acc + y, t1_acc.min(t), t2_acc.max(t))
            },
        );
        let n = points.len() as f64;
        let (x, y) = (x / n, y / n);
        let bbox = Bbox::new(&[[x, y, t1], [x, y, t2]]);
        PathElement::Stop(bbox)
    }
}

pub type Path = Vec<PathElement>;

trait PathTrait {
    fn verify(&self) -> bool;
}

impl PathTrait for Path {
    fn verify(&self) -> bool {
        let mut is_stop = false;
        for path_element in self {
            is_stop = !is_stop;
            if is_stop ^ path_element.is_stop() {
                return false;
            }
        }
        self[self.len() - 1].is_stop()
    }
}

/// Returns a vector of paths given a stream
///
/// The stream is split between two points if their temporal difference exceeds
/// `connection_timeout` as it is set in the [config](Config) file.
/// Then, paths are constructed from the stream using a [stop detector](StopDetector).
pub fn get_paths(stream: Vec<[f64; 3]>, config: &Config) -> Vec<Path> {
    let splitted_streams = split_stream_on_timeout(&stream, config.connection_timeout);
    let paths: Vec<Path> = splitted_streams
        .into_iter()
        .map(|stream| build_path(stream, *config))
        .collect();
    if paths.is_empty() {
        println!("No paths could be created for given stream:\n{:?}", stream);
    }
    paths
}

fn build_path(stream: Vec<[f64; 3]>, config: Config) -> Path {
    // This function should be called after split_stream
    let mut sd = StopDetector::new(&config);
    let mut md = MotionDetector::new(&config);
    let mut builder: PathBuilder = PathBuilder::new();
    stream
        .into_iter()
        .for_each(|point| builder.add_pt(point, !sd.is_stopped(point) & !md.is_stopped(point)));
    builder.get_path()
}

fn split_stream_on_timeout(stream: &[[f64; 3]], connection_timeout: f64) -> Vec<Vec<[f64; 3]>> {
    let mut last_timestamp = stream[0][2];
    let mut result = vec![];
    let mut partial_result = vec![];
    for point in stream {
        let current_timestamp = point[2];
        if current_timestamp - last_timestamp > connection_timeout {
            result.push(partial_result);
            partial_result = vec![*point];
        } else {
            partial_result.push(*point);
        }
        last_timestamp = current_timestamp;
    }
    result.push(partial_result);
    result
}

#[derive(Clone, Debug)]
enum PointsForElement {
    Stop(Vec<[f64; 3]>),
    Route(Vec<[f64; 3]>),
}

impl PointsForElement {
    fn to_path_element(&self) -> PathElement {
        match self {
            PointsForElement::Stop(pts) => PathElement::stop_from(pts),
            PointsForElement::Route(pts) => PathElement::Route(pts.clone()),
        }
    }

    fn is_stop(&self) -> bool {
        matches!(self, &PointsForElement::Stop(_))
    }

    fn add_point(&mut self, point: [f64; 3]) {
        match self {
            PointsForElement::Stop(pts) => pts.push(point),
            PointsForElement::Route(pts) => pts.push(point),
        };
    }

    fn get_points(&self) -> Vec<[f64; 3]> {
        match self {
            PointsForElement::Stop(pts) => pts.clone(),
            PointsForElement::Route(pts) => pts.clone(),
        }
    }
}

struct PathBuilder {
    path: Path,
    path_element: Option<PointsForElement>,
}

impl PathBuilder {
    pub fn new() -> PathBuilder {
        PathBuilder {
            path: vec![],
            path_element: None,
        }
    }

    fn add_pt(&mut self, pt: [f64; 3], is_moving: bool) {
        if self.path_element.is_none() {
            let path_element = PointsForElement::Stop(vec![pt]);
            self.path_element = Some(path_element);
        }
        let mut path_element = self.path_element.clone().unwrap();
        path_element.add_point(pt);
        match (is_moving, &path_element) {
            (true, PointsForElement::Stop(_)) => {
                self.path.push(path_element.to_path_element());
                let path_element = PointsForElement::Route(vec![pt]);
                self.path_element = Some(path_element);
            }
            (false, PointsForElement::Route(_)) => {
                self.path.push(path_element.to_path_element());
                let path_element = PointsForElement::Stop(vec![pt]);
                self.path_element = Some(path_element);
            }
            (_, _) => {
                self.path_element = Some(path_element);
            }
        }
    }

    fn finalize_path(&mut self) {
        if let Some(points_for_element) = &self.path_element {
            let path_element = points_for_element.clone().to_path_element();
            self.path.push(path_element);
            if !points_for_element.is_stop() {
                // We shouldn't end in the middle of a trajectory
                // so we construct a degenerate Bbox
                let trj = points_for_element.get_points();
                let path_element = PathElement::Stop(Bbox::new(&[trj[trj.len() - 1]]));
                self.path.push(path_element);
            }
        }

        // We don't want trajectories containing only a few points
        // if this is the case, we merge the two bboxes at each side
        // and remove the trajectory
        self.path
            .clone()
            .iter()
            .enumerate()
            .filter(|(_, e)| {
                if let PathElement::Route(trj) = *e {
                    if trj.len() < 4 {
                        return true;
                    }
                }
                false
            })
            .map(|(idx, _)| (idx))
            .rev()
            .for_each(|idx| self.cut_route(idx));
        // ensure the trajectories in the path are monotone
        self.path = self
            .path
            .clone()
            .into_iter()
            .map(|e| {
                if let PathElement::Route(trj) = e {
                    let trj = trj.make_monotone();
                    PathElement::Route(trj)
                } else {
                    e
                }
            })
            .collect();
    }

    fn cut_route(&mut self, idx: usize) {
        self.path.remove(idx).get_trj().unwrap();
        let bbox = self.path.remove(idx).get_bbox().unwrap();
        let other = self.path.remove(idx - 1).get_bbox().unwrap();
        let bbox = bbox.union(&other);
        self.path.insert(idx - 1, PathElement::Stop(bbox));
    }

    fn get_path(&mut self) -> Path {
        self.finalize_path();
        self.path.clone()
    }
}

#[derive(Debug, Clone)]
pub struct PathBuilderError;

impl fmt::Display for PathBuilderError {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Warning: Path ended during the construction of a trajectory. Degenerate vertex inserted.")
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn split_stream_test() {
        let stream = vec![
            [0., 0., 1.],
            [0., 0., 2.],
            [0., 0., 3.],
            [0., 0., 4.],
            [0., 0., 9.],
            [0., 0., 10.],
            [0., 0., 11.],
            [0., 0., 12.],
            [0., 0., 13.],
            [0., 0., 14.],
        ];
        let connection_timeout = 3.0;
        let streams = split_stream_on_timeout(&stream, connection_timeout);
        assert_eq!(streams.len(), 2);
        assert_eq!(streams[0].len(), 4);
        assert_eq!(streams[1].len(), 6);
    }

    #[test]
    fn new_test() {
        let pb = PathBuilder::new();
        assert!(pb.path.is_empty());
        assert!(pb.path_element.is_none());
    }

    #[test]
    fn add_point_stopped_test() {
        let mut pb = PathBuilder::new();
        pb.add_pt([1., 1., 2.], false);
        assert!(pb.path.is_empty());
        assert!(pb.path_element.is_some());
    }

    #[test]
    fn add_point_moving_test() {
        let mut pb = PathBuilder::new();
        pb.add_pt([1., 1., 2.], true);
        assert_eq!(pb.path.len(), 1);
        assert!(pb.path_element.is_some());
    }

    #[test]
    fn add_point_alternate_moving_test() {
        let mut pb = PathBuilder::new();
        pb.add_pt([0., 0., 0.], false);
        pb.add_pt([1., 1., 1.], true);
        pb.add_pt([2., 2., 2.], false);
        pb.add_pt([3., 3., 3.], true);
        pb.add_pt([4., 4., 4.], false);
        pb.add_pt([5., 5., 5.], true);
        pb.add_pt([6., 6., 6.], false);
        assert_eq!(pb.path.len(), 6, "{:?}", pb.path);
    }

    #[test]
    fn verify_test() {
        let trj: Vec<[f64; 3]> = vec![];
        let bbox = Bbox::new(&[[0., 0., 0.]]);
        let path: Vec<PathElement> = vec![
            PathElement::Stop(bbox),
            PathElement::Route(trj.clone()),
            PathElement::Stop(bbox),
            PathElement::Route(trj),
            PathElement::Stop(bbox),
        ];
        assert!(path.verify());
    }
}
