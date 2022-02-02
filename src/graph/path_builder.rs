use crate::graph::{Path, PathBuilderStats, PathElement};
use crate::utility::IsStopped;
use crate::utility::{Bbox, MotionDetector, StopDetector};
use crate::CONFIG;

/// Returns a vector of paths given a stream
///
/// The stream is split between two points if their temporal difference exceeds
/// `connection_timeout` as it is set in the [config](Config) file.
/// Then, paths are constructed from the stream using a [stop detector](StopDetector).
pub fn get_paths(stream: Vec<[f64; 3]>, stats: &mut PathBuilderStats) -> Vec<Path> {
    stats.streams_handled += 1;
    let splitted_streams = split_stream_on_timeout(&stream, stats);
    let paths: Vec<Path> = splitted_streams
        .into_iter()
        .map(|stream| {
            let path = build_path(stream);
            stats.path_lens.push(path.len());
            path
        })
        .collect();
    paths
}

/// Classify stops and routes of a stream
///
/// Should be called after `split_stream`.
fn build_path(stream: Vec<[f64; 3]>) -> Path {
    // This function should be called after split_stream
    let mut sd = StopDetector::new();
    let mut md = MotionDetector::new();
    let mut builder: PathBuilder = PathBuilder::new();
    stream.into_iter().for_each(|point| {
        let md_stop: IsStopped = md.is_stopped(&point);
        // The stop detector switches to `IsStopped::No` when its spatial
        // limit is exceeded. Then, it is `reset` once the motion detector
        // senses movement speed is above a fixed threshold.
        // Note that resetting sd result in is_stopped() => No
        match md_stop {
            IsStopped::Maybe | IsStopped::Yes => (),
            IsStopped::No => sd.reset(),
        }
        let sd_stop: IsStopped = sd.is_stopped(&point);
        builder.add_pt(point, sd_stop);
    });
    let path = builder.get_path();
    path.verify();
    path
}

fn split_stream_on_timeout(
    stream: &[[f64; 3]],
    stats: &mut PathBuilderStats,
) -> Vec<Vec<[f64; 3]>> {
    let connection_timeout = CONFIG.connection_timeout;
    let mut last_timestamp = stream[0][2];
    let mut result = vec![];
    let mut partial_result = vec![];
    let mut timeouts = vec![];
    for point in stream {
        let current_timestamp = point[2];
        if current_timestamp - last_timestamp > connection_timeout {
            result.push(partial_result);
            partial_result = vec![*point];
            timeouts.push(current_timestamp - last_timestamp);
        } else {
            partial_result.push(*point);
        }
        last_timestamp = current_timestamp;
    }
    result.push(partial_result);
    stats.timeouts.append(&mut timeouts);
    result
}

#[derive(Clone, Debug)]
/// Stores points until we can classify them as either `Stop` or `Route`
struct PointsForElement {
    pts: Vec<[f64; 3]>,
}

impl PointsForElement {
    fn push(&mut self, point: [f64; 3]) {
        self.pts.push(point);
    }

    fn to_stop(&self) -> PathElement {
        PathElement::new_stop(&self.pts)
    }

    fn to_route(&self) -> PathElement {
        PathElement::new_route(&self.pts)
    }

    /// Remove all points
    fn reset(&mut self) {
        self.pts = vec![];
    }
}

struct PathBuilder {
    path: Path,
    trj: Vec<[f64; 3]>,
    bbox: Option<Bbox>,
    building_initial_stop: bool,
}

impl PathBuilder {
    pub fn new() -> PathBuilder {
        PathBuilder {
            path: Path::new(),
            trj: vec![],
            bbox: None,
            building_initial_stop: true,
        }
    }
    /// All points are added to a trajectory.
    /// Initially, a stop contain a single point p.
    /// Routes connected to a stop start/end with p.
    /// Stops are expanded in `finalize_path`.
    fn add_pt(&mut self, point: [f64; 3], is_stopped: IsStopped) {
        // add point to the trj
        self.trj.push(point);
        if self.building_initial_stop {
            // We're building the initial stop
            if let Some(bbox) = self.bbox {
                let mut tmp_bbox = bbox;
                tmp_bbox.insert_point(&point);
                if tmp_bbox.verify_spatial() {
                    self.bbox = Some(tmp_bbox);
                } else {
                    let stop = PathElement::Stop(bbox);
                    self.bbox = None;
                    self.path.push(stop);
                    self.building_initial_stop = false;
                }
            } else {
                self.bbox = Some(Bbox::new(&[point]));
            }
        } else {
            match is_stopped {
                IsStopped::Maybe => {
                    // Try adding to current bbox if it exists
                    // if its full push to path and set bbox to none.
                    if let Some(bbox) = self.bbox {
                        let mut tmp_bbox = bbox;
                        tmp_bbox.insert_point(&point);
                        if tmp_bbox.verify_spatial() {
                            self.bbox = Some(tmp_bbox);
                        } else {
                            // insert route leading to this stop
                            let last_point = self.trj.pop().unwrap();
                            let route = PathElement::Route(self.trj.clone());
                            self.path.push(route);
                            // Save the stop to path and clear bbox.
                            self.trj = vec![last_point, point];
                            let stop = PathElement::Stop(bbox);
                            self.bbox = None;
                            self.path.push(stop);
                        }
                    }
                }
                IsStopped::Yes => {
                    if let Some(mut bbox) = self.bbox {
                        bbox.insert_point(&point);
                        if bbox.verify_spatial() {
                            self.bbox = Some(bbox)
                        } else {
                            panic!(
                                "Odd, this shouldn't happen!\npath: {}\npoint:{:?}\nbbox:{}",
                                self.path, point, bbox
                            );
                        }
                    } else {
                        let bbox = Bbox::new(&[point]);
                        self.bbox = Some(bbox);
                        if !self.path.is_empty() {
                            let route = PathElement::Route(self.trj.clone());
                            self.path.push(route);
                            self.trj = vec![point];
                        } else {
                            let stop = PathElement::Stop(bbox);
                            self.path.push(stop);
                        }
                    }
                }
                IsStopped::No => {
                    if let Some(bbox) = self.bbox {
                        let stop = PathElement::Stop(bbox);
                        self.path.push(stop);
                    }
                    self.bbox = None;
                }
            }
        }
    }

    fn finalize_path(&mut self) {
        if let Some(last_elm) = self.path.last_element() {
            // Ensure path ends with a `Stop`
            if let PathElement::Route(trj) = last_elm {
                let start = trj[0];
                let end = trj[trj.len() - 1];
                let bbox = self.bbox.unwrap();
                if bbox.contains_point(&start) & bbox.contains_point(&end) {
                    // the current route is completely contained in Stop
                    // so remove it from the path
                    self.path.remove_last();
                } else {
                    self.path.push(PathElement::Stop(self.bbox.unwrap()));
                }
            } else if let PathElement::Stop(_) = last_elm {
                let trj = self.trj.clone();
                let end = trj[trj.len() - 1];
                let route = PathElement::Route(trj);
                self.path.push(route);
                let stop = if let Some(bbox) = self.bbox {
                    PathElement::Stop(bbox)
                } else {
                    // Construct degenerate stop
                    PathElement::Stop(Bbox::new(&[end]))
                };
                self.path.push(stop);
            }
            if self.path.len() > 1 {
                self.path.expand_stops();
            }
        } else if self.trj.len() > 5 {
            // When a certain amount of points have been used we don't ignore the stop
            self.path.push(PathElement::Stop(self.bbox.unwrap()));
        }
    }

    fn get_path(&mut self) -> Path {
        self.finalize_path();
        self.path.merge_nodes();
        self.path.rm_single_points();
        self.path.verify();
        self.path.clone()
    }
}

#[cfg(test)]
mod test {
    use super::*;

    #[test]
    fn split_stream() {
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
        let mut stats = PathBuilderStats::default();
        let streams = split_stream_on_timeout(&stream, &mut stats);
        assert_eq!(streams.len(), 2);
        assert_eq!(streams[0].len(), 4);
        assert_eq!(streams[1].len(), 6);
    }

    #[test]
    fn new() {
        let config = Config::default();
        let pb = PathBuilder::new(&config);
        assert!(pb.path.is_empty());
        assert!(pb.path_element.is_none());
    }

    #[test]
    fn add_point_stopped() {
        let config = Config::default();
        let mut pb = PathBuilder::new(&config);
        pb.add_pt([1., 1., 2.], false);
        assert!(pb.path.is_empty());
        assert!(pb.path_element.is_some());
    }

    #[test]
    fn add_point_moving() {
        let config = Config::default();
        let mut pb = PathBuilder::new(&config);
        pb.add_pt([1., 1., 2.], true);
        assert_eq!(pb.path.len(), 1);
        assert!(pb.path_element.is_some());
    }

    #[test]
    fn add_point_alternate_moving() {
        let config = Config::default();
        let mut pb = PathBuilder::new(&config);
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
