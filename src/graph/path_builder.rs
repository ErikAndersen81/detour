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
        let sd_stop: IsStopped = sd.is_stopped(&point);
        let md_stop: IsStopped = md.is_stopped(&point);
        // The stop detector switches to `IsStopped::No` when its spatial
        // limit is exceeded. Then, it is `reset` once the motion detector
        // senses movement speed is above a fixed threshold.
        // Note that classification of `IsStopped::Maybe` is postponed
        // until either a `IsStopped::Yes` or `IsStopped::No`
        // is sent to `PathBuilder`.
        match md_stop {
            IsStopped::Maybe | IsStopped::Yes => (),
            IsStopped::No => sd.reset(),
        }
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
    last_point_in_stop: Option<[f64; 3]>,
    points: PointsForElement,
}

impl PathBuilder {
    pub fn new() -> PathBuilder {
        PathBuilder {
            path: Path::new(),
            last_point_in_stop: None,
            points: PointsForElement { pts: vec![] },
        }
    }
    /// If we don't know if we are building a `PathElement`
    /// then points are stored in `points`.
    /// Otherwise points are added to the last element of `path`.
    /// `last_point_in_stop` keeps track of the last point added to a stop,
    /// s.t. we can start a subsequent route inside the stop.
    /// Regardless, we always let `path` start with a stop.
    fn add_pt(&mut self, pt: [f64; 3], is_stopped: IsStopped) {
        if self.path.is_empty() {
            self.path.push(PathElement::Stop(Bbox::new(&[pt])));
            self.last_point_in_stop = Some(pt);
        } else {
            match is_stopped {
                IsStopped::Maybe => self.points.push(pt),
                IsStopped::Yes => self.add_to_stop(pt),
                IsStopped::No => self.add_to_route(pt),
            }
        }
    }

    fn handle_points(&mut self, is_stopped: IsStopped) {
        let last_element = self.path.last_element().unwrap();
        match last_element {
            PathElement::Stop(bbox) => {
                // Expand the stop as much as possible
                let mut points = self.points.pts.iter();
                let mut tmp_bbox = bbox;
                while tmp_bbox.verify_spatial() {
                    if let Some(point) = points.next() {
                        tmp_bbox.insert_point(point);
                        self.last_point_in_stop = Some(*point);
                    } else {
                        break;
                    }
                }
                self.path.replace_last_bbox(tmp_bbox);
                // If there are points left, use them to build a new route
                for point in points {
                    if let Some(last_element) = self.path.last_element() {
                        match last_element {
                            PathElement::Stop(_) => {
                                // Create a new route
                                let route =
                                    PathElement::Route(vec![self.last_point_in_stop.unwrap()]);
                                self.last_point_in_stop = None;
                                self.path.push(route);
                            }
                            PathElement::Route(_) => self.path.add_points(&[*point]),
                        }
                    }
                }
            }
            PathElement::Route(_) => {
                if matches!(is_stopped, IsStopped::Yes) {
                    // Create stop with the last point, expanding it backwards
                    let mut points = self.points.pts.clone();
                    points.reverse();
                    let mut points = points.iter();
                    let last_point = *points.next().unwrap();
                    self.last_point_in_stop = Some(last_point);
                    let mut bbox = Bbox::new(&[last_point]);
                    let mut connect_point = last_point;
                    while bbox.verify_spatial() {
                        if let Some(point) = points.next() {
                            bbox.insert_point(point);
                            connect_point = *point;
                        } else {
                            break;
                        }
                    }
                    let mut points = points.copied().collect::<Vec<[f64; 3]>>();
                    points.reverse();
                    self.path.add_points(&points);
                    self.path.add_points(&[connect_point]);
                    self.path.push(PathElement::Stop(bbox));
                } else {
                    self.path.add_points(&self.points.pts);
                }
            }
        }
        self.points.reset();
    }

    fn add_to_stop(&mut self, pt: [f64; 3]) {
        if !self.points.pts.is_empty() {
            self.handle_points(IsStopped::Yes);
        }
        let last_element = self.path.last_element().unwrap();
        match last_element {
            PathElement::Stop(bbox) => {
                // Continue building stop
                let mut tmp_bbox = bbox;
                tmp_bbox.insert_point(&pt);
                // Check if adding the point forms a valid bbox
                if !tmp_bbox.verify_spatial() {
                    panic!("Warning: bbox exceeds limit!! {}", tmp_bbox);
                } else {
                    self.last_point_in_stop = Some(pt);
                    self.path.add_points(&[pt]);
                }
            }
            PathElement::Route(_) => {
                // Create a new stop
                // First ensure it's connected to the route leading here
                self.path.add_points(&[pt]);
                // Then create and insert the new stop
                let stop = PathElement::Stop(Bbox::new(&[pt]));
                self.path.push(stop);
                self.last_point_in_stop = Some(pt);
            }
        }
    }

    fn add_to_route(&mut self, pt: [f64; 3]) {
        if !self.points.pts.is_empty() {
            self.handle_points(IsStopped::No);
        }
        let last_element = self.path.last_element().unwrap();
        match last_element {
            PathElement::Stop(_) => {
                // Create a new route
                // and ensure that we are connected to the previous stop
                let lst_pt = self.last_point_in_stop.unwrap();
                let route = PathElement::Route(vec![lst_pt, pt]);
                self.path.push(route);
            }
            PathElement::Route(_) => self.path.add_points(&[pt]),
        }
    }

    fn finalize_path(&mut self) {
        let last_elm = self.path.last_element().unwrap();
        if let PathElement::Route(trj) = last_elm {
            if !self.points.pts.is_empty() {
                let pts = self.points.pts.clone();
                self.path.add_points(&pts);
                let last_pt = pts[pts.len() - 1];
                // Create a new stop and expand is as much as possible
                let mut bbox = Bbox::new(&[last_pt]);
                let mut pts = pts;
                pts.reverse();
                let mut tmp_bbox = bbox;
                for pt in pts {
                    tmp_bbox.insert_point(&pt);
                    if tmp_bbox.verify_spatial() {
                        break;
                    } else {
                        bbox = tmp_bbox;
                    }
                }
                self.path.push(PathElement::Stop(bbox));
            } else {
                let last_pt = trj[trj.len() - 1];
                let stop = PathElement::Stop(Bbox::new(&[last_pt]));
                self.path.push(stop);
            }
        } else if let PathElement::Stop(_) = last_elm {
            // If there's only one stop but several unclassified points,
            // try and create a route of the points, or alternatively
            // add them to the stop.
            if (self.path.len() < 2) & (!self.points.pts.is_empty()) {
                // Create a new route starting in the last stop
                let route = PathElement::Route(vec![self.last_point_in_stop.unwrap()]);
                self.path.push(route);
                // Add the unclassified points
                self.path.add_points(&self.points.pts);
                // create a stop wth the last element of unclassified points
                let pt = self.points.pts[self.points.pts.len() - 1];
                let stop = PathElement::Stop(Bbox::new(&[pt]));
                self.path.push(stop);
            }
        }
    }

    fn get_path(&mut self) -> Path {
        self.finalize_path();
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
        let streams = split_stream_on_timeout(&stream, connection_timeout, &mut stats);
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
