use crate::graph::{Path, PathBuilderStats, PathElement};
use crate::utility::IsStopped;
use crate::utility::{Bbox, MotionDetector, StopDetector};
use crate::Config;

/// Returns a vector of paths given a stream
///
/// The stream is split between two points if their temporal difference exceeds
/// `connection_timeout` as it is set in the [config](Config) file.
/// Then, paths are constructed from the stream using a [stop detector](StopDetector).
pub fn get_paths(
    stream: Vec<[f64; 3]>,
    config: &Config,
    stats: &mut PathBuilderStats,
) -> Vec<Path> {
    stats.streams_handled += 1;
    let splitted_streams = split_stream_on_timeout(&stream, config.connection_timeout, stats);
    let paths: Vec<Path> = splitted_streams
        .into_iter()
        .map(|stream| {
            let path = build_path(stream, *config);
            stats.path_lens.push(path.len());
            path
        })
        .collect();
    paths
}

/// Classify stops and routes of a stream
///
/// Should be called after `split_stream`.
fn build_path(stream: Vec<[f64; 3]>, config: Config) -> Path {
    // This function should be called after split_stream
    let mut sd = StopDetector::new(&config);
    let mut md = MotionDetector::new(&config);
    let mut builder: PathBuilder = PathBuilder::new(&config);
    stream.into_iter().for_each(|point| {
        let sd_stop: IsStopped = sd.is_stopped(&point);
        let md_stop: IsStopped = md.is_stopped(&point);
        let is_stopped: IsStopped = match (sd_stop, md_stop) {
            (IsStopped::Maybe, IsStopped::Maybe) => IsStopped::Maybe,
            (IsStopped::Maybe, IsStopped::Yes) => IsStopped::Maybe,
            (IsStopped::Maybe, IsStopped::No) => IsStopped::Maybe,
            (IsStopped::Yes, IsStopped::Maybe) => IsStopped::Maybe,
            (IsStopped::Yes, IsStopped::Yes) => {
                sd.reset();
                IsStopped::Yes
            }
            (IsStopped::Yes, IsStopped::No) => IsStopped::Maybe,
            (IsStopped::No, IsStopped::Maybe) => IsStopped::Maybe,
            (IsStopped::No, IsStopped::Yes) => IsStopped::Maybe,
            (IsStopped::No, IsStopped::No) => IsStopped::No,
        };
        builder.add_pt(point, is_stopped)
    });
    let path = builder.get_path();
    path.verify();
    path
}

fn split_stream_on_timeout(
    stream: &[[f64; 3]],
    connection_timeout: f64,
    stats: &mut PathBuilderStats,
) -> Vec<Vec<[f64; 3]>> {
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

    /// Remove all but the last point s.t. the next element is connected to the previous
    fn reset(&mut self) {
        let last = self.pts.len() - 1;
        self.pts = vec![self.pts[last]];
    }
}

struct PathBuilder<'a> {
    path: Path,
    points: PointsForElement,
    config: &'a Config,
}

impl<'a> PathBuilder<'a> {
    pub fn new(config: &'a Config) -> PathBuilder {
        PathBuilder {
            path: Path::new(),
            points: PointsForElement { pts: vec![] },
            config,
        }
    }

    /// If we don't know if we are building a `PathElement`
    /// then points are stored in `path_element`.
    /// Otherwise points are added to the last element of `path`.
    /// Regardless, we always let Path start with a `Stop`.
    fn add_pt(&mut self, pt: [f64; 3], is_stopped: IsStopped) {
        if self.path.is_empty() {
            self.points.push(pt);
            self.path.push(PathElement::Stop(Bbox::new(&[pt])));
        } else {
            match is_stopped {
                IsStopped::Maybe => self.points.push(pt),
                IsStopped::Yes => self.add_to_stop(pt),
                IsStopped::No => self.add_to_route(pt),
            }
        }
    }

    fn add_to_stop(&mut self, pt: [f64; 3]) {
        let last_element = self.path.last_element().unwrap();
        match last_element {
            PathElement::Stop(mut bbox) => bbox.insert_point(&pt),
            PathElement::Route(_) => {
                self.points.push(pt);
                let stop = self.points.to_stop();
                self.points.reset();
                self.path.push(stop);
            }
        }
    }

    fn add_to_route(&mut self, pt: [f64; 3]) {
        let last_element = self.path.last_element().unwrap();
        match last_element {
            PathElement::Stop(_) => {
                self.points.push(pt);
                let route = self.points.to_route();
                self.points.reset();
                self.path.push(route);
            }
            PathElement::Route(mut trj) => trj.push(pt),
        }
    }

    fn finalize_path(&mut self) {
        let last_elm = self.path.last_element().unwrap();
        if let PathElement::Route(_) = last_elm {
            let pts = self.points.pts.clone();
            self.path.add_points(&pts);
            let last_pt = pts[pts.len() - 1];
            let stop = PathElement::Stop(Bbox::new(&[last_pt]));
            self.path.push(stop);
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
        let pb = PathBuilder::new();
        assert!(pb.path.is_empty());
        assert!(pb.path_element.is_none());
    }

    #[test]
    fn add_point_stopped() {
        let mut pb = PathBuilder::new();
        pb.add_pt([1., 1., 2.], false);
        assert!(pb.path.is_empty());
        assert!(pb.path_element.is_some());
    }

    #[test]
    fn add_point_moving() {
        let mut pb = PathBuilder::new();
        pb.add_pt([1., 1., 2.], true);
        assert_eq!(pb.path.len(), 1);
        assert!(pb.path_element.is_some());
    }

    #[test]
    fn add_point_alternate_moving() {
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
