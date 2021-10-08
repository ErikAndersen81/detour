use crate::trajectory::{get_common_time_span, Coord2D, Coord3D, Line2D, Trajectory};

pub fn proj_dist(trj_a: &Trajectory, trj_b: &Trajectory, axis: usize) -> f64 {
    // Note: Trajectories must be trimmed before calling
    let mut events = get_event_queue(trj_a, trj_b, axis);
    let mut points: [Coord2D; 2] = [events.pop().unwrap().0, events.pop().unwrap().0];
    let mut intersect_flag = false;
    let mut area = 0.0;
    while !events.is_empty() {
        if intersect_flag {
            // previous event was an intersection point
            intersect_flag = false;
            let tmp = events.pop().unwrap().0;
            points[1] = events.pop().unwrap().0;
            area += calc_area(&points[0], &points[1], &tmp);
            points[0] = tmp;
        } else {
            let event = events.pop().unwrap();
            if event.1 {
                // event is an intersection point
                intersect_flag = true;
                area += calc_area(&points[0], &points[1], &(event.0));
                points[0] = event.0;
            } else {
                // normal point
                area += calc_area(&points[0], &points[1], &(event.0));
                points[0] = points[1].clone();
                points[1] = event.0;
            }
        }
    }
    area
}

pub fn distance(trj_a: &Trajectory, trj_b: &Trajectory) -> Option<f64> {
    // Returns TraDis of two trajectories.
    let span = get_common_time_span(trj_a, trj_b);
    match span {
        Some((start, end)) => {
            let trj_a = trj_a.trim(start, end);
            let trj_b = trj_b.trim(start, end);
            Some(proj_dist(&trj_a, &trj_b, 0) + proj_dist(&trj_a, &trj_b, 1))
        }
        None => None,
    }
}

fn get_event_queue(trj_a: &Trajectory, trj_b: &Trajectory, axis: usize) -> Vec<(Coord2D, bool)> {
    let coords_a = trj_a.get_projection(axis);
    let coords_b = trj_b.get_projection(axis);
    let mut events: Vec<(Coord2D, bool)> = Vec::new();
    let (mut i, mut j): (usize, usize) = (1, 1);
    while (i < trj_a.len()) & (j < trj_b.len()) {
        let a1 = &coords_a[i - 1];
        let a2 = &coords_a[i];
        let b1 = &coords_b[j - 1];
        let b2 = &coords_b[j];
        let line_a = Line2D { start: a1, end: a2 };
        let line_b = Line2D { start: b1, end: b2 };
        let inc_i;
        let (fst, snd): (Coord2D, Coord2D);
        if a1.y < b1.y {
            fst = a1.clone();
            snd = b1.clone();
            i = if i == trj_a.len() { i } else { i + 1 };
            inc_i = true;
        } else {
            fst = b1.clone();
            snd = a1.clone();
            j = if j == trj_b.len() { j } else { j + 1 };
            inc_i = false;
        };
        let ints = intersection_point(&line_a, &line_b);
        if let Some(ints) = ints {
            if inc_i {
                j = if j == trj_b.len() { j } else { j + 1 };
            } else {
                i = if i == trj_a.len() { i } else { i + 1 };
            }
            events.push((fst, false));
            events.push((snd, false));
            events.push((ints, true));
        } else {
            events.push((fst, false));
        }
    }
    let end_a = &coords_a[i - 1];
    let end_b = &coords_b[j - 1];
    events.push((end_a.clone(), false));
    events.push((end_b.clone(), false));
    events
}

fn calc_area(p: &Coord2D, q: &Coord2D, r: &Coord2D) -> f64 {
    0.5 * (p.x * (q.y - r.y) + q.x * (r.y - p.y) + r.x * (p.y - q.y)).abs()
}

pub fn interpolate(t: f64, p: &Coord3D, q: &Coord3D) -> Coord3D {
    // It must hold that p.t <= t <= q.t
    let dx = q.x - p.x;
    let dy = q.y - p.y;
    let s = (t - p.t) / (q.t - p.t);
    Coord3D {
        x: p.x + dx * s,
        y: p.y + dy * s,
        t,
    }
}

fn intersection_point(p: &Line2D, q: &Line2D) -> Option<Coord2D> {
    let (x1, y1, x2, y2) = (p.start.x, p.start.y, p.end.x, p.end.y);
    let (x3, y3, x4, y4) = (q.start.x, q.start.y, q.end.x, q.end.y);
    let denom: f64 = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    let t: f64 = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
    let u: f64 = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom;
    if (0.0..=1.0).contains(&t) {
        return Some(Coord2D {
            x: x1 + t * (x2 - x1),
            y: y1 + t * (y2 - y1),
        });
    } else if (0.0..=1.0).contains(&u) {
        return Some(Coord2D {
            x: x3 + u * (x4 - x3),
            y: y3 + u * (y4 - y3),
        });
    };
    None
}

//////////////////
// Test Section //
//////////////////

#[cfg(test)]
mod tradis_test {
    use super::*;

    const TRJ_A: [[f64; 3]; 68] = [
        [5.567963400000, 1.254953400000, 1.631162641494],
        [5.567959600000, 1.254969400000, 1.631162648677],
        [5.567955100000, 1.254988400000, 1.631162657204],
        [5.567952000000, 1.255001200000, 1.631162662968],
        [5.567940300000, 1.255050200000, 1.631162684986],
        [5.567937000000, 1.255063900000, 1.631162691150],
        [5.567916100000, 1.255049400000, 1.631162707619],
        [5.567904300000, 1.255042000000, 1.631162716804],
        [5.567881600000, 1.255027800000, 1.631162734469],
        [5.567857100000, 1.255012500000, 1.631162753531],
        [5.567831600000, 1.254997600000, 1.631162773234],
        [5.567798500000, 1.254976900000, 1.631162798991],
        [5.567795800000, 1.254975200000, 1.631162801093],
        [5.567774100000, 1.254962200000, 1.631162817902],
        [5.567771500000, 1.254977200000, 1.631162824395],
        [5.567762800000, 1.255016100000, 1.631162841711],
        [5.567744100000, 1.255004000000, 1.631162856319],
        [5.567762800000, 1.255016100000, 1.631162870927],
        [5.567771500000, 1.254977200000, 1.631162888243],
        [5.567774100000, 1.254962200000, 1.631162894736],
        [5.567795800000, 1.254975200000, 1.631162911545],
        [5.567798500000, 1.254976900000, 1.631162913647],
        [5.567831600000, 1.254997600000, 1.631162939404],
        [5.567840200000, 1.254960700000, 1.631162955925],
        [5.567848700000, 1.254923800000, 1.631162972419],
        [5.567853800000, 1.254901900000, 1.631162982223],
        [5.567855100000, 1.254885800000, 1.631162988953],
        [5.567856700000, 1.254870700000, 1.631162995310],
        [5.567863100000, 1.254843100000, 1.631163007658],
        [5.567866600000, 1.254828200000, 1.631163014337],
        [5.567868400000, 1.254820500000, 1.631163017786],
        [5.567869300000, 1.254816800000, 1.631163019453],
        [5.567872900000, 1.254801400000, 1.631163026351],
        [5.567883400000, 1.254753500000, 1.631163047616],
        [5.567865800000, 1.254741800000, 1.631163061409],
        [5.567863300000, 1.254740100000, 1.631163063373],
        [5.567850500000, 1.254731300000, 1.631163073447],
        [5.567848600000, 1.254730000000, 1.631163074941],
        [5.567838200000, 1.254722900000, 1.631163083119],
        [5.567822800000, 1.254712200000, 1.631163095257],
        [5.567817100000, 1.254708300000, 1.631163099740],
        [5.567808100000, 1.254702100000, 1.631163106825],
        [5.567804100000, 1.254699300000, 1.631163109980],
        [5.567808100000, 1.254702100000, 1.631163113135],
        [5.567817100000, 1.254708300000, 1.631163120220],
        [5.567822800000, 1.254712200000, 1.631163124703],
        [5.567838200000, 1.254722900000, 1.631163136841],
        [5.567848600000, 1.254730000000, 1.631163145019],
        [5.567850500000, 1.254731300000, 1.631163146513],
        [5.567863300000, 1.254740100000, 1.631163156587],
        [5.567865800000, 1.254741800000, 1.631163158551],
        [5.567883400000, 1.254753500000, 1.631163172344],
        [5.567925900000, 1.254781600000, 1.631163205630],
        [5.567931000000, 1.254784700000, 1.631163209586],
        [5.567933400000, 1.254786200000, 1.631163211453],
        [5.567954600000, 1.254800500000, 1.631163228098],
        [5.567964400000, 1.254808700000, 1.631163236050],
        [5.567969300000, 1.254812900000, 1.631163240043],
        [5.567974800000, 1.254817800000, 1.631163244559],
        [5.567977500000, 1.254843400000, 1.631163255335],
        [5.567982800000, 1.254873400000, 1.631163268343],
        [5.567979200000, 1.254888800000, 1.631163275241],
        [5.567977700000, 1.254895300000, 1.631163278147],
        [5.567975100000, 1.254904700000, 1.631163282479],
        [5.567973600000, 1.254911000000, 1.631163285308],
        [5.567972200000, 1.254916900000, 1.631163287956],
        [5.567969000000, 1.254930200000, 1.631163293939],
        [5.567963400000, 1.254953400000, 1.631163304381],
    ];

    const TRJ_B: [[f64; 3]; 60] = [
        [5.567850700000, 1.254707900000, 1.631162640000],
        [5.567850800000, 1.254707800000, 1.631162640083],
        [5.567848600000, 1.254730000000, 1.631162649409],
        [5.567850500000, 1.254731300000, 1.631162650903],
        [5.567863300000, 1.254740100000, 1.631162660977],
        [5.567865800000, 1.254741800000, 1.631162662941],
        [5.567883400000, 1.254753500000, 1.631162676734],
        [5.567925900000, 1.254781600000, 1.631162710020],
        [5.567931000000, 1.254784700000, 1.631162713976],
        [5.567933400000, 1.254786200000, 1.631162715843],
        [5.567954600000, 1.254800500000, 1.631162732488],
        [5.567964400000, 1.254808700000, 1.631162740440],
        [5.567969300000, 1.254812900000, 1.631162744433],
        [5.567974800000, 1.254817800000, 1.631162748949],
        [5.567977500000, 1.254843400000, 1.631162759725],
        [5.567982800000, 1.254873400000, 1.631162772733],
        [5.567979200000, 1.254888800000, 1.631162779631],
        [5.567977700000, 1.254895300000, 1.631162782537],
        [5.567975100000, 1.254904700000, 1.631162786869],
        [5.567973600000, 1.254911000000, 1.631162789698],
        [5.567972200000, 1.254916900000, 1.631162792346],
        [5.567969000000, 1.254930200000, 1.631162798329],
        [5.567963400000, 1.254953400000, 1.631162808771],
        [5.567959600000, 1.254969400000, 1.631162815954],
        [5.567955100000, 1.254988400000, 1.631162824481],
        [5.567952000000, 1.255001200000, 1.631162830245],
        [5.567940300000, 1.255050200000, 1.631162852263],
        [5.567937000000, 1.255063900000, 1.631162858427],
        [5.567915200000, 1.255151700000, 1.631162898123],
        [5.567829800000, 1.255088900000, 1.631162965970],
        [5.567813000000, 1.255078000000, 1.631162979098],
        [5.567816100000, 1.255064700000, 1.631162985053],
        [5.567820400000, 1.255045900000, 1.631162993448],
        [5.567831600000, 1.254997600000, 1.631163015058],
        [5.567840200000, 1.254960700000, 1.631163031579],
        [5.567848700000, 1.254923800000, 1.631163048073],
        [5.567853800000, 1.254901900000, 1.631163057877],
        [5.567855100000, 1.254885800000, 1.631163064607],
        [5.567834000000, 1.254873900000, 1.631163080856],
        [5.567829400000, 1.254871300000, 1.631163084399],
        [5.567808900000, 1.254860000000, 1.631163100153],
        [5.567805000000, 1.254857800000, 1.631163103156],
        [5.567772700000, 1.254840000000, 1.631163127978],
        [5.567805000000, 1.254857800000, 1.631163152800],
        [5.567808900000, 1.254860000000, 1.631163155803],
        [5.567829400000, 1.254871300000, 1.631163171557],
        [5.567834000000, 1.254873900000, 1.631163175100],
        [5.567855100000, 1.254885800000, 1.631163191349],
        [5.567856700000, 1.254870700000, 1.631163197706],
        [5.567863100000, 1.254843100000, 1.631163210054],
        [5.567866600000, 1.254828200000, 1.631163216733],
        [5.567868400000, 1.254820500000, 1.631163220182],
        [5.567869300000, 1.254816800000, 1.631163221849],
        [5.567872900000, 1.254801400000, 1.631163228747],
        [5.567883400000, 1.254753500000, 1.631163250012],
        [5.567925900000, 1.254781600000, 1.631163283298],
        [5.567931000000, 1.254784700000, 1.631163287254],
        [5.567933400000, 1.254786200000, 1.631163289121],
        [5.567929600000, 1.254803000000, 1.631163296611],
        [5.567927400000, 1.254813400000, 1.631163301207],
    ];

    #[test]
    fn distance_symmetry() {
        let trj_a: Trajectory = Trajectory::from_array((&TRJ_A).to_vec());
        let trj_b: Trajectory = Trajectory::from_array((&TRJ_B).to_vec());
        let dist1 = distance(&trj_a, &trj_b).unwrap();
        let dist2 = distance(&trj_b, &trj_a).unwrap();
        assert!((dist1 - dist2).abs() < 0.000000001);
    }

    #[test]
    fn get_event_queue_test() {
        let trj_a: Trajectory = Trajectory::from_array((&TRJ_A).to_vec());
        let trj_b: Trajectory = Trajectory::from_array((&TRJ_B).to_vec());
        let (start, end) = get_common_time_span(&trj_a, &trj_b).unwrap();
        let trj_a = trj_a.trim(start, end);
        let trj_b = trj_b.trim(start, end);
        let q = get_event_queue(&trj_a, &trj_b, 0);
        let ys = q.iter().map(|p| p.0.y).collect::<Vec<f64>>();
        print!("{:?}", ys);
    }

    #[test]
    fn intersect_simple() {
        let p1 = Coord2D { x: 0.0, y: 0.0 };
        let p2 = Coord2D { x: 2.0, y: 2.0 };
        let q1 = Coord2D { x: 2.0, y: 0.0 };
        let q2 = Coord2D { x: 0.0, y: 2.0 };
        let r = intersection_point(
            &Line2D {
                start: &p1,
                end: &p2,
            },
            &Line2D {
                start: &q1,
                end: &q2,
            },
        );
        let r = r.unwrap();
        assert_eq!((r.x, r.y), (1.0, 1.0))
    }

    #[test]
    fn intersect_on_line() {
        let p1 = Coord2D { x: 0.0, y: 0.0 };
        let p2 = Coord2D { x: 2.0, y: 2.0 };
        let q1 = Coord2D { x: 2.0, y: 0.0 };
        let q2 = Coord2D { x: 1.0, y: 1.0 };
        let r = intersection_point(
            &Line2D {
                start: &p1,
                end: &p2,
            },
            &Line2D {
                start: &q1,
                end: &q2,
            },
        );
        let r = r.unwrap();
        assert_eq!((r.x, r.y), (1.0, 1.0))
    }

    #[test]
    fn intersect_parallel() {
        let p1 = Coord2D { x: 0.0, y: 0.0 };
        let p2 = Coord2D { x: 0.0, y: 2.0 };
        let q1 = Coord2D { x: 2.0, y: 0.0 };
        let q2 = Coord2D { x: 2.0, y: 2.0 };
        let r = intersection_point(
            &Line2D {
                start: &p1,
                end: &p2,
            },
            &Line2D {
                start: &q1,
                end: &q2,
            },
        );
        assert!(Option::is_none(&r))
    }

    #[test]
    fn interpolation_simple() {
        let p: Coord3D = Coord3D::from_array([0.0, 0.0, 0.0]);
        let q: Coord3D = Coord3D::from_array([2.0, 2.0, 2.0]);
        let ans: Coord3D = Coord3D::from_array([1.0, 1.0, 1.0]);
        let res = interpolate(1.0, &p, &q);
        assert_eq!((res.x, res.y, res.t), (ans.x, ans.y, ans.t));
    }

    #[test]
    fn interpolation_simple2() {
        let p: Coord3D = Coord3D::from_array([0.0, 0.0, 0.0]);
        let q: Coord3D = Coord3D::from_array([1.0, 1.0, 1.0]);
        let ans: Coord3D = Coord3D::from_array([0.5, 0.5, 0.5]);
        let res = interpolate(0.5, &q, &p);
        assert_eq!((res.x, res.y, res.t), (ans.x, ans.y, ans.t));
    }

    #[test]
    fn interpolation_offset() {
        let p: Coord3D = Coord3D::from_array([2.0, 2.0, 2.0]);
        let q: Coord3D = Coord3D::from_array([4.0, 4.0, 4.0]);
        let ans: Coord3D = Coord3D::from_array([3.0, 3.0, 3.0]);
        let res = interpolate(3.0, &p, &q);
        assert_eq!((res.x, res.y, res.t), (ans.x, ans.y, ans.t));
    }

    #[test]
    fn interpolation_single_dim() {
        let p: Coord3D = Coord3D::from_array([2.0, 2.0, 2.0]);
        let q: Coord3D = Coord3D::from_array([4.0, 2.0, 4.0]);
        let ans: Coord3D = Coord3D::from_array([3.0, 2.0, 3.0]);
        let res = interpolate(3.0, &p, &q);
        assert_eq!((res.x, res.y, res.t), (ans.x, ans.y, ans.t));
    }

    #[test]
    fn area_simple() {
        let p = Coord2D { x: 0.0, y: 0.0 };
        let q = Coord2D { x: 1.0, y: 0.0 };
        let r = Coord2D { x: 0.0, y: 1.0 };
        assert!((calc_area(&p, &q, &r) - 0.5).abs() < 0.0001);
    }

    #[test]
    fn area_simple2() {
        let p = Coord2D { x: 1.0, y: 1.0 };
        let q = Coord2D { x: 2.0, y: 1.0 };
        let r = Coord2D { x: 1.0, y: 2.0 };
        assert!((calc_area(&p, &q, &r) - 0.5).abs() < 0.0001);
    }
}
