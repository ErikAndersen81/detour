pub use distance::distance;
use std::fs::File;
use std::io::{BufWriter, Write};
mod distance;

#[derive(Debug, Clone)]
pub struct Trajectory {
    pub coords: Vec<Coord3D>,
}

impl std::ops::Index<usize> for Trajectory {
    type Output = Coord3D;
    fn index(&self, idx: usize) -> &Self::Output {
        &self.coords[idx]
    }
}

impl std::ops::IndexMut<usize> for Trajectory {
    fn index_mut(&mut self, idx: usize) -> &mut Self::Output {
        &mut self.coords[idx]
    }
}

impl Trajectory {
    pub fn trim(&self, start: f64, end: f64) -> Trajectory {
        let trj: &Vec<Coord3D> = &self.coords;
        let mut start_idx: usize = 0;
        let mut end_idx: usize = trj.len() - 1;
        let mut new_start: Coord3D = trj[start_idx].clone();
        let mut new_end: Coord3D = trj[end_idx].clone();
        let mut point_p: Coord3D;
        let mut point_q: Coord3D;
        for i in (0..(trj.len() - 1)) {
            point_p = trj[i].clone();
            point_q = trj[i + 1].clone();
            if (point_p.t..point_q.t).contains(&start) {
                new_start = interpolate(start, &point_p, &point_q);
                start_idx = i;
            };
            if (point_p.t < end) & (end < point_q.t) {
                new_end = interpolate(end, &point_p, &point_q);
                end_idx = i + 1;
            };
        }
        let mut coords: Vec<Coord3D> = trj[start_idx..(end_idx + 1)].to_vec();
        coords[0] = new_start;
        coords[end_idx - start_idx] = new_end;
        Trajectory { coords }
    }

    pub fn len(&self) -> usize {
        self.coords.len()
    }

    pub fn to_csv(&self, filename: String) {
        let f = File::create(filename).expect("Cannot create file.");
        let mut f = BufWriter::new(f);
        writeln!(f, "lon,lat,time").expect("Error writing to file.");
        for c in &self.coords {
            write!(f, "{}", c).expect("Error writing to file.");
        }
    }

    pub fn get_projection(&self, axis: usize) -> Vec<Coord2D> {
        self.coords
            .iter()
            .map(|c: &Coord3D| c.project(axis))
            .collect::<Vec<Coord2D>>()
    }

    pub fn from_array(v: Vec<[f64; 3]>) -> Trajectory {
        let coords: Vec<Coord3D> = v
            .iter()
            .map(|a| Coord3D::from_array(*a))
            .collect::<Vec<Coord3D>>();
        Trajectory { coords }
    }

    pub fn to_array(&self) -> Vec<[f64; 3]> {
        self.coords
            .iter()
            .map(|c| Coord3D::to_array(c))
            .collect::<Vec<[f64; 3]>>()
    }
}

#[derive(Debug, Clone, Copy)]
pub struct Coord3D {
    pub x: f64,
    pub y: f64,
    pub t: f64,
}

impl std::fmt::Display for Coord3D {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        writeln!(f, "{},{},{}", self.x, self.y, self.t)
    }
}

impl Coord3D {
    pub fn from_array(a: [f64; 3]) -> Coord3D {
        Coord3D {
            x: a[0],
            y: a[1],
            t: a[2],
        }
    }
    pub fn to_array(c: &Coord3D) -> [f64; 3] {
        [c.x, c.y, c.t]
    }

    fn project(&self, axis: usize) -> Coord2D {
        match axis {
            0 => Coord2D {
                x: self.x,
                y: self.t,
            },
            _ => Coord2D {
                x: self.y,
                y: self.t,
            },
        }
    }
}

#[derive(Debug, Clone)]
pub struct Coord2D {
    pub x: f64,
    pub y: f64,
}

pub struct Line2D<'a> {
    pub start: &'a Coord2D,
    pub end: &'a Coord2D,
}

pub fn get_common_time_span(trj_a: &Trajectory, trj_b: &Trajectory) -> Option<(f64, f64)> {
    let a_len = trj_a.len() - 1;
    let b_len = trj_b.len() - 1;
    let start: f64 = if trj_a[0].t > trj_b[0].t {
        trj_a[0].t
    } else {
        trj_b[0].t
    };
    let end: f64 = if trj_a[a_len].t < trj_b[b_len].t {
        trj_a[a_len].t
    } else {
        trj_b[b_len].t
    };
    if end - start < 0.0 {
        None
    } else {
        Some((start, end))
    }
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

#[cfg(test)]
mod trajectory_test {
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
    fn common_time_span_end() {
        let real_end: f64 = 1.631163301207;
        let trj_a: Trajectory = Trajectory::from_array((&TRJ_A).to_vec());
        let trj_b: Trajectory = Trajectory::from_array((&TRJ_B).to_vec());
        let (_start, end) = get_common_time_span(&trj_a, &trj_b).unwrap();
        assert!((end - real_end).abs() < 0.000000001, "Wrong end time")
    }

    #[test]
    fn common_time_span_start() {
        let real_start: f64 = 1.631162641494;
        let trj_a: Trajectory = Trajectory::from_array((&TRJ_A).to_vec());
        let trj_b: Trajectory = Trajectory::from_array((&TRJ_B).to_vec());
        let (start, _end) = get_common_time_span(&trj_a, &trj_b).unwrap();
        assert!((start - real_start).abs() < 0.000000001, "Wrong start time")
    }

    #[test]
    fn common_time_span_symmetry() {
        let trj_a: Trajectory = Trajectory::from_array((&TRJ_A).to_vec());
        let trj_b: Trajectory = Trajectory::from_array((&TRJ_B).to_vec());
        let (start1, end1) = get_common_time_span(&trj_a, &trj_b).unwrap();
        let (start2, end2) = get_common_time_span(&trj_b, &trj_a).unwrap();
        assert!((start1 - start2).abs() < 0.000000001, "Asymmetrical start");
        assert!((end1 - end2).abs() < 0.000000001, "Asymmetrical end");
    }

    #[test]
    fn trim_trajectory() {
        let real_start: f64 = 1.631162641494;
        let real_end: f64 = 1.631163301207;
        let trj_a: Trajectory = Trajectory::from_array((&TRJ_A).to_vec());
        let trj_b: Trajectory = Trajectory::from_array((&TRJ_B).to_vec());
        let (start, end) = get_common_time_span(&trj_a, &trj_b).unwrap();
        let trj_a = trj_a.trim(start, end);
        let (new_start, new_end) = (trj_a.coords[0].t, trj_a.coords[trj_a.coords.len() - 1].t);
        assert!(
            (new_start - real_start).abs() + (new_end - real_end).abs() < 0.000000001,
            "Wrong end time"
        )
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
}
