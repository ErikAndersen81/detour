use geo::{algorithm::simplifyvw::SimplifyVwIdx, Coordinate, LineString};

pub fn visvalingam(trj: &[[f64; 3]], epsilon: f64) -> Vec<[f64; 3]> {
    let line_string: LineString<f64> = trj
        .iter()
        .map(|[x, y, _]| Coordinate { x: *x, y: *y })
        .collect();
    line_string
        .simplifyvw_idx(&epsilon)
        .iter()
        .map(|idx| trj[*idx])
        .collect::<Vec<[f64; 3]>>()
}
