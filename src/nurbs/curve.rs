use crate::nurbs::basis::{basis_funs, find_span};
use crate::nurbs::types::Point3;

pub fn evaluate(
    degree: usize,
    knots: &[f64],
    control_points: &[Point3],
    weights: Option<&[f64]>,
    t: f64,
) -> Point3 {
    let span = find_span(degree, knots, t);
    let basis = basis_funs(span, t, degree, knots);

    let start = span - degree;
    let mut point = Point3::new(0.0, 0.0, 0.0);
    let mut weight_sum: f64 = 0.0;

    for (i, &b) in basis.iter().enumerate() {
        let idx = start + i;
        let w = weights.map(|ws| ws[idx]).unwrap_or(1.0);
        let weighted = w * b;
        point.x += weighted * control_points[idx].x;
        point.y += weighted * control_points[idx].y;
        point.z += weighted * control_points[idx].z;
        weight_sum += weighted;
    }

    if weight_sum.abs() < f64::EPSILON {
        return Point3::new(0.0, 0.0, 0.0);
    }

    Point3::new(
        point.x / weight_sum,
        point.y / weight_sum,
        point.z / weight_sum,
    )
}
