use crate::nurbs::basis::{basis_funs, find_span};
use crate::nurbs::types::{ControlNet, NurbsSurface, Point3};

pub fn from_direct(spec: crate::input::direct::DirectSpec) -> NurbsSurface {
    from_control_net(spec.net)
}

pub fn from_control_net(net: ControlNet) -> NurbsSurface {
    NurbsSurface { net }
}

pub fn evaluate(net: &ControlNet, u: f64, v: f64) -> Point3 {
    let span_u = find_span(net.degree_u, &net.knots_u, u);
    let span_v = find_span(net.degree_v, &net.knots_v, v);
    let basis_u = basis_funs(span_u, u, net.degree_u, &net.knots_u);
    let basis_v = basis_funs(span_v, v, net.degree_v, &net.knots_v);

    let start_u = span_u - net.degree_u;
    let start_v = span_v - net.degree_v;

    let mut point = Point3::new(0.0, 0.0, 0.0);
    let mut weight_sum: f64 = 0.0;

    for (i, &bu) in basis_u.iter().enumerate() {
        for (j, &bv) in basis_v.iter().enumerate() {
            // FIXED: Explicit lookup mapping bypassing the nested .copied() ambiguity
            let w = net.weights
                .as_ref()
                .and_then(|ws| ws.get(start_u + i))
                .and_then(|row| row.get(start_v + j))
                .map(|&val| val) // Safely copies the f64 by matching the exact inner reference
                .unwrap_or(1.0);

            let factor = w * bu * bv;
            let cp = net.control_points[start_u + i][start_v + j];
            point.x += factor * cp.x;
            point.y += factor * cp.y;
            point.z += factor * cp.z;
            weight_sum += factor;
        }
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
