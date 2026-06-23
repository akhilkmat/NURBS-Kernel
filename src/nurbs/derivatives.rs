use crate::nurbs::basis::{ders_basis_funs, find_span};
use crate::nurbs::types::{ControlNet, Point3};

/// Analytic partial derivatives of a rational NURBS surface at `(u, v)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceDerivatives {
    pub point: Point3,
    pub du: Point3,
    pub dv: Point3,
    pub duu: Option<Point3>,
    pub duv: Option<Point3>,
    pub dvv: Option<Point3>,
}

/// Evaluate position and partial derivatives up to the requested order (1 or 2).
///
/// Uses the rational-surface derivative formula from Piegl & Tiller (Eq. 4.9):
/// `S^{(k,l)} = (A^{(k,l)} - Σ S^{(i,j)} W^{(k-i,l-j)}) / W`.
pub fn evaluate(net: &ControlNet, u: f64, v: f64, max_order: usize) -> SurfaceDerivatives {
    let max_order = max_order.min(2);

    let span_u = find_span(net.degree_u, &net.knots_u, u);
    let span_v = find_span(net.degree_v, &net.knots_v, v);

    let ndu = ders_basis_funs(span_u, u, net.degree_u, &net.knots_u, max_order);
    let ndv = ders_basis_funs(span_v, v, net.degree_v, &net.knots_v, max_order);

    let start_u = span_u - net.degree_u;
    let start_v = span_v - net.degree_v;

    let mut a = [[Point3::ZERO; 3]; 3];
    let mut w = [[0.0; 3]; 3];

    for i in 0..=net.degree_u {
        for j in 0..=net.degree_v {
            let weight = control_weight(net, start_u + i, start_v + j);
            let cp = net.control_points[start_u + i][start_v + j];

            for ku in 0..=max_order.min(2) {
                for kv in 0..=(max_order.min(2) - ku) {
                    let bu = ndu[ku][i];
                    let bv = ndv[kv][j];
                    let term = weight * bu * bv;
                    a[ku][kv] = a[ku][kv].add(cp.scale(term));
                    w[ku][kv] += term;
                }
            }
        }
    }

    let mut s = [[Point3::ZERO; 3]; 3];
    for ku in 0..=max_order.min(2) {
        for kv in 0..=(max_order.min(2) - ku) {
            if w[0][0].abs() < f64::EPSILON {
                continue;
            }

            let mut numerator = a[ku][kv];
            
            // Full rectangular iteration for quotient rule cross-terms
            for i in 0..=ku {
                for j in 0..=kv {
                    
                    if i == ku && j == kv {
                        continue;
                    }
                    let coeff = binomial(ku, i) * binomial(kv, j);
                    numerator = numerator.sub(s[i][j].scale(coeff * w[ku - i][kv - j]));
                }
            }
            
            //  divides the total combined numerator by w[0][0]
            s[ku][kv] = numerator.scale(1.0 / w[0][0]);
        }
    }

    SurfaceDerivatives {
        point: s[0][0],
        du: s[1][0],
        dv: s[0][1],
        duu: if max_order >= 2 { Some(s[2][0]) } else { None },
        duv: if max_order >= 2 { Some(s[1][1]) } else { None },
        dvv: if max_order >= 2 { Some(s[0][2]) } else { None },
    }
}

fn control_weight(net: &ControlNet, i: usize, j: usize) -> f64 {
    net.weights
        .as_ref()
        .and_then(|ws| ws.get(i))
        .and_then(|row| row.get(j))
        .copied()
        .unwrap_or(1.0)
}

fn binomial(n: usize, k: usize) -> f64 {
    if k > n {
        return 0.0;
    }
    let mut result = 1.0;
    for i in 0..k {
        result *= (n - i) as f64;
        result /= (i + 1) as f64;
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nurbs::surface;

    fn bilinear_net() -> ControlNet {
        ControlNet {
            degree_u: 1,
            degree_v: 1,
            knots_u: vec![0.0, 0.0, 1.0, 1.0],
            knots_v: vec![0.0, 0.0, 1.0, 1.0],
            control_points: vec![
                vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
                vec![Point3::new(0.0, 1.0, 1.0), Point3::new(1.0, 1.0, 0.0)],
            ],
            weights: None,
        }
    }

    fn finite_diff_u(net: &ControlNet, u: f64, v: f64, eps: f64) -> Point3 {
        let s0 = surface::evaluate(net, u - eps, v);
        let s1 = surface::evaluate(net, u + eps, v);
        Point3::new(
            (s1.x - s0.x) / (2.0 * eps),
            (s1.y - s0.y) / (2.0 * eps),
            (s1.z - s0.z) / (2.0 * eps),
        )
    }

    #[test]
    fn analytic_du_matches_finite_difference() {
        let net = bilinear_net();
        let u = 0.35;
        let v = 0.65;
        let eps = 1e-6;

        let analytic = evaluate(&net, u, v, 1);
        let numeric_u = finite_diff_u(&net, u, v, eps);

        assert!((analytic.du.x - numeric_u.x).abs() < 1e-4);
        assert!((analytic.du.y - numeric_u.y).abs() < 1e-4);
        assert!((analytic.du.z - numeric_u.z).abs() < 1e-4);
    }

    #[test]
    fn analytic_dv_matches_finite_difference() {
        let net = bilinear_net();
        let u = 0.35;
        let v = 0.65;
        let eps = 1e-6;

        let analytic = evaluate(&net, u, v, 1);
        let numeric_v = {
            let s0 = surface::evaluate(&net, u, v - eps);
            let s1 = surface::evaluate(&net, u, v + eps);
            Point3::new(
                (s1.x - s0.x) / (2.0 * eps),
                (s1.y - s0.y) / (2.0 * eps),
                (s1.z - s0.z) / (2.0 * eps),
            )
        };

        assert!((analytic.dv.x - numeric_v.x).abs() < 1e-4);
        assert!((analytic.dv.y - numeric_v.y).abs() < 1e-4);
        assert!((analytic.dv.z - numeric_v.z).abs() < 1e-4);
    }
}
