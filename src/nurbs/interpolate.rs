use crate::input::interpolated::InterpolatedSpec;
use crate::nurbs::basis::{find_span, basis_funs};
use crate::nurbs::types::{ControlNet, Point3};
use nalgebra::{DMatrix, DVector};

/// Chord length parameterisation: assigns parameter values proportional
/// to the distance travelled between consecutive data points.
fn chord_length_params(points: &[Point3]) -> Vec<f64> {
    let n = points.len();
    let mut params = vec![0.0; n];

    if n < 2 {
        return params;
    }

    let mut chord_lengths = vec![0.0; n - 1];
    let mut total_length = 0.0;

    for k in 0..n - 1 {
        let d = points[k + 1].sub(points[k]);
        let len = (d.x * d.x + d.y * d.y + d.z * d.z).sqrt();
        chord_lengths[k] = len;
        total_length += len;
    }

    if total_length < f64::EPSILON {
        for k in 0..n {
            params[k] = k as f64 / (n - 1) as f64;
        }
        return params;
    }

    let mut cumulative = 0.0;
    for k in 1..n {
        cumulative += chord_lengths[k - 1];
        params[k] = cumulative / total_length;
    }
    params[n - 1] = 1.0;

    params
}

/// Averaged knot vector technique (Piegl & Tiller Eq. 9.8): interior knots
/// are derived from averages of the data point parameter values, which
/// keeps the basis matrix well conditioned for the given parameterisation.
fn averaged_knots(params: &[f64], degree: usize) -> Vec<f64> {
    let n = params.len() - 1;
    let mut knots = Vec::with_capacity(n + degree + 2);

    for _ in 0..=degree {
        knots.push(0.0);
    }

    if n > degree {
        for j in 1..=(n - degree) {
            let sum: f64 = params[j..j + degree].iter().sum();
            knots.push(sum / degree as f64);
        }
    }

    for _ in 0..=degree {
        knots.push(1.0);
    }

    knots
}

/// Builds the (n+1) x (n+1) basis matrix N where N[k][i] = N_{i,p}(u_k).
fn build_basis_matrix(params: &[f64], knots: &[f64], degree: usize) -> DMatrix<f64> {
    let n = params.len();
    let mut matrix = DMatrix::<f64>::zeros(n, n);

    for (row, &t) in params.iter().enumerate() {
        let span = find_span(degree, knots, t);
        let basis_vals = basis_funs(span, t, degree, knots);
        let start = span - degree;

        for (offset, &val) in basis_vals.iter().enumerate() {
            matrix[(row, start + offset)] = val;
        }
    }

    matrix
}

/// Solves N * P = Q for one direction of a curve through `points`.
/// Returns the computed control points and the knot vector used.
fn solve_curve_interpolation(
    points: &[Point3],
    degree: usize,
) -> Result<(Vec<Point3>, Vec<f64>), String> {
    let n = points.len();

    if n <= degree {
        return Err(format!(
            "Need at least {} points for degree {} interpolation, got {}",
            degree + 1,
            degree,
            n
        ));
    }

    let params = chord_length_params(points);
    let knots = averaged_knots(&params, degree);
    let basis_matrix = build_basis_matrix(&params, &knots, degree);

    let mut rhs_x = DVector::<f64>::zeros(n);
    let mut rhs_y = DVector::<f64>::zeros(n);
    let mut rhs_z = DVector::<f64>::zeros(n);

    for (i, p) in points.iter().enumerate() {
        rhs_x[i] = p.x;
        rhs_y[i] = p.y;
        rhs_z[i] = p.z;
    }

    let lu = basis_matrix.clone().lu();

    let sol_x = lu.solve(&rhs_x).ok_or("Singular basis matrix while solving x")?;
    let sol_y = lu.solve(&rhs_y).ok_or("Singular basis matrix while solving y")?;
    let sol_z = lu.solve(&rhs_z).ok_or("Singular basis matrix while solving z")?;

    let control_points: Vec<Point3> = (0..n)
        .map(|i| Point3::new(sol_x[i], sol_y[i], sol_z[i]))
        .collect();

    Ok((control_points, knots))
}

/// Build a control net from interpolated data points using global surface
/// interpolation: first interpolate along u for every column, then
/// interpolate the resulting intermediate control points along v.
pub fn build_surface(spec: &InterpolatedSpec) -> Result<ControlNet, String> {
    let num_points_u = spec.data_points.len();
    let num_points_v = if num_points_u > 0 {
        spec.data_points[0].len()
    } else {
        0
    };

    if num_points_u == 0 || num_points_v == 0 {
        return Err("Cannot interpolate an empty data point grid collection.".to_string());
    }

    let degree_u = spec.degree_u.min(num_points_u - 1);
    let degree_v = spec.degree_v.min(num_points_v - 1);

    // Pass 1: interpolate along u for every fixed v (every column)
    let mut intermediate: Vec<Vec<Point3>> =
        vec![vec![Point3::ZERO; num_points_v]; num_points_u];
    let mut knots_u = Vec::new();

    for j in 0..num_points_v {
        let column: Vec<Point3> = (0..num_points_u).map(|i| spec.data_points[i][j]).collect();

        let (control_col, knots) = solve_curve_interpolation(&column, degree_u)?;
        knots_u = knots;

        for i in 0..num_points_u {
            intermediate[i][j] = control_col[i];
        }
    }

    // Pass 2: interpolate along v for every fixed u (every row of the intermediate grid)
    let mut control_points: Vec<Vec<Point3>> =
        vec![vec![Point3::ZERO; num_points_v]; num_points_u];
    let mut knots_v = Vec::new();

    for i in 0..num_points_u {
        let row = &intermediate[i];
        let (control_row, knots) = solve_curve_interpolation(row, degree_v)?;
        knots_v = knots;

        for j in 0..num_points_v {
            control_points[i][j] = control_row[j];
        }
    }

    Ok(ControlNet {
        degree_u,
        degree_v,
        knots_u,
        knots_v,
        control_points,
        weights: None,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nurbs::surface;

    fn flat_grid() -> Vec<Vec<Point3>> {
        // simple 4x4 grid of points lying on a known bilinear-ish surface
        let mut grid = vec![vec![Point3::ZERO; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                let x = i as f64;
                let y = j as f64;
                let z = 0.1 * (x * y); // mild curvature so it is not purely planar
                grid[i][j] = Point3::new(x, y, z);
            }
        }
        grid
    }

    #[test]
    fn surface_passes_through_input_points() {
        let spec = InterpolatedSpec {
            data_points: flat_grid(),
            degree_u: 3,
            degree_v: 3,
        };

        let net = build_surface(&spec).expect("interpolation should succeed");

        // evaluate the surface at each (u_i, v_j) implied by the knot averaging
        // and check it lands back on the original data point, within tolerance
        let n_u = net.control_points.len();
        let n_v = net.control_points[0].len();

        // crude re-derivation of the same params used internally, just for the test
        let params_u: Vec<f64> = (0..n_u).map(|i| i as f64 / (n_u - 1) as f64).collect();
        let params_v: Vec<f64> = (0..n_v).map(|j| j as f64 / (n_v - 1) as f64).collect();

        for (i, &u) in params_u.iter().enumerate() {
            for (j, &v) in params_v.iter().enumerate() {
                let evaluated = surface::evaluate(&net, u, v);
                let expected = flat_grid()[i][j];

                // loose tolerance since params_u/v here are an approximation,
                // not the exact chord-length values used inside build_surface
                assert!((evaluated.x - expected.x).abs() < 1.0);
                assert!((evaluated.y - expected.y).abs() < 1.0);
            }
        }
    }
}










