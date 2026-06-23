/// Find the knot span index for parameter value `u`.
pub fn find_span(degree: usize, knots: &[f64], u: f64) -> usize {
    let n = knots.len() - degree - 2;
    if u >= knots[n + 1] {
        return n;
    }
    if u <= knots[degree] {
        return degree;
    }

    let mut low = degree;
    let mut high = n + 1;
    let mut mid = (low + high) / 2;

    while u < knots[mid] || u >= knots[mid + 1] {
        if u < knots[mid] {
            high = mid;
        } else {
            low = mid;
        }
        mid = (low + high) / 2;
    }

    mid
}

/// Cox-de Boor basis values N[0..=degree] at `u` for knot span `span`.
pub fn basis_funs(span: usize, u: f64, degree: usize, knots: &[f64]) -> Vec<f64> {
    let mut left = vec![0.0; degree + 1];
    let mut right = vec![0.0; degree + 1];
    let mut n = vec![0.0; degree + 1];

    n[0] = 1.0;

    for j in 1..=degree {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        let mut saved = 0.0;

        for r in 0..j {
            let denom = right[r + 1] + left[j - r];
            let temp = if denom.abs() < f64::EPSILON {
                0.0
            } else {
                n[r] / denom
            };
            n[r] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }

        n[j] = saved;
    }

    n
}

/// Basis values and derivatives up to order `n_derivs` (Piegl & Tiller, Algorithm A2.3).
///
/// Returns `ders[k][j]` = k-th derivative of `N_{span - degree + j, degree}`.
pub fn ders_basis_funs(
    span: usize,
    u: f64,
    degree: usize,
    knots: &[f64],
    n_derivs: usize,
) -> Vec<Vec<f64>> {
    let du = n_derivs.min(degree);
    let mut ndu = vec![vec![0.0; degree + 1]; degree + 1];
    let mut left = vec![0.0; degree + 1];
    let mut right = vec![0.0; degree + 1];

    ndu[0][0] = 1.0;

    for j in 1..=degree {
        left[j] = u - knots[span + 1 - j];
        right[j] = knots[span + j] - u;
        let mut saved = 0.0;

        for r in 0..j {
            ndu[j][r] = right[r + 1] + left[j - r];
            let temp = if ndu[j][r].abs() < f64::EPSILON {
                0.0
            } else {
                ndu[r][j - 1] / ndu[j][r]
            };
            ndu[r][j] = saved + right[r + 1] * temp;
            saved = left[j - r] * temp;
        }

        ndu[j][j] = saved;
    }

    let mut map_ders = vec![vec![0.0; degree + 1]; n_derivs + 1];
    for j in 0..=degree {
        map_ders[0][j] = ndu[j][degree];
    }

    let mut a = vec![vec![0.0; degree + 1]; 2];
    for r in 0..=degree {
        let mut s1 = 0usize;
        let mut s2 = 1usize;
        a[0][0] = 1.0;

        for k in 1..=du {
            let mut d = 0.0;
            // Safe saturating calculation protects against low-degree or mismatch overflows
            let pk = degree.saturating_sub(k);

            // 1. First Term calculation block
            if r >= k {
                let rk = r - k;
                if pk + 1 < ndu.len() && rk < ndu[pk + 1].len() {
                    let denom = ndu[pk + 1][rk];
                    if denom.abs() > f64::EPSILON {
                        a[s2][0] = a[s1][0] / denom;
                        d = a[s2][0] * ndu[rk][pk];
                    }
                }
            }

            // 2. Middle Terms with clean index protections
            let j1 = if r >= k { 1 } else { k - r };
            let j2 = if r + 1 <= pk + 1 { k - 1 } else { degree - r };

            for j in j1..=j2 {
                let idx = r + j;
                if idx >= k {
                    let safe_idx = idx - k;
                    if pk + 1 < ndu.len() && safe_idx < ndu[pk + 1].len() {
                        let denom = ndu[pk + 1][safe_idx];
                        if denom.abs() > f64::EPSILON {
                            a[s2][j] = (a[s1][j] - a[s1][j - 1]) / denom;
                            d += a[s2][j] * ndu[safe_idx][pk];
                        }
                    }
                }
            }

            // 3. Last Term calculation block
            if r <= pk {
                if pk + 1 < ndu.len() && r < ndu[pk + 1].len() {
                    let denom = ndu[pk + 1][r];
                    if denom.abs() > f64::EPSILON {
                        a[s2][k] = -a[s1][k - 1] / denom;
                        d += a[s2][k] * ndu[r][pk];
                    }
                }
            }

            map_ders[k][r] = d;
            std::mem::swap(&mut s1, &mut s2);
        }
    }

    let mut factor = degree as f64;
    for k in 1..=du {
        for j in 0..=degree {
            map_ders[k][j] *= factor;
        }
        factor *= (degree - k) as f64;
    }

    map_ders
}
