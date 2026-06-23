use crate::mesh::types::{MeshVertex, TriangleMesh};
use crate::nurbs::types::NurbsSurface;

/// Parametric grid tessellation using analytic surface frames for normals.
pub fn from_surface(surface: &NurbsSurface, segments_u: usize, segments_v: usize) -> TriangleMesh {
    assert!(segments_u >= 1, "segments_u must be >= 1");
    assert!(segments_v >= 1, "segments_v must be >= 1");

    let nu = segments_u + 1;
    let nv = segments_v + 1;
    let mut vertices = Vec::with_capacity(nu * nv);


    // Fetch the true boundaries from the knot vectors
    let u_min = surface.net.knots_u[0];
    let u_max = *surface.net.knots_u.last().unwrap();
    let v_min = surface.net.knots_v[0];
    let v_max = *surface.net.knots_v.last().unwrap();

    // Map the loop ratio across the true range
    for iv in 0..nv {
        let t_v = iv as f64 / segments_v as f64;
        let v = v_min + t_v * (v_max - v_min); // Remaps [0, 1] to [v_min, v_max]
        
        for iu in 0..nu {
            let t_u = iu as f64 / segments_u as f64;
            let u = u_min + t_u * (u_max - u_min); // Remaps [0, 1] to [u_min, u_max]
            
            let frame = surface.frame(u, v);
               vertices.push(MeshVertex {
                  position: frame.position,
                  normal: frame.normal,
                     uv: (u, v),
                 });
        }
    }



    let mut triangles = Vec::with_capacity(segments_u * segments_v * 2);
    for iv in 0..segments_v {
        for iu in 0..segments_u {
            let i0 = iv * nu + iu;
            let i1 = i0 + 1;
            let i2 = i0 + nu;
            let i3 = i2 + 1;

            triangles.push([i0, i2, i1]);
            triangles.push([i1, i2, i3]);
        }
    }

    TriangleMesh {
        vertices,
        triangles,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nurbs::types::{ControlNet, Point3};

    #[test]
    fn grid_has_expected_counts() {
        let surface = NurbsSurface {
            net: ControlNet {
                degree_u: 1,
                degree_v: 1,
                knots_u: vec![0.0, 0.0, 1.0, 1.0],
                knots_v: vec![0.0, 0.0, 1.0, 1.0],
                control_points: vec![
                    vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
                    vec![Point3::new(0.0, 1.0, 1.0), Point3::new(1.0, 1.0, 0.0)],
                ],
                weights: None,
            },
        };

        let mesh = from_surface(&surface, 4, 3);
        assert_eq!(mesh.vertex_count(), 5 * 4);
        assert_eq!(mesh.triangle_count(), 4 * 3 * 2);
    }
}
