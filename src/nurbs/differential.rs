use crate::nurbs::derivatives::{evaluate, SurfaceDerivatives};
use crate::nurbs::types::{ControlNet, Point3};

/// Local differential frame on a NURBS surface.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceFrame {
    pub position: Point3,
    /// Raw parametric tangent `∂S/∂u` (not unit length).
    pub tangent_u: Point3,
    /// Raw parametric tangent `∂S/∂v` (not unit length).
    pub tangent_v: Point3,
    /// Unit normal `normalize(tangent_u × tangent_v)`.
    pub normal: Point3,
    pub duu: Option<Point3>,
    pub duv: Option<Point3>,
    pub dvv: Option<Point3>,
}

pub fn frame(net: &ControlNet, u: f64, v: f64) -> SurfaceFrame {
    frame_with_order(net, u, v, 1)
}

pub fn frame_with_order(net: &ControlNet, u: f64, v: f64, max_order: usize) -> SurfaceFrame {
    let d = evaluate(net, u, v, max_order);
    from_derivatives(&d)
}

pub fn from_derivatives(d: &SurfaceDerivatives) -> SurfaceFrame {
    SurfaceFrame {
        position: d.point,
        tangent_u: d.du,
        tangent_v: d.dv,
        normal: d.du.cross(d.dv).normalize(),
        duu: d.duu,
        duv: d.duv,
        dvv: d.dvv,
    }
}

/// First fundamental form coefficients at `(u, v)`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FirstFundamentalForm {
    pub e: f64,
    pub f: f64,
    pub g: f64,
}

pub fn first_fundamental_form(frame: &SurfaceFrame) -> FirstFundamentalForm {
    FirstFundamentalForm {
        e: frame.tangent_u.dot(frame.tangent_u),
        f: frame.tangent_u.dot(frame.tangent_v),
        g: frame.tangent_v.dot(frame.tangent_v),
    }
}

/// Mean and Gaussian curvature from analytic first/second fundamental forms.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceCurvature {
    pub mean: f64,
    pub gaussian: f64,
}

pub fn curvature(frame: &SurfaceFrame) -> Option<SurfaceCurvature> {
    let (duu, duv, dvv) = (frame.duu?, frame.duv?, frame.dvv?);
    let n = frame.normal;

    let ii = duu.dot(n);
    let ij = duv.dot(n);
    let jj = dvv.dot(n);

    let ff = first_fundamental_form(frame);
    let det = ff.e * ff.g - ff.f * ff.f;
    if det.abs() < f64::EPSILON {
        return None;
    }

    let mean = (ff.g * ii - 2.0 * ff.f * ij + ff.e * jj) / (2.0 * det);
    let gaussian = (ii * jj - ij * ij) / det;

    Some(SurfaceCurvature { mean, gaussian })
}
