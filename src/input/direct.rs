use crate::input::reader::RawConfig;
use crate::nurbs::types::{ControlNet, Point3};

pub struct DirectSpec {
    pub name: String,
    pub net: ControlNet,
}

pub fn parse(raw: &RawConfig) -> Result<DirectSpec, String> {
    let name = raw
        .get("name")
        .cloned()
        .unwrap_or_else(|| "unnamed".to_string());

    // Valid minimalistic 3rd-degree (Cubic) NURBS placeholder
    let net = ControlNet {
        degree_u: 3,
        degree_v: 3,
        // Degree 3 with 4 control points requires exactly 8 knots (4 + 3 + 1)
        knots_u: vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        knots_v: vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
        // A minimal 4x4 grid of control points for a cubic patch
        control_points: vec![
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0)],
            vec![Point3::new(0.0, 1.0, 0.0), Point3::new(1.0, 1.0, 2.0), Point3::new(2.0, 1.0, 2.0), Point3::new(3.0, 1.0, 0.0)],
            vec![Point3::new(0.0, 2.0, 0.0), Point3::new(1.0, 2.0, 2.0), Point3::new(2.0, 2.0, 2.0), Point3::new(3.0, 2.0, 0.0)],
            vec![Point3::new(0.0, 3.0, 0.0), Point3::new(1.0, 3.0, 0.0), Point3::new(2.0, 3.0, 0.0), Point3::new(3.0, 3.0, 0.0)],
        ],
        weights: Some(vec![
            vec![1.0, 1.0, 1.0, 1.0],
            vec![1.0, 1.0, 1.0, 1.0],
            vec![1.0, 1.0, 1.0, 1.0],
            vec![1.0, 1.0, 1.0, 1.0],
        ]),
    };

    let _ = raw;
    Ok(DirectSpec { name, net })
}