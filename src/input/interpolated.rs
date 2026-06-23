use crate::input::reader::RawConfig;
use crate::nurbs::types::Point3;

pub struct InterpolatedSpec {
    pub name: String,
    pub degree_u: usize,
    pub degree_v: usize,
    pub data_points: Vec<Vec<Point3>>,
}

pub fn parse(raw: &RawConfig) -> Result<InterpolatedSpec, String> {
    let name = raw
        .get("name")
        .cloned()
        .unwrap_or_else(|| "unnamed".to_string());

    // 1. Dynamically read degrees from your TOML file instead of hardcoding them!
    //    If they aren't found or fail to parse, default to 3.
    let degree_u = raw.get("degree_u")
        .and_then(|v| v.as_str().parse::<usize>().ok())
        .unwrap_or(3);

    let degree_v = raw.get("degree_v")
        .and_then(|v| v.as_str().parse::<usize>().ok())
        .unwrap_or(3);

    // Your 4x4 grid of physical target points
    let data_points = vec![
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
        ],
        vec![
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 1.2), 
            Point3::new(2.0, 1.0, 1.3),
            Point3::new(3.0, 1.0, 0.0),
        ],
        vec![
            Point3::new(0.0, 2.0, 0.0),
            Point3::new(1.0, 2.0, 1.3),
            Point3::new(2.0, 2.0, -1.5),
            Point3::new(3.0, 2.0, 0.0),
        ],
        vec![
            Point3::new(0.0, 3.0, 0.0),
            Point3::new(1.0, 3.0, 0.0),
            Point3::new(2.0, 3.0, 0.0),
            Point3::new(3.0, 3.0, 0.0),
        ],
    ];

    Ok(InterpolatedSpec {
        name,
        degree_u,
        degree_v,
        data_points,
    })
}