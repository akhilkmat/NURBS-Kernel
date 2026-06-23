use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

use crate::mesh::types::TriangleMesh;

/// Write legacy VTK polydata (`.vtk`) for ParaView with analytic curvature arrays.
pub fn write_polydata(
    mesh: &TriangleMesh,
    mean_curvatures: &[f64],
    gaussian_curvatures: &[f64],
    path: &Path,
) -> Result<(), String> {
    let file = File::create(path).map_err(|e| e.to_string())?;
    let mut out = BufWriter::new(file);

    writeln!(out, "# vtk DataFile Version 3.0").map_err(|e| e.to_string())?;
    writeln!(out, "NURBS surface mesh").map_err(|e| e.to_string())?;
    writeln!(out, "ASCII").map_err(|e| e.to_string())?;
    writeln!(out, "DATASET POLYDATA").map_err(|e| e.to_string())?;

    // 1. Points
    writeln!(out, "POINTS {} double", mesh.vertex_count()).map_err(|e| e.to_string())?;
    for v in &mesh.vertices {
        writeln!(out, "{} {} {}", v.position.x, v.position.y, v.position.z)
            .map_err(|e| e.to_string())?;
    }

    // 2. Polygons
    let polygon_size = mesh.triangle_count() * 4;
    writeln!(out, "POLYGONS {} {}", mesh.triangle_count(), polygon_size)
        .map_err(|e| e.to_string())?;
    for tri in &mesh.triangles {
        writeln!(out, "3 {} {} {}", tri[0], tri[1], tri[2]).map_err(|e| e.to_string())?;
    }

    // 3. Point Data Section Header
    writeln!(out, "POINT_DATA {}", mesh.vertex_count()).map_err(|e| e.to_string())?;

    // 4. Normals
    writeln!(out, "NORMALS normals double").map_err(|e| e.to_string())?;
    for v in &mesh.vertices {
        writeln!(out, "{} {} {}", v.normal.x, v.normal.y, v.normal.z)
            .map_err(|e| e.to_string())?;
    }

    // 5. Texture Coordinates (UVs)
    writeln!(out, "TEXTURE_COORDINATES uv 2 double").map_err(|e| e.to_string())?;
    for v in &mesh.vertices {
        writeln!(out, "{} {}", v.uv.0, v.uv.1).map_err(|e| e.to_string())?;
    }

    // 6. Analytic Mean Curvature (SCALARS block)
    writeln!(out, "SCALARS mean_curvature double 1").map_err(|e| e.to_string())?;
    writeln!(out, "LOOKUP_TABLE default").map_err(|e| e.to_string())?;
    for h in mean_curvatures {
        writeln!(out, "{}", h).map_err(|e| e.to_string())?;
    }

    // 7. Analytic Gaussian Curvature (SCALARS block)
    writeln!(out, "SCALARS gaussian_curvature double 1").map_err(|e| e.to_string())?;
    writeln!(out, "LOOKUP_TABLE default").map_err(|e| e.to_string())?;
    for k in gaussian_curvatures {
        writeln!(out, "{}", k).map_err(|e| e.to_string())?;
    }

    out.flush().map_err(|e| e.to_string())
}