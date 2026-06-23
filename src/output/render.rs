use std::path::Path;

use crate::nurbs::types::NurbsSurface;

/// Stub for later PNG export. For now this only validates the output path idea.
pub fn save_wireframe(surface: &NurbsSurface, path: &Path) -> Result<(), String> {
    let _ = surface;
    Err(format!(
        "PNG rendering not implemented yet (requested path: {})",
        path.display()
    ))
}
