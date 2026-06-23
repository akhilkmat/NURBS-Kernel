use crate::nurbs::types::Point3;

#[derive(Debug, Clone, PartialEq)]
pub struct MeshVertex {
    pub position: Point3,
    pub normal: Point3,
    pub uv: (f64, f64),
}

#[derive(Debug, Clone, PartialEq)]
pub struct TriangleMesh {
    pub vertices: Vec<MeshVertex>,
    pub triangles: Vec<[usize; 3]>,
}

impl TriangleMesh {
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }
}
