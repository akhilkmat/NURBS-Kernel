#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Point3 {
    pub const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
    };

    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }

    pub fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }

    pub fn scale(self, s: f64) -> Self {
        Self {
            x: self.x * s,
            y: self.y * s,
            z: self.z * s,
        }
    }

    pub fn dot(self, other: Self) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn cross(self, other: Self) -> Self {
        Self {
            x: self.y * other.z - self.z * other.y,
            y: self.z * other.x - self.x * other.z,
            z: self.x * other.y - self.y * other.x,
        }
    }

    pub fn length(self) -> f64 {
        self.dot(self).sqrt()
    }

    pub fn normalize(self) -> Self {
        let len = self.length();
        if len < f64::EPSILON {
            return Self::ZERO;
        }
        self.scale(1.0 / len)
    }
}

#[derive(Debug, Clone)]
pub struct ControlNet {
    pub degree_u: usize,
    pub degree_v: usize,
    pub knots_u: Vec<f64>,
    pub knots_v: Vec<f64>,
    pub control_points: Vec<Vec<Point3>>,
    pub weights: Option<Vec<Vec<f64>>>,
}

#[derive(Debug, Clone)]
pub struct NurbsSurface {
    pub net: ControlNet,
}

impl NurbsSurface {
    pub fn evaluate(&self, u: f64, v: f64) -> Point3 {
        crate::nurbs::surface::evaluate(&self.net, u, v)
    }

    pub fn derivatives(&self, u: f64, v: f64) -> crate::nurbs::derivatives::SurfaceDerivatives {
        crate::nurbs::derivatives::evaluate(&self.net, u, v, 1)
    }

    pub fn frame(&self, u: f64, v: f64) -> crate::nurbs::differential::SurfaceFrame {
        crate::nurbs::differential::frame_with_order(&self.net, u, v, 2)
    }
}
