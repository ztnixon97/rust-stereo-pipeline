use super::{CameraModel, distortion::DistortionModel};
use nalgebra::Vector3;

/// Pinhole camera model with optional distortion
#[derive(Debug, Clone)]
pub struct PinholeCamera {
    width: usize,
    height: usize,
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
    distortion: DistortionModel,
}

impl PinholeCamera {
    /// Create a new pinhole camera with Brown-Conrady distortion
    pub fn new_brown_conrady(
        width: usize,
        height: usize,
        fx: f64,
        fy: f64,
        cx: f64,
        cy: f64,
        k1: f64,
        k2: f64,
        k3: f64,
        p1: f64,
        p2: f64,
    ) -> Self {
        Self {
            width,
            height,
            fx,
            fy,
            cx,
            cy,
            distortion: DistortionModel::BrownConrady { k1, k2, k3, p1, p2 },
        }
    }

    /// Create a new pinhole camera with no distortion
    pub fn new_ideal(width: usize, height: usize, fx: f64, fy: f64, cx: f64, cy: f64) -> Self {
        Self {
            width,
            height,
            fx,
            fy,
            cx,
            cy,
            distortion: DistortionModel::None,
        }
    }

    /// Get focal lengths
    pub fn focal_length(&self) -> (f64, f64) {
        (self.fx, self.fy)
    }

    /// Get principal point
    pub fn principal_point(&self) -> (f64, f64) {
        (self.cx, self.cy)
    }
}

impl CameraModel for PinholeCamera {
    fn project(&self, point_camera: &Vector3<f64>) -> Option<(f64, f64)> {
        if point_camera.z <= 0.0 {
            return None;
        }

        // Normalized coordinates
        let x_norm = point_camera.x / point_camera.z;
        let y_norm = point_camera.y / point_camera.z;

        // Apply distortion
        let (x_dist, y_dist) = self.distortion.distort(x_norm, y_norm);

        // To pixel coordinates
        let u = self.fx * x_dist + self.cx;
        let v = self.fy * y_dist + self.cy;

        Some((u, v))
    }

    fn unproject(&self, pixel: (f64, f64)) -> Result<Vector3<f64>, super::DistortionError> {
        // Pixel to distorted normalized coordinates
        let x_dist = (pixel.0 - self.cx) / self.fx;
        let y_dist = (pixel.1 - self.cy) / self.fy;

        // Remove distortion
        let (x_norm, y_norm) = self.distortion.undistort(x_dist, y_dist)?;

        // Ray in camera frame (unit vector)
        Ok(Vector3::new(x_norm, y_norm, 1.0).normalize())
    }

    fn image_size(&self) -> (usize, usize) {
        (self.width, self.height)
    }
}
