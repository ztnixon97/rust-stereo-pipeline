use super::{CameraModel, distortion::DistortionModel};
use nalgebra::Vector3;

/// Fisheye camera model
#[derive(Debug, Clone)]
pub struct FisheyeCamera {
    width: usize,
    height: usize,
    fx: f64,
    fy: f64,
    cx: f64,
    cy: f64,
    distortion: DistortionModel,
}

impl FisheyeCamera {
    /// Create a new fisheye camera
    pub fn new(
        width: usize,
        height: usize,
        fx: f64,
        fy: f64,
        cx: f64,
        cy: f64,
        k1: f64,
        k2: f64,
        k3: f64,
        k4: f64,
    ) -> Self {
        Self {
            width,
            height,
            fx,
            fy,
            cx,
            cy,
            distortion: DistortionModel::Fisheye { k1, k2, k3, k4 },
        }
    }
}

impl CameraModel for FisheyeCamera {
    fn project(&self, point_camera: &Vector3<f64>) -> Option<(f64, f64)> {
        if point_camera.z <= 0.0 {
            return None;
        }

        let x_norm = point_camera.x / point_camera.z;
        let y_norm = point_camera.y / point_camera.z;

        let (x_dist, y_dist) = self.distortion.distort(x_norm, y_norm);

        let u = self.fx * x_dist + self.cx;
        let v = self.fy * y_dist + self.cy;

        Some((u, v))
    }

    fn unproject(&self, pixel: (f64, f64)) -> Result<Vector3<f64>, super::DistortionError> {
        let x_dist = (pixel.0 - self.cx) / self.fx;
        let y_dist = (pixel.1 - self.cy) / self.fy;

        let (x_norm, y_norm) = self.distortion.undistort(x_dist, y_dist)?;

        Ok(Vector3::new(x_norm, y_norm, 1.0).normalize())
    }

    fn image_size(&self) -> (usize, usize) {
        (self.width, self.height)
    }
}
