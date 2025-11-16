use super::{distortion::DistortionModel, CameraModel};
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

    fn unproject(&self, pixel: (f64, f64)) -> Vector3<f64> {
        let x_dist = (pixel.0 - self.cx) / self.fx;
        let y_dist = (pixel.1 - self.cy) / self.fy;

        let (x_norm, y_norm) = self.distortion.undistort(x_dist, y_dist);

        Vector3::new(x_norm, y_norm, 1.0).normalize()
    }

    fn image_size(&self) -> (usize, usize) {
        (self.width, self.height)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fisheye_construction() {
        let camera = FisheyeCamera::new(
            1920, 1080,
            800.0, 800.0,
            960.0, 540.0,
            -0.1, 0.05, -0.01, 0.001,
        );

        let (w, h) = camera.image_size();
        assert_eq!(w, 1920);
        assert_eq!(h, 1080);
    }

    #[test]
    fn test_fisheye_center_projection() {
        let camera = FisheyeCamera::new(
            1920, 1080,
            800.0, 800.0,
            960.0, 540.0,
            0.0, 0.0, 0.0, 0.0, // No distortion for this test
        );

        // Test center point
        let point = Vector3::new(0.0, 0.0, 1.0);
        let pixel = camera.project(&point).unwrap();

        // Should be close to principal point
        assert!((pixel.0 - 960.0).abs() < 1e-3);
        assert!((pixel.1 - 540.0).abs() < 1e-3);
    }

    #[test]
    fn test_fisheye_behind_camera() {
        let camera = FisheyeCamera::new(
            1920, 1080,
            800.0, 800.0,
            960.0, 540.0,
            -0.1, 0.05, -0.01, 0.001,
        );

        // Point behind camera
        let point = Vector3::new(0.0, 0.0, -1.0);
        let result = camera.project(&point);
        assert!(result.is_none());
    }

    #[test]
    fn test_fisheye_unproject() {
        let camera = FisheyeCamera::new(
            1920, 1080,
            800.0, 800.0,
            960.0, 540.0,
            0.0, 0.0, 0.0, 0.0,
        );

        // Unproject center pixel
        let ray = camera.unproject((960.0, 540.0));

        // Ray should be normalized and pointing forward
        assert!((ray.norm() - 1.0).abs() < 1e-6);
        assert!(ray.z > 0.0);
    }

    #[test]
    fn test_fisheye_roundtrip_no_distortion() {
        let camera = FisheyeCamera::new(
            1920, 1080,
            800.0, 800.0,
            960.0, 540.0,
            0.0, 0.0, 0.0, 0.0, // No distortion
        );

        let point = Vector3::new(0.5, 0.3, 2.0);
        let pixel = camera.project(&point).unwrap();
        let ray = camera.unproject(pixel);

        // Ray direction should be parallel to original point
        let original_normalized = point.normalize();
        let dot = ray.dot(&original_normalized);
        assert!((dot - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_fisheye_with_distortion() {
        let camera = FisheyeCamera::new(
            1920, 1080,
            800.0, 800.0,
            960.0, 540.0,
            -0.1, 0.05, -0.01, 0.001,
        );

        // Test that distortion has some effect
        let point = Vector3::new(0.5, 0.3, 1.0);
        let pixel = camera.project(&point);
        assert!(pixel.is_some());

        let (u, v) = pixel.unwrap();
        assert!(u > 0.0 && u < 1920.0);
        assert!(v > 0.0 && v < 1080.0);
    }

    #[test]
    fn test_fisheye_wide_angle() {
        let camera = FisheyeCamera::new(
            1920, 1080,
            800.0, 800.0,
            960.0, 540.0,
            -0.1, 0.05, -0.01, 0.001,
        );

        // Fisheye cameras can handle wide angles
        let point = Vector3::new(1.5, 1.0, 1.0);
        let pixel = camera.project(&point);
        assert!(pixel.is_some());
    }

    #[test]
    fn test_fisheye_image_size() {
        let camera = FisheyeCamera::new(
            2560, 1440,
            1000.0, 1000.0,
            1280.0, 720.0,
            -0.1, 0.05, -0.01, 0.001,
        );

        let (w, h) = camera.image_size();
        assert_eq!(w, 2560);
        assert_eq!(h, 1440);
    }
}
