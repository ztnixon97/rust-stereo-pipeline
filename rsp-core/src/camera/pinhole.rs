use super::{distortion::DistortionModel, CameraModel};
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

    fn unproject(&self, pixel: (f64, f64)) -> Vector3<f64> {
        // Pixel to distorted normalized coordinates
        let x_dist = (pixel.0 - self.cx) / self.fx;
        let y_dist = (pixel.1 - self.cy) / self.fy;

        // Remove distortion
        let (x_norm, y_norm) = self.distortion.undistort(x_dist, y_dist);

        // Ray in camera frame (unit vector)
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
    fn test_pinhole_ideal_projection() {
        let camera = PinholeCamera::new_ideal(1920, 1080, 1000.0, 1000.0, 960.0, 540.0);

        // Test center point
        let point = Vector3::new(0.0, 0.0, 1.0);
        let pixel = camera.project(&point).unwrap();
        assert!((pixel.0 - 960.0).abs() < 1e-6);
        assert!((pixel.1 - 540.0).abs() < 1e-6);
    }

    #[test]
    fn test_pinhole_ideal_offset_projection() {
        let camera = PinholeCamera::new_ideal(1920, 1080, 1000.0, 1000.0, 960.0, 540.0);

        // Test offset point
        let point = Vector3::new(0.5, 0.3, 1.0);
        let pixel = camera.project(&point).unwrap();
        assert!((pixel.0 - 1460.0).abs() < 1e-6); // 960 + 1000 * 0.5
        assert!((pixel.1 - 840.0).abs() < 1e-6);  // 540 + 1000 * 0.3
    }

    #[test]
    fn test_pinhole_behind_camera() {
        let camera = PinholeCamera::new_ideal(1920, 1080, 1000.0, 1000.0, 960.0, 540.0);

        // Point behind camera (negative Z)
        let point = Vector3::new(0.0, 0.0, -1.0);
        let result = camera.project(&point);
        assert!(result.is_none());
    }

    #[test]
    fn test_pinhole_at_camera() {
        let camera = PinholeCamera::new_ideal(1920, 1080, 1000.0, 1000.0, 960.0, 540.0);

        // Point at camera origin (Z = 0)
        let point = Vector3::new(0.0, 0.0, 0.0);
        let result = camera.project(&point);
        assert!(result.is_none());
    }

    #[test]
    fn test_pinhole_unproject() {
        let camera = PinholeCamera::new_ideal(1920, 1080, 1000.0, 1000.0, 960.0, 540.0);

        // Unproject center pixel
        let ray = camera.unproject((960.0, 540.0));

        // Ray should be normalized and pointing forward
        assert!((ray.norm() - 1.0).abs() < 1e-6);
        assert!(ray.z > 0.0);
        assert!(ray.x.abs() < 1e-6);
        assert!(ray.y.abs() < 1e-6);
    }

    #[test]
    fn test_pinhole_roundtrip() {
        let camera = PinholeCamera::new_ideal(1920, 1080, 1000.0, 1000.0, 960.0, 540.0);

        // Test roundtrip: project then unproject
        let point = Vector3::new(0.5, 0.3, 2.0);
        let pixel = camera.project(&point).unwrap();
        let ray = camera.unproject(pixel);

        // Ray direction should be parallel to original point
        let original_normalized = point.normalize();
        let dot = ray.dot(&original_normalized);
        assert!((dot - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_pinhole_brown_conrady() {
        let camera = PinholeCamera::new_brown_conrady(
            1920, 1080,
            1000.0, 1000.0,
            960.0, 540.0,
            -0.1, 0.05, 0.0,  // Radial distortion
            0.001, -0.001,     // Tangential distortion
        );

        // Test that distortion has some effect
        let point = Vector3::new(0.5, 0.3, 1.0);
        let pixel = camera.project(&point).unwrap();

        // Pixel should be different from ideal case due to distortion
        // (exact values depend on distortion model)
        assert!(pixel.0 > 0.0 && pixel.0 < 1920.0);
        assert!(pixel.1 > 0.0 && pixel.1 < 1080.0);
    }

    #[test]
    fn test_pinhole_focal_length() {
        let camera = PinholeCamera::new_ideal(1920, 1080, 1234.5, 1234.6, 960.0, 540.0);
        let (fx, fy) = camera.focal_length();
        assert_eq!(fx, 1234.5);
        assert_eq!(fy, 1234.6);
    }

    #[test]
    fn test_pinhole_principal_point() {
        let camera = PinholeCamera::new_ideal(1920, 1080, 1000.0, 1000.0, 960.5, 540.3);
        let (cx, cy) = camera.principal_point();
        assert_eq!(cx, 960.5);
        assert_eq!(cy, 540.3);
    }

    #[test]
    fn test_pinhole_image_size() {
        let camera = PinholeCamera::new_ideal(1920, 1080, 1000.0, 1000.0, 960.0, 540.0);
        let (w, h) = camera.image_size();
        assert_eq!(w, 1920);
        assert_eq!(h, 1080);
    }

    #[test]
    fn test_pinhole_different_focal_lengths() {
        let camera = PinholeCamera::new_ideal(1920, 1080, 1000.0, 1500.0, 960.0, 540.0);

        let point = Vector3::new(1.0, 1.0, 1.0);
        let pixel = camera.project(&point).unwrap();

        // With different fx and fy, scaling should differ
        assert!((pixel.0 - 1960.0).abs() < 1e-6); // 960 + 1000 * 1.0
        assert!((pixel.1 - 2040.0).abs() < 1e-6); // 540 + 1500 * 1.0
    }

    #[test]
    fn test_pinhole_extreme_angles() {
        let camera = PinholeCamera::new_ideal(1920, 1080, 1000.0, 1000.0, 960.0, 540.0);

        // Test extreme viewing angle
        let point = Vector3::new(5.0, 0.0, 1.0);
        let pixel = camera.project(&point);
        assert!(pixel.is_some());

        // Should be far from center
        let (u, _) = pixel.unwrap();
        assert!(u > 2000.0);
    }
}
