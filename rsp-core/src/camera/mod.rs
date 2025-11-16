//! Camera models and projections

mod distortion;
mod fisheye;
mod pinhole;

pub use distortion::DistortionError;
pub use fisheye::FisheyeCamera;
pub use pinhole::PinholeCamera;

use nalgebra::Vector3;

/// Generic CameraModel
pub trait CameraModel {
    /// Project 3D point in camera frame to image coordinates
    /// Returns None if point is behind camera

    fn project(&self, point_camera: &Vector3<f64>) -> Option<(f64, f64)>;

    /// Unproject image coordinates to unit ray in camera frame
    fn unproject(&self, pixel: (f64, f64)) -> Result<Vector3<f64>, DistortionError>;

    /// Get image dimesnsions this camera is calibrated for
    fn image_size(&self) -> (usize, usize);
}
