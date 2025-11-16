use thiserror::Error;

/// Common errors across the photogrammetry pipeline
#[derive(Error, Debug)]
pub enum RspError {
    #[error("Projection error: {0}")]
    Projection(#[from] ProjectionError),

    #[error("Coordinate transform error: {0}")]
    CoordinateTransform(#[from] CoordinateError),

    #[error("I/O error: {0}")]
    Io(String),

    #[error("Invalid input: {0}")]
    InvalidInput(String),

    #[error("Numerical error: {0}")]
    Numerical(String),
}

#[derive(Error, Debug)]
pub enum ProjectionError {
    #[error("Point behind camera")]
    BehindCamera,

    #[error("Point outside image bounds")]
    OutOfBounds,

    #[error("Invalid RPC coefficients")]
    InvalidRpc,

    #[error("Projection did not converge after {0} iterations")]
    NoConvergence(usize),
}

#[derive(Error, Debug)]
pub enum CoordinateError {
    #[error("Invalid latitude: {0} (must be -90 to 90)")]
    InvalidLatitude(f64),

    #[error("Invalid longitude: {0} (must be -180 to 180)")]
    InvalidLongitude(f64),

    #[error("Invalid height: {0}")]
    InvalidHeight(f64),

    #[error("Coordinate transform failed: {0}")]
    TransformFailed(String),
}

pub type Result<T> = std::result::Result<T, RspError>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_projection_error_display() {
        let err = ProjectionError::BehindCamera;
        assert_eq!(err.to_string(), "Point behind camera");

        let err = ProjectionError::OutOfBounds;
        assert_eq!(err.to_string(), "Point outside image bounds");

        let err = ProjectionError::InvalidRpc;
        assert_eq!(err.to_string(), "Invalid RPC coefficients");

        let err = ProjectionError::NoConvergence(20);
        assert_eq!(err.to_string(), "Projection did not converge after 20 iterations");
    }

    #[test]
    fn test_coordinate_error_display() {
        let err = CoordinateError::InvalidLatitude(95.0);
        assert_eq!(err.to_string(), "Invalid latitude: 95 (must be -90 to 90)");

        let err = CoordinateError::InvalidLongitude(200.0);
        assert_eq!(err.to_string(), "Invalid longitude: 200 (must be -180 to 180)");

        let err = CoordinateError::InvalidHeight(-1000000.0);
        assert_eq!(err.to_string(), "Invalid height: -1000000");
    }

    #[test]
    fn test_rsp_error_from_projection_error() {
        let proj_err = ProjectionError::BehindCamera;
        let rsp_err: RspError = proj_err.into();
        assert!(matches!(rsp_err, RspError::Projection(_)));
    }

    #[test]
    fn test_rsp_error_from_coordinate_error() {
        let coord_err = CoordinateError::InvalidLatitude(95.0);
        let rsp_err: RspError = coord_err.into();
        assert!(matches!(rsp_err, RspError::CoordinateTransform(_)));
    }

    #[test]
    fn test_rsp_error_io() {
        let err = RspError::Io("File not found".to_string());
        assert_eq!(err.to_string(), "I/O error: File not found");
    }

    #[test]
    fn test_rsp_error_invalid_input() {
        let err = RspError::InvalidInput("Invalid parameter".to_string());
        assert_eq!(err.to_string(), "Invalid input: Invalid parameter");
    }

    #[test]
    fn test_rsp_error_numerical() {
        let err = RspError::Numerical("Matrix is singular".to_string());
        assert_eq!(err.to_string(), "Numerical error: Matrix is singular");
    }
}
