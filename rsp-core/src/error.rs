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
