pub mod image;
pub mod metadata;

pub use image::Image;
pub use metadata::ImageMetadata;

// Re-export from rsp-core for convenience
pub use rsp_core::sensor::RpcCoefficients;
pub use rsp_core::error::{RspError, Result};
