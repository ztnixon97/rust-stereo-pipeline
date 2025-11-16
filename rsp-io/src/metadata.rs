use gdal::{Dataset, Metadata};
use std::collections::HashMap;
use nalgebra::{UnitQuaternion, Vector3};
use thiserror::Error;

/// RPC coefficients extracted from image metadata
#[derive(Debug, Clone)]
pub struct RpcCoefficients {
    // Polynomial coefficients (20 each)
    pub line_num_coeff: [f64; 20],
    pub line_den_coeff: [f64; 20],
    pub samp_num_coeff: [f64; 20],
    pub samp_den_coeff: [f64; 20],

    // Normalization offsets and scales
    pub lat_off: f64,
    pub lat_scale: f64,
    pub lon_off: f64,
    pub lon_scale: f64,
    pub height_off: f64,
    pub height_scale: f64,
    pub line_off: f64,
    pub line_scale: f64,
    pub samp_off: f64,
    pub samp_scale: f64,
}

#[derive(Error, Debug)]
pub enum MetadataError {
    #[error("RPC metadata not found")]
    RpcNotFound,
    #[error("Failed to parse coefficient: {0}")]
    ParseError(String),
    #[error("Missing parameter: {0}")]
    MissingParameter(String),
}

/// Container for all image metadata
#[derive(Debug, Clone, Default)]
pub struct ImageMetadata {
    pub rpc: Option<RpcCoefficients>,

    // External metadata (from trajectory files, etc.)
    pub gps_position: Option<Vector3<f64>>, // ECEF
    pub imu_orientation: Option<UnitQuaternion<f64>>,
    pub timestamp: Option<f64>,

    // Reference to camera model (stored separately)
    pub camera_id: Option<String>,
}

impl ImageMetadata {
    /// Extract all available metadata from GDAL dataset
    pub fn from_gdal_dataset(dataset: &Dataset) -> Self {
        Self {
            rpc: extract_rpc(dataset).ok(),
            ..Default::default()
        }
    }

    /// Check if image has RPC
    pub fn has_rpc(&self) -> bool {
        self.rpc.is_some()
    }
}

/// Extract RPC from GDAL dataset
fn extract_rpc(dataset: &Dataset) -> Result<RpcCoefficients, MetadataError> {
    let metadata = dataset
        .metadata_domain("RPC")
        .ok_or(MetadataError::RpcNotFound)?;

    let metadata: HashMap<String, String> = metadata
        .into_iter()
        .filter_map(|entry| {
            entry
                .split_once('=')
                .map(|(k, v)| (k.to_owned(), v.to_owned()))
        })
        .collect();

    Ok(RpcCoefficients {
        line_num_coeff: parse_coeff_array(&metadata, "LINE_NUM_COEFF")?,
        line_den_coeff: parse_coeff_array(&metadata, "LINE_DEN_COEFF")?,
        samp_num_coeff: parse_coeff_array(&metadata, "SAMP_NUM_COEFF")?,
        samp_den_coeff: parse_coeff_array(&metadata, "SAMP_DEN_COEFF")?,

        lat_off: parse_single(&metadata, "LAT_OFF")?,
        lat_scale: parse_single(&metadata, "LAT_SCALE")?,
        lon_off: parse_single(&metadata, "LONG_OFF")?,
        lon_scale: parse_single(&metadata, "LONG_SCALE")?,
        height_off: parse_single(&metadata, "HEIGHT_OFF")?,
        height_scale: parse_single(&metadata, "HEIGHT_SCALE")?,
        line_off: parse_single(&metadata, "LINE_OFF")?,
        line_scale: parse_single(&metadata, "LINE_SCALE")?,
        samp_off: parse_single(&metadata, "SAMP_OFF")?,
        samp_scale: parse_single(&metadata, "SAMP_SCALE")?,
    })
}

/// Parse array of 20 coefficients from GDAL metadata
fn parse_coeff_array(
    metadata: &std::collections::HashMap<String, String>,
    prefix: &str,
) -> Result<[f64; 20], MetadataError> {
    let mut coeffs = [0.0; 20];

    for i in 1..=20 {
        let key = format!("{}_{}", prefix, i);
        let value = metadata
            .get(&key)
            .ok_or_else(|| MetadataError::MissingParameter(key.clone()))?;

        coeffs[i - 1] = value
            .trim()
            .parse()
            .map_err(|_| MetadataError::ParseError(key))?;
    }

    Ok(coeffs)
}

/// Parse single scalar value from GDAL metadata
fn parse_single(
    metadata: &std::collections::HashMap<String, String>,
    key: &str,
) -> Result<f64, MetadataError> {
    let value = metadata
        .get(key)
        .ok_or_else(|| MetadataError::MissingParameter(key.to_string()))?;

    value
        .trim()
        .parse()
        .map_err(|_| MetadataError::ParseError(key.to_string()))
}
