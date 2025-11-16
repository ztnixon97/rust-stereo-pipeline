use gdal::Dataset;
use nalgebra::{Vector3, UnitQuaternion};
use rsp_core::sensor::RpcCoefficients;
use rsp_core::error::{RspError, Result};

/// Container for all image metadata
#[derive(Debug, Clone, Default)]
pub struct ImageMetadata {
    pub rpc: Option<RpcCoefficients>,
    pub gps_position: Option<Vector3<f64>>,
    pub imu_orientation: Option<UnitQuaternion<f64>>,
    pub timestamp: Option<f64>,
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
fn extract_rpc(dataset: &Dataset) -> Result<RpcCoefficients> {
    let metadata = dataset
        .metadata_domain("RPC")
        .ok_or_else(|| RspError::Io("RPC metadata not found".to_string()))?;
    
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

fn parse_coeff_array(
    metadata: &std::collections::HashMap<String, String>,
    prefix: &str,
) -> Result<[f64; 20]> {
    let mut coeffs = [0.0; 20];
    
    for i in 1..=20 {
        let key = format!("{}_{}", prefix, i);
        let value = metadata
            .get(&key)
            .ok_or_else(|| RspError::Io(format!("Missing RPC parameter: {}", key)))?;
        
        coeffs[i - 1] = value
            .trim()
            .parse()
            .map_err(|_| RspError::Io(format!("Failed to parse RPC coefficient: {}", key)))?;
    }
    
    Ok(coeffs)
}

fn parse_single(
    metadata: &std::collections::HashMap<String, String>,
    key: &str,
) -> Result<f64> {
    let value = metadata
        .get(key)
        .ok_or_else(|| RspError::Io(format!("Missing RPC parameter: {}", key)))?;
    
    value
        .trim()
        .parse()
        .map_err(|_| RspError::Io(format!("Failed to parse RPC parameter: {}", key)))
}
