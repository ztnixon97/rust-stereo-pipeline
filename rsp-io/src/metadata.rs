use gdal::{Dataset, Metadata};
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
    let metadata_vec = dataset
        .metadata_domain("RPC")
        .ok_or_else(|| RspError::Io("RPC metadata not found".to_string()))?;

    // Convert Vec<String> of "KEY=VALUE" pairs to HashMap
    let mut metadata = std::collections::HashMap::new();
    for item in metadata_vec.iter() {
        // item is &String, convert to &str first
        let item_str: &str = &item;
        let parts: Vec<&str> = item_str.splitn(2, '=').collect();
        if parts.len() == 2 {
            metadata.insert(parts[0].to_string(), parts[1].to_string());
        }
    }

    if metadata.is_empty() {
        return Err(RspError::Io("RPC metadata not found or empty".to_string()));
    }
    
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_image_metadata_default() {
        let metadata = ImageMetadata::default();
        assert!(metadata.rpc.is_none());
        assert!(metadata.gps_position.is_none());
        assert!(metadata.imu_orientation.is_none());
        assert!(metadata.timestamp.is_none());
        assert!(metadata.camera_id.is_none());
    }

    #[test]
    fn test_image_metadata_has_rpc() {
        let mut metadata = ImageMetadata::default();
        assert!(!metadata.has_rpc());

        // Create minimal RPC coefficients
        let rpc = RpcCoefficients {
            line_num_coeff: [0.0; 20],
            line_den_coeff: [0.0; 20],
            samp_num_coeff: [0.0; 20],
            samp_den_coeff: [0.0; 20],
            lat_off: 0.0,
            lat_scale: 1.0,
            lon_off: 0.0,
            lon_scale: 1.0,
            height_off: 0.0,
            height_scale: 1.0,
            line_off: 0.0,
            line_scale: 1.0,
            samp_off: 0.0,
            samp_scale: 1.0,
        };

        metadata.rpc = Some(rpc);
        assert!(metadata.has_rpc());
    }

    #[test]
    fn test_parse_coeff_array_success() {
        let mut metadata = std::collections::HashMap::new();
        for i in 1..=20 {
            metadata.insert(format!("TEST_COEFF_{}", i), format!("{}.0", i));
        }

        let result = parse_coeff_array(&metadata, "TEST_COEFF");
        assert!(result.is_ok());

        let coeffs = result.unwrap();
        assert_eq!(coeffs.len(), 20);
        assert_eq!(coeffs[0], 1.0);
        assert_eq!(coeffs[19], 20.0);
    }

    #[test]
    fn test_parse_coeff_array_missing_coefficient() {
        let mut metadata = std::collections::HashMap::new();
        // Only add 19 coefficients instead of 20
        for i in 1..=19 {
            metadata.insert(format!("TEST_COEFF_{}", i), format!("{}.0", i));
        }

        let result = parse_coeff_array(&metadata, "TEST_COEFF");
        assert!(result.is_err());
    }

    #[test]
    fn test_parse_coeff_array_invalid_value() {
        let mut metadata = std::collections::HashMap::new();
        for i in 1..=20 {
            if i == 10 {
                metadata.insert(format!("TEST_COEFF_{}", i), "not_a_number".to_string());
            } else {
                metadata.insert(format!("TEST_COEFF_{}", i), format!("{}.0", i));
            }
        }

        let result = parse_coeff_array(&metadata, "TEST_COEFF");
        assert!(result.is_err());
    }

    #[test]
    fn test_parse_single_success() {
        let mut metadata = std::collections::HashMap::new();
        metadata.insert("TEST_PARAM".to_string(), "42.5".to_string());

        let result = parse_single(&metadata, "TEST_PARAM");
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), 42.5);
    }

    #[test]
    fn test_parse_single_with_whitespace() {
        let mut metadata = std::collections::HashMap::new();
        metadata.insert("TEST_PARAM".to_string(), "  42.5  ".to_string());

        let result = parse_single(&metadata, "TEST_PARAM");
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), 42.5);
    }

    #[test]
    fn test_parse_single_missing() {
        let metadata = std::collections::HashMap::new();

        let result = parse_single(&metadata, "MISSING_PARAM");
        assert!(result.is_err());
    }

    #[test]
    fn test_parse_single_invalid() {
        let mut metadata = std::collections::HashMap::new();
        metadata.insert("TEST_PARAM".to_string(), "not_a_number".to_string());

        let result = parse_single(&metadata, "TEST_PARAM");
        assert!(result.is_err());
    }

    #[test]
    fn test_image_metadata_clone() {
        let metadata1 = ImageMetadata {
            rpc: None,
            gps_position: Some(Vector3::new(1.0, 2.0, 3.0)),
            imu_orientation: None,
            timestamp: Some(12345.6),
            camera_id: Some("CAM01".to_string()),
        };

        let metadata2 = metadata1.clone();
        assert!(metadata2.gps_position.is_some());
        assert_eq!(metadata2.timestamp, Some(12345.6));
        assert_eq!(metadata2.camera_id, Some("CAM01".to_string()));
    }
}
