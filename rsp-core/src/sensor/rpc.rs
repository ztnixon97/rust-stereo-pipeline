
use crate::coordinate::{ecef_to_lla, lla_to_ecef, EcefCoord, LlaCoord};
use crate::error::{ProjectionError, Result};

/// RPC (Rational Polynomial Coefficients) for satellite imagery
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

/// RPC sensor model for ground-to-image and image-to-ground projection
#[derive(Debug, Clone)]
pub struct RpcModel {
    coeffs: RpcCoefficients,
}

impl RpcModel {
    /// Create a new RPC model from coefficients
    pub fn new(coeffs: RpcCoefficients) -> Self {
        Self { coeffs }
    }
    
    /// Get reference to coefficients
    pub fn coefficients(&self) -> &RpcCoefficients {
        &self.coeffs
    }
    
    /// Project ground point (ECEF) to image coordinates (line, sample)
    pub fn ground_to_image(&self, ground_ecef: &EcefCoord) -> Result<(f64, f64)> {
        // Convert ECEF to LLA
        let lla = ecef_to_lla(ground_ecef)?;
        self.lla_to_image(&lla)
    }
    
    /// Project LLA to image coordinates (line, sample)
    pub fn lla_to_image(&self, lla: &LlaCoord) -> Result<(f64, f64)> {
        // Normalize coordinates
        let p = (lla.lon - self.coeffs.lon_off) / self.coeffs.lon_scale;
        let l = (lla.lat - self.coeffs.lat_off) / self.coeffs.lat_scale;
        let h = (lla.alt - self.coeffs.height_off) / self.coeffs.height_scale;
        
        // Evaluate rational polynomials
        let line_num = eval_polynomial(&self.coeffs.line_num_coeff, p, l, h);
        let line_den = eval_polynomial(&self.coeffs.line_den_coeff, p, l, h);
        let samp_num = eval_polynomial(&self.coeffs.samp_num_coeff, p, l, h);
        let samp_den = eval_polynomial(&self.coeffs.samp_den_coeff, p, l, h);
        
        if line_den.abs() < 1e-10 || samp_den.abs() < 1e-10 {
            return Err(ProjectionError::InvalidRpc.into());
        }
        
        // Denormalize
        let line = line_num / line_den * self.coeffs.line_scale + self.coeffs.line_off;
        let samp = samp_num / samp_den * self.coeffs.samp_scale + self.coeffs.samp_off;
        
        Ok((line, samp))
    }
    
    /// Project image coordinates to ground point at given height (ECEF)
    /// Uses Newton-Raphson iteration to invert the RPC
    pub fn image_to_ground(&self, line: f64, sample: f64, height: f64) -> Result<EcefCoord> {
        let lla = self.image_to_lla(line, sample, height)?;
        lla_to_ecef(&lla)
    }
    
    /// Project image coordinates to LLA at given height
    pub fn image_to_lla(&self, line: f64, sample: f64, height: f64) -> Result<LlaCoord> {
        // Initial guess - use center of RPC normalization
        let mut lat = self.coeffs.lat_off;
        let mut lon = self.coeffs.lon_off;
        
        // Newton-Raphson iteration
        for iter in 0..20 {
            let lla = LlaCoord { lat, lon, alt: height };
            let (proj_line, proj_samp) = self.lla_to_image(&lla)?;
            
            let line_err = line - proj_line;
            let samp_err = sample - proj_samp;
            
            // Check convergence
            if line_err.abs() < 1e-6 && samp_err.abs() < 1e-6 {
                return Ok(lla);
            }
            
            // Compute Jacobian using finite differences
            let delta = 1e-7;
            
            let lla_lat_plus = LlaCoord { lat: lat + delta, lon, alt: height };
            let (line_lat_plus, samp_lat_plus) = self.lla_to_image(&lla_lat_plus)?;
            let dline_dlat = (line_lat_plus - proj_line) / delta;
            let dsamp_dlat = (samp_lat_plus - proj_samp) / delta;
            
            let lla_lon_plus = LlaCoord { lat, lon: lon + delta, alt: height };
            let (line_lon_plus, samp_lon_plus) = self.lla_to_image(&lla_lon_plus)?;
            let dline_dlon = (line_lon_plus - proj_line) / delta;
            let dsamp_dlon = (samp_lon_plus - proj_samp) / delta;
            
            // Solve 2x2 system: J * [dlat, dlon]' = [line_err, samp_err]'
            let det = dline_dlat * dsamp_dlon - dline_dlon * dsamp_dlat;
            
            if det.abs() < 1e-10 {
                return Err(ProjectionError::NoConvergence(iter).into());
            }
            
            let dlat = (dsamp_dlon * line_err - dline_dlon * samp_err) / det;
            let dlon = (dline_dlat * samp_err - dsamp_dlat * line_err) / det;
            
            lat += dlat;
            lon += dlon;
        }
        
        Err(ProjectionError::NoConvergence(20).into())
    }
}

/// Evaluate RPC polynomial with 20 coefficients
fn eval_polynomial(coeffs: &[f64; 20], p: f64, l: f64, h: f64) -> f64 {
    coeffs[0]
        + coeffs[1] * l
        + coeffs[2] * p
        + coeffs[3] * h
        + coeffs[4] * l * p
        + coeffs[5] * l * h
        + coeffs[6] * p * h
        + coeffs[7] * l * l
        + coeffs[8] * p * p
        + coeffs[9] * h * h
        + coeffs[10] * p * l * h
        + coeffs[11] * l * l * l
        + coeffs[12] * l * p * p
        + coeffs[13] * l * h * h
        + coeffs[14] * l * l * p
        + coeffs[15] * p * p * p
        + coeffs[16] * p * h * h
        + coeffs[17] * l * l * h
        + coeffs[18] * p * p * h
        + coeffs[19] * h * h * h
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::error::{RspError, ProjectionError};

    fn create_simple_rpc() -> RpcCoefficients {
        let mut coeffs = RpcCoefficients {
            line_num_coeff: [0.0; 20],
            line_den_coeff: [0.0; 20],
            samp_num_coeff: [0.0; 20],
            samp_den_coeff: [0.0; 20],
            lat_off: 39.0,
            lat_scale: 1.0,
            lon_off: -77.0,
            lon_scale: 1.0,
            height_off: 100.0,
            height_scale: 500.0,
            line_off: 5000.0,
            line_scale: 5000.0,
            samp_off: 5000.0,
            samp_scale: 5000.0,
        };

        // Simple linear RPC (just for testing)
        coeffs.line_num_coeff[1] = 1.0;  // lat term
        coeffs.line_den_coeff[0] = 1.0;
        coeffs.samp_num_coeff[2] = 1.0;  // lon term
        coeffs.samp_den_coeff[0] = 1.0;

        coeffs
    }

    #[test]
    fn test_rpc_roundtrip() {
        let coeffs = create_simple_rpc();
        let rpc = RpcModel::new(coeffs);

        // Test forward projection
        let lla = LlaCoord {
            lat: 39.1,
            lon: -76.9,
            alt: 100.0,
        };

        let (line, samp) = rpc.lla_to_image(&lla).unwrap();

        // Test inverse projection (should get close to original)
        let lla2 = rpc.image_to_lla(line, samp, 100.0).unwrap();

        assert!((lla.lat - lla2.lat).abs() < 1e-3);
        assert!((lla.lon - lla2.lon).abs() < 1e-3);
    }

    #[test]
    fn test_rpc_ground_to_image() {
        let coeffs = create_simple_rpc();
        let rpc = RpcModel::new(coeffs);

        let lla = LlaCoord {
            lat: 39.0,
            lon: -77.0,
            alt: 100.0,
        };

        // Convert to ECEF
        let ecef = lla_to_ecef(&lla).unwrap();

        // Project to image using ECEF
        let (line, samp) = rpc.ground_to_image(&ecef).unwrap();

        // Verify it's in reasonable range
        assert!(line > 0.0);
        assert!(samp > 0.0);
    }

    #[test]
    fn test_rpc_image_to_ground() {
        let coeffs = create_simple_rpc();
        let rpc = RpcModel::new(coeffs);

        let line = 5000.0;
        let samp = 5000.0;
        let height = 100.0;

        // This should converge to a point
        let result = rpc.image_to_ground(line, samp, height);
        assert!(result.is_ok());

        let ecef = result.unwrap();
        // Verify it's a reasonable ECEF coordinate
        let magnitude = (ecef.x * ecef.x + ecef.y * ecef.y + ecef.z * ecef.z).sqrt();
        assert!(magnitude > 6_000_000.0); // Should be at least Earth radius
        assert!(magnitude < 7_000_000.0); // But not too far
    }

    #[test]
    fn test_rpc_coefficients_access() {
        let coeffs = create_simple_rpc();
        let rpc = RpcModel::new(coeffs.clone());

        let retrieved = rpc.coefficients();
        assert_eq!(retrieved.lat_off, coeffs.lat_off);
        assert_eq!(retrieved.lon_off, coeffs.lon_off);
        assert_eq!(retrieved.height_off, coeffs.height_off);
    }

    #[test]
    fn test_rpc_multiple_points() {
        let coeffs = create_simple_rpc();
        let rpc = RpcModel::new(coeffs);

        let test_points = vec![
            LlaCoord { lat: 38.8, lon: -77.1, alt: 100.0 },
            LlaCoord { lat: 39.0, lon: -77.0, alt: 100.0 },
            LlaCoord { lat: 39.2, lon: -76.9, alt: 100.0 },
        ];

        for lla in test_points {
            let (line, samp) = rpc.lla_to_image(&lla).unwrap();
            let lla2 = rpc.image_to_lla(line, samp, lla.alt).unwrap();

            assert!((lla.lat - lla2.lat).abs() < 1e-3);
            assert!((lla.lon - lla2.lon).abs() < 1e-3);
        }
    }

    #[test]
    fn test_rpc_different_heights() {
        let coeffs = create_simple_rpc();
        let rpc = RpcModel::new(coeffs);

        let lat = 39.0;
        let lon = -77.0;
        let heights = vec![0.0, 100.0, 500.0, 1000.0];

        for height in heights {
            let lla = LlaCoord { lat, lon, alt: height };
            let (line, samp) = rpc.lla_to_image(&lla).unwrap();
            let lla2 = rpc.image_to_lla(line, samp, height).unwrap();

            assert!((lla.lat - lla2.lat).abs() < 1e-3);
            assert!((lla.lon - lla2.lon).abs() < 1e-3);
        }
    }

    #[test]
    fn test_eval_polynomial() {
        // Test polynomial evaluation with known values
        let coeffs = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0,
                      11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0];

        let p = 0.0;
        let l = 0.0;
        let h = 0.0;

        // With all zeros, only constant term should contribute
        let result = eval_polynomial(&coeffs, p, l, h);
        assert_eq!(result, 1.0);
    }

    #[test]
    fn test_rpc_normalization() {
        let coeffs = create_simple_rpc();
        let rpc = RpcModel::new(coeffs);

        // Test that coordinates near the offset work properly
        let lla = LlaCoord {
            lat: 39.0,  // Equal to lat_off
            lon: -77.0, // Equal to lon_off
            alt: 100.0, // Equal to height_off
        };

        let result = rpc.lla_to_image(&lla);
        assert!(result.is_ok());
    }

    #[test]
    fn test_rpc_zero_denominator() {
        // Create RPC with potential zero denominator
        let mut coeffs = create_simple_rpc();

        // Set denominator coefficients to all zeros (which would cause division by zero)
        coeffs.line_den_coeff = [0.0; 20];
        coeffs.samp_den_coeff = [0.0; 20];

        let rpc = RpcModel::new(coeffs);

        let lla = LlaCoord {
            lat: 39.0,
            lon: -77.0,
            alt: 100.0,
        };

        let result = rpc.lla_to_image(&lla);
        assert!(result.is_err());
        assert!(matches!(result.unwrap_err(), RspError::Projection(ProjectionError::InvalidRpc)));
    }
}
