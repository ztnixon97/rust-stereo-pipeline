
use nalgebra::Vector3;
use crate::error::{CoordinateError, Result};

/// ECEF coordinates (Earth-Centered, Earth-Fixed)
pub type EcefCoord = Vector3<f64>;

/// LLA coordinates (Latitude, Longitude, Altitude)
#[derive(Debug, Clone, Copy)]
pub struct LlaCoord {
    pub lat: f64,  // degrees
    pub lon: f64,  // degrees
    pub alt: f64,  // meters above WGS84 ellipsoid
}

// WGS84 ellipsoid parameters
const WGS84_A: f64 = 6378137.0;              // semi-major axis (meters)
const WGS84_E2: f64 = 0.00669437999014;      // first eccentricity squared

/// Convert ECEF to LLA (Latitude, Longitude, Altitude)
pub fn ecef_to_lla(ecef: &EcefCoord) -> Result<LlaCoord> {
    let x = ecef.x;
    let y = ecef.y;
    let z = ecef.z;
    
    let p = (x * x + y * y).sqrt();
    
    // Longitude
    let lon = y.atan2(x).to_degrees();
    
    // Iterative solution for latitude and altitude
    let mut lat = (z / p).atan();
    let mut n = WGS84_A;
    let mut alt = 0.0;
    
    for _ in 0..10 {
        let sin_lat = lat.sin();
        n = WGS84_A / (1.0 - WGS84_E2 * sin_lat * sin_lat).sqrt();
        alt = p / lat.cos() - n;
        lat = (z / p / (1.0 - WGS84_E2 * n / (n + alt))).atan();
    }
    
    let lat_deg = lat.to_degrees();
    
    if lat_deg < -90.0 || lat_deg > 90.0 {
        return Err(CoordinateError::InvalidLatitude(lat_deg).into());
    }
    
    Ok(LlaCoord {
        lat: lat_deg,
        lon,
        alt,
    })
}

/// Convert LLA to ECEF
pub fn lla_to_ecef(lla: &LlaCoord) -> Result<EcefCoord> {
    if lla.lat < -90.0 || lla.lat > 90.0 {
        return Err(CoordinateError::InvalidLatitude(lla.lat).into());
    }
    
    let lat_rad = lla.lat.to_radians();
    let lon_rad = lla.lon.to_radians();
    
    let sin_lat = lat_rad.sin();
    let cos_lat = lat_rad.cos();
    let sin_lon = lon_rad.sin();
    let cos_lon = lon_rad.cos();
    
    let n = WGS84_A / (1.0 - WGS84_E2 * sin_lat * sin_lat).sqrt();
    
    let x = (n + lla.alt) * cos_lat * cos_lon;
    let y = (n + lla.alt) * cos_lat * sin_lon;
    let z = (n * (1.0 - WGS84_E2) + lla.alt) * sin_lat;
    
    Ok(Vector3::new(x, y, z))
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_lla_ecef_roundtrip() {
        let lla = LlaCoord {
            lat: 38.8977,   // Washington DC
            lon: -77.0365,
            alt: 100.0,
        };
        
        let ecef = lla_to_ecef(&lla).unwrap();
        let lla2 = ecef_to_lla(&ecef).unwrap();
        
        assert!((lla.lat - lla2.lat).abs() < 1e-6);
        assert!((lla.lon - lla2.lon).abs() < 1e-6);
        assert!((lla.alt - lla2.alt).abs() < 1e-3);
    }
}
