
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
    let mut alt = 0.0;
    let mut n;

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
    use crate::error::RspError;

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

    #[test]
    fn test_equator_prime_meridian() {
        // Test point on equator at prime meridian
        let lla = LlaCoord {
            lat: 0.0,
            lon: 0.0,
            alt: 0.0,
        };

        let ecef = lla_to_ecef(&lla).unwrap();

        // At equator, prime meridian: x should be ~semi-major axis, y and z should be ~0
        assert!((ecef.x - WGS84_A).abs() < 1.0);
        assert!(ecef.y.abs() < 1.0);
        assert!(ecef.z.abs() < 1.0);

        // Round trip
        let lla2 = ecef_to_lla(&ecef).unwrap();
        assert!((lla.lat - lla2.lat).abs() < 1e-6);
        assert!((lla.lon - lla2.lon).abs() < 1e-6);
        assert!((lla.alt - lla2.alt).abs() < 1e-3);
    }

    #[test]
    fn test_north_pole() {
        let lla = LlaCoord {
            lat: 90.0,
            lon: 0.0,
            alt: 1000.0,
        };

        let ecef = lla_to_ecef(&lla).unwrap();
        let lla2 = ecef_to_lla(&ecef).unwrap();

        // At north pole, x and y should be very small, z should be large
        assert!(ecef.x.abs() < 1.0);
        assert!(ecef.y.abs() < 1.0);
        assert!(ecef.z > 6_000_000.0); // Should be ~6.36M meters

        assert!((lla.lat - lla2.lat).abs() < 1e-6);
        assert!((lla.alt - lla2.alt).abs() < 1e-3);
    }

    #[test]
    fn test_south_pole() {
        let lla = LlaCoord {
            lat: -90.0,
            lon: 0.0,
            alt: 1000.0,
        };

        let ecef = lla_to_ecef(&lla).unwrap();
        let lla2 = ecef_to_lla(&ecef).unwrap();

        // At south pole, x and y should be very small, z should be large negative
        assert!(ecef.x.abs() < 1.0);
        assert!(ecef.y.abs() < 1.0);
        assert!(ecef.z < -6_000_000.0);

        assert!((lla.lat - lla2.lat).abs() < 1e-6);
        assert!((lla.alt - lla2.alt).abs() < 1e-3);
    }

    #[test]
    fn test_high_altitude() {
        let lla = LlaCoord {
            lat: 45.0,
            lon: 90.0,
            alt: 500000.0, // 500km altitude (ISS orbit)
        };

        let ecef = lla_to_ecef(&lla).unwrap();
        let lla2 = ecef_to_lla(&ecef).unwrap();

        assert!((lla.lat - lla2.lat).abs() < 1e-6);
        assert!((lla.lon - lla2.lon).abs() < 1e-6);
        assert!((lla.alt - lla2.alt).abs() < 1e-1);
    }

    #[test]
    fn test_negative_altitude() {
        // Dead Sea is about 430m below sea level
        let lla = LlaCoord {
            lat: 31.5,
            lon: 35.5,
            alt: -430.0,
        };

        let ecef = lla_to_ecef(&lla).unwrap();
        let lla2 = ecef_to_lla(&ecef).unwrap();

        assert!((lla.lat - lla2.lat).abs() < 1e-6);
        assert!((lla.lon - lla2.lon).abs() < 1e-6);
        assert!((lla.alt - lla2.alt).abs() < 1e-3);
    }

    #[test]
    fn test_invalid_latitude_positive() {
        let lla = LlaCoord {
            lat: 95.0,
            lon: 0.0,
            alt: 0.0,
        };

        let result = lla_to_ecef(&lla);
        assert!(result.is_err());
        assert!(matches!(result.unwrap_err(), RspError::CoordinateTransform(_)));
    }

    #[test]
    fn test_invalid_latitude_negative() {
        let lla = LlaCoord {
            lat: -95.0,
            lon: 0.0,
            alt: 0.0,
        };

        let result = lla_to_ecef(&lla);
        assert!(result.is_err());
    }

    #[test]
    fn test_longitude_wraparound() {
        // Longitude values outside [-180, 180] should still work
        // Testing 181 degrees (same as -179)
        let lla1 = LlaCoord {
            lat: 40.0,
            lon: 181.0,
            alt: 100.0,
        };

        let lla2 = LlaCoord {
            lat: 40.0,
            lon: -179.0,
            alt: 100.0,
        };

        let ecef1 = lla_to_ecef(&lla1).unwrap();
        let ecef2 = lla_to_ecef(&lla2).unwrap();

        // These should produce nearly identical ECEF coordinates
        assert!((ecef1.x - ecef2.x).abs() < 1.0);
        assert!((ecef1.y - ecef2.y).abs() < 1.0);
        assert!((ecef1.z - ecef2.z).abs() < 1.0);
    }

    #[test]
    fn test_known_location_tokyo() {
        // Tokyo, Japan
        let lla = LlaCoord {
            lat: 35.6762,
            lon: 139.6503,
            alt: 40.0,
        };

        let ecef = lla_to_ecef(&lla).unwrap();
        let lla2 = ecef_to_lla(&ecef).unwrap();

        assert!((lla.lat - lla2.lat).abs() < 1e-6);
        assert!((lla.lon - lla2.lon).abs() < 1e-6);
        assert!((lla.alt - lla2.alt).abs() < 1e-3);
    }

    #[test]
    fn test_known_location_sydney() {
        // Sydney, Australia (Southern hemisphere)
        let lla = LlaCoord {
            lat: -33.8688,
            lon: 151.2093,
            alt: 50.0,
        };

        let ecef = lla_to_ecef(&lla).unwrap();
        let lla2 = ecef_to_lla(&ecef).unwrap();

        assert!((lla.lat - lla2.lat).abs() < 1e-6);
        assert!((lla.lon - lla2.lon).abs() < 1e-6);
        assert!((lla.alt - lla2.alt).abs() < 1e-3);
    }
}
