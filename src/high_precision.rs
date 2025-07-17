use nalgebra::{Matrix3, Vector3, Matrix6, Vector6};
use crate::{Anchor, Position, SPEED_OF_SOUND_WATER};
use std::f64::consts::PI;

/// High-precision coordinate transformations and error propagation for sub-meter accuracy
pub struct HighPrecisionTransformer {
    /// Earth semi-major axis (WGS84) in meters
    pub earth_radius_a: f64,
    /// Earth semi-minor axis (WGS84) in meters
    pub earth_radius_b: f64,
    /// Earth flattening factor
    pub flattening: f64,
    /// Earth eccentricity squared
    pub eccentricity_squared: f64,
    /// Reference position for local tangent plane
    pub reference_position: Option<Position>,
    /// Cached transformation matrices
    cached_enu_matrix: Option<Matrix3<f64>>,
    /// Cached reference point in ECEF
    cached_reference_ecef: Option<Vector3<f64>>,
}

/// Environmental correction parameters for sound speed
pub struct EnvironmentalCorrections {
    /// Base sound speed in water (m/s)
    pub base_sound_speed: f64,
    /// Water temperature (Celsius)
    pub temperature: f64,
    /// Salinity (parts per thousand)
    pub salinity: f64,
    /// Depth (meters)
    pub depth: f64,
    /// Pressure (dbar)
    pub pressure: f64,
}

/// Error propagation result for position uncertainty
pub struct PositionUncertainty {
    /// Position covariance matrix (3x3)
    pub covariance: Matrix3<f64>,
    /// Standard deviation in x direction (meters)
    pub std_dev_x: f64,
    /// Standard deviation in y direction (meters)
    pub std_dev_y: f64,
    /// Standard deviation in z direction (meters)
    pub std_dev_z: f64,
    /// 95% confidence radius (meters)
    pub confidence_radius_95: f64,
}

impl Default for HighPrecisionTransformer {
    fn default() -> Self {
        // WGS84 ellipsoid parameters
        let a = 6378137.0; // semi-major axis in meters
        let f = 1.0 / 298.257223563; // flattening
        let b = a * (1.0 - f); // semi-minor axis
        let e_squared = f * (2.0 - f); // eccentricity squared
        
        Self {
            earth_radius_a: a,
            earth_radius_b: b,
            flattening: f,
            eccentricity_squared: e_squared,
            reference_position: None,
            cached_enu_matrix: None,
            cached_reference_ecef: None,
        }
    }
}

impl HighPrecisionTransformer {
    /// Create a new transformer with default WGS84 parameters
    pub fn new() -> Self {
        Self::default()
    }
    
    /// Set reference position for local coordinate transformations
    pub fn set_reference_position(&mut self, reference: Position) {
        self.reference_position = Some(reference);
        self.cached_enu_matrix = None;
        self.cached_reference_ecef = None;
    }
    
    /// Convert geodetic coordinates (lat, lon, height) to ECEF (Earth-Centered, Earth-Fixed)
    pub fn geodetic_to_ecef(&self, position: &Position) -> Vector3<f64> {
        let lat_rad = position.lat.to_radians();
        let lon_rad = position.lon.to_radians();
        let height = -position.depth; // Convert depth to height (negative depth = positive height)
        
        // Calculate radius of curvature in the prime vertical
        let n = self.earth_radius_a / (1.0 - self.eccentricity_squared * lat_rad.sin().powi(2)).sqrt();
        
        // Calculate ECEF coordinates
        let x = (n + height) * lat_rad.cos() * lon_rad.cos();
        let y = (n + height) * lat_rad.cos() * lon_rad.sin();
        let z = (n * (1.0 - self.eccentricity_squared) + height) * lat_rad.sin();
        
        Vector3::new(x, y, z)
    }
    
    /// Convert ECEF coordinates to geodetic (lat, lon, height)
    pub fn ecef_to_geodetic(&self, ecef: &Vector3<f64>) -> Position {
        let x = ecef.x;
        let y = ecef.y;
        let z = ecef.z;
        
        let p = (x.powi(2) + y.powi(2)).sqrt();
        let theta = (z * self.earth_radius_a).atan2(p * self.earth_radius_b);
        
        let lat_rad = (z + self.eccentricity_squared.powi(2) * self.earth_radius_b * theta.sin().powi(3)) /
                     (p - self.eccentricity_squared * self.earth_radius_a * theta.cos().powi(3))
                     .atan();
        
        let lon_rad = y.atan2(x);
        
        // Calculate height above ellipsoid
        let n = self.earth_radius_a / (1.0 - self.eccentricity_squared * lat_rad.sin().powi(2)).sqrt();
        let height = p / lat_rad.cos() - n;
        
        Position {
            lat: lat_rad.to_degrees(),
            lon: lon_rad.to_degrees(),
            depth: -height, // Convert height to depth
        }
    }
    
    /// Convert ECEF coordinates to ENU (East-North-Up) local coordinates
    pub fn ecef_to_enu(&self, ecef: &Vector3<f64>, reference_ecef: &Vector3<f64>, enu_matrix: &Matrix3<f64>) -> Vector3<f64> {
        // Calculate ECEF offset from reference
        let delta_ecef = ecef - reference_ecef;
        
        // Transform to ENU
        enu_matrix * delta_ecef
    }
    
    /// Convert ENU local coordinates to ECEF
    pub fn enu_to_ecef(&self, enu: &Vector3<f64>, reference_ecef: &Vector3<f64>, enu_matrix: &Matrix3<f64>) -> Vector3<f64> {
        // Transform from ENU to ECEF
        let delta_ecef = enu_matrix.transpose() * enu;
        
        // Add reference ECEF
        reference_ecef + delta_ecef
    }
    
    /// Get or compute the ENU transformation matrix for a reference position
    fn get_enu_matrix(&mut self, reference: &Position) -> Matrix3<f64> {
        // Check if we can use cached matrix
        if let Some(ref cached_ref) = self.reference_position {
            if (cached_ref.lat - reference.lat).abs() < 1e-10 &&
               (cached_ref.lon - reference.lon).abs() < 1e-10 &&
               (cached_ref.depth - reference.depth).abs() < 1e-10 {
                if let Some(matrix) = self.cached_enu_matrix {
                    return matrix;
                }
            }
        }
        
        // Need to compute new matrix
        let lat_rad = reference.lat.to_radians();
        let lon_rad = reference.lon.to_radians();
        
        // Rotation matrix from ECEF to ENU
        let sin_lat = lat_rad.sin();
        let cos_lat = lat_rad.cos();
        let sin_lon = lon_rad.sin();
        let cos_lon = lon_rad.cos();
        
        let enu_matrix = Matrix3::new(
            -sin_lon,           cos_lon,          0.0,
            -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat,
            cos_lat * cos_lon,  cos_lat * sin_lon,  sin_lat
        );
        
        // Cache the results
        self.reference_position = Some(reference.clone());
        self.cached_enu_matrix = Some(enu_matrix);
        self.cached_reference_ecef = Some(self.geodetic_to_ecef(reference));
        
        enu_matrix
    }
    
    /// Convert geodetic coordinates to local ENU coordinates
    pub fn geodetic_to_enu(&mut self, position: &Position, reference: &Position) -> Vector3<f64> {
        // Get or compute the ENU matrix
        let enu_matrix = self.get_enu_matrix(reference);
        
        // Convert both positions to ECEF
        let ecef = self.geodetic_to_ecef(position);
        let reference_ecef = self.cached_reference_ecef.unwrap_or_else(|| self.geodetic_to_ecef(reference));
        
        // Convert ECEF to ENU
        self.ecef_to_enu(&ecef, &reference_ecef, &enu_matrix)
    }
    
    /// Convert local ENU coordinates to geodetic coordinates
    pub fn enu_to_geodetic(&mut self, enu: &Vector3<f64>, reference: &Position) -> Position {
        // Get or compute the ENU matrix
        let enu_matrix = self.get_enu_matrix(reference);
        
        // Get reference ECEF
        let reference_ecef = self.cached_reference_ecef.unwrap_or_else(|| self.geodetic_to_ecef(reference));
        
        // Convert ENU to ECEF
        let ecef = self.enu_to_ecef(enu, &reference_ecef, &enu_matrix);
        
        // Convert ECEF to geodetic
        self.ecef_to_geodetic(&ecef)
    }
    
    /// Calculate sound speed based on environmental parameters using the UNESCO formula
    pub fn calculate_sound_speed(&self, env: &EnvironmentalCorrections) -> f64 {
        let t = env.temperature;
        let s = env.salinity;
        let p = env.pressure;
        
        // UNESCO equation coefficients
        let c00 = 1402.388;
        let c01 = 5.03830;
        let c02 = -5.81090e-2;
        let c03 = 3.3432e-4;
        let c04 = -1.47797e-6;
        let c05 = 3.1419e-9;
        let c10 = 0.153563;
        let c11 = 6.8999e-4;
        let c12 = -8.1829e-6;
        let c13 = 1.3632e-7;
        let c14 = -6.1260e-10;
        let c20 = 3.1260e-5;
        let c21 = -1.7111e-6;
        let c22 = 2.5986e-8;
        let c23 = -2.5353e-10;
        let c24 = 1.0415e-12;
        let c30 = -9.7729e-9;
        let c31 = 3.8513e-10;
        let c32 = -2.3654e-12;
        
        // Calculate sound speed
        let cw = c00 + c01*t + c02*t.powi(2) + c03*t.powi(3) + c04*t.powi(4) + c05*t.powi(5) +
                (c10 + c11*t + c12*t.powi(2) + c13*t.powi(3) + c14*t.powi(4)) * p +
                (c20 + c21*t + c22*t.powi(2) + c23*t.powi(3) + c24*t.powi(4)) * p.powi(2) +
                (c30 + c31*t + c32*t.powi(2)) * p.powi(3);
        
        // Add salinity correction
        let a = 1.389 - 1.262e-2*t + 7.166e-5*t.powi(2) - 1.262e-6*t.powi(3);
        let b = 9.4742e-5 - 1.2583e-5*t + 6.4928e-8*t.powi(2);
        let c = 1.340e-6;
        
        cw + a*s + b*s.powi(3/2) + c*(s.powi(2))
    }
    
    /// Calculate position uncertainty from range measurements and anchor positions
    pub fn calculate_position_uncertainty(
        &self,
        position: &Vector3<f64>,
        anchor_positions: &[Vector3<f64>],
        range_std_devs: &[f64],
    ) -> PositionUncertainty {
        let n = anchor_positions.len();
        if n < 3 || range_std_devs.len() != n {
            // Not enough data or mismatched arrays
            return PositionUncertainty {
                covariance: Matrix3::identity(),
                std_dev_x: 1.0,
                std_dev_y: 1.0,
                std_dev_z: 1.0,
                confidence_radius_95: 1.0,
            };
        }
        
        // Construct Jacobian matrix
        let mut jacobian = Matrix3::zeros();
        
        for i in 0..n.min(3) {
            let delta = position - anchor_positions[i];
            let range = delta.norm();
            
            if range > 1e-6 {
                // Unit vector from anchor to position
                let unit_vector = delta / range;
                
                // Add to Jacobian (weighted by measurement variance)
                let weight = 1.0 / range_std_devs[i].powi(2);
                jacobian[(0, 0)] += unit_vector.x * unit_vector.x * weight;
                jacobian[(0, 1)] += unit_vector.x * unit_vector.y * weight;
                jacobian[(0, 2)] += unit_vector.x * unit_vector.z * weight;
                jacobian[(1, 0)] += unit_vector.y * unit_vector.x * weight;
                jacobian[(1, 1)] += unit_vector.y * unit_vector.y * weight;
                jacobian[(1, 2)] += unit_vector.y * unit_vector.z * weight;
                jacobian[(2, 0)] += unit_vector.z * unit_vector.x * weight;
                jacobian[(2, 1)] += unit_vector.z * unit_vector.y * weight;
                jacobian[(2, 2)] += unit_vector.z * unit_vector.z * weight;
            }
        }
        
        // Invert to get covariance matrix
        let covariance = match jacobian.try_inverse() {
            Some(inv) => inv,
            None => Matrix3::identity(), // Fallback if matrix is singular
        };
        
        // Extract standard deviations
        let std_dev_x = covariance[(0, 0)].sqrt();
        let std_dev_y = covariance[(1, 1)].sqrt();
        let std_dev_z = covariance[(2, 2)].sqrt();
        
        // Calculate 95% confidence radius (assuming normal distribution)
        let confidence_radius_95 = 1.96 * (std_dev_x.powi(2) + std_dev_y.powi(2) + std_dev_z.powi(2)).sqrt();
        
        PositionUncertainty {
            covariance,
            std_dev_x,
            std_dev_y,
            std_dev_z,
            confidence_radius_95,
        }
    }
    
    /// Apply environmental corrections to sound speed
    pub fn apply_sound_speed_correction(&self, base_speed: f64, env: &EnvironmentalCorrections) -> f64 {
        // Calculate corrected sound speed
        self.calculate_sound_speed(env)
    }
    
    /// Calculate range correction due to sound speed variations
    pub fn calculate_range_correction(
        &self,
        range: f64,
        nominal_sound_speed: f64,
        actual_sound_speed: f64
    ) -> f64 {
        // Correct range based on sound speed ratio
        range * (actual_sound_speed / nominal_sound_speed)
    }
    
    /// Propagate error from range measurements to position estimate
    pub fn propagate_range_error_to_position(
        &self,
        position: &Vector3<f64>,
        anchor_positions: &[Vector3<f64>],
        range_errors: &[f64],
    ) -> Vector3<f64> {
        let n = anchor_positions.len();
        if n < 3 || range_errors.len() != n {
            return Vector3::zeros();
        }
        
        // Calculate unit vectors from anchors to position
        let mut unit_vectors = Vec::with_capacity(n);
        for anchor_pos in anchor_positions {
            let delta = position - anchor_pos;
            let range = delta.norm();
            
            if range > 1e-6 {
                unit_vectors.push(delta / range);
            } else {
                unit_vectors.push(Vector3::zeros());
            }
        }
        
        // Calculate position error as weighted sum of unit vectors
        let mut position_error = Vector3::zeros();
        for i in 0..n {
            position_error += unit_vectors[i] * range_errors[i];
        }
        
        position_error
    }
    
    /// Calculate the Geometric Dilution of Precision (GDOP) for a given configuration
    pub fn calculate_gdop(
        &self,
        position: &Vector3<f64>,
        anchor_positions: &[Vector3<f64>],
    ) -> f64 {
        let n = anchor_positions.len();
        if n < 4 {
            return f64::INFINITY; // Need at least 4 anchors for 3D positioning
        }
        
        // Construct geometry matrix
        let mut h = nalgebra::DMatrix::zeros(n, 4);
        
        for i in 0..n {
            let delta = position - anchor_positions[i];
            let range = delta.norm();
            
            if range > 1e-6 {
                // Unit vector components
                h[(i, 0)] = (position.x - anchor_positions[i].x) / range;
                h[(i, 1)] = (position.y - anchor_positions[i].y) / range;
                h[(i, 2)] = (position.z - anchor_positions[i].z) / range;
                h[(i, 3)] = 1.0; // For clock bias
            }
        }
        
        // Calculate (H^T * H)^-1
        let ht_h = h.transpose() * &h;
        
        // Calculate GDOP as sqrt(trace((H^T * H)^-1))
        match ht_h.try_inverse() {
            Some(inv) => (inv.trace()).sqrt(),
            None => f64::INFINITY, // Singular matrix
        }
    }
    
    /// Apply tidal corrections to depth measurements
    pub fn apply_tidal_correction(&self, depth: f64, tidal_height: f64) -> f64 {
        // Adjust depth based on tidal height
        depth - tidal_height
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_geodetic_to_ecef_conversion() {
        let transformer = HighPrecisionTransformer::new();
        
        // Test with known position
        let position = Position {
            lat: 0.0,
            lon: 0.0,
            depth: 0.0,
        };
        
        let ecef = transformer.geodetic_to_ecef(&position);
        
        // At origin, ECEF x should be approximately Earth radius
        assert!((ecef.x - transformer.earth_radius_a).abs() < 1.0);
        assert!(ecef.y.abs() < 1.0);
        assert!(ecef.z.abs() < 1.0);
    }
    
    #[test]
    fn test_ecef_to_geodetic_conversion() {
        let transformer = HighPrecisionTransformer::new();
        
        // Test with known ECEF coordinates
        let ecef = Vector3::new(transformer.earth_radius_a, 0.0, 0.0);
        
        let position = transformer.ecef_to_geodetic(&ecef);
        
        // Should be at equator, prime meridian
        assert!(position.lat.abs() < 1e-6);
        assert!(position.lon.abs() < 1e-6);
        assert!(position.depth.abs() < 1.0);
    }
    
    #[test]
    fn test_geodetic_to_enu_conversion() {
        let mut transformer = HighPrecisionTransformer::new();
        
        // Reference position
        let reference = Position {
            lat: 0.0,
            lon: 0.0,
            depth: 0.0,
        };
        
        // Test position 1km north
        let position = Position {
            lat: 0.009, // ~1km north
            lon: 0.0,
            depth: 0.0,
        };
        
        let enu = transformer.geodetic_to_enu(&position, &reference);
        
        // Should be approximately 1000m north
        assert!(enu.x.abs() < 1.0); // East
        assert!((enu.y - 1000.0).abs() < 10.0); // North
        assert!(enu.z.abs() < 1.0); // Up
    }
    
    #[test]
    fn test_enu_to_geodetic_conversion() {
        let mut transformer = HighPrecisionTransformer::new();
        
        // Reference position
        let reference = Position {
            lat: 0.0,
            lon: 0.0,
            depth: 0.0,
        };
        
        // Test position 1km east
        let enu = Vector3::new(1000.0, 0.0, 0.0);
        
        let position = transformer.enu_to_geodetic(&enu, &reference);
        
        // Should be approximately 0.009 degrees east
        assert!(position.lat.abs() < 1e-6);
        assert!((position.lon - 0.009).abs() < 1e-3);
        assert!(position.depth.abs() < 1.0);
    }
    
    #[test]
    fn test_sound_speed_calculation() {
        let transformer = HighPrecisionTransformer::new();
        
        // Test with standard conditions
        let env = EnvironmentalCorrections {
            base_sound_speed: 1500.0,
            temperature: 15.0,
            salinity: 35.0,
            depth: 0.0,
            pressure: 10.0,
        };
        
        let speed = transformer.calculate_sound_speed(&env);
        
        // Sound speed should be reasonable
        assert!(speed > 1450.0);
        assert!(speed < 1550.0);
    }
    
    #[test]
    fn test_position_uncertainty_calculation() {
        let transformer = HighPrecisionTransformer::new();
        
        // Test position
        let position = Vector3::new(0.0, 0.0, 0.0);
        
        // Test with 4 anchors in good geometry
        let anchor_positions = vec![
            Vector3::new(100.0, 0.0, 0.0),
            Vector3::new(0.0, 100.0, 0.0),
            Vector3::new(-100.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 100.0),
        ];
        
        // 1m standard deviation for all ranges
        let range_std_devs = vec![1.0, 1.0, 1.0, 1.0];
        
        let uncertainty = transformer.calculate_position_uncertainty(
            &position,
            &anchor_positions,
            &range_std_devs,
        );
        
        // Uncertainty should be reasonable
        assert!(uncertainty.std_dev_x > 0.0);
        assert!(uncertainty.std_dev_x < 2.0);
        assert!(uncertainty.std_dev_y > 0.0);
        assert!(uncertainty.std_dev_y < 2.0);
        assert!(uncertainty.std_dev_z > 0.0);
        assert!(uncertainty.std_dev_z < 2.0);
        assert!(uncertainty.confidence_radius_95 > 0.0);
        assert!(uncertainty.confidence_radius_95 < 5.0);
    }
    
    #[test]
    fn test_gdop_calculation() {
        let transformer = HighPrecisionTransformer::new();
        
        // Test position
        let position = Vector3::new(0.0, 0.0, 0.0);
        
        // Test with 4 anchors in good geometry
        let good_geometry = vec![
            Vector3::new(100.0, 0.0, 0.0),
            Vector3::new(0.0, 100.0, 0.0),
            Vector3::new(-100.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 100.0),
        ];
        
        // Test with 4 anchors in poor geometry (nearly collinear)
        let poor_geometry = vec![
            Vector3::new(100.0, 1.0, 1.0),
            Vector3::new(200.0, 2.0, 2.0),
            Vector3::new(300.0, 3.0, 3.0),
            Vector3::new(400.0, 4.0, 4.0),
        ];
        
        let good_gdop = transformer.calculate_gdop(&position, &good_geometry);
        let poor_gdop = transformer.calculate_gdop(&position, &poor_geometry);
        
        // Good geometry should have lower GDOP
        assert!(good_gdop < poor_gdop);
        assert!(good_gdop < 5.0);
        assert!(poor_gdop > 10.0);
    }
}