//! Embedded-optimized coordinate system management for microcontroller deployment
//! 
//! This module provides memory-efficient coordinate system transformations
//! specifically designed for resource-constrained embedded systems. Features:
//! - Reference point caching for computational efficiency
//! - Precision-optimized transformations for local operations
//! - WGS84 geodetic coordinate support
//! - Local grid system transformations
//! - Earth curvature corrections for large operational areas
//! - Stack-based operations with minimal memory footprint

use crate::core::Position;

/// Maximum number of cached reference points for embedded systems
pub const MAX_CACHED_REFERENCES: usize = 4;

/// Earth radius in meters (WGS84)
pub const EARTH_RADIUS_WGS84: f64 = 6378137.0;

/// Earth flattening factor (WGS84)
pub const EARTH_FLATTENING_WGS84: f64 = 1.0 / 298.257223563;

/// Eccentricity squared (WGS84)
pub const ECCENTRICITY_SQUARED_WGS84: f64 = 2.0 * EARTH_FLATTENING_WGS84 - EARTH_FLATTENING_WGS84 * EARTH_FLATTENING_WGS84;

/// Fixed-point scaling for coordinate calculations
pub const COORD_FIXED_POINT_SCALE: i64 = 1_000_000; // 6 decimal places precision

/// Coordinate system types supported
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CoordinateSystem {
    /// WGS84 geodetic coordinates (latitude, longitude, altitude)
    WGS84Geodetic,
    /// Local tangent plane (East-North-Up)
    LocalTangentPlane,
    /// Universal Transverse Mercator
    UTM,
    /// Local grid system (custom projection)
    LocalGrid,
}

/// Reference point for coordinate transformations
#[derive(Debug, Clone)]
pub struct ReferencePoint {
    /// Geodetic position of reference point
    pub geodetic: Position,
    /// UTM zone (if applicable)
    pub utm_zone: Option<u8>,
    /// UTM hemisphere (true = North, false = South)
    pub utm_north: bool,
    /// Local grid parameters
    pub grid_params: LocalGridParams,
    /// Timestamp when this reference was cached
    pub timestamp_ms: u64,
    /// Usage count for cache management
    pub usage_count: u32,
}

/// Local grid system parameters
#[derive(Debug, Clone, Copy)]
pub struct LocalGridParams {
    /// Scale factor for local grid
    pub scale_factor: f64,
    /// False easting (meters)
    pub false_easting: f64,
    /// False northing (meters)
    pub false_northing: f64,
    /// Central meridian (degrees)
    pub central_meridian: f64,
    /// Standard parallel (degrees)
    pub standard_parallel: f64,
}

impl Default for LocalGridParams {
    fn default() -> Self {
        Self {
            scale_factor: 1.0,
            false_easting: 0.0,
            false_northing: 0.0,
            central_meridian: 0.0,
            standard_parallel: 0.0,
        }
    }
}

/// Fixed-point coordinate for embedded systems
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FixedCoordinate {
    pub x: i64, // Scaled by COORD_FIXED_POINT_SCALE
    pub y: i64, // Scaled by COORD_FIXED_POINT_SCALE
    pub z: i64, // Scaled by COORD_FIXED_POINT_SCALE
}

impl FixedCoordinate {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            x: (x * COORD_FIXED_POINT_SCALE as f64) as i64,
            y: (y * COORD_FIXED_POINT_SCALE as f64) as i64,
            z: (z * COORD_FIXED_POINT_SCALE as f64) as i64,
        }
    }

    pub fn to_f64(&self) -> (f64, f64, f64) {
        (
            self.x as f64 / COORD_FIXED_POINT_SCALE as f64,
            self.y as f64 / COORD_FIXED_POINT_SCALE as f64,
            self.z as f64 / COORD_FIXED_POINT_SCALE as f64,
        )
    }

    pub fn distance_to(&self, other: &Self) -> i64 {
        // Convert to floating point for distance calculation to avoid overflow issues
        let (x1, y1, z1) = self.to_f64();
        let (x2, y2, z2) = other.to_f64();
        
        let dx = x1 - x2;
        let dy = y1 - y2;
        let dz = z1 - z2;
        
        let distance = (dx * dx + dy * dy + dz * dz).sqrt();
        (distance * COORD_FIXED_POINT_SCALE as f64) as i64
    }

    /// Fixed-point square root using Newton's method
    fn fixed_point_sqrt(&self, value: i64) -> i64 {
        if value <= 0 {
            return 0;
        }
        
        let mut x = value / 2;
        for _ in 0..10 { // Limited iterations for embedded systems
            if x == 0 {
                break;
            }
            let new_x = (x + value / x) / 2;
            if (new_x - x).abs() < 100 { // Smaller tolerance for scaled values
                break;
            }
            x = new_x;
        }
        x
    }
}

/// Embedded coordinate system manager
pub struct EmbeddedCoordinateManager {
    /// Cached reference points for efficiency
    reference_cache: [Option<ReferencePoint>; MAX_CACHED_REFERENCES],
    /// Current active reference point index
    active_reference: Option<usize>,
    /// Use fixed-point arithmetic
    use_fixed_point: bool,
    /// Enable Earth curvature corrections
    enable_curvature_correction: bool,
    /// Maximum operational area radius (meters)
    max_operational_radius: f64,
}

impl Default for EmbeddedCoordinateManager {
    fn default() -> Self {
        Self {
            reference_cache: [const { None }; MAX_CACHED_REFERENCES],
            active_reference: None,
            use_fixed_point: false,
            enable_curvature_correction: true,
            max_operational_radius: 10000.0, // 10km default
        }
    }
}

impl EmbeddedCoordinateManager {
    pub fn new() -> Self {
        Self::default()
    }

    /// Configure for fixed-point arithmetic
    pub fn with_fixed_point() -> Self {
        Self {
            use_fixed_point: true,
            ..Default::default()
        }
    }

    /// Configure for large operational areas
    pub fn with_large_area(max_radius: f64) -> Self {
        Self {
            enable_curvature_correction: true,
            max_operational_radius: max_radius,
            ..Default::default()
        }
    }

    /// Set reference point for coordinate transformations
    pub fn set_reference_point(&mut self, reference: Position, timestamp_ms: u64) -> Result<(), &'static str> {
        // Find an empty slot or replace least recently used
        let slot_index = self.find_cache_slot();
        
        let reference_point = ReferencePoint {
            utm_zone: Some(self.calculate_utm_zone(reference.lon)),
            utm_north: reference.lat >= 0.0,
            geodetic: reference,
            grid_params: LocalGridParams::default(),
            timestamp_ms,
            usage_count: 1,
        };

        self.reference_cache[slot_index] = Some(reference_point);
        self.active_reference = Some(slot_index);
        
        Ok(())
    }

    /// Get current reference point
    pub fn get_reference_point(&self) -> Option<&ReferencePoint> {
        self.active_reference.and_then(|idx| self.reference_cache[idx].as_ref())
    }

    /// Convert geodetic to local tangent plane coordinates
    pub fn geodetic_to_local(&mut self, position: &Position) -> Result<(f64, f64, f64), &'static str> {
        let use_fixed_point = self.use_fixed_point;
        let reference = self.get_active_reference()?.clone();
        
        if use_fixed_point {
            let fixed_result = self.geodetic_to_local_fixed(position, &reference)?;
            Ok(fixed_result.to_f64())
        } else {
            self.geodetic_to_local_floating(position, &reference)
        }
    }

    /// Convert local tangent plane to geodetic coordinates
    pub fn local_to_geodetic(&mut self, local: (f64, f64, f64)) -> Result<Position, &'static str> {
        let use_fixed_point = self.use_fixed_point;
        let reference = self.get_active_reference()?.clone();
        
        if use_fixed_point {
            let fixed_local = FixedCoordinate::new(local.0, local.1, local.2);
            self.local_to_geodetic_fixed(&fixed_local, &reference)
        } else {
            self.local_to_geodetic_floating(local, &reference)
        }
    }

    /// Convert geodetic to UTM coordinates
    pub fn geodetic_to_utm(&mut self, position: &Position) -> Result<(f64, f64, f64, u8, bool), &'static str> {
        let zone = self.calculate_utm_zone(position.lon);
        let is_north = position.lat >= 0.0;
        
        let (easting, northing) = self.geodetic_to_utm_projection(position, zone)?;
        
        Ok((easting, northing, -position.depth, zone, is_north))
    }

    /// Convert UTM to geodetic coordinates
    pub fn utm_to_geodetic(&self, easting: f64, northing: f64, depth: f64, zone: u8, is_north: bool) -> Result<Position, &'static str> {
        let (lat, lon) = self.utm_to_geodetic_projection(easting, northing, zone, is_north)?;
        
        Ok(Position {
            lat,
            lon,
            depth: -depth, // Convert altitude back to depth
        })
    }

    /// Validate coordinate system transformation
    pub fn validate_transformation(&self, from: CoordinateSystem, to: CoordinateSystem) -> Result<(), &'static str> {
        match (from, to) {
            (CoordinateSystem::WGS84Geodetic, CoordinateSystem::LocalTangentPlane) => {
                if self.active_reference.is_none() {
                    return Err("No reference point set for local tangent plane transformation");
                }
            },
            (CoordinateSystem::WGS84Geodetic, CoordinateSystem::UTM) => {
                // UTM is always valid for WGS84
            },
            (CoordinateSystem::LocalTangentPlane, CoordinateSystem::WGS84Geodetic) => {
                if self.active_reference.is_none() {
                    return Err("No reference point set for geodetic transformation");
                }
            },
            _ => return Err("Unsupported coordinate transformation"),
        }
        
        Ok(())
    }

    /// Apply Earth curvature correction for large areas
    pub fn apply_curvature_correction(&self, local: (f64, f64, f64), reference: &ReferencePoint) -> (f64, f64, f64) {
        if !self.enable_curvature_correction {
            return local;
        }

        let distance = (local.0 * local.0 + local.1 * local.1).sqrt();
        
        // Only apply correction for distances > 1km
        if distance < 1000.0 {
            return local;
        }

        let lat_rad = reference.geodetic.lat.to_radians();
        
        // Earth curvature correction
        let curvature_correction = distance * distance / (2.0 * EARTH_RADIUS_WGS84);
        
        // Apply correction to vertical component
        let corrected_z = local.2 - curvature_correction * lat_rad.cos();
        
        (local.0, local.1, corrected_z)
    }

    /// Get transformation precision estimate
    pub fn get_transformation_precision(&self, distance_m: f64) -> f64 {
        if self.use_fixed_point {
            // Fixed-point precision
            let base_precision = 1.0 / COORD_FIXED_POINT_SCALE as f64;
            
            // Precision degrades with distance
            base_precision * (1.0 + distance_m / 1000.0)
        } else {
            // Floating-point precision
            let base_precision = 1e-6; // ~1mm
            
            // Account for Earth curvature effects
            if self.enable_curvature_correction && distance_m > 1000.0 {
                base_precision * (1.0 + distance_m / 10000.0)
            } else {
                base_precision
            }
        }
    }

    /// Private helper methods

    fn get_active_reference(&mut self) -> Result<&ReferencePoint, &'static str> {
        match self.active_reference {
            Some(idx) => {
                // Update usage count
                if let Some(ref mut reference) = self.reference_cache[idx] {
                    reference.usage_count += 1;
                    Ok(reference)
                } else {
                    Err("Invalid reference point index")
                }
            },
            None => Err("No active reference point set"),
        }
    }

    fn find_cache_slot(&self) -> usize {
        // Find empty slot first
        for (i, slot) in self.reference_cache.iter().enumerate() {
            if slot.is_none() {
                return i;
            }
        }

        // Find least recently used slot
        let mut lru_index = 0;
        let mut min_usage = u32::MAX;
        
        for (i, slot) in self.reference_cache.iter().enumerate() {
            if let Some(ref point) = slot {
                if point.usage_count < min_usage {
                    min_usage = point.usage_count;
                    lru_index = i;
                }
            }
        }

        lru_index
    }

    fn calculate_utm_zone(&self, longitude: f64) -> u8 {
        ((longitude + 180.0) / 6.0).floor() as u8 + 1
    }

    fn geodetic_to_local_floating(&self, position: &Position, reference: &ReferencePoint) -> Result<(f64, f64, f64), &'static str> {
        let lat_diff = (position.lat - reference.geodetic.lat).to_radians();
        let lon_diff = (position.lon - reference.geodetic.lon).to_radians();
        let ref_lat_rad = reference.geodetic.lat.to_radians();
        
        // Calculate radius of curvature
        let n = EARTH_RADIUS_WGS84 / (1.0 - ECCENTRICITY_SQUARED_WGS84 * ref_lat_rad.sin().powi(2)).sqrt();
        let m = EARTH_RADIUS_WGS84 * (1.0 - ECCENTRICITY_SQUARED_WGS84) / (1.0 - ECCENTRICITY_SQUARED_WGS84 * ref_lat_rad.sin().powi(2)).powf(1.5);
        
        // Local tangent plane coordinates
        let x = n * ref_lat_rad.cos() * lon_diff;
        let y = m * lat_diff;
        let z = position.depth - reference.geodetic.depth;
        
        // Apply curvature correction if enabled
        let corrected = self.apply_curvature_correction((x, y, z), reference);
        
        Ok(corrected)
    }

    fn geodetic_to_local_fixed(&self, position: &Position, reference: &ReferencePoint) -> Result<FixedCoordinate, &'static str> {
        let (x, y, z) = self.geodetic_to_local_floating(position, reference)?;
        Ok(FixedCoordinate::new(x, y, z))
    }

    fn local_to_geodetic_floating(&self, local: (f64, f64, f64), reference: &ReferencePoint) -> Result<Position, &'static str> {
        let ref_lat_rad = reference.geodetic.lat.to_radians();
        
        // Calculate radius of curvature
        let n = EARTH_RADIUS_WGS84 / (1.0 - ECCENTRICITY_SQUARED_WGS84 * ref_lat_rad.sin().powi(2)).sqrt();
        let m = EARTH_RADIUS_WGS84 * (1.0 - ECCENTRICITY_SQUARED_WGS84) / (1.0 - ECCENTRICITY_SQUARED_WGS84 * ref_lat_rad.sin().powi(2)).powf(1.5);
        
        // Convert back to geodetic
        let lat_diff = local.1 / m;
        let lon_diff = local.0 / (n * ref_lat_rad.cos());
        
        Ok(Position {
            lat: reference.geodetic.lat + lat_diff.to_degrees(),
            lon: reference.geodetic.lon + lon_diff.to_degrees(),
            depth: reference.geodetic.depth + local.2,
        })
    }

    fn local_to_geodetic_fixed(&self, local: &FixedCoordinate, reference: &ReferencePoint) -> Result<Position, &'static str> {
        let (x, y, z) = local.to_f64();
        self.local_to_geodetic_floating((x, y, z), reference)
    }

    fn geodetic_to_utm_projection(&self, position: &Position, zone: u8) -> Result<(f64, f64), &'static str> {
        let lat_rad = position.lat.to_radians();
        let lon_rad = position.lon.to_radians();
        
        // Central meridian for the zone
        let central_meridian = ((zone as f64 - 1.0) * 6.0 - 180.0 + 3.0).to_radians();
        let lon_diff = lon_rad - central_meridian;
        
        // UTM constants
        let k0 = 0.9996; // Scale factor
        let false_easting = 500000.0;
        let false_northing = if position.lat < 0.0 { 10000000.0 } else { 0.0 };
        
        // Calculate UTM coordinates using simplified formulas for embedded systems
        let n = EARTH_RADIUS_WGS84 / (1.0 - ECCENTRICITY_SQUARED_WGS84 * lat_rad.sin().powi(2)).sqrt();
        let t = lat_rad.tan();
        let c = ECCENTRICITY_SQUARED_WGS84 * lat_rad.cos().powi(2) / (1.0 - ECCENTRICITY_SQUARED_WGS84);
        let a = lat_rad.cos() * lon_diff;
        
        // Simplified UTM formulas (good for most underwater applications)
        let easting = false_easting + k0 * n * (a + (1.0 - t * t + c) * a.powi(3) / 6.0);
        
        let m = EARTH_RADIUS_WGS84 * (
            (1.0 - ECCENTRICITY_SQUARED_WGS84 / 4.0 - 3.0 * ECCENTRICITY_SQUARED_WGS84.powi(2) / 64.0) * lat_rad
            - (3.0 * ECCENTRICITY_SQUARED_WGS84 / 8.0 + 3.0 * ECCENTRICITY_SQUARED_WGS84.powi(2) / 32.0) * (2.0 * lat_rad).sin()
            + (15.0 * ECCENTRICITY_SQUARED_WGS84.powi(2) / 256.0) * (4.0 * lat_rad).sin()
        );
        
        let northing = false_northing + k0 * (m + n * t * (a.powi(2) / 2.0 + (5.0 - t * t + 9.0 * c + 4.0 * c.powi(2)) * a.powi(4) / 24.0));
        
        Ok((easting, northing))
    }

    fn utm_to_geodetic_projection(&self, easting: f64, northing: f64, zone: u8, is_north: bool) -> Result<(f64, f64), &'static str> {
        let k0 = 0.9996;
        let false_easting = 500000.0;
        let false_northing = if !is_north { 10000000.0 } else { 0.0 };
        
        let central_meridian = ((zone as f64 - 1.0) * 6.0 - 180.0 + 3.0).to_radians();
        
        let x = easting - false_easting;
        let y = northing - false_northing;
        
        // Simplified inverse UTM transformation
        let m = y / k0;
        let mu = m / (EARTH_RADIUS_WGS84 * (1.0 - ECCENTRICITY_SQUARED_WGS84 / 4.0));
        
        // Approximate latitude
        let lat_rad = mu + (3.0 * ECCENTRICITY_SQUARED_WGS84 / 8.0) * (2.0 * mu).sin();
        
        let n = EARTH_RADIUS_WGS84 / (1.0 - ECCENTRICITY_SQUARED_WGS84 * lat_rad.sin().powi(2)).sqrt();
        let t = lat_rad.tan();
        let c = ECCENTRICITY_SQUARED_WGS84 * lat_rad.cos().powi(2) / (1.0 - ECCENTRICITY_SQUARED_WGS84);
        let r = EARTH_RADIUS_WGS84 * (1.0 - ECCENTRICITY_SQUARED_WGS84) / (1.0 - ECCENTRICITY_SQUARED_WGS84 * lat_rad.sin().powi(2)).powf(1.5);
        let d = x / (n * k0);
        
        let lat = lat_rad - (n * t / r) * (d.powi(2) / 2.0 - (5.0 + 3.0 * t.powi(2) + 10.0 * c - 4.0 * c.powi(2)) * d.powi(4) / 24.0);
        let lon = central_meridian + (d - (1.0 + 2.0 * t.powi(2) + c) * d.powi(3) / 6.0) / lat_rad.cos();
        
        Ok((lat.to_degrees(), lon.to_degrees()))
    }
}

/// Coordinate system validation utilities
pub struct CoordinateValidator;

impl CoordinateValidator {
    /// Validate WGS84 geodetic coordinates
    pub fn validate_wgs84(position: &Position) -> Result<(), &'static str> {
        if position.lat < -90.0 || position.lat > 90.0 {
            return Err("Invalid latitude: must be between -90 and 90 degrees");
        }
        
        if position.lon < -180.0 || position.lon > 180.0 {
            return Err("Invalid longitude: must be between -180 and 180 degrees");
        }
        
        if position.depth < -11000.0 || position.depth > 9000.0 {
            return Err("Invalid depth: must be between -11000m and 9000m");
        }
        
        Ok(())
    }

    /// Validate UTM coordinates
    pub fn validate_utm(easting: f64, northing: f64, zone: u8) -> Result<(), &'static str> {
        if zone < 1 || zone > 60 {
            return Err("Invalid UTM zone: must be between 1 and 60");
        }
        
        if easting < 100000.0 || easting > 900000.0 {
            return Err("Invalid UTM easting: outside typical range");
        }
        
        if northing < 0.0 || northing > 10000000.0 {
            return Err("Invalid UTM northing: outside valid range");
        }
        
        Ok(())
    }

    /// Validate local coordinates against operational area
    pub fn validate_local(local: (f64, f64, f64), max_radius: f64) -> Result<(), &'static str> {
        let distance = (local.0 * local.0 + local.1 * local.1).sqrt();
        
        if distance > max_radius {
            return Err("Local coordinates outside operational area");
        }
        
        if local.2.abs() > 11000.0 {
            return Err("Invalid depth in local coordinates");
        }
        
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fixed_coordinate_operations() {
        let coord1 = FixedCoordinate::new(100.0, 200.0, 50.0);
        let coord2 = FixedCoordinate::new(300.0, 400.0, 100.0);
        
        let distance = coord1.distance_to(&coord2);
        let expected_distance = ((200.0 * 200.0 + 200.0 * 200.0 + 50.0 * 50.0) as f64).sqrt();
        
        let actual_distance = distance as f64 / COORD_FIXED_POINT_SCALE as f64;
        
        // Should be accurate to within 1 meter
        assert!((actual_distance - expected_distance).abs() < 1.0);
    }

    #[test]
    fn test_coordinate_manager_reference_point() {
        let mut manager = EmbeddedCoordinateManager::new();
        
        let reference = Position {
            lat: 37.7749,
            lon: -122.4194,
            depth: 0.0,
        };
        
        assert!(manager.set_reference_point(reference, 1000).is_ok());
        assert!(manager.get_reference_point().is_some());
    }

    #[test]
    fn test_geodetic_to_local_transformation() {
        let mut manager = EmbeddedCoordinateManager::new();
        
        let reference = Position {
            lat: 0.0,
            lon: 0.0,
            depth: 0.0,
        };
        
        manager.set_reference_point(reference, 1000).unwrap();
        
        let test_position = Position {
            lat: 0.001, // ~111 meters north
            lon: 0.001, // ~111 meters east
            depth: 10.0,
        };
        
        let local = manager.geodetic_to_local(&test_position).unwrap();
        
        // Should be approximately 111 meters in each direction
        assert!((local.0 - 111.0).abs() < 10.0);
        assert!((local.1 - 111.0).abs() < 10.0);
        assert!((local.2 - 10.0).abs() < 0.1);
    }

    #[test]
    fn test_utm_zone_calculation() {
        let manager = EmbeddedCoordinateManager::new();
        
        assert_eq!(manager.calculate_utm_zone(-122.4194), 10); // San Francisco
        assert_eq!(manager.calculate_utm_zone(0.0), 31); // Greenwich
        assert_eq!(manager.calculate_utm_zone(139.6917), 54); // Tokyo
    }

    #[test]
    fn test_coordinate_validation() {
        let valid_position = Position {
            lat: 37.7749,
            lon: -122.4194,
            depth: 100.0,
        };
        
        assert!(CoordinateValidator::validate_wgs84(&valid_position).is_ok());
        
        let invalid_position = Position {
            lat: 91.0, // Invalid latitude
            lon: -122.4194,
            depth: 100.0,
        };
        
        assert!(CoordinateValidator::validate_wgs84(&invalid_position).is_err());
    }

    #[test]
    fn test_fixed_point_coordinate_manager() {
        let mut manager = EmbeddedCoordinateManager::with_fixed_point();
        
        let reference = Position {
            lat: 0.0,
            lon: 0.0,
            depth: 0.0,
        };
        
        manager.set_reference_point(reference, 1000).unwrap();
        
        let test_position = Position {
            lat: 0.001,
            lon: 0.001,
            depth: 10.0,
        };
        
        let local = manager.geodetic_to_local(&test_position).unwrap();
        let back_to_geodetic = manager.local_to_geodetic(local).unwrap();
        
        // Round-trip conversion should be accurate
        assert!((back_to_geodetic.lat - test_position.lat).abs() < 1e-5);
        assert!((back_to_geodetic.lon - test_position.lon).abs() < 1e-5);
        assert!((back_to_geodetic.depth - test_position.depth).abs() < 0.01);
    }

    #[test]
    fn test_curvature_correction() {
        let manager = EmbeddedCoordinateManager::with_large_area(50000.0);
        
        let reference = ReferencePoint {
            geodetic: Position { lat: 0.0, lon: 0.0, depth: 0.0 },
            utm_zone: Some(31),
            utm_north: true,
            grid_params: LocalGridParams::default(),
            timestamp_ms: 1000,
            usage_count: 1,
        };
        
        // Test large distance (10km)
        let local = (10000.0, 0.0, 0.0);
        let corrected = manager.apply_curvature_correction(local, &reference);
        
        // Should apply curvature correction
        assert!(corrected.2 != local.2);
    }
}