use nalgebra::Vector3;
use std::collections::HashMap;
use std::f64::consts::PI;
use crate::Position;

/// Cached reference point for coordinate transformations
#[derive(Debug, Clone)]
pub struct ReferencePoint {
    pub position: Position,
    pub lat_rad: f64,
    pub cos_lat: f64,
    pub sin_lat: f64,
    pub meters_per_deg_lat: f64,
    pub meters_per_deg_lon: f64,
    pub creation_time: std::time::Instant,
}

impl ReferencePoint {
    pub fn new(position: Position) -> Self {
        let lat_rad = position.lat * PI / 180.0;
        let cos_lat = lat_rad.cos();
        let sin_lat = lat_rad.sin();
        
        // WGS84 ellipsoid parameters
        const A: f64 = 6378137.0; // Semi-major axis in meters
        const E2: f64 = 0.00669437999014; // First eccentricity squared
        
        // More accurate meters per degree calculations using WGS84
        let n = A / (1.0 - E2 * sin_lat * sin_lat).sqrt();
        let meters_per_deg_lat = PI * A * (1.0 - E2) / (180.0 * (1.0 - E2 * sin_lat * sin_lat).powf(1.5));
        let meters_per_deg_lon = PI * n * cos_lat / 180.0;
        
        Self {
            position,
            lat_rad,
            cos_lat,
            sin_lat,
            meters_per_deg_lat,
            meters_per_deg_lon,
            creation_time: std::time::Instant::now(),
        }
    }
    
    /// Check if this reference point is still valid (within reasonable distance and time)
    pub fn is_valid_for(&self, position: &Position, max_distance_m: f64, max_age_secs: u64) -> bool {
        // Check age
        if self.creation_time.elapsed().as_secs() > max_age_secs {
            return false;
        }
        
        // Check distance (rough approximation)
        let dlat = (position.lat - self.position.lat).abs();
        let dlon = (position.lon - self.position.lon).abs();
        let dist_lat = dlat * self.meters_per_deg_lat;
        let dist_lon = dlon * self.meters_per_deg_lon;
        let approx_distance = (dist_lat * dist_lat + dist_lon * dist_lon).sqrt();
        
        approx_distance <= max_distance_m
    }
}

/// Optimized coordinate system manager with caching and performance optimizations
#[derive(Debug)]
pub struct CoordinateSystemManager {
    reference_cache: HashMap<String, ReferencePoint>,
    cache_max_age_secs: u64,
    cache_max_distance_m: f64,
    transformation_count: u64,
    cache_hits: u64,
}

impl CoordinateSystemManager {
    pub fn new() -> Self {
        Self {
            reference_cache: HashMap::new(),
            cache_max_age_secs: 300, // 5 minutes
            cache_max_distance_m: 1000.0, // 1 km
            transformation_count: 0,
            cache_hits: 0,
        }
    }
    
    /// Get or create a cached reference point for the given position
    fn get_or_create_reference(&mut self, position: &Position) -> ReferencePoint {
        let key = format!("{:.6}_{:.6}", position.lat, position.lon);
        
        // Check if we have a valid cached reference point
        if let Some(ref_point) = self.reference_cache.get(&key) {
            if ref_point.is_valid_for(position, self.cache_max_distance_m, self.cache_max_age_secs) {
                self.cache_hits += 1;
                return ref_point.clone();
            }
        }
        
        // Create new reference point
        let ref_point = ReferencePoint::new(position.clone());
        self.reference_cache.insert(key, ref_point.clone());
        ref_point
    }
    
    /// Optimized geodetic to local tangent plane conversion with caching
    pub fn geodetic_to_local_optimized(&mut self, pos: &Position, reference: &Position) -> Vector3<f64> {
        self.transformation_count += 1;
        
        let ref_point = self.get_or_create_reference(reference);
        
        let dlat = pos.lat - reference.lat;
        let dlon = pos.lon - reference.lon;
        
        let north = dlat * ref_point.meters_per_deg_lat;
        let east = dlon * ref_point.meters_per_deg_lon;
        let down = pos.depth;
        
        Vector3::new(east, north, down)
    }
    
    /// Optimized local to geodetic conversion with caching
    pub fn local_to_geodetic_optimized(&mut self, local_pos: &Vector3<f64>, reference: &Position) -> Position {
        self.transformation_count += 1;
        
        let ref_point = self.get_or_create_reference(reference);
        
        let east = local_pos.x;
        let north = local_pos.y;
        let down = local_pos.z;
        
        let lat_est = reference.lat + north / ref_point.meters_per_deg_lat;
        let lon_est = reference.lon + east / ref_point.meters_per_deg_lon;
        let depth_est = down;
        
        Position {
            lat: lat_est,
            lon: lon_est,
            depth: depth_est,
        }
    }
    
    /// Batch conversion for multiple positions (more efficient for repeated operations)
    pub fn geodetic_to_local_batch(&mut self, positions: &[Position], reference: &Position) -> Vec<Vector3<f64>> {
        let ref_point = self.get_or_create_reference(reference);
        let mut results = Vec::with_capacity(positions.len());
        
        for pos in positions {
            self.transformation_count += 1;
            
            let dlat = pos.lat - reference.lat;
            let dlon = pos.lon - reference.lon;
            
            let north = dlat * ref_point.meters_per_deg_lat;
            let east = dlon * ref_point.meters_per_deg_lon;
            let down = pos.depth;
            
            results.push(Vector3::new(east, north, down));
        }
        
        results
    }
    
    /// High-precision transformation for local operations (sub-meter accuracy)
    pub fn geodetic_to_local_high_precision(&mut self, pos: &Position, reference: &Position) -> Vector3<f64> {
        self.transformation_count += 1;
        
        // Use more precise calculations for high-accuracy requirements
        let lat1_rad = reference.lat * PI / 180.0;
        let lat2_rad = pos.lat * PI / 180.0;
        let lon1_rad = reference.lon * PI / 180.0;
        let lon2_rad = pos.lon * PI / 180.0;
        
        // WGS84 ellipsoid parameters
        const A: f64 = 6378137.0; // Semi-major axis
        const F: f64 = 1.0 / 298.257223563; // Flattening
        const E2: f64 = 2.0 * F - F * F; // First eccentricity squared
        
        // Calculate precise meridional radius of curvature
        let sin_lat1 = lat1_rad.sin();
        let cos_lat1 = lat1_rad.cos();
        let m1 = A * (1.0 - E2) / (1.0 - E2 * sin_lat1 * sin_lat1).powf(1.5);
        
        // Calculate precise prime vertical radius of curvature
        let n1 = A / (1.0 - E2 * sin_lat1 * sin_lat1).sqrt();
        
        // High-precision coordinate differences
        let dlat = lat2_rad - lat1_rad;
        let dlon = lon2_rad - lon1_rad;
        
        // Convert to local coordinates with high precision
        let north = dlat * m1;
        let east = dlon * n1 * cos_lat1;
        let down = pos.depth;
        
        Vector3::new(east, north, down)
    }
    
    /// Get performance statistics
    pub fn get_performance_stats(&self) -> CoordinateSystemStats {
        CoordinateSystemStats {
            total_transformations: self.transformation_count,
            cache_hits: self.cache_hits,
            cache_hit_rate: if self.transformation_count > 0 {
                self.cache_hits as f64 / self.transformation_count as f64
            } else {
                0.0
            },
            cached_references: self.reference_cache.len(),
        }
    }
    
    /// Clear expired cache entries
    pub fn cleanup_cache(&mut self) {
        let now = std::time::Instant::now();
        self.reference_cache.retain(|_, ref_point| {
            now.duration_since(ref_point.creation_time).as_secs() <= self.cache_max_age_secs
        });
    }
    
    /// Configure cache parameters
    pub fn configure_cache(&mut self, max_age_secs: u64, max_distance_m: f64) {
        self.cache_max_age_secs = max_age_secs;
        self.cache_max_distance_m = max_distance_m;
    }
}

/// Performance statistics for coordinate transformations
#[derive(Debug, Clone)]
pub struct CoordinateSystemStats {
    pub total_transformations: u64,
    pub cache_hits: u64,
    pub cache_hit_rate: f64,
    pub cached_references: usize,
}

impl Default for CoordinateSystemManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_reference_point_creation() {
        let pos = Position { lat: 32.123, lon: 45.456, depth: 10.0 };
        let ref_point = ReferencePoint::new(pos.clone());
        
        assert!((ref_point.position.lat - pos.lat).abs() < 1e-10);
        assert!((ref_point.position.lon - pos.lon).abs() < 1e-10);
        assert!(ref_point.meters_per_deg_lat > 0.0);
        assert!(ref_point.meters_per_deg_lon > 0.0);
    }
    
    #[test]
    fn test_coordinate_transformation_consistency() {
        let mut manager = CoordinateSystemManager::new();
        let reference = Position { lat: 32.123, lon: 45.456, depth: 0.0 };
        let test_pos = Position { lat: 32.124, lon: 45.457, depth: 5.0 };
        
        // Forward transformation
        let local = manager.geodetic_to_local_optimized(&test_pos, &reference);
        
        // Reverse transformation
        let recovered = manager.local_to_geodetic_optimized(&local, &reference);
        
        // Check consistency (should be very close)
        assert!((recovered.lat - test_pos.lat).abs() < 1e-8);
        assert!((recovered.lon - test_pos.lon).abs() < 1e-8);
        assert!((recovered.depth - test_pos.depth).abs() < 1e-10);
    }
    
    #[test]
    fn test_cache_functionality() {
        let mut manager = CoordinateSystemManager::new();
        let reference = Position { lat: 32.123, lon: 45.456, depth: 0.0 };
        let test_pos = Position { lat: 32.124, lon: 45.457, depth: 5.0 };
        
        // First transformation (cache miss)
        let _local1 = manager.geodetic_to_local_optimized(&test_pos, &reference);
        let stats1 = manager.get_performance_stats();
        
        // Second transformation with same reference (cache hit)
        let _local2 = manager.geodetic_to_local_optimized(&test_pos, &reference);
        let stats2 = manager.get_performance_stats();
        
        assert_eq!(stats1.cache_hits, 0);
        assert_eq!(stats2.cache_hits, 1);
        assert!(stats2.cache_hit_rate > 0.0);
    }
    
    #[test]
    fn test_batch_transformation() {
        let mut manager = CoordinateSystemManager::new();
        let reference = Position { lat: 32.123, lon: 45.456, depth: 0.0 };
        let positions = vec![
            Position { lat: 32.124, lon: 45.457, depth: 5.0 },
            Position { lat: 32.125, lon: 45.458, depth: 10.0 },
            Position { lat: 32.126, lon: 45.459, depth: 15.0 },
        ];
        
        let local_positions = manager.geodetic_to_local_batch(&positions, &reference);
        
        assert_eq!(local_positions.len(), 3);
        
        // Verify each transformation individually
        for (i, pos) in positions.iter().enumerate() {
            let individual = manager.geodetic_to_local_optimized(pos, &reference);
            let batch_result = &local_positions[i];
            
            assert!((individual.x - batch_result.x).abs() < 1e-10);
            assert!((individual.y - batch_result.y).abs() < 1e-10);
            assert!((individual.z - batch_result.z).abs() < 1e-10);
        }
    }
}
/// Supported coordinate system types
#[derive(Debug, Clone, PartialEq)]
pub enum CoordinateSystemType {
    WGS84,
    UTM { zone: u8, northern: bool },
    StatePlane { zone: u16, datum: Datum },
    LocalGrid { origin: Position, rotation: f64 },
}

/// Supported geodetic datums
#[derive(Debug, Clone, PartialEq)]
pub enum Datum {
    WGS84,
    NAD83,
    NAD27,
}

/// UTM coordinate representation
#[derive(Debug, Clone)]
pub struct UTMCoordinate {
    pub easting: f64,
    pub northing: f64,
    pub zone: u8,
    pub northern: bool,
    pub elevation: f64,
}

/// State Plane coordinate representation
#[derive(Debug, Clone)]
pub struct StatePlaneCoordinate {
    pub x: f64,
    pub y: f64,
    pub zone: u16,
    pub datum: Datum,
    pub elevation: f64,
}

/// Multi-coordinate system manager with validation and error checking
#[derive(Debug)]
pub struct MultiCoordinateSystemManager {
    primary_system: CoordinateSystemType,
    coordinate_manager: CoordinateSystemManager,
    utm_cache: HashMap<u8, UTMProjectionParams>,
    validation_enabled: bool,
    earth_curvature_correction: bool,
    max_operational_area_km: f64,
}

/// UTM projection parameters for caching
#[derive(Debug, Clone)]
struct UTMProjectionParams {
    central_meridian: f64,
    scale_factor: f64,
    false_easting: f64,
    false_northing: f64,
}

impl MultiCoordinateSystemManager {
    pub fn new(primary_system: CoordinateSystemType) -> Self {
        Self {
            primary_system,
            coordinate_manager: CoordinateSystemManager::new(),
            utm_cache: HashMap::new(),
            validation_enabled: true,
            earth_curvature_correction: true,
            max_operational_area_km: 50.0, // 50km operational area
        }
    }
    
    /// Convert WGS84 position to UTM coordinates
    pub fn wgs84_to_utm(&mut self, position: &Position) -> Result<UTMCoordinate, String> {
        if !self.validate_position(position)? {
            return Err("Invalid WGS84 position".to_string());
        }
        
        let zone = self.calculate_utm_zone(position.lon);
        let northern = position.lat >= 0.0;
        
        let utm_params = self.get_or_create_utm_params(zone).clone();
        let earth_curvature_correction = self.earth_curvature_correction;
        let max_operational_area_km = self.max_operational_area_km;
        
        let lat_rad = position.lat.to_radians();
        let lon_rad = position.lon.to_radians();
        let central_meridian_rad = utm_params.central_meridian.to_radians();
        
        // WGS84 ellipsoid parameters
        const A: f64 = 6378137.0; // Semi-major axis
        const E2: f64 = 0.00669437999014; // First eccentricity squared
        const E4: f64 = E2 * E2;
        const E6: f64 = E4 * E2;
        
        let sin_lat = lat_rad.sin();
        let cos_lat = lat_rad.cos();
        let tan_lat = lat_rad.tan();
        
        let dlon = lon_rad - central_meridian_rad;
        let _cos_dlon = dlon.cos();
        let _sin_dlon = dlon.sin();
        
        // Calculate meridional arc
        let m = A * ((1.0 - E2/4.0 - 3.0*E4/64.0 - 5.0*E6/256.0) * lat_rad
                   - (3.0*E2/8.0 + 3.0*E4/32.0 + 45.0*E6/1024.0) * (2.0*lat_rad).sin()
                   + (15.0*E4/256.0 + 45.0*E6/1024.0) * (4.0*lat_rad).sin()
                   - (35.0*E6/3072.0) * (6.0*lat_rad).sin());
        
        // Calculate radius of curvature in prime vertical
        let n = A / (1.0 - E2 * sin_lat * sin_lat).sqrt();
        
        // Calculate T, C, A terms
        let t = tan_lat * tan_lat;
        let c = E2 * cos_lat * cos_lat / (1.0 - E2);
        let a_term = cos_lat * dlon;
        
        // Apply Earth curvature correction for large operational areas
        let curvature_correction = if earth_curvature_correction && 
                                     max_operational_area_km > 10.0 {
            let distance_from_central_meridian = (position.lon - utm_params.central_meridian).abs();
            if distance_from_central_meridian > 3.0 { // More than 3 degrees from central meridian
                1.0 + distance_from_central_meridian * distance_from_central_meridian * 1e-6
            } else {
                1.0
            }
        } else {
            1.0
        };
        
        // Calculate UTM coordinates
        let easting = utm_params.scale_factor * n * (
            a_term + (1.0 - t + c) * a_term.powi(3) / 6.0
            + (5.0 - 18.0*t + t*t + 72.0*c - 58.0*E2) * a_term.powi(5) / 120.0
        ) * curvature_correction + utm_params.false_easting;
        
        let northing = utm_params.scale_factor * (
            m + n * tan_lat * (
                a_term.powi(2) / 2.0
                + (5.0 - t + 9.0*c + 4.0*c*c) * a_term.powi(4) / 24.0
                + (61.0 - 58.0*t + t*t + 600.0*c - 330.0*E2) * a_term.powi(6) / 720.0
            )
        ) * curvature_correction + if northern { 0.0 } else { 10000000.0 };
        
        Ok(UTMCoordinate {
            easting,
            northing,
            zone,
            northern,
            elevation: position.depth,
        })
    }
    
    /// Convert UTM coordinates back to WGS84
    pub fn utm_to_wgs84(&mut self, utm: &UTMCoordinate) -> Result<Position, String> {
        self.validate_utm_coordinate(utm)?;
        
        let utm_params = self.get_or_create_utm_params(utm.zone).clone();
        
        // Remove false easting/northing
        let x = utm.easting - utm_params.false_easting;
        let y = if utm.northern { utm.northing } else { utm.northing - 10000000.0 };
        
        // WGS84 ellipsoid parameters
        const A: f64 = 6378137.0;
        const E2: f64 = 0.00669437999014;
        let e1 = (1.0 - (1.0 - E2).sqrt()) / (1.0 + (1.0 - E2).sqrt());
        
        let m = y / utm_params.scale_factor;
        
        // Calculate footprint latitude
        let mu = m / (A * (1.0 - E2/4.0 - 3.0*E2*E2/64.0 - 5.0*E2*E2*E2/256.0));
        
        let lat1 = mu + (3.0*e1/2.0 - 27.0*e1*e1*e1/32.0) * (2.0*mu).sin()
                      + (21.0*e1*e1/16.0 - 55.0*e1*e1*e1*e1/32.0) * (4.0*mu).sin()
                      + (151.0*e1*e1*e1/96.0) * (6.0*mu).sin();
        
        let sin_lat1 = lat1.sin();
        let cos_lat1 = lat1.cos();
        let tan_lat1 = lat1.tan();
        
        let n1 = A / (1.0 - E2 * sin_lat1 * sin_lat1).sqrt();
        let t1 = tan_lat1 * tan_lat1;
        let c1 = E2 * cos_lat1 * cos_lat1 / (1.0 - E2);
        let r1 = A * (1.0 - E2) / (1.0 - E2 * sin_lat1 * sin_lat1).powf(1.5);
        let d = x / (n1 * utm_params.scale_factor);
        
        // Calculate latitude
        let lat = lat1 - (n1 * tan_lat1 / r1) * (
            d*d/2.0 - (5.0 + 3.0*t1 + 10.0*c1 - 4.0*c1*c1 - 9.0*E2) * d.powi(4) / 24.0
            + (61.0 + 90.0*t1 + 298.0*c1 + 45.0*t1*t1 - 252.0*E2 - 3.0*c1*c1) * d.powi(6) / 720.0
        );
        
        // Calculate longitude
        let lon = utm_params.central_meridian.to_radians() + (
            d - (1.0 + 2.0*t1 + c1) * d.powi(3) / 6.0
            + (5.0 - 2.0*c1 + 28.0*t1 - 3.0*c1*c1 + 8.0*E2 + 24.0*t1*t1) * d.powi(5) / 120.0
        ) / cos_lat1;
        
        Ok(Position {
            lat: lat.to_degrees(),
            lon: lon.to_degrees(),
            depth: utm.elevation,
        })
    }
    
    /// Validate WGS84 position coordinates
    pub fn validate_position(&self, position: &Position) -> Result<bool, String> {
        if !self.validation_enabled {
            return Ok(true);
        }
        
        // Check latitude bounds
        if position.lat < -90.0 || position.lat > 90.0 {
            return Err(format!("Invalid latitude: {} (must be between -90 and 90)", position.lat));
        }
        
        // Check longitude bounds
        if position.lon < -180.0 || position.lon > 180.0 {
            return Err(format!("Invalid longitude: {} (must be between -180 and 180)", position.lon));
        }
        
        // Check depth (reasonable underwater limits)
        if position.depth < -1000.0 || position.depth > 11000.0 {
            return Err(format!("Invalid depth: {} (must be between -1000 and 11000 meters)", position.depth));
        }
        
        Ok(true)
    }
    
    /// Validate UTM coordinates
    pub fn validate_utm_coordinate(&self, utm: &UTMCoordinate) -> Result<(), String> {
        if !self.validation_enabled {
            return Ok(());
        }
        
        // Check UTM zone
        if utm.zone < 1 || utm.zone > 60 {
            return Err(format!("Invalid UTM zone: {} (must be between 1 and 60)", utm.zone));
        }
        
        // Check easting bounds (typical UTM bounds)
        if utm.easting < 160000.0 || utm.easting > 840000.0 {
            return Err(format!("Invalid UTM easting: {} (outside typical bounds)", utm.easting));
        }
        
        // Check northing bounds
        let max_northing = if utm.northern { 9400000.0 } else { 10000000.0 };
        if utm.northing < 0.0 || utm.northing > max_northing {
            return Err(format!("Invalid UTM northing: {} (outside bounds for hemisphere)", utm.northing));
        }
        
        Ok(())
    }
    
    /// Calculate UTM zone from longitude
    fn calculate_utm_zone(&self, longitude: f64) -> u8 {
        ((longitude + 180.0) / 6.0).floor() as u8 + 1
    }
    
    /// Get or create UTM projection parameters for a zone
    fn get_or_create_utm_params(&mut self, zone: u8) -> &UTMProjectionParams {
        if !self.utm_cache.contains_key(&zone) {
            let central_meridian = -183.0 + (zone as f64) * 6.0;
            let params = UTMProjectionParams {
                central_meridian,
                scale_factor: 0.9996,
                false_easting: 500000.0,
                false_northing: 0.0,
            };
            self.utm_cache.insert(zone, params);
        }
        self.utm_cache.get(&zone).unwrap()
    }
    
    /// Apply Earth curvature corrections for large operational areas
    pub fn apply_earth_curvature_correction(&mut self, positions: &mut [Position], reference: &Position) -> Result<(), String> {
        if !self.earth_curvature_correction {
            return Ok(());
        }
        
        // Calculate maximum distance from reference
        let mut max_distance: f64 = 0.0;
        for pos in positions.iter() {
            let distance = self.calculate_great_circle_distance(reference, pos)?;
            max_distance = max_distance.max(distance);
        }
        
        // Apply correction if operational area is large
        if max_distance > self.max_operational_area_km {
            for pos in positions.iter_mut() {
                let distance = self.calculate_great_circle_distance(reference, pos)?;
                let correction_factor = 1.0 + (distance / 6371000.0).powi(2) / 24.0; // Earth radius approximation
                
                // Apply small correction to coordinates
                let dlat = pos.lat - reference.lat;
                let dlon = pos.lon - reference.lon;
                
                pos.lat = reference.lat + dlat * correction_factor;
                pos.lon = reference.lon + dlon * correction_factor;
            }
        }
        
        Ok(())
    }
    
    /// Calculate great circle distance between two positions
    fn calculate_great_circle_distance(&self, pos1: &Position, pos2: &Position) -> Result<f64, String> {
        let lat1_rad = pos1.lat.to_radians();
        let lat2_rad = pos2.lat.to_radians();
        let dlat_rad = (pos2.lat - pos1.lat).to_radians();
        let dlon_rad = (pos2.lon - pos1.lon).to_radians();
        
        let a = (dlat_rad / 2.0).sin().powi(2) + 
                lat1_rad.cos() * lat2_rad.cos() * (dlon_rad / 2.0).sin().powi(2);
        let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
        
        const EARTH_RADIUS_M: f64 = 6371000.0;
        Ok(EARTH_RADIUS_M * c / 1000.0) // Return in kilometers
    }
    
    /// Configure validation and correction settings
    pub fn configure_validation(&mut self, enable_validation: bool, enable_curvature_correction: bool, max_area_km: f64) {
        self.validation_enabled = enable_validation;
        self.earth_curvature_correction = enable_curvature_correction;
        self.max_operational_area_km = max_area_km;
    }
    
    /// Get coordinate system information
    pub fn get_system_info(&self) -> CoordinateSystemInfo {
        CoordinateSystemInfo {
            primary_system: self.primary_system.clone(),
            validation_enabled: self.validation_enabled,
            earth_curvature_correction: self.earth_curvature_correction,
            max_operational_area_km: self.max_operational_area_km,
            cached_utm_zones: self.utm_cache.keys().cloned().collect(),
        }
    }
}

/// Information about the coordinate system configuration
#[derive(Debug, Clone)]
pub struct CoordinateSystemInfo {
    pub primary_system: CoordinateSystemType,
    pub validation_enabled: bool,
    pub earth_curvature_correction: bool,
    pub max_operational_area_km: f64,
    pub cached_utm_zones: Vec<u8>,
}

#[cfg(test)]
mod multi_coord_tests {
    use super::*;
    
    #[test]
    fn test_wgs84_to_utm_conversion() {
        let mut manager = MultiCoordinateSystemManager::new(CoordinateSystemType::WGS84);
        let wgs84_pos = Position { lat: 40.7128, lon: -74.0060, depth: 0.0 }; // New York City
        
        let utm_result = manager.wgs84_to_utm(&wgs84_pos).unwrap();
        
        assert_eq!(utm_result.zone, 18); // NYC is in UTM zone 18
        assert!(utm_result.northern);
        assert!(utm_result.easting > 0.0);
        assert!(utm_result.northing > 0.0);
    }
    
    #[test]
    fn test_utm_to_wgs84_roundtrip() {
        let mut manager = MultiCoordinateSystemManager::new(CoordinateSystemType::WGS84);
        let original_pos = Position { lat: 32.123, lon: 45.456, depth: 10.0 };
        
        // Convert to UTM and back
        let utm = manager.wgs84_to_utm(&original_pos).unwrap();
        let recovered_pos = manager.utm_to_wgs84(&utm).unwrap();
        
        // Should be very close (within 1 meter accuracy)
        assert!((recovered_pos.lat - original_pos.lat).abs() < 1e-5);
        assert!((recovered_pos.lon - original_pos.lon).abs() < 1e-5);
        assert!((recovered_pos.depth - original_pos.depth).abs() < 1e-10);
    }
    
    #[test]
    fn test_position_validation() {
        let manager = MultiCoordinateSystemManager::new(CoordinateSystemType::WGS84);
        
        // Valid position
        let valid_pos = Position { lat: 32.123, lon: 45.456, depth: 10.0 };
        assert!(manager.validate_position(&valid_pos).unwrap());
        
        // Invalid latitude
        let invalid_lat = Position { lat: 95.0, lon: 45.456, depth: 10.0 };
        assert!(manager.validate_position(&invalid_lat).is_err());
        
        // Invalid longitude
        let invalid_lon = Position { lat: 32.123, lon: 185.0, depth: 10.0 };
        assert!(manager.validate_position(&invalid_lon).is_err());
        
        // Invalid depth
        let invalid_depth = Position { lat: 32.123, lon: 45.456, depth: 15000.0 };
        assert!(manager.validate_position(&invalid_depth).is_err());
    }
    
    #[test]
    fn test_utm_zone_calculation() {
        let manager = MultiCoordinateSystemManager::new(CoordinateSystemType::WGS84);
        
        assert_eq!(manager.calculate_utm_zone(-74.0), 18); // New York
        assert_eq!(manager.calculate_utm_zone(0.0), 31);   // Greenwich
        assert_eq!(manager.calculate_utm_zone(45.0), 38);  // Eastern Europe
    }
}