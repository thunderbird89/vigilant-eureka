use nalgebra::Vector3;
use std::collections::HashMap;
use std::f64::consts::PI;

/// Position representation compatible with existing code
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Position {
    pub lat: f64,  // degrees
    pub lon: f64,  // degrees
    pub depth: f64, // meters (positive down)
}

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
        
        let lat_rad = position.lat.to_radians();
        let lon_rad = position.lon.to_radians();
        let central_meridian_rad = utm_params.central_meridian.to_radians();
        
        // WGS84 ellipsoid parameters
        const A: f64 = 6378137.0; // Semi-major axis
        const E2: f64 = 0.00669437999014; // First eccentricity squared
        
        let sin_lat = lat_rad.sin();
        let cos_lat = lat_rad.cos();
        let tan_lat = lat_rad.tan();
        
        let dlon = lon_rad - central_meridian_rad;
        
        // Calculate meridional arc
        let m = A * ((1.0 - E2/4.0 - 3.0*E2*E2/64.0) * lat_rad
                   - (3.0*E2/8.0 + 3.0*E2*E2/32.0) * (2.0*lat_rad).sin()
                   + (15.0*E2*E2/256.0) * (4.0*lat_rad).sin());
        
        // Calculate radius of curvature in prime vertical
        let n = A / (1.0 - E2 * sin_lat * sin_lat).sqrt();
        
        // Calculate T, C, A terms
        let t = tan_lat * tan_lat;
        let c = E2 * cos_lat * cos_lat / (1.0 - E2);
        let a_term = cos_lat * dlon;
        
        // Calculate UTM coordinates
        let easting = utm_params.scale_factor * n * (
            a_term + (1.0 - t + c) * a_term.powi(3) / 6.0
            + (5.0 - 18.0*t + t*t + 72.0*c - 58.0*E2) * a_term.powi(5) / 120.0
        ) + utm_params.false_easting;
        
        let northing = utm_params.scale_factor * (
            m + n * tan_lat * (
                a_term.powi(2) / 2.0
                + (5.0 - t + 9.0*c + 4.0*c*c) * a_term.powi(4) / 24.0
                + (61.0 - 58.0*t + t*t + 600.0*c - 330.0*E2) * a_term.powi(6) / 720.0
            )
        ) + if northern { 0.0 } else { 10000000.0 };
        
        Ok(UTMCoordinate {
            easting,
            northing,
            zone,
            northern,
            elevation: position.depth,
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
    
    /// Calculate great circle distance between two positions
    pub fn calculate_great_circle_distance(&self, pos1: &Position, pos2: &Position) -> Result<f64, String> {
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
mod tests {
    use super::*;
    
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
    fn test_wgs84_to_utm_conversion() {
        let mut manager = MultiCoordinateSystemManager::new(CoordinateSystemType::WGS84);
        let wgs84_pos = Position { lat: 40.7128, lon: -74.0060, depth: 0.0 }; // New York City
        
        let utm_result = manager.wgs84_to_utm(&wgs84_pos).unwrap();
        
        assert_eq!(utm_result.zone, 18); // NYC is in UTM zone 18
        assert!(utm_result.northern);
        assert!(utm_result.easting > 0.0);
        assert!(utm_result.northing > 0.0);
    }
}