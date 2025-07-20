//! Unit tests for coordinate system transformations
//! 
//! Tests cover:
//! - WGS84 geodetic coordinate validation
//! - Local tangent plane transformations
//! - UTM coordinate system conversions
//! - Coordinate system validation and error handling

use crate::algorithms::embedded_coordinates::{
    EmbeddedCoordinateManager, CoordinateSystem, CoordinateValidator, 
    ReferencePoint, LocalGridParams, FixedCoordinate
};
use crate::core::Position;

/// Test utilities for coordinate system testing
pub struct CoordinateTestUtils;

impl CoordinateTestUtils {
    /// Create test positions around the world
    pub fn create_global_test_positions() -> Vec<Position> {
        vec![
            // Equator and Prime Meridian
            Position { lat: 0.0, lon: 0.0, depth: 0.0 },
            
            // Major cities
            Position { lat: 40.7128, lon: -74.0060, depth: 0.0 }, // New York
            Position { lat: 51.5074, lon: -0.1278, depth: 0.0 },  // London
            Position { lat: 35.6762, lon: 139.6503, depth: 0.0 }, // Tokyo
            Position { lat: -33.8688, lon: 151.2093, depth: 0.0 }, // Sydney
            Position { lat: -22.9068, lon: -43.1729, depth: 0.0 }, // Rio de Janeiro
            
            // Extreme coordinates
            Position { lat: 89.0, lon: 0.0, depth: 0.0 },    // Near North Pole
            Position { lat: -89.0, lon: 0.0, depth: 0.0 },   // Near South Pole
            Position { lat: 0.0, lon: 179.0, depth: 0.0 },   // Near Date Line
            Position { lat: 0.0, lon: -179.0, depth: 0.0 },  // Near Date Line (other side)
            
            // Various depths
            Position { lat: 37.7749, lon: -122.4194, depth: 100.0 },  // San Francisco, 100m deep
            Position { lat: 25.7617, lon: -80.1918, depth: 1000.0 },  // Miami, 1km deep
            Position { lat: 36.2048, lon: 138.2529, depth: 5000.0 },  // Japan Trench area, 5km deep
        ]
    }

    /// Create reference positions for transformation testing
    pub fn create_reference_positions() -> Vec<Position> {
        vec![
            Position { lat: 0.0, lon: 0.0, depth: 0.0 },      // Origin
            Position { lat: 45.0, lon: 0.0, depth: 0.0 },     // Mid-latitude
            Position { lat: 60.0, lon: 30.0, depth: 0.0 },    // High latitude
            Position { lat: -30.0, lon: -60.0, depth: 0.0 },  // Southern hemisphere
            Position { lat: 0.0, lon: 180.0, depth: 0.0 },    // Date line
        ]
    }

    /// Calculate expected UTM zone for longitude
    pub fn calculate_expected_utm_zone(longitude: f64) -> u8 {
        ((longitude + 180.0) / 6.0).floor() as u8 + 1
    }

    /// Validate coordinate transformation accuracy
    pub fn validate_coordinate_accuracy(
        original: &Position,
        transformed: &Position,
        max_error_m: f64,
    ) -> bool {
        let lat_error_m = (original.lat - transformed.lat).abs() * 111320.0;
        let lon_error_m = (original.lon - transformed.lon).abs() * 111320.0 * original.lat.to_radians().cos();
        let depth_error_m = (original.depth - transformed.depth).abs();
        
        let total_error = (lat_error_m.powi(2) + lon_error_m.powi(2) + depth_error_m.powi(2)).sqrt();
        total_error <= max_error_m
    }

    /// Create test cases for coordinate validation
    pub fn create_validation_test_cases() -> Vec<(Position, bool, &'static str)> {
        vec![
            // Valid coordinates
            (Position { lat: 0.0, lon: 0.0, depth: 0.0 }, true, "Origin"),
            (Position { lat: 45.0, lon: -122.0, depth: 100.0 }, true, "Normal position"),
            (Position { lat: 90.0, lon: 0.0, depth: 0.0 }, true, "North Pole"),
            (Position { lat: -90.0, lon: 0.0, depth: 0.0 }, true, "South Pole"),
            (Position { lat: 0.0, lon: 180.0, depth: 0.0 }, true, "Date Line East"),
            (Position { lat: 0.0, lon: -180.0, depth: 0.0 }, true, "Date Line West"),
            (Position { lat: 0.0, lon: 0.0, depth: 11000.0 }, true, "Mariana Trench depth"),
            
            // Invalid coordinates
            (Position { lat: 91.0, lon: 0.0, depth: 0.0 }, false, "Invalid latitude (too high)"),
            (Position { lat: -91.0, lon: 0.0, depth: 0.0 }, false, "Invalid latitude (too low)"),
            (Position { lat: 0.0, lon: 181.0, depth: 0.0 }, false, "Invalid longitude (too high)"),
            (Position { lat: 0.0, lon: -181.0, depth: 0.0 }, false, "Invalid longitude (too low)"),
            (Position { lat: 0.0, lon: 0.0, depth: -12000.0 }, false, "Invalid depth (too deep)"),
            (Position { lat: 0.0, lon: 0.0, depth: 10000.0 }, false, "Invalid depth (above sea level)"),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_coordinate_validation() {
        let test_cases = CoordinateTestUtils::create_validation_test_cases();
        
        for (position, should_be_valid, description) in test_cases {
            let result = CoordinateValidator::validate_wgs84(&position);
            
            if should_be_valid {
                assert!(result.is_ok(), "Should be valid: {} - {:?}", description, position);
            } else {
                assert!(result.is_err(), "Should be invalid: {} - {:?}", description, position);
            }
        }
    }

    #[test]
    fn test_utm_zone_calculation() {
        let test_cases = vec![
            (-180.0, 1),   // Western edge
            (-174.0, 1),   // Zone 1
            (-168.0, 2),   // Zone 2
            (0.0, 31),     // Greenwich (Zone 31)
            (6.0, 32),     // Zone 32
            (120.0, 50),   // Zone 50
            (174.0, 60),   // Zone 60
            (180.0, 60),   // Eastern edge (still Zone 60)
        ];
        
        for (longitude, expected_zone) in test_cases {
            let calculated_zone = CoordinateTestUtils::calculate_expected_utm_zone(longitude);
            assert_eq!(calculated_zone, expected_zone,
                      "UTM zone for longitude {}: expected {}, got {}", 
                      longitude, expected_zone, calculated_zone);
        }
    }

    #[test]
    fn test_coordinate_manager_reference_point_management() {
        let mut manager = EmbeddedCoordinateManager::new();
        
        // Initially no reference point
        assert!(manager.get_reference_point().is_none());
        
        // Set first reference point
        let ref1 = Position { lat: 37.7749, lon: -122.4194, depth: 0.0 };
        let result1 = manager.set_reference_point(ref1, 1000);
        assert!(result1.is_ok());
        assert!(manager.get_reference_point().is_some());
        
        let retrieved_ref = manager.get_reference_point().unwrap();
        assert_eq!(retrieved_ref.geodetic.lat, ref1.lat);
        assert_eq!(retrieved_ref.geodetic.lon, ref1.lon);
        assert_eq!(retrieved_ref.geodetic.depth, ref1.depth);
        assert_eq!(retrieved_ref.timestamp_ms, 1000);
        
        // Set second reference point (should replace first)
        let ref2 = Position { lat: 40.7128, lon: -74.0060, depth: 0.0 };
        let result2 = manager.set_reference_point(ref2, 2000);
        assert!(result2.is_ok());
        
        let retrieved_ref2 = manager.get_reference_point().unwrap();
        assert_eq!(retrieved_ref2.geodetic.lat, ref2.lat);
        assert_eq!(retrieved_ref2.geodetic.lon, ref2.lon);
        assert_eq!(retrieved_ref2.timestamp_ms, 2000);
    }

    #[test]
    fn test_geodetic_to_local_transformations() {
        let mut manager = EmbeddedCoordinateManager::new();
        let test_positions = CoordinateTestUtils::create_global_test_positions();
        let reference_positions = CoordinateTestUtils::create_reference_positions();
        
        for (ref_idx, reference) in reference_positions.iter().enumerate() {
            manager.set_reference_point(*reference, 1000 + ref_idx as u64).unwrap();
            
            for (pos_idx, position) in test_positions.iter().enumerate() {
                // Skip if position is same as reference
                if (position.lat - reference.lat).abs() < 1e-10 && 
                   (position.lon - reference.lon).abs() < 1e-10 &&
                   (position.depth - reference.depth).abs() < 1e-10 {
                    continue;
                }
                
                let local_result = manager.geodetic_to_local(position);
                
                if local_result.is_ok() {
                    let (x, y, z) = local_result.unwrap();
                    
                    // Local coordinates should be reasonable
                    assert!(x.abs() < 20000000.0, // 20,000 km max
                           "Local X coordinate too large: {} for ref {} pos {}", x, ref_idx, pos_idx);
                    assert!(y.abs() < 20000000.0, // 20,000 km max
                           "Local Y coordinate too large: {} for ref {} pos {}", y, ref_idx, pos_idx);
                    assert!(z.abs() < 15000.0, // 15 km max depth
                           "Local Z coordinate too large: {} for ref {} pos {}", z, ref_idx, pos_idx);
                    
                    // Test round-trip conversion
                    let back_to_geodetic = manager.local_to_geodetic((x, y, z));
                    assert!(back_to_geodetic.is_ok(), 
                           "Round-trip conversion should succeed for ref {} pos {}", ref_idx, pos_idx);
                    
                    let converted_back = back_to_geodetic.unwrap();
                    
                    // Validate round-trip accuracy (allow larger tolerance for global coordinates)
                    let max_error = if (position.lat.abs() > 80.0) || 
                                      ((position.lon.abs() - 180.0).abs() < 1.0) {
                        100.0 // 100m tolerance for polar/date line regions
                    } else {
                        10.0  // 10m tolerance for normal regions
                    };
                    
                    let is_accurate = CoordinateTestUtils::validate_coordinate_accuracy(
                        position, &converted_back, max_error
                    );
                    
                    assert!(is_accurate,
                           "Round-trip accuracy failed for ref {} pos {}: original={:?}, converted={:?}",
                           ref_idx, pos_idx, position, converted_back);
                }
            }
        }
    }

    #[test]
    fn test_local_coordinate_properties() {
        let mut manager = EmbeddedCoordinateManager::new();
        let reference = Position { lat: 45.0, lon: 0.0, depth: 0.0 };
        manager.set_reference_point(reference, 1000).unwrap();
        
        // Test cardinal directions from reference point
        let test_cases = vec![
            // (description, position, expected_local_approx)
            ("1km North", Position { lat: 45.009, lon: 0.0, depth: 0.0 }, (0.0, 1000.0, 0.0)),
            ("1km South", Position { lat: 44.991, lon: 0.0, depth: 0.0 }, (0.0, -1000.0, 0.0)),
            ("1km East", Position { lat: 45.0, lon: 0.0127, depth: 0.0 }, (1000.0, 0.0, 0.0)),
            ("1km West", Position { lat: 45.0, lon: -0.0127, depth: 0.0 }, (-1000.0, 0.0, 0.0)),
            ("100m Down", Position { lat: 45.0, lon: 0.0, depth: 100.0 }, (0.0, 0.0, -100.0)),
            ("100m Up", Position { lat: 45.0, lon: 0.0, depth: -100.0 }, (0.0, 0.0, 100.0)),
        ];
        
        for (description, position, expected) in test_cases {
            let local_result = manager.geodetic_to_local(&position);
            assert!(local_result.is_ok(), "Conversion should succeed for {}", description);
            
            let (x, y, z) = local_result.unwrap();
            let tolerance = 50.0; // 50m tolerance for coordinate approximations
            
            assert!((x - expected.0).abs() < tolerance,
                   "{}: East component expected {}, got {} (diff: {})", 
                   description, expected.0, x, (x - expected.0).abs());
            assert!((y - expected.1).abs() < tolerance,
                   "{}: North component expected {}, got {} (diff: {})", 
                   description, expected.1, y, (y - expected.1).abs());
            assert!((z - expected.2).abs() < tolerance,
                   "{}: Up component expected {}, got {} (diff: {})", 
                   description, expected.2, z, (z - expected.2).abs());
        }
    }

    #[test]
    fn test_utm_coordinate_conversions() {
        let mut manager = EmbeddedCoordinateManager::new();
        let test_positions = vec![
            Position { lat: 40.7128, lon: -74.0060, depth: 100.0 }, // New York
            Position { lat: 51.5074, lon: -0.1278, depth: 50.0 },   // London
            Position { lat: 35.6762, lon: 139.6503, depth: 200.0 }, // Tokyo
            Position { lat: -33.8688, lon: 151.2093, depth: 75.0 }, // Sydney
        ];
        
        for (i, position) in test_positions.iter().enumerate() {
            let utm_result = manager.geodetic_to_utm(position);
            assert!(utm_result.is_ok(), "UTM conversion should succeed for position {}", i);
            
            let (easting, northing, altitude, zone, is_north) = utm_result.unwrap();
            
            // Validate UTM coordinates
            assert!(easting > 100000.0 && easting < 900000.0,
                   "UTM easting should be valid: {} for position {}", easting, i);
            assert!(northing >= 0.0 && northing <= 10000000.0,
                   "UTM northing should be valid: {} for position {}", northing, i);
            assert_eq!(altitude, -position.depth,
                      "UTM altitude should match negative depth for position {}", i);
            
            // Validate zone
            let expected_zone = CoordinateTestUtils::calculate_expected_utm_zone(position.lon);
            assert_eq!(zone, expected_zone,
                      "UTM zone should be correct: expected {}, got {} for position {}", 
                      expected_zone, zone, i);
            
            // Validate hemisphere
            assert_eq!(is_north, position.lat >= 0.0,
                      "UTM hemisphere should match latitude sign for position {}", i);
            
            // Test round-trip conversion
            let back_to_geodetic = manager.utm_to_geodetic(easting, northing, altitude, zone, is_north);
            assert!(back_to_geodetic.is_ok(), 
                   "UTM to geodetic conversion should succeed for position {}", i);
            
            let converted_back = back_to_geodetic.unwrap();
            
            // Validate round-trip accuracy (UTM has some inherent distortion)
            let is_accurate = CoordinateTestUtils::validate_coordinate_accuracy(
                position, &converted_back, 1.0 // 1m tolerance for UTM
            );
            
            assert!(is_accurate,
                   "UTM round-trip accuracy failed for position {}: original={:?}, converted={:?}",
                   i, position, converted_back);
        }
    }

    #[test]
    fn test_coordinate_system_validation_interface() {
        let mut manager = EmbeddedCoordinateManager::new();
        
        // Test validation without reference point
        let result1 = manager.validate_transformation(
            CoordinateSystem::WGS84Geodetic, 
            CoordinateSystem::LocalTangentPlane
        );
        assert!(result1.is_err(), "Should fail without reference point");
        
        // Set reference point
        let reference = Position { lat: 0.0, lon: 0.0, depth: 0.0 };
        manager.set_reference_point(reference, 1000).unwrap();
        
        // Test validation with reference point
        let result2 = manager.validate_transformation(
            CoordinateSystem::WGS84Geodetic, 
            CoordinateSystem::LocalTangentPlane
        );
        assert!(result2.is_ok(), "Should succeed with reference point");
        
        // Test UTM validation (always valid)
        let result3 = manager.validate_transformation(
            CoordinateSystem::WGS84Geodetic, 
            CoordinateSystem::UTM
        );
        assert!(result3.is_ok(), "UTM transformation should always be valid");
        
        // Test unsupported transformation
        let result4 = manager.validate_transformation(
            CoordinateSystem::LocalGrid, 
            CoordinateSystem::UTM
        );
        assert!(result4.is_err(), "Unsupported transformation should fail");
    }

    #[test]
    fn test_fixed_point_coordinate_operations() {
        let coord1 = FixedCoordinate::new(123.456, -78.901, 45.678);
        let coord2 = FixedCoordinate::new(234.567, 89.012, -56.789);
        
        // Test conversion accuracy
        let (x1, y1, z1) = coord1.to_f64();
        assert!((x1 - 123.456).abs() < 1e-5, "X conversion accuracy");
        assert!((y1 - (-78.901)).abs() < 1e-5, "Y conversion accuracy");
        assert!((z1 - 45.678).abs() < 1e-5, "Z conversion accuracy");
        
        // Test distance calculation
        let distance_fixed = coord1.distance_to(&coord2);
        let distance_f64 = distance_fixed as f64 / crate::algorithms::embedded_coordinates::COORD_FIXED_POINT_SCALE as f64;
        
        // Calculate expected distance
        let dx = 234.567 - 123.456;
        let dy = 89.012 - (-78.901);
        let dz = -56.789 - 45.678;
        let expected_distance = ((dx * dx + dy * dy + dz * dz) as f64).sqrt();
        
        assert!((distance_f64 - expected_distance).abs() < 0.01,
               "Fixed-point distance calculation: expected {}, got {}", 
               expected_distance, distance_f64);
    }

    #[test]
    fn test_coordinate_transformation_precision_estimates() {
        let manager = EmbeddedCoordinateManager::new();
        
        // Test precision estimates at different distances
        let test_distances = vec![1.0, 10.0, 100.0, 1000.0, 10000.0];
        
        for distance in test_distances {
            let precision = manager.get_transformation_precision(distance);
            
            // Precision should degrade with distance
            assert!(precision > 0.0, "Precision should be positive for distance {}", distance);
            assert!(precision < 1.0, "Precision should be reasonable for distance {}", distance);
            
            // Precision should be worse for larger distances
            if distance > 1000.0 {
                let short_precision = manager.get_transformation_precision(100.0);
                assert!(precision >= short_precision,
                       "Precision should degrade with distance: {} vs {} for distance {}", 
                       precision, short_precision, distance);
            }
        }
    }

    #[test]
    fn test_coordinate_manager_with_large_operational_area() {
        let mut manager = EmbeddedCoordinateManager::with_large_area(50000.0); // 50km radius
        let reference = Position { lat: 0.0, lon: 0.0, depth: 0.0 };
        manager.set_reference_point(reference, 1000).unwrap();
        
        // Test large distance transformation
        let far_position = Position { lat: 0.45, lon: 0.45, depth: 1000.0 }; // ~50km away
        let local_result = manager.geodetic_to_local(&far_position);
        
        assert!(local_result.is_ok(), "Large area transformation should succeed");
        let (x, y, z) = local_result.unwrap();
        
        // Should be approximately 50km in each direction
        assert!(x > 45000.0 && x < 55000.0, "Large area X coordinate: {}", x);
        assert!(y > 45000.0 && y < 55000.0, "Large area Y coordinate: {}", y);
        assert!((z - (-1000.0)).abs() < 10.0, "Large area Z coordinate: {}", z);
        
        // Test round-trip accuracy for large distances
        let back_to_geodetic = manager.local_to_geodetic((x, y, z));
        assert!(back_to_geodetic.is_ok(), "Large area round-trip should succeed");
        
        let converted_back = back_to_geodetic.unwrap();
        let is_accurate = CoordinateTestUtils::validate_coordinate_accuracy(
            &far_position, &converted_back, 10.0 // 10m tolerance for large areas
        );
        
        assert!(is_accurate, "Large area round-trip should be accurate");
    }

    #[test]
    fn test_coordinate_validation_edge_cases() {
        // Test boundary conditions
        let boundary_cases = vec![
            (Position { lat: 90.0, lon: 0.0, depth: 0.0 }, true, "North Pole"),
            (Position { lat: -90.0, lon: 0.0, depth: 0.0 }, true, "South Pole"),
            (Position { lat: 0.0, lon: 180.0, depth: 0.0 }, true, "Date Line East"),
            (Position { lat: 0.0, lon: -180.0, depth: 0.0 }, true, "Date Line West"),
            (Position { lat: 89.999999, lon: 179.999999, depth: 0.0 }, true, "Near limits"),
            (Position { lat: -89.999999, lon: -179.999999, depth: 0.0 }, true, "Near limits negative"),
        ];
        
        for (position, should_be_valid, description) in boundary_cases {
            let result = CoordinateValidator::validate_wgs84(&position);
            
            if should_be_valid {
                assert!(result.is_ok(), "Boundary case should be valid: {}", description);
            } else {
                assert!(result.is_err(), "Boundary case should be invalid: {}", description);
            }
        }
        
        // Test UTM boundary conditions
        let utm_boundary_cases = vec![
            (500000.0, 0.0, 1, true, "UTM equator"),
            (500000.0, 10000000.0, 1, true, "UTM north limit"),
            (100000.0, 5000000.0, 1, true, "UTM west limit"),
            (900000.0, 5000000.0, 1, true, "UTM east limit"),
            (50000.0, 5000000.0, 1, false, "UTM too far west"),
            (950000.0, 5000000.0, 1, false, "UTM too far east"),
            (500000.0, 5000000.0, 0, false, "UTM invalid zone low"),
            (500000.0, 5000000.0, 61, false, "UTM invalid zone high"),
        ];
        
        for (easting, northing, zone, should_be_valid, description) in utm_boundary_cases {
            let result = CoordinateValidator::validate_utm(easting, northing, zone);
            
            if should_be_valid {
                assert!(result.is_ok(), "UTM boundary case should be valid: {}", description);
            } else {
                assert!(result.is_err(), "UTM boundary case should be invalid: {}", description);
            }
        }
    }
}