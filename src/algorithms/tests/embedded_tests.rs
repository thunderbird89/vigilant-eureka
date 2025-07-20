//! Unit tests for embedded-optimized algorithms
//! 
//! Tests cover:
//! - Fixed-point arithmetic accuracy and precision
//! - Memory usage validation within embedded constraints
//! - Stack-based operations without dynamic allocation
//! - Performance validation for real-time constraints

use crate::algorithms::embedded_trilateration::{
    EmbeddedTrilateration, FixedVector3, FixedMatrix3x3, EmbeddedAnchorData, 
    FIXED_POINT_SCALE, MAX_ANCHORS_EMBEDDED, config
};
use crate::algorithms::embedded_coordinates::{
    EmbeddedCoordinateManager, FixedCoordinate, CoordinateSystem, CoordinateValidator,
    COORD_FIXED_POINT_SCALE
};
use crate::core::{Anchor, Position};
use std::time::Instant;

/// Test utilities for embedded algorithm testing
pub struct EmbeddedTestUtils;

impl EmbeddedTestUtils {
    /// Create test anchors within embedded constraints
    pub fn create_embedded_test_anchors() -> Vec<Anchor> {
        vec![
            Anchor {
                id: "E1".to_string(),
                timestamp: 1000,
                position: Position { lat: 0.0, lon: 0.0, depth: 0.0 },
            },
            Anchor {
                id: "E2".to_string(),
                timestamp: 1001,
                position: Position { lat: 0.001, lon: 0.0, depth: 0.0 },
            },
            Anchor {
                id: "E3".to_string(),
                timestamp: 1002,
                position: Position { lat: 0.0, lon: 0.001, depth: 0.0 },
            },
            Anchor {
                id: "E4".to_string(),
                timestamp: 1003,
                position: Position { lat: 0.001, lon: 0.001, depth: 50.0 },
            },
        ]
    }

    /// Create maximum anchor set for embedded testing
    pub fn create_max_embedded_anchors() -> Vec<Anchor> {
        (0..MAX_ANCHORS_EMBEDDED).map(|i| {
            let angle = i as f64 * 2.0 * std::f64::consts::PI / MAX_ANCHORS_EMBEDDED as f64;
            let radius = 100.0; // 100m radius
            let x = radius * angle.cos();
            let y = radius * angle.sin();
            
            Anchor {
                id: format!("MAX{}", i + 1),
                timestamp: 1000 + i as u64,
                position: Position {
                    lat: y / 111320.0, // Convert meters to degrees
                    lon: x / 111320.0,
                    depth: (i as f64 * 10.0) % 100.0, // Varying depths
                },
            }
        }).collect()
    }

    /// Validate fixed-point precision
    pub fn validate_fixed_point_precision(original: f64, fixed_point: i32, tolerance: f64) -> bool {
        let converted_back = fixed_point as f64 / FIXED_POINT_SCALE as f64;
        (original - converted_back).abs() <= tolerance
    }

    /// Measure memory usage (approximate)
    pub fn estimate_memory_usage<T>() -> usize {
        std::mem::size_of::<T>()
    }

    /// Validate real-time performance constraint
    pub fn validate_timing_constraint<F>(operation: F, max_duration_ms: u64) -> bool 
    where
        F: FnOnce(),
    {
        let start = Instant::now();
        operation();
        let duration = start.elapsed();
        duration.as_millis() <= max_duration_ms as u128
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fixed_vector3_operations() {
        // Test basic fixed-point vector operations
        let v1 = FixedVector3::from_f64(10.5, -5.25, 3.75);
        let v2 = FixedVector3::from_f64(2.5, 1.5, -1.25);
        
        // Test conversion accuracy
        let (x1, y1, z1) = v1.to_f64();
        assert!(EmbeddedTestUtils::validate_fixed_point_precision(10.5, v1.x, 1e-4));
        assert!(EmbeddedTestUtils::validate_fixed_point_precision(-5.25, v1.y, 1e-4));
        assert!(EmbeddedTestUtils::validate_fixed_point_precision(3.75, v1.z, 1e-4));
        
        // Test addition
        let sum = v1.add(&v2);
        let (sum_x, sum_y, sum_z) = sum.to_f64();
        assert!((sum_x - 13.0).abs() < 1e-3, "Addition X: expected 13.0, got {}", sum_x);
        assert!((sum_y - (-3.75)).abs() < 1e-3, "Addition Y: expected -3.75, got {}", sum_y);
        assert!((sum_z - 2.5).abs() < 1e-3, "Addition Z: expected 2.5, got {}", sum_z);
        
        // Test subtraction
        let diff = v1.sub(&v2);
        let (diff_x, diff_y, diff_z) = diff.to_f64();
        assert!((diff_x - 8.0).abs() < 1e-3, "Subtraction X: expected 8.0, got {}", diff_x);
        assert!((diff_y - (-6.75)).abs() < 1e-3, "Subtraction Y: expected -6.75, got {}", diff_y);
        assert!((diff_z - 5.0).abs() < 1e-3, "Subtraction Z: expected 5.0, got {}", diff_z);
        
        // Test dot product
        let dot = v1.dot(&v2);
        let expected_dot = 10.5 * 2.5 + (-5.25) * 1.5 + 3.75 * (-1.25);
        let actual_dot = dot as f64 / FIXED_POINT_SCALE as f64;
        assert!((actual_dot - expected_dot).abs() < 1e-2, 
                "Dot product: expected {}, got {}", expected_dot, actual_dot);
        
        // Test scaling
        let scale_factor = (2.0 * FIXED_POINT_SCALE as f64) as i32;
        let scaled = v1.scale(scale_factor);
        let (scaled_x, scaled_y, scaled_z) = scaled.to_f64();
        assert!((scaled_x - 21.0).abs() < 1e-3, "Scaling X: expected 21.0, got {}", scaled_x);
        assert!((scaled_y - (-10.5)).abs() < 1e-3, "Scaling Y: expected -10.5, got {}", scaled_y);
        assert!((scaled_z - 7.5).abs() < 1e-3, "Scaling Z: expected 7.5, got {}", scaled_z);
    }

    #[test]
    fn test_fixed_matrix3x3_operations() {
        let mut matrix = FixedMatrix3x3::identity();
        let vector = FixedVector3::from_f64(5.0, 3.0, 2.0);
        
        // Test identity matrix multiplication
        let result = matrix.multiply_vector(&vector);
        let (x, y, z) = result.to_f64();
        assert!((x - 5.0).abs() < 1e-3, "Identity multiplication X: expected 5.0, got {}", x);
        assert!((y - 3.0).abs() < 1e-3, "Identity multiplication Y: expected 3.0, got {}", y);
        assert!((z - 2.0).abs() < 1e-3, "Identity multiplication Z: expected 2.0, got {}", z);
        
        // Test diagonal addition
        let diagonal_value = (0.5 * FIXED_POINT_SCALE as f64) as i32;
        matrix.add_diagonal(diagonal_value);
        
        let result2 = matrix.multiply_vector(&vector);
        let (x2, y2, z2) = result2.to_f64();
        assert!((x2 - 7.5).abs() < 1e-3, "Modified matrix X: expected 7.5, got {}", x2);
        assert!((y2 - 4.5).abs() < 1e-3, "Modified matrix Y: expected 4.5, got {}", y2);
        assert!((z2 - 3.0).abs() < 1e-3, "Modified matrix Z: expected 3.0, got {}", z2);
    }

    #[test]
    fn test_embedded_anchor_data_management() {
        let mut anchor_data = EmbeddedAnchorData::new();
        
        // Test initial state
        assert_eq!(anchor_data.count, 0);
        
        // Test adding anchors
        let positions = vec![
            FixedVector3::from_f64(100.0, 0.0, 0.0),
            FixedVector3::from_f64(0.0, 100.0, 0.0),
            FixedVector3::from_f64(-100.0, 0.0, 0.0),
            FixedVector3::from_f64(0.0, -100.0, 0.0),
        ];
        
        for (i, pos) in positions.iter().enumerate() {
            let range = ((i + 1) as f64 * 150.0 * FIXED_POINT_SCALE as f64) as i32;
            let weight = FIXED_POINT_SCALE; // Weight = 1.0
            
            let result = anchor_data.add_anchor(*pos, range, weight);
            assert!(result.is_ok(), "Should be able to add anchor {}", i);
            assert_eq!(anchor_data.count, i + 1);
        }
        
        // Test maximum capacity
        for i in 4..MAX_ANCHORS_EMBEDDED {
            let pos = FixedVector3::from_f64(i as f64, i as f64, 0.0);
            let range = (100.0 * FIXED_POINT_SCALE as f64) as i32;
            let weight = FIXED_POINT_SCALE;
            
            let result = anchor_data.add_anchor(pos, range, weight);
            assert!(result.is_ok(), "Should be able to add anchor {} within capacity", i);
        }
        
        // Test exceeding capacity
        let overflow_pos = FixedVector3::from_f64(999.0, 999.0, 0.0);
        let overflow_result = anchor_data.add_anchor(overflow_pos, 1000, FIXED_POINT_SCALE);
        assert!(overflow_result.is_err(), "Should fail when exceeding maximum capacity");
        assert!(overflow_result.unwrap_err().contains("Maximum anchor count exceeded"));
    }

    #[test]
    fn test_embedded_trilateration_floating_point() {
        let trilateration = EmbeddedTrilateration::new();
        let anchors = EmbeddedTestUtils::create_embedded_test_anchors();
        
        let result = trilateration.calculate_position_embedded(&anchors, 1005);
        
        assert!(result.is_ok(), "Floating-point embedded trilateration should succeed");
        let position = result.unwrap();
        
        // Position should be reasonable
        assert!(position.lat.abs() < 0.01, "Latitude should be reasonable: {}", position.lat);
        assert!(position.lon.abs() < 0.01, "Longitude should be reasonable: {}", position.lon);
        assert!(position.depth >= 0.0 && position.depth <= 100.0, "Depth should be reasonable: {}", position.depth);
    }

    #[test]
    fn test_embedded_trilateration_fixed_point() {
        let trilateration = EmbeddedTrilateration::with_fixed_point();
        let anchors = EmbeddedTestUtils::create_embedded_test_anchors();
        
        let result = trilateration.calculate_position_embedded(&anchors, 1005);
        
        assert!(result.is_ok(), "Fixed-point embedded trilateration should succeed");
        let position = result.unwrap();
        
        // Position should be reasonable (may have slightly lower precision)
        assert!(position.lat.abs() < 0.01, "Fixed-point latitude should be reasonable: {}", position.lat);
        assert!(position.lon.abs() < 0.01, "Fixed-point longitude should be reasonable: {}", position.lon);
        assert!(position.depth >= 0.0 && position.depth <= 100.0, "Fixed-point depth should be reasonable: {}", position.depth);
    }

    #[test]
    fn test_embedded_trilateration_performance_modes() {
        let anchors = EmbeddedTestUtils::create_embedded_test_anchors();
        let receiver_time = 1005;
        
        // Test different performance configurations
        let configurations = vec![
            ("fast", EmbeddedTrilateration::with_fast_mode()),
            ("balanced", config::BALANCED),
            ("precision", config::HIGH_PRECISION),
        ];
        
        for (name, trilateration) in configurations {
            let result = trilateration.calculate_position_embedded(&anchors, receiver_time);
            
            assert!(result.is_ok(), "{} mode should succeed", name);
            let position = result.unwrap();
            
            // All modes should produce reasonable results
            assert!(position.lat.abs() < 0.01, "{} mode latitude: {}", name, position.lat);
            assert!(position.lon.abs() < 0.01, "{} mode longitude: {}", name, position.lon);
            assert!(position.depth >= 0.0 && position.depth <= 100.0, "{} mode depth: {}", name, position.depth);
        }
    }

    #[test]
    fn test_embedded_trilateration_max_anchors() {
        let trilateration = EmbeddedTrilateration::new();
        let anchors = EmbeddedTestUtils::create_max_embedded_anchors();
        
        assert_eq!(anchors.len(), MAX_ANCHORS_EMBEDDED);
        
        let result = trilateration.calculate_position_embedded(&anchors, 1010);
        
        assert!(result.is_ok(), "Should handle maximum number of anchors");
        let position = result.unwrap();
        
        // Position should be at center of anchor configuration
        assert!(position.lat.abs() < 0.001, "Max anchors latitude: {}", position.lat);
        assert!(position.lon.abs() < 0.001, "Max anchors longitude: {}", position.lon);
    }

    #[test]
    fn test_embedded_trilateration_insufficient_anchors() {
        let trilateration = EmbeddedTrilateration::new();
        let anchors = vec![
            Anchor {
                id: "A1".to_string(),
                timestamp: 1000,
                position: Position { lat: 0.0, lon: 0.0, depth: 0.0 },
            },
            Anchor {
                id: "A2".to_string(),
                timestamp: 1001,
                position: Position { lat: 0.001, lon: 0.0, depth: 0.0 },
            },
        ];
        
        let result = trilateration.calculate_position_embedded(&anchors, 1005);
        
        assert!(result.is_err(), "Should fail with insufficient anchors");
        assert!(result.unwrap_err().contains("At least 3 anchors required"));
    }

    #[test]
    fn test_embedded_trilateration_too_many_anchors() {
        let trilateration = EmbeddedTrilateration::new();
        let mut anchors = EmbeddedTestUtils::create_max_embedded_anchors();
        
        // Add one more anchor to exceed capacity
        anchors.push(Anchor {
            id: "OVERFLOW".to_string(),
            timestamp: 2000,
            position: Position { lat: 0.01, lon: 0.01, depth: 200.0 },
        });
        
        let result = trilateration.calculate_position_embedded(&anchors, 1010);
        
        assert!(result.is_err(), "Should fail with too many anchors");
        assert!(result.unwrap_err().contains("Too many anchors for embedded mode"));
    }

    #[test]
    fn test_fixed_coordinate_operations() {
        // Test fixed-point coordinate operations
        let coord1 = FixedCoordinate::new(100.5, -50.25, 25.125);
        let coord2 = FixedCoordinate::new(200.75, 30.5, -10.25);
        
        // Test conversion accuracy
        let (x1, y1, z1) = coord1.to_f64();
        assert!((x1 - 100.5).abs() < 1e-5, "Coordinate X conversion: expected 100.5, got {}", x1);
        assert!((y1 - (-50.25)).abs() < 1e-5, "Coordinate Y conversion: expected -50.25, got {}", y1);
        assert!((z1 - 25.125).abs() < 1e-5, "Coordinate Z conversion: expected 25.125, got {}", z1);
        
        // Test distance calculation
        let distance = coord1.distance_to(&coord2);
        let distance_f64 = distance as f64 / COORD_FIXED_POINT_SCALE as f64;
        
        // Calculate expected distance
        let dx = 200.75 - 100.5;
        let dy = 30.5 - (-50.25);
        let dz = -10.25 - 25.125;
        let expected_distance = ((dx * dx + dy * dy + dz * dz) as f64).sqrt();
        
        assert!((distance_f64 - expected_distance).abs() < 0.1,
                "Distance calculation: expected {}, got {}", expected_distance, distance_f64);
    }

    #[test]
    fn test_embedded_coordinate_manager() {
        let mut manager = EmbeddedCoordinateManager::new();
        
        let reference = Position { lat: 37.7749, lon: -122.4194, depth: 0.0 };
        let result = manager.set_reference_point(reference, 1000);
        
        assert!(result.is_ok(), "Should be able to set reference point");
        assert!(manager.get_reference_point().is_some(), "Reference point should be available");
        
        // Test coordinate transformation
        let test_position = Position { lat: 37.7759, lon: -122.4184, depth: 50.0 };
        let local_result = manager.geodetic_to_local(&test_position);
        
        assert!(local_result.is_ok(), "Geodetic to local conversion should succeed");
        let (x, y, z) = local_result.unwrap();
        
        // Should be approximately 1km north and 1km east
        assert!(y > 900.0 && y < 1100.0, "North component should be ~1000m: {}", y);
        assert!(x > 900.0 && x < 1100.0, "East component should be ~1000m: {}", x);
        assert!((z - 50.0).abs() < 1.0, "Depth component should be ~50m: {}", z);
        
        // Test round-trip conversion
        let back_to_geodetic = manager.local_to_geodetic((x, y, z));
        assert!(back_to_geodetic.is_ok(), "Local to geodetic conversion should succeed");
        
        let converted_back = back_to_geodetic.unwrap();
        assert!((converted_back.lat - test_position.lat).abs() < 1e-5, "Round-trip latitude error");
        assert!((converted_back.lon - test_position.lon).abs() < 1e-5, "Round-trip longitude error");
        assert!((converted_back.depth - test_position.depth).abs() < 0.1, "Round-trip depth error");
    }

    #[test]
    fn test_embedded_coordinate_manager_fixed_point() {
        let mut manager = EmbeddedCoordinateManager::with_fixed_point();
        
        let reference = Position { lat: 0.0, lon: 0.0, depth: 0.0 };
        manager.set_reference_point(reference, 1000).unwrap();
        
        let test_position = Position { lat: 0.001, lon: 0.001, depth: 10.0 };
        let local_result = manager.geodetic_to_local(&test_position);
        
        assert!(local_result.is_ok(), "Fixed-point coordinate conversion should succeed");
        let (x, y, z) = local_result.unwrap();
        
        // Should be approximately 111m in each direction
        assert!(x > 100.0 && x < 120.0, "Fixed-point East component: {}", x);
        assert!(y > 100.0 && y < 120.0, "Fixed-point North component: {}", y);
        assert!((z - 10.0).abs() < 1.0, "Fixed-point depth component: {}", z);
    }

    #[test]
    fn test_coordinate_system_validation() {
        // Test WGS84 validation
        let valid_position = Position { lat: 45.0, lon: -122.0, depth: 100.0 };
        assert!(CoordinateValidator::validate_wgs84(&valid_position).is_ok());
        
        let invalid_lat = Position { lat: 91.0, lon: -122.0, depth: 100.0 };
        assert!(CoordinateValidator::validate_wgs84(&invalid_lat).is_err());
        
        let invalid_lon = Position { lat: 45.0, lon: 181.0, depth: 100.0 };
        assert!(CoordinateValidator::validate_wgs84(&invalid_lon).is_err());
        
        let invalid_depth = Position { lat: 45.0, lon: -122.0, depth: -12000.0 };
        assert!(CoordinateValidator::validate_wgs84(&invalid_depth).is_err());
        
        // Test UTM validation
        assert!(CoordinateValidator::validate_utm(500000.0, 5000000.0, 10).is_ok());
        assert!(CoordinateValidator::validate_utm(50000.0, 5000000.0, 10).is_err()); // Invalid easting
        assert!(CoordinateValidator::validate_utm(500000.0, 5000000.0, 61).is_err()); // Invalid zone
        
        // Test local coordinate validation
        assert!(CoordinateValidator::validate_local((100.0, 200.0, 50.0), 1000.0).is_ok());
        assert!(CoordinateValidator::validate_local((2000.0, 200.0, 50.0), 1000.0).is_err()); // Outside radius
    }

    #[test]
    fn test_memory_usage_constraints() {
        // Test that key structures fit within embedded memory constraints
        let trilateration_size = EmbeddedTestUtils::estimate_memory_usage::<EmbeddedTrilateration>();
        let anchor_data_size = EmbeddedTestUtils::estimate_memory_usage::<EmbeddedAnchorData>();
        let coord_manager_size = EmbeddedTestUtils::estimate_memory_usage::<EmbeddedCoordinateManager>();
        
        // These should be reasonable for embedded systems
        assert!(trilateration_size < 1024, "EmbeddedTrilateration should be < 1KB: {} bytes", trilateration_size);
        assert!(anchor_data_size < 2048, "EmbeddedAnchorData should be < 2KB: {} bytes", anchor_data_size);
        assert!(coord_manager_size < 4096, "EmbeddedCoordinateManager should be < 4KB: {} bytes", coord_manager_size);
        
        // Test stack-based array sizes
        let fixed_vector_size = EmbeddedTestUtils::estimate_memory_usage::<FixedVector3>();
        let fixed_matrix_size = EmbeddedTestUtils::estimate_memory_usage::<FixedMatrix3x3>();
        
        assert!(fixed_vector_size <= 16, "FixedVector3 should be <= 16 bytes: {} bytes", fixed_vector_size);
        assert!(fixed_matrix_size <= 64, "FixedMatrix3x3 should be <= 64 bytes: {} bytes", fixed_matrix_size);
    }

    #[test]
    fn test_real_time_performance_constraints() {
        let trilateration = EmbeddedTrilateration::new();
        let anchors = EmbeddedTestUtils::create_embedded_test_anchors();
        
        // Test that trilateration completes within 100ms constraint
        let is_fast_enough = EmbeddedTestUtils::validate_timing_constraint(|| {
            let _ = trilateration.calculate_position_embedded(&anchors, 1005);
        }, 100);
        
        assert!(is_fast_enough, "Embedded trilateration should complete within 100ms");
        
        // Test coordinate transformations are fast enough
        let mut coord_manager = EmbeddedCoordinateManager::new();
        let reference = Position { lat: 0.0, lon: 0.0, depth: 0.0 };
        coord_manager.set_reference_point(reference, 1000).unwrap();
        
        let test_position = Position { lat: 0.001, lon: 0.001, depth: 10.0 };
        
        let coord_is_fast_enough = EmbeddedTestUtils::validate_timing_constraint(|| {
            let _ = coord_manager.geodetic_to_local(&test_position);
        }, 10); // Coordinate transformations should be very fast
        
        assert!(coord_is_fast_enough, "Coordinate transformations should complete within 10ms");
    }

    #[test]
    fn test_fixed_point_precision_vs_floating_point() {
        let anchors = EmbeddedTestUtils::create_embedded_test_anchors();
        let receiver_time = 1005;
        
        // Compare fixed-point vs floating-point results
        let floating_point = EmbeddedTrilateration::new();
        let fixed_point = EmbeddedTrilateration::with_fixed_point();
        
        let fp_result = floating_point.calculate_position_embedded(&anchors, receiver_time);
        let fixed_result = fixed_point.calculate_position_embedded(&anchors, receiver_time);
        
        assert!(fp_result.is_ok() && fixed_result.is_ok(), "Both algorithms should succeed");
        
        let fp_pos = fp_result.unwrap();
        let fixed_pos = fixed_result.unwrap();
        
        // Fixed-point should be reasonably close to floating-point
        let lat_diff = (fp_pos.lat - fixed_pos.lat).abs() * 111320.0; // Convert to meters
        let lon_diff = (fp_pos.lon - fixed_pos.lon).abs() * 111320.0;
        let depth_diff = (fp_pos.depth - fixed_pos.depth).abs();
        
        assert!(lat_diff < 1.0, "Fixed-point latitude should be within 1m of floating-point: {} m", lat_diff);
        assert!(lon_diff < 1.0, "Fixed-point longitude should be within 1m of floating-point: {} m", lon_diff);
        assert!(depth_diff < 1.0, "Fixed-point depth should be within 1m of floating-point: {} m", depth_diff);
    }

    #[test]
    fn test_embedded_error_handling() {
        let trilateration = EmbeddedTrilateration::new();
        
        // Test invalid timing
        let anchors = EmbeddedTestUtils::create_embedded_test_anchors();
        let invalid_time = 500; // Earlier than anchor timestamps
        
        let result = trilateration.calculate_position_embedded(&anchors, invalid_time);
        assert!(result.is_err(), "Should handle invalid timing");
        assert!(result.unwrap_err().contains("Receiver time earlier than anchor time"));
        
        // Test empty anchor list
        let empty_anchors = vec![];
        let result2 = trilateration.calculate_position_embedded(&empty_anchors, 1000);
        assert!(result2.is_err(), "Should handle empty anchor list");
        assert!(result2.unwrap_err().contains("At least 3 anchors required"));
    }

    #[test]
    fn test_embedded_configuration_modes() {
        let anchors = EmbeddedTestUtils::create_embedded_test_anchors();
        let receiver_time = 1005;
        
        // Test ultra-fast mode
        let ultra_fast = config::ULTRA_FAST;
        let result = ultra_fast.calculate_position_embedded(&anchors, receiver_time);
        assert!(result.is_ok(), "Ultra-fast mode should work");
        
        // Test that ultra-fast mode is indeed faster (fewer iterations)
        assert_eq!(ultra_fast.max_iterations, 5, "Ultra-fast should have minimal iterations");
        assert!(ultra_fast.use_fixed_point, "Ultra-fast should use fixed-point");
        
        // Test precision mode
        let precision = config::HIGH_PRECISION;
        let precision_result = precision.calculate_position_embedded(&anchors, receiver_time);
        assert!(precision_result.is_ok(), "High-precision mode should work");
        
        // Precision mode should have more iterations
        assert_eq!(precision.max_iterations, 50, "High-precision should have more iterations");
        assert!(!precision.use_fixed_point, "High-precision should use floating-point");
    }
}