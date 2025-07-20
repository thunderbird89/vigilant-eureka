//! Unit tests for high-precision coordinate transformations and error propagation
//! 
//! Tests cover:
//! - Coordinate system transformation accuracy
//! - Error propagation calculations
//! - Environmental corrections (sound speed, temperature, etc.)
//! - Precision validation for sub-meter accuracy requirements

use crate::algorithms::precision::{HighPrecisionTransformer, EnvironmentalCorrections, PositionUncertainty};
use crate::core::Position;
use nalgebra::Vector3;
use std::f64::consts::PI;

/// Test utilities for precision testing
pub struct PrecisionTestUtils;

impl PrecisionTestUtils {
    /// Create test positions at various scales
    pub fn create_test_positions() -> Vec<Position> {
        vec![
            // Origin
            Position { lat: 0.0, lon: 0.0, depth: 0.0 },
            // Local scale (100m)
            Position { lat: 0.001, lon: 0.001, depth: 10.0 },
            // Regional scale (10km)
            Position { lat: 0.1, lon: 0.1, depth: 100.0 },
            // Global scale (1000km)
            Position { lat: 10.0, lon: 10.0, depth: 1000.0 },
            // Extreme coordinates
            Position { lat: 89.0, lon: 179.0, depth: 5000.0 },
            Position { lat: -89.0, lon: -179.0, depth: 10000.0 },
        ]
    }

    /// Create reference positions for transformation testing
    pub fn create_reference_positions() -> Vec<Position> {
        vec![
            // Equator, Prime Meridian
            Position { lat: 0.0, lon: 0.0, depth: 0.0 },
            // Mid-latitude
            Position { lat: 45.0, lon: 0.0, depth: 0.0 },
            // High latitude
            Position { lat: 80.0, lon: 0.0, depth: 0.0 },
            // Various longitudes
            Position { lat: 0.0, lon: 90.0, depth: 0.0 },
            Position { lat: 0.0, lon: 180.0, depth: 0.0 },
            Position { lat: 0.0, lon: -90.0, depth: 0.0 },
        ]
    }

    /// Create environmental test conditions
    pub fn create_environmental_conditions() -> Vec<EnvironmentalCorrections> {
        vec![
            // Standard conditions
            EnvironmentalCorrections {
                base_sound_speed: 1500.0,
                temperature: 15.0,
                salinity: 35.0,
                depth: 0.0,
                pressure: 10.0,
            },
            // Cold water
            EnvironmentalCorrections {
                base_sound_speed: 1500.0,
                temperature: 4.0,
                salinity: 35.0,
                depth: 100.0,
                pressure: 20.0,
            },
            // Warm water
            EnvironmentalCorrections {
                base_sound_speed: 1500.0,
                temperature: 25.0,
                salinity: 35.0,
                depth: 10.0,
                pressure: 11.0,
            },
            // Deep water
            EnvironmentalCorrections {
                base_sound_speed: 1500.0,
                temperature: 2.0,
                salinity: 35.0,
                depth: 4000.0,
                pressure: 410.0,
            },
            // Low salinity (fresh water)
            EnvironmentalCorrections {
                base_sound_speed: 1500.0,
                temperature: 20.0,
                salinity: 0.0,
                depth: 50.0,
                pressure: 15.0,
            },
        ]
    }

    /// Calculate expected distance between two positions (Haversine formula)
    pub fn haversine_distance(pos1: &Position, pos2: &Position) -> f64 {
        const R: f64 = 6371000.0; // Earth radius in meters
        
        let lat1_rad = pos1.lat.to_radians();
        let lat2_rad = pos2.lat.to_radians();
        let delta_lat = (pos2.lat - pos1.lat).to_radians();
        let delta_lon = (pos2.lon - pos1.lon).to_radians();
        
        let a = (delta_lat / 2.0).sin().powi(2) + 
                lat1_rad.cos() * lat2_rad.cos() * (delta_lon / 2.0).sin().powi(2);
        let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
        
        let horizontal_distance = R * c;
        let vertical_distance = pos2.depth - pos1.depth;
        
        (horizontal_distance.powi(2) + vertical_distance.powi(2)).sqrt()
    }

    /// Validate transformation precision
    pub fn validate_transformation_precision(
        original: &Position,
        transformed_back: &Position,
        max_error_m: f64,
    ) -> bool {
        let distance_error = Self::haversine_distance(original, transformed_back);
        distance_error <= max_error_m
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_geodetic_to_ecef_conversion_accuracy() {
        let transformer = HighPrecisionTransformer::new();
        let test_positions = PrecisionTestUtils::create_test_positions();
        
        for (i, position) in test_positions.iter().enumerate() {
            let ecef = transformer.geodetic_to_ecef(position);
            let back_to_geodetic = transformer.ecef_to_geodetic(&ecef);
            
            // Validate round-trip conversion accuracy
            let is_precise = PrecisionTestUtils::validate_transformation_precision(
                position, &back_to_geodetic, 0.001 // 1mm precision
            );
            
            assert!(is_precise, 
                    "Round-trip geodetic<->ECEF conversion failed for position {}: original={:?}, converted={:?}",
                    i, position, back_to_geodetic);
        }
    }

    #[test]
    fn test_geodetic_to_enu_conversion_accuracy() {
        let mut transformer = HighPrecisionTransformer::new();
        let test_positions = PrecisionTestUtils::create_test_positions();
        let reference_positions = PrecisionTestUtils::create_reference_positions();
        
        for (ref_idx, reference) in reference_positions.iter().enumerate() {
            for (pos_idx, position) in test_positions.iter().enumerate() {
                // Skip if position is same as reference
                if position.lat == reference.lat && position.lon == reference.lon && position.depth == reference.depth {
                    continue;
                }
                
                let enu = transformer.geodetic_to_enu(position, reference);
                let back_to_geodetic = transformer.enu_to_geodetic(&enu, reference);
                
                // Validate round-trip conversion accuracy
                let is_precise = PrecisionTestUtils::validate_transformation_precision(
                    position, &back_to_geodetic, 0.01 // 1cm precision for local transformations
                );
                
                assert!(is_precise, 
                        "Round-trip geodetic<->ENU conversion failed for ref {} pos {}: original={:?}, converted={:?}",
                        ref_idx, pos_idx, position, back_to_geodetic);
            }
        }
    }

    #[test]
    fn test_enu_coordinate_system_properties() {
        let mut transformer = HighPrecisionTransformer::new();
        let reference = Position { lat: 45.0, lon: 0.0, depth: 0.0 };
        
        // Test cardinal directions
        let test_cases = vec![
            // 1km North
            (Position { lat: 45.009, lon: 0.0, depth: 0.0 }, (0.0, 1000.0, 0.0)),
            // 1km East  
            (Position { lat: 45.0, lon: 0.0127, depth: 0.0 }, (1000.0, 0.0, 0.0)),
            // 100m Down (positive depth)
            (Position { lat: 45.0, lon: 0.0, depth: 100.0 }, (0.0, 0.0, -100.0)),
        ];
        
        for (position, expected_enu) in test_cases {
            let enu = transformer.geodetic_to_enu(&position, &reference);
            
            // Allow 1% tolerance for coordinate system approximations
            let tolerance = 10.0; // 10m tolerance
            assert!((enu.x - expected_enu.0).abs() < tolerance,
                    "ENU East component: expected {}, got {}", expected_enu.0, enu.x);
            assert!((enu.y - expected_enu.1).abs() < tolerance,
                    "ENU North component: expected {}, got {}", expected_enu.1, enu.y);
            assert!((enu.z - expected_enu.2).abs() < tolerance,
                    "ENU Up component: expected {}, got {}", expected_enu.2, enu.z);
        }
    }

    #[test]
    fn test_sound_speed_calculation_accuracy() {
        let transformer = HighPrecisionTransformer::new();
        let environmental_conditions = PrecisionTestUtils::create_environmental_conditions();
        
        for (i, env) in environmental_conditions.iter().enumerate() {
            let sound_speed = transformer.calculate_sound_speed(env);
            
            // Sound speed should be within reasonable range for water
            assert!(sound_speed >= 1400.0 && sound_speed <= 1600.0,
                    "Sound speed {} should be in range [1400, 1600] m/s for condition {}", 
                    sound_speed, i);
            
            // Test specific known conditions
            match i {
                0 => { // Standard conditions
                    assert!((sound_speed - 1500.0).abs() < 50.0,
                            "Standard conditions should give ~1500 m/s, got {}", sound_speed);
                },
                1 => { // Cold water
                    assert!(sound_speed < 1500.0,
                            "Cold water should have lower sound speed, got {}", sound_speed);
                },
                2 => { // Warm water
                    assert!(sound_speed > 1500.0,
                            "Warm water should have higher sound speed, got {}", sound_speed);
                },
                3 => { // Deep water (high pressure)
                    assert!(sound_speed > 1500.0,
                            "Deep water (high pressure) should have higher sound speed, got {}", sound_speed);
                },
                4 => { // Fresh water (low salinity)
                    assert!(sound_speed < 1500.0,
                            "Fresh water should have lower sound speed, got {}", sound_speed);
                },
                _ => {}
            }
        }
    }

    #[test]
    fn test_environmental_sound_speed_corrections() {
        let transformer = HighPrecisionTransformer::new();
        
        // Test temperature effect
        let base_env = EnvironmentalCorrections {
            base_sound_speed: 1500.0,
            temperature: 15.0,
            salinity: 35.0,
            depth: 0.0,
            pressure: 10.0,
        };
        
        let warm_env = EnvironmentalCorrections {
            temperature: 25.0, // 10°C warmer
            ..base_env
        };
        
        let cold_env = EnvironmentalCorrections {
            temperature: 5.0, // 10°C colder
            ..base_env
        };
        
        let base_speed = transformer.calculate_sound_speed(&base_env);
        let warm_speed = transformer.calculate_sound_speed(&warm_env);
        let cold_speed = transformer.calculate_sound_speed(&cold_env);
        
        // Temperature should have predictable effect on sound speed
        assert!(warm_speed > base_speed, 
                "Warmer water should have higher sound speed: {} vs {}", warm_speed, base_speed);
        assert!(cold_speed < base_speed, 
                "Colder water should have lower sound speed: {} vs {}", cold_speed, base_speed);
        
        // Temperature coefficient should be approximately 4 m/s per °C
        let temp_coefficient = (warm_speed - cold_speed) / 20.0; // 20°C difference
        assert!(temp_coefficient > 2.0 && temp_coefficient < 6.0,
                "Temperature coefficient should be ~4 m/s/°C, got {}", temp_coefficient);
    }

    #[test]
    fn test_range_correction_with_sound_speed() {
        let transformer = HighPrecisionTransformer::new();
        
        let nominal_speed = 1500.0;
        let actual_speeds = vec![1450.0, 1480.0, 1520.0, 1550.0];
        let test_range = 1000.0; // 1km range
        
        for actual_speed in actual_speeds {
            let corrected_range = transformer.calculate_range_correction(
                test_range, nominal_speed, actual_speed
            );
            
            let expected_correction = test_range * (actual_speed / nominal_speed);
            assert!((corrected_range - expected_correction).abs() < 0.001,
                    "Range correction failed: expected {}, got {}", expected_correction, corrected_range);
            
            // Verify correction direction
            if actual_speed > nominal_speed {
                assert!(corrected_range > test_range,
                        "Higher sound speed should increase corrected range");
            } else if actual_speed < nominal_speed {
                assert!(corrected_range < test_range,
                        "Lower sound speed should decrease corrected range");
            }
        }
    }

    #[test]
    fn test_position_uncertainty_calculation() {
        let transformer = HighPrecisionTransformer::new();
        
        // Test with good anchor geometry (square)
        let position = Vector3::new(0.0, 0.0, 0.0);
        let anchor_positions = vec![
            Vector3::new(100.0, 100.0, 0.0),
            Vector3::new(-100.0, 100.0, 0.0),
            Vector3::new(-100.0, -100.0, 0.0),
            Vector3::new(100.0, -100.0, 0.0),
        ];
        let range_std_devs = vec![1.0, 1.0, 1.0, 1.0]; // 1m standard deviation
        
        let uncertainty = transformer.calculate_position_uncertainty(
            &position, &anchor_positions, &range_std_devs
        );
        
        // Uncertainty should be reasonable for good geometry
        assert!(uncertainty.std_dev_x > 0.0 && uncertainty.std_dev_x < 3.0,
                "X standard deviation should be reasonable: {}", uncertainty.std_dev_x);
        assert!(uncertainty.std_dev_y > 0.0 && uncertainty.std_dev_y < 3.0,
                "Y standard deviation should be reasonable: {}", uncertainty.std_dev_y);
        assert!(uncertainty.std_dev_z > 0.0 && uncertainty.std_dev_z < 3.0,
                "Z standard deviation should be reasonable: {}", uncertainty.std_dev_z);
        
        // 95% confidence radius should be larger than individual standard deviations
        assert!(uncertainty.confidence_radius_95 > uncertainty.std_dev_x,
                "95% confidence radius should be larger than X std dev");
        assert!(uncertainty.confidence_radius_95 > uncertainty.std_dev_y,
                "95% confidence radius should be larger than Y std dev");
        
        // Covariance matrix should be positive definite (diagonal elements positive)
        assert!(uncertainty.covariance[(0, 0)] > 0.0, "Covariance matrix should be positive definite");
        assert!(uncertainty.covariance[(1, 1)] > 0.0, "Covariance matrix should be positive definite");
        assert!(uncertainty.covariance[(2, 2)] > 0.0, "Covariance matrix should be positive definite");
    }

    #[test]
    fn test_position_uncertainty_with_poor_geometry() {
        let transformer = HighPrecisionTransformer::new();
        
        // Test with poor anchor geometry (collinear)
        let position = Vector3::new(0.0, 0.0, 0.0);
        let anchor_positions = vec![
            Vector3::new(-100.0, 0.0, 0.0),
            Vector3::new(-50.0, 0.0, 0.0),
            Vector3::new(50.0, 0.0, 0.0),
            Vector3::new(100.0, 0.0, 0.0),
        ];
        let range_std_devs = vec![1.0, 1.0, 1.0, 1.0];
        
        let uncertainty = transformer.calculate_position_uncertainty(
            &position, &anchor_positions, &range_std_devs
        );
        
        // Poor geometry should result in higher uncertainty
        // Y uncertainty should be much higher than X uncertainty for collinear anchors along X-axis
        assert!(uncertainty.std_dev_y > uncertainty.std_dev_x,
                "Y uncertainty ({}) should be higher than X uncertainty ({}) for collinear geometry",
                uncertainty.std_dev_y, uncertainty.std_dev_x);
        
        // Overall uncertainty should be higher than good geometry case
        assert!(uncertainty.confidence_radius_95 > 5.0,
                "Poor geometry should result in higher uncertainty: {}", uncertainty.confidence_radius_95);
    }

    #[test]
    fn test_error_propagation_from_ranges() {
        let transformer = HighPrecisionTransformer::new();
        
        let position = Vector3::new(0.0, 0.0, 0.0);
        let anchor_positions = vec![
            Vector3::new(100.0, 0.0, 0.0),
            Vector3::new(0.0, 100.0, 0.0),
            Vector3::new(-100.0, 0.0, 0.0),
            Vector3::new(0.0, -100.0, 0.0),
        ];
        
        // Test different error magnitudes
        let error_magnitudes = vec![0.1, 0.5, 1.0, 2.0];
        
        for error_mag in error_magnitudes {
            let range_errors = vec![error_mag, error_mag, error_mag, error_mag];
            
            let position_error = transformer.propagate_range_error_to_position(
                &position, &anchor_positions, &range_errors
            );
            
            // Position error should be proportional to range error
            let position_error_magnitude = position_error.norm();
            assert!(position_error_magnitude > 0.0,
                    "Position error should be non-zero for range error {}", error_mag);
            assert!(position_error_magnitude < error_mag * 2.0,
                    "Position error {} should be reasonable for range error {}", 
                    position_error_magnitude, error_mag);
        }
    }

    #[test]
    fn test_gdop_calculation_precision() {
        let transformer = HighPrecisionTransformer::new();
        
        // Test GDOP calculation for different geometries
        let test_cases = vec![
            // Good geometry (regular tetrahedron)
            (vec![
                Vector3::new(100.0, 100.0, 100.0),
                Vector3::new(100.0, -100.0, -100.0),
                Vector3::new(-100.0, 100.0, -100.0),
                Vector3::new(-100.0, -100.0, 100.0),
            ], 1.0, 5.0), // Expected GDOP range
            
            // Poor geometry (coplanar)
            (vec![
                Vector3::new(100.0, 0.0, 0.0),
                Vector3::new(0.0, 100.0, 0.0),
                Vector3::new(-100.0, 0.0, 0.0),
                Vector3::new(0.0, -100.0, 0.0),
            ], 5.0, 50.0), // Expected GDOP range
        ];
        
        for (i, (anchor_positions, min_gdop, max_gdop)) in test_cases.iter().enumerate() {
            let position = Vector3::new(0.0, 0.0, 0.0);
            let gdop = transformer.calculate_gdop(&position, anchor_positions);
            
            assert!(gdop >= *min_gdop && gdop <= *max_gdop,
                    "GDOP {} should be in range [{}, {}] for test case {}", 
                    gdop, min_gdop, max_gdop, i);
            assert!(gdop.is_finite(), "GDOP should be finite");
        }
    }

    #[test]
    fn test_tidal_correction() {
        let transformer = HighPrecisionTransformer::new();
        
        let base_depth = 50.0;
        let tidal_heights = vec![-2.0, -1.0, 0.0, 1.0, 2.0]; // meters
        
        for tidal_height in tidal_heights {
            let corrected_depth = transformer.apply_tidal_correction(base_depth, tidal_height);
            let expected_depth = base_depth - tidal_height;
            
            assert!((corrected_depth - expected_depth).abs() < 1e-10,
                    "Tidal correction failed: depth={}, tidal_height={}, expected={}, got={}",
                    base_depth, tidal_height, expected_depth, corrected_depth);
        }
    }

    #[test]
    fn test_coordinate_transformation_consistency() {
        let mut transformer = HighPrecisionTransformer::new();
        
        // Test consistency across multiple reference points
        let test_position = Position { lat: 37.7749, lon: -122.4194, depth: 100.0 }; // San Francisco
        let reference_positions = vec![
            Position { lat: 37.0, lon: -122.0, depth: 0.0 },
            Position { lat: 38.0, lon: -122.0, depth: 0.0 },
            Position { lat: 37.0, lon: -123.0, depth: 0.0 },
        ];
        
        for (i, reference) in reference_positions.iter().enumerate() {
            // Transform to local coordinates and back
            let enu = transformer.geodetic_to_enu(&test_position, reference);
            let back_to_geodetic = transformer.enu_to_geodetic(&enu, reference);
            
            // Verify consistency
            let is_consistent = PrecisionTestUtils::validate_transformation_precision(
                &test_position, &back_to_geodetic, 0.01 // 1cm precision
            );
            
            assert!(is_consistent,
                    "Transformation consistency failed for reference {}: original={:?}, converted={:?}",
                    i, test_position, back_to_geodetic);
        }
    }

    #[test]
    fn test_precision_at_different_scales() {
        let mut transformer = HighPrecisionTransformer::new();
        let reference = Position { lat: 0.0, lon: 0.0, depth: 0.0 };
        
        // Test precision at different distance scales
        let test_scales = vec![
            (1.0, 0.001),      // 1m, 1mm precision
            (10.0, 0.01),      // 10m, 1cm precision  
            (100.0, 0.1),      // 100m, 10cm precision
            (1000.0, 1.0),     // 1km, 1m precision
            (10000.0, 10.0),   // 10km, 10m precision
        ];
        
        for (distance, max_error) in test_scales {
            let test_position = Position {
                lat: distance / 111320.0, // Convert meters to degrees
                lon: distance / 111320.0,
                depth: distance / 10.0,    // Reasonable depth
            };
            
            let enu = transformer.geodetic_to_enu(&test_position, &reference);
            let back_to_geodetic = transformer.enu_to_geodetic(&enu, &reference);
            
            let is_precise = PrecisionTestUtils::validate_transformation_precision(
                &test_position, &back_to_geodetic, max_error
            );
            
            assert!(is_precise,
                    "Precision test failed at {}m scale: max_error={}m, original={:?}, converted={:?}",
                    distance, max_error, test_position, back_to_geodetic);
        }
    }
}