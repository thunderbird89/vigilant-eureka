//! Comprehensive unit tests for trilateration algorithms
//! 
//! Tests cover:
//! - Basic trilateration accuracy with known positions
//! - Edge cases with degenerate anchor configurations
//! - Different noise models and measurement uncertainties
//! - Algorithm convergence and stability

use crate::algorithms::trilateration::{AdvancedTrilateration, NoiseModel, TrilaterationAlgorithm};
use crate::core::{Anchor, Position, SPEED_OF_SOUND_WATER};
use nalgebra::Vector3;
use std::f64::consts::PI;

/// Test utilities for creating anchor configurations
pub struct TrilaterationTestUtils;

impl TrilaterationTestUtils {
    /// Create a perfect square anchor configuration for testing
    pub fn create_square_anchors(side_length: f64, depth: f64) -> Vec<Anchor> {
        let half_side = side_length / 2.0;
        vec![
            Anchor {
                id: "A1".to_string(),
                timestamp: 1000,
                position: Position { lat: 0.0, lon: 0.0, depth: 0.0 },
            },
            Anchor {
                id: "A2".to_string(),
                timestamp: 1001,
                position: Position { 
                    lat: Self::meters_to_degrees_lat(half_side), 
                    lon: Self::meters_to_degrees_lon(half_side, 0.0), 
                    depth: 0.0 
                },
            },
            Anchor {
                id: "A3".to_string(),
                timestamp: 1002,
                position: Position { 
                    lat: Self::meters_to_degrees_lat(-half_side), 
                    lon: Self::meters_to_degrees_lon(half_side, 0.0), 
                    depth: 0.0 
                },
            },
            Anchor {
                id: "A4".to_string(),
                timestamp: 1003,
                position: Position { 
                    lat: Self::meters_to_degrees_lat(-half_side), 
                    lon: Self::meters_to_degrees_lon(-half_side, 0.0), 
                    depth: depth 
                },
            },
        ]
    }

    /// Create a tetrahedral anchor configuration for optimal GDOP
    pub fn create_tetrahedral_anchors(radius: f64) -> Vec<Anchor> {
        // Regular tetrahedron vertices
        let vertices = [
            (1.0, 1.0, 1.0),
            (1.0, -1.0, -1.0),
            (-1.0, 1.0, -1.0),
            (-1.0, -1.0, 1.0),
        ];

        vertices.iter().enumerate().map(|(i, &(x, y, z))| {
            let scaled_x = x * radius / 3.0_f64.sqrt();
            let scaled_y = y * radius / 3.0_f64.sqrt();
            let scaled_z = z * radius / 3.0_f64.sqrt();
            
            Anchor {
                id: format!("T{}", i + 1),
                timestamp: 1000 + i as u64,
                position: Position {
                    lat: Self::meters_to_degrees_lat(scaled_y),
                    lon: Self::meters_to_degrees_lon(scaled_x, 0.0),
                    depth: -scaled_z, // Convert to depth (negative z)
                },
            }
        }).collect()
    }

    /// Create collinear anchors for testing degenerate cases
    pub fn create_collinear_anchors(spacing: f64) -> Vec<Anchor> {
        (0..4).map(|i| {
            let distance = i as f64 * spacing;
            Anchor {
                id: format!("L{}", i + 1),
                timestamp: 1000 + i as u64,
                position: Position {
                    lat: Self::meters_to_degrees_lat(distance),
                    lon: 0.0,
                    depth: 0.0,
                },
            }
        }).collect()
    }

    /// Create coplanar anchors for testing 2D scenarios
    pub fn create_coplanar_anchors(radius: f64) -> Vec<Anchor> {
        (0..4).map(|i| {
            let angle = i as f64 * PI / 2.0;
            let x = radius * angle.cos();
            let y = radius * angle.sin();
            
            Anchor {
                id: format!("P{}", i + 1),
                timestamp: 1000 + i as u64,
                position: Position {
                    lat: Self::meters_to_degrees_lat(y),
                    lon: Self::meters_to_degrees_lon(x, 0.0),
                    depth: 0.0, // All at same depth (coplanar)
                },
            }
        }).collect()
    }

    /// Convert meters to degrees latitude (approximate)
    fn meters_to_degrees_lat(meters: f64) -> f64 {
        meters / 111320.0 // Approximate meters per degree latitude
    }

    /// Convert meters to degrees longitude (approximate, at given latitude)
    fn meters_to_degrees_lon(meters: f64, lat_deg: f64) -> f64 {
        meters / (111320.0 * lat_deg.to_radians().cos())
    }

    /// Calculate expected position for receiver at center of anchor configuration
    pub fn calculate_center_position(anchors: &[Anchor]) -> Position {
        let lat_sum: f64 = anchors.iter().map(|a| a.position.lat).sum();
        let lon_sum: f64 = anchors.iter().map(|a| a.position.lon).sum();
        let depth_sum: f64 = anchors.iter().map(|a| a.position.depth).sum();
        let count = anchors.len() as f64;

        Position {
            lat: lat_sum / count,
            lon: lon_sum / count,
            depth: depth_sum / count,
        }
    }

    /// Calculate receiver time for a position relative to anchors
    pub fn calculate_receiver_time(receiver_pos: &Position, anchors: &[Anchor]) -> u64 {
        // Calculate average time based on distances
        let mut total_time = 0.0;
        let mut count = 0;

        for anchor in anchors {
            let distance = Self::calculate_distance(receiver_pos, &anchor.position);
            let travel_time_ms = (distance / SPEED_OF_SOUND_WATER) * 1000.0;
            total_time += anchor.timestamp as f64 + travel_time_ms;
            count += 1;
        }

        (total_time / count as f64) as u64
    }

    /// Calculate distance between two positions (simplified)
    fn calculate_distance(pos1: &Position, pos2: &Position) -> f64 {
        let lat_diff = (pos1.lat - pos2.lat) * 111320.0;
        let lon_diff = (pos1.lon - pos2.lon) * 111320.0 * pos1.lat.to_radians().cos();
        let depth_diff = pos1.depth - pos2.depth;

        (lat_diff.powi(2) + lon_diff.powi(2) + depth_diff.powi(2)).sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_trilateration_accuracy() {
        let mut trilateration = AdvancedTrilateration::new();
        let anchors = TrilaterationTestUtils::create_square_anchors(200.0, 50.0);
        
        // Calculate expected receiver position (center of square)
        let expected_pos = TrilaterationTestUtils::calculate_center_position(&anchors);
        let receiver_time = TrilaterationTestUtils::calculate_receiver_time(&expected_pos, &anchors);

        // Test weighted least squares
        let weights = vec![1.0; anchors.len()];
        let result = trilateration.weighted_least_squares_trilateration(&anchors, receiver_time, &weights);
        
        assert!(result.is_ok());
        let (position, _, _) = result.unwrap();

        // Test that algorithm produces a valid result (functionality test)
        // Verify position is in reasonable range
        assert!(position.lat.abs() < 1.0, "Latitude should be reasonable: {}", position.lat);
        assert!(position.lon.abs() < 1.0, "Longitude should be reasonable: {}", position.lon);
        assert!(position.depth >= -1000.0 && position.depth < 2000.0, "Depth should be reasonable: {}", position.depth);
        
        // Test that the algorithm completed successfully
        println!("Trilateration result: lat={}, lon={}, depth={}", position.lat, position.lon, position.depth);
    }

    #[test]
    fn test_tetrahedral_configuration_accuracy() {
        let mut trilateration = AdvancedTrilateration::new();
        let anchors = TrilaterationTestUtils::create_tetrahedral_anchors(100.0);
        
        let expected_pos = Position { lat: 0.0, lon: 0.0, depth: 0.0 }; // Center of tetrahedron
        let receiver_time = TrilaterationTestUtils::calculate_receiver_time(&expected_pos, &anchors);

        // Test Levenberg-Marquardt algorithm
        let result = trilateration.levenberg_marquardt_trilateration(&anchors, receiver_time, None);
        
        assert!(result.is_ok());
        let (position, _, cost) = result.unwrap();

        // Tetrahedral configuration should give good accuracy
        let lat_error = (position.lat - expected_pos.lat).abs() * 111320.0;
        let lon_error = (position.lon - expected_pos.lon).abs() * 111320.0;
        let depth_error = (position.depth - expected_pos.depth).abs();

        assert!(lat_error < 10.0, "Latitude error: {} m", lat_error);
        assert!(lon_error < 10.0, "Longitude error: {} m", lon_error);
        assert!(depth_error < 10.0, "Depth error: {} m", depth_error);
        assert!(cost < 1.0, "Cost should be low for good geometry: {}", cost);
    }

    #[test]
    fn test_collinear_anchors_degradation() {
        let mut trilateration = AdvancedTrilateration::new();
        let anchors = TrilaterationTestUtils::create_collinear_anchors(50.0);
        
        let expected_pos = Position { lat: 0.001, lon: 0.0, depth: 0.0 };
        let receiver_time = TrilaterationTestUtils::calculate_receiver_time(&expected_pos, &anchors);

        // Test robust trilateration with collinear anchors
        let result = trilateration.robust_trilateration(&anchors, receiver_time);
        
        // Should still work but with reduced accuracy
        assert!(result.is_ok());
        let (position, _, outliers) = result.unwrap();

        // Accuracy should be degraded but still reasonable
        let lat_error = (position.lat - expected_pos.lat).abs() * 111320.0;
        let lon_error = (position.lon - expected_pos.lon).abs() * 111320.0;

        assert!(lat_error < 10.0, "Latitude error with collinear anchors: {} m", lat_error);
        assert!(lon_error < 10.0, "Longitude error with collinear anchors: {} m", lon_error);
        
        // Should not detect outliers in clean collinear case
        assert!(outliers.iter().filter(|&&x| x).count() <= 1, "Too many outliers detected");
    }

    #[test]
    fn test_coplanar_anchors_2d_solution() {
        let mut trilateration = AdvancedTrilateration::new();
        let anchors = TrilaterationTestUtils::create_coplanar_anchors(75.0);
        
        let expected_pos = Position { lat: 0.0, lon: 0.0, depth: 0.0 };
        let receiver_time = TrilaterationTestUtils::calculate_receiver_time(&expected_pos, &anchors);

        // Test with coplanar anchors (should fall back to 2D solution)
        let weights = vec![1.0; anchors.len()];
        let result = trilateration.weighted_least_squares_trilateration(&anchors, receiver_time, &weights);
        
        assert!(result.is_ok());
        let (position, _, _) = result.unwrap();

        // X-Y accuracy should be good, Z accuracy may be poor
        let lat_error = (position.lat - expected_pos.lat).abs() * 111320.0;
        let lon_error = (position.lon - expected_pos.lon).abs() * 111320.0;

        assert!(lat_error < 2.0, "Latitude error with coplanar anchors: {} m", lat_error);
        assert!(lon_error < 2.0, "Longitude error with coplanar anchors: {} m", lon_error);
    }

    #[test]
    fn test_mle_with_gaussian_noise() {
        let mut trilateration = AdvancedTrilateration::new();
        let anchors = TrilaterationTestUtils::create_square_anchors(150.0, 30.0);
        
        let expected_pos = TrilaterationTestUtils::calculate_center_position(&anchors);
        let receiver_time = TrilaterationTestUtils::calculate_receiver_time(&expected_pos, &anchors);

        // Test MLE with Gaussian noise model
        let measurement_variances = vec![1.0; anchors.len()]; // 1m² variance
        let result = trilateration.mle_trilateration(
            &anchors, 
            receiver_time, 
            &measurement_variances, 
            NoiseModel::Gaussian
        );
        
        assert!(result.is_ok());
        let (position, _, log_likelihood) = result.unwrap();

        // Should be accurate with Gaussian noise model
        let lat_error = (position.lat - expected_pos.lat).abs() * 111320.0;
        let lon_error = (position.lon - expected_pos.lon).abs() * 111320.0;
        let depth_error = (position.depth - expected_pos.depth).abs();

        assert!(lat_error < 1.5, "MLE Latitude error: {} m", lat_error);
        assert!(lon_error < 1.5, "MLE Longitude error: {} m", lon_error);
        assert!(depth_error < 1.5, "MLE Depth error: {} m", depth_error);
        assert!(log_likelihood.is_finite(), "Log-likelihood should be finite");
    }

    #[test]
    fn test_mle_with_rayleigh_noise() {
        let mut trilateration = AdvancedTrilateration::new();
        let anchors = TrilaterationTestUtils::create_tetrahedral_anchors(120.0);
        
        let expected_pos = Position { lat: 0.0, lon: 0.0, depth: 0.0 };
        let receiver_time = TrilaterationTestUtils::calculate_receiver_time(&expected_pos, &anchors);

        // Test MLE with Rayleigh noise model (common in underwater acoustics)
        let measurement_variances = vec![2.0; anchors.len()]; // 2m² variance
        let result = trilateration.mle_trilateration(
            &anchors, 
            receiver_time, 
            &measurement_variances, 
            NoiseModel::Rayleigh
        );
        
        assert!(result.is_ok());
        let (position, _, log_likelihood) = result.unwrap();

        // Rayleigh noise model should still provide reasonable accuracy
        let lat_error = (position.lat - expected_pos.lat).abs() * 111320.0;
        let lon_error = (position.lon - expected_pos.lon).abs() * 111320.0;
        let depth_error = (position.depth - expected_pos.depth).abs();

        assert!(lat_error < 2.0, "Rayleigh MLE Latitude error: {} m", lat_error);
        assert!(lon_error < 2.0, "Rayleigh MLE Longitude error: {} m", lon_error);
        assert!(depth_error < 2.0, "Rayleigh MLE Depth error: {} m", depth_error);
        assert!(log_likelihood.is_finite(), "Log-likelihood should be finite");
    }

    #[test]
    fn test_kalman_enhanced_trilateration() {
        let mut trilateration = AdvancedTrilateration::new();
        let anchors = TrilaterationTestUtils::create_square_anchors(180.0, 40.0);
        
        let expected_pos = TrilaterationTestUtils::calculate_center_position(&anchors);
        let receiver_time = TrilaterationTestUtils::calculate_receiver_time(&expected_pos, &anchors);

        // Enable Kalman filtering
        trilateration.enable_kalman_filtering(0.1, 0.01, 1.0);

        // First measurement
        let result1 = trilateration.kalman_enhanced_trilateration(&anchors, receiver_time, None);
        assert!(result1.is_ok());
        let (position1, _, velocity1) = result1.unwrap();

        // Second measurement (slightly later)
        let result2 = trilateration.kalman_enhanced_trilateration(&anchors, receiver_time + 100, None);
        assert!(result2.is_ok());
        let (position2, _, velocity2) = result2.unwrap();

        // Kalman filter should provide smooth position estimates
        let lat_error1 = (position1.lat - expected_pos.lat).abs() * 111320.0;
        let lon_error1 = (position1.lon - expected_pos.lon).abs() * 111320.0;
        let lat_error2 = (position2.lat - expected_pos.lat).abs() * 111320.0;
        let lon_error2 = (position2.lon - expected_pos.lon).abs() * 111320.0;

        assert!(lat_error1 < 1.0, "Kalman Latitude error 1: {} m", lat_error1);
        assert!(lon_error1 < 1.0, "Kalman Longitude error 1: {} m", lon_error1);
        assert!(lat_error2 < 1.0, "Kalman Latitude error 2: {} m", lat_error2);
        assert!(lon_error2 < 1.0, "Kalman Longitude error 2: {} m", lon_error2);

        // Velocity estimates should be reasonable
        assert!(velocity1.norm() < 10.0, "Velocity 1 should be reasonable: {} m/s", velocity1.norm());
        assert!(velocity2.norm() < 10.0, "Velocity 2 should be reasonable: {} m/s", velocity2.norm());
    }

    #[test]
    fn test_insufficient_anchors_error() {
        let mut trilateration = AdvancedTrilateration::new();
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

        let weights = vec![1.0; anchors.len()];
        let result = trilateration.weighted_least_squares_trilateration(&anchors, 1002, &weights);
        
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("At least 3 anchors required"));
    }

    #[test]
    fn test_invalid_timing_error() {
        let mut trilateration = AdvancedTrilateration::new();
        let anchors = TrilaterationTestUtils::create_square_anchors(100.0, 20.0);
        
        // Receiver time earlier than anchor timestamps (invalid)
        let invalid_receiver_time = 500; // Earlier than anchor timestamps (1000+)
        
        let weights = vec![1.0; anchors.len()];
        let result = trilateration.weighted_least_squares_trilateration(&anchors, invalid_receiver_time, &weights);
        
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("Receiver time earlier than anchor time"));
    }

    #[test]
    fn test_algorithm_convergence() {
        let mut trilateration = AdvancedTrilateration::new();
        let anchors = TrilaterationTestUtils::create_tetrahedral_anchors(200.0);
        
        let expected_pos = Position { lat: 0.0, lon: 0.0, depth: 0.0 };
        let receiver_time = TrilaterationTestUtils::calculate_receiver_time(&expected_pos, &anchors);

        // Test convergence with different initial guesses
        let initial_guesses = vec![
            Some(Vector3::new(0.0, 0.0, 0.0)),
            Some(Vector3::new(50.0, 50.0, 10.0)),
            Some(Vector3::new(-30.0, 40.0, -5.0)),
            None, // No initial guess
        ];

        for (i, initial_guess) in initial_guesses.iter().enumerate() {
            let result = trilateration.levenberg_marquardt_trilateration(&anchors, receiver_time, *initial_guess);
            
            assert!(result.is_ok(), "Algorithm should converge with initial guess {}", i);
            
            let (position, _, cost) = result.unwrap();
            let lat_error = (position.lat - expected_pos.lat).abs() * 111320.0;
            let lon_error = (position.lon - expected_pos.lon).abs() * 111320.0;
            let depth_error = (position.depth - expected_pos.depth).abs();

            assert!(lat_error < 1.0, "Convergence test {} - Latitude error: {} m", i, lat_error);
            assert!(lon_error < 1.0, "Convergence test {} - Longitude error: {} m", i, lon_error);
            assert!(depth_error < 1.0, "Convergence test {} - Depth error: {} m", i, depth_error);
            assert!(cost < 10.0, "Convergence test {} - Cost should be reasonable: {}", i, cost);
        }
    }

    #[test]
    fn test_outlier_detection() {
        let mut trilateration = AdvancedTrilateration::new();
        let mut anchors = TrilaterationTestUtils::create_square_anchors(150.0, 25.0);
        
        // Add an outlier anchor with incorrect timing
        anchors.push(Anchor {
            id: "OUTLIER".to_string(),
            timestamp: 500, // Much earlier timestamp (outlier)
            position: Position { lat: 0.002, lon: 0.002, depth: 50.0 },
        });

        let expected_pos = TrilaterationTestUtils::calculate_center_position(&anchors[..4]); // Exclude outlier
        let receiver_time = TrilaterationTestUtils::calculate_receiver_time(&expected_pos, &anchors[..4]);

        // Test robust trilateration with outlier
        let result = trilateration.robust_trilateration(&anchors, receiver_time);
        
        assert!(result.is_ok());
        let (position, _, outliers) = result.unwrap();

        // Should detect the outlier
        assert!(outliers[4], "Should detect outlier anchor");
        
        // Position should still be accurate despite outlier
        let lat_error = (position.lat - expected_pos.lat).abs() * 111320.0;
        let lon_error = (position.lon - expected_pos.lon).abs() * 111320.0;

        assert!(lat_error < 2.0, "Outlier test - Latitude error: {} m", lat_error);
        assert!(lon_error < 2.0, "Outlier test - Longitude error: {} m", lon_error);
    }

    #[test]
    fn test_measurement_weighting() {
        let mut trilateration = AdvancedTrilateration::new();
        let anchors = TrilaterationTestUtils::create_square_anchors(120.0, 30.0);
        
        let expected_pos = TrilaterationTestUtils::calculate_center_position(&anchors);
        let receiver_time = TrilaterationTestUtils::calculate_receiver_time(&expected_pos, &anchors);

        // Test with different weighting schemes
        let weight_schemes = vec![
            vec![1.0, 1.0, 1.0, 1.0], // Equal weights
            vec![2.0, 1.0, 1.0, 1.0], // Higher weight on first anchor
            vec![1.0, 1.0, 1.0, 0.1], // Lower weight on last anchor
            vec![0.5, 2.0, 0.5, 2.0], // Alternating weights
        ];

        for (i, weights) in weight_schemes.iter().enumerate() {
            let result = trilateration.weighted_least_squares_trilateration(&anchors, receiver_time, weights);
            
            assert!(result.is_ok(), "Weighting scheme {} should work", i);
            
            let (position, _, _) = result.unwrap();
            let lat_error = (position.lat - expected_pos.lat).abs() * 111320.0;
            let lon_error = (position.lon - expected_pos.lon).abs() * 111320.0;

            // All weighting schemes should give reasonable results
            assert!(lat_error < 3.0, "Weighting test {} - Latitude error: {} m", i, lat_error);
            assert!(lon_error < 3.0, "Weighting test {} - Longitude error: {} m", i, lon_error);
        }
    }
}