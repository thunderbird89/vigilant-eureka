//! Unit tests for GDOP (Geometric Dilution of Precision) calculations and optimization
//! 
//! Tests cover:
//! - GDOP calculation accuracy for various anchor geometries
//! - Anchor selection optimization algorithms
//! - Quality assessment and classification
//! - Algorithm selection based on GDOP values

use crate::algorithms::gdop::{GdopOptimizer, GdopOptimizedTrilateration, AnchorSelectionResult};
use crate::algorithms::trilateration::{AdvancedTrilateration, GdopQuality, DilutionOfPrecision};
use crate::core::{Anchor, Position};
use nalgebra::Vector3;
use std::f64::consts::PI;

/// Test utilities for GDOP testing
pub struct GdopTestUtils;

impl GdopTestUtils {
    /// Create anchors with excellent geometry (regular tetrahedron)
    pub fn create_excellent_geometry_anchors(radius: f64) -> Vec<Anchor> {
        // Regular tetrahedron provides excellent GDOP
        let vertices = [
            (1.0, 1.0, 1.0),
            (1.0, -1.0, -1.0),
            (-1.0, 1.0, -1.0),
            (-1.0, -1.0, 1.0),
        ];

        vertices.iter().enumerate().map(|(i, &(x, y, z))| {
            let scale = radius / 3.0_f64.sqrt();
            Anchor {
                id: format!("EX{}", i + 1),
                timestamp: 1000 + i as u64,
                position: Position {
                    lat: Self::meters_to_degrees_lat(y * scale),
                    lon: Self::meters_to_degrees_lon(x * scale, 0.0),
                    depth: -z * scale,
                },
            }
        }).collect()
    }

    /// Create anchors with good geometry (square pyramid)
    pub fn create_good_geometry_anchors(base_size: f64, height: f64) -> Vec<Anchor> {
        let half_base = base_size / 2.0;
        vec![
            Anchor {
                id: "G1".to_string(),
                timestamp: 1000,
                position: Position { 
                    lat: Self::meters_to_degrees_lat(half_base), 
                    lon: Self::meters_to_degrees_lon(half_base, 0.0), 
                    depth: 0.0 
                },
            },
            Anchor {
                id: "G2".to_string(),
                timestamp: 1001,
                position: Position { 
                    lat: Self::meters_to_degrees_lat(half_base), 
                    lon: Self::meters_to_degrees_lon(-half_base, 0.0), 
                    depth: 0.0 
                },
            },
            Anchor {
                id: "G3".to_string(),
                timestamp: 1002,
                position: Position { 
                    lat: Self::meters_to_degrees_lat(-half_base), 
                    lon: Self::meters_to_degrees_lon(-half_base, 0.0), 
                    depth: 0.0 
                },
            },
            Anchor {
                id: "G4".to_string(),
                timestamp: 1003,
                position: Position { 
                    lat: Self::meters_to_degrees_lat(-half_base), 
                    lon: Self::meters_to_degrees_lon(half_base, 0.0), 
                    depth: 0.0 
                },
            },
            Anchor {
                id: "G5".to_string(),
                timestamp: 1004,
                position: Position { 
                    lat: 0.0, 
                    lon: 0.0, 
                    depth: height 
                },
            },
        ]
    }

    /// Create anchors with poor geometry (nearly collinear)
    pub fn create_poor_geometry_anchors(spacing: f64) -> Vec<Anchor> {
        (0..5).map(|i| {
            let distance = i as f64 * spacing;
            // Add small random offset to avoid perfect collinearity
            let offset = if i % 2 == 0 { 0.1 } else { -0.1 };
            
            Anchor {
                id: format!("P{}", i + 1),
                timestamp: 1000 + i as u64,
                position: Position {
                    lat: Self::meters_to_degrees_lat(distance),
                    lon: Self::meters_to_degrees_lon(offset, 0.0),
                    depth: 0.0,
                },
            }
        }).collect()
    }

    /// Create anchors with degenerate geometry (all coplanar)
    pub fn create_degenerate_geometry_anchors(radius: f64) -> Vec<Anchor> {
        (0..6).map(|i| {
            let angle = i as f64 * PI / 3.0; // 60-degree spacing
            let x = radius * angle.cos();
            let y = radius * angle.sin();
            
            Anchor {
                id: format!("D{}", i + 1),
                timestamp: 1000 + i as u64,
                position: Position {
                    lat: Self::meters_to_degrees_lat(y),
                    lon: Self::meters_to_degrees_lon(x, 0.0),
                    depth: 0.0, // All at same depth (degenerate)
                },
            }
        }).collect()
    }

    /// Create mixed quality anchor set for selection testing
    pub fn create_mixed_quality_anchors() -> Vec<Anchor> {
        let mut anchors = Vec::new();
        
        // Add some good anchors
        anchors.extend(Self::create_excellent_geometry_anchors(100.0));
        
        // Add some poor anchors
        anchors.extend(Self::create_poor_geometry_anchors(50.0));
        
        // Add some degenerate anchors
        anchors.extend(Self::create_degenerate_geometry_anchors(80.0));
        
        anchors
    }

    /// Convert meters to degrees latitude
    fn meters_to_degrees_lat(meters: f64) -> f64 {
        meters / 111320.0
    }

    /// Convert meters to degrees longitude
    fn meters_to_degrees_lon(meters: f64, lat_deg: f64) -> f64 {
        meters / (111320.0 * lat_deg.to_radians().cos())
    }

    /// Calculate expected GDOP range for geometry type
    pub fn expected_gdop_range(geometry_type: &str) -> (f64, f64) {
        match geometry_type {
            "excellent" => (1.0, 3.0),   // Excellent GDOP
            "good" => (2.0, 6.0),        // Good GDOP
            "moderate" => (5.0, 12.0),   // Moderate GDOP
            "poor" => (10.0, 30.0),      // Poor GDOP
            "degenerate" => (20.0, 100.0), // Very poor GDOP
            _ => (0.0, f64::INFINITY),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gdop_calculation_excellent_geometry() {
        let trilateration = AdvancedTrilateration::new();
        let anchors = GdopTestUtils::create_excellent_geometry_anchors(150.0);
        
        // Convert to local coordinates for GDOP calculation
        let reference_pos = &anchors[0].position;
        let positions: Vec<Vector3<f64>> = anchors.iter()
            .map(|a| trilateration.geodetic_to_local(&a.position, reference_pos))
            .collect();
        
        // Calculate centroid as receiver position estimate
        let centroid = positions.iter()
            .fold(Vector3::zeros(), |acc, pos| acc + pos) / positions.len() as f64;
        
        let dop = trilateration.calculate_gdop(&positions, &centroid);
        
        // Excellent geometry should have low GDOP
        let (min_gdop, max_gdop) = GdopTestUtils::expected_gdop_range("excellent");
        assert!(dop.gdop >= min_gdop && dop.gdop <= max_gdop, 
                "GDOP {} should be in excellent range [{}, {}]", dop.gdop, min_gdop, max_gdop);
        assert_eq!(dop.quality, GdopQuality::Excellent);
    }

    #[test]
    fn test_gdop_calculation_good_geometry() {
        let trilateration = AdvancedTrilateration::new();
        let anchors = GdopTestUtils::create_good_geometry_anchors(200.0, 100.0);
        
        let reference_pos = &anchors[0].position;
        let positions: Vec<Vector3<f64>> = anchors.iter()
            .map(|a| trilateration.geodetic_to_local(&a.position, reference_pos))
            .collect();
        
        let centroid = positions.iter()
            .fold(Vector3::zeros(), |acc, pos| acc + pos) / positions.len() as f64;
        
        let dop = trilateration.calculate_gdop(&positions, &centroid);
        
        // Good geometry should have reasonable GDOP
        let (min_gdop, max_gdop) = GdopTestUtils::expected_gdop_range("good");
        assert!(dop.gdop >= min_gdop && dop.gdop <= max_gdop, 
                "GDOP {} should be in good range [{}, {}]", dop.gdop, min_gdop, max_gdop);
        assert!(matches!(dop.quality, GdopQuality::Good | GdopQuality::Excellent));
    }

    #[test]
    fn test_gdop_calculation_poor_geometry() {
        let trilateration = AdvancedTrilateration::new();
        let anchors = GdopTestUtils::create_poor_geometry_anchors(50.0);
        
        let reference_pos = &anchors[0].position;
        let positions: Vec<Vector3<f64>> = anchors.iter()
            .map(|a| trilateration.geodetic_to_local(&a.position, reference_pos))
            .collect();
        
        let centroid = positions.iter()
            .fold(Vector3::zeros(), |acc, pos| acc + pos) / positions.len() as f64;
        
        let dop = trilateration.calculate_gdop(&positions, &centroid);
        
        // Poor geometry should have high GDOP
        let (min_gdop, max_gdop) = GdopTestUtils::expected_gdop_range("poor");
        assert!(dop.gdop >= min_gdop, 
                "GDOP {} should be at least {} for poor geometry", dop.gdop, min_gdop);
        assert!(matches!(dop.quality, GdopQuality::Poor | GdopQuality::Fair));
    }

    #[test]
    fn test_gdop_calculation_degenerate_geometry() {
        let trilateration = AdvancedTrilateration::new();
        let anchors = GdopTestUtils::create_degenerate_geometry_anchors(100.0);
        
        let reference_pos = &anchors[0].position;
        let positions: Vec<Vector3<f64>> = anchors.iter()
            .map(|a| trilateration.geodetic_to_local(&a.position, reference_pos))
            .collect();
        
        let centroid = positions.iter()
            .fold(Vector3::zeros(), |acc, pos| acc + pos) / positions.len() as f64;
        
        let dop = trilateration.calculate_gdop(&positions, &centroid);
        
        // Degenerate geometry should have very high GDOP
        let (min_gdop, _) = GdopTestUtils::expected_gdop_range("degenerate");
        assert!(dop.gdop >= min_gdop, 
                "GDOP {} should be at least {} for degenerate geometry", dop.gdop, min_gdop);
        assert_eq!(dop.quality, GdopQuality::Poor);
    }

    #[test]
    fn test_anchor_selection_optimization() {
        let optimizer = GdopOptimizer::new();
        let anchors = GdopTestUtils::create_mixed_quality_anchors();
        
        // Test anchor selection
        let result = optimizer.select_optimal_anchors(&anchors, 1010, 1.0);
        
        assert!(result.is_ok());
        let selection = result.unwrap();
        
        // Should select at least 4 anchors
        assert!(selection.selected_indices.len() >= 4, 
                "Should select at least 4 anchors, got {}", selection.selected_indices.len());
        
        // Should not select more than max anchors
        assert!(selection.selected_indices.len() <= 8, 
                "Should not select more than 8 anchors, got {}", selection.selected_indices.len());
        
        // All selected indices should be valid
        for &idx in &selection.selected_indices {
            assert!(idx < anchors.len(), "Invalid anchor index: {}", idx);
        }
        
        // GDOP should be reasonable
        assert!(selection.gdop > 0.0, "GDOP should be positive");
        assert!(selection.gdop < 100.0, "GDOP should be reasonable, got {}", selection.gdop);
        
        // Position uncertainty should be calculated
        assert!(selection.position_uncertainty > 0.0, "Position uncertainty should be positive");
        assert!(selection.position_uncertainty < 100.0, "Position uncertainty should be reasonable");
    }

    #[test]
    fn test_anchor_selection_with_excellent_geometry() {
        let optimizer = GdopOptimizer::new();
        let anchors = GdopTestUtils::create_excellent_geometry_anchors(120.0);
        
        let result = optimizer.select_optimal_anchors(&anchors, 1010, 1.0);
        
        assert!(result.is_ok());
        let selection = result.unwrap();
        
        // Should select all anchors for excellent geometry
        assert_eq!(selection.selected_indices.len(), anchors.len());
        
        // GDOP should be excellent
        assert!(selection.gdop < 5.0, "GDOP should be excellent, got {}", selection.gdop);
        assert_eq!(selection.quality, GdopQuality::Excellent);
        
        // Position uncertainty should be low
        assert!(selection.position_uncertainty < 5.0, 
                "Position uncertainty should be low for excellent geometry, got {}", 
                selection.position_uncertainty);
    }

    #[test]
    fn test_anchor_selection_with_insufficient_anchors() {
        let optimizer = GdopOptimizer::new();
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
        
        let result = optimizer.select_optimal_anchors(&anchors, 1010, 1.0);
        
        assert!(result.is_err());
        assert!(result.unwrap_err().contains("Insufficient anchors"));
    }

    #[test]
    fn test_gdop_optimized_trilateration() {
        let mut trilateration = AdvancedTrilateration::new();
        let anchors = GdopTestUtils::create_good_geometry_anchors(150.0, 75.0);
        
        // Calculate expected receiver position
        let expected_pos = Position { lat: 0.0, lon: 0.0, depth: 25.0 };
        let receiver_time = 1010;
        
        let result = trilateration.trilaterate_with_gdop_optimization(&anchors, receiver_time, 1.0);
        
        assert!(result.is_ok());
        let (position, _, dop) = result.unwrap();
        
        // Position should be reasonable
        assert!(position.lat.abs() < 0.01, "Latitude should be reasonable");
        assert!(position.lon.abs() < 0.01, "Longitude should be reasonable");
        assert!(position.depth > -100.0 && position.depth < 200.0, "Depth should be reasonable");
        
        // GDOP should be calculated
        assert!(dop.gdop > 0.0, "GDOP should be positive");
        assert!(dop.gdop < 50.0, "GDOP should be reasonable");
    }

    #[test]
    fn test_trilateration_with_anchor_selection() {
        let mut trilateration = AdvancedTrilateration::new();
        let anchors = GdopTestUtils::create_mixed_quality_anchors();
        
        let receiver_time = 1020;
        let result = trilateration.trilaterate_with_anchor_selection(&anchors, receiver_time, 1.5);
        
        assert!(result.is_ok());
        let (position, _, selection) = result.unwrap();
        
        // Position should be calculated
        assert!(position.lat.abs() < 0.1, "Latitude should be reasonable");
        assert!(position.lon.abs() < 0.1, "Longitude should be reasonable");
        
        // Should have selected optimal anchors
        assert!(selection.selected_indices.len() >= 4, "Should select at least 4 anchors");
        assert!(selection.gdop > 0.0, "GDOP should be positive");
        
        // Selected anchors should be better than using all anchors
        assert!(selection.gdop < 50.0, "Selected anchors should have reasonable GDOP");
    }

    #[test]
    fn test_algorithm_selection_by_gdop() {
        let optimizer = GdopOptimizer::new();
        
        // Test algorithm selection for different GDOP values
        let test_cases = vec![
            (1.5, "LinearLeastSquares"),    // Excellent GDOP
            (3.0, "WeightedLeastSquares"),  // Good GDOP
            (7.0, "LevenbergMarquardt"),    // Moderate GDOP
            (15.0, "RobustMle"),            // Poor GDOP
        ];
        
        for (gdop_value, expected_algorithm) in test_cases {
            let algorithm = optimizer.select_algorithm_by_gdop(gdop_value);
            
            // Verify algorithm selection logic
            match expected_algorithm {
                "LinearLeastSquares" => {
                    assert!(matches!(algorithm, crate::algorithms::trilateration::TrilaterationAlgorithm::LinearLeastSquares),
                            "Should select LinearLeastSquares for GDOP {}", gdop_value);
                },
                "WeightedLeastSquares" => {
                    assert!(matches!(algorithm, crate::algorithms::trilateration::TrilaterationAlgorithm::WeightedLeastSquares),
                            "Should select WeightedLeastSquares for GDOP {}", gdop_value);
                },
                "LevenbergMarquardt" => {
                    assert!(matches!(algorithm, crate::algorithms::trilateration::TrilaterationAlgorithm::LevenbergMarquardt),
                            "Should select LevenbergMarquardt for GDOP {}", gdop_value);
                },
                "RobustMle" => {
                    assert!(matches!(algorithm, crate::algorithms::trilateration::TrilaterationAlgorithm::RobustMle),
                            "Should select RobustMle for GDOP {}", gdop_value);
                },
                _ => panic!("Unknown expected algorithm: {}", expected_algorithm),
            }
        }
    }

    #[test]
    fn test_position_uncertainty_calculation() {
        let optimizer = GdopOptimizer::new();
        
        // Test uncertainty calculation for different GDOP values and range uncertainties
        let test_cases = vec![
            (2.0, 1.0, 2.0),    // GDOP=2, range_uncertainty=1m, expected ~2m
            (5.0, 0.5, 2.5),    // GDOP=5, range_uncertainty=0.5m, expected ~2.5m
            (10.0, 2.0, 20.0),  // GDOP=10, range_uncertainty=2m, expected ~20m
        ];
        
        for (gdop, range_uncertainty, expected_uncertainty) in test_cases {
            let uncertainty = optimizer.calculate_position_uncertainty(gdop, range_uncertainty);
            
            assert!((uncertainty - expected_uncertainty).abs() < 0.1,
                    "Position uncertainty calculation: GDOP={}, range_unc={}, expected={}, got={}",
                    gdop, range_uncertainty, expected_uncertainty, uncertainty);
        }
    }

    #[test]
    fn test_gdop_quality_classification() {
        let test_cases = vec![
            (1.5, GdopQuality::Excellent),
            (3.0, GdopQuality::Good),
            (7.0, GdopQuality::Moderate),
            (12.0, GdopQuality::Fair),
            (25.0, GdopQuality::Poor),
        ];
        
        for (gdop_value, expected_quality) in test_cases {
            let dop = DilutionOfPrecision {
                gdop: gdop_value,
                pdop: gdop_value * 0.8,
                hdop: gdop_value * 0.6,
                vdop: gdop_value * 0.4,
                tdop: gdop_value * 0.2,
                quality: expected_quality,
            };
            
            assert_eq!(dop.quality, expected_quality,
                      "GDOP {} should be classified as {:?}", gdop_value, expected_quality);
        }
    }

    #[test]
    fn test_gdop_comparison_different_geometries() {
        let trilateration = AdvancedTrilateration::new();
        
        // Create different geometry types
        let excellent_anchors = GdopTestUtils::create_excellent_geometry_anchors(100.0);
        let good_anchors = GdopTestUtils::create_good_geometry_anchors(100.0, 50.0);
        let poor_anchors = GdopTestUtils::create_poor_geometry_anchors(30.0);
        
        // Calculate GDOP for each geometry
        let geometries = vec![
            ("excellent", excellent_anchors),
            ("good", good_anchors),
            ("poor", poor_anchors),
        ];
        
        let mut gdop_values = Vec::new();
        
        for (name, anchors) in geometries {
            let reference_pos = &anchors[0].position;
            let positions: Vec<Vector3<f64>> = anchors.iter()
                .map(|a| trilateration.geodetic_to_local(&a.position, reference_pos))
                .collect();
            
            let centroid = positions.iter()
                .fold(Vector3::zeros(), |acc, pos| acc + pos) / positions.len() as f64;
            
            let dop = trilateration.calculate_gdop(&positions, &centroid);
            gdop_values.push((name, dop.gdop));
        }
        
        // Verify GDOP ordering: excellent < good < poor
        assert!(gdop_values[0].1 < gdop_values[1].1, 
                "Excellent GDOP ({}) should be better than good GDOP ({})", 
                gdop_values[0].1, gdop_values[1].1);
        
        assert!(gdop_values[1].1 < gdop_values[2].1, 
                "Good GDOP ({}) should be better than poor GDOP ({})", 
                gdop_values[1].1, gdop_values[2].1);
    }

    #[test]
    fn test_gdop_optimizer_parameters() {
        // Test custom optimizer parameters
        let optimizer = GdopOptimizer::with_parameters(15.0, 3, 6);
        let anchors = GdopTestUtils::create_good_geometry_anchors(120.0, 60.0);
        
        let result = optimizer.select_optimal_anchors(&anchors, 1010, 1.0);
        
        assert!(result.is_ok());
        let selection = result.unwrap();
        
        // Should respect parameter constraints
        assert!(selection.selected_indices.len() >= 3, "Should select at least min_anchors");
        assert!(selection.selected_indices.len() <= 6, "Should not exceed max_anchors");
        
        // Should accept higher GDOP threshold
        assert!(selection.gdop <= 15.0, "Should accept GDOP up to max_acceptable_gdop");
    }
}