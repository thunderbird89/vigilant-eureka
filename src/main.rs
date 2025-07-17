mod high_precision;
mod accuracy_validation;
mod optimization_cache;
mod performance_monitor;
mod noise_filtering;
mod gdop_optimization;
mod advanced_trilateration;
mod kalman_filter;

// Structure definitions
pub struct Position {
    pub lat: f64,
    pub lon: f64,
    pub depth: f64,
}

impl Clone for Position {
    fn clone(&self) -> Self {
        Self {
            lat: self.lat,
            lon: self.lon,
            depth: self.depth,
        }
    }
}

pub struct Anchor {
    pub id: String,
    pub timestamp: u64,
    pub position: Position,
}

impl Clone for Anchor {
    fn clone(&self) -> Self {
        Self {
            id: self.id.clone(),
            timestamp: self.timestamp,
            position: self.position.clone(),
        }
    }
}

// Import types from advanced_trilateration module
pub use advanced_trilateration::{GdopQuality, DilutionOfPrecision, AnchorGeometryAssessment};

// Re-export types from other modules for easier access
pub use high_precision::{HighPrecisionTransformer, EnvironmentalCorrections};
pub use accuracy_validation::{AccuracyValidator, PositionError, AccuracyStatistics};

// Constants
pub const SPEED_OF_SOUND_WATER: f64 = 1500.0; // m/s in standard conditions

// Main function
fn main() {
    println!("Underwater Positioning System Demo");
    
    // Run the sub-meter accuracy optimization demo
    submeter_accuracy_optimization_demo();
}

// Sub-meter accuracy optimization demo
pub fn submeter_accuracy_optimization_demo() {
    use crate::{HighPrecisionTransformer, EnvironmentalCorrections, AccuracyValidator};
    use crate::performance_monitor::PerformanceMonitor;
    use crate::optimization_cache::OptimizationCache;
    use nalgebra::Vector3;
    
    println!("=== SUB-METER ACCURACY OPTIMIZATION DEMO ===\n");
    
    // Create components
    let mut transformer = HighPrecisionTransformer::new();
    let mut validator = AccuracyValidator::new();
    let mut monitor = PerformanceMonitor::new();
    let mut cache = OptimizationCache::new();
    
    // Create test anchor configurations
    let base_time = 1723111199000u64;
    
    // Good geometry (tetrahedron-like)
    let good_geometry_anchors = vec![
        Anchor {
            id: "001".to_string(),
            timestamp: base_time + 986,
            position: Position { lat: 32.12345, lon: 45.47675, depth: 0.0 },
        },
        Anchor {
            id: "002".to_string(),
            timestamp: base_time + 988,
            position: Position { lat: 32.12365, lon: 45.47695, depth: 0.0 },
        },
        Anchor {
            id: "003".to_string(),
            timestamp: base_time + 988,
            position: Position { lat: 32.12365, lon: 45.47655, depth: 0.0 },
        },
        Anchor {
            id: "004".to_string(),
            timestamp: base_time + 986,
            position: Position { lat: 32.12385, lon: 45.47675, depth: 10.0 },
        },
    ];
    
    // Reference position for local coordinate conversion
    let reference_pos = &good_geometry_anchors[0].position;
    
    // Convert anchor positions to local coordinates
    let good_geometry: Vec<Vector3<f64>> = good_geometry_anchors.iter()
        .map(|a| {
            let local = transformer.geodetic_to_enu(&a.position, reference_pos);
            local
        })
        .collect();
    
    println!("1. HIGH-PRECISION COORDINATE TRANSFORMATIONS");
    
    // Test position
    let test_position = Position {
        lat: 32.12355,
        lon: 45.47685,
        depth: 5.0,
    };
    
    // Convert to ECEF and back
    let ecef = transformer.geodetic_to_ecef(&test_position);
    let geodetic = transformer.ecef_to_geodetic(&ecef);
    
    println!("   Original position:");
    println!("   - Latitude: {:.6}", test_position.lat);
    println!("   - Longitude: {:.6}", test_position.lon);
    println!("   - Depth: {:.2} m", test_position.depth);
    
    println!("\n   ECEF coordinates:");
    println!("   - X: {:.2} m", ecef.x);
    println!("   - Y: {:.2} m", ecef.y);
    println!("   - Z: {:.2} m", ecef.z);
    
    println!("\n   Converted back to geodetic:");
    println!("   - Latitude: {:.6}", geodetic.lat);
    println!("   - Longitude: {:.6}", geodetic.lon);
    println!("   - Depth: {:.2} m", geodetic.depth);
    
    println!("\n   Conversion error:");
    println!("   - Latitude: {:.8} degrees", (test_position.lat - geodetic.lat).abs());
    println!("   - Longitude: {:.8} degrees", (test_position.lon - geodetic.lon).abs());
    println!("   - Depth: {:.4} m", (test_position.depth - geodetic.depth).abs());
    
    println!("\n2. ENVIRONMENTAL CORRECTIONS FOR SOUND SPEED");
    
    // Create environmental parameters
    let env = EnvironmentalCorrections {
        base_sound_speed: 1500.0,
        temperature: 15.0,
        salinity: 35.0,
        depth: 50.0,
        pressure: 5.0,
    };
    
    // Calculate corrected sound speed
    let corrected_speed = transformer.calculate_sound_speed(&env);
    
    println!("   Standard sound speed: 1500.0 m/s");
    println!("   Corrected sound speed: {:.2} m/s", corrected_speed);
    println!("   Correction factor: {:.4}", corrected_speed / 1500.0);
    
    // Calculate impact on range measurements
    let example_range = 1000.0;
    let corrected_range = transformer.calculate_range_correction(
        example_range, 1500.0, corrected_speed
    );
    
    println!("\n   For a 1000m range measurement:");
    println!("   - Uncorrected: 1000.000 m");
    println!("   - Corrected: {:.3} m", corrected_range);
    println!("   - Difference: {:.3} m", corrected_range - example_range);
    
    println!("\n3. POSITION UNCERTAINTY ANALYSIS");
    
    // True position for simulation
    let true_position = Vector3::new(50.0, 50.0, 50.0);
    
    // Calculate position uncertainty
    let range_std_devs = vec![0.5, 0.5, 0.5, 0.5];
    let uncertainty = transformer.calculate_position_uncertainty(
        &true_position, &good_geometry, &range_std_devs
    );
    
    println!("   Position uncertainty with good geometry:");
    println!("   - Standard deviation X: {:.3} m", uncertainty.std_dev_x);
    println!("   - Standard deviation Y: {:.3} m", uncertainty.std_dev_y);
    println!("   - Standard deviation Z: {:.3} m", uncertainty.std_dev_z);
    println!("   - 95% confidence radius: {:.3} m", uncertainty.confidence_radius_95);
    
    println!("\n4. ACCURACY VALIDATION");
    
    // Simulate positioning with different range uncertainties
    println!("   Simulating positioning with different range uncertainties:");
    
    let range_uncertainties = [0.1, 0.3, 0.5, 1.0];
    
    for &range_uncertainty in &range_uncertainties {
        let result = validator.simulate_accuracy(&true_position, &good_geometry, range_uncertainty, 100);
        
        println!("\n   Range uncertainty: {:.1} m", range_uncertainty);
        println!("   - Mean error: {:.3} m", result.statistics.mean_error);
        println!("   - 95% error: {:.3} m", result.statistics.error_95_percentile);
        println!("   - Sub-meter accuracy rate: {:.1}%", result.submeter_accuracy_rate * 100.0);
    }
    
    println!("\n5. FACTORS AFFECTING SUB-METER ACCURACY");
    
    let factors = validator.analyze_submeter_accuracy_factors();
    for (factor, impact) in factors {
        println!("   - {}: {:.3} correlation", factor, impact);
    }
    
    println!("\n6. RECOMMENDATIONS FOR SUB-METER ACCURACY");
    
    let recommendations = validator.optimize_for_submeter_accuracy(&good_geometry, &true_position);
    for (i, recommendation) in recommendations.iter().enumerate() {
        println!("   {}. {}", i+1, recommendation);
    }
    
    println!("\n7. PERFORMANCE MONITORING");
    
    // Generate and print performance report
    let report = monitor.generate_report();
    println!("{}", report);
    
    println!("\n8. BEST PRACTICES FOR SUB-METER ACCURACY");
    println!("   1. Deploy anchors in tetrahedral configuration");
    println!("   2. Use at least 5-6 anchors for redundancy");
    println!("   3. Apply environmental corrections for sound speed");
    println!("   4. Use adaptive signal processing for noise and multipath mitigation");
    println!("   5. Apply Kalman filtering for temporal smoothing");
    println!("   6. Use GDOP-based anchor selection");
    println!("   7. Regularly calibrate and validate system performance");
    println!("   8. Optimize algorithms for specific underwater conditions");
}

// Demonstration of GDOP optimization (simplified for available modules)
pub fn gdop_optimization_demo() {
    use nalgebra::Vector3;
    
    println!("=== GDOP OPTIMIZATION DEMO ===\n");
    println!("This demo requires additional modules that are not yet implemented.");

    // Create test anchor configurations 
    let base_time = 1723111199000u64;
    
    // Good geometry (tetrahedron-like)
    let good_geometry_anchors = vec![
        Anchor {
            id: "001".to_string(),
            timestamp: base_time + 986,
            position: Position { lat: 32.12345, lon: 45.47675, depth: 0.0 },
        },
        Anchor {
            id: "002".to_string(),
            timestamp: base_time + 988,
            position: Position { lat: 32.12365, lon: 45.47695, depth: 0.0 },
        },
        Anchor {
            id: "003".to_string(),
            timestamp: base_time + 988,
            position: Position { lat: 32.12365, lon: 45.47655, depth: 0.0 },
        },
        Anchor {
            id: "004".to_string(),
            timestamp: base_time + 986,
            position: Position { lat: 32.12385, lon: 45.47675, depth: 10.0 },
        },
    ];
    
    // Poor geometry (nearly collinear)
    let poor_geometry_anchors = vec![
        Anchor {
            id: "001".to_string(),
            timestamp: base_time + 986,
            position: Position { lat: 32.12345, lon: 45.47675, depth: 0.0 },
        },
        Anchor {
            id: "002".to_string(),
            timestamp: base_time + 988,
            position: Position { lat: 32.12346, lon: 45.47676, depth: 0.0 },
        },
        Anchor {
            id: "003".to_string(),
            timestamp: base_time + 988,
            position: Position { lat: 32.12347, lon: 45.47677, depth: 0.0 },
        },
        Anchor {
            id: "004".to_string(),
            timestamp: base_time + 986,
            position: Position { lat: 32.12348, lon: 45.47678, depth: 0.1 },
        },
    ];
    
    // Mixed geometry
    let mixed_geometry_anchors = vec![
        Anchor {
            id: "001".to_string(),
            timestamp: base_time + 986,
            position: Position { lat: 32.12345, lon: 45.47675, depth: 0.0 },
        },
        Anchor {
            id: "002".to_string(),
            timestamp: base_time + 988,
            position: Position { lat: 32.12365, lon: 45.47695, depth: 0.0 },
        },
        Anchor {
            id: "003".to_string(),
            timestamp: base_time + 988,
            position: Position { lat: 32.12365, lon: 45.47655, depth: 0.0 },
        },
        Anchor {
            id: "004".to_string(),
            timestamp: base_time + 986,
            position: Position { lat: 32.12385, lon: 45.47675, depth: 10.0 },
        },
        Anchor {
            id: "005".to_string(),
            timestamp: base_time + 990,
            position: Position { lat: 32.12355, lon: 45.47685, depth: 5.0 },
        },
        Anchor {
            id: "006".to_string(),
            timestamp: base_time + 992,
            position: Position { lat: 32.12375, lon: 45.47665, depth: 7.0 },
        },
    ];
    
    println!("1. GDOP CALCULATION FOR DIFFERENT GEOMETRIES");
    
    // Reference position for local coordinate conversion
    let reference_pos = &good_geometry_anchors[0].position;
    let receiver_pos = Vector3::new(0.0, 0.0, 5.0);
    
    println!("   Anchor configurations created for demonstration purposes.");
    println!("   - Good geometry: {} anchors", good_geometry_anchors.len());
    println!("   - Poor geometry: {} anchors", poor_geometry_anchors.len());
    println!("   - Mixed geometry: {} anchors", mixed_geometry_anchors.len());
    
    println!("\n7. BEST PRACTICES FOR GDOP OPTIMIZATION");
    println!("   1. Deploy anchors in tetrahedral configuration when possible");
    println!("   2. Avoid collinear or coplanar anchor arrangements");
    println!("   3. Maintain good spatial distribution around the target area");
    println!("   4. Monitor GDOP values and adjust anchor positions if needed");
    println!("   5. Use adaptive algorithm selection based on geometry quality");
}