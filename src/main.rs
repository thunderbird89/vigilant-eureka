mod high_precision;
mod accuracy_validation;
mod optimization_cache;
mod performance_monitor;
mod noise_filtering;
mod gdop_optimization;
mod advanced_trilateration;
mod kalman_filter;
mod message_parser;
mod data_validator;
mod error_handling;
mod graceful_degradation;
mod configuration;

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
pub use message_parser::{MessageParser, AnchorMessage, GeodeticPosition, ParseError, MessageVersion, RawMessage};
pub use data_validator::{DataValidator, ValidationConfig, ValidationError, ValidationResult, GeometryQuality};
pub use error_handling::{PositioningError, ErrorReporter, ErrorContext, SystemState, DiagnosticInfo, RecoveryStrategy};
pub use graceful_degradation::{GracefulDegradationManager, SystemHealthMonitor, SystemHealth, PositioningCapability, DegradationDecision};
pub use configuration::{ConfigurationManager, SystemConfig, AnchorConfig, ConfigError, ValidationResult as ConfigValidationResult, ParameterUpdates, ParameterUpdateResult, ParameterSummary, ConfigurationSnapshot};

// Constants
pub const SPEED_OF_SOUND_WATER: f64 = 1500.0; // m/s in standard conditions

// Main function
fn main() {
    println!("Underwater Positioning System Demo");
    
    // Run the message parsing and validation demo
    message_parsing_validation_demo();
    
    println!("\n{}\n", "=".repeat(60));
    
    // Run the sub-meter accuracy optimization demo
    submeter_accuracy_optimization_demo();
    
    println!("\n{}\n", "=".repeat(60));
    
    // Run the error handling and graceful degradation demo
    error_handling_graceful_degradation_demo();
    
    println!("\n{}\n", "=".repeat(60));
    
    // Run the configuration management demo
    configuration_management_demo();
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

// Message parsing and validation demo
pub fn message_parsing_validation_demo() {
    use crate::{MessageParser, DataValidator, ValidationConfig, MessageVersion, GeodeticPosition, RawMessage};
    
    println!("=== MESSAGE PARSING AND VALIDATION DEMO ===\n");
    
    // Create parser and validator
    let parser = MessageParser::new();
    let mut validator = DataValidator::with_config(ValidationConfig {
        max_message_age_ms: 60000,      // 1 minute
        max_time_drift_ms: 5000,        // 5 seconds
        min_signal_quality: 60,         // Minimum quality threshold
        max_position_jump_m: 200.0,     // 200 meters max jump
        max_depth_m: 1000.0,            // 1km max depth
        min_anchor_count: 3,            // Minimum for positioning
        strict_geometry_validation: true,
    });
    
    println!("1. CREATING TEST MESSAGES");
    
    // Create test raw messages for different versions and scenarios
    let current_time = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64;
    
    // Valid V1 message
    let v1_raw = create_test_v1_message(101, current_time - 1000, 32.123456, -117.654321, 15.5, 85, 1);
    
    // Valid V2 message  
    let v2_raw = create_test_v2_message(102, current_time - 500, 32.124000, -117.655000, 12.0, 90, 1, current_time);
    
    // Message with poor signal quality
    let poor_quality_raw = create_test_v1_message(103, current_time - 800, 32.125000, -117.656000, 18.0, 45, 1);
    
    // Message that's too old
    let stale_raw = create_test_v1_message(104, current_time - 120000, 32.126000, -117.657000, 20.0, 95, 1);
    
    println!("   Created {} test raw messages", 4);
    
    println!("\n2. PARSING MESSAGES");
    
    let raw_messages = vec![v1_raw, v2_raw, poor_quality_raw, stale_raw];
    let mut parsed_messages = Vec::new();
    let mut parse_errors = Vec::new();
    
    for (i, raw) in raw_messages.iter().enumerate() {
        match parser.parse_message(raw) {
            Ok(message) => {
                println!("   Message {}: Parsed successfully", i + 1);
                println!("     - Anchor ID: {}", message.anchor_id);
                println!("     - Version: {:?}", message.version);
                println!("     - Position: ({:.6}, {:.6}, {:.2}m)", 
                         message.position.latitude, message.position.longitude, message.position.depth);
                println!("     - Quality: {}", message.signal_quality);
                parsed_messages.push(message);
            }
            Err(error) => {
                println!("   Message {}: Parse failed - {}", i + 1, error);
                parse_errors.push(error);
            }
        }
    }
    
    println!("\n   Parse Results:");
    println!("   - Successfully parsed: {}", parsed_messages.len());
    println!("   - Parse errors: {}", parse_errors.len());
    
    println!("\n3. VALIDATING MESSAGES");
    
    let validation_result = validator.validate_messages(parsed_messages.clone());
    
    println!("   Validation Results:");
    println!("   - Valid messages: {}", validation_result.valid_messages.len());
    println!("   - Rejected messages: {}", validation_result.rejected_messages.len());
    println!("   - Warnings: {}", validation_result.warnings.len());
    println!("   - Geometry quality: {:?}", validation_result.geometry_quality);
    
    // Display rejected messages with reasons
    if !validation_result.rejected_messages.is_empty() {
        println!("\n   Rejected Messages:");
        for (message, error) in &validation_result.rejected_messages {
            println!("     - Anchor {}: {}", message.anchor_id, error);
        }
    }
    
    // Display warnings
    if !validation_result.warnings.is_empty() {
        println!("\n   Warnings:");
        for warning in &validation_result.warnings {
            println!("     - {}", warning);
        }
    }
    
    println!("\n4. TESTING COMPACT FORMAT CONVERSION");
    
    if let Some(message) = validation_result.valid_messages.first() {
        let compact = parser.to_compact_format(message);
        let restored = parser.from_compact_format(&compact, current_time - 10000);
        
        println!("   Original message size: ~{} bytes", std::mem::size_of::<crate::AnchorMessage>());
        println!("   Compact message size: {} bytes", std::mem::size_of_val(&compact));
        
        println!("   Conversion accuracy:");
        println!("     - Latitude diff: {:.8} degrees", 
                 (message.position.latitude - restored.position.latitude).abs());
        println!("     - Longitude diff: {:.8} degrees", 
                 (message.position.longitude - restored.position.longitude).abs());
        println!("     - Depth diff: {:.3} meters", 
                 (message.position.depth - restored.position.depth).abs());
    }
    
    println!("\n5. TESTING HISTORICAL VALIDATION");
    
    // Create a sequence of messages from the same anchor to test historical validation
    let anchor_id = 201;
    let base_pos = GeodeticPosition {
        latitude: 32.130000,
        longitude: -117.660000,
        depth: 25.0,
    };
    
    // First message - establishes history
    let msg1 = crate::AnchorMessage {
        anchor_id,
        timestamp_ms: current_time - 5000,
        position: base_pos.clone(),
        signal_quality: 80,
        message_sequence: 10,
        version: MessageVersion::V1,
    };
    
    // Second message - normal progression
    let msg2 = crate::AnchorMessage {
        anchor_id,
        timestamp_ms: current_time - 4000,
        position: GeodeticPosition {
            latitude: base_pos.latitude + 0.0001, // Small movement
            longitude: base_pos.longitude + 0.0001,
            depth: base_pos.depth + 0.5,
        },
        signal_quality: 82,
        message_sequence: 11,
        version: MessageVersion::V1,
    };
    
    // Third message - large position jump (should be rejected)
    let msg3 = crate::AnchorMessage {
        anchor_id,
        timestamp_ms: current_time - 3000,
        position: GeodeticPosition {
            latitude: base_pos.latitude + 0.01, // Large jump (~1km)
            longitude: base_pos.longitude + 0.01,
            depth: base_pos.depth,
        },
        signal_quality: 85,
        message_sequence: 12,
        version: MessageVersion::V1,
    };
    
    println!("   Testing sequence validation for anchor {}:", anchor_id);
    
    // Validate messages in sequence
    let result1 = validator.validate_messages(vec![msg1]);
    println!("     Message 1: {} valid, {} rejected", 
             result1.valid_messages.len(), result1.rejected_messages.len());
    
    let result2 = validator.validate_messages(vec![msg2]);
    println!("     Message 2: {} valid, {} rejected", 
             result2.valid_messages.len(), result2.rejected_messages.len());
    
    let result3 = validator.validate_messages(vec![msg3]);
    println!("     Message 3: {} valid, {} rejected", 
             result3.valid_messages.len(), result3.rejected_messages.len());
    
    if !result3.rejected_messages.is_empty() {
        println!("     Rejection reason: {}", result3.rejected_messages[0].1);
    }
    
    println!("\n6. ANCHOR STATISTICS");
    
    if let Some(stats) = validator.get_anchor_statistics(anchor_id) {
        println!("   Statistics for anchor {}:", anchor_id);
        println!("     - Total messages: {}", stats.message_count);
        println!("     - Last seen: {} ms ago", current_time - stats.last_seen);
        println!("     - Average quality: {:.1}", stats.average_quality);
        println!("     - Last position: ({:.6}, {:.6}, {:.2}m)", 
                 stats.last_position.latitude, stats.last_position.longitude, stats.last_position.depth);
    }
    
    println!("\n7. MULTIPLE MESSAGE FORMAT SUPPORT");
    
    // Test parsing different versions in the same batch
    let mixed_messages = vec![
        create_test_v1_message(301, current_time - 2000, 32.140000, -117.670000, 30.0, 88, 1),
        create_test_v2_message(302, current_time - 1800, 32.141000, -117.671000, 28.0, 92, 1, current_time),
        create_test_v1_message(303, current_time - 1600, 32.142000, -117.672000, 32.0, 85, 1),
    ];
    
    let mut mixed_parsed = Vec::new();
    for raw in mixed_messages {
        if let Ok(message) = parser.parse_message(&raw) {
            mixed_parsed.push(message);
        }
    }
    
    let mixed_result = validator.validate_messages(mixed_parsed);
    println!("   Mixed format validation:");
    println!("     - Valid messages: {}", mixed_result.valid_messages.len());
    println!("     - Geometry quality: {:?}", mixed_result.geometry_quality);
    
    // Show version distribution
    let mut v1_count = 0;
    let mut v2_count = 0;
    for message in &mixed_result.valid_messages {
        match message.version {
            MessageVersion::V1 => v1_count += 1,
            MessageVersion::V2 => v2_count += 1,
        }
    }
    println!("     - V1 messages: {}", v1_count);
    println!("     - V2 messages: {}", v2_count);
    
    println!("\n8. PERFORMANCE CHARACTERISTICS");
    
    // Test parsing performance
    let start_time = std::time::Instant::now();
    let test_raw = create_test_v1_message(999, current_time, 32.0, -117.0, 10.0, 80, 1);
    
    for _ in 0..1000 {
        let _ = parser.parse_message(&test_raw);
    }
    
    let parse_duration = start_time.elapsed();
    println!("   Parse performance: {} messages/sec", 
             1000.0 / parse_duration.as_secs_f64());
    
    // Test validation performance
    let test_message = crate::AnchorMessage {
        anchor_id: 999,
        timestamp_ms: current_time,
        position: GeodeticPosition {
            latitude: 32.0,
            longitude: -117.0,
            depth: 10.0,
        },
        signal_quality: 80,
        message_sequence: 1,
        version: MessageVersion::V1,
    };
    
    let start_time = std::time::Instant::now();
    for _ in 0..1000 {
        let _ = validator.validate_messages(vec![test_message.clone()]);
    }
    
    let validate_duration = start_time.elapsed();
    println!("   Validation performance: {} messages/sec", 
             1000.0 / validate_duration.as_secs_f64());
    
    println!("\n=== MESSAGE PARSING AND VALIDATION DEMO COMPLETE ===");
}

// Configuration management demo
pub fn configuration_management_demo() {
    use crate::{ConfigurationManager, SystemConfig, AnchorConfig, ParameterUpdates};
    use crate::configuration::GeodeticPosition;
    
    println!("=== CONFIGURATION MANAGEMENT DEMO ===\n");
    
    // Create configuration manager
    let mut config_manager = ConfigurationManager::new();
    
    println!("1. INITIAL SYSTEM CONFIGURATION");
    let initial_summary = config_manager.get_parameter_summary();
    println!("   Sound speed: {:.1} m/s", initial_summary.sound_speed_ms);
    println!("   Position timeout: {} ms", initial_summary.position_timeout_ms);
    println!("   Accuracy threshold: {:.1} m", initial_summary.accuracy_threshold_m);
    println!("   Max anchor age: {} ms", initial_summary.max_anchor_age_ms);
    println!("   Water temperature: {:.1}°C", initial_summary.water_temperature_c);
    println!("   Water salinity: {:.1} PSU", initial_summary.water_salinity_psu);
    println!("   Water pressure: {:.1} bar", initial_summary.water_pressure_bar);
    println!("   Auto sound speed correction: {}", initial_summary.auto_sound_speed_correction);
    println!("   Corrected sound speed: {:.1} m/s", initial_summary.corrected_sound_speed_ms);
    
    println!("\n2. ADDING ANCHOR CONFIGURATIONS");
    
    // Add test anchors
    let anchors = vec![
        AnchorConfig {
            id: 101,
            position: GeodeticPosition {
                latitude: 32.123456,
                longitude: -117.654321,
                depth: 10.0,
            },
            max_range_m: 1000.0,
            enabled: true,
            quality_weight: 1.0,
            timeout_override_ms: None,
            reference_signal_strength: Some(85),
            range_calibration_offset_m: 0.0,
        },
        AnchorConfig {
            id: 102,
            position: GeodeticPosition {
                latitude: 32.124000,
                longitude: -117.655000,
                depth: 12.0,
            },
            max_range_m: 1200.0,
            enabled: true,
            quality_weight: 0.9,
            timeout_override_ms: Some(250),
            reference_signal_strength: Some(80),
            range_calibration_offset_m: 0.5,
        },
        AnchorConfig {
            id: 103,
            position: GeodeticPosition {
                latitude: 32.125000,
                longitude: -117.656000,
                depth: 8.0,
            },
            max_range_m: 800.0,
            enabled: true,
            quality_weight: 0.8,
            timeout_override_ms: None,
            reference_signal_strength: Some(75),
            range_calibration_offset_m: -0.2,
        },
        AnchorConfig {
            id: 104,
            position: GeodeticPosition {
                latitude: 32.126000,
                longitude: -117.657000,
                depth: 15.0,
            },
            max_range_m: 1500.0,
            enabled: true,
            quality_weight: 1.0,
            timeout_override_ms: None,
            reference_signal_strength: Some(90),
            range_calibration_offset_m: 0.0,
        },
    ];
    
    for anchor in anchors {
        match config_manager.set_anchor_config(anchor.clone()) {
            Ok(_) => println!("   Added anchor {}: ({:.6}, {:.6}, {:.1}m)", 
                             anchor.id, anchor.position.latitude, anchor.position.longitude, anchor.position.depth),
            Err(e) => println!("   Failed to add anchor {}: {}", anchor.id, e),
        }
    }
    
    let updated_summary = config_manager.get_parameter_summary();
    println!("   Total anchors: {}", updated_summary.total_anchor_count);
    println!("   Enabled anchors: {}", updated_summary.enabled_anchor_count);
    
    println!("\n3. RUNTIME PARAMETER ADJUSTMENTS");
    
    // Test individual parameter updates
    println!("   Updating sound speed to 1520 m/s...");
    match config_manager.set_sound_speed(1520.0) {
        Ok(old_value) => println!("     Changed from {:.1} to {:.1} m/s", old_value, 1520.0),
        Err(e) => println!("     Failed: {}", e),
    }
    
    println!("   Updating water temperature to 18°C...");
    match config_manager.set_water_temperature(18.0) {
        Ok(old_value) => println!("     Changed from {:.1} to {:.1}°C", old_value, 18.0),
        Err(e) => println!("     Failed: {}", e),
    }
    
    println!("   Updating position timeout to 300ms...");
    match config_manager.set_position_timeout(300) {
        Ok(old_value) => println!("     Changed from {} to {} ms", old_value, 300),
        Err(e) => println!("     Failed: {}", e),
    }
    
    // Show corrected sound speed calculation
    let corrected_speed = config_manager.calculate_corrected_sound_speed();
    println!("   Corrected sound speed: {:.1} m/s", corrected_speed);
    
    println!("\n4. BATCH PARAMETER UPDATES");
    
    // Create batch update
    let batch_updates = ParameterUpdates::new()
        .with_sound_speed(1530.0)
        .with_accuracy_threshold(1.5)
        .with_water_salinity(30.0)
        .with_anchor_quality_weight(102, 0.95)
        .with_anchor_enabled(103, false);
    
    match config_manager.update_parameters(batch_updates) {
        Ok(result) => {
            println!("   Batch update completed:");
            println!("     Applied updates: {}", result.applied_updates.len());
            println!("     Failed updates: {}", result.failed_updates.len());
            println!("     Total updates: {}", result.total_updates);
            
            for update in &result.applied_updates {
                println!("       ✓ {}", update);
            }
            
            for failure in &result.failed_updates {
                println!("       ✗ {}", failure);
            }
        }
        Err(e) => println!("   Batch update failed: {}", e),
    }
    
    println!("\n5. ANCHOR MANAGEMENT");
    
    let enabled_anchors = config_manager.get_enabled_anchors();
    println!("   Enabled anchors: {:?}", enabled_anchors);
    
    // Try to disable too many anchors
    println!("   Attempting to disable anchor 101 (should fail - would go below minimum)...");
    match config_manager.set_anchor_enabled(101, false) {
        Ok(_) => println!("     Successfully disabled anchor 101"),
        Err(e) => println!("     Failed as expected: {}", e),
    }
    
    // Re-enable anchor 103
    println!("   Re-enabling anchor 103...");
    match config_manager.set_anchor_enabled(103, true) {
        Ok(old_value) => println!("     Changed from {} to {}", old_value, true),
        Err(e) => println!("     Failed: {}", e),
    }
    
    println!("\n6. CONFIGURATION VALIDATION");
    
    // Test system geometry validation
    match config_manager.validate_system_geometry() {
        Ok(validation) => {
            println!("   System geometry validation:");
            println!("     Valid: {}", validation.is_valid);
            println!("     Errors: {}", validation.errors.len());
            println!("     Warnings: {}", validation.warnings.len());
            println!("     Suggestions: {}", validation.suggestions.len());
            
            for warning in &validation.warnings {
                println!("       Warning: {}", warning);
            }
            
            for suggestion in &validation.suggestions {
                println!("       Suggestion: {}", suggestion);
            }
        }
        Err(e) => println!("   Validation failed: {}", e),
    }
    
    println!("\n7. CONFIGURATION SNAPSHOT AND ROLLBACK");
    
    // Create snapshot
    let snapshot = config_manager.create_snapshot();
    println!("   Created configuration snapshot");
    
    // Make some changes
    let _ = config_manager.set_sound_speed(1600.0);
    let _ = config_manager.set_water_temperature(25.0);
    println!("   Made temporary changes:");
    println!("     Sound speed: {:.1} m/s", config_manager.get_sound_speed());
    println!("     Water temperature: {:.1}°C", config_manager.get_water_temperature());
    
    // Restore from snapshot
    config_manager.restore_from_snapshot(snapshot);
    println!("   Restored from snapshot:");
    println!("     Sound speed: {:.1} m/s", config_manager.get_sound_speed());
    println!("     Water temperature: {:.1}°C", config_manager.get_water_temperature());
    
    println!("\n8. CONFIGURATION PERSISTENCE");
    
    // Save configuration to file
    match config_manager.save_to_file("demo_config.json") {
        Ok(_) => {
            println!("   Configuration saved to demo_config.json");
            
            // Load configuration from file
            match ConfigurationManager::from_file("demo_config.json") {
                Ok(loaded_manager) => {
                    let loaded_summary = loaded_manager.get_parameter_summary();
                    println!("   Configuration loaded successfully:");
                    println!("     Sound speed: {:.1} m/s", loaded_summary.sound_speed_ms);
                    println!("     Total anchors: {}", loaded_summary.total_anchor_count);
                    println!("     Enabled anchors: {}", loaded_summary.enabled_anchor_count);
                }
                Err(e) => println!("   Failed to load configuration: {}", e),
            }
        }
        Err(e) => println!("   Failed to save configuration: {}", e),
    }
    
    println!("\n9. FINAL PARAMETER SUMMARY");
    
    let final_summary = config_manager.get_parameter_summary();
    println!("   System Configuration:");
    println!("     Sound speed: {:.1} m/s", final_summary.sound_speed_ms);
    println!("     Corrected sound speed: {:.1} m/s", final_summary.corrected_sound_speed_ms);
    println!("     Position timeout: {} ms", final_summary.position_timeout_ms);
    println!("     Accuracy threshold: {:.1} m", final_summary.accuracy_threshold_m);
    println!("     Max anchor age: {} ms", final_summary.max_anchor_age_ms);
    println!("   Environmental Parameters:");
    println!("     Water temperature: {:.1}°C", final_summary.water_temperature_c);
    println!("     Water salinity: {:.1} PSU", final_summary.water_salinity_psu);
    println!("     Water pressure: {:.1} bar", final_summary.water_pressure_bar);
    println!("     Auto correction: {}", final_summary.auto_sound_speed_correction);
    println!("   Anchor Status:");
    println!("     Total anchors: {}", final_summary.total_anchor_count);
    println!("     Enabled anchors: {}", final_summary.enabled_anchor_count);
    
    // Show individual anchor status
    for anchor_id in [101, 102, 103, 104] {
        if let Some(enabled) = config_manager.is_anchor_enabled(anchor_id) {
            let weight = config_manager.get_anchor_quality_weight(anchor_id).unwrap_or(0.0);
            println!("     Anchor {}: {} (weight: {:.2})", 
                     anchor_id, 
                     if enabled { "enabled" } else { "disabled" }, 
                     weight);
        }
    }
    
    println!("\n10. BEST PRACTICES FOR CONFIGURATION MANAGEMENT");
    println!("   1. Always validate configuration changes before applying");
    println!("   2. Use batch updates for multiple related parameter changes");
    println!("   3. Create snapshots before making significant changes");
    println!("   4. Monitor system geometry when adding/removing anchors");
    println!("   5. Apply environmental corrections for accurate positioning");
    println!("   6. Regularly save configuration to persistent storage");
    println!("   7. Use appropriate timeout and threshold values for your environment");
    println!("   8. Maintain minimum anchor count for reliable positioning");
    
    println!("\n=== CONFIGURATION MANAGEMENT DEMO COMPLETE ===");
}

// Helper function to create test V1 message
fn create_test_v1_message(anchor_id: u16, timestamp: u64, lat: f64, lon: f64, depth: f64, quality: u8, sequence: u16) -> RawMessage {
    let parser = MessageParser::new();
    let mut data = Vec::new();
    
    data.push(1u8); // Version V1
    data.extend_from_slice(&anchor_id.to_le_bytes());
    data.extend_from_slice(&timestamp.to_le_bytes());
    data.extend_from_slice(&lat.to_le_bytes());
    data.extend_from_slice(&lon.to_le_bytes());
    data.extend_from_slice(&depth.to_le_bytes());
    data.push(quality);
    data.extend_from_slice(&sequence.to_le_bytes());
    
    // Calculate and append checksum
    let checksum = parser.calculate_checksum(&data);
    data.extend_from_slice(&checksum.to_le_bytes());
    
    RawMessage {
        data,
        timestamp_received: timestamp,
        transceiver_id: 1,
    }
}

// Helper function to create test V2 message
fn create_test_v2_message(anchor_id: u16, timestamp: u64, lat: f64, lon: f64, depth: f64, quality: u8, sequence: u16, received_time: u64) -> RawMessage {
    let parser = MessageParser::new();
    let mut data = Vec::new();
    
    data.push(2u8); // Version V2
    data.extend_from_slice(&anchor_id.to_le_bytes());
    
    let timestamp_rel = (received_time - timestamp) as u32;
    data.extend_from_slice(&timestamp_rel.to_le_bytes());
    
    let lat_micro = (lat * 1_000_000.0) as i32;
    let lon_micro = (lon * 1_000_000.0) as i32;
    let depth_mm = (depth * 1000.0) as u16;
    
    data.extend_from_slice(&lat_micro.to_le_bytes());
    data.extend_from_slice(&lon_micro.to_le_bytes());
    data.extend_from_slice(&depth_mm.to_le_bytes());
    data.push(quality);
    data.extend_from_slice(&sequence.to_le_bytes());
    
    // Calculate and append checksum
    let checksum = parser.calculate_checksum(&data);
    data.extend_from_slice(&checksum.to_le_bytes());
    
    RawMessage {
        data,
        timestamp_received: received_time,
        transceiver_id: 1,
    }
}

// Error handling and graceful degradation demo
pub fn error_handling_graceful_degradation_demo() {
    use crate::{ErrorReporter, GracefulDegradationManager, SystemHealthMonitor};
    use crate::error_handling::{PositioningError, ErrorContext, SystemState, PositioningMode, PerformanceMetrics, EnvironmentalConditions};
    use crate::graceful_degradation::{SystemHealth, PositioningCapability, DegradationAction};
    
    println!("=== ERROR HANDLING AND GRACEFUL DEGRADATION DEMO ===\n");
    
    // Create error reporter and degradation manager
    let mut error_reporter = ErrorReporter::new();
    let mut degradation_manager = GracefulDegradationManager::new();
    let mut health_monitor = SystemHealthMonitor::new();
    
    println!("1. ERROR CLASSIFICATION AND REPORTING");
    
    // Create sample error context
    let error_context = ErrorContext {
        timestamp_ms: std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis() as u64,
        system_state: SystemState {
            positioning_mode: PositioningMode::FullPrecision3D,
            anchor_count: 4,
            last_position_update_ms: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_millis() as u64 - 1000,
            computation_load: 0.6,
            memory_usage_bytes: 2 * 1024 * 1024, // 2MB
            uptime_ms: 3600 * 1000, // 1 hour
        },
        active_anchors: vec![101, 102, 103, 104],
        last_successful_position: Some((32.123456, -117.654321, 15.5)),
        performance_metrics: PerformanceMetrics {
            avg_computation_time_ms: 45.0,
            max_computation_time_ms: 120.0,
            position_update_rate_hz: 5.0,
            accuracy_estimate_m: 1.2,
            successful_positions: 1800,
            failed_positions: 12,
        },
        environmental_conditions: EnvironmentalConditions {
            sound_speed_ms: 1520.0,
            estimated_depth_m: 25.0,
            signal_noise_level: 0.15,
            multipath_detected: false,
            temperature_c: Some(18.0),
            pressure_bar: Some(2.5),
        },
    };
    
    // Report various types of errors
    let errors_to_report = vec![
        PositioningError::InsufficientAnchors {
            available: 2,
            required: 3,
            anchor_ids: vec![101, 102],
        },
        PositioningError::DegenerateGeometry {
            condition_number: 1250.0,
            anchor_positions: vec![
                (101, 32.123, -117.654, 10.0),
                (102, 32.124, -117.655, 10.1),
                (103, 32.125, -117.656, 10.2),
            ],
            geometry_type: crate::error_handling::GeometryIssue::Collinear,
        },
        PositioningError::AnchorTimeout {
            anchor_id: 103,
            last_seen_ms: error_context.timestamp_ms - 45000,
            timeout_threshold_ms: 30000,
        },
        PositioningError::ComputationFailure {
            operation: "Matrix inversion".to_string(),
            error_code: crate::error_handling::ComputationErrorCode::NumericalInstability,
            context: "Singular matrix in trilateration".to_string(),
        },
    ];
    
    let mut error_ids = Vec::new();
    for error in errors_to_report {
        let error_id = error_reporter.report_error(error, error_context.clone());
        error_ids.push(error_id);
        println!("   Reported error ID: {}", error_id);
    }
    
    println!("\n2. ERROR STATISTICS AND REPORTING");
    
    let error_report = error_reporter.generate_error_report();
    println!("   Total errors reported: {}", error_report.total_errors);
    println!("   Unresolved errors: {}", error_report.unresolved_errors);
    println!("   Critical errors: {}", error_report.critical_errors);
    
    println!("\n   Error type distribution:");
    for (error_type, count) in &error_report.error_type_distribution {
        println!("     - {}: {}", error_type, count);
    }
    
    // Simulate error resolution
    println!("\n3. ERROR RESOLUTION TRACKING");
    
    if let Some(&first_error_id) = error_ids.first() {
        error_reporter.update_resolution_status(
            first_error_id, 
            crate::error_handling::ResolutionStatus::Resolved
        );
        println!("   Marked error {} as resolved", first_error_id);
        
        error_reporter.add_debug_data(
            first_error_id,
            "resolution_method".to_string(),
            "Waited for additional anchor signals".to_string(),
        );
        println!("   Added debug data to error {}", first_error_id);
    }
    
    println!("\n4. SYSTEM HEALTH MONITORING");
    
    // Create test anchor messages for health monitoring
    let current_time = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_millis() as u64;
    
    let test_messages = vec![
        AnchorMessage {
            anchor_id: 101,
            timestamp_ms: current_time - 1000,
            position: GeodeticPosition {
                latitude: 32.123456,
                longitude: -117.654321,
                depth: 10.0,
            },
            signal_quality: 180,
            message_sequence: 1001,
            version: MessageVersion::V1,
        },
        AnchorMessage {
            anchor_id: 102,
            timestamp_ms: current_time - 800,
            position: GeodeticPosition {
                latitude: 32.124000,
                longitude: -117.655000,
                depth: 12.0,
            },
            signal_quality: 165,
            message_sequence: 1002,
            version: MessageVersion::V1,
        },
        AnchorMessage {
            anchor_id: 103,
            timestamp_ms: current_time - 1200,
            position: GeodeticPosition {
                latitude: 32.125000,
                longitude: -117.656000,
                depth: 8.0,
            },
            signal_quality: 95, // Poor quality
            message_sequence: 1003,
            version: MessageVersion::V1,
        },
    ];
    
    // Update health monitor with messages
    for message in &test_messages {
        health_monitor.update_anchor_health(message);
    }
    
    // Perform health check
    let system_health = health_monitor.perform_health_check();
    println!("   System health: {:?}", system_health);
    
    // Get health report
    let health_report = health_monitor.get_health_report();
    println!("   Active anchors: {}", health_report.anchor_health.len());
    println!("   System uptime: {:.1} minutes", health_report.system_uptime_ms as f64 / 60000.0);
    
    println!("\n   Anchor health status:");
    for (anchor_id, anchor_health) in &health_report.anchor_health {
        println!("     - Anchor {}: {:?} (reliability: {:.2})", 
                 anchor_id, anchor_health.status, anchor_health.reliability_score);
    }
    
    println!("\n5. GRACEFUL DEGRADATION");
    
    // Test degradation scenarios
    println!("   Testing degradation scenarios:");
    
    // Scenario 1: Normal operation with 4 anchors
    let good_messages = vec![
        test_messages[0].clone(),
        test_messages[1].clone(),
        AnchorMessage {
            anchor_id: 104,
            timestamp_ms: current_time - 500,
            position: GeodeticPosition {
                latitude: 32.126000,
                longitude: -117.657000,
                depth: 15.0,
            },
            signal_quality: 200,
            message_sequence: 1004,
            version: MessageVersion::V1,
        },
        AnchorMessage {
            anchor_id: 105,
            timestamp_ms: current_time - 300,
            position: GeodeticPosition {
                latitude: 32.127000,
                longitude: -117.658000,
                depth: 18.0,
            },
            signal_quality: 190,
            message_sequence: 1005,
            version: MessageVersion::V1,
        },
    ];
    
    let decision1 = degradation_manager.update(&good_messages);
    println!("\n     Scenario 1 - Normal operation (4+ anchors):");
    println!("       Action: {:?}", decision1.action);
    println!("       System health: {:?}", decision1.system_health);
    println!("       Positioning capability: {:?}", decision1.positioning_capability);
    println!("       Confidence: {:.2}", decision1.confidence);
    
    // Scenario 2: Reduced anchors (3 anchors)
    let reduced_messages = vec![
        test_messages[0].clone(),
        test_messages[1].clone(),
        test_messages[2].clone(),
    ];
    
    let decision2 = degradation_manager.update(&reduced_messages);
    println!("\n     Scenario 2 - Reduced anchors (3 anchors):");
    println!("       Action: {:?}", decision2.action);
    println!("       System health: {:?}", decision2.system_health);
    println!("       Positioning capability: {:?}", decision2.positioning_capability);
    println!("       Confidence: {:.2}", decision2.confidence);
    
    // Scenario 3: Minimal anchors (2 anchors)
    let minimal_messages = vec![
        test_messages[0].clone(),
        test_messages[1].clone(),
    ];
    
    let decision3 = degradation_manager.update(&minimal_messages);
    println!("\n     Scenario 3 - Minimal anchors (2 anchors):");
    println!("       Action: {:?}", decision3.action);
    println!("       System health: {:?}", decision3.system_health);
    println!("       Positioning capability: {:?}", decision3.positioning_capability);
    println!("       Confidence: {:.2}", decision3.confidence);
    
    // Scenario 4: Single anchor
    let single_message = vec![test_messages[0].clone()];
    
    let decision4 = degradation_manager.update(&single_message);
    println!("\n     Scenario 4 - Single anchor:");
    println!("       Action: {:?}", decision4.action);
    println!("       System health: {:?}", decision4.system_health);
    println!("       Positioning capability: {:?}", decision4.positioning_capability);
    println!("       Confidence: {:.2}", decision4.confidence);
    
    // Scenario 5: No anchors
    let no_messages: Vec<AnchorMessage> = vec![];
    
    let decision5 = degradation_manager.update(&no_messages);
    println!("\n     Scenario 5 - No anchors:");
    println!("       Action: {:?}", decision5.action);
    println!("       System health: {:?}", decision5.system_health);
    println!("       Positioning capability: {:?}", decision5.positioning_capability);
    println!("       Confidence: {:.2}", decision5.confidence);
    
    println!("\n6. SYSTEM STATUS SUMMARY");
    
    let system_status = degradation_manager.get_status();
    println!("   Current algorithm: {:?}", system_status.current_algorithm);
    println!("   System health: {:?}", system_status.system_health);
    println!("   Active anchors: {}", system_status.active_anchors);
    
    if let Some(last_pos) = &system_status.last_known_position {
        println!("   Last known position: ({:.6}, {:.6}, {:.2}m)", 
                 last_pos.latitude, last_pos.longitude, last_pos.depth);
    } else {
        println!("   Last known position: None");
    }
    
    println!("\n7. RECOVERY DEMONSTRATION");
    
    // Simulate recovery by providing good conditions again
    println!("   Simulating recovery with improved conditions...");
    
    let recovery_messages = vec![
        AnchorMessage {
            anchor_id: 101,
            timestamp_ms: current_time,
            position: GeodeticPosition {
                latitude: 32.123456,
                longitude: -117.654321,
                depth: 10.0,
            },
            signal_quality: 220, // Excellent quality
            message_sequence: 1101,
            version: MessageVersion::V1,
        },
        AnchorMessage {
            anchor_id: 102,
            timestamp_ms: current_time - 100,
            position: GeodeticPosition {
                latitude: 32.124000,
                longitude: -117.655000,
                depth: 12.0,
            },
            signal_quality: 210,
            message_sequence: 1102,
            version: MessageVersion::V1,
        },
        AnchorMessage {
            anchor_id: 103,
            timestamp_ms: current_time - 200,
            position: GeodeticPosition {
                latitude: 32.125000,
                longitude: -117.656000,
                depth: 8.0,
            },
            signal_quality: 200,
            message_sequence: 1103,
            version: MessageVersion::V1,
        },
        AnchorMessage {
            anchor_id: 104,
            timestamp_ms: current_time - 150,
            position: GeodeticPosition {
                latitude: 32.126000,
                longitude: -117.657000,
                depth: 15.0,
            },
            signal_quality: 215,
            message_sequence: 1104,
            version: MessageVersion::V1,
        },
        AnchorMessage {
            anchor_id: 105,
            timestamp_ms: current_time - 50,
            position: GeodeticPosition {
                latitude: 32.127000,
                longitude: -117.658000,
                depth: 18.0,
            },
            signal_quality: 225,
            message_sequence: 1105,
            version: MessageVersion::V1,
        },
    ];
    
    let recovery_decision = degradation_manager.update(&recovery_messages);
    println!("   Recovery decision:");
    println!("     Action: {:?}", recovery_decision.action);
    println!("     System health: {:?}", recovery_decision.system_health);
    println!("     Positioning capability: {:?}", recovery_decision.positioning_capability);
    println!("     Confidence: {:.2}", recovery_decision.confidence);
    
    println!("\n8. ERROR HANDLING BEST PRACTICES");
    println!("   1. Classify errors by severity and impact on positioning");
    println!("   2. Implement automatic recovery strategies for common issues");
    println!("   3. Monitor system health continuously");
    println!("   4. Gracefully degrade functionality when conditions deteriorate");
    println!("   5. Provide clear diagnostic information for troubleshooting");
    println!("   6. Track error patterns to identify systemic issues");
    println!("   7. Implement fallback positioning modes for reliability");
    println!("   8. Maintain position history for dead reckoning");
    
    println!("\n=== ERROR HANDLING AND GRACEFUL DEGRADATION DEMO COMPLETE ===");
}