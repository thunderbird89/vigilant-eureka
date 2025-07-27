use serde::{Deserialize, Serialize};
use std::path::PathBuf;
use uuid::Uuid;
use crate::{ScenarioType, MovementPattern, ExportFormat};
use crate::batch::{BatchConfig, BatchOperation, BatchSettings};
use crate::cli::ScenarioTemplate;

/// Generate scenario configuration based on template type
pub fn generate_scenario_config(
    template: ScenarioTemplate,
    beacon_count: Option<usize>,
    area_size: Option<f64>,
    duration: Option<u64>,
    _include_receiver: bool,
) -> Result<ScenarioConfig, String> {
    match template {
        ScenarioTemplate::Performance => generate_performance_scenario(beacon_count, area_size, duration),
        ScenarioTemplate::Accuracy => generate_accuracy_scenario(beacon_count, area_size, duration),
        ScenarioTemplate::Stress => generate_stress_scenario(beacon_count, area_size, duration),
        ScenarioTemplate::Integration => generate_integration_scenario(beacon_count, area_size, duration),
        ScenarioTemplate::Custom => generate_custom_scenario(beacon_count, area_size, duration),
        ScenarioTemplate::All => Err("Use generate_all_scenarios function for 'all' template".to_string()),
    }
}

/// Configuration for a complete test scenario
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScenarioConfig {
    pub name: String,
    pub description: String,
    pub batch_config: BatchConfig,
    pub receiver_config: Option<ReceiverConfig>,
    pub expected_results: Option<ExpectedResults>,
}

/// Virtual receiver configuration for testing
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReceiverConfig {
    pub channel: String,
    pub position: [f64; 3], // [lat, lon, depth]
    pub update_interval: u32, // milliseconds
    pub positioning_algorithm: String,
    pub output_file: Option<PathBuf>,
}

/// Expected test results for validation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExpectedResults {
    pub min_beacons_detected: usize,
    pub max_position_error: f64, // meters
    pub min_update_rate: f64, // Hz
    pub test_duration: u64, // seconds
}

/// Generate performance testing scenario
fn generate_performance_scenario(
    beacon_count: Option<usize>,
    area_size: Option<f64>,
    duration: Option<u64>,
) -> Result<ScenarioConfig, String> {
    let count = beacon_count.unwrap_or(10);
    let area = area_size.unwrap_or(1000.0);
    let test_duration = duration.unwrap_or(300); // 5 minutes

    let mut operations = Vec::new();

    // Create multiple scenarios for performance testing
    operations.push(BatchOperation::CreateScenario {
        scenario_type: ScenarioType::Grid,
        count: count / 2,
        spacing: area / 4.0,
        center: [32.0, 45.0, 10.0],
        start_all: false,
        interval: 1000, // High frequency for performance testing
    });

    operations.push(BatchOperation::CreateScenario {
        scenario_type: ScenarioType::Triangle,
        count: 3,
        spacing: area / 2.0,
        center: [32.1, 45.1, 15.0],
        start_all: false,
        interval: 2000,
    });

    // Add some moving beacons for dynamic testing
    for i in 0..3 {
        operations.push(BatchOperation::CreateBeacon {
            id: None,
            position: [32.0 + i as f64 * 0.01, 45.0 + i as f64 * 0.01, 20.0],
            interval: 1500,
            movement: MovementPattern::Linear { speed_m_per_s: 2.0, bearing_deg: 45.0 + i as f64 * 30.0 },
            start: false,
            config_file: None,
        });
    }

    // Start all beacons
    operations.push(BatchOperation::StartAllBeacons);

    // Run for specified duration
    operations.push(BatchOperation::Wait { duration: test_duration });

    // Export performance data
    operations.push(BatchOperation::ExportLogs {
        output: PathBuf::from("performance_test_results.json"),
        format: ExportFormat::Json,
        duration: test_duration + 60,
        all_time: false,
        beacon: None,
        include_messages: true,
    });

    // Cleanup
    operations.push(BatchOperation::StopAllBeacons { remove: true });

    let batch_config = BatchConfig {
        operations,
        settings: BatchSettings {
            timeout_seconds: test_duration + 120,
            continue_on_error: false,
            verbose: true,
        },
    };

    Ok(ScenarioConfig {
        name: "Performance Test".to_string(),
        description: format!("High-throughput performance test with {} beacons over {}m area for {}s", 
                           count + 3, area, test_duration),
        batch_config,
        receiver_config: Some(ReceiverConfig {
            channel: "default".to_string(),
            position: [32.05, 45.05, 5.0],
            update_interval: 1000,
            positioning_algorithm: "trilateration".to_string(),
            output_file: Some(PathBuf::from("performance_receiver_output.json")),
        }),
        expected_results: Some(ExpectedResults {
            min_beacons_detected: count,
            max_position_error: 10.0,
            min_update_rate: 0.5,
            test_duration,
        }),
    })
}

/// Generate accuracy validation scenario
fn generate_accuracy_scenario(
    beacon_count: Option<usize>,
    area_size: Option<f64>,
    duration: Option<u64>,
) -> Result<ScenarioConfig, String> {
    let count = beacon_count.unwrap_or(4);
    let area = area_size.unwrap_or(200.0);
    let test_duration = duration.unwrap_or(180); // 3 minutes

    let mut operations = Vec::new();

    // Create precise geometric arrangement for accuracy testing
    operations.push(BatchOperation::CreateScenario {
        scenario_type: ScenarioType::Square,
        count: 4,
        spacing: area / 2.0,
        center: [32.123456, 45.654321, 10.0], // Precise coordinates
        start_all: false,
        interval: 5000, // Standard interval for accuracy
    });

    // Add additional beacons if requested
    if count > 4 {
        for i in 0..(count - 4) {
            let angle = (i as f64) * (360.0 / (count - 4) as f64);
            let radius = area / 3.0;
            let lat_offset = radius * angle.to_radians().cos() / 111320.0; // Approximate degrees per meter
            let lon_offset = radius * angle.to_radians().sin() / (111320.0 * 32.123456_f64.to_radians().cos());
            
            operations.push(BatchOperation::CreateBeacon {
                id: None,
                position: [32.123456 + lat_offset, 45.654321 + lon_offset, 12.0],
                interval: 5000,
                movement: MovementPattern::Stationary,
                start: false,
                config_file: None,
            });
        }
    }

    // Start all beacons
    operations.push(BatchOperation::StartAllBeacons);

    // Run for specified duration
    operations.push(BatchOperation::Wait { duration: test_duration });

    // Export accuracy data
    operations.push(BatchOperation::ExportLogs {
        output: PathBuf::from("accuracy_test_results.json"),
        format: ExportFormat::Json,
        duration: test_duration + 30,
        all_time: false,
        beacon: None,
        include_messages: true,
    });

    // Cleanup
    operations.push(BatchOperation::StopAllBeacons { remove: true });

    let batch_config = BatchConfig {
        operations,
        settings: BatchSettings {
            timeout_seconds: test_duration + 60,
            continue_on_error: false,
            verbose: true,
        },
    };

    Ok(ScenarioConfig {
        name: "Accuracy Validation".to_string(),
        description: format!("Precision positioning test with {} beacons in {}m area for {}s", 
                           count, area, test_duration),
        batch_config,
        receiver_config: Some(ReceiverConfig {
            channel: "default".to_string(),
            position: [32.123456, 45.654321, 5.0], // Known position for accuracy validation
            update_interval: 2000,
            positioning_algorithm: "trilateration".to_string(),
            output_file: Some(PathBuf::from("accuracy_receiver_output.json")),
        }),
        expected_results: Some(ExpectedResults {
            min_beacons_detected: count,
            max_position_error: 2.0, // High accuracy requirement
            min_update_rate: 0.4,
            test_duration,
        }),
    })
}

/// Generate stress testing scenario
fn generate_stress_scenario(
    beacon_count: Option<usize>,
    area_size: Option<f64>,
    duration: Option<u64>,
) -> Result<ScenarioConfig, String> {
    let count = beacon_count.unwrap_or(50);
    let area = area_size.unwrap_or(2000.0);
    let test_duration = duration.unwrap_or(600); // 10 minutes

    let mut operations = Vec::new();

    // Create multiple overlapping scenarios for stress testing
    let scenarios_per_type = count / 4;
    
    operations.push(BatchOperation::CreateScenario {
        scenario_type: ScenarioType::Grid,
        count: scenarios_per_type,
        spacing: area / 8.0,
        center: [32.0, 45.0, 10.0],
        start_all: false,
        interval: 500, // Very high frequency
    });

    operations.push(BatchOperation::CreateScenario {
        scenario_type: ScenarioType::Line,
        count: scenarios_per_type,
        spacing: area / 10.0,
        center: [32.1, 45.0, 15.0],
        start_all: false,
        interval: 750,
    });

    operations.push(BatchOperation::CreateScenario {
        scenario_type: ScenarioType::Triangle,
        count: scenarios_per_type,
        spacing: area / 6.0,
        center: [32.0, 45.1, 20.0],
        start_all: false,
        interval: 1000,
    });

    // Add many individual beacons with various movement patterns
    let remaining_beacons = count - (scenarios_per_type * 3);
    for i in 0..remaining_beacons {
        let movement = match i % 4 {
            0 => MovementPattern::Stationary,
            1 => MovementPattern::Linear { speed_m_per_s: 1.0, bearing_deg: (i as f64) * 10.0 },
            2 => MovementPattern::Circular { radius_m: 50.0, period_s: 120.0 },
            _ => MovementPattern::Random { max_speed_m_per_s: 0.5 },
        };

        operations.push(BatchOperation::CreateBeacon {
            id: None,
            position: [
                32.0 + (i as f64 * 0.001) % 0.1,
                45.0 + (i as f64 * 0.001) % 0.1,
                10.0 + (i as f64) % 30.0,
            ],
            interval: 500 + (i as u32 * 100) % 2000,
            movement,
            start: false,
            config_file: None,
        });
    }

    // Start all beacons gradually to simulate realistic deployment
    operations.push(BatchOperation::StartAllBeacons);

    // Run for extended duration
    operations.push(BatchOperation::Wait { duration: test_duration });

    // Export stress test data
    operations.push(BatchOperation::ExportLogs {
        output: PathBuf::from("stress_test_results.json"),
        format: ExportFormat::Json,
        duration: test_duration + 60,
        all_time: false,
        beacon: None,
        include_messages: false, // Exclude messages to reduce data size
    });

    // Cleanup
    operations.push(BatchOperation::StopAllBeacons { remove: true });

    let batch_config = BatchConfig {
        operations,
        settings: BatchSettings {
            timeout_seconds: test_duration + 180,
            continue_on_error: true, // Continue on errors for stress testing
            verbose: false, // Reduce output for stress testing
        },
    };

    Ok(ScenarioConfig {
        name: "Stress Test".to_string(),
        description: format!("High-load stress test with {} beacons over {}m area for {}s", 
                           count, area, test_duration),
        batch_config,
        receiver_config: Some(ReceiverConfig {
            channel: "default".to_string(),
            position: [32.05, 45.05, 5.0],
            update_interval: 5000, // Slower updates for stress testing
            positioning_algorithm: "trilateration".to_string(),
            output_file: Some(PathBuf::from("stress_receiver_output.json")),
        }),
        expected_results: Some(ExpectedResults {
            min_beacons_detected: count / 2, // Lower expectation for stress test
            max_position_error: 20.0,
            min_update_rate: 0.1,
            test_duration,
        }),
    })
}

/// Generate integration testing scenario
fn generate_integration_scenario(
    _beacon_count: Option<usize>,
    area_size: Option<f64>,
    duration: Option<u64>,
) -> Result<ScenarioConfig, String> {
    let area = area_size.unwrap_or(500.0);
    let test_duration = duration.unwrap_or(240); // 4 minutes

    let mut operations = Vec::new();

    // Phase 1: Basic setup
    operations.push(BatchOperation::CreateScenario {
        scenario_type: ScenarioType::Triangle,
        count: 3,
        spacing: area / 3.0,
        center: [32.0, 45.0, 10.0],
        start_all: true,
        interval: 5000,
    });

    operations.push(BatchOperation::Wait { duration: 30 });

    // Phase 2: Add dynamic beacons
    operations.push(BatchOperation::CreateBeacon {
        id: None,
        position: [32.01, 45.01, 15.0],
        interval: 3000,
        movement: MovementPattern::Linear { speed_m_per_s: 1.5, bearing_deg: 90.0 },
        start: true,
        config_file: None,
    });

    operations.push(BatchOperation::Wait { duration: 30 });

    // Phase 3: Test updates
    operations.push(BatchOperation::CreateBeacon {
        id: Some(Uuid::parse_str("550e8400-e29b-41d4-a716-446655440000").unwrap()),
        position: [32.02, 45.02, 20.0],
        interval: 4000,
        movement: MovementPattern::Stationary,
        start: true,
        config_file: None,
    });

    operations.push(BatchOperation::Wait { duration: 20 });

    operations.push(BatchOperation::UpdateBeacon {
        id: Uuid::parse_str("550e8400-e29b-41d4-a716-446655440000").unwrap(),
        position: Some([32.025, 45.025, 25.0]),
        interval: Some(2000),
        movement: Some(MovementPattern::Circular { radius_m: 30.0, period_s: 60.0 }),
        restart: true,
    });

    operations.push(BatchOperation::Wait { duration: 40 });

    // Phase 4: Test stop/start cycles
    operations.push(BatchOperation::StopBeacon {
        id: Uuid::parse_str("550e8400-e29b-41d4-a716-446655440000").unwrap(),
        remove: false,
    });

    operations.push(BatchOperation::Wait { duration: 20 });

    operations.push(BatchOperation::StartBeacon {
        id: Uuid::parse_str("550e8400-e29b-41d4-a716-446655440000").unwrap(),
    });

    operations.push(BatchOperation::Wait { duration: 30 });

    // Phase 5: Export intermediate results
    operations.push(BatchOperation::ExportLogs {
        output: PathBuf::from("integration_intermediate_results.json"),
        format: ExportFormat::Json,
        duration: 180,
        all_time: false,
        beacon: None,
        include_messages: true,
    });

    // Phase 6: Final testing
    operations.push(BatchOperation::Wait { duration: test_duration - 180 });

    // Final export
    operations.push(BatchOperation::ExportLogs {
        output: PathBuf::from("integration_final_results.json"),
        format: ExportFormat::Json,
        duration: test_duration + 30,
        all_time: false,
        beacon: None,
        include_messages: true,
    });

    // Cleanup
    operations.push(BatchOperation::StopAllBeacons { remove: true });

    let batch_config = BatchConfig {
        operations,
        settings: BatchSettings {
            timeout_seconds: test_duration + 120,
            continue_on_error: false,
            verbose: true,
        },
    };

    Ok(ScenarioConfig {
        name: "Integration Test".to_string(),
        description: format!("Multi-phase integration test over {}m area for {}s", area, test_duration),
        batch_config,
        receiver_config: Some(ReceiverConfig {
            channel: "default".to_string(),
            position: [32.01, 45.01, 5.0],
            update_interval: 2000,
            positioning_algorithm: "trilateration".to_string(),
            output_file: Some(PathBuf::from("integration_receiver_output.json")),
        }),
        expected_results: Some(ExpectedResults {
            min_beacons_detected: 4,
            max_position_error: 5.0,
            min_update_rate: 0.3,
            test_duration,
        }),
    })
}

/// Generate custom scenario based on parameters
fn generate_custom_scenario(
    beacon_count: Option<usize>,
    area_size: Option<f64>,
    duration: Option<u64>,
) -> Result<ScenarioConfig, String> {
    let count = beacon_count.unwrap_or(6);
    let area = area_size.unwrap_or(300.0);
    let test_duration = duration.unwrap_or(120);

    let mut operations = Vec::new();

    // Create main scenario
    operations.push(BatchOperation::CreateScenario {
        scenario_type: ScenarioType::Square,
        count: std::cmp::min(count, 4),
        spacing: area / 2.0,
        center: [32.0, 45.0, 10.0],
        start_all: false,
        interval: 5000,
    });

    // Add additional beacons if needed
    if count > 4 {
        for i in 0..(count - 4) {
            operations.push(BatchOperation::CreateBeacon {
                id: None,
                position: [32.0 + (i as f64 * 0.01), 45.0 + (i as f64 * 0.01), 15.0],
                interval: 5000,
                movement: MovementPattern::Stationary,
                start: false,
                config_file: None,
            });
        }
    }

    // Start all beacons
    operations.push(BatchOperation::StartAllBeacons);

    // Run for specified duration
    operations.push(BatchOperation::Wait { duration: test_duration });

    // Export results
    operations.push(BatchOperation::ExportLogs {
        output: PathBuf::from("custom_test_results.json"),
        format: ExportFormat::Json,
        duration: test_duration + 30,
        all_time: false,
        beacon: None,
        include_messages: true,
    });

    // Cleanup
    operations.push(BatchOperation::StopAllBeacons { remove: true });

    let batch_config = BatchConfig {
        operations,
        settings: BatchSettings {
            timeout_seconds: test_duration + 60,
            continue_on_error: false,
            verbose: true,
        },
    };

    Ok(ScenarioConfig {
        name: "Custom Test".to_string(),
        description: format!("Custom test scenario with {} beacons over {}m area for {}s", 
                           count, area, test_duration),
        batch_config,
        receiver_config: Some(ReceiverConfig {
            channel: "default".to_string(),
            position: [32.0, 45.0, 5.0],
            update_interval: 2000,
            positioning_algorithm: "trilateration".to_string(),
            output_file: Some(PathBuf::from("custom_receiver_output.json")),
        }),
        expected_results: Some(ExpectedResults {
            min_beacons_detected: count,
            max_position_error: 10.0,
            min_update_rate: 0.4,
            test_duration,
        }),
    })
}

/// Generate all standard scenario templates
pub fn generate_all_scenarios(
    output_dir: &PathBuf,
    beacon_count: Option<usize>,
    area_size: Option<f64>,
    duration: Option<u64>,
) -> Result<Vec<PathBuf>, String> {
    let templates = vec![
        ScenarioTemplate::Performance,
        ScenarioTemplate::Accuracy,
        ScenarioTemplate::Stress,
        ScenarioTemplate::Integration,
        ScenarioTemplate::Custom,
    ];

    let mut generated_files = Vec::new();

    for template in templates {
        let _config = generate_scenario_config(template.clone(), beacon_count, area_size, duration, true)?;
        let filename = format!("{}_scenario.json", 
                              match template {
                                  ScenarioTemplate::Performance => "performance",
                                  ScenarioTemplate::Accuracy => "accuracy",
                                  ScenarioTemplate::Stress => "stress",
                                  ScenarioTemplate::Integration => "integration",
                                  ScenarioTemplate::Custom => "custom",
                                  _ => "unknown",
                              });
        
        let file_path = output_dir.join(filename);
        generated_files.push(file_path);
    }

    Ok(generated_files)
}