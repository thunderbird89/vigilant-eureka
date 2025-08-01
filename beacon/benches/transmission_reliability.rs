use criterion::{black_box, criterion_group, criterion_main, Criterion, BenchmarkId};
use std::time::{Duration, SystemTime};
use uuid::Uuid;

use beacon::{BeaconController, BeaconConfig, MessageVersion};
use shared_positioning::{
    GpsConfig, PowerConfig, CommunicationConfig, MockGpsManager, MockPowerManager,
    MockCommunicationManager, MockTransceiverInterface, MessageBuilder, GeodeticPosition,
    GpsPosition, TransceiverInterface, GpsManager, TransmissionManager, TransmissionConfig,
    TransmissionPriority, TransmissionMessageVersion
};

fn create_test_beacon() -> BeaconController<MockGpsManager, MockPowerManager, MockCommunicationManager, MockTransceiverInterface> {
    let config = BeaconConfig {
        beacon_id: Uuid::new_v4(),
        transmission_interval_ms: 5000,
        message_version: MessageVersion::V3,
        gps_config: GpsConfig::default(),
        power_config: PowerConfig::default(),
        communication_config: CommunicationConfig::default(),
        emergency_config: beacon::EmergencyConfig::default(),
    };
    
    let mut gps_manager = MockGpsManager::with_test_positions(config.gps_config.clone()).unwrap();
    gps_manager.set_acquisition_delay(Duration::from_millis(10)); // Fast acquisition for benchmarks
    
    let power_manager = MockPowerManager::new();
    let comm_manager = MockCommunicationManager::new();
    let transceiver = MockTransceiverInterface::new();
    
    BeaconController::new(config, gps_manager, power_manager, comm_manager, transceiver).unwrap()
}

fn benchmark_message_building(c: &mut Criterion) {
    let mut group = c.benchmark_group("message_building");
    
    let beacon_id = Uuid::new_v4();
    let position = GeodeticPosition {
        latitude: 37.7749,
        longitude: -122.4194,
        altitude: 10.0,
    };
    let message_builder = MessageBuilder::new();
    
    group.bench_function("build_v1_message", |b| {
        b.iter(|| {
            black_box(message_builder.build_v1_message(
                beacon_id,
                position.clone(),
                85,
                123
            ).unwrap());
        });
    });
    
    group.bench_function("build_v2_message", |b| {
        b.iter(|| {
            black_box(message_builder.build_v2_message(
                beacon_id,
                position.clone(),
                87,
                124
            ).unwrap());
        });
    });
    
    group.bench_function("build_v3_message", |b| {
        b.iter(|| {
            black_box(message_builder.build_v3_message(
                beacon_id,
                position.clone(),
                89,
                125
            ).unwrap());
        });
    });
    
    group.finish();
}

fn benchmark_transmission_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("transmission_operations");
    
    group.bench_function("single_transmission", |b| {
        let mut beacon = create_test_beacon();
        beacon.gps_manager.start_acquisition().unwrap();
        beacon.gps_manager.update().unwrap(); // Acquire GPS lock
        
        b.iter(|| {
            black_box(beacon.handle_transmission().unwrap());
        });
    });
    
    group.bench_function("transmission_with_power_check", |b| {
        let mut beacon = create_test_beacon();
        beacon.gps_manager.start_acquisition().unwrap();
        beacon.gps_manager.update().unwrap();
        
        b.iter(|| {
            let _power_status = black_box(beacon.check_power_status());
            black_box(beacon.handle_transmission().unwrap());
        });
    });
    
    group.bench_function("transmission_sequence_increment", |b| {
        let mut beacon = create_test_beacon();
        beacon.gps_manager.start_acquisition().unwrap();
        beacon.gps_manager.update().unwrap();
        
        b.iter(|| {
            let old_sequence = beacon.transmission_sequence;
            black_box(beacon.handle_transmission().unwrap());
            assert!(beacon.transmission_sequence > old_sequence);
        });
    });
    
    group.finish();
}

fn benchmark_transmission_reliability(c: &mut Criterion) {
    let mut group = c.benchmark_group("transmission_reliability");
    
    let failure_rates = vec![0.0, 0.1, 0.2, 0.5];
    
    for failure_rate in failure_rates {
        group.bench_with_input(
            BenchmarkId::new("transmission_with_failures", (failure_rate * 100.0) as u32),
            &failure_rate,
            |b, &failure_rate| {
                let mut beacon = create_test_beacon();
                beacon.gps_manager.start_acquisition().unwrap();
                beacon.gps_manager.update().unwrap();
                
                // Set failure rate (1.0 - success_rate)
                beacon.transceiver.set_failure_rate(failure_rate);
                
                b.iter(|| {
                    // Attempt transmission, may fail based on failure rate
                    let result = beacon.handle_transmission();
                    black_box(result);
                });
            },
        );
    }
    
    group.finish();
}

fn benchmark_transmission_power_levels(c: &mut Criterion) {
    let mut group = c.benchmark_group("transmission_power_levels");
    
    let power_levels = vec![50, 100, 150, 200, 255];
    
    for power_level in power_levels {
        group.bench_with_input(
            BenchmarkId::new("transmission_power", power_level),
            &power_level,
            |b, &power_level| {
                let mut beacon = create_test_beacon();
                beacon.gps_manager.start_acquisition().unwrap();
                beacon.gps_manager.update().unwrap();
                
                b.iter(|| {
                    beacon.transceiver.set_transmission_power(power_level).unwrap();
                    black_box(beacon.handle_transmission().unwrap());
                    black_box(beacon.transceiver.get_transmission_status());
                });
            },
        );
    }
    
    group.finish();
}

fn benchmark_message_validation(c: &mut Criterion) {
    let mut group = c.benchmark_group("message_validation");
    
    let beacon_id = Uuid::new_v4();
    let position = GeodeticPosition {
        latitude: 37.7749,
        longitude: -122.4194,
        altitude: 10.0,
    };
    let message_builder = MessageBuilder::new();
    
    // Pre-build messages for validation benchmarks
    let v1_message = message_builder.build_v1_message(beacon_id, position.clone(), 85, 123).unwrap();
    let v2_message = message_builder.build_v2_message(beacon_id, position.clone(), 87, 124).unwrap();
    let v3_message = message_builder.build_v3_message(beacon_id, position, 89, 125).unwrap();
    
    group.bench_function("validate_v1_message", |b| {
        b.iter(|| {
            black_box(message_builder.validate_message(&v1_message).unwrap());
        });
    });
    
    group.bench_function("validate_v2_message", |b| {
        b.iter(|| {
            black_box(message_builder.validate_message(&v2_message).unwrap());
        });
    });
    
    group.bench_function("validate_v3_message", |b| {
        b.iter(|| {
            black_box(message_builder.validate_message(&v3_message).unwrap());
        });
    });
    
    group.finish();
}

fn benchmark_transmission_statistics(c: &mut Criterion) {
    let mut group = c.benchmark_group("transmission_statistics");
    
    group.bench_function("update_transmission_stats", |b| {
        let mut beacon = create_test_beacon();
        b.iter(|| {
            beacon.transmission_statistics.messages_sent += 1;
            beacon.transmission_statistics.last_transmission_time = Some(SystemTime::now());
            black_box(&beacon.transmission_statistics);
        });
    });
    
    group.bench_function("get_transmission_stats", |b| {
        let beacon = create_test_beacon();
        b.iter(|| {
            black_box(beacon.get_transmission_stats());
        });
    });
    
    group.finish();
}

fn benchmark_emergency_transmissions(c: &mut Criterion) {
    let mut group = c.benchmark_group("emergency_transmissions");
    
    group.bench_function("emergency_message_transmission", |b| {
        let mut beacon = create_test_beacon();
        beacon.gps_manager.start_acquisition().unwrap();
        beacon.gps_manager.update().unwrap();
        beacon.operational_state = beacon::OperationalState::Emergency;
        
        b.iter(|| {
            black_box(beacon.send_emergency_messages().unwrap());
        });
    });
    
    group.bench_function("emergency_transmission_burst", |b| {
        let mut beacon = create_test_beacon();
        beacon.gps_manager.start_acquisition().unwrap();
        beacon.gps_manager.update().unwrap();
        beacon.operational_state = beacon::OperationalState::Emergency;
        
        b.iter(|| {
            // Send multiple emergency messages in burst
            for _ in 0..beacon.config.emergency_config.emergency_message_count {
                black_box(beacon.handle_transmission().unwrap());
            }
        });
    });
    
    group.finish();
}

fn benchmark_transmission_intervals(c: &mut Criterion) {
    let mut group = c.benchmark_group("transmission_intervals");
    
    let intervals = vec![1000, 2000, 5000, 10000]; // milliseconds
    
    for interval in intervals {
        group.bench_with_input(
            BenchmarkId::new("transmission_interval", interval),
            &interval,
            |b, &interval| {
                let mut config = BeaconConfig::default();
                config.transmission_interval_ms = interval;
                
                let gps_manager = MockGpsManager::with_test_positions(config.gps_config.clone()).unwrap();
                let power_manager = MockPowerManager::new();
                let comm_manager = MockCommunicationManager::new();
                let transceiver = MockTransceiverInterface::new();
                
                let mut beacon = BeaconController::new(config, gps_manager, power_manager, comm_manager, transceiver).unwrap();
                beacon.gps_manager.start_acquisition().unwrap();
                beacon.gps_manager.update().unwrap();
                
                b.iter(|| {
                    // Simulate checking if transmission interval has elapsed
                    let now = SystemTime::now();
                    let should_transmit = if let Some(last_transmission) = beacon.last_transmission {
                        now.duration_since(last_transmission).unwrap_or(Duration::from_secs(0)) 
                            >= Duration::from_millis(interval as u64)
                    } else {
                        true
                    };
                    
                    if should_transmit {
                        black_box(beacon.handle_transmission().unwrap());
                    }
                });
            },
        );
    }
    
    group.finish();
}

fn benchmark_signal_quality_tracking(c: &mut Criterion) {
    let mut group = c.benchmark_group("signal_quality_tracking");
    
    group.bench_function("update_signal_quality", |b| {
        let mut beacon = create_test_beacon();
        let mut signal_quality = 85u8;
        
        b.iter(|| {
            signal_quality = (signal_quality + 1) % 100;
            beacon.transmission_statistics.signal_quality_history.push(signal_quality);
            
            // Keep history bounded
            if beacon.transmission_statistics.signal_quality_history.len() > 100 {
                beacon.transmission_statistics.signal_quality_history.remove(0);
            }
            
            black_box(signal_quality);
        });
    });
    
    group.bench_function("calculate_average_signal_quality", |b| {
        let mut beacon = create_test_beacon();
        
        // Fill with sample data
        for i in 0..100 {
            beacon.transmission_statistics.signal_quality_history.push((i % 100) as u8);
        }
        
        b.iter(|| {
            let average = if !beacon.transmission_statistics.signal_quality_history.is_empty() {
                let sum: u32 = beacon.transmission_statistics.signal_quality_history.iter().map(|&x| x as u32).sum();
                sum / beacon.transmission_statistics.signal_quality_history.len() as u32
            } else {
                0
            };
            black_box(average);
        });
    });
    
    group.finish();
}

fn benchmark_transmission_manager_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("transmission_manager");
    
    group.bench_function("create_transmission_manager", |b| {
        let config = TransmissionConfig::default();
        b.iter(|| {
            black_box(TransmissionManager::new(config.clone()));
        });
    });
    
    group.bench_function("schedule_transmission", |b| {
        let config = TransmissionConfig::default();
        let mut manager = TransmissionManager::new(config);
        let beacon_id = Uuid::new_v4();
        let position = GeodeticPosition {
            latitude: 37.7749,
            longitude: -122.4194,
            altitude: 10.0,
        };
        
        b.iter(|| {
            black_box(manager.schedule_transmission(
                beacon_id,
                position.clone(),
                85,
                TransmissionPriority::Normal,
                TransmissionMessageVersion::V3
            ).unwrap());
        });
    });
    
    group.finish();
}

fn benchmark_concurrent_transmissions(c: &mut Criterion) {
    let mut group = c.benchmark_group("concurrent_transmissions");
    
    group.bench_function("multiple_beacon_simulation", |b| {
        // Simulate multiple beacons transmitting
        let mut beacons = Vec::new();
        for _ in 0..5 {
            let mut beacon = create_test_beacon();
            beacon.gps_manager.start_acquisition().unwrap();
            beacon.gps_manager.update().unwrap();
            beacons.push(beacon);
        }
        
        b.iter(|| {
            for beacon in &mut beacons {
                black_box(beacon.handle_transmission().unwrap());
            }
        });
    });
    
    group.finish();
}

fn benchmark_transmission_error_handling(c: &mut Criterion) {
    let mut group = c.benchmark_group("transmission_error_handling");
    
    group.bench_function("handle_transmission_failure", |b| {
        let mut beacon = create_test_beacon();
        beacon.gps_manager.start_acquisition().unwrap();
        beacon.gps_manager.update().unwrap();
        beacon.transceiver.set_simulate_failures(true);
        
        b.iter(|| {
            let result = beacon.handle_transmission();
            // Handle the error (should be an error due to simulated failures)
            match result {
                Ok(_) => {},
                Err(e) => {
                    // Log error and update statistics
                    beacon.transmission_statistics.transmission_failures += 1;
                    black_box(e);
                }
            }
        });
    });
    
    group.bench_function("transmission_retry_logic", |b| {
        let mut beacon = create_test_beacon();
        beacon.gps_manager.start_acquisition().unwrap();
        beacon.gps_manager.update().unwrap();
        beacon.transceiver.set_failure_rate(0.7); // 70% failure rate
        
        b.iter(|| {
            let mut attempts = 0;
            let max_attempts = 3;
            
            while attempts < max_attempts {
                match beacon.handle_transmission() {
                    Ok(_) => break,
                    Err(_) => {
                        attempts += 1;
                        if attempts < max_attempts {
                            // Brief delay before retry (simulated)
                            std::thread::sleep(Duration::from_millis(1));
                        }
                    }
                }
            }
            
            black_box(attempts);
        });
    });
    
    group.finish();
}

criterion_group!(
    transmission_benches,
    benchmark_message_building,
    benchmark_transmission_operations,
    benchmark_transmission_reliability,
    benchmark_transmission_power_levels,
    benchmark_message_validation,
    benchmark_transmission_statistics,
    benchmark_emergency_transmissions,
    benchmark_transmission_intervals,
    benchmark_signal_quality_tracking,
    benchmark_transmission_manager_operations,
    benchmark_concurrent_transmissions,
    benchmark_transmission_error_handling
);

criterion_main!(transmission_benches);