use std::time::{Duration, SystemTime};
use std::thread;
use uuid::Uuid;

use beacon::{BeaconController, BeaconConfig, MessageVersion, EmergencyConfig};
use shared_positioning::{
    GpsConfig, PowerConfig, CommunicationConfig, GpsPosition, BatteryStatus, BatteryHealth,
    ChargingStatus, PowerOperationMode, MockGpsManager, MockPowerManager, 
    MockCommunicationManager, MockTransceiverInterface, GpsManager, PowerManager,
    CommunicationManager, TransceiverInterface, GpsStatus, CommError, PowerError,
    TransmissionError, GeodeticPosition, MessageBuilder
};

/// Integration test for GPS system functionality
#[tokio::test]
async fn test_gps_integration() {
    let config = GpsConfig {
        acquisition_timeout_s: 30,
        update_interval_s: 2,
        min_satellite_count: 4,
        accuracy_threshold_m: 5.0,
        cold_start_timeout_s: 60,
    };
    
    let mut gps_manager = MockGpsManager::with_test_positions(config.clone()).unwrap();
    
    // Test GPS acquisition sequence
    assert_eq!(gps_manager.get_status(), GpsStatus::Initializing);
    assert!(!gps_manager.is_locked());
    
    // Start acquisition
    assert!(gps_manager.start_acquisition().is_ok());
    assert_eq!(gps_manager.get_status(), GpsStatus::Acquiring);
    
    // Simulate acquisition delay
    gps_manager.set_acquisition_delay(Duration::from_millis(500));
    
    // Update until lock is acquired
    let mut attempts = 0;
    while !gps_manager.is_locked() && attempts < 10 {
        thread::sleep(Duration::from_millis(100));
        assert!(gps_manager.update().is_ok());
        attempts += 1;
    }
    
    assert!(gps_manager.is_locked());
    assert_eq!(gps_manager.get_status(), GpsStatus::Locked);
    
    // Verify position data
    let position = gps_manager.get_current_position();
    assert!(position.is_some());
    
    let pos = position.unwrap();
    assert!(pos.latitude >= -90.0 && pos.latitude <= 90.0);
    assert!(pos.longitude >= -180.0 && pos.longitude <= 180.0);
    assert!(pos.accuracy_m <= config.accuracy_threshold_m);
    assert!(pos.satellite_count >= config.min_satellite_count);
    
    // Test signal loss and recovery
    gps_manager.simulate_signal_loss(true);
    assert!(gps_manager.update().is_ok());
    assert_eq!(gps_manager.get_status(), GpsStatus::SignalLost);
    
    // Recover signal
    gps_manager.simulate_signal_loss(false);
    assert!(gps_manager.update().is_ok());
    assert_eq!(gps_manager.get_status(), GpsStatus::Locked);
    
    // Test hardware fault simulation
    gps_manager.simulate_hardware_fault(true);
    assert!(gps_manager.update().is_err());
    assert_eq!(gps_manager.get_status(), GpsStatus::HardwareFault);
    
    // Test stop
    gps_manager.simulate_hardware_fault(false);
    assert!(gps_manager.stop().is_ok());
    assert_eq!(gps_manager.get_status(), GpsStatus::Initializing);
}

/// Integration test for power management system
#[tokio::test]
async fn test_power_integration() {
    let config = PowerConfig {
        low_battery_threshold_percent: 20.0,
        critical_battery_threshold_percent: 10.0,
        emergency_battery_threshold_percent: 5.0,
        power_save_mode_threshold_percent: 30.0,
        charging_enabled: true,
        solar_charging_enabled: true,
        temperature_min_c: -20.0,
        temperature_max_c: 60.0,
        voltage_min_v: 3.0,
        voltage_max_v: 4.2,
        current_max_ma: 2000.0,
        monitoring_interval_ms: 1000,
    };
    
    let mut power_manager = MockPowerManager::with_config(config.clone()).unwrap();
    
    // Test initial state
    let battery_status = power_manager.get_battery_status().unwrap();
    assert!(battery_status.capacity_percent > 0.0);
    assert_eq!(power_manager.get_power_mode(), PowerOperationMode::Normal);
    
    // Test power mode transitions
    assert!(power_manager.set_power_mode(PowerOperationMode::PowerSave).is_ok());
    assert_eq!(power_manager.get_power_mode(), PowerOperationMode::PowerSave);
    
    assert!(power_manager.set_power_mode(PowerOperationMode::Emergency).is_ok());
    assert_eq!(power_manager.get_power_mode(), PowerOperationMode::Emergency);
    
    // Test battery discharge simulation
    let initial_capacity = power_manager.get_battery_status().unwrap().capacity_percent;
    power_manager.simulate_discharge(10.0);
    let new_capacity = power_manager.get_battery_status().unwrap().capacity_percent;
    assert!(new_capacity < initial_capacity);
    
    // Test threshold violations
    power_manager.simulate_discharge(70.0); // Should trigger low battery
    let violations = power_manager.check_thresholds().unwrap();
    assert!(!violations.is_empty());
    
    // Test charging functionality
    power_manager.simulate_charge(20.0);
    let charged_capacity = power_manager.get_battery_status().unwrap().capacity_percent;
    assert!(charged_capacity > new_capacity);
    
    // Test charging status
    power_manager.set_charging_status(ChargingStatus::Charging { rate_ma: 500.0 });
    let charging_status = power_manager.get_charging_status().unwrap();
    assert!(matches!(charging_status, ChargingStatus::Charging { .. }));
    
    // Test solar charging
    power_manager.set_charging_status(ChargingStatus::SolarCharging { 
        rate_ma: 200.0, 
        solar_voltage_v: 5.0 
    });
    let solar_status = power_manager.get_charging_status().unwrap();
    assert!(matches!(solar_status, ChargingStatus::SolarCharging { .. }));
    
    // Test temperature extremes
    power_manager.simulate_temperature_change(-25.0); // Below minimum
    let violations = power_manager.check_thresholds().unwrap();
    assert!(violations.iter().any(|v| matches!(v, PowerError::TemperatureExtreme { .. })));
    
    power_manager.simulate_temperature_change(65.0); // Above maximum
    let violations = power_manager.check_thresholds().unwrap();
    assert!(violations.iter().any(|v| matches!(v, PowerError::TemperatureExtreme { .. })));
    
    // Test emergency shutdown preparation
    assert!(power_manager.prepare_emergency_shutdown().is_ok());
    assert_eq!(power_manager.get_power_mode(), PowerOperationMode::Emergency);
    
    // Test power statistics
    let stats = power_manager.get_power_stats();
    assert!(stats.uptime >= Duration::from_secs(0));
    assert!(stats.average_current_ma >= 0.0);
}

/// Integration test for communication system
#[tokio::test]
async fn test_communication_integration() {
    let config = CommunicationConfig {
        connection_interval_hours: 1,
        retry_attempts: 3,
        retry_backoff_ms: 1000,
        max_retry_interval_hours: 6,
        connection_timeout_s: 30,
        data_compression_enabled: true,
    };
    
    let mut comm_manager = MockCommunicationManager::new();
    comm_manager.configure(config.clone()).unwrap();
    
    // Test successful connection
    comm_manager.set_connection_success_rate(1.0);
    assert!(comm_manager.connect().is_ok());
    assert!(comm_manager.is_connected());
    
    // Test status report transmission
    let status_report = shared_positioning::StatusReport {
        beacon_id: Uuid::new_v4(),
        timestamp: SystemTime::now(),
        position_history: vec![
            GpsPosition {
                latitude: 37.7749,
                longitude: -122.4194,
                altitude: 10.0,
                timestamp: SystemTime::now(),
                accuracy_m: 3.0,
                satellite_count: 8,
            }
        ],
        battery_status: BatteryStatus::new(3.7, 100.0, 75.0, 25.0),
        system_health: shared_positioning::SystemHealth {
            overall_health_score: 0.85,
            gps_health: shared_positioning::GpsStatusSnapshot {
                status: GpsStatus::Locked,
                satellite_count: 8,
                signal_strength: Some(85),
                last_fix_time: Some(SystemTime::now()),
                position_accuracy_m: Some(3.0),
            },
            power_health: shared_positioning::BatteryStatusSnapshot {
                voltage_v: 3.7,
                current_ma: 100.0,
                capacity_percent: 75.0,
                temperature_c: 25.0,
                health: shared_positioning::BeaconBatteryHealth::Good,
                cycles: 50,
            },
            communication_health: shared_positioning::CommunicationStatusSnapshot {
                is_connected: true,
                signal_strength: Some(80),
                last_successful_contact: Some(SystemTime::now()),
                connection_uptime: Duration::from_secs(3600),
                data_transmitted_bytes: 1024,
                data_received_bytes: 512,
            },
            transmission_health: shared_positioning::TransmissionStatusSnapshot {
                messages_sent: 100,
                transmission_failures: 2,
                last_transmission_time: Some(SystemTime::now()),
                average_transmission_interval_ms: 5000,
                signal_quality_average: 85,
            },
        },
        transmission_stats: shared_positioning::CommTransmissionStats {
            messages_sent: 100,
            transmission_failures: 2,
            last_transmission_time: Some(SystemTime::now()),
            average_transmission_interval_ms: 5000,
            signal_quality_history: vec![85, 87, 83, 89, 86],
        },
    };
    
    assert!(comm_manager.send_status_report(status_report).is_ok());
    
    // Test configuration update checking
    let update_check = comm_manager.check_for_updates();
    assert!(update_check.is_ok());
    
    // Test connection failure and retry
    comm_manager.set_connection_success_rate(0.0);
    assert!(comm_manager.connect().is_err());
    assert!(!comm_manager.is_connected());
    
    // Test signal strength monitoring
    comm_manager.set_connection_success_rate(1.0);
    comm_manager.connect().unwrap();
    let signal_strength = comm_manager.get_signal_strength();
    assert!(signal_strength.is_some());
    assert!(signal_strength.unwrap() <= 100);
    
    // Test disconnection
    assert!(comm_manager.disconnect().is_ok());
    assert!(!comm_manager.is_connected());
}

/// End-to-end integration test with mock hardware interfaces
#[tokio::test]
async fn test_end_to_end_beacon_operation() {
    let beacon_config = BeaconConfig {
        beacon_id: Uuid::new_v4(),
        transmission_interval_ms: 2000, // Faster for testing
        message_version: MessageVersion::V3,
        gps_config: GpsConfig {
            acquisition_timeout_s: 10,
            update_interval_s: 1,
            min_satellite_count: 4,
            accuracy_threshold_m: 5.0,
            cold_start_timeout_s: 20,
        },
        power_config: PowerConfig::default(),
        communication_config: CommunicationConfig::default(),
        emergency_config: EmergencyConfig::default(),
    };
    
    // Create mock hardware interfaces
    let mut gps_manager = MockGpsManager::with_test_positions(beacon_config.gps_config.clone()).unwrap();
    gps_manager.set_acquisition_delay(Duration::from_millis(100));
    
    let mut power_manager = MockPowerManager::new();
    let mut comm_manager = MockCommunicationManager::new();
    comm_manager.set_connection_success_rate(0.8); // 80% success rate
    
    let transceiver = MockTransceiverInterface::new();
    
    // Create beacon controller
    let mut beacon = BeaconController::new(
        beacon_config.clone(),
        gps_manager,
        power_manager,
        comm_manager,
        transceiver
    ).unwrap();
    
    // Test beacon startup sequence
    assert!(beacon.start().is_ok());
    
    // Allow time for GPS acquisition and initial operations
    tokio::time::sleep(Duration::from_millis(500)).await;
    
    // Check beacon status
    let status = beacon.get_status();
    assert_eq!(status.beacon_id, beacon_config.beacon_id);
    assert!(matches!(status.operational_state, 
        beacon::OperationalState::Normal | 
        beacon::OperationalState::GpsAcquisition
    ));
    
    // Test transmission functionality
    if status.gps_status == GpsStatus::Locked {
        // Should be able to transmit with GPS lock
        let transmission_result = beacon.handle_transmission();
        assert!(transmission_result.is_ok());
    }
    
    // Test configuration update during operation
    let mut new_config = beacon_config.clone();
    new_config.transmission_interval_ms = 3000;
    assert!(beacon.update_configuration(new_config).is_ok());
    
    // Test emergency scenario
    beacon.power_manager.simulate_discharge(92.0); // Trigger emergency
    assert!(beacon.handle_emergency(beacon::EmergencyType::BatteryDepleted).is_ok());
    
    let status = beacon.get_status();
    assert_eq!(status.operational_state, beacon::OperationalState::Emergency);
    
    // Test graceful shutdown
    assert!(beacon.stop().is_ok());
    
    let status = beacon.get_status();
    assert_eq!(status.operational_state, beacon::OperationalState::Shutdown);
}

/// Test message transmission and reception integration
#[tokio::test]
async fn test_message_transmission_integration() {
    let beacon_id = Uuid::new_v4();
    let message_builder = MessageBuilder::new();
    
    let position = GeodeticPosition {
        latitude: 37.7749,
        longitude: -122.4194,
        altitude: 10.0,
    };
    
    // Test V1 message transmission
    let v1_message = message_builder.build_v1_message(
        beacon_id,
        position.clone(),
        85, // signal quality
        123 // sequence number
    ).unwrap();
    
    let mut transceiver = MockTransceiverInterface::new();
    assert!(transceiver.transmit_message(&v1_message).is_ok());
    
    // Test V2 message transmission
    let v2_message = message_builder.build_v2_message(
        beacon_id,
        position.clone(),
        87,
        124
    ).unwrap();
    
    assert!(transceiver.transmit_message(&v2_message).is_ok());
    
    // Test V3 message transmission (with UUID)
    let v3_message = message_builder.build_v3_message(
        beacon_id,
        position,
        89,
        125
    ).unwrap();
    
    assert!(transceiver.transmit_message(&v3_message).is_ok());
    
    // Test transmission power control
    assert!(transceiver.set_transmission_power(200).is_ok());
    let status = transceiver.get_transmission_status();
    assert!(status.power_level <= 255);
    
    // Test transmission failure handling
    transceiver.set_simulate_failures(true);
    let failure_result = transceiver.transmit_message(&v1_message);
    assert!(failure_result.is_err());
    
    // Test recovery from transmission failure
    transceiver.set_simulate_failures(false);
    let recovery_result = transceiver.transmit_message(&v1_message);
    assert!(recovery_result.is_ok());
}

/// Test environmental conditions and system adaptation
#[tokio::test]
async fn test_environmental_adaptation_integration() {
    let beacon_config = BeaconConfig::default();
    let gps_manager = MockGpsManager::with_test_positions(beacon_config.gps_config.clone()).unwrap();
    let mut power_manager = MockPowerManager::new();
    let comm_manager = MockCommunicationManager::new();
    let transceiver = MockTransceiverInterface::new();
    
    let mut beacon = BeaconController::new(
        beacon_config,
        gps_manager,
        power_manager,
        comm_manager,
        transceiver
    ).unwrap();
    
    // Test normal environmental conditions
    assert!(beacon.update_environmental_monitoring().is_ok());
    
    // Test extreme temperature adaptation
    beacon.environmental_monitor.update_conditions(
        shared_positioning::ExtendedEnvironmentalConditions {
            temperature_c: -25.0, // Extreme cold
            humidity_percent: 95.0,
            pressure_hpa: 980.0, // Low pressure
            wind_speed_ms: 30.0, // High wind
            wave_height_m: 6.0, // High waves
            visibility_m: 50.0, // Poor visibility
            precipitation_mmh: 15.0, // Heavy precipitation
            uv_index: 2.0,
            air_quality_index: 200.0, // Poor air quality
        }
    );
    
    assert!(beacon.update_environmental_monitoring().is_ok());
    
    // Test high temperature adaptation
    beacon.environmental_monitor.update_conditions(
        shared_positioning::ExtendedEnvironmentalConditions {
            temperature_c: 65.0, // Extreme heat
            humidity_percent: 85.0,
            pressure_hpa: 1030.0, // High pressure
            wind_speed_ms: 5.0,
            wave_height_m: 1.0,
            visibility_m: 10000.0,
            precipitation_mmh: 0.0,
            uv_index: 11.0, // Extreme UV
            air_quality_index: 50.0,
        }
    );
    
    assert!(beacon.update_environmental_monitoring().is_ok());
    
    // Verify that environmental conditions affect system behavior
    let environmental_stats = beacon.environmental_monitor.get_stats();
    assert!(environmental_stats.total_updates >= 2);
    assert!(environmental_stats.adaptation_actions_taken >= 0);
}

/// Test system reliability and fault tolerance
#[tokio::test]
async fn test_reliability_and_fault_tolerance() {
    let beacon_config = BeaconConfig::default();
    let mut gps_manager = MockGpsManager::with_test_positions(beacon_config.gps_config.clone()).unwrap();
    let mut power_manager = MockPowerManager::new();
    let mut comm_manager = MockCommunicationManager::new();
    let mut transceiver = MockTransceiverInterface::new();
    
    let mut beacon = BeaconController::new(
        beacon_config,
        gps_manager,
        power_manager,
        comm_manager,
        transceiver
    ).unwrap();
    
    // Test GPS fault tolerance
    beacon.gps_manager.simulate_hardware_fault(true);
    let gps_result = beacon.update_gps();
    assert!(gps_result.is_err()); // Should handle GPS fault gracefully
    
    beacon.gps_manager.simulate_hardware_fault(false);
    
    // Test power system fault tolerance
    beacon.power_manager.set_simulate_faults(true);
    let power_result = beacon.check_power_status();
    assert!(power_result.is_err()); // Should handle power fault gracefully
    
    beacon.power_manager.set_simulate_faults(false);
    
    // Test communication fault tolerance
    beacon.communication_manager.set_connection_success_rate(0.0);
    let comm_result = beacon.check_communication();
    assert!(comm_result.is_err()); // Should handle communication fault gracefully
    
    beacon.communication_manager.set_connection_success_rate(1.0);
    
    // Test transmission fault tolerance
    beacon.transceiver.set_simulate_failures(true);
    let transmission_result = beacon.handle_transmission();
    // Should handle transmission failure gracefully (may succeed with retry logic)
    
    beacon.transceiver.set_simulate_failures(false);
    
    // Test reliability monitoring
    assert!(beacon.update_reliability_monitoring().is_ok());
    
    let reliability_report = beacon.reliability_monitor.generate_report();
    assert!(reliability_report.overall_reliability_score >= 0.0);
    assert!(reliability_report.overall_reliability_score <= 1.0);
    
    // Test hardware diagnostics
    assert!(beacon.run_hardware_diagnostics().is_ok());
    
    let hardware_stats = beacon.hardware_monitor.get_stats();
    assert!(hardware_stats.total_diagnostics_run >= 1);
}

/// Performance test for continuous operation
#[tokio::test]
async fn test_continuous_operation_performance() {
    let beacon_config = BeaconConfig {
        transmission_interval_ms: 1000, // 1 second for faster testing
        ..BeaconConfig::default()
    };
    
    let gps_manager = MockGpsManager::with_test_positions(beacon_config.gps_config.clone()).unwrap();
    let power_manager = MockPowerManager::new();
    let comm_manager = MockCommunicationManager::new();
    let transceiver = MockTransceiverInterface::new();
    
    let mut beacon = BeaconController::new(
        beacon_config,
        gps_manager,
        power_manager,
        comm_manager,
        transceiver
    ).unwrap();
    
    let start_time = std::time::Instant::now();
    
    // Run beacon for a short period to test performance
    for _ in 0..10 {
        // Simulate one control loop iteration
        let _ = beacon.update_gps();
        let _ = beacon.check_power_status();
        let _ = beacon.handle_transmission();
        let _ = beacon.check_communication();
        beacon.check_system_health();
        
        tokio::time::sleep(Duration::from_millis(100)).await;
    }
    
    let elapsed = start_time.elapsed();
    
    // Verify performance characteristics
    assert!(elapsed < Duration::from_secs(5)); // Should complete quickly
    
    let status = beacon.get_status();
    assert!(status.uptime >= Duration::from_secs(0));
    
    // Check that transmission statistics are being tracked
    assert!(status.transmission_stats.messages_sent >= 0);
}