#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};
    use std::collections::VecDeque;
    use shared_positioning::{
        MockGpsManager, MockPowerManager, MockCommunicationManager, MockTransceiverInterface,
        GpsPosition, BatteryStatus, BatteryHealth, ChargingStatus, PowerOperationMode,
        TransmissionError, CommError
    };
    use tokio_test;
    use serial_test::serial;

    // Helper function to create test beacon configuration
    fn create_test_config() -> BeaconConfig {
        BeaconConfig {
            beacon_id: Uuid::new_v4(),
            transmission_interval_ms: 5000,
            message_version: MessageVersion::V3,
            gps_config: GpsConfig::default(),
            power_config: PowerConfig::default(),
            communication_config: CommunicationConfig::default(),
            emergency_config: EmergencyConfig::default(),
        }
    }

    // Helper function to create mock managers for testing
    fn create_mock_managers() -> (MockGpsManager, MockPowerManager, MockCommunicationManager, MockTransceiverInterface) {
        let gps_manager = MockGpsManager::with_test_positions(GpsConfig::default()).unwrap();
        let power_manager = MockPowerManager::new();
        let comm_manager = MockCommunicationManager::new();
        let transceiver = MockTransceiverInterface::new();
        
        (gps_manager, power_manager, comm_manager, transceiver)
    }

    #[test]
    fn test_beacon_controller_creation() {
        let config = create_test_config();
        let (gps, power, comm, transceiver) = create_mock_managers();
        
        let controller = BeaconController::new(config.clone(), gps, power, comm, transceiver);
        assert!(controller.is_ok());
        
        let controller = controller.unwrap();
        assert_eq!(controller.get_config().beacon_id, config.beacon_id);
        assert_eq!(controller.operational_state, OperationalState::Initializing);
    }

    #[test]
    fn test_config_validation() {
        let mut config = create_test_config();
        
        // Valid configuration should pass
        assert!(BeaconController::validate_config(&config).is_ok());
        
        // Invalid transmission interval
        config.transmission_interval_ms = 500; // Too low
        assert!(BeaconController::validate_config(&config).is_err());
        
        config.transmission_interval_ms = 70000; // Too high
        assert!(BeaconController::validate_config(&config).is_err());
        
        // Reset to valid value
        config.transmission_interval_ms = 5000;
        
        // Invalid emergency transmission interval
        config.emergency_config.emergency_transmission_interval_ms = 500; // Too low
        assert!(BeaconController::validate_config(&config).is_err());
        
        // Invalid emergency power threshold
        config.emergency_config.emergency_transmission_interval_ms = 5000;
        config.emergency_config.emergency_power_threshold_percent = 0.5; // Too low
        assert!(BeaconController::validate_config(&config).is_err());
        
        config.emergency_config.emergency_power_threshold_percent = 25.0; // Too high
        assert!(BeaconController::validate_config(&config).is_err());
    }

    #[test]
    #[serial]
    fn test_beacon_start_stop() {
        let config = create_test_config();
        let (mut gps, power, comm, transceiver) = create_mock_managers();
        
        // Set up GPS to acquire lock quickly
        gps.set_acquisition_delay(Duration::from_millis(100));
        
        let mut controller = BeaconController::new(config, gps, power, comm, transceiver).unwrap();
        
        // Test start
        assert!(controller.start().is_ok());
        assert!(controller.running);
        
        // Test stop
        assert!(controller.stop().is_ok());
        assert!(!controller.running);
        assert_eq!(controller.operational_state, OperationalState::Shutdown);
    }

    #[test]
    fn test_configuration_update() {
        let config = create_test_config();
        let (gps, power, comm, transceiver) = create_mock_managers();
        
        let mut controller = BeaconController::new(config, gps, power, comm, transceiver).unwrap();
        
        let mut new_config = create_test_config();
        new_config.transmission_interval_ms = 10000;
        new_config.gps_config.update_interval_s = 10;
        
        assert!(controller.update_configuration(new_config.clone()).is_ok());
        assert_eq!(controller.get_config().transmission_interval_ms, 10000);
        assert_eq!(controller.get_config().gps_config.update_interval_s, 10);
    }

    #[test]
    fn test_emergency_handling() {
        let config = create_test_config();
        let (gps, mut power, comm, transceiver) = create_mock_managers();
        
        // Set up low battery condition
        let mut battery_status = BatteryStatus::new(3.0, 100.0, 3.0, 25.0);
        battery_status.health = BatteryHealth::Critical;
        power.set_battery_status(battery_status);
        
        let mut controller = BeaconController::new(config, gps, power, comm, transceiver).unwrap();
        
        // Test battery depletion emergency
        assert!(controller.handle_emergency(EmergencyType::BatteryDepleted).is_ok());
        assert_eq!(controller.operational_state, OperationalState::Emergency);
        
        // Test hardware fault emergency
        assert!(controller.handle_emergency(EmergencyType::HardwareFault).is_ok());
        assert!(matches!(controller.operational_state, OperationalState::Error(_)));
        
        // Test GPS signal lost emergency
        assert!(controller.handle_emergency(EmergencyType::GpsSignalLost).is_ok());
        
        // Test temperature extreme emergency
        assert!(controller.handle_emergency(EmergencyType::TemperatureExtreme).is_ok());
        
        // Test system overload emergency
        assert!(controller.handle_emergency(EmergencyType::SystemOverload).is_ok());
        assert_eq!(controller.operational_state, OperationalState::PowerSave);
    }

    #[test]
    fn test_gps_state_transitions() {
        let config = create_test_config();
        let (mut gps, power, comm, transceiver) = create_mock_managers();
        
        let mut controller = BeaconController::new(config, gps, power, comm, transceiver).unwrap();
        
        // Test GPS acquisition timeout
        controller.gps_manager.set_acquisition_delay(Duration::from_secs(120)); // Longer than timeout
        assert!(controller.update_gps().is_err());
        
        // Test GPS signal loss
        controller.gps_manager.simulate_signal_loss(true);
        controller.last_gps_update = Some(SystemTime::now() - Duration::from_secs(400)); // Longer than emergency timeout
        assert!(controller.update_gps().is_err());
        
        // Test GPS hardware fault
        controller.gps_manager.simulate_hardware_fault(true);
        assert!(controller.update_gps().is_err());
    }

    #[test]
    fn test_power_status_monitoring() {
        let config = create_test_config();
        let (gps, mut power, comm, transceiver) = create_mock_managers();
        
        let mut controller = BeaconController::new(config, gps, power, comm, transceiver).unwrap();
        
        // Test normal power status
        assert!(controller.check_power_status().is_ok());
        
        // Test emergency battery level
        controller.power_manager.simulate_discharge(92.0); // Should trigger emergency
        assert!(controller.check_power_status().is_err());
        assert_eq!(controller.operational_state, OperationalState::Emergency);
        
        // Test power save mode trigger
        let mut controller = BeaconController::new(create_test_config(), 
            MockGpsManager::with_test_positions(GpsConfig::default()).unwrap(),
            MockPowerManager::new(), 
            MockCommunicationManager::new(), 
            MockTransceiverInterface::new()).unwrap();
        
        controller.operational_state = OperationalState::Normal;
        controller.power_manager.simulate_discharge(55.0); // Should trigger power save
        assert!(controller.check_power_status().is_ok());
        assert_eq!(controller.operational_state, OperationalState::PowerSave);
    }

    #[test]
    fn test_transmission_handling() {
        let config = create_test_config();
        let (mut gps, power, comm, transceiver) = create_mock_managers();
        
        // Set up GPS with valid position
        gps.start_acquisition().unwrap();
        gps.update().unwrap(); // Should acquire lock with test positions
        
        let mut controller = BeaconController::new(config, gps, power, comm, transceiver).unwrap();
        controller.operational_state = OperationalState::Normal;
        
        // Test successful transmission
        assert!(controller.handle_transmission().is_ok());
        assert!(controller.last_transmission.is_some());
        assert_eq!(controller.transmission_sequence, 1);
        
        // Test transmission without GPS lock
        controller.gps_manager.simulate_signal_loss(true);
        controller.gps_manager.update().unwrap(); // Should lose lock
        assert!(controller.handle_transmission().is_ok()); // Should still work with last known position
    }

    #[test]
    fn test_communication_management() {
        let config = create_test_config();
        let (gps, power, mut comm, transceiver) = create_mock_managers();
        
        // Set up communication manager
        comm.set_connection_success_rate(1.0); // Always succeed
        
        let mut controller = BeaconController::new(config, gps, power, comm, transceiver).unwrap();
        
        // Test successful communication check
        assert!(controller.check_communication().is_ok());
        
        // Test communication failure
        controller.communication_manager.set_connection_success_rate(0.0); // Always fail
        assert!(controller.check_communication().is_err());
    }

    #[test]
    fn test_system_health_monitoring() {
        let config = create_test_config();
        let (gps, power, comm, transceiver) = create_mock_managers();
        
        let mut controller = BeaconController::new(config, gps, power, comm, transceiver).unwrap();
        
        // Test system health check
        controller.check_system_health();
        
        let status = controller.get_status();
        assert!(status.system_health.overall_health_score >= 0.0);
        assert!(status.system_health.overall_health_score <= 1.0);
    }

    #[test]
    fn test_diagnostic_reporting() {
        let config = create_test_config();
        let (gps, power, comm, transceiver) = create_mock_managers();
        
        let mut controller = BeaconController::new(config, gps, power, comm, transceiver).unwrap();
        
        // Generate diagnostic report
        let report = controller.generate_diagnostic_report();
        
        assert!(report.system_health.overall_health_score >= 0.0);
        assert!(report.system_health.overall_health_score <= 1.0);
        assert_eq!(report.beacon_id, controller.config.beacon_id);
        assert!(report.error_statistics.total_errors >= 0);
    }

    #[test]
    fn test_environmental_monitoring() {
        let config = create_test_config();
        let (gps, power, comm, transceiver) = create_mock_managers();
        
        let mut controller = BeaconController::new(config, gps, power, comm, transceiver).unwrap();
        
        // Test environmental monitoring update
        assert!(controller.update_environmental_monitoring().is_ok());
        
        // Test with extreme conditions
        controller.environmental_monitor.update_conditions(ExtendedEnvironmentalConditions {
            temperature_c: 70.0, // Extreme temperature
            humidity_percent: 95.0,
            pressure_hpa: 1013.25,
            wind_speed_ms: 25.0, // High wind
            wave_height_m: 5.0, // High waves
            visibility_m: 100.0, // Poor visibility
            precipitation_mmh: 10.0, // Heavy rain
            uv_index: 8.0,
            air_quality_index: 150.0, // Poor air quality
        });
        
        assert!(controller.update_environmental_monitoring().is_ok());
    }

    #[test]
    fn test_hardware_diagnostics() {
        let config = create_test_config();
        let (gps, power, comm, transceiver) = create_mock_managers();
        
        let mut controller = BeaconController::new(config, gps, power, comm, transceiver).unwrap();
        
        // Test hardware diagnostics
        assert!(controller.run_hardware_diagnostics().is_ok());
        
        let stats = controller.hardware_monitor.get_stats();
        assert!(stats.total_diagnostics_run >= 1);
    }

    #[test]
    fn test_reliability_monitoring() {
        let config = create_test_config();
        let (gps, power, comm, transceiver) = create_mock_managers();
        
        let mut controller = BeaconController::new(config, gps, power, comm, transceiver).unwrap();
        
        // Test reliability monitoring update
        assert!(controller.update_reliability_monitoring().is_ok());
        
        let report = controller.reliability_monitor.generate_report();
        assert!(report.overall_reliability_score >= 0.0);
        assert!(report.overall_reliability_score <= 1.0);
    }

    #[test]
    fn test_error_recovery_strategies() {
        let config = create_test_config();
        let (gps, power, comm, transceiver) = create_mock_managers();
        
        let mut controller = BeaconController::new(config, gps, power, comm, transceiver).unwrap();
        
        // Test GPS error recovery
        let gps_error = BeaconError::GpsError {
            error_type: GpsErrorType::SignalLost,
            last_known_position: None,
            satellite_count: 0,
            signal_strength: None,
        };
        
        let recovery_strategy = controller.log_beacon_error(gps_error);
        assert!(recovery_strategy.is_some());
        
        if let Some(strategy) = recovery_strategy {
            assert!(controller.apply_recovery_strategy(strategy).is_ok());
        }
        
        // Test power error recovery
        let power_error = BeaconError::PowerError {
            error_type: PowerErrorType::BatteryDepleted,
            battery_status: BatteryStatusSnapshot {
                voltage_v: 3.0,
                current_ma: 100.0,
                capacity_percent: 3.0,
                temperature_c: 25.0,
                health: BeaconBatteryHealth::Critical,
                cycles: 100,
            },
            power_mode: BeaconPowerMode::Emergency,
            charging_status: BeaconChargingStatus::NotCharging,
        };
        
        let recovery_strategy = controller.log_beacon_error(power_error);
        assert!(recovery_strategy.is_some());
        
        if let Some(strategy) = recovery_strategy {
            assert!(controller.apply_recovery_strategy(strategy).is_ok());
        }
    }

    #[test]
    fn test_message_building_and_transmission() {
        let config = create_test_config();
        let (mut gps, power, comm, transceiver) = create_mock_managers();
        
        // Set up GPS with valid position
        gps.start_acquisition().unwrap();
        gps.update().unwrap();
        
        let mut controller = BeaconController::new(config, gps, power, comm, transceiver).unwrap();
        
        if let Some(position) = controller.gps_manager.get_current_position() {
            let geodetic_pos = GeodeticPosition {
                latitude: position.latitude,
                longitude: position.longitude,
                altitude: position.altitude,
            };
            
            // Test V1 message building
            let message_builder = MessageBuilder::new();
            let result = message_builder.build_v1_message(
                controller.config.beacon_id,
                geodetic_pos.clone(),
                8, // signal quality
                controller.transmission_sequence
            );
            assert!(result.is_ok());
            
            // Test V2 message building
            let result = message_builder.build_v2_message(
                controller.config.beacon_id,
                geodetic_pos.clone(),
                8,
                controller.transmission_sequence
            );
            assert!(result.is_ok());
            
            // Test V3 message building (with UUID)
            let result = message_builder.build_v3_message(
                controller.config.beacon_id,
                geodetic_pos,
                8,
                controller.transmission_sequence
            );
            assert!(result.is_ok());
        }
    }

    #[test]
    fn test_concurrent_operations() {
        use std::sync::Arc;
        use std::thread;
        
        let config = create_test_config();
        let (gps, power, comm, transceiver) = create_mock_managers();
        
        let controller = Arc::new(Mutex::new(
            BeaconController::new(config, gps, power, comm, transceiver).unwrap()
        ));
        
        let mut handles = vec![];
        
        // Spawn multiple threads to test concurrent access
        for i in 0..5 {
            let controller_clone = Arc::clone(&controller);
            let handle = thread::spawn(move || {
                for _ in 0..10 {
                    let mut ctrl = controller_clone.lock().unwrap();
                    let status = ctrl.get_status();
                    assert_eq!(status.beacon_id, ctrl.config.beacon_id);
                    
                    // Simulate some work
                    thread::sleep(Duration::from_millis(1));
                }
            });
            handles.push(handle);
        }
        
        // Wait for all threads to complete
        for handle in handles {
            handle.join().unwrap();
        }
    }

    #[test]
    fn test_memory_usage_patterns() {
        let config = create_test_config();
        let (gps, power, comm, transceiver) = create_mock_managers();
        
        let mut controller = BeaconController::new(config, gps, power, comm, transceiver).unwrap();
        
        // Test that error log doesn't grow unbounded
        for i in 0..1000 {
            let error = BeaconError::SystemError {
                error_type: SystemErrorType::ResourceExhausted,
                system_state: BeaconSystemState::Normal,
                resource_usage: ResourceUsageSnapshot {
                    memory_usage_bytes: 1024 * i,
                    cpu_usage_percent: 50.0,
                    storage_usage_bytes: 2048 * i,
                    network_usage_bytes: 512 * i,
                },
                available_resources: ResourceUsageSnapshot {
                    memory_usage_bytes: 1024 * 1024,
                    cpu_usage_percent: 100.0,
                    storage_usage_bytes: 1024 * 1024 * 10,
                    network_usage_bytes: 1024 * 1024,
                },
            };
            
            controller.log_beacon_error(error);
        }
        
        // Error log should be bounded (e.g., max 100 entries)
        assert!(controller.error_log.len() <= 100);
    }

    #[test]
    fn test_configuration_persistence() {
        let config = create_test_config();
        let (gps, power, comm, transceiver) = create_mock_managers();
        
        let controller = BeaconController::new(config.clone(), gps, power, comm, transceiver).unwrap();
        
        // Test that configuration is properly stored and retrievable
        let stored_config = controller.get_config();
        assert_eq!(stored_config.beacon_id, config.beacon_id);
        assert_eq!(stored_config.transmission_interval_ms, config.transmission_interval_ms);
        assert_eq!(stored_config.gps_config.acquisition_timeout_s, config.gps_config.acquisition_timeout_s);
    }
}