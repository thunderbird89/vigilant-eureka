#[cfg(test)]
mod tests {
    // Removed unused super import
    // Removed unused time imports
    use shared_positioning::{
        BeaconConfig, MockGpsManager, MockPowerManager, MockCommunicationManager,
        MockTransceiver, beacon_config::MessageVersion
    };
    use uuid::Uuid;
    
    use crate::{BeaconController, LocalEmergencyType};

    fn create_test_config() -> BeaconConfig {
        BeaconConfig {
            beacon_id: Uuid::new_v4(),
            transmission: shared_positioning::beacon_config::TransmissionConfig {
                interval_ms: 5000,
                message_version: MessageVersion::V3,
                power_level: 128,
                max_retries: 3,
                retry_delay_ms: 1000,
                adaptive_power: true,
                sequence_rollover: 65535,
            },
            gps: shared_positioning::beacon_config::GpsConfig::default(),
            power: shared_positioning::beacon_config::PowerConfig::default(),
            communication: shared_positioning::beacon_config::CommunicationConfig::default(),
            emergency: shared_positioning::beacon_config::EmergencyConfig::default(),
            hardware: shared_positioning::beacon_config::HardwareConfig::default(),
            metadata: shared_positioning::beacon_config::BeaconConfigMetadata::default(),
        }
    }

    fn create_mock_managers() -> (MockGpsManager, MockPowerManager, MockCommunicationManager, MockTransceiver) {
        let gps = MockGpsManager::with_test_positions(shared_positioning::GpsConfig::default()).unwrap();
        let power = MockPowerManager::new();
        let comm = MockCommunicationManager::new();
        let transceiver = MockTransceiver::new(1);
        (gps, power, comm, transceiver)
    }

    #[test]
    fn test_beacon_controller_creation() {
        let config = create_test_config();
        let (gps, power, comm, transceiver) = create_mock_managers();
        
        let controller = BeaconController::new(config.clone(), gps, power, comm, transceiver);
        if let Err(ref e) = controller {
            println!("Controller creation failed: {:?}", e);
        }
        assert!(controller.is_ok());
        
        let controller = controller.unwrap();
        assert_eq!(controller.get_config().beacon_id, config.beacon_id);
    }

    #[test]
    fn test_config_validation() {
        let mut config = create_test_config();
        
        // Valid configuration should pass
        assert!(BeaconController::<MockGpsManager, MockPowerManager, MockCommunicationManager, MockTransceiver>::validate_config(&config).is_ok());
        
        // Invalid transmission interval should fail
        config.transmission.interval_ms = 500; // Too low
        assert!(BeaconController::<MockGpsManager, MockPowerManager, MockCommunicationManager, MockTransceiver>::validate_config(&config).is_err());
        
        config.transmission.interval_ms = 70000; // Too high
        assert!(BeaconController::<MockGpsManager, MockPowerManager, MockCommunicationManager, MockTransceiver>::validate_config(&config).is_err());
    }

    #[test]
    fn test_beacon_start_stop() {
        let config = create_test_config();
        let (gps, power, comm, transceiver) = create_mock_managers();

        let mut controller = BeaconController::new(config, gps, power, comm, transceiver).unwrap();

        assert!(controller.start().is_ok());
        assert!(controller.stop().is_ok());
    }

    #[test]
    fn test_configuration_update() {
        let config = create_test_config();
        let (gps, power, comm, transceiver) = create_mock_managers();
        
        let mut controller = BeaconController::new(config, gps, power, comm, transceiver).unwrap();
        
        let mut new_config = create_test_config();
        new_config.transmission.interval_ms = 10000;
        new_config.gps.update_interval_s = 10;
        
        assert!(controller.update_configuration(new_config).is_ok());
        assert_eq!(controller.get_config().transmission.interval_ms, 10000);
        assert_eq!(controller.get_config().gps.update_interval_s, 10);
    }

    #[test]
    fn test_emergency_handling() {
        let config = create_test_config();
        let (gps, power, comm, transceiver) = create_mock_managers();
        
        let mut controller = BeaconController::new(config, gps, power, comm, transceiver).unwrap();
        
        // Test different emergency types
        assert!(controller.handle_emergency(LocalEmergencyType::BatteryDepleted).is_ok());
        assert!(controller.handle_emergency(LocalEmergencyType::HardwareFault).is_ok());
        assert!(controller.handle_emergency(LocalEmergencyType::GpsSignalLost).is_ok());
        assert!(controller.handle_emergency(LocalEmergencyType::TemperatureExtreme).is_ok());
        assert!(controller.handle_emergency(LocalEmergencyType::SystemOverload).is_ok());
    }

    #[test]
    fn test_status_reporting() {
        let config = create_test_config();
        let (gps, power, comm, transceiver) = create_mock_managers();
        
        let controller = BeaconController::new(config, gps, power, comm, transceiver).unwrap();
        
        let status = controller.get_status();
        assert_eq!(status.beacon_id, controller.get_config().beacon_id);
        assert!(status.system_health.cpu_usage_percent >= 0.0);
        assert!(status.system_health.memory_usage_percent >= 0.0);
    }

    #[test]
    fn test_diagnostic_reporting() {
        let config = create_test_config();
        let (gps, power, comm, transceiver) = create_mock_managers();
        
        let mut controller = BeaconController::new(config, gps, power, comm, transceiver).unwrap();
        
        // Generate diagnostic report
        let report = controller.generate_diagnostic_report();
        
        assert_eq!(report.beacon_id, Some(controller.get_config().beacon_id));
        assert!(report.error_statistics.total_errors >= 0);
    }

    #[test]
    fn test_emergency_messages() {
        let config = create_test_config();
        let (gps, power, comm, transceiver) = create_mock_managers();
        
        let mut controller = BeaconController::new(config, gps, power, comm, transceiver).unwrap();
        
        // Test emergency message sending
        assert!(controller.send_emergency_messages().is_ok());
    }

    #[test]
    fn test_emergency_shutdown() {
        let config = create_test_config();
        let (gps, power, comm, transceiver) = create_mock_managers();
        
        let mut controller = BeaconController::new(config, gps, power, comm, transceiver).unwrap();
        
        // Test emergency shutdown preparation
        assert!(controller.prepare_emergency_shutdown().is_ok());
    }
}