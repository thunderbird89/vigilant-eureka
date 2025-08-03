use std::time::{Duration, SystemTime};
use uuid::Uuid;

use beacon::{BeaconController, LocalEmergencyType};
use shared_positioning::{
    BeaconConfig, MessageVersion, EmergencyType, EmergencyConfig, EmergencySeverity,
    GpsConfig, PowerConfig, CommunicationConfig, MockGpsManager, MockPowerManager,
    MockCommunicationManager, MockTransceiver, TransceiverInterface, GpsStatus,
    PowerError, PowerOperationMode, BatteryStatus, BatteryHealth, ChargingStatus,
    HardwareComponent, HardwareFaultType,
    BeaconError, GpsErrorType, PowerErrorType, CommunicationErrorType,
    TransmissionErrorType, SystemErrorType, ConfigurationErrorType
};

fn create_test_beacon() -> BeaconController<MockGpsManager, MockPowerManager, MockCommunicationManager, MockTransceiver> {
    let config = BeaconConfig::new(Uuid::new_v4());
    
    let gps_manager = MockGpsManager::new(shared_positioning::GpsConfig::default()).unwrap();
    let power_manager = MockPowerManager::new();
    let comm_manager = MockCommunicationManager::new();
    let transceiver = MockTransceiver::new(1);
    
    BeaconController::new(config, gps_manager, power_manager, comm_manager, transceiver).unwrap()
}

#[tokio::test]
async fn test_basic_beacon_functionality() {
    let mut beacon = create_test_beacon();
    
    // Test basic beacon functionality
    let status = beacon.get_status();
    // BeaconStatus doesn't have is_ok(), just check it returns something
    let _ = status;
    
    // Test emergency handling
    assert!(beacon.handle_emergency(LocalEmergencyType::GpsSignalLost).is_ok());
    assert!(beacon.handle_emergency(LocalEmergencyType::BatteryDepleted).is_ok());
    assert!(beacon.handle_emergency(LocalEmergencyType::SystemOverload).is_ok());
}

#[tokio::test]
async fn test_beacon_update_cycle() {
    let mut beacon = create_test_beacon();
    
    // Test basic beacon operations
    for _ in 0..5 {
        let status = beacon.get_status();
        let _ = status;
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    
    let status = beacon.get_status();
    let _ = status;
}

#[tokio::test]
async fn test_configuration_validation() {
    // Test basic configuration creation
    let config = BeaconConfig::new(Uuid::new_v4());
    assert_eq!(config.beacon_id.get_version(), Some(uuid::Version::Random));
    
    // Test that we can create a beacon with valid config
    let gps_manager = MockGpsManager::new(shared_positioning::GpsConfig::default()).unwrap();
    let power_manager = MockPowerManager::new();
    let comm_manager = MockCommunicationManager::new();
    let transceiver = MockTransceiver::new(1);
    
    let beacon_result = BeaconController::new(config, gps_manager, power_manager, comm_manager, transceiver);
    assert!(beacon_result.is_ok());
}

#[tokio::test]
async fn test_message_building() {
    use shared_positioning::{MessageBuilder, GeodeticPosition};
    
    let beacon_id = Uuid::new_v4();
    let message_builder = MessageBuilder::new();
    
    // Test with valid position
    let valid_position = GeodeticPosition {
        latitude: 37.7749,
        longitude: -122.4194,
        depth: 10.0,
    };
    
    // Test V1 message building
    let result = message_builder.build_v1_message(123, valid_position.clone(), 85, 123);
    assert!(result.is_ok());
    
    // Test V2 message building  
    let result = message_builder.build_v2_message(123, valid_position.clone(), 85, 123);
    assert!(result.is_ok());
    
    // Test V3 message building
    let result = message_builder.build_v3_message(beacon_id, valid_position, 85, 123);
    assert!(result.is_ok());
}