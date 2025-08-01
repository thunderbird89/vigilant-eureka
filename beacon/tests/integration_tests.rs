use std::time::Duration;
use uuid::Uuid;
use beacon::{BeaconController, LocalEmergencyType};
use shared_positioning::{
    BeaconConfig,
    beacon_config::{TransmissionConfig, GpsConfig, PowerConfig, CommunicationConfig, EmergencyConfig, HardwareConfig, BeaconConfigMetadata},
    MockGpsManager, MockPowerManager, MockCommunicationManager,
    MockTransceiver as MockTransceiverInterface,
    GeodeticPosition, MessageBuilder, GpsStatus, TransceiverInterface
};

fn create_beacon() -> BeaconController<MockGpsManager, MockPowerManager, MockCommunicationManager, MockTransceiverInterface> {
    let config = BeaconConfig {
        beacon_id: Uuid::new_v4(),
        transmission: TransmissionConfig::default(),
        gps: GpsConfig::default(),
        power: PowerConfig::default(),
        communication: CommunicationConfig::default(),
        emergency: EmergencyConfig::default(),
        hardware: HardwareConfig::default(),
        metadata: BeaconConfigMetadata::default(),
    };

    let gps_manager = MockGpsManager::with_test_positions(shared_positioning::GpsConfig::default()).unwrap();
    let power_manager = MockPowerManager::new();
    let comm_manager = MockCommunicationManager::new();
    let transceiver = MockTransceiverInterface::new(1);

    BeaconController::new(config, gps_manager, power_manager, comm_manager, transceiver).unwrap()
}

#[tokio::test]
async fn test_basic_beacon_startup() {
    let mut beacon = create_beacon();
    assert!(beacon.start().is_ok());
    tokio::time::sleep(Duration::from_millis(100)).await;
    assert!(beacon.stop().is_ok());
}

#[tokio::test]
async fn test_message_transmission_integration() {
    let beacon_uuid = Uuid::new_v4();
    let beacon_id = (beacon_uuid.as_u128() & 0xFFFF) as u16;
    let message_builder = MessageBuilder::new();

    let position = GeodeticPosition { latitude: 37.0, longitude: -122.0, depth: 10.0 };

    let v1 = message_builder.build_v1_message(beacon_id, position.clone(), 80, 1).unwrap();
    let mut transceiver = MockTransceiverInterface::new(1);
    assert!(transceiver.transmit_message(&v1).is_ok());

    let v3 = message_builder.build_v3_message(beacon_uuid, position, 80, 2).unwrap();
    assert!(transceiver.transmit_message(&v3).is_ok());
}

#[tokio::test]
async fn test_emergency_handling() {
    let mut beacon = create_beacon();
    assert!(beacon.handle_emergency(LocalEmergencyType::BatteryDepleted).is_ok());
}
