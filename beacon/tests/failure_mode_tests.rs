use uuid::Uuid;
use beacon::{BeaconController, LocalEmergencyType};
use shared_positioning::{
    BeaconConfig,
    beacon_config::{TransmissionConfig, GpsConfig, PowerConfig, CommunicationConfig, EmergencyConfig, HardwareConfig, BeaconConfigMetadata},
    MockGpsManager, MockPowerManager, MockCommunicationManager,
    MockTransceiver as MockTransceiverInterface,
    HardwareComponent, HardwareFaultType, BeaconError
};

fn create_test_beacon() -> BeaconController<MockGpsManager, MockPowerManager, MockCommunicationManager, MockTransceiverInterface> {
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
async fn test_hardware_error_logging() {
    let mut beacon = create_test_beacon();
    let result = beacon.handle_emergency(LocalEmergencyType::HardwareFault);
    assert!(result.is_ok());
}
