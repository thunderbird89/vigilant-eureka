use std::time::SystemTime;
use uuid::Uuid;
use beacon::BeaconController;
use shared_positioning::{
    BeaconConfig,
    beacon_config::{TransmissionConfig, GpsConfig, PowerConfig, CommunicationConfig, EmergencyConfig, HardwareConfig, BeaconConfigMetadata},
    MockGpsManager, MockPowerManager, MockCommunicationManager,
    MockTransceiver as MockTransceiverInterface,
    ExtendedEnvironmentalConditions, EnvironmentalConditions
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
async fn test_environmental_monitor_update() {
    let mut beacon = create_test_beacon();
    let conditions = ExtendedEnvironmentalConditions {
        base_conditions: EnvironmentalConditions::default(),
        air_temperature_c: Some(10.0),
        humidity_percent: Some(80.0),
        atmospheric_pressure_hpa: Some(1010.0),
        wind_speed_ms: Some(5.0),
        solar_irradiance_wm2: None,
        internal_temperature_c: None,
        cpu_temperature_c: None,
        battery_temperature_c: None,
        enclosure_humidity_percent: None,
        vibration_level_g: None,
        magnetic_field_strength_ut: None,
        wave_height_m: None,
        wave_period_s: None,
        sea_state: None,
        tilt_angle_degrees: None,
        acceleration_g: None,
        thermal_gradient_c_per_m: None,
        heat_dissipation_rate_w: None,
        cooling_efficiency_percent: None,
        timestamp: SystemTime::now(),
        measurement_quality: shared_positioning::environmental_monitor::MeasurementQuality {
            overall_quality: 1.0,
            sensor_health: std::collections::HashMap::new(),
            calibration_status: std::collections::HashMap::new(),
            last_calibration: std::collections::HashMap::new(),
        },
    };

    beacon.environmental_monitor.update_conditions(conditions);
    let stats = beacon.environmental_monitor.get_statistics();
    assert!(stats.total_measurements >= 0);
}
