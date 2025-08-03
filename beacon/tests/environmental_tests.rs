use std::time::{Duration, SystemTime};
use std::collections::HashMap;
use uuid::Uuid;

use beacon::BeaconController;
use shared_positioning::{
    BeaconConfig, MockGpsManager, MockPowerManager, MockCommunicationManager, MockTransceiver,
    ExtendedEnvironmentalConditions, EnvironmentalConditions, MeasurementQuality,
    EnvironmentalMonitor, EnvironmentalThresholds
};

fn create_test_beacon() -> BeaconController<MockGpsManager, MockPowerManager, MockCommunicationManager, MockTransceiver> {
    let config = BeaconConfig::new(Uuid::new_v4());
    
    let gps_manager = MockGpsManager::new(shared_positioning::GpsConfig::default()).unwrap();
    let power_manager = MockPowerManager::new();
    let comm_manager = MockCommunicationManager::new();
    let transceiver = MockTransceiver::new(1);
    
    BeaconController::new(config, gps_manager, power_manager, comm_manager, transceiver).unwrap()
}

fn create_test_environmental_conditions() -> ExtendedEnvironmentalConditions {
    ExtendedEnvironmentalConditions {
        base_conditions: EnvironmentalConditions::default(),
        air_temperature_c: Some(20.0),
        humidity_percent: Some(60.0),
        atmospheric_pressure_hpa: Some(1015.0),
        wind_speed_ms: Some(5.0),
        wave_height_m: Some(1.0),
        solar_irradiance_wm2: Some(800.0),
        internal_temperature_c: Some(25.0),
        cpu_temperature_c: Some(45.0),
        battery_temperature_c: Some(22.0),
        enclosure_humidity_percent: Some(55.0),
        vibration_level_g: Some(0.1),
        magnetic_field_strength_ut: Some(50.0),
        wave_period_s: Some(8.0),
        sea_state: None,
        tilt_angle_degrees: Some(2.0),
        acceleration_g: Some(1.0),
        thermal_gradient_c_per_m: Some(1.0),
        heat_dissipation_rate_w: Some(5.0),
        cooling_efficiency_percent: Some(85.0),
        timestamp: SystemTime::now(),
        measurement_quality: MeasurementQuality {
            overall_quality: 1.0,
            sensor_health: HashMap::new(),
            calibration_status: HashMap::new(),
            last_calibration: HashMap::new(),
        },
    }
}

#[tokio::test]
async fn test_environmental_monitor_creation() {
    let thresholds = EnvironmentalThresholds::default();
    let monitor = EnvironmentalMonitor::new(thresholds);
    
    // Test that monitor was created successfully
    // This is a basic test since we can't access private fields
    let _ = monitor;
}

#[tokio::test]
async fn test_environmental_conditions_update() {
    let thresholds = EnvironmentalThresholds::default();
    let mut monitor = EnvironmentalMonitor::new(thresholds);
    
    let conditions = create_test_environmental_conditions();
    
    // Test updating conditions
    let result = monitor.update_conditions(conditions);
    assert!(result.is_ok());
}

#[tokio::test]
async fn test_beacon_environmental_integration() {
    let beacon = create_test_beacon();
    
    // Test basic beacon functionality with environmental monitoring
    let status = beacon.get_status();
    let _ = status;
}

#[tokio::test]
async fn test_extreme_environmental_conditions() {
    let thresholds = EnvironmentalThresholds::default();
    let mut monitor = EnvironmentalMonitor::new(thresholds);
    
    // Test extreme conditions
    let extreme_conditions = ExtendedEnvironmentalConditions {
        base_conditions: EnvironmentalConditions::default(),
        air_temperature_c: Some(50.0), // High temperature
        humidity_percent: Some(95.0), // High humidity
        atmospheric_pressure_hpa: Some(960.0), // Low pressure
        wind_speed_ms: Some(30.0), // High wind
        wave_height_m: Some(6.0), // High waves
        solar_irradiance_wm2: None,
        internal_temperature_c: Some(70.0), // High internal temp
        cpu_temperature_c: Some(80.0), // High CPU temp
        battery_temperature_c: Some(45.0), // High battery temp
        enclosure_humidity_percent: Some(90.0),
        vibration_level_g: Some(2.5), // High vibration
        magnetic_field_strength_ut: None,
        wave_period_s: None,
        sea_state: None,
        tilt_angle_degrees: Some(20.0), // High tilt
        acceleration_g: Some(2.0), // High acceleration
        thermal_gradient_c_per_m: Some(8.0), // High thermal gradient
        heat_dissipation_rate_w: None,
        cooling_efficiency_percent: Some(50.0), // Low cooling efficiency
        timestamp: SystemTime::now(),
        measurement_quality: MeasurementQuality {
            overall_quality: 0.8,
            sensor_health: HashMap::new(),
            calibration_status: HashMap::new(),
            last_calibration: HashMap::new(),
        },
    };
    
    let result = monitor.update_conditions(extreme_conditions);
    // Should handle extreme conditions gracefully
    assert!(result.is_ok());
}