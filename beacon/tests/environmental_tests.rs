use std::time::{Duration, SystemTime};
use uuid::Uuid;

use beacon::{BeaconController, BeaconConfig, MessageVersion, EmergencyType};
use shared_positioning::{
    GpsConfig, PowerConfig, CommunicationConfig, MockGpsManager, MockPowerManager,
    MockCommunicationManager, MockTransceiverInterface, ExtendedEnvironmentalConditions,
    EnvironmentalThresholds, EnvironmentalMonitor, PowerOperationMode, BatteryStatus,
    BatteryHealth, GpsStatus, AdaptationAction, EnvironmentalError
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
    
    let gps_manager = MockGpsManager::with_test_positions(config.gps_config.clone()).unwrap();
    let power_manager = MockPowerManager::new();
    let comm_manager = MockCommunicationManager::new();
    let transceiver = MockTransceiverInterface::new();
    
    BeaconController::new(config, gps_manager, power_manager, comm_manager, transceiver).unwrap()
}

#[tokio::test]
async fn test_extreme_cold_conditions() {
    let mut beacon = create_test_beacon();
    
    // Simulate extreme cold conditions
    let cold_conditions = ExtendedEnvironmentalConditions {
        temperature_c: -35.0, // Extreme cold
        humidity_percent: 85.0,
        pressure_hpa: 1030.0, // High pressure (cold front)
        wind_speed_ms: 20.0, // High wind
        wave_height_m: 3.0,
        visibility_m: 500.0, // Reduced visibility
        precipitation_mmh: 2.0, // Light snow
        uv_index: 1.0, // Low UV in winter
        air_quality_index: 30.0, // Good air quality
    };
    
    beacon.environmental_monitor.update_conditions(cold_conditions);
    
    // Test environmental monitoring response
    assert!(beacon.update_environmental_monitoring().is_ok());
    
    // Verify adaptation actions were taken
    let stats = beacon.environmental_monitor.get_stats();
    assert!(stats.adaptation_actions_taken > 0);
    
    // Test that power management adapts to cold
    beacon.power_manager.simulate_temperature_change(-35.0);
    let violations = beacon.power_manager.check_thresholds().unwrap();
    assert!(violations.iter().any(|v| matches!(v, shared_positioning::PowerError::TemperatureExtreme { .. })));
    
    // Test GPS performance in cold conditions
    beacon.gps_manager.set_accuracy_variation(2.0); // GPS accuracy may degrade in cold
    assert!(beacon.update_gps().is_ok());
    
    // Test transmission power adjustment for cold conditions
    assert!(beacon.handle_transmission().is_ok());
    
    // Verify system continues to operate despite extreme conditions
    let status = beacon.get_status();
    assert!(matches!(status.operational_state, 
        beacon::OperationalState::Normal | 
        beacon::OperationalState::PowerSave |
        beacon::OperationalState::Emergency
    ));
}

#[tokio::test]
async fn test_extreme_heat_conditions() {
    let mut beacon = create_test_beacon();
    
    // Simulate extreme heat conditions
    let hot_conditions = ExtendedEnvironmentalConditions {
        temperature_c: 55.0, // Extreme heat
        humidity_percent: 95.0, // High humidity
        pressure_hpa: 995.0, // Low pressure (heat wave)
        wind_speed_ms: 5.0, // Low wind (stagnant air)
        wave_height_m: 1.0,
        visibility_m: 2000.0, // Haze reduces visibility
        precipitation_mmh: 0.0, // No precipitation
        uv_index: 12.0, // Extreme UV
        air_quality_index: 180.0, // Poor air quality
    };
    
    beacon.environmental_monitor.update_conditions(hot_conditions);
    
    // Test environmental monitoring response
    assert!(beacon.update_environmental_monitoring().is_ok());
    
    // Test power management response to heat
    beacon.power_manager.simulate_temperature_change(55.0);
    let violations = beacon.power_manager.check_thresholds().unwrap();
    assert!(violations.iter().any(|v| matches!(v, shared_positioning::PowerError::TemperatureExtreme { .. })));
    
    // Heat should trigger power save mode
    assert!(beacon.handle_emergency(EmergencyType::TemperatureExtreme).is_ok());
    assert_eq!(beacon.power_manager.get_power_mode(), PowerOperationMode::PowerSave);
    
    // Test that system reduces activity in extreme heat
    let status = beacon.get_status();
    assert!(matches!(status.operational_state, 
        beacon::OperationalState::PowerSave |
        beacon::OperationalState::Emergency
    ));
}

#[tokio::test]
async fn test_storm_conditions() {
    let mut beacon = create_test_beacon();
    
    // Simulate severe storm conditions
    let storm_conditions = ExtendedEnvironmentalConditions {
        temperature_c: 15.0,
        humidity_percent: 98.0, // Near saturation
        pressure_hpa: 960.0, // Very low pressure (severe storm)
        wind_speed_ms: 45.0, // Hurricane-force winds
        wave_height_m: 8.0, // Very high waves
        visibility_m: 100.0, // Very poor visibility
        precipitation_mmh: 50.0, // Heavy rain
        uv_index: 2.0, // Low due to cloud cover
        air_quality_index: 80.0, // Moderate (dust/particles)
    };
    
    beacon.environmental_monitor.update_conditions(storm_conditions);
    
    // Test environmental monitoring response
    assert!(beacon.update_environmental_monitoring().is_ok());
    
    // Storm conditions should trigger multiple adaptations
    let stats = beacon.environmental_monitor.get_stats();
    assert!(stats.adaptation_actions_taken > 0);
    assert!(stats.extreme_conditions_detected > 0);
    
    // Test GPS performance in storm (signal may be degraded)
    beacon.gps_manager.set_accuracy_variation(5.0); // Degraded accuracy
    beacon.gps_manager.simulate_signal_loss(true); // Intermittent signal loss
    
    let gps_result = beacon.update_gps();
    // GPS may fail or have degraded performance
    if gps_result.is_err() {
        assert_eq!(beacon.gps_manager.get_status(), GpsStatus::SignalLost);
    }
    
    // Test transmission reliability in storm conditions
    beacon.transceiver.set_failure_rate(0.3); // 30% failure rate due to interference
    let transmission_result = beacon.handle_transmission();
    // May succeed or fail, but should handle gracefully
    
    // Test power system stability in storm
    beacon.power_manager.simulate_discharge(5.0); // Increased power consumption
    let power_status = beacon.power_manager.get_battery_status().unwrap();
    assert!(power_status.capacity_percent > 0.0);
    
    // System should continue operating despite storm conditions
    let status = beacon.get_status();
    assert!(!matches!(status.operational_state, beacon::OperationalState::Shutdown));
}

#[tokio::test]
async fn test_marine_fog_conditions() {
    let mut beacon = create_test_beacon();
    
    // Simulate dense marine fog
    let fog_conditions = ExtendedEnvironmentalConditions {
        temperature_c: 12.0,
        humidity_percent: 99.0, // Near saturation
        pressure_hpa: 1015.0, // Stable pressure
        wind_speed_ms: 2.0, // Light wind (fog persists)
        wave_height_m: 0.5, // Calm seas
        visibility_m: 50.0, // Dense fog
        precipitation_mmh: 0.5, // Light drizzle
        uv_index: 1.0, // Very low due to fog
        air_quality_index: 60.0, // Moderate
    };
    
    beacon.environmental_monitor.update_conditions(fog_conditions);
    
    // Test environmental monitoring response
    assert!(beacon.update_environmental_monitoring().is_ok());
    
    // Fog primarily affects visibility, not system operation
    let stats = beacon.environmental_monitor.get_stats();
    assert!(stats.total_updates > 0);
    
    // GPS should still work in fog (satellite signals penetrate)
    assert!(beacon.update_gps().is_ok());
    
    // Transmission should work normally in fog
    assert!(beacon.handle_transmission().is_ok());
    
    // System should operate normally in fog conditions
    let status = beacon.get_status();
    assert_eq!(status.operational_state, beacon::OperationalState::Normal);
}

#[tokio::test]
async fn test_high_altitude_conditions() {
    let mut beacon = create_test_beacon();
    
    // Simulate high altitude conditions (mountain deployment)
    let altitude_conditions = ExtendedEnvironmentalConditions {
        temperature_c: -10.0, // Cold at altitude
        humidity_percent: 40.0, // Low humidity
        pressure_hpa: 850.0, // Low pressure at altitude
        wind_speed_ms: 25.0, // High winds at altitude
        wave_height_m: 0.0, // Not applicable at altitude
        visibility_m: 15000.0, // Excellent visibility
        precipitation_mmh: 0.0, // Clear conditions
        uv_index: 10.0, // High UV at altitude
        air_quality_index: 20.0, // Excellent air quality
    };
    
    beacon.environmental_monitor.update_conditions(altitude_conditions);
    
    // Test environmental monitoring response
    assert!(beacon.update_environmental_monitoring().is_ok());
    
    // Test GPS performance at altitude (should be better)
    beacon.gps_manager.set_accuracy_variation(-1.0); // Better accuracy at altitude
    assert!(beacon.update_gps().is_ok());
    
    // Test power system at altitude (cold affects battery)
    beacon.power_manager.simulate_temperature_change(-10.0);
    let battery_status = beacon.power_manager.get_battery_status().unwrap();
    assert!(battery_status.temperature_c < 0.0);
    
    // Test transmission at altitude (may have better range)
    assert!(beacon.handle_transmission().is_ok());
    
    // System should adapt to altitude conditions
    let status = beacon.get_status();
    assert!(!matches!(status.operational_state, beacon::OperationalState::Error(_)));
}

#[tokio::test]
async fn test_tropical_cyclone_conditions() {
    let mut beacon = create_test_beacon();
    
    // Simulate tropical cyclone conditions
    let cyclone_conditions = ExtendedEnvironmentalConditions {
        temperature_c: 28.0, // Warm tropical air
        humidity_percent: 95.0, // Very high humidity
        pressure_hpa: 940.0, // Extremely low pressure (intense cyclone)
        wind_speed_ms: 60.0, // Category 4 hurricane winds
        wave_height_m: 12.0, // Extreme wave heights
        visibility_m: 200.0, // Very poor visibility
        precipitation_mmh: 100.0, // Torrential rain
        uv_index: 1.0, // Low due to dense cloud cover
        air_quality_index: 120.0, // Poor due to debris
    };
    
    beacon.environmental_monitor.update_conditions(cyclone_conditions);
    
    // Test environmental monitoring response
    assert!(beacon.update_environmental_monitoring().is_ok());
    
    // Extreme conditions should trigger emergency adaptations
    let stats = beacon.environmental_monitor.get_stats();
    assert!(stats.extreme_conditions_detected > 0);
    assert!(stats.adaptation_actions_taken > 0);
    
    // GPS may be severely affected
    beacon.gps_manager.set_accuracy_variation(10.0); // Very poor accuracy
    beacon.gps_manager.simulate_signal_loss(true); // Frequent signal loss
    
    // Power system should enter emergency mode
    beacon.power_manager.simulate_discharge(10.0); // Increased consumption due to stress
    
    // Transmission may be severely impacted
    beacon.transceiver.set_failure_rate(0.7); // 70% failure rate
    
    // System should enter emergency mode
    assert!(beacon.handle_emergency(EmergencyType::SystemOverload).is_ok());
    
    let status = beacon.get_status();
    assert!(matches!(status.operational_state, 
        beacon::OperationalState::Emergency |
        beacon::OperationalState::PowerSave
    ));
}

#[tokio::test]
async fn test_arctic_conditions() {
    let mut beacon = create_test_beacon();
    
    // Simulate arctic conditions
    let arctic_conditions = ExtendedEnvironmentalConditions {
        temperature_c: -45.0, // Extreme arctic cold
        humidity_percent: 70.0, // Moderate humidity (cold air holds less moisture)
        pressure_hpa: 1040.0, // High pressure (arctic high)
        wind_speed_ms: 15.0, // Moderate wind
        wave_height_m: 2.0, // Ice may limit wave action
        visibility_m: 1000.0, // Reduced by blowing snow
        precipitation_mmh: 1.0, // Light snow
        uv_index: 3.0, // Moderate due to snow reflection
        air_quality_index: 15.0, // Excellent air quality
    };
    
    beacon.environmental_monitor.update_conditions(arctic_conditions);
    
    // Test environmental monitoring response
    assert!(beacon.update_environmental_monitoring().is_ok());
    
    // Extreme cold should trigger emergency procedures
    beacon.power_manager.simulate_temperature_change(-45.0);
    let violations = beacon.power_manager.check_thresholds().unwrap();
    assert!(violations.iter().any(|v| matches!(v, shared_positioning::PowerError::TemperatureExtreme { .. })));
    
    // Battery performance severely degraded in extreme cold
    beacon.power_manager.simulate_discharge(20.0); // Rapid discharge in cold
    let battery_status = beacon.power_manager.get_battery_status().unwrap();
    assert!(battery_status.health == BatteryHealth::Poor || battery_status.health == BatteryHealth::Critical);
    
    // GPS may have issues in extreme cold
    beacon.gps_manager.set_accuracy_variation(3.0);
    
    // System should enter emergency mode due to extreme conditions
    assert!(beacon.handle_emergency(EmergencyType::TemperatureExtreme).is_ok());
    
    let status = beacon.get_status();
    assert!(matches!(status.operational_state, 
        beacon::OperationalState::Emergency |
        beacon::OperationalState::PowerSave
    ));
}

#[tokio::test]
async fn test_desert_conditions() {
    let mut beacon = create_test_beacon();
    
    // Simulate desert conditions
    let desert_conditions = ExtendedEnvironmentalConditions {
        temperature_c: 50.0, // Extreme desert heat
        humidity_percent: 15.0, // Very low humidity
        pressure_hpa: 1010.0, // Normal pressure
        wind_speed_ms: 10.0, // Moderate wind
        wave_height_m: 0.0, // Not applicable in desert
        visibility_m: 5000.0, // Reduced by dust/haze
        precipitation_mmh: 0.0, // No precipitation
        uv_index: 13.0, // Extreme UV
        air_quality_index: 150.0, // Poor due to dust
    };
    
    beacon.environmental_monitor.update_conditions(desert_conditions);
    
    // Test environmental monitoring response
    assert!(beacon.update_environmental_monitoring().is_ok());
    
    // Extreme heat and UV should trigger adaptations
    let stats = beacon.environmental_monitor.get_stats();
    assert!(stats.adaptation_actions_taken > 0);
    
    // Heat affects power system
    beacon.power_manager.simulate_temperature_change(50.0);
    let violations = beacon.power_manager.check_thresholds().unwrap();
    assert!(violations.iter().any(|v| matches!(v, shared_positioning::PowerError::TemperatureExtreme { .. })));
    
    // GPS should work well in clear desert conditions
    beacon.gps_manager.set_accuracy_variation(-0.5); // Better accuracy in clear conditions
    assert!(beacon.update_gps().is_ok());
    
    // System should adapt to desert conditions
    assert!(beacon.handle_emergency(EmergencyType::TemperatureExtreme).is_ok());
    
    let status = beacon.get_status();
    assert!(matches!(status.operational_state, 
        beacon::OperationalState::PowerSave |
        beacon::OperationalState::Emergency
    ));
}

#[tokio::test]
async fn test_rapid_weather_changes() {
    let mut beacon = create_test_beacon();
    
    // Test rapid weather transitions (common in marine environments)
    let conditions_sequence = vec![
        ExtendedEnvironmentalConditions {
            temperature_c: 20.0,
            humidity_percent: 60.0,
            pressure_hpa: 1015.0,
            wind_speed_ms: 5.0,
            wave_height_m: 1.0,
            visibility_m: 10000.0,
            precipitation_mmh: 0.0,
            uv_index: 6.0,
            air_quality_index: 50.0,
        },
        ExtendedEnvironmentalConditions {
            temperature_c: 15.0, // Temperature drop
            humidity_percent: 85.0, // Humidity increase
            pressure_hpa: 995.0, // Pressure drop (front approaching)
            wind_speed_ms: 20.0, // Wind increase
            wave_height_m: 3.0, // Wave increase
            visibility_m: 2000.0, // Visibility decrease
            precipitation_mmh: 10.0, // Rain starts
            uv_index: 2.0,
            air_quality_index: 70.0,
        },
        ExtendedEnvironmentalConditions {
            temperature_c: 8.0, // Further temperature drop
            humidity_percent: 95.0, // Near saturation
            pressure_hpa: 980.0, // Low pressure system
            wind_speed_ms: 35.0, // Storm conditions
            wave_height_m: 6.0, // High waves
            visibility_m: 500.0, // Poor visibility
            precipitation_mmh: 25.0, // Heavy rain
            uv_index: 1.0,
            air_quality_index: 90.0,
        },
    ];
    
    for (i, conditions) in conditions_sequence.iter().enumerate() {
        beacon.environmental_monitor.update_conditions(*conditions);
        assert!(beacon.update_environmental_monitoring().is_ok());
        
        // Test system adaptation to changing conditions
        let stats = beacon.environmental_monitor.get_stats();
        assert_eq!(stats.total_updates, i + 1);
        
        // Verify system continues to operate through changes
        let status = beacon.get_status();
        assert!(!matches!(status.operational_state, beacon::OperationalState::Shutdown));
        
        // Allow time for system to adapt
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    
    // Verify that rapid changes triggered appropriate adaptations
    let final_stats = beacon.environmental_monitor.get_stats();
    assert!(final_stats.adaptation_actions_taken > 0);
    assert!(final_stats.extreme_conditions_detected > 0);
}

#[tokio::test]
async fn test_environmental_threshold_configuration() {
    let mut beacon = create_test_beacon();
    
    // Test custom environmental thresholds
    let custom_thresholds = EnvironmentalThresholds {
        temperature_min_c: -30.0,
        temperature_max_c: 45.0,
        humidity_max_percent: 90.0,
        pressure_min_hpa: 970.0,
        pressure_max_hpa: 1040.0,
        wind_speed_max_ms: 25.0,
        wave_height_max_m: 5.0,
        visibility_min_m: 100.0,
        precipitation_max_mmh: 20.0,
        uv_index_max: 10.0,
        air_quality_max: 100.0,
    };
    
    beacon.environmental_monitor = EnvironmentalMonitor::new(custom_thresholds);
    
    // Test conditions that exceed custom thresholds
    let extreme_conditions = ExtendedEnvironmentalConditions {
        temperature_c: 50.0, // Exceeds custom max
        humidity_percent: 95.0, // Exceeds custom max
        pressure_hpa: 960.0, // Below custom min
        wind_speed_ms: 30.0, // Exceeds custom max
        wave_height_m: 6.0, // Exceeds custom max
        visibility_m: 50.0, // Below custom min
        precipitation_mmh: 25.0, // Exceeds custom max
        uv_index: 12.0, // Exceeds custom max
        air_quality_index: 150.0, // Exceeds custom max
    };
    
    beacon.environmental_monitor.update_conditions(extreme_conditions);
    assert!(beacon.update_environmental_monitoring().is_ok());
    
    // Verify that multiple threshold violations were detected
    let stats = beacon.environmental_monitor.get_stats();
    assert!(stats.extreme_conditions_detected > 0);
    assert!(stats.adaptation_actions_taken > 0);
}

#[tokio::test]
async fn test_environmental_data_logging() {
    let mut beacon = create_test_beacon();
    
    // Test that environmental data is properly logged over time
    let test_conditions = vec![
        ExtendedEnvironmentalConditions {
            temperature_c: 20.0,
            humidity_percent: 60.0,
            pressure_hpa: 1015.0,
            wind_speed_ms: 5.0,
            wave_height_m: 1.0,
            visibility_m: 10000.0,
            precipitation_mmh: 0.0,
            uv_index: 6.0,
            air_quality_index: 50.0,
        },
        ExtendedEnvironmentalConditions {
            temperature_c: 25.0,
            humidity_percent: 65.0,
            pressure_hpa: 1012.0,
            wind_speed_ms: 8.0,
            wave_height_m: 1.5,
            visibility_m: 8000.0,
            precipitation_mmh: 2.0,
            uv_index: 7.0,
            air_quality_index: 60.0,
        },
        ExtendedEnvironmentalConditions {
            temperature_c: 30.0,
            humidity_percent: 70.0,
            pressure_hpa: 1008.0,
            wind_speed_ms: 12.0,
            wave_height_m: 2.0,
            visibility_m: 6000.0,
            precipitation_mmh: 5.0,
            uv_index: 8.0,
            air_quality_index: 75.0,
        },
    ];
    
    for conditions in test_conditions {
        beacon.environmental_monitor.update_conditions(conditions);
        assert!(beacon.update_environmental_monitoring().is_ok());
        tokio::time::sleep(Duration::from_millis(10)).await;
    }
    
    // Verify environmental data history is maintained
    let stats = beacon.environmental_monitor.get_stats();
    assert_eq!(stats.total_updates, 3);
    
    // Test environmental report generation
    let report = beacon.environmental_monitor.generate_report();
    assert!(report.len() > 0);
}