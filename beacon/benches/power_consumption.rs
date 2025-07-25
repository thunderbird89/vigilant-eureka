use criterion::{black_box, criterion_group, criterion_main, Criterion, BenchmarkId};
use std::time::{Duration, SystemTime};
use uuid::Uuid;

use beacon::{BeaconController, BeaconConfig, MessageVersion, EmergencyConfig};
use shared_positioning::{
    GpsConfig, PowerConfig, CommunicationConfig, MockGpsManager, MockPowerManager,
    MockCommunicationManager, MockTransceiverInterface, PowerOperationMode, BatteryStatus,
    PowerManager, GpsManager, CommunicationManager, TransceiverInterface
};

fn create_test_beacon() -> BeaconController<MockGpsManager, MockPowerManager, MockCommunicationManager, MockTransceiverInterface> {
    let config = BeaconConfig {
        beacon_id: Uuid::new_v4(),
        transmission_interval_ms: 5000,
        message_version: MessageVersion::V3,
        gps_config: GpsConfig::default(),
        power_config: PowerConfig::default(),
        communication_config: CommunicationConfig::default(),
        emergency_config: EmergencyConfig::default(),
    };
    
    let gps_manager = MockGpsManager::with_test_positions(config.gps_config.clone()).unwrap();
    let power_manager = MockPowerManager::new();
    let comm_manager = MockCommunicationManager::new();
    let transceiver = MockTransceiverInterface::new();
    
    BeaconController::new(config, gps_manager, power_manager, comm_manager, transceiver).unwrap()
}

fn benchmark_power_mode_transitions(c: &mut Criterion) {
    let mut group = c.benchmark_group("power_mode_transitions");
    
    let modes = vec![
        PowerOperationMode::Normal,
        PowerOperationMode::PowerSave,
        PowerOperationMode::Emergency,
        PowerOperationMode::Shutdown,
    ];
    
    for mode in modes {
        group.bench_with_input(
            BenchmarkId::new("set_power_mode", format!("{:?}", mode)),
            &mode,
            |b, mode| {
                let mut beacon = create_test_beacon();
                b.iter(|| {
                    black_box(beacon.power_manager.set_power_mode(mode.clone()).unwrap());
                });
            },
        );
    }
    
    group.finish();
}

fn benchmark_battery_status_monitoring(c: &mut Criterion) {
    let mut group = c.benchmark_group("battery_monitoring");
    
    group.bench_function("get_battery_status", |b| {
        let beacon = create_test_beacon();
        b.iter(|| {
            black_box(beacon.power_manager.get_battery_status().unwrap());
        });
    });
    
    group.bench_function("check_power_thresholds", |b| {
        let beacon = create_test_beacon();
        b.iter(|| {
            black_box(beacon.power_manager.check_thresholds().unwrap());
        });
    });
    
    group.bench_function("estimate_remaining_time", |b| {
        let beacon = create_test_beacon();
        b.iter(|| {
            black_box(beacon.power_manager.estimate_remaining_time().unwrap());
        });
    });
    
    group.finish();
}

fn benchmark_power_consumption_simulation(c: &mut Criterion) {
    let mut group = c.benchmark_group("power_consumption_simulation");
    
    let discharge_amounts = vec![1.0, 5.0, 10.0, 25.0];
    
    for amount in discharge_amounts {
        group.bench_with_input(
            BenchmarkId::new("simulate_discharge", amount),
            &amount,
            |b, &amount| {
                let mut beacon = create_test_beacon();
                b.iter(|| {
                    beacon.power_manager.simulate_discharge(amount);
                    black_box(beacon.power_manager.get_battery_status().unwrap());
                });
            },
        );
    }
    
    group.finish();
}

fn benchmark_power_statistics_tracking(c: &mut Criterion) {
    let mut group = c.benchmark_group("power_statistics");
    
    group.bench_function("get_power_stats", |b| {
        let beacon = create_test_beacon();
        b.iter(|| {
            black_box(beacon.power_manager.get_power_stats());
        });
    });
    
    group.bench_function("reset_power_stats", |b| {
        let mut beacon = create_test_beacon();
        b.iter(|| {
            beacon.power_manager.reset_power_stats();
        });
    });
    
    group.finish();
}

fn benchmark_emergency_power_scenarios(c: &mut Criterion) {
    let mut group = c.benchmark_group("emergency_power_scenarios");
    
    group.bench_function("prepare_emergency_shutdown", |b| {
        let mut beacon = create_test_beacon();
        b.iter(|| {
            black_box(beacon.power_manager.prepare_emergency_shutdown().unwrap());
        });
    });
    
    group.bench_function("critical_battery_handling", |b| {
        let mut beacon = create_test_beacon();
        b.iter(|| {
            // Simulate critical battery condition
            beacon.power_manager.simulate_discharge(95.0);
            let status = beacon.power_manager.get_battery_status().unwrap();
            black_box(status.is_critical());
        });
    });
    
    group.finish();
}

fn benchmark_power_configuration_updates(c: &mut Criterion) {
    let mut group = c.benchmark_group("power_configuration");
    
    group.bench_function("configure_power_thresholds", |b| {
        let mut beacon = create_test_beacon();
        let new_config = PowerConfig {
            low_battery_threshold_percent: 25.0,
            critical_battery_threshold_percent: 15.0,
            emergency_battery_threshold_percent: 8.0,
            power_save_mode_threshold_percent: 35.0,
            charging_enabled: true,
            solar_charging_enabled: true,
            temperature_min_c: -25.0,
            temperature_max_c: 65.0,
            voltage_min_v: 2.8,
            voltage_max_v: 4.3,
            current_max_ma: 2500.0,
            monitoring_interval_ms: 500,
        };
        
        b.iter(|| {
            black_box(beacon.power_manager.configure_power_thresholds(new_config.clone()).unwrap());
        });
    });
    
    group.finish();
}

fn benchmark_continuous_power_monitoring(c: &mut Criterion) {
    let mut group = c.benchmark_group("continuous_monitoring");
    
    group.bench_function("power_monitoring_cycle", |b| {
        let mut beacon = create_test_beacon();
        b.iter(|| {
            // Simulate a complete power monitoring cycle
            let _status = black_box(beacon.power_manager.get_battery_status().unwrap());
            let _charging = black_box(beacon.power_manager.get_charging_status().unwrap());
            let _violations = black_box(beacon.power_manager.check_thresholds().unwrap());
            let _remaining = black_box(beacon.power_manager.estimate_remaining_time().unwrap());
            let _stats = black_box(beacon.power_manager.get_power_stats());
        });
    });
    
    group.bench_function("extended_operation_simulation", |b| {
        let mut beacon = create_test_beacon();
        b.iter(|| {
            // Simulate extended operation with gradual battery discharge
            for i in 0..100 {
                beacon.power_manager.simulate_discharge(0.1); // Small discharge increments
                if i % 10 == 0 {
                    black_box(beacon.power_manager.get_battery_status().unwrap());
                }
            }
        });
    });
    
    group.finish();
}

fn benchmark_power_mode_efficiency(c: &mut Criterion) {
    let mut group = c.benchmark_group("power_mode_efficiency");
    
    let modes = vec![
        ("Normal", PowerOperationMode::Normal),
        ("PowerSave", PowerOperationMode::PowerSave),
        ("Emergency", PowerOperationMode::Emergency),
    ];
    
    for (name, mode) in modes {
        group.bench_with_input(
            BenchmarkId::new("mode_efficiency", name),
            &mode,
            |b, mode| {
                let mut beacon = create_test_beacon();
                beacon.power_manager.set_power_mode(mode.clone()).unwrap();
                
                b.iter(|| {
                    // Simulate operations in different power modes
                    let _remaining = black_box(beacon.power_manager.estimate_remaining_time().unwrap());
                    beacon.power_manager.simulate_discharge(0.01); // Minimal discharge
                    let _status = black_box(beacon.power_manager.get_battery_status().unwrap());
                });
            },
        );
    }
    
    group.finish();
}

fn benchmark_charging_system_performance(c: &mut Criterion) {
    let mut group = c.benchmark_group("charging_system");
    
    group.bench_function("charging_status_updates", |b| {
        let mut beacon = create_test_beacon();
        b.iter(|| {
            beacon.power_manager.set_charging_enabled(true).unwrap();
            beacon.power_manager.set_solar_charging_enabled(true).unwrap();
            black_box(beacon.power_manager.get_charging_status().unwrap());
        });
    });
    
    group.bench_function("charge_simulation", |b| {
        let mut beacon = create_test_beacon();
        b.iter(|| {
            beacon.power_manager.simulate_charge(1.0);
            black_box(beacon.power_manager.get_battery_status().unwrap());
        });
    });
    
    group.finish();
}

fn benchmark_temperature_monitoring(c: &mut Criterion) {
    let mut group = c.benchmark_group("temperature_monitoring");
    
    let temperatures = vec![-30.0, -10.0, 0.0, 25.0, 45.0, 70.0];
    
    for temp in temperatures {
        group.bench_with_input(
            BenchmarkId::new("temperature_change", temp),
            &temp,
            |b, &temp| {
                let mut beacon = create_test_beacon();
                b.iter(|| {
                    beacon.power_manager.simulate_temperature_change(temp);
                    let violations = black_box(beacon.power_manager.check_thresholds().unwrap());
                    black_box(violations.len());
                });
            },
        );
    }
    
    group.finish();
}

criterion_group!(
    power_benches,
    benchmark_power_mode_transitions,
    benchmark_battery_status_monitoring,
    benchmark_power_consumption_simulation,
    benchmark_power_statistics_tracking,
    benchmark_emergency_power_scenarios,
    benchmark_power_configuration_updates,
    benchmark_continuous_power_monitoring,
    benchmark_power_mode_efficiency,
    benchmark_charging_system_performance,
    benchmark_temperature_monitoring
);

criterion_main!(power_benches);