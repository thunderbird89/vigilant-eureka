use shared_positioning::{
    PowerManager, BasicPowerManager, MockPowerManager, PowerConfig, PowerOperationMode,
    BatteryStatus, ChargingStatus, PowerError
};
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Power Management System Demo ===\n");

    // Demonstrate basic power manager functionality
    demo_basic_power_manager()?;
    
    // Demonstrate mock power manager for testing
    demo_mock_power_manager()?;
    
    // Demonstrate threshold monitoring and alerts
    demo_threshold_monitoring()?;
    
    // Demonstrate power mode management
    demo_power_mode_management()?;
    
    // Demonstrate charging system
    demo_charging_system()?;

    println!("=== Power Management Demo Complete ===");
    Ok(())
}

fn demo_basic_power_manager() -> Result<(), PowerError> {
    println!("1. Basic Power Manager Demo");
    println!("---------------------------");
    
    let config = PowerConfig::default();
    let mut manager = BasicPowerManager::new(config)?;
    
    // Get initial battery status
    let status = manager.get_battery_status()?;
    println!("Initial battery status:");
    println!("  Voltage: {:.2}V", status.voltage_v);
    println!("  Current: {:.1}mA", status.current_ma);
    println!("  Capacity: {:.1}%", status.capacity_percent);
    println!("  Temperature: {:.1}°C", status.temperature_c);
    println!("  Health: {:?}", status.health);
    
    // Check estimated remaining time
    let remaining_time = manager.estimate_remaining_time()?;
    println!("  Estimated remaining time: {:.1} hours", remaining_time.as_secs_f32() / 3600.0);
    
    // Get charging status
    let charging = manager.get_charging_status()?;
    println!("  Charging status: {:?}", charging);
    
    println!();
    Ok(())
}

fn demo_mock_power_manager() -> Result<(), PowerError> {
    println!("2. Mock Power Manager Demo (for testing)");
    println!("----------------------------------------");
    
    let mut mock = MockPowerManager::new();
    
    // Simulate battery discharge
    println!("Simulating battery discharge...");
    mock.simulate_discharge(30.0);
    
    let status = mock.get_battery_status()?;
    println!("After discharge:");
    println!("  Capacity: {:.1}%", status.capacity_percent);
    println!("  Voltage: {:.2}V", status.voltage_v);
    
    // Simulate charging
    println!("\nSimulating battery charging...");
    mock.simulate_charge(15.0);
    
    let status = mock.get_battery_status()?;
    println!("After charging:");
    println!("  Capacity: {:.1}%", status.capacity_percent);
    println!("  Voltage: {:.2}V", status.voltage_v);
    
    println!();
    Ok(())
}

fn demo_threshold_monitoring() -> Result<(), PowerError> {
    println!("3. Threshold Monitoring and Alerts Demo");
    println!("---------------------------------------");
    
    let mut mock = MockPowerManager::new();
    
    // Test different battery levels and threshold violations
    let test_levels = vec![25.0, 15.0, 8.0, 3.0];
    
    for level in test_levels {
        mock.simulate_discharge(100.0 - level); // Set to specific level
        
        let violations = mock.check_thresholds()?;
        println!("Battery at {:.1}%:", level);
        
        if violations.is_empty() {
            println!("  ✓ No threshold violations");
        } else {
            for violation in violations {
                println!("  ⚠️  {}", violation);
            }
        }
        
        // Reset for next test
        mock.simulate_charge(100.0);
    }
    
    println!();
    Ok(())
}

fn demo_power_mode_management() -> Result<(), PowerError> {
    println!("4. Power Mode Management Demo");
    println!("-----------------------------");
    
    let mut mock = MockPowerManager::new();
    
    let modes = vec![
        PowerOperationMode::Normal,
        PowerOperationMode::PowerSave,
        PowerOperationMode::Emergency,
        PowerOperationMode::Shutdown,
    ];
    
    for mode in modes {
        mock.set_power_mode(mode.clone())?;
        let current_mode = mock.get_power_mode();
        let remaining_time = mock.estimate_remaining_time()?;
        
        println!("Mode: {:?}", current_mode);
        println!("  Estimated runtime: {:.1} hours", remaining_time.as_secs_f32() / 3600.0);
    }
    
    println!();
    Ok(())
}

fn demo_charging_system() -> Result<(), PowerError> {
    println!("5. Charging System Demo");
    println!("-----------------------");
    
    let mut mock = MockPowerManager::new();
    
    // Test different charging scenarios
    println!("Testing charging control:");
    
    // Enable charging
    mock.set_charging_enabled(true)?;
    println!("  ✓ Charging enabled");
    
    // Enable solar charging
    mock.set_solar_charging_enabled(true)?;
    println!("  ✓ Solar charging enabled");
    
    // Simulate different charging states
    let charging_states = vec![
        ChargingStatus::NotCharging,
        ChargingStatus::Charging { rate_ma: 500.0 },
        ChargingStatus::SolarCharging { rate_ma: 200.0, solar_voltage_v: 5.0 },
        ChargingStatus::ChargingComplete,
    ];
    
    for state in charging_states {
        mock.set_charging_status(state.clone());
        let current_state = mock.get_charging_status()?;
        println!("  Charging state: {:?}", current_state);
    }
    
    // Test emergency shutdown preparation
    println!("\nTesting emergency shutdown:");
    mock.prepare_emergency_shutdown()?;
    println!("  ✓ Emergency shutdown prepared");
    println!("  Current mode: {:?}", mock.get_power_mode());
    
    println!();
    Ok(())
}