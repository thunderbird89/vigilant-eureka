// Beacon system main application
// Uses shared-positioning library for common functionality

mod beacon_controller;

use beacon_controller::{BeaconController, BeaconConfig, MessageVersion, EmergencyConfig};
use shared_positioning::{
    MessageBuilder, TransceiverInterface, MockTransceiver, GeodeticPosition,
    SystemConfig, ErrorLogger, ErrorSeverity, ConsoleLogHandler,
    GpsConfig, PowerConfig, CommunicationConfig,
    MockGpsManager, MockPowerManager, MockCommunicationManager
};
use uuid::Uuid;
use std::time::Duration;

fn main() {
    println!("=== BEACON CONTROLLER DEMO ===");
    
    // Initialize error logging
    let mut error_logger = ErrorLogger::new(100, ErrorSeverity::Warning);
    error_logger.add_output_handler(Box::new(ConsoleLogHandler::new(ErrorSeverity::Info)));
    
    // Create beacon configuration
    let beacon_config = BeaconConfig {
        beacon_id: Uuid::new_v4(),
        transmission_interval_ms: 5000, // 5 seconds
        message_version: MessageVersion::V3,
        gps_config: GpsConfig::default(),
        power_config: PowerConfig::default(),
        communication_config: CommunicationConfig::default(),
        emergency_config: EmergencyConfig::default(),
    };
    
    println!("Beacon ID: {}", beacon_config.beacon_id);
    println!("Transmission interval: {}ms", beacon_config.transmission_interval_ms);
    
    // Create mock managers for demonstration
    let gps_manager = match MockGpsManager::with_test_positions(beacon_config.gps_config.clone()) {
        Ok(manager) => manager,
        Err(e) => {
            eprintln!("Failed to create GPS manager: {}", e);
            return;
        }
    };
    
    let power_manager = MockPowerManager::new();
    let communication_manager = MockCommunicationManager::new();
    let transceiver = MockTransceiver::new(1);
    
    // Create beacon controller
    let mut beacon_controller = match BeaconController::new(
        beacon_config,
        gps_manager,
        power_manager,
        communication_manager,
        transceiver,
    ) {
        Ok(controller) => controller,
        Err(e) => {
            eprintln!("Failed to create beacon controller: {}", e);
            return;
        }
    };
    
    println!("\n=== BEACON CONTROLLER STATUS ===");
    let initial_status = beacon_controller.get_status();
    println!("Initial state: {:?}", initial_status.operational_state);
    println!("GPS status: {:?}", initial_status.gps_status);
    println!("Battery capacity: {:.1}%", initial_status.battery_status.capacity_percent);
    println!("Uptime: {:?}", initial_status.uptime);
    
    // Demonstrate configuration update
    println!("\n=== CONFIGURATION UPDATE DEMO ===");
    let mut updated_config = beacon_controller.get_config().clone();
    updated_config.transmission_interval_ms = 3000; // Change to 3 seconds
    updated_config.message_version = MessageVersion::V2;
    
    match beacon_controller.update_configuration(updated_config) {
        Ok(()) => println!("Configuration updated successfully"),
        Err(e) => eprintln!("Configuration update failed: {}", e),
    }
    
    // Demonstrate emergency handling
    println!("\n=== EMERGENCY HANDLING DEMO ===");
    match beacon_controller.handle_emergency(beacon_controller::EmergencyType::BatteryDepleted) {
        Ok(()) => println!("Emergency handled successfully"),
        Err(e) => eprintln!("Emergency handling failed: {}", e),
    }
    
    let emergency_status = beacon_controller.get_status();
    println!("State after emergency: {:?}", emergency_status.operational_state);
    
    // Demonstrate different emergency types
    println!("\n=== ADDITIONAL EMERGENCY SCENARIOS ===");
    
    let emergency_scenarios = vec![
        beacon_controller::EmergencyType::GpsSignalLost,
        beacon_controller::EmergencyType::TemperatureExtreme,
        beacon_controller::EmergencyType::CommunicationLost,
    ];
    
    for emergency in emergency_scenarios {
        println!("Testing emergency: {:?}", emergency);
        match beacon_controller.handle_emergency(emergency) {
            Ok(()) => println!("  Emergency handled successfully"),
            Err(e) => eprintln!("  Emergency handling failed: {}", e),
        }
    }
    
    // Show final status
    println!("\n=== FINAL BEACON STATUS ===");
    let final_status = beacon_controller.get_status();
    println!("Final state: {:?}", final_status.operational_state);
    println!("GPS status: {:?}", final_status.gps_status);
    println!("Battery capacity: {:.1}%", final_status.battery_status.capacity_percent);
    println!("Messages sent: {}", final_status.transmission_stats.messages_sent);
    println!("System health:");
    println!("  CPU usage: {:.1}%", final_status.system_health.cpu_usage_percent);
    println!("  Memory usage: {:.1}%", final_status.system_health.memory_usage_percent);
    println!("  GPS signal quality: {}", final_status.system_health.gps_signal_quality);
    println!("  Comm signal quality: {}", final_status.system_health.comm_signal_quality);
    
    println!("\n=== BEACON CONTROLLER DEMO COMPLETE ===");
    println!("Beacon controller features demonstrated:");
    println!("  - Configuration management and validation");
    println!("  - Operational state management");
    println!("  - Emergency handling and recovery");
    println!("  - GPS, power, and communication coordination");
    println!("  - System health monitoring");
    println!("  - Comprehensive error handling");
}