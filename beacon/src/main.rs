// Beacon system main application
// Uses shared-positioning library for common functionality

use shared_positioning::{
    MessageBuilder, TransceiverInterface, MockTransceiver, GeodeticPosition,
    SystemConfig, ErrorLogger, ErrorSeverity, ConsoleLogHandler
};
use uuid::Uuid;
use std::time::Duration;

fn main() {
    println!("=== BEACON SYSTEM DEMO ===");
    
    // Initialize error logging
    let mut error_logger = ErrorLogger::new(100, ErrorSeverity::Warning);
    error_logger.add_output_handler(Box::new(ConsoleLogHandler::new(ErrorSeverity::Info)));
    
    // Create beacon configuration
    let config = SystemConfig::new();
    println!("Beacon system initialized with sound speed: {:.1} m/s", config.sound_speed_m_per_s);
    
    // Initialize message builder for beacon transmissions
    let message_builder = MessageBuilder::new();
    
    // Create mock transceiver for demonstration
    let mut transceiver = MockTransceiver::new(1);
    let transceiver_config = shared_positioning::TransceiverConfig::default();
    
    if let Err(e) = transceiver.configure(transceiver_config) {
        eprintln!("Failed to configure transceiver: {}", e);
        return;
    }
    
    // Generate unique beacon ID
    let beacon_uuid = Uuid::new_v4();
    println!("Beacon UUID: {}", beacon_uuid);
    
    // Beacon position (example coordinates)
    let beacon_position = GeodeticPosition {
        latitude: 32.123456,
        longitude: -117.654321,
        depth: 5.0,
    };
    
    println!("Beacon position: lat={:.6}, lon={:.6}, depth={:.1}m", 
             beacon_position.latitude, beacon_position.longitude, beacon_position.depth);
    
    // Demonstrate message building and transmission
    println!("\n=== MESSAGE TRANSMISSION DEMO ===");
    
    for sequence in 1..=5 {
        // Build V3 message with UUID
        match message_builder.build_v3_message(beacon_uuid, beacon_position, 200, sequence) {
            Ok(message_data) => {
                println!("Built V3 message #{} ({} bytes)", sequence, message_data.len());
                
                // Transmit message
                match transceiver.transmit_message(&message_data) {
                    Ok(()) => {
                        let tx_status = transceiver.get_transmission_status();
                        println!("  Transmitted successfully (total: {})", tx_status.transmission_count);
                    }
                    Err(e) => {
                        eprintln!("  Transmission failed: {}", e);
                    }
                }
            }
            Err(e) => {
                eprintln!("Failed to build message: {}", e);
            }
        }
        
        // Simulate transmission interval
        std::thread::sleep(Duration::from_millis(100));
    }
    
    // Show transceiver status
    let status = transceiver.get_status();
    println!("\n=== TRANSCEIVER STATUS ===");
    println!("Connected: {}", status.is_connected);
    if let Some(signal) = status.signal_strength {
        println!("Signal strength: {}", signal);
    }
    if let Some(battery) = status.battery_level {
        println!("Battery level: {}%", battery);
    }
    
    let tx_status = transceiver.get_transmission_status();
    println!("Messages transmitted: {}", tx_status.transmission_count);
    println!("Transmission failures: {}", tx_status.transmission_failures);
    println!("Current power level: {}", tx_status.current_power_level);
    
    // Demonstrate backward compatibility with V1 and V2 messages
    println!("\n=== BACKWARD COMPATIBILITY DEMO ===");
    
    // Build V1 message (legacy format with u16 ID)
    let legacy_id = (beacon_uuid.as_bytes()[0] as u16) << 8 | (beacon_uuid.as_bytes()[1] as u16);
    match message_builder.build_v1_message(legacy_id, beacon_position, 200, 100) {
        Ok(v1_message) => {
            println!("Built V1 message ({} bytes) for legacy compatibility", v1_message.len());
            if let Err(e) = transceiver.transmit_message(&v1_message) {
                eprintln!("V1 transmission failed: {}", e);
            }
        }
        Err(e) => {
            eprintln!("Failed to build V1 message: {}", e);
        }
    }
    
    // Build V2 message (compact format)
    match message_builder.build_v2_message(legacy_id, beacon_position, 200, 101) {
        Ok(v2_message) => {
            println!("Built V2 message ({} bytes) for compact transmission", v2_message.len());
            if let Err(e) = transceiver.transmit_message(&v2_message) {
                eprintln!("V2 transmission failed: {}", e);
            }
        }
        Err(e) => {
            eprintln!("Failed to build V2 message: {}", e);
        }
    }
    
    println!("\n=== BEACON DEMO COMPLETE ===");
    println!("Shared library components used successfully:");
    println!("  - MessageBuilder for V1, V2, and V3 message formats");
    println!("  - TransceiverInterface for hardware abstraction");
    println!("  - GeodeticPosition for coordinate representation");
    println!("  - SystemConfig for configuration management");
    println!("  - Error handling and logging systems");
}