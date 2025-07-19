//! Hardware abstraction layer demonstration
//! 
//! This example shows how to use the different transceiver interfaces
//! for communicating with JANUS transceivers.

use trilateration::hardware::{
    TransceiverInterface, TransceiverConfig, MockTransceiver, 
    SerialTransceiver, I2CTransceiver, CommError
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Underwater Positioning System - Hardware Demo ===\n");
    
    // Demonstrate Mock Transceiver
    demo_mock_transceiver()?;
    
    // Demonstrate Serial Transceiver
    demo_serial_transceiver()?;
    
    // Demonstrate I2C Transceiver
    demo_i2c_transceiver()?;
    
    // Demonstrate Error Handling
    demo_error_handling()?;
    
    println!("Hardware abstraction layer demo completed successfully!");
    Ok(())
}

fn demo_mock_transceiver() -> Result<(), Box<dyn std::error::Error>> {
    println!("--- Mock Transceiver Demo ---");
    
    let mut mock = MockTransceiver::new(1);
    println!("Created mock transceiver with ID: {}", mock.get_id());
    
    // Add some test anchor messages
    mock.add_anchor_message(101, 47.6062, -122.3321, 15.0); // Seattle coordinates
    mock.add_anchor_message(102, 47.6205, -122.3493, 20.0);
    mock.add_anchor_message(103, 47.5951, -122.3316, 12.0);
    
    println!("Added {} test messages", mock.queued_message_count());
    
    // Read messages
    for i in 0..3 {
        match mock.read_message()? {
            Some(message) => {
                println!("Message {}: {} bytes, timestamp: {}", 
                    i + 1, message.data.len(), message.timestamp_ms);
            }
            None => println!("No message available"),
        }
    }
    
    // Test error simulation
    mock.simulate_errors(true, 0.5); // 50% error rate
    println!("Enabled error simulation");
    
    // Try to read with errors
    for _ in 0..3 {
        match mock.read_message() {
            Ok(Some(_)) => println!("Message received successfully"),
            Ok(None) => println!("No message available"),
            Err(e) => println!("Error: {}", e),
        }
    }
    
    let status = mock.get_status();
    println!("Final status: connected={}, errors={}, messages={}\n", 
        status.connected, status.error_count, status.messages_received);
    
    Ok(())
}

fn demo_serial_transceiver() -> Result<(), Box<dyn std::error::Error>> {
    println!("--- Serial Transceiver Demo ---");
    
    let config = TransceiverConfig::serial(2, 115200);
    let mut serial = SerialTransceiver::new(2, "/dev/ttyUSB0".to_string(), config)?;
    
    println!("Created serial transceiver for port: /dev/ttyUSB0");
    println!("Baud rate: 115200");
    
    // Try to connect (will simulate connection)
    match serial.connect() {
        Ok(()) => println!("Connected successfully"),
        Err(e) => println!("Connection failed: {}", e),
    }
    
    // Check status
    let status = serial.get_status();
    println!("Status: connected={}, firmware={:?}", 
        status.connected, status.firmware_version);
    
    // Try to send a configuration message
    let config_msg = vec![0x01, 0x02, 0x03]; // Mock configuration
    match serial.send_message(&config_msg) {
        Ok(()) => println!("Configuration message sent"),
        Err(e) => println!("Send failed: {}", e),
    }
    
    // Try to read messages (will simulate periodic messages)
    for i in 0..3 {
        std::thread::sleep(std::time::Duration::from_millis(100));
        match serial.read_message()? {
            Some(message) => {
                println!("Received message {}: {} bytes", i + 1, message.data.len());
            }
            None => println!("No message available"),
        }
    }
    
    println!("Serial transceiver demo completed\n");
    Ok(())
}

fn demo_i2c_transceiver() -> Result<(), Box<dyn std::error::Error>> {
    println!("--- I2C Transceiver Demo ---");
    
    let config = TransceiverConfig::i2c(3, 0x42);
    let mut i2c = I2CTransceiver::new(3, config)?;
    
    println!("Created I2C transceiver at address: 0x42");
    
    // Try to connect
    match i2c.connect() {
        Ok(()) => println!("I2C device connected"),
        Err(e) => println!("I2C connection failed: {}", e),
    }
    
    // Check status
    let status = i2c.get_status();
    println!("Status: connected={}, firmware={:?}", 
        status.connected, status.firmware_version);
    
    // Send a test command
    let test_cmd = vec![0xAA, 0xBB, 0xCC];
    match i2c.send_message(&test_cmd) {
        Ok(()) => println!("Test command sent via I2C"),
        Err(e) => println!("I2C send failed: {}", e),
    }
    
    // Try to read messages (will simulate periodic messages)
    for i in 0..3 {
        std::thread::sleep(std::time::Duration::from_millis(200));
        match i2c.read_message()? {
            Some(message) => {
                println!("I2C message {}: {} bytes, signal strength: {:?}", 
                    i + 1, message.data.len(), message.signal_strength);
            }
            None => println!("No I2C message available"),
        }
    }
    
    println!("I2C transceiver demo completed\n");
    Ok(())
}

fn demo_error_handling() -> Result<(), Box<dyn std::error::Error>> {
    println!("--- Error Handling Demo ---");
    
    // Create a mock transceiver for error testing
    let mut mock = MockTransceiver::new(4);
    
    // Test different error scenarios
    println!("Testing connection loss...");
    mock.disconnect();
    match mock.read_message() {
        Err(CommError::ConnectionLost { transceiver_id }) => {
            println!("✓ Connection lost error detected for transceiver {}", transceiver_id);
            println!("  Recovery strategy: {:?}", 
                CommError::ConnectionLost { transceiver_id }.recovery_strategy());
        }
        _ => println!("✗ Expected connection lost error"),
    }
    
    // Reconnect and test timeout
    mock.reconnect();
    mock.simulate_errors(true, 1.0); // 100% error rate
    
    println!("Testing timeout errors...");
    for _ in 0..3 {
        match mock.read_message() {
            Err(e) => {
                println!("✓ Error: {}", e);
                println!("  Recoverable: {}", e.is_recoverable());
                println!("  Strategy: {:?}", e.recovery_strategy());
                break;
            }
            _ => continue,
        }
    }
    
    // Test configuration errors
    println!("Testing configuration errors...");
    let mut bad_config = TransceiverConfig::i2c(5, 0xFF); // Invalid I2C address
    bad_config.max_message_size = 0; // Invalid size
    
    match bad_config.validate() {
        Err(e) => {
            println!("✓ Configuration error: {}", e);
            println!("  Recoverable: {}", e.is_recoverable());
        }
        Ok(()) => println!("✗ Expected configuration error"),
    }
    
    println!("Error handling demo completed\n");
    Ok(())
}