// Demonstration and testing of transceiver interface implementations
use crate::transceiver_interface::{
    TransceiverInterface, MockTransceiver, SerialTransceiver, I2CTransceiver,
    TransceiverConfig, PowerMode, CommError, RecoveryStrategy, ErrorRecoveryManager
};
use crate::message_parser::MessageParser;


/// Comprehensive demonstration of transceiver interface functionality
pub fn transceiver_interface_demo() {
    println!("=== TRANSCEIVER INTERFACE DEMONSTRATION ===\n");
    
    // Test mock transceiver
    test_mock_transceiver();
    
    // Test serial transceiver
    test_serial_transceiver();
    
    // Test I2C transceiver
    test_i2c_transceiver();
    
    // Test error recovery
    test_error_recovery();
    
    // Test multi-transceiver scenario
    test_multi_transceiver_setup();
}

/// Test mock transceiver implementation
fn test_mock_transceiver() {
    println!("1. MOCK TRANSCEIVER TEST");
    println!("   Testing mock transceiver for development and testing...\n");
    
    let mut mock = MockTransceiver::new(1);
    
    // Configure transceiver
    let config = TransceiverConfig {
        baud_rate: 9600,
        timeout_ms: 500,
        buffer_size: 1024,
        retry_attempts: 3,
        enable_flow_control: false,
        enable_error_correction: true,
        power_mode: PowerMode::Normal,
        frequency_channel: Some(1),
    };
    
    match mock.configure(config) {
        Ok(()) => println!("   ✓ Mock transceiver configured successfully"),
        Err(e) => println!("   ✗ Configuration failed: {}", e),
    }
    
    // Add test messages
    let test_message1 = MockTransceiver::create_test_anchor_message(1, 32.123, -117.456, 10.0);
    let test_message2 = MockTransceiver::create_test_anchor_message(2, 32.124, -117.457, 12.0);
    let test_message3 = MockTransceiver::create_test_anchor_message(3, 32.125, -117.458, 8.0);
    
    mock.add_test_message(test_message1);
    mock.add_test_message(test_message2);
    mock.add_test_message(test_message3);
    
    println!("   Added 3 test anchor messages");
    
    // Test message reading
    let mut message_count = 0;
    let mut parser = MessageParser::new();
    
    for i in 0..5 {
        match mock.read_message() {
            Ok(Some(raw_msg)) => {
                message_count += 1;
                println!("   Message {}: {} bytes from transceiver {}, signal: {:?}", 
                         i + 1, raw_msg.data.len(), raw_msg.transceiver_id, raw_msg.signal_strength);
                
                // Try to parse the message
                match parser.parse_message(&raw_msg) {
                    Ok(anchor_msg) => {
                        println!("     Parsed: Anchor {} at ({:.6}, {:.6}, {:.1}m)", 
                                 anchor_msg.anchor_id, 
                                 anchor_msg.position.latitude,
                                 anchor_msg.position.longitude,
                                 anchor_msg.position.depth);
                    }
                    Err(e) => println!("     Parse error: {}", e),
                }
            }
            Ok(None) => println!("   No message available (attempt {})", i + 1),
            Err(e) => println!("   Read error: {}", e),
        }
    }
    
    // Test status reporting
    let status = mock.get_status();
    println!("   Status: connected={}, signal={:?}, battery={:?}", 
             status.is_connected, status.signal_strength, status.battery_level);
    
    // Test error simulation
    mock.enable_error_simulation(0.3); // 30% error rate
    println!("   Enabled error simulation (30% error rate)");
    
    let mut error_count = 0;
    for i in 0..10 {
        match mock.read_message() {
            Ok(_) => {},
            Err(_) => error_count += 1,
        }
    }
    println!("   Error simulation: {}/10 operations failed", error_count);
    
    // Generate diagnostic report
    println!("   Diagnostic Report:");
    let report = mock.get_diagnostic_report();
    for line in report.lines().take(10) {
        println!("     {}", line);
    }
    
    println!("   ✓ Mock transceiver test completed\n");
}

/// Test serial transceiver implementation
fn test_serial_transceiver() {
    println!("2. SERIAL TRANSCEIVER TEST");
    println!("   Testing serial/UART communication interface...\n");
    
    let mut serial = SerialTransceiver::new(2, "/dev/ttyUSB0".to_string());
    
    // Configure for typical JANUS transceiver
    let config = TransceiverConfig {
        baud_rate: 19200,
        timeout_ms: 1000,
        buffer_size: 512,
        retry_attempts: 5,
        enable_flow_control: true,
        enable_error_correction: true,
        power_mode: PowerMode::Normal,
        frequency_channel: Some(2),
    };
    
    match serial.configure(config) {
        Ok(()) => println!("   ✓ Serial transceiver configured (19200 baud)"),
        Err(e) => println!("   ✗ Configuration failed: {}", e),
    }
    
    // Test connection
    match serial.open() {
        Ok(()) => println!("   ✓ Serial port opened successfully"),
        Err(e) => println!("   ✗ Failed to open serial port: {}", e),
    }
    
    // Test command sending
    let test_commands = [
        (b"\x01".as_slice(), "Ping command"),
        (b"\x02".as_slice(), "Status query"),
        (b"\x03".as_slice(), "Version query"),
        (b"\x04".as_slice(), "Signal strength"),
    ];
    
    for (command, description) in &test_commands {
        match serial.send_command(command) {
            Ok(response) => {
                let response_str = String::from_utf8_lossy(&response);
                println!("   Command '{}': Response = '{}'", description, response_str.trim());
            }
            Err(e) => println!("   Command '{}': Error = {}", description, e),
        }
    }
    
    // Test power mode changes
    let power_modes = [PowerMode::Normal, PowerMode::PowerSave, PowerMode::Sleep];
    for mode in &power_modes {
        match serial.set_power_mode(*mode) {
            Ok(()) => println!("   ✓ Power mode set to {:?}", mode),
            Err(e) => println!("   ✗ Failed to set power mode {:?}: {}", mode, e),
        }
    }
    
    // Test message reading (will be empty in simulation)
    for i in 0..3 {
        match serial.read_message() {
            Ok(Some(msg)) => println!("   Received message: {} bytes", msg.data.len()),
            Ok(None) => println!("   No message available (attempt {})", i + 1),
            Err(e) => println!("   Read error: {}", e),
        }
    }
    
    // Test reset functionality
    match serial.reset() {
        Ok(()) => println!("   ✓ Transceiver reset successfully"),
        Err(e) => println!("   ✗ Reset failed: {}", e),
    }
    
    let status = serial.get_status();
    println!("   Final status: connected={}, firmware={:?}", 
             status.is_connected, status.firmware_version);
    
    println!("   ✓ Serial transceiver test completed\n");
}

/// Test I2C transceiver implementation
fn test_i2c_transceiver() {
    println!("3. I2C TRANSCEIVER TEST");
    println!("   Testing I2C communication interface...\n");
    
    let mut i2c = I2CTransceiver::new(3, 0x42, 1); // Device address 0x42, bus 1
    
    // Initialize I2C connection
    match i2c.initialize() {
        Ok(()) => println!("   ✓ I2C transceiver initialized (address 0x42, bus 1)"),
        Err(e) => println!("   ✗ I2C initialization failed: {}", e),
    }
    
    // Configure I2C transceiver
    let config = TransceiverConfig {
        baud_rate: 0, // Not used for I2C
        timeout_ms: 800,
        buffer_size: 256,
        retry_attempts: 4,
        enable_flow_control: false,
        enable_error_correction: true,
        power_mode: PowerMode::PowerSave,
        frequency_channel: Some(3),
    };
    
    match i2c.configure(config) {
        Ok(()) => println!("   ✓ I2C transceiver configured"),
        Err(e) => println!("   ✗ Configuration failed: {}", e),
    }
    
    // Test I2C command interface
    let i2c_commands = [
        (vec![0x01], "Device status"),
        (vec![0x02, 0x10], "Read register 0x10"),
        (vec![0x03], "Get version"),
        (vec![0x04, 0x05, 0xAA], "Write 0xAA to register 0x05"),
    ];
    
    for (command, description) in &i2c_commands {
        match i2c.send_command(command) {
            Ok(response) => {
                println!("   I2C command '{}': Response = {:02X?}", description, response);
            }
            Err(e) => println!("   I2C command '{}': Error = {}", description, e),
        }
    }
    
    // Test message reading
    for i in 0..3 {
        match i2c.read_message() {
            Ok(Some(msg)) => {
                println!("   I2C message received: {} bytes, signal: {:?}", 
                         msg.data.len(), msg.signal_strength);
            }
            Ok(None) => println!("   No I2C message available (attempt {})", i + 1),
            Err(e) => println!("   I2C read error: {}", e),
        }
    }
    
    // Test power management
    let power_modes = [PowerMode::Normal, PowerMode::PowerSave, PowerMode::Emergency];
    for mode in &power_modes {
        match i2c.set_power_mode(*mode) {
            Ok(()) => println!("   ✓ I2C power mode set to {:?}", mode),
            Err(e) => println!("   ✗ Failed to set I2C power mode {:?}: {}", mode, e),
        }
    }
    
    let status = i2c.get_status();
    println!("   I2C status: connected={}, hardware_id={:?}", 
             status.is_connected, status.hardware_id);
    
    println!("   ✓ I2C transceiver test completed\n");
}

/// Test error recovery mechanisms
fn test_error_recovery() {
    println!("4. ERROR RECOVERY TEST");
    println!("   Testing error detection and recovery strategies...\n");
    
    let mut recovery_manager = ErrorRecoveryManager::new();
    
    // Test different error types and recovery strategies
    let test_errors = [
        CommError::Timeout { timeout_ms: 1000 },
        CommError::ConnectionFailed { details: "Port disconnected".to_string() },
        CommError::HardwareError { error_code: 0x1234, details: "Sensor fault".to_string() },
        CommError::IntegrityError { details: "Checksum mismatch".to_string() },
        CommError::BufferOverflow { buffer_size: 1024 },
    ];
    
    for error in &test_errors {
        println!("   Testing error: {}", error);
        
        match recovery_manager.handle_error(error) {
            Some(RecoveryStrategy::Retry { max_attempts, delay_ms }) => {
                println!("     → Recovery: Retry (max {} attempts, {}ms delay)", max_attempts, delay_ms);
            }
            Some(RecoveryStrategy::Reset) => {
                println!("     → Recovery: Reset transceiver");
            }
            Some(RecoveryStrategy::Failover { backup_id }) => {
                println!("     → Recovery: Failover to backup transceiver {}", backup_id);
            }
            Some(RecoveryStrategy::Ignore) => {
                println!("     → Recovery: Ignore error and continue");
            }
            Some(RecoveryStrategy::Shutdown) => {
                println!("     → Recovery: Shutdown system (critical error)");
            }
            None => {
                println!("     → No recovery strategy available");
            }
        }
    }
    
    // Test repeated error handling
    println!("   Testing repeated timeout errors:");
    for i in 1..=5 {
        let error = CommError::Timeout { timeout_ms: 1000 };
        match recovery_manager.handle_error(&error) {
            Some(strategy) => println!("     Attempt {}: {:?}", i, strategy),
            None => println!("     Attempt {}: No more recovery attempts", i),
        }
    }
    
    println!("   ✓ Error recovery test completed\n");
}

/// Test multi-transceiver setup with failover
fn test_multi_transceiver_setup() {
    println!("5. MULTI-TRANSCEIVER SETUP TEST");
    println!("   Testing multiple transceiver coordination...\n");
    
    // Create multiple transceivers
    let mut transceivers: Vec<Box<dyn TransceiverInterface>> = vec![
        Box::new(MockTransceiver::new(1)),
        Box::new(SerialTransceiver::new(2, "/dev/ttyUSB0".to_string())),
        Box::new(I2CTransceiver::new(3, 0x42, 1)),
    ];
    
    // Configure all transceivers
    let config = TransceiverConfig::default();
    for (i, transceiver) in transceivers.iter_mut().enumerate() {
        match transceiver.configure(config.clone()) {
            Ok(()) => println!("   ✓ Transceiver {} configured", i + 1),
            Err(e) => println!("   ✗ Transceiver {} configuration failed: {}", i + 1, e),
        }
    }
    
    // Initialize specific transceivers
    // Note: In a real implementation, we would need proper trait object downcasting
    // For now, we'll skip the mock-specific initialization in the multi-transceiver test
    
    // Test message reading from all transceivers
    println!("   Reading messages from all transceivers:");
    for (i, transceiver) in transceivers.iter_mut().enumerate() {
        match transceiver.read_message() {
            Ok(Some(msg)) => {
                println!("     Transceiver {}: Received {} bytes, signal: {:?}", 
                         i + 1, msg.data.len(), msg.signal_strength);
            }
            Ok(None) => {
                println!("     Transceiver {}: No message available", i + 1);
            }
            Err(e) => {
                println!("     Transceiver {}: Error - {}", i + 1, e);
            }
        }
    }
    
    // Test status reporting
    println!("   Transceiver status summary:");
    for (i, transceiver) in transceivers.iter().enumerate() {
        let status = transceiver.get_status();
        println!("     Transceiver {}: connected={}, signal={:?}, firmware={:?}", 
                 i + 1, status.is_connected, status.signal_strength, status.firmware_version);
    }
    
    // Test coordinated power management
    println!("   Setting all transceivers to power save mode:");
    for (i, transceiver) in transceivers.iter_mut().enumerate() {
        match transceiver.set_power_mode(PowerMode::PowerSave) {
            Ok(()) => println!("     ✓ Transceiver {} power mode updated", i + 1),
            Err(e) => println!("     ✗ Transceiver {} power mode failed: {}", i + 1, e),
        }
    }
    
    println!("   ✓ Multi-transceiver setup test completed\n");
}

/// Performance test for transceiver interfaces
pub fn transceiver_performance_test() {
    println!("=== TRANSCEIVER PERFORMANCE TEST ===\n");
    
    let mut mock = MockTransceiver::new(1);
    mock.configure(TransceiverConfig::default()).unwrap();
    
    // Add many test messages
    for i in 0..100 {
        let msg = MockTransceiver::create_test_anchor_message(
            (i % 4) + 1, 
            32.123 + (i as f64 * 0.001), 
            -117.456 + (i as f64 * 0.001), 
            10.0 + (i as f64 * 0.1)
        );
        mock.add_test_message(msg);
    }
    
    // Measure message processing performance
    let start_time = std::time::Instant::now();
    let mut processed_count = 0;
    let mut error_count = 0;
    let mut parser = MessageParser::new();
    
    for _ in 0..100 {
        match mock.read_message() {
            Ok(Some(raw_msg)) => {
                match parser.parse_message(&raw_msg) {
                    Ok(_) => processed_count += 1,
                    Err(_) => error_count += 1,
                }
            }
            Ok(None) => break,
            Err(_) => error_count += 1,
        }
    }
    
    let elapsed = start_time.elapsed();
    let messages_per_second = processed_count as f64 / elapsed.as_secs_f64();
    
    println!("Performance Results:");
    println!("  Messages processed: {}", processed_count);
    println!("  Errors encountered: {}", error_count);
    println!("  Total time: {:.2} ms", elapsed.as_millis());
    println!("  Processing rate: {:.1} messages/second", messages_per_second);
    println!("  Average time per message: {:.2} ms", elapsed.as_millis() as f64 / processed_count as f64);
    
    // Memory usage estimation
    let mock_size = std::mem::size_of_val(&mock);
    let parser_size = std::mem::size_of_val(&parser);
    println!("  Memory usage: Mock={} bytes, Parser={} bytes", mock_size, parser_size);
    
    println!("  ✓ Performance test completed\n");
}

