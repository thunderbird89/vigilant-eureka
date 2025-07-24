use shared_positioning::{
    MessageParser, MessageValidator, RawMessage, AnchorMessage, GeodeticPosition,
    MessageParseError
};

/// Demonstration of message parsing and validation system
pub fn message_parsing_demo() {
    println!("=== MESSAGE PARSING AND VALIDATION DEMO ===\n");

    // Initialize parser and validator
    let mut parser = MessageParser::new();
    let mut validator = MessageValidator::new();

    println!("1. TESTING VERSION 1 MESSAGE FORMAT");
    test_v1_message_parsing(&mut parser, &mut validator);

    println!("\n2. TESTING VERSION 2 MESSAGE FORMAT");
    test_v2_message_parsing(&mut parser, &mut validator);

    println!("\n3. TESTING ERROR HANDLING");
    test_error_handling(&mut parser);

    println!("\n4. TESTING MESSAGE VALIDATION");
    test_message_validation(&mut validator);

    println!("\n5. TESTING MULTIPLE MESSAGE FORMATS");
    test_multiple_formats(&mut parser, &mut validator);

    println!("\n6. PERFORMANCE AND STATISTICS");
    display_statistics(&parser, &validator);
}

/// Test Version 1 message parsing
fn test_v1_message_parsing(parser: &mut MessageParser, validator: &mut MessageValidator) {
    println!("  Creating V1 message with full precision...");
    
    // Create a realistic V1 message
    let mut data = Vec::new();
    data.push(1u8); // version
    data.extend_from_slice(&101u16.to_le_bytes()); // anchor_id
    let current_time = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_millis() as u64;
    data.extend_from_slice(&current_time.to_le_bytes()); // current timestamp
    data.extend_from_slice(&32.123456789f64.to_le_bytes()); // latitude (high precision)
    data.extend_from_slice(&(-117.654321098f64).to_le_bytes()); // longitude (high precision)
    data.extend_from_slice(&15.75f32.to_le_bytes()); // depth
    data.push(220u8); // signal_quality
    data.extend_from_slice(&1001u16.to_le_bytes()); // sequence
    
    // Calculate and append checksum
    let checksum = parser.calculate_checksum(&data);
    data.extend_from_slice(&checksum.to_le_bytes());
    
    let raw_message = RawMessage {
        data,
        timestamp_received: current_time,
        transceiver_id: 1,
        signal_strength: Some(220),
    };
    
    match parser.parse_message(&raw_message) {
        Ok(message) => {
            println!("  ✓ V1 message parsed successfully:");
            println!("    Anchor ID: {}", message.anchor_id);
            println!("    Timestamp: {} ms", message.timestamp_ms);
            println!("    Position: lat={:.9}, lon={:.9}, depth={:.2}m", 
                     message.position.latitude, message.position.longitude, message.position.depth);
            println!("    Signal Quality: {}", message.signal_quality);
            println!("    Sequence: {}", message.message_sequence);
            println!("    Checksum: 0x{:04X}", message.checksum);
            
            // Validate the message
            let validation = validator.validate_anchor_message(&message);
            println!("    Validation: {} (quality score: {:.3})", 
                     if validation.is_valid { "PASS" } else { "FAIL" }, validation.quality_score);
            
            if !validation.warnings.is_empty() {
                println!("    Warnings: {:?}", validation.warnings);
            }
        }
        Err(e) => {
            println!("  ✗ V1 message parsing failed: {}", e);
        }
    }
}

/// Test Version 2 message parsing
fn test_v2_message_parsing(parser: &mut MessageParser, validator: &mut MessageValidator) {
    println!("  Creating V2 message with compact format...");
    
    // Create a realistic V2 message (more compact)
    let mut data = Vec::new();
    data.push(2u8); // version
    data.extend_from_slice(&102u16.to_le_bytes()); // anchor_id
    let current_time = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_millis() as u64;
    let relative_timestamp = ((current_time - 1704067200000) / 1000) as u32; // relative seconds to 2024-01-01
    data.extend_from_slice(&relative_timestamp.to_le_bytes()); // relative timestamp
    data.extend_from_slice(&32123457i32.to_le_bytes()); // latitude * 1e6
    data.extend_from_slice(&(-117654321i32).to_le_bytes()); // longitude * 1e6
    data.extend_from_slice(&15750u16.to_le_bytes()); // depth in mm
    data.push(210u8); // signal_quality
    data.extend_from_slice(&1002u16.to_le_bytes()); // sequence
    data.push(0u8); // flags (reserved)
    
    // Calculate and append checksum
    let checksum = parser.calculate_checksum(&data);
    data.extend_from_slice(&checksum.to_le_bytes());
    
    let raw_message = RawMessage {
        data,
        timestamp_received: current_time,
        transceiver_id: 2,
        signal_strength: Some(210),
    };
    
    match parser.parse_message(&raw_message) {
        Ok(message) => {
            println!("  ✓ V2 message parsed successfully:");
            println!("    Anchor ID: {}", message.anchor_id);
            println!("    Timestamp: {} ms", message.timestamp_ms);
            println!("    Position: lat={:.6}, lon={:.6}, depth={:.3}m", 
                     message.position.latitude, message.position.longitude, message.position.depth);
            println!("    Signal Quality: {}", message.signal_quality);
            println!("    Sequence: {}", message.message_sequence);
            println!("    Checksum: 0x{:04X}", message.checksum);
            
            // Validate the message
            let validation = validator.validate_anchor_message(&message);
            println!("    Validation: {} (quality score: {:.3})", 
                     if validation.is_valid { "PASS" } else { "FAIL" }, validation.quality_score);
        }
        Err(e) => {
            println!("  ✗ V2 message parsing failed: {}", e);
        }
    }
}

/// Test error handling scenarios
fn test_error_handling(parser: &mut MessageParser) {
    println!("  Testing various error conditions...");
    
    // Test 1: Invalid message length
    let short_message = RawMessage {
        data: vec![1, 2, 3], // Too short
        timestamp_received: 1704067200000,
        transceiver_id: 1,
        signal_strength: Some(200),
    };
    
    match parser.parse_message(&short_message) {
        Err(MessageParseError::InvalidLength { expected, actual }) => {
            println!("  ✓ Correctly detected invalid length: expected {}, got {}", expected, actual);
        }
        _ => println!("  ✗ Failed to detect invalid length"),
    }
    
    // Test 2: Invalid version
    let invalid_version_data = vec![99u8; 36]; // Version 99 doesn't exist
    let invalid_version_message = RawMessage {
        data: invalid_version_data,
        timestamp_received: 1704067200000,
        transceiver_id: 1,
        signal_strength: Some(200),
    };
    
    match parser.parse_message(&invalid_version_message) {
        Err(MessageParseError::InvalidVersion { version }) => {
            println!("  ✓ Correctly detected invalid version: {}", version);
        }
        _ => println!("  ✗ Failed to detect invalid version"),
    }
    
    // Test 3: Checksum mismatch
    let mut bad_checksum_data = Vec::new();
    bad_checksum_data.push(1u8); // version
    bad_checksum_data.extend_from_slice(&103u16.to_le_bytes()); // anchor_id
    bad_checksum_data.extend_from_slice(&1704067200000u64.to_le_bytes()); // timestamp
    bad_checksum_data.extend_from_slice(&32.0f64.to_le_bytes()); // latitude
    bad_checksum_data.extend_from_slice(&(-117.0f64).to_le_bytes()); // longitude
    bad_checksum_data.extend_from_slice(&10.0f32.to_le_bytes()); // depth
    bad_checksum_data.push(200u8); // signal_quality
    bad_checksum_data.extend_from_slice(&1003u16.to_le_bytes()); // sequence
    bad_checksum_data.extend_from_slice(&0xDEADu16.to_le_bytes()); // Wrong checksum
    
    let bad_checksum_message = RawMessage {
        data: bad_checksum_data,
        timestamp_received: 1704067200000,
        transceiver_id: 1,
        signal_strength: Some(200),
    };
    
    match parser.parse_message(&bad_checksum_message) {
        Err(MessageParseError::ChecksumMismatch { expected, calculated }) => {
            println!("  ✓ Correctly detected checksum mismatch: expected 0x{:04X}, calculated 0x{:04X}", 
                     expected, calculated);
        }
        _ => println!("  ✗ Failed to detect checksum mismatch"),
    }
}

/// Test message validation features
fn test_message_validation(validator: &mut MessageValidator) {
    println!("  Testing message validation features...");
    
    // Test 1: Valid message
    let current_time = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_millis() as u64;
    
    let valid_message = AnchorMessage {
        anchor_id: 104,
        timestamp_ms: current_time - 1000, // Recent timestamp
        position: GeodeticPosition {
            latitude: 32.123456,
            longitude: -117.654321,
            depth: 20.5,
        },
        signal_quality: 180,
        message_sequence: 1004,
        message_version: 1,
        checksum: 0x1234,
    };
    
    let result = validator.validate_anchor_message(&valid_message);
    println!("  ✓ Valid message validation: {} (score: {:.3})", 
             if result.is_valid { "PASS" } else { "FAIL" }, result.quality_score);
    
    // Test 2: Low signal quality
    let low_quality_message = AnchorMessage {
        anchor_id: 105,
        timestamp_ms: current_time - 2000,
        position: GeodeticPosition {
            latitude: 32.123456,
            longitude: -117.654321,
            depth: 20.5,
        },
        signal_quality: 30, // Below threshold
        message_sequence: 1005,
        message_version: 1,
        checksum: 0x1235,
    };
    
    let result = validator.validate_anchor_message(&low_quality_message);
    println!("  ✓ Low quality message validation: {} (score: {:.3})", 
             if result.is_valid { "PASS" } else { "FAIL" }, result.quality_score);
    if !result.warnings.is_empty() {
        println!("    Warnings: {:?}", result.warnings);
    }
    
    // Test 3: Duplicate sequence number
    let duplicate_message = AnchorMessage {
        anchor_id: 104, // Same anchor as first test
        timestamp_ms: current_time - 3000,
        position: GeodeticPosition {
            latitude: 32.123456,
            longitude: -117.654321,
            depth: 20.5,
        },
        signal_quality: 180,
        message_sequence: 1004, // Same sequence as first test
        message_version: 1,
        checksum: 0x1236,
    };
    
    let result = validator.validate_anchor_message(&duplicate_message);
    println!("  ✓ Duplicate sequence validation: {} (score: {:.3})", 
             if result.is_valid { "PASS" } else { "FAIL" }, result.quality_score);
    if !result.errors.is_empty() {
        println!("    Errors: {:?}", result.errors);
    }
}

/// Test multiple message formats in sequence
fn test_multiple_formats(parser: &mut MessageParser, validator: &mut MessageValidator) {
    println!("  Processing mixed V1 and V2 messages...");
    
    let current_time = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_millis() as u64;
    
    let test_messages = vec![
        create_test_v1_message(201, current_time - 1000, 32.1, -117.1, 5.0, 200, 2001),
        create_test_v2_message(202, ((current_time - 1704067200000) / 1000) as u32, 32.2, -117.2, 6.0, 210, 2002),
        create_test_v1_message(203, current_time - 2000, 32.3, -117.3, 7.0, 190, 2003),
        create_test_v2_message(204, ((current_time - 1704067200000) / 1000) as u32 + 1, 32.4, -117.4, 8.0, 220, 2004),
    ];
    
    let mut successful_parses = 0;
    let mut successful_validations = 0;
    
    for (i, raw_message) in test_messages.iter().enumerate() {
        match parser.parse_message(raw_message) {
            Ok(message) => {
                successful_parses += 1;
                println!("    Message {}: V{} parsed successfully (anchor {})", 
                         i + 1, message.message_version, message.anchor_id);
                
                let validation = validator.validate_anchor_message(&message);
                if validation.is_valid {
                    successful_validations += 1;
                }
                println!("      Validation: {} (score: {:.3})", 
                         if validation.is_valid { "PASS" } else { "FAIL" }, validation.quality_score);
            }
            Err(e) => {
                println!("    Message {}: Parse failed - {}", i + 1, e);
            }
        }
    }
    
    println!("  Summary: {}/{} parsed, {}/{} validated", 
             successful_parses, test_messages.len(),
             successful_validations, test_messages.len());
}

/// Display comprehensive statistics
fn display_statistics(parser: &MessageParser, validator: &MessageValidator) {
    println!("  Parser Statistics:");
    let parser_stats = parser.get_stats();
    println!("    Total processed: {}", parser_stats.total_messages_processed);
    println!("    Successful: {}", parser_stats.successful_parses);
    println!("    Errors: {}", parser_stats.parse_errors);
    println!("    Checksum failures: {}", parser_stats.checksum_failures);
    
    if parser_stats.total_messages_processed > 0 {
        let success_rate = (parser_stats.successful_parses as f64 / parser_stats.total_messages_processed as f64) * 100.0;
        println!("    Success rate: {:.1}%", success_rate);
    }
    
    println!("\n  Validator Statistics:");
    let validator_stats = validator.get_stats();
    println!("    Total validations: {}", validator_stats.total_validations);
    println!("    Passed: {}", validator_stats.passed_validations);
    println!("    Failed: {}", validator_stats.failed_validations);
    println!("    Duplicates detected: {}", validator_stats.duplicate_messages);
    println!("    Out-of-order detected: {}", validator_stats.out_of_order_messages);
    
    if validator_stats.total_validations > 0 {
        let validation_rate = (validator_stats.passed_validations as f64 / validator_stats.total_validations as f64) * 100.0;
        println!("    Validation rate: {:.1}%", validation_rate);
    }
    
    // Generate detailed reports
    println!("\n  Diagnostic reports would be generated here...");
    println!("    Parser statistics available through get_statistics()");
    println!("    Validator statistics available through validation results");
    
    // Show a sample of available information
    println!("\n  Available parser information:");
    println!("    Parser can process V1, V2, and V3 message formats");
    println!("    Validator can check message integrity and format compliance");
}

/// Helper function to create test V1 message
fn create_test_v1_message(
    anchor_id: u16,
    timestamp: u64,
    lat: f64,
    lon: f64,
    depth: f64,
    quality: u8,
    sequence: u16,
) -> RawMessage {
    let mut data = Vec::new();
    data.push(1u8); // version
    data.extend_from_slice(&anchor_id.to_le_bytes());
    data.extend_from_slice(&timestamp.to_le_bytes());
    data.extend_from_slice(&lat.to_le_bytes());
    data.extend_from_slice(&lon.to_le_bytes());
    data.extend_from_slice(&(depth as f32).to_le_bytes());
    data.push(quality);
    data.extend_from_slice(&sequence.to_le_bytes());
    
    // Calculate checksum
    let temp_parser = MessageParser::new();
    let checksum = temp_parser.calculate_checksum(&data);
    data.extend_from_slice(&checksum.to_le_bytes());
    
    RawMessage {
        data,
        timestamp_received: timestamp,
        transceiver_id: 1,
        signal_strength: Some(quality),
    }
}

/// Helper function to create test V2 message
fn create_test_v2_message(
    anchor_id: u16,
    relative_timestamp: u32,
    lat: f64,
    lon: f64,
    depth: f64,
    quality: u8,
    sequence: u16,
) -> RawMessage {
    let mut data = Vec::new();
    data.push(2u8); // version
    data.extend_from_slice(&anchor_id.to_le_bytes());
    data.extend_from_slice(&relative_timestamp.to_le_bytes());
    data.extend_from_slice(&((lat * 1_000_000.0) as i32).to_le_bytes());
    data.extend_from_slice(&((lon * 1_000_000.0) as i32).to_le_bytes());
    data.extend_from_slice(&((depth * 1000.0) as u16).to_le_bytes());
    data.push(quality);
    data.extend_from_slice(&sequence.to_le_bytes());
    data.push(0u8); // flags
    
    // Calculate checksum
    let temp_parser = MessageParser::new();
    let checksum = temp_parser.calculate_checksum(&data);
    data.extend_from_slice(&checksum.to_le_bytes());
    
    RawMessage {
        data,
        timestamp_received: 1704067200000 + relative_timestamp as u64,
        transceiver_id: 2,
        signal_strength: Some(quality),
    }
}

/// Demonstrate timestamp precision handling
pub fn timestamp_precision_demo() {
    println!("\n=== TIMESTAMP PRECISION HANDLING DEMO ===\n");
    
    let mut parser = MessageParser::new();
    let mut validator = MessageValidator::new();
    
    println!("Testing millisecond precision timestamp handling...");
    
    let current_time = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_millis() as u64;
    
    // Test different timestamp precisions
    let test_timestamps = vec![
        current_time - 3000,  // 3 seconds ago, precise to seconds
        current_time - 2123,  // Precise to milliseconds
        current_time - 1456,  // Different millisecond value
    ];
    
    for (i, &timestamp) in test_timestamps.iter().enumerate() {
        let raw_message = create_test_v1_message(
            300 + i as u16,
            timestamp,
            32.0 + i as f64 * 0.001,
            -117.0 - i as f64 * 0.001,
            10.0 + i as f64,
            200,
            3000 + i as u16,
        );
        
        match parser.parse_message(&raw_message) {
            Ok(message) => {
                println!("  Message {}: timestamp {} ms", i + 1, message.timestamp_ms);
                
                let validation = validator.validate_anchor_message(&message);
                println!("    Precision validation: {} (score: {:.3})", 
                         if validation.is_valid { "PASS" } else { "FAIL" }, validation.quality_score);
                
                if !validation.warnings.is_empty() {
                    println!("    Warnings: {:?}", validation.warnings);
                }
            }
            Err(e) => {
                println!("  Message {}: Failed to parse - {}", i + 1, e);
            }
        }
    }
    
    // Test timestamp age validation
    println!("\nTesting timestamp age validation...");
    
    let current_time = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_millis() as u64;
    
    let old_timestamp = current_time - 70000; // 70 seconds ago (should be rejected)
    let future_timestamp = current_time + 5000; // 5 seconds in future (should be rejected)
    let valid_timestamp = current_time - 1000; // 1 second ago (should be accepted)
    
    let test_cases = vec![
        ("Old timestamp", old_timestamp),
        ("Future timestamp", future_timestamp),
        ("Valid timestamp", valid_timestamp),
    ];
    
    for (description, timestamp) in test_cases {
        let raw_message = create_test_v1_message(400, timestamp, 32.0, -117.0, 10.0, 200, 4000);
        
        match parser.parse_message(&raw_message) {
            Ok(message) => {
                println!("  {}: Parsed successfully", description);
                let validation = validator.validate_anchor_message(&message);
                println!("    Age validation: {}", if validation.is_valid { "PASS" } else { "FAIL" });
            }
            Err(e) => {
                println!("  {}: Parse failed - {}", description, e);
            }
        }
    }
}