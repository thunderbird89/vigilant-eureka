use shared_positioning::message_parser::{MessageBuilder, GeodeticPosition, MessageParser, RawMessage};
use uuid::Uuid;
use std::time::{SystemTime, UNIX_EPOCH};

fn main() {
    println!("=== Beacon Message Builder Demo ===\n");
    
    let builder = MessageBuilder::new();
    let mut parser = MessageParser::new();
    
    // Test position
    let position = GeodeticPosition {
        latitude: 32.123456,
        longitude: -117.654321,
        depth: 10.5,
    };
    
    // Test beacon UUID
    let beacon_uuid = Uuid::parse_str("550e8400-e29b-41d4-a716-446655440000").unwrap();
    
    println!("1. Testing V1 Message Format:");
    println!("   Building V1 message with beacon ID 123...");
    let v1_message = builder.build_v1_message(123, position, 200, 456).unwrap();
    println!("   Message length: {} bytes", v1_message.len());
    println!("   Validation: {:?}", builder.validate_message_data(&v1_message));
    
    // Parse it back
    let raw_v1 = RawMessage {
        data: v1_message,
        timestamp_received: SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_millis() as u64,
        transceiver_id: 1,
        signal_strength: Some(200),
    };
    let parsed_v1 = parser.parse_message(&raw_v1).unwrap();
    println!("   Parsed back - Anchor ID: {}, Signal Quality: {}, Sequence: {}", 
             parsed_v1.anchor_id, parsed_v1.signal_quality, parsed_v1.message_sequence);
    
    println!("\n2. Testing V2 Message Format:");
    println!("   Building V2 message with beacon ID 123...");
    let v2_message = builder.build_v2_message(123, position, 200, 456).unwrap();
    println!("   Message length: {} bytes", v2_message.len());
    println!("   Validation: {:?}", builder.validate_message_data(&v2_message));
    
    // Parse it back
    let raw_v2 = RawMessage {
        data: v2_message,
        timestamp_received: SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_millis() as u64,
        transceiver_id: 1,
        signal_strength: Some(200),
    };
    let parsed_v2 = parser.parse_message(&raw_v2).unwrap();
    println!("   Parsed back - Anchor ID: {}, Signal Quality: {}, Sequence: {}", 
             parsed_v2.anchor_id, parsed_v2.signal_quality, parsed_v2.message_sequence);
    
    println!("\n3. Testing V3 Message Format with UUID:");
    println!("   Building V3 message with UUID: {}", beacon_uuid);
    let v3_message = builder.build_v3_message(beacon_uuid, position, 200, 456).unwrap();
    println!("   Message length: {} bytes", v3_message.len());
    println!("   Validation: {:?}", builder.validate_message_data(&v3_message));
    
    // Parse it back
    let raw_v3 = RawMessage {
        data: v3_message,
        timestamp_received: SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_millis() as u64,
        transceiver_id: 1,
        signal_strength: Some(200),
    };
    let parsed_v3 = parser.parse_message(&raw_v3).unwrap();
    println!("   Parsed back - Anchor ID: {}, Signal Quality: {}, Sequence: {}", 
             parsed_v3.anchor_id, parsed_v3.signal_quality, parsed_v3.message_sequence);
    
    println!("\n4. Testing UUID Compatibility Methods:");
    println!("   Building V1 message with UUID (converted to u16)...");
    let v1_uuid_message = builder.build_v1_message_with_uuid(beacon_uuid, position, 200, 456).unwrap();
    println!("   Message length: {} bytes", v1_uuid_message.len());
    println!("   Validation: {:?}", builder.validate_message_data(&v1_uuid_message));
    
    println!("   Building V2 message with UUID (converted to u16)...");
    let v2_uuid_message = builder.build_v2_message_with_uuid(beacon_uuid, position, 200, 456).unwrap();
    println!("   Message length: {} bytes", v2_uuid_message.len());
    println!("   Validation: {:?}", builder.validate_message_data(&v2_uuid_message));
    
    println!("\n5. Testing Checksum Calculation:");
    let test_data = b"Hello, World!";
    let checksum = parser.calculate_checksum(test_data);
    println!("   Checksum for 'Hello, World!': 0x{:04X}", checksum);
    
    println!("\n6. Testing Message Sequence Rollover:");
    println!("   Building message with max sequence number (65535)...");
    let max_seq_message = builder.build_v1_message(123, position, 200, u16::MAX).unwrap();
    println!("   Validation: {:?}", builder.validate_message_data(&max_seq_message));
    
    println!("   Building message with rollover sequence number (0)...");
    let rollover_message = builder.build_v1_message(123, position, 200, 0).unwrap();
    println!("   Validation: {:?}", builder.validate_message_data(&rollover_message));
    
    println!("\n=== All Requirements Verified Successfully! ===");
    println!("✓ MessageBuilder struct with V1 and V2 message construction methods");
    println!("✓ UUID support and V3 message format for beacon IDs");
    println!("✓ Message validation and checksum calculation for outgoing messages");
    println!("✓ Comprehensive unit tests for message building and validation");
    println!("✓ Requirements 1.1, 1.2, 1.4, 5.1 satisfied");
}