use crate::virtual_transceiver::{VirtualTransceiver, VirtualTransceiverFactory, VirtualMessage};
use shared_positioning::{TransceiverInterface, MessageParser, GeodeticPosition};
use crate::{Anchor, Position, trilaterate};
use std::time::{SystemTime, UNIX_EPOCH};
use uuid::Uuid;

/// Demonstration of virtual receiver functionality
pub async fn virtual_receiver_demo() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== VIRTUAL RECEIVER DEMO ===\n");
    
    // Create virtual transceiver
    let mut transceiver = VirtualTransceiverFactory::create_and_connect(1, "demo_channel".to_string()).await?;
    
    println!("Virtual transceiver connected to channel 'demo_channel'");
    println!("Status: {:?}", transceiver.get_status());
    
    // Create some test virtual messages (simulating beacon transmissions)
    let test_messages = create_test_virtual_messages();
    
    println!("\nSimulating beacon transmissions...");
    
    // In a real implementation, these messages would come from the beacon emulator
    // For demo purposes, we'll simulate receiving them
    let mut message_parser = MessageParser::new();
    let mut anchor_messages: Vec<Anchor> = Vec::new();
    
    for (i, virtual_msg) in test_messages.iter().enumerate() {
        println!("Simulated beacon {} transmission:", i + 1);
        println!("  Beacon ID: {}", virtual_msg.beacon_id);
        println!("  Position: lat={:.6}, lon={:.6}, depth={:.2}m", 
                 virtual_msg.position.latitude, virtual_msg.position.longitude, virtual_msg.position.depth);
        println!("  Signal quality: {}", virtual_msg.signal_quality);
        
        // Convert virtual message to anchor format
        let anchor = Anchor {
            id: virtual_msg.beacon_id.to_string(),
            timestamp: virtual_msg.timestamp
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_millis() as u64,
            position: Position {
                lat: virtual_msg.position.latitude,
                lon: virtual_msg.position.longitude,
                depth: virtual_msg.position.depth,
            },
        };
        
        anchor_messages.push(anchor);
    }
    
    // Attempt trilateration with simulated data
    println!("\nAttempting positioning with {} anchors...", anchor_messages.len());
    
    let current_time = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_millis() as u64;
    
    match trilaterate(&anchor_messages, current_time) {
        Ok((geodetic_pos, local_pos)) => {
            println!("\n=== POSITIONING SUCCESSFUL ===");
            println!("Local position (ENU): east={:.2}m, north={:.2}m, down={:.2}m", 
                     local_pos.x, local_pos.y, local_pos.z);
            println!("Geodetic position: lat={:.6}°, lon={:.6}°, depth={:.2}m", 
                     geodetic_pos.lat, geodetic_pos.lon, geodetic_pos.depth);
        }
        Err(e) => {
            println!("Positioning failed: {}", e);
        }
    }
    
    // Test transceiver commands
    println!("\n=== TESTING VIRTUAL TRANSCEIVER COMMANDS ===");
    
    let status_response = transceiver.send_command(&[0x02])?;
    println!("Status command response: {}", String::from_utf8_lossy(&status_response));
    
    let version_response = transceiver.send_command(&[0x03])?;
    println!("Version command response: {}", String::from_utf8_lossy(&version_response));
    
    let channel_response = transceiver.send_command(&[0x05])?;
    println!("Channel command response: {}", String::from_utf8_lossy(&channel_response));
    
    println!("\nVirtual receiver demo completed successfully!");
    
    Ok(())
}

/// Create test virtual messages simulating beacon transmissions
fn create_test_virtual_messages() -> Vec<VirtualMessage> {
    let base_time = SystemTime::now();
    
    vec![
        VirtualMessage {
            beacon_id: Uuid::new_v4(),
            timestamp: base_time,
            position: GeodeticPosition {
                latitude: 32.12345,
                longitude: 45.47675,
                depth: 0.0,
            },
            message_data: create_test_beacon_message_data(1, 32.12345, 45.47675, 0.0),
            signal_quality: 255,
        },
        VirtualMessage {
            beacon_id: Uuid::new_v4(),
            timestamp: base_time,
            position: GeodeticPosition {
                latitude: 32.12365,
                longitude: 45.47695,
                depth: 5.0,
            },
            message_data: create_test_beacon_message_data(2, 32.12365, 45.47695, 5.0),
            signal_quality: 250,
        },
        VirtualMessage {
            beacon_id: Uuid::new_v4(),
            timestamp: base_time,
            position: GeodeticPosition {
                latitude: 32.12365,
                longitude: 45.47655,
                depth: 10.0,
            },
            message_data: create_test_beacon_message_data(3, 32.12365, 45.47655, 10.0),
            signal_quality: 245,
        },
        VirtualMessage {
            beacon_id: Uuid::new_v4(),
            timestamp: base_time,
            position: GeodeticPosition {
                latitude: 32.12385,
                longitude: 45.47675,
                depth: 15.0,
            },
            message_data: create_test_beacon_message_data(4, 32.12385, 45.47675, 15.0),
            signal_quality: 240,
        },
    ]
}

/// Create test beacon message data in the format expected by the message parser
fn create_test_beacon_message_data(beacon_id: u16, lat: f64, lon: f64, depth: f64) -> Vec<u8> {
    let mut data = Vec::new();
    
    // Version 3 message format (simplified)
    data.push(3u8); // version
    data.extend_from_slice(&beacon_id.to_le_bytes()); // beacon_id
    
    // Current timestamp
    let timestamp = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_millis() as u64;
    data.extend_from_slice(&timestamp.to_le_bytes()); // timestamp
    
    data.extend_from_slice(&lat.to_le_bytes()); // latitude
    data.extend_from_slice(&lon.to_le_bytes()); // longitude
    data.extend_from_slice(&(depth as f32).to_le_bytes()); // depth
    data.push(255u8); // signal_quality
    data.extend_from_slice(&1u16.to_le_bytes()); // sequence
    
    // Calculate simple checksum
    let mut checksum: u16 = 0;
    for &byte in &data {
        checksum = checksum.wrapping_add(byte as u16);
    }
    data.extend_from_slice(&checksum.to_le_bytes()); // checksum
    
    data
}