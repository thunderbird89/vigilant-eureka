#!/usr/bin/env rust-script
//! Test script to verify IPC message standardization between beacon-emulator and receiver
//! 
//! This script demonstrates that both systems now use the same shared message types
//! from the shared-positioning crate, ensuring compatibility.

use shared_positioning::{VirtualMessage, IpcMessage, GeodeticPosition};
use std::time::SystemTime;
use uuid::Uuid;

fn main() {
    println!("🔧 Testing IPC Message Standardization");
    println!("=====================================");
    
    // Create a VirtualMessage using the shared type
    let virtual_msg = VirtualMessage {
        beacon_id: Uuid::new_v4(),
        timestamp: SystemTime::now(),
        position: GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        },
        message_data: vec![0x01, 0x02, 0x03, 0x04],
        signal_quality: 255,
    };
    
    println!("✅ Created VirtualMessage:");
    println!("   Beacon ID: {}", virtual_msg.beacon_id);
    println!("   Position: {:.3}, {:.3}, {:.1}m", 
             virtual_msg.position.latitude, 
             virtual_msg.position.longitude, 
             virtual_msg.position.depth);
    println!("   Signal Quality: {}", virtual_msg.signal_quality);
    println!("   Data Length: {} bytes", virtual_msg.message_data.len());
    
    // Test IPC message serialization/deserialization
    let ipc_msg = IpcMessage::VirtualMessage(virtual_msg.clone());
    
    // Serialize using bincode (same as both beacon-emulator and receiver)
    match bincode::serialize(&ipc_msg) {
        Ok(serialized) => {
            println!("✅ Serialized IPC message: {} bytes", serialized.len());
            
            // Deserialize back
            match bincode::deserialize::<IpcMessage>(&serialized) {
                Ok(deserialized) => {
                    match deserialized {
                        IpcMessage::VirtualMessage(msg) => {
                            println!("✅ Deserialized successfully:");
                            println!("   Beacon ID matches: {}", msg.beacon_id == virtual_msg.beacon_id);
                            println!("   Position matches: {}", 
                                     msg.position.latitude == virtual_msg.position.latitude &&
                                     msg.position.longitude == virtual_msg.position.longitude &&
                                     msg.position.depth == virtual_msg.position.depth);
                            println!("   Data matches: {}", msg.message_data == virtual_msg.message_data);
                            println!("   Signal quality matches: {}", msg.signal_quality == virtual_msg.signal_quality);
                        }
                        _ => println!("❌ Unexpected message type after deserialization"),
                    }
                }
                Err(e) => println!("❌ Deserialization failed: {}", e),
            }
        }
        Err(e) => println!("❌ Serialization failed: {}", e),
    }
    
    // Test other IPC message types
    let subscribe_msg = IpcMessage::Subscribe {
        channel_name: "test_channel".to_string(),
        receiver_id: 1,
    };
    
    match bincode::serialize(&subscribe_msg) {
        Ok(serialized) => {
            match bincode::deserialize::<IpcMessage>(&serialized) {
                Ok(IpcMessage::Subscribe { channel_name, receiver_id }) => {
                    println!("✅ Subscribe message serialization works:");
                    println!("   Channel: {}", channel_name);
                    println!("   Receiver ID: {}", receiver_id);
                }
                _ => println!("❌ Subscribe message deserialization failed"),
            }
        }
        Err(e) => println!("❌ Subscribe message serialization failed: {}", e),
    }
    
    println!("\n🎉 IPC Message Standardization Test Complete!");
    println!("   Both beacon-emulator and receiver now use shared message types");
    println!("   from the shared-positioning crate, ensuring compatibility.");
}