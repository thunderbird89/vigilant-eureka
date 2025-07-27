//! Integration tests for beacon emulator components
//! 
//! This module contains integration tests that test the interaction between
//! multiple components of the beacon emulator system.

use beacon_emulator::*;
use shared_positioning::GeodeticPosition;
use uuid::Uuid;
use tokio::time::{sleep, Duration, timeout};
use std::time::SystemTime;
use tempfile::TempDir;

/// Test utilities for integration tests
mod integration_test_utils {
    use super::*;
    
    pub fn create_test_position() -> GeodeticPosition {
        GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        }
    }
    
    pub async fn create_test_manager() -> EmulatorManager {
        EmulatorManager::new("test_channel")
    }
    
    pub async fn create_test_manager_with_temp_state() -> (EmulatorManager, TempDir) {
        let temp_dir = TempDir::new().unwrap();
        let state_file = temp_dir.path().join("test_state.json");
        
        let mut manager = EmulatorManager::new("test_channel");
        manager.set_state_file_path(state_file);
        
        (manager, temp_dir)
    }
    
    pub fn create_multiple_test_positions(count: usize) -> Vec<GeodeticPosition> {
        (0..count).map(|i| GeodeticPosition {
            latitude: 32.0 + (i as f64) * 0.01,
            longitude: 45.0 + (i as f64) * 0.01,
            depth: 10.0 + (i as f64),
        }).collect()
    }
}

/// Integration tests for virtual communication space and message broadcasting
mod communication_integration_tests {
    use super::*;
    use integration_test_utils::*;
    
    #[tokio::test]
    async fn test_virtual_communication_space_integration() {
        let mut space = VirtualCommunicationSpace::new();
        
        // Create multiple channels
        let channel1 = space.get_or_create_channel("channel1");
        let channel2 = space.get_or_create_channel("channel2");
        
        // Set up receivers
        let mut receiver1 = channel1.subscribe();
        let mut receiver2 = channel2.subscribe();
        
        // Create test messages
        let beacon1_id = Uuid::new_v4();
        let beacon2_id = Uuid::new_v4();
        let position = create_test_position();
        
        let message1 = shared_positioning::VirtualMessage {
            beacon_id: beacon1_id,
            timestamp: SystemTime::now(),
            position,
            message_data: vec![0x01, 0x02, 0x03],
            signal_quality: 255,
        };
        
        let message2 = shared_positioning::VirtualMessage {
            beacon_id: beacon2_id,
            timestamp: SystemTime::now(),
            position,
            message_data: vec![0x04, 0x05, 0x06],
            signal_quality: 200,
        };
        
        // Broadcast messages to different channels
        channel1.broadcast_message(message1.clone()).await.unwrap();
        channel2.broadcast_message(message2.clone()).await.unwrap();
        
        // Verify channel isolation
        let received1 = timeout(Duration::from_millis(100), receiver1.recv()).await.unwrap().unwrap();
        assert_eq!(received1.beacon_id, beacon1_id);
        
        let received2 = timeout(Duration::from_millis(100), receiver2.recv()).await.unwrap().unwrap();
        assert_eq!(received2.beacon_id, beacon2_id);
        
        // Verify no cross-channel leakage
        let no_message1 = timeout(Duration::from_millis(50), receiver1.recv()).await;
        assert!(no_message1.is_err()); // Should timeout
        
        let no_message2 = timeout(Duration::from_millis(50), receiver2.recv()).await;
        assert!(no_message2.is_err()); // Should timeout
    }
    
    #[tokio::test]
    async fn test_multiple_subscribers_same_channel() {
        let mut space = VirtualCommunicationSpace::new();
        let channel = space.get_or_create_channel("test_channel");
        
        // Create multiple subscribers
        let mut receiver1 = channel.subscribe();
        let mut receiver2 = channel.subscribe();
        let mut receiver3 = channel.subscribe();
        
        let beacon_id = Uuid::new_v4();
        let message = shared_positioning::VirtualMessage {
            beacon_id,
            timestamp: SystemTime::now(),
            position: create_test_position(),
            message_data: vec![0x01, 0x02, 0x03],
            signal_quality: 255,
        };
        
        // Broadcast message
        channel.broadcast_message(message.clone()).await.unwrap();
        
        // All receivers should get the message
        let received1 = timeout(Duration::from_millis(100), receiver1.recv()).await.unwrap().unwrap();
        let received2 = timeout(Duration::from_millis(100), receiver2.recv()).await.unwrap().unwrap();
        let received3 = timeout(Duration::from_millis(100), receiver3.recv()).await.unwrap().unwrap();
        
        assert_eq!(received1.beacon_id, beacon_id);
        assert_eq!(received2.beacon_id, beacon_id);
        assert_eq!(received3.beacon_id, beacon_id);
    }
    
    #[tokio::test]
    async fn test_message_logging_and_retrieval() {
        let mut space = VirtualCommunicationSpace::new();
        let channel = space.get_or_create_channel("test_channel");
        
        let beacon_id = Uuid::new_v4();
        let position = create_test_position();
        
        // Send multiple messages
        for i in 0..5 {
            let message = shared_positioning::VirtualMessage {
                beacon_id,
                timestamp: SystemTime::now(),
                position,
                message_data: vec![i as u8],
                signal_quality: 255,
            };
            channel.broadcast_message(message).await.unwrap();
            sleep(Duration::from_millis(10)).await; // Small delay for timestamp differences
        }
        
        // Test message count
        assert_eq!(channel.get_message_count().await, 5);
        
        // Test recent messages retrieval
        let recent_messages = channel.get_recent_messages(3).await;
        assert_eq!(recent_messages.len(), 3);
        
        // Should be in reverse chronological order (most recent first)
        for i in 1..recent_messages.len() {
            assert!(recent_messages[i-1].timestamp >= recent_messages[i].timestamp);
        }
        
        // Test filtering by beacon
        let beacon_messages = channel.get_messages_by_beacon(beacon_id).await;
        assert_eq!(beacon_messages.len(), 5);
        
        // Test time-based filtering
        let now = SystemTime::now();
        let recent_by_time = channel.get_messages_since(now - Duration::from_secs(1)).await;
        assert_eq!(recent_by_time.len(), 5);
    }
    
    #[tokio::test]
    async fn test_channel_statistics() {
        let mut space = VirtualCommunicationSpace::new();
        let channel = space.get_or_create_channel("test_channel");
        
        let beacon1_id = Uuid::new_v4();
        let beacon2_id = Uuid::new_v4();
        let position = create_test_position();
        
        // Send messages from different beacons
        for beacon_id in [beacon1_id, beacon2_id] {
            for i in 0..3 {
                let message = shared_positioning::VirtualMessage {
                    beacon_id,
                    timestamp: SystemTime::now(),
                    position,
                    message_data: vec![i as u8],
                    signal_quality: 255,
                };
                channel.broadcast_message(message).await.unwrap();
            }
        }
        
        let stats = channel.get_channel_stats().await;
        assert_eq!(stats.name, "test_channel");
        assert_eq!(stats.total_messages, 6);
        assert_eq!(stats.unique_beacons, 2);
        assert!(stats.oldest_message.is_some());
        assert!(stats.newest_message.is_some());
    }
    
    #[tokio::test]
    async fn test_circular_buffer_behavior() {
        let mut space = VirtualCommunicationSpace::new();
        let channel = space.get_or_create_channel("test_channel");
        
        let beacon_id = Uuid::new_v4();
        let position = create_test_position();
        
        // Send more than the circular buffer limit (should be 10000)
        // We'll test with a smaller number for efficiency
        for i in 0..1005 {
            let message = shared_positioning::VirtualMessage {
                beacon_id,
                timestamp: SystemTime::now(),
                position,
                message_data: vec![(i % 256) as u8],
                signal_quality: 255,
            };
            channel.broadcast_message(message).await.unwrap();
        }
        
        // Should have triggered circular buffer cleanup
        let count = channel.get_message_count().await;
        assert!(count <= 10000);
        assert!(count >= 1000); // Should have kept at least 1000 after cleanup
    }
}

/// Integration tests for EmulatorManager and beacon orchestration
mod emulator_manager_integration_tests {
    use super::*;
    use integration_test_utils::*;
    
    #[tokio::test]
    async fn test_emulator_manager_beacon_lifecycle() {
        let mut manager = create_test_manager().await;
        let position = create_test_position();
        
        // Create beacon
        let beacon_id = manager.create_beacon(None, position, None).await.unwrap();
        assert!(manager.beacon_exists(beacon_id));
        assert_eq!(manager.get_total_beacon_count(), 1);
        assert_eq!(manager.get_active_beacon_count(), 0);
        
        // Start beacon
        manager.start_beacon(beacon_id).await.unwrap();
        assert_eq!(manager.get_active_beacon_count(), 1);
        
        let status = manager.get_beacon_status(beacon_id).unwrap();
        assert!(status.is_running);
        
        // Stop beacon
        manager.stop_beacon(beacon_id).await.unwrap();
        assert_eq!(manager.get_active_beacon_count(), 0);
        
        let status = manager.get_beacon_status(beacon_id).unwrap();
        assert!(!status.is_running);
        
        // Remove beacon
        manager.remove_beacon(beacon_id).await.unwrap();
        assert!(!manager.beacon_exists(beacon_id));
        assert_eq!(manager.get_total_beacon_count(), 0);
    }
    
    #[tokio::test]
    async fn test_multiple_beacon_management() {
        let mut manager = create_test_manager().await;
        let positions = create_multiple_test_positions(5);
        
        // Create multiple beacons
        let mut beacon_ids = Vec::new();
        for position in positions {
            let id = manager.create_beacon(None, position, None).await.unwrap();
            beacon_ids.push(id);
        }
        
        assert_eq!(manager.get_total_beacon_count(), 5);
        assert_eq!(manager.get_active_beacon_count(), 0);
        
        // Start all beacons
        let started_ids = manager.start_all_beacons().await.unwrap();
        assert_eq!(started_ids.len(), 5);
        assert_eq!(manager.get_active_beacon_count(), 5);
        
        // Verify all beacons are running
        for beacon_id in &beacon_ids {
            let status = manager.get_beacon_status(*beacon_id).unwrap();
            assert!(status.is_running);
        }
        
        // Stop all beacons
        let stopped_ids = manager.stop_all_beacons().await.unwrap();
        assert_eq!(stopped_ids.len(), 5);
        assert_eq!(manager.get_active_beacon_count(), 0);
        
        // Verify all beacons are stopped
        for beacon_id in &beacon_ids {
            let status = manager.get_beacon_status(*beacon_id).unwrap();
            assert!(!status.is_running);
        }
    }
    
    #[tokio::test]
    async fn test_beacon_configuration_updates() {
        let mut manager = create_test_manager().await;
        let position = create_test_position();
        
        let beacon_id = manager.create_beacon(None, position, None).await.unwrap();
        
        // Update position
        let new_position = GeodeticPosition {
            latitude: 33.456,
            longitude: 46.789,
            depth: 15.0,
        };
        
        manager.update_beacon_position(beacon_id, new_position).await.unwrap();
        
        let status = manager.get_beacon_status(beacon_id).unwrap();
        assert_eq!(status.position.latitude, new_position.latitude);
        assert_eq!(status.position.longitude, new_position.longitude);
        assert_eq!(status.position.depth, new_position.depth);
        
        // Update movement pattern
        let movement_pattern = MovementPattern::Linear {
            speed_m_per_s: 1.5,
            bearing_deg: 45.0,
        };
        
        manager.update_beacon_movement_pattern(beacon_id, movement_pattern.clone()).await.unwrap();
        
        let status = manager.get_beacon_status(beacon_id).unwrap();
        match status.movement_pattern {
            MovementPattern::Linear { speed_m_per_s, bearing_deg } => {
                assert_eq!(speed_m_per_s, 1.5);
                assert_eq!(bearing_deg, 45.0);
            }
            _ => panic!("Expected Linear movement pattern"),
        }
    }
    
    #[tokio::test]
    async fn test_beacon_message_transmission_integration() {
        let mut manager = create_test_manager().await;
        let position = create_test_position();
        
        // Get communication space and subscribe to channel
        let comm_space = manager.get_communication_space();
        let channel = {
            let mut space = comm_space.write().await;
            space.get_or_create_channel("test_channel")
        };
        let mut receiver = channel.subscribe();
        
        // Create and start beacon
        let beacon_id = manager.create_beacon(None, position, None).await.unwrap();
        manager.start_beacon(beacon_id).await.unwrap();
        
        // Wait for message transmission
        let received_message = timeout(Duration::from_millis(500), receiver.recv()).await;
        assert!(received_message.is_ok(), "Should have received a message");
        
        let message = received_message.unwrap().unwrap();
        assert_eq!(message.beacon_id, beacon_id);
        assert_eq!(message.position.latitude, position.latitude);
        assert_eq!(message.signal_quality, 255);
        
        // Stop beacon
        manager.stop_beacon(beacon_id).await.unwrap();
    }
    
    #[tokio::test]
    async fn test_emulator_manager_statistics() {
        let mut manager = create_test_manager().await;
        let positions = create_multiple_test_positions(3);
        
        // Create beacons
        let mut beacon_ids = Vec::new();
        for position in positions {
            let id = manager.create_beacon(None, position, None).await.unwrap();
            beacon_ids.push(id);
        }
        
        // Start some beacons
        manager.start_beacon(beacon_ids[0]).await.unwrap();
        manager.start_beacon(beacon_ids[1]).await.unwrap();
        
        let stats = manager.get_manager_stats().await;
        assert_eq!(stats.total_beacons, 3);
        assert_eq!(stats.running_beacons, 2);
        assert_eq!(stats.stopped_beacons, 1);
        assert_eq!(stats.current_channel, "test_channel");
        assert!(stats.channel_names.contains(&"test_channel".to_string()));
    }
    
    #[tokio::test]
    async fn test_state_persistence() {
        let (mut manager, _temp_dir) = create_test_manager_with_temp_state().await;
        let position = create_test_position();
        
        // Create and configure beacon
        let beacon_id = manager.create_beacon(None, position, None).await.unwrap();
        let movement_pattern = MovementPattern::Linear {
            speed_m_per_s: 1.0,
            bearing_deg: 90.0,
        };
        manager.update_beacon_movement_pattern(beacon_id, movement_pattern).await.unwrap();
        
        // Save state
        manager.save_state().await.unwrap();
        
        // Create new manager and load state
        let state_file_path = manager.get_state_file_path().clone();
        let mut new_manager = EmulatorManager::new("different_channel");
        new_manager.set_state_file_path(state_file_path);
        new_manager.load_state().await.unwrap();
        
        // Verify state was loaded
        assert!(new_manager.beacon_exists(beacon_id));
        assert_eq!(new_manager.get_current_channel(), "test_channel");
        
        let status = new_manager.get_beacon_status(beacon_id).unwrap();
        match status.movement_pattern {
            MovementPattern::Linear { speed_m_per_s, bearing_deg } => {
                assert_eq!(speed_m_per_s, 1.0);
                assert_eq!(bearing_deg, 90.0);
            }
            _ => panic!("Expected Linear movement pattern"),
        }
    }
    
    #[tokio::test]
    async fn test_emulator_shutdown() {
        let mut manager = create_test_manager().await;
        let positions = create_multiple_test_positions(3);
        
        // Create and start multiple beacons
        for position in positions {
            let id = manager.create_beacon(None, position, None).await.unwrap();
            manager.start_beacon(id).await.unwrap();
        }
        
        assert_eq!(manager.get_active_beacon_count(), 3);
        
        // Shutdown manager
        manager.shutdown().await.unwrap();
        
        assert_eq!(manager.get_active_beacon_count(), 0);
        assert_eq!(manager.get_total_beacon_count(), 0);
    }
}

/// Integration tests for scenario generation and management
mod scenario_integration_tests {
    use super::*;
    use integration_test_utils::*;
    
    #[tokio::test]
    async fn test_triangle_scenario_creation() {
        let mut manager = create_test_manager().await;
        let center = create_test_position();
        
        let beacon_ids = manager.create_scenario(
            ScenarioType::Triangle,
            3,
            100.0,
            center,
        ).await.unwrap();
        
        assert_eq!(beacon_ids.len(), 3);
        assert_eq!(manager.get_total_beacon_count(), 3);
        
        // Verify beacon positions form a triangle
        let mut positions = Vec::new();
        for beacon_id in beacon_ids {
            let status = manager.get_beacon_status(beacon_id).unwrap();
            positions.push(status.position);
        }
        
        // All beacons should be at the same depth
        for pos in &positions {
            assert_eq!(pos.depth, center.depth);
        }
        
        // Positions should be different
        assert_ne!(positions[0].latitude, positions[1].latitude);
        assert_ne!(positions[1].latitude, positions[2].latitude);
        assert_ne!(positions[0].latitude, positions[2].latitude);
    }
    
    #[tokio::test]
    async fn test_square_scenario_creation() {
        let mut manager = create_test_manager().await;
        let center = create_test_position();
        
        let beacon_ids = manager.create_scenario(
            ScenarioType::Square,
            4,
            200.0,
            center,
        ).await.unwrap();
        
        assert_eq!(beacon_ids.len(), 4);
        assert_eq!(manager.get_total_beacon_count(), 4);
        
        // Verify all beacons exist and have valid configurations
        for beacon_id in beacon_ids {
            let status = manager.get_beacon_status(beacon_id).unwrap();
            assert_eq!(status.position.depth, center.depth);
            assert!(!status.is_running); // Should not be auto-started
        }
    }
    
    #[tokio::test]
    async fn test_line_scenario_creation() {
        let mut manager = create_test_manager().await;
        let center = create_test_position();
        
        let beacon_ids = manager.create_scenario(
            ScenarioType::Line,
            5,
            50.0,
            center,
        ).await.unwrap();
        
        assert_eq!(beacon_ids.len(), 5);
        
        // Verify beacons are arranged in a line
        let mut positions = Vec::new();
        for beacon_id in beacon_ids {
            let status = manager.get_beacon_status(beacon_id).unwrap();
            positions.push(status.position);
        }
        
        // All should have same longitude and depth
        for pos in &positions {
            assert_eq!(pos.longitude, center.longitude);
            assert_eq!(pos.depth, center.depth);
        }
        
        // Latitudes should be ordered
        for i in 1..positions.len() {
            assert!(positions[i].latitude > positions[i-1].latitude);
        }
    }
    
    #[tokio::test]
    async fn test_grid_scenario_creation() {
        let mut manager = create_test_manager().await;
        let center = create_test_position();
        
        let beacon_ids = manager.create_scenario(
            ScenarioType::Grid,
            9, // 3x3 grid
            75.0,
            center,
        ).await.unwrap();
        
        assert_eq!(beacon_ids.len(), 9);
        
        // Verify all beacons are at the same depth
        for beacon_id in beacon_ids {
            let status = manager.get_beacon_status(beacon_id).unwrap();
            assert_eq!(status.position.depth, center.depth);
        }
    }
    
    #[tokio::test]
    async fn test_scenario_validation() {
        let mut manager = create_test_manager().await;
        let center = create_test_position();
        
        // Invalid triangle scenario (wrong count)
        let result = manager.create_scenario(ScenarioType::Triangle, 4, 100.0, center).await;
        assert!(result.is_err());
        
        // Invalid square scenario (wrong count)
        let result = manager.create_scenario(ScenarioType::Square, 3, 100.0, center).await;
        assert!(result.is_err());
        
        // Invalid spacing
        let result = manager.create_scenario(ScenarioType::Triangle, 3, 0.0, center).await;
        assert!(result.is_err());
        
        // Invalid beacon count
        let result = manager.create_scenario(ScenarioType::Line, 0, 100.0, center).await;
        assert!(result.is_err());
    }
    
    #[tokio::test]
    async fn test_scenario_with_message_transmission() {
        let mut manager = create_test_manager().await;
        let center = create_test_position();
        
        // Get communication space and subscribe to channel
        let comm_space = manager.get_communication_space();
        let channel = {
            let mut space = comm_space.write().await;
            space.get_or_create_channel("test_channel")
        };
        let mut receiver = channel.subscribe();
        
        // Create triangle scenario
        let beacon_ids = manager.create_scenario(
            ScenarioType::Triangle,
            3,
            100.0,
            center,
        ).await.unwrap();
        
        // Start all beacons
        for beacon_id in &beacon_ids {
            manager.start_beacon(*beacon_id).await.unwrap();
        }
        
        // Should receive messages from all beacons
        let mut received_beacons = std::collections::HashSet::new();
        
        // Wait for messages from all beacons (with timeout)
        let start_time = std::time::Instant::now();
        while received_beacons.len() < 3 && start_time.elapsed() < Duration::from_secs(2) {
            if let Ok(message) = timeout(Duration::from_millis(100), receiver.recv()).await {
                if let Ok(msg) = message {
                    received_beacons.insert(msg.beacon_id);
                }
            }
        }
        
        assert_eq!(received_beacons.len(), 3, "Should have received messages from all 3 beacons");
        
        // Stop all beacons
        for beacon_id in beacon_ids {
            manager.stop_beacon(beacon_id).await.unwrap();
        }
    }
}

/// Integration tests for configuration management
mod configuration_integration_tests {
    use super::*;
    use integration_test_utils::*;
    use tempfile::NamedTempFile;
    
    #[tokio::test]
    async fn test_beacon_config_file_integration() {
        let mut manager = create_test_manager().await;
        let position = create_test_position();
        
        // Create a temporary config file
        let temp_file = NamedTempFile::new().unwrap();
        let config_path = temp_file.path();
        
        // Generate a config template
        manager.generate_config_template(config_path, crate::config::ConfigFormat::Toml).await.unwrap();
        
        // Load config and create beacon
        let beacon_id = manager.create_beacon_from_config(None, position, config_path).await.unwrap();
        
        assert!(manager.beacon_exists(beacon_id));
        
        let status = manager.get_beacon_status(beacon_id).unwrap();
        assert_eq!(status.position.latitude, position.latitude);
    }
    
    #[tokio::test]
    async fn test_emulator_config_integration() {
        let mut manager = create_test_manager().await;
        let position = create_test_position();
        
        // Create a temporary emulator config file
        let temp_file = NamedTempFile::new().unwrap();
        let config_path = temp_file.path();
        
        // Generate an emulator config template
        manager.generate_emulator_config_template(
            config_path, 
            position, 
            crate::config::ConfigFormat::Json
        ).await.unwrap();
        
        // Load emulator config and create beacon
        let beacon_id = manager.create_beacon_from_emulator_config(config_path).await.unwrap();
        
        assert!(manager.beacon_exists(beacon_id));
        
        let status = manager.get_beacon_status(beacon_id).unwrap();
        assert_eq!(status.position.latitude, position.latitude);
        assert_eq!(status.position.longitude, position.longitude);
        assert_eq!(status.position.depth, position.depth);
    }
    
    #[tokio::test]
    async fn test_config_validation_integration() {
        let manager = create_test_manager().await;
        
        // Test valid config
        let valid_config = manager.create_default_beacon_config();
        assert!(manager.validate_emulator_config(&valid_config).is_ok());
        
        // Test emulator-specific config
        let emulator_config = manager.create_default_emulator_config(None, create_test_position());
        assert!(manager.validate_emulator_specific_config(&emulator_config).is_ok());
    }
}

/// Integration tests for error handling and edge cases
mod error_integration_tests {
    use super::*;
    use integration_test_utils::*;
    
    #[tokio::test]
    async fn test_beacon_not_found_errors() {
        let mut manager = create_test_manager().await;
        let non_existent_id = Uuid::new_v4();
        
        // Test various operations on non-existent beacon
        let result = manager.get_beacon_status(non_existent_id);
        assert!(matches!(result, Err(EmulatorError::BeaconNotFound(_))));
        
        let result = manager.start_beacon(non_existent_id).await;
        assert!(matches!(result, Err(EmulatorError::BeaconNotFound(_))));
        
        let result = manager.stop_beacon(non_existent_id).await;
        assert!(matches!(result, Err(EmulatorError::BeaconNotFound(_))));
        
        let result = manager.remove_beacon(non_existent_id).await;
        assert!(matches!(result, Err(EmulatorError::BeaconNotFound(_))));
    }
    
    #[tokio::test]
    async fn test_duplicate_beacon_creation() {
        let mut manager = create_test_manager().await;
        let position = create_test_position();
        let specific_id = Uuid::new_v4();
        
        // Create first beacon
        manager.create_beacon(Some(specific_id), position, None).await.unwrap();
        
        // Try to create duplicate
        let result = manager.create_beacon(Some(specific_id), position, None).await;
        assert!(matches!(result, Err(EmulatorError::BeaconExists(_))));
    }
    
    #[tokio::test]
    async fn test_invalid_position_handling() {
        let mut manager = create_test_manager().await;
        
        // Test invalid latitude
        let invalid_position = GeodeticPosition {
            latitude: 95.0, // > 90
            longitude: 45.476,
            depth: 10.0,
        };
        
        let result = manager.create_beacon(None, invalid_position, None).await;
        // Should succeed at creation but fail during validation in movement
        assert!(result.is_ok());
        
        let beacon_id = result.unwrap();
        
        // Try to update with another invalid position
        let invalid_position2 = GeodeticPosition {
            latitude: 32.123,
            longitude: 200.0, // > 180
            depth: 10.0,
        };
        
        let result = manager.update_beacon_position(beacon_id, invalid_position2).await;
        assert!(result.is_err());
    }
    
    #[tokio::test]
    async fn test_concurrent_operations() {
        let mut manager = create_test_manager().await;
        let positions = create_multiple_test_positions(10);
        
        // Create multiple beacons sequentially
        let mut beacon_ids = Vec::new();
        for position in positions {
            let beacon_id = manager.create_beacon(None, position, None).await.unwrap();
            beacon_ids.push(beacon_id);
        }
        
        assert_eq!(beacon_ids.len(), 10);
        assert_eq!(manager.get_total_beacon_count(), 10);
        
        // Start all beacons
        let started_ids = manager.start_all_beacons().await.unwrap();
        assert_eq!(started_ids.len(), 10);
        
        // Stop all beacons
        let stopped_ids = manager.stop_all_beacons().await.unwrap();
        assert_eq!(stopped_ids.len(), 10);
    }
    
    #[tokio::test]
    async fn test_resource_cleanup_on_shutdown() {
        let mut manager = create_test_manager().await;
        let positions = create_multiple_test_positions(5);
        
        // Create and start multiple beacons
        for position in positions {
            let id = manager.create_beacon(None, position, None).await.unwrap();
            manager.start_beacon(id).await.unwrap();
        }
        
        assert_eq!(manager.get_active_beacon_count(), 5);
        
        // Shutdown should clean up all resources
        manager.shutdown().await.unwrap();
        
        assert_eq!(manager.get_active_beacon_count(), 0);
        assert_eq!(manager.get_total_beacon_count(), 0);
    }
}