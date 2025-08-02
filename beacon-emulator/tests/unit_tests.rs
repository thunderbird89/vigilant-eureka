//! Unit tests for beacon emulator components
//! 
//! This module contains unit tests for individual components of the beacon emulator,
//! focusing on isolated functionality testing with mocked dependencies.

use beacon_emulator::*;
use shared_positioning::{BeaconConfig, GeodeticPosition};
use uuid::Uuid;
use tokio::time::{sleep, Duration};


/// Test utilities and helper functions
mod test_utils {
    use super::*;
    
    pub fn create_test_position() -> GeodeticPosition {
        GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        }
    }
    
    pub fn create_test_beacon_config() -> BeaconConfig {
        BeaconConfig::new(Uuid::new_v4())
    }
    
    pub fn create_test_channel() -> VirtualChannel {
        VirtualChannel::new("test_channel")
    }
    
    pub async fn create_test_beacon() -> VirtualBeacon {
        let id = Uuid::new_v4();
        let config = create_test_beacon_config();
        let position = create_test_position();
        let channel = create_test_channel();
        
        VirtualBeacon::new(id, config, position, channel).unwrap()
    }
}

/// Unit tests for VirtualBeacon functionality
mod virtual_beacon_tests {
    use super::*;
    use test_utils::*;
    
    #[tokio::test]
    async fn test_beacon_creation() {
        let beacon_id = Uuid::new_v4();
        let config = create_test_beacon_config();
        let position = create_test_position();
        let channel = create_test_channel();
        
        let beacon = VirtualBeacon::new(beacon_id, config, position, channel).unwrap();
        
        assert_eq!(beacon.id(), beacon_id);
        assert_eq!(beacon.position().latitude, position.latitude);
        assert_eq!(beacon.position().longitude, position.longitude);
        assert_eq!(beacon.position().depth, position.depth);
        assert!(!beacon.is_running());
    }
    
    #[tokio::test]
    async fn test_beacon_status() {
        let beacon = create_test_beacon().await;
        let status = beacon.get_status();
        
        assert!(!status.is_running);
        assert!(matches!(status.movement_pattern, MovementPattern::Stationary));
        assert_eq!(status.stats.messages_sent, 0);
        assert!(status.stats.last_transmission.is_none());
        assert_eq!(status.stats.transmission_failures, 0);
    }
    
    #[tokio::test]
    async fn test_beacon_position_update() {
        let mut beacon = create_test_beacon().await;
        let initial_position = beacon.position();
        
        let new_position = GeodeticPosition {
            latitude: 33.456,
            longitude: 46.789,
            depth: 15.0,
        };
        
        beacon.update_position(new_position).unwrap();
        
        let updated_position = beacon.position();
        assert_eq!(updated_position.latitude, new_position.latitude);
        assert_eq!(updated_position.longitude, new_position.longitude);
        assert_eq!(updated_position.depth, new_position.depth);
        assert_ne!(updated_position.latitude, initial_position.latitude);
    }
    
    #[tokio::test]
    async fn test_beacon_position_validation() {
        let mut beacon = create_test_beacon().await;
        
        // Test invalid latitude
        let invalid_position = GeodeticPosition {
            latitude: 95.0, // Invalid: > 90
            longitude: 45.476,
            depth: 10.0,
        };
        
        let result = beacon.update_position(invalid_position);
        assert!(result.is_err());
        assert!(matches!(result.unwrap_err(), EmulatorError::MovementError(_)));
    }
    
    #[tokio::test]
    async fn test_beacon_movement_pattern_update() {
        let mut beacon = create_test_beacon().await;
        
        let linear_pattern = MovementPattern::Linear {
            speed_m_per_s: 1.5,
            bearing_deg: 45.0,
        };
        
        beacon.set_movement_pattern(linear_pattern.clone()).unwrap();
        
        let status = beacon.get_status();
        match status.movement_pattern {
            MovementPattern::Linear { speed_m_per_s, bearing_deg } => {
                assert_eq!(speed_m_per_s, 1.5);
                assert_eq!(bearing_deg, 45.0);
            }
            _ => panic!("Expected Linear movement pattern"),
        }
    }
    
    #[tokio::test]
    async fn test_beacon_movement_pattern_validation() {
        let mut beacon = create_test_beacon().await;
        
        // Test invalid speed
        let invalid_pattern = MovementPattern::Linear {
            speed_m_per_s: -1.0, // Invalid: negative speed
            bearing_deg: 45.0,
        };
        
        let result = beacon.set_movement_pattern(invalid_pattern);
        assert!(result.is_err());
        assert!(matches!(result.unwrap_err(), EmulatorError::MovementError(_)));
    }
    
    #[tokio::test]
    async fn test_beacon_config_update() {
        let mut beacon = create_test_beacon().await;
        let original_config = beacon.get_status().config;
        
        let mut new_config = create_test_beacon_config();
        new_config.transmission.interval_ms = 5000;
        
        beacon.update_config(new_config.clone()).unwrap();
        
        let updated_status = beacon.get_status();
        assert_eq!(updated_status.config.transmission.interval_ms, 5000);
        assert_ne!(updated_status.config.transmission.interval_ms, 
                  original_config.transmission.interval_ms);
    }
    
    #[tokio::test]
    async fn test_beacon_lifecycle() {
        let mut beacon = create_test_beacon().await;
        
        // Initially not running
        assert!(!beacon.is_running());
        
        // Start beacon
        let task_handle = beacon.start().await.unwrap();
        assert!(beacon.is_running());
        
        // Let it run briefly
        sleep(Duration::from_millis(100)).await;
        
        // Stop beacon
        beacon.stop().unwrap();
        assert!(!beacon.is_running());
        
        // Clean up task
        task_handle.abort();
    }
    
    #[tokio::test]
    async fn test_beacon_double_start_error() {
        let mut beacon = create_test_beacon().await;
        
        let _task1 = beacon.start().await.unwrap();
        
        // Try to start again - should fail
        let result = beacon.start().await;
        assert!(result.is_err());
        assert!(matches!(result.unwrap_err(), EmulatorError::ConfigError(_)));
        
        beacon.stop().unwrap();
    }
    
    #[tokio::test]
    async fn test_beacon_message_transmission() {
        let channel = create_test_channel();
        let mut receiver = channel.subscribe();
        
        let beacon_id = Uuid::new_v4();
        let config = create_test_beacon_config();
        let position = create_test_position();
        
        let mut beacon = VirtualBeacon::new(beacon_id, config, position, channel.clone()).unwrap();
        
        // Start beacon
        let task_handle = beacon.start().await.unwrap();
        
        // Wait for message transmission
        let received_message = tokio::time::timeout(
            Duration::from_millis(500),
            receiver.recv()
        ).await;
        
        assert!(received_message.is_ok(), "Should have received a message");
        
        let message = received_message.unwrap().unwrap();
        assert_eq!(message.beacon_id, beacon_id);
        assert_eq!(message.position.latitude, position.latitude);
        assert_eq!(message.signal_quality, 255);
        assert!(!message.message_data.is_empty());
        
        // Stop beacon
        beacon.stop().unwrap();
        task_handle.abort();
    }
    
    #[tokio::test]
    async fn test_beacon_statistics_tracking() {
        let channel = create_test_channel();
        let mut beacon = VirtualBeacon::new(
            Uuid::new_v4(),
            create_test_beacon_config(),
            create_test_position(),
            channel,
        ).unwrap();
        
        let initial_stats = beacon.get_status().stats;
        assert_eq!(initial_stats.messages_sent, 0);
        assert!(initial_stats.last_transmission.is_none());
        
        // Start beacon and let it run briefly
        let task_handle = beacon.start().await.unwrap();
        sleep(Duration::from_millis(200)).await;
        beacon.stop().unwrap();
        
        let final_stats = beacon.get_status().stats;
        assert!(final_stats.uptime.as_millis() > 0);
        
        task_handle.abort();
    }
}

/// Unit tests for MovementPattern functionality
mod movement_pattern_tests {
    use super::*;
    use beacon_emulator::movement::{MovementPatternValidator, MovementCoordinateTransformer};
    
    #[test]
    fn test_movement_pattern_validation() {
        // Valid patterns
        assert!(MovementPatternValidator::validate_pattern(&MovementPattern::Stationary).is_ok());
        
        assert!(MovementPatternValidator::validate_pattern(&MovementPattern::Linear {
            speed_m_per_s: 1.5,
            bearing_deg: 45.0,
        }).is_ok());
        
        assert!(MovementPatternValidator::validate_pattern(&MovementPattern::Circular {
            radius_m: 100.0,
            period_s: 60.0,
        }).is_ok());
        
        assert!(MovementPatternValidator::validate_pattern(&MovementPattern::Random {
            max_speed_m_per_s: 2.0,
        }).is_ok());
        
        // Invalid patterns
        assert!(MovementPatternValidator::validate_pattern(&MovementPattern::Linear {
            speed_m_per_s: -1.0, // Negative speed
            bearing_deg: 45.0,
        }).is_err());
        
        assert!(MovementPatternValidator::validate_pattern(&MovementPattern::Linear {
            speed_m_per_s: 1.0,
            bearing_deg: 400.0, // Invalid bearing
        }).is_err());
        
        assert!(MovementPatternValidator::validate_pattern(&MovementPattern::Circular {
            radius_m: -100.0, // Negative radius
            period_s: 60.0,
        }).is_err());
        
        assert!(MovementPatternValidator::validate_pattern(&MovementPattern::Random {
            max_speed_m_per_s: 0.0, // Zero speed for random
        }).is_err());
    }
    
    #[test]
    fn test_position_validation() {
        // Valid positions
        let valid_pos = GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        };
        assert!(MovementPatternValidator::validate_position(valid_pos).is_ok());
        
        // Edge cases
        let north_pole = GeodeticPosition {
            latitude: 90.0,
            longitude: 0.0,
            depth: 0.0,
        };
        assert!(MovementPatternValidator::validate_position(north_pole).is_ok());
        
        let south_pole = GeodeticPosition {
            latitude: -90.0,
            longitude: 180.0,
            depth: 11000.0,
        };
        assert!(MovementPatternValidator::validate_position(south_pole).is_ok());
        
        // Invalid positions
        let invalid_lat = GeodeticPosition {
            latitude: 95.0, // > 90
            longitude: 45.476,
            depth: 10.0,
        };
        assert!(MovementPatternValidator::validate_position(invalid_lat).is_err());
        
        let invalid_lon = GeodeticPosition {
            latitude: 32.123,
            longitude: 200.0, // > 180
            depth: 10.0,
        };
        assert!(MovementPatternValidator::validate_position(invalid_lon).is_err());
        
        let invalid_depth = GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: -5.0, // Negative depth
        };
        assert!(MovementPatternValidator::validate_position(invalid_depth).is_err());
    }
    
    #[test]
    fn test_coordinate_transformer_linear_movement() {
        let mut transformer = MovementCoordinateTransformer::new();
        
        let start_pos = GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        };
        
        // Test northward movement (bearing 0°)
        let result = transformer.apply_linear_movement(start_pos, 1.0, 0.0, 1.0);
        assert!(result.is_ok());
        let new_pos = result.unwrap();
        assert!(new_pos.latitude > start_pos.latitude); // Should move north
        assert_eq!(new_pos.longitude, start_pos.longitude); // Longitude unchanged
        assert_eq!(new_pos.depth, start_pos.depth); // Depth unchanged
        
        // Test eastward movement (bearing 90°)
        let result = transformer.apply_linear_movement(start_pos, 1.0, 90.0, 1.0);
        assert!(result.is_ok());
        let new_pos = result.unwrap();
        assert_eq!(new_pos.latitude, start_pos.latitude); // Latitude unchanged
        assert!(new_pos.longitude > start_pos.longitude); // Should move east
        
        // Test invalid parameters
        assert!(transformer.apply_linear_movement(start_pos, -1.0, 0.0, 1.0).is_err());
        assert!(transformer.apply_linear_movement(start_pos, 1.0, 400.0, 1.0).is_err());
        assert!(transformer.apply_linear_movement(start_pos, 1.0, 0.0, -1.0).is_err());
    }
    
    #[test]
    fn test_coordinate_transformer_circular_movement() {
        let mut transformer = MovementCoordinateTransformer::new();
        
        let center_pos = GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        };
        
        // Test circular movement at different times
        let result1 = transformer.apply_circular_movement(center_pos, 100.0, 60.0, 0.0);
        assert!(result1.is_ok());
        
        let result2 = transformer.apply_circular_movement(center_pos, 100.0, 60.0, 15.0);
        assert!(result2.is_ok());
        
        let pos1 = result1.unwrap();
        let pos2 = result2.unwrap();
        
        // Positions should be different
        assert_ne!(pos1.latitude, pos2.latitude);
        assert_ne!(pos1.longitude, pos2.longitude);
        assert_eq!(pos1.depth, pos2.depth); // Depth should remain the same
        
        // Test invalid parameters
        assert!(transformer.apply_circular_movement(center_pos, -100.0, 60.0, 0.0).is_err());
        assert!(transformer.apply_circular_movement(center_pos, 100.0, -60.0, 0.0).is_err());
        assert!(transformer.apply_circular_movement(center_pos, 100.0, 60.0, -1.0).is_err());
    }
    
    #[test]
    fn test_coordinate_transformer_random_movement() {
        let mut transformer = MovementCoordinateTransformer::new();
        
        let start_pos = GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        };
        
        // Test multiple random movements
        for _ in 0..10 {
            let result = transformer.apply_random_movement(start_pos, 1.0, 1.0);
            assert!(result.is_ok());
            
            let new_pos = result.unwrap();
            assert!(MovementPatternValidator::validate_position(new_pos).is_ok());
            assert_eq!(new_pos.depth, start_pos.depth); // Depth should remain the same
        }
        
        // Test invalid parameters
        assert!(transformer.apply_random_movement(start_pos, -1.0, 1.0).is_err());
        assert!(transformer.apply_random_movement(start_pos, 1.0, -1.0).is_err());
    }
    
    #[test]
    fn test_movement_pattern_string_parsing() {
        use std::str::FromStr;
        
        // Valid patterns
        assert!(MovementPattern::from_str("stationary").is_ok());
        assert!(MovementPattern::from_str("linear:1.5:45").is_ok());
        assert!(MovementPattern::from_str("circular:100:60").is_ok());
        assert!(MovementPattern::from_str("random:2.0").is_ok());
        
        // Invalid patterns
        assert!(MovementPattern::from_str("invalid").is_err());
        assert!(MovementPattern::from_str("linear:1.5").is_err()); // Missing bearing
        assert!(MovementPattern::from_str("linear:-1.5:45").is_err()); // Negative speed
        assert!(MovementPattern::from_str("circular:100").is_err()); // Missing period
        assert!(MovementPattern::from_str("random").is_err()); // Missing max speed
    }
    
    #[test]
    fn test_movement_pattern_display() {
        let stationary = MovementPattern::Stationary;
        assert_eq!(stationary.to_string(), "stationary");
        
        let linear = MovementPattern::Linear {
            speed_m_per_s: 1.5,
            bearing_deg: 45.0,
        };
        assert_eq!(linear.to_string(), "linear:1.5:45");
        
        let circular = MovementPattern::Circular {
            radius_m: 100.0,
            period_s: 60.0,
        };
        assert_eq!(circular.to_string(), "circular:100:60");
        
        let random = MovementPattern::Random {
            max_speed_m_per_s: 2.0,
        };
        assert_eq!(random.to_string(), "random:2");
    }
}

/// Unit tests for message building functionality
mod message_building_tests {
    use super::*;
    use shared_positioning::MessageBuilder;
    
    #[test]
    fn test_message_builder_creation() {
        let _builder = MessageBuilder::new();
        // MessageBuilder should be created successfully
        // (This is mainly testing that the constructor works)
    }
    
    #[tokio::test]
    async fn test_message_versions() {
        let beacon_id = Uuid::new_v4();
        let position = GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        };
        let signal_quality = 255;
        let sequence_number = 1;
        
        let builder = MessageBuilder::new();
        
        // Test V1 message building
        let v1_result = builder.build_v1_message_with_uuid(
            beacon_id,
            position,
            signal_quality,
            sequence_number,
        );
        assert!(v1_result.is_ok());
        let v1_message = v1_result.unwrap();
        assert!(!v1_message.is_empty());
        
        // Test V2 message building
        let v2_result = builder.build_v2_message_with_uuid(
            beacon_id,
            position,
            signal_quality,
            sequence_number,
        );
        assert!(v2_result.is_ok());
        let v2_message = v2_result.unwrap();
        assert!(!v2_message.is_empty());
        
        // Test V3 message building
        let v3_result = builder.build_v3_message(
            beacon_id,
            position,
            signal_quality,
            sequence_number,
        );
        assert!(v3_result.is_ok());
        let v3_message = v3_result.unwrap();
        assert!(!v3_message.is_empty());
        
        // Messages should be different lengths/formats
        assert_ne!(v1_message.len(), v2_message.len());
        assert_ne!(v2_message.len(), v3_message.len());
    }
    
    #[test]
    fn test_message_sequence_numbers() {
        let beacon_id = Uuid::new_v4();
        let position = GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        };
        let builder = MessageBuilder::new();
        
        // Test sequence number wrapping
        let max_seq = u16::MAX;
        let result = builder.build_v3_message(beacon_id, position, 255, max_seq);
        assert!(result.is_ok());
        
        // Test sequence number 0
        let result = builder.build_v3_message(beacon_id, position, 255, 0);
        assert!(result.is_ok());
    }
}

/// Unit tests for error handling
mod error_handling_tests {
    use super::*;
    
    #[test]
    fn test_emulator_error_types() {
        let beacon_id = Uuid::new_v4();
        
        // Test different error types
        let beacon_exists_error = EmulatorError::BeaconExists(beacon_id);
        assert!(beacon_exists_error.to_string().contains(&beacon_id.to_string()));
        
        let beacon_not_found_error = EmulatorError::BeaconNotFound(beacon_id);
        assert!(beacon_not_found_error.to_string().contains(&beacon_id.to_string()));
        
        let config_error = EmulatorError::ConfigError("Test config error".to_string());
        assert!(config_error.to_string().contains("Test config error"));
        
        let movement_error = EmulatorError::MovementError("Test movement error".to_string());
        assert!(movement_error.to_string().contains("Test movement error"));
    }
    
    #[test]
    fn test_error_conversion() {
        // Test std::io::Error conversion
        let io_error = std::io::Error::new(std::io::ErrorKind::NotFound, "File not found");
        let emulator_error: EmulatorError = io_error.into();
        assert!(matches!(emulator_error, EmulatorError::IoError(_)));
        
        // Test serde_json::Error conversion
        let json_error = serde_json::from_str::<serde_json::Value>("invalid json");
        assert!(json_error.is_err());
        let emulator_error: EmulatorError = json_error.unwrap_err().into();
        assert!(matches!(emulator_error, EmulatorError::SerializationError(_)));
    }
}