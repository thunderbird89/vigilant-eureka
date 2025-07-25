#[cfg(test)]
mod tests {
    use uuid::Uuid;
    use std::time::{SystemTime, Duration};
    use shared_positioning::GeodeticPosition;
    use crate::{
        MovementPattern, ScenarioType, ExportFormat,
        VirtualBeaconStats, VirtualMessage
    };

    #[test]
    fn test_movement_pattern_parsing() {
        // Test stationary
        let stationary = "stationary".parse::<MovementPattern>().unwrap();
        assert!(matches!(stationary, MovementPattern::Stationary));

        // Test linear
        let linear = "linear:1.5:45.0".parse::<MovementPattern>().unwrap();
        assert!(matches!(linear, MovementPattern::Linear { speed_m_per_s: 1.5, bearing_deg: 45.0 }));

        // Test circular
        let circular = "circular:10.0:30.0".parse::<MovementPattern>().unwrap();
        assert!(matches!(circular, MovementPattern::Circular { radius_m: 10.0, period_s: 30.0 }));

        // Test random
        let random = "random:2.0".parse::<MovementPattern>().unwrap();
        assert!(matches!(random, MovementPattern::Random { max_speed_m_per_s: 2.0 }));

        // Test invalid patterns
        assert!("invalid".parse::<MovementPattern>().is_err());
        assert!("linear:invalid:45.0".parse::<MovementPattern>().is_err());
        assert!("linear:-1.0:45.0".parse::<MovementPattern>().is_err());
        assert!("linear:1.0:400.0".parse::<MovementPattern>().is_err());
    }

    #[test]
    fn test_movement_pattern_display() {
        let stationary = MovementPattern::Stationary;
        assert_eq!(stationary.to_string(), "stationary");

        let linear = MovementPattern::Linear { speed_m_per_s: 1.5, bearing_deg: 45.0 };
        assert_eq!(linear.to_string(), "linear:1.5:45");

        let circular = MovementPattern::Circular { radius_m: 10.0, period_s: 30.0 };
        assert_eq!(circular.to_string(), "circular:10:30");

        let random = MovementPattern::Random { max_speed_m_per_s: 2.0 };
        assert_eq!(random.to_string(), "random:2");
    }

    #[test]
    fn test_scenario_type_parsing() {
        assert!(matches!("triangle".parse::<ScenarioType>().unwrap(), ScenarioType::Triangle));
        assert!(matches!("square".parse::<ScenarioType>().unwrap(), ScenarioType::Square));
        assert!(matches!("line".parse::<ScenarioType>().unwrap(), ScenarioType::Line));
        assert!(matches!("grid".parse::<ScenarioType>().unwrap(), ScenarioType::Grid));
        assert!("invalid".parse::<ScenarioType>().is_err());
    }

    #[test]
    fn test_export_format_parsing() {
        assert!(matches!("json".parse::<ExportFormat>().unwrap(), ExportFormat::Json));
        assert!(matches!("csv".parse::<ExportFormat>().unwrap(), ExportFormat::Csv));
        assert!("invalid".parse::<ExportFormat>().is_err());
    }

    #[test]
    fn test_virtual_beacon_stats_serialization() {
        let mut stats = VirtualBeaconStats::new();
        stats.messages_sent = 42;
        stats.last_transmission = Some(SystemTime::now());
        stats.uptime = Duration::from_secs(3600);
        stats.transmission_failures = 1;

        // Test serialization
        let json = serde_json::to_string(&stats).unwrap();
        assert!(json.contains("\"messages_sent\":42"));
        assert!(json.contains("\"transmission_failures\":1"));

        // Test deserialization
        let deserialized: VirtualBeaconStats = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized.messages_sent, 42);
        assert_eq!(deserialized.transmission_failures, 1);
        assert_eq!(deserialized.uptime, Duration::from_secs(3600));
        assert!(deserialized.last_transmission.is_some());
    }

    #[test]
    fn test_virtual_message_serialization() {
        let message = VirtualMessage {
            beacon_id: Uuid::new_v4(),
            timestamp: SystemTime::now(),
            position: GeodeticPosition {
                latitude: 32.123,
                longitude: 45.476,
                depth: 10.0,
            },
            message_data: vec![1, 2, 3, 4],
            signal_quality: 255,
        };

        // Test serialization
        let json = serde_json::to_string(&message).unwrap();
        assert!(json.contains("\"latitude\":32.123"));
        assert!(json.contains("\"signal_quality\":255"));

        // Test deserialization
        let deserialized: VirtualMessage = serde_json::from_str(&json).unwrap();
        assert_eq!(deserialized.beacon_id, message.beacon_id);
        assert_eq!(deserialized.position.latitude, 32.123);
        assert_eq!(deserialized.signal_quality, 255);
        assert_eq!(deserialized.message_data, vec![1, 2, 3, 4]);
    }

    #[test]
    fn test_movement_pattern_serialization() {
        let patterns = vec![
            MovementPattern::Stationary,
            MovementPattern::Linear { speed_m_per_s: 1.5, bearing_deg: 45.0 },
            MovementPattern::Circular { radius_m: 10.0, period_s: 30.0 },
            MovementPattern::Random { max_speed_m_per_s: 2.0 },
        ];

        for pattern in patterns {
            let json = serde_json::to_string(&pattern).unwrap();
            let deserialized: MovementPattern = serde_json::from_str(&json).unwrap();
            
            // Compare using Debug format since we don't have PartialEq
            assert_eq!(format!("{:?}", pattern), format!("{:?}", deserialized));
        }
    }
}