#[cfg(test)]
mod tests {
    use super::*;
    use crate::{EmulatorManager, ScenarioType};
    use shared_positioning::GeodeticPosition;
    use tokio_test;

    #[tokio::test]
    async fn test_create_triangle_scenario() {
        let mut manager = EmulatorManager::new("test_channel");
        
        let center = GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        };
        
        let beacon_ids = manager.create_scenario(
            ScenarioType::Triangle,
            3,
            100.0,
            center,
        ).await.unwrap();
        
        assert_eq!(beacon_ids.len(), 3);
        
        // Verify beacons were created
        let beacons = manager.list_beacons();
        assert_eq!(beacons.len(), 3);
        
        // Verify all beacons are at the same depth
        for beacon in &beacons {
            assert_eq!(beacon.position.depth, 10.0);
        }
    }
    
    #[tokio::test]
    async fn test_create_square_scenario() {
        let mut manager = EmulatorManager::new("test_channel");
        
        let center = GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 15.0,
        };
        
        let beacon_ids = manager.create_scenario(
            ScenarioType::Square,
            4,
            200.0,
            center,
        ).await.unwrap();
        
        assert_eq!(beacon_ids.len(), 4);
        
        // Verify beacons were created
        let beacons = manager.list_beacons();
        assert_eq!(beacons.len(), 4);
        
        // Verify all beacons are at the same depth
        for beacon in &beacons {
            assert_eq!(beacon.position.depth, 15.0);
        }
    }
    
    #[tokio::test]
    async fn test_create_line_scenario() {
        let mut manager = EmulatorManager::new("test_channel");
        
        let center = GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 5.0,
        };
        
        let beacon_ids = manager.create_scenario(
            ScenarioType::Line,
            5,
            50.0,
            center,
        ).await.unwrap();
        
        assert_eq!(beacon_ids.len(), 5);
        
        // Verify beacons were created
        let beacons = manager.list_beacons();
        assert_eq!(beacons.len(), 5);
        
        // Verify all beacons have the same longitude and depth
        for beacon in &beacons {
            assert_eq!(beacon.position.longitude, center.longitude);
            assert_eq!(beacon.position.depth, 5.0);
        }
    }
    
    #[tokio::test]
    async fn test_create_grid_scenario() {
        let mut manager = EmulatorManager::new("test_channel");
        
        let center = GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 20.0,
        };
        
        let beacon_ids = manager.create_scenario(
            ScenarioType::Grid,
            9, // 3x3 grid
            75.0,
            center,
        ).await.unwrap();
        
        assert_eq!(beacon_ids.len(), 9);
        
        // Verify beacons were created
        let beacons = manager.list_beacons();
        assert_eq!(beacons.len(), 9);
        
        // Verify all beacons are at the same depth
        for beacon in &beacons {
            assert_eq!(beacon.position.depth, 20.0);
        }
    }
    
    #[tokio::test]
    async fn test_scenario_validation_errors() {
        let mut manager = EmulatorManager::new("test_channel");
        
        let center = GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        };
        
        // Triangle with wrong count
        let result = manager.create_scenario(
            ScenarioType::Triangle,
            4, // Should be 3
            100.0,
            center,
        ).await;
        assert!(result.is_err());
        
        // Square with wrong count
        let result = manager.create_scenario(
            ScenarioType::Square,
            3, // Should be 4
            100.0,
            center,
        ).await;
        assert!(result.is_err());
        
        // Line with too few beacons
        let result = manager.create_scenario(
            ScenarioType::Line,
            1, // Should be at least 2
            100.0,
            center,
        ).await;
        assert!(result.is_err());
        
        // Grid with too few beacons
        let result = manager.create_scenario(
            ScenarioType::Grid,
            3, // Should be at least 4
            100.0,
            center,
        ).await;
        assert!(result.is_err());
        
        // Invalid spacing
        let result = manager.create_scenario(
            ScenarioType::Triangle,
            3,
            0.0, // Invalid spacing
            center,
        ).await;
        assert!(result.is_err());
    }
}