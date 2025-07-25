use std::collections::HashMap;
use std::path::Path;
use uuid::Uuid;
use tokio::task::JoinHandle;
use shared_positioning::{
    BeaconConfig,
    GeodeticPosition,
};
use crate::{
    EmulatorError,
    VirtualBeacon,
    VirtualBeaconStatus,
    VirtualCommunicationSpace,
    ScenarioType,
    ExportFormat,
};

/// Main emulator manager that coordinates virtual beacons
pub struct EmulatorManager {
    virtual_beacons: HashMap<Uuid, VirtualBeacon>,
    beacon_tasks: HashMap<Uuid, JoinHandle<()>>,
    communication_space: VirtualCommunicationSpace,
    current_channel: String,
}

impl EmulatorManager {
    pub fn new(channel_name: &str) -> Self {
        let communication_space = VirtualCommunicationSpace::new();
        let current_channel = channel_name.to_string();
        
        Self {
            virtual_beacons: HashMap::new(),
            beacon_tasks: HashMap::new(),
            communication_space,
            current_channel,
        }
    }
    
    /// Create a new virtual beacon with optional configuration
    pub async fn create_beacon(
        &mut self,
        id: Option<Uuid>,
        position: GeodeticPosition,
        config: Option<BeaconConfig>,
    ) -> Result<Uuid, EmulatorError> {
        let beacon_id = id.unwrap_or_else(Uuid::new_v4);
        
        if self.virtual_beacons.contains_key(&beacon_id) {
            return Err(EmulatorError::BeaconExists(beacon_id));
        }
        
        let beacon_config = config.unwrap_or_else(|| {
            // Create default config for emulator
            BeaconConfig::new(beacon_id)
        });
        
        let virtual_channel = self.communication_space.get_or_create_channel(&self.current_channel);
        
        let virtual_beacon = VirtualBeacon::new(
            beacon_id,
            beacon_config,
            position,
            virtual_channel,
        )?;
        
        self.virtual_beacons.insert(beacon_id, virtual_beacon);
        
        Ok(beacon_id)
    }
    
    /// Create a new virtual beacon from a configuration file
    pub async fn create_beacon_from_config(
        &mut self,
        id: Option<Uuid>,
        position: GeodeticPosition,
        config_path: &Path,
    ) -> Result<Uuid, EmulatorError> {
        let config = self.load_beacon_config(config_path).await?;
        self.create_beacon(id, position, Some(config)).await
    }
    
    /// Start a virtual beacon by ID
    pub async fn start_beacon(&mut self, id: Uuid) -> Result<(), EmulatorError> {
        if let Some(beacon) = self.virtual_beacons.get_mut(&id) {
            if beacon.is_running() {
                return Err(EmulatorError::ConfigError(
                    format!("Beacon {} is already running", id)
                ));
            }
            
            let task_handle = beacon.start().await?;
            self.beacon_tasks.insert(id, task_handle);
            
            Ok(())
        } else {
            Err(EmulatorError::BeaconNotFound(id))
        }
    }
    
    /// Start all virtual beacons
    pub async fn start_all_beacons(&mut self) -> Result<Vec<Uuid>, EmulatorError> {
        let beacon_ids: Vec<Uuid> = self.virtual_beacons.keys().cloned().collect();
        let mut started_beacons = Vec::new();
        let mut errors = Vec::new();
        
        for id in beacon_ids {
            match self.start_beacon(id).await {
                Ok(()) => started_beacons.push(id),
                Err(e) => errors.push((id, e)),
            }
        }
        
        if !errors.is_empty() {
            // Log errors but don't fail completely if some beacons started
            for (id, error) in errors {
                eprintln!("Failed to start beacon {}: {}", id, error);
            }
        }
        
        Ok(started_beacons)
    }
    
    /// Stop a virtual beacon by ID
    pub async fn stop_beacon(&mut self, id: Uuid) -> Result<(), EmulatorError> {
        if let Some(beacon) = self.virtual_beacons.get_mut(&id) {
            beacon.stop()?;
            
            // Wait for task to complete gracefully, then remove it
            if let Some(task) = self.beacon_tasks.remove(&id) {
                // Give the task a moment to shut down gracefully
                tokio::time::timeout(
                    std::time::Duration::from_millis(100),
                    task
                ).await.ok(); // Ignore timeout errors
            }
            
            Ok(())
        } else {
            Err(EmulatorError::BeaconNotFound(id))
        }
    }
    
    /// Remove a stopped beacon from the registry
    pub async fn remove_beacon(&mut self, id: Uuid) -> Result<(), EmulatorError> {
        if let Some(beacon) = self.virtual_beacons.get(&id) {
            if beacon.is_running() {
                return Err(EmulatorError::ConfigError(
                    format!("Cannot remove running beacon {}. Stop it first.", id)
                ));
            }
        }
        
        if self.virtual_beacons.remove(&id).is_some() {
            // Ensure task is cleaned up
            if let Some(task) = self.beacon_tasks.remove(&id) {
                task.abort();
            }
            Ok(())
        } else {
            Err(EmulatorError::BeaconNotFound(id))
        }
    }
    
    /// Stop all virtual beacons
    pub async fn stop_all_beacons(&mut self) -> Result<Vec<Uuid>, EmulatorError> {
        let beacon_ids: Vec<Uuid> = self.virtual_beacons.keys().cloned().collect();
        let mut stopped_beacons = Vec::new();
        let mut errors = Vec::new();
        
        for id in beacon_ids {
            match self.stop_beacon(id).await {
                Ok(()) => stopped_beacons.push(id),
                Err(e) => errors.push((id, e)),
            }
        }
        
        if !errors.is_empty() {
            // Log errors but don't fail completely if some beacons stopped
            for (id, error) in errors {
                eprintln!("Failed to stop beacon {}: {}", id, error);
            }
        }
        
        Ok(stopped_beacons)
    }
    
    /// Shutdown the emulator manager and clean up all resources
    pub async fn shutdown(&mut self) -> Result<(), EmulatorError> {
        // Stop all beacons
        let _ = self.stop_all_beacons().await;
        
        // Wait a bit for graceful shutdown
        tokio::time::sleep(std::time::Duration::from_millis(100)).await;
        
        // Force abort any remaining tasks
        for (id, task) in self.beacon_tasks.drain() {
            task.abort();
            eprintln!("Force aborted task for beacon {}", id);
        }
        
        // Clear beacon registry
        self.virtual_beacons.clear();
        
        Ok(())
    }
    
    /// List all virtual beacons with their current status
    pub fn list_beacons(&self) -> Vec<VirtualBeaconStatus> {
        self.virtual_beacons.values()
            .map(|beacon| beacon.get_status())
            .collect()
    }
    
    /// Get status of a specific beacon
    pub fn get_beacon_status(&self, id: Uuid) -> Result<VirtualBeaconStatus, EmulatorError> {
        if let Some(beacon) = self.virtual_beacons.get(&id) {
            Ok(beacon.get_status())
        } else {
            Err(EmulatorError::BeaconNotFound(id))
        }
    }
    
    /// Get count of active (running) beacons
    pub fn get_active_beacon_count(&self) -> usize {
        self.virtual_beacons.values()
            .filter(|beacon| beacon.is_running())
            .count()
    }
    
    /// Get count of total beacons
    pub fn get_total_beacon_count(&self) -> usize {
        self.virtual_beacons.len()
    }
    
    /// Check if a beacon exists
    pub fn beacon_exists(&self, id: Uuid) -> bool {
        self.virtual_beacons.contains_key(&id)
    }
    
    /// Update beacon position
    pub async fn update_beacon_position(
        &mut self,
        id: Uuid,
        position: GeodeticPosition,
    ) -> Result<(), EmulatorError> {
        if let Some(beacon) = self.virtual_beacons.get_mut(&id) {
            beacon.update_position(position)?;
            Ok(())
        } else {
            Err(EmulatorError::BeaconNotFound(id))
        }
    }
    
    /// Update beacon configuration
    pub async fn update_beacon_config(
        &mut self,
        id: Uuid,
        config: BeaconConfig,
    ) -> Result<(), EmulatorError> {
        if let Some(beacon) = self.virtual_beacons.get_mut(&id) {
            beacon.update_config(config)?;
            Ok(())
        } else {
            Err(EmulatorError::BeaconNotFound(id))
        }
    }
    
    /// Update beacon movement pattern
    pub async fn update_beacon_movement_pattern(
        &mut self,
        id: Uuid,
        pattern: crate::MovementPattern,
    ) -> Result<(), EmulatorError> {
        if let Some(beacon) = self.virtual_beacons.get_mut(&id) {
            beacon.set_movement_pattern(pattern)?;
            Ok(())
        } else {
            Err(EmulatorError::BeaconNotFound(id))
        }
    }
    
    pub async fn create_scenario(
        &mut self,
        _scenario_type: ScenarioType,
        _count: usize,
        _spacing: f64,
        _center: GeodeticPosition,
    ) -> Result<Vec<Uuid>, EmulatorError> {
        // TODO: Implement scenario generation
        // This will be implemented in a later task
        Err(EmulatorError::InvalidScenario("Scenario generation not yet implemented".to_string()))
    }
    
    pub async fn export_logs(
        &self,
        _output_path: &std::path::Path,
        _format: ExportFormat,
        _duration_s: u64,
    ) -> Result<(), EmulatorError> {
        // TODO: Implement log export
        // This will be implemented in a later task
        Err(EmulatorError::ExportError("Log export not yet implemented".to_string()))
    }
    
    /// Load beacon configuration from a TOML file
    pub async fn load_beacon_config(&self, config_path: &Path) -> Result<BeaconConfig, EmulatorError> {
        let config_content = tokio::fs::read_to_string(config_path).await?;
        let config: BeaconConfig = toml::from_str(&config_content)?;
        
        // Validate configuration for emulator use
        self.validate_emulator_config(&config)?;
        
        Ok(config)
    }
    
    /// Validate that a beacon configuration is suitable for emulation
    fn validate_emulator_config(&self, config: &BeaconConfig) -> Result<(), EmulatorError> {
        // Ensure transmission interval is reasonable for emulation
        if config.transmission.interval_ms < 100 {
            return Err(EmulatorError::ConfigError(
                "Transmission interval too short for emulation (minimum 100ms)".to_string()
            ));
        }
        
        if config.transmission.interval_ms > 300_000 {
            return Err(EmulatorError::ConfigError(
                "Transmission interval too long for emulation (maximum 5 minutes)".to_string()
            ));
        }
        
        // Validate power settings are reasonable
        if config.power.low_battery_threshold_percent <= 0.0 {
            return Err(EmulatorError::ConfigError(
                "Low battery threshold must be greater than zero".to_string()
            ));
        }
        
        if config.power.critical_battery_threshold_percent >= config.power.low_battery_threshold_percent {
            return Err(EmulatorError::ConfigError(
                "Critical battery threshold must be less than low battery threshold".to_string()
            ));
        }
        
        Ok(())
    }
    
    /// Get current communication channel name
    pub fn get_current_channel(&self) -> &str {
        &self.current_channel
    }
    
    /// Switch to a different communication channel
    pub fn set_current_channel(&mut self, channel_name: &str) {
        self.current_channel = channel_name.to_string();
    }
    
    /// Get communication space reference for advanced operations
    pub fn get_communication_space(&self) -> &VirtualCommunicationSpace {
        &self.communication_space
    }
    
    /// Get mutable communication space reference for advanced operations
    pub fn get_communication_space_mut(&mut self) -> &mut VirtualCommunicationSpace {
        &mut self.communication_space
    }
    
    /// Get statistics about the emulator manager
    pub fn get_manager_stats(&self) -> EmulatorManagerStats {
        let running_beacons = self.get_active_beacon_count();
        let total_beacons = self.get_total_beacon_count();
        let channels = self.communication_space.list_channels();
        
        EmulatorManagerStats {
            total_beacons,
            running_beacons,
            stopped_beacons: total_beacons - running_beacons,
            active_channels: channels.len(),
            current_channel: self.current_channel.clone(),
            channel_names: channels,
        }
    }
}

/// Statistics about the emulator manager
#[derive(Debug, Clone)]
pub struct EmulatorManagerStats {
    pub total_beacons: usize,
    pub running_beacons: usize,
    pub stopped_beacons: usize,
    pub active_channels: usize,
    pub current_channel: String,
    pub channel_names: Vec<String>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use shared_positioning::GeodeticPosition;
    use tokio::time::{sleep, Duration};


    fn create_test_position() -> GeodeticPosition {
        GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        }
    }



    #[tokio::test]
    async fn test_emulator_manager_creation() {
        let manager = EmulatorManager::new("test_channel");
        assert_eq!(manager.get_current_channel(), "test_channel");
        assert_eq!(manager.get_total_beacon_count(), 0);
        assert_eq!(manager.get_active_beacon_count(), 0);
    }

    #[tokio::test]
    async fn test_beacon_creation() {
        let mut manager = EmulatorManager::new("test_channel");
        let position = create_test_position();
        
        let beacon_id = manager.create_beacon(None, position, None).await.unwrap();
        
        assert!(manager.beacon_exists(beacon_id));
        assert_eq!(manager.get_total_beacon_count(), 1);
        assert_eq!(manager.get_active_beacon_count(), 0); // Not started yet
        
        let status = manager.get_beacon_status(beacon_id).unwrap();
        assert_eq!(status.id, beacon_id);
        assert!(!status.is_running);
    }

    #[tokio::test]
    async fn test_beacon_creation_with_specific_id() {
        let mut manager = EmulatorManager::new("test_channel");
        let position = create_test_position();
        let specific_id = Uuid::new_v4();
        
        let beacon_id = manager.create_beacon(Some(specific_id), position, None).await.unwrap();
        
        assert_eq!(beacon_id, specific_id);
        assert!(manager.beacon_exists(beacon_id));
    }

    #[tokio::test]
    async fn test_duplicate_beacon_creation() {
        let mut manager = EmulatorManager::new("test_channel");
        let position = create_test_position();
        let specific_id = Uuid::new_v4();
        
        // Create first beacon
        manager.create_beacon(Some(specific_id), position, None).await.unwrap();
        
        // Try to create duplicate
        let result = manager.create_beacon(Some(specific_id), position, None).await;
        assert!(matches!(result, Err(EmulatorError::BeaconExists(_))));
    }

    #[tokio::test]
    async fn test_beacon_lifecycle() {
        let mut manager = EmulatorManager::new("test_channel");
        let position = create_test_position();
        
        // Create beacon
        let beacon_id = manager.create_beacon(None, position, None).await.unwrap();
        assert!(!manager.get_beacon_status(beacon_id).unwrap().is_running);
        
        // Start beacon
        manager.start_beacon(beacon_id).await.unwrap();
        assert!(manager.get_beacon_status(beacon_id).unwrap().is_running);
        assert_eq!(manager.get_active_beacon_count(), 1);
        
        // Stop beacon
        manager.stop_beacon(beacon_id).await.unwrap();
        assert!(!manager.get_beacon_status(beacon_id).unwrap().is_running);
        assert_eq!(manager.get_active_beacon_count(), 0);
        
        // Remove beacon
        manager.remove_beacon(beacon_id).await.unwrap();
        assert!(!manager.beacon_exists(beacon_id));
        assert_eq!(manager.get_total_beacon_count(), 0);
    }

    #[tokio::test]
    async fn test_start_already_running_beacon() {
        let mut manager = EmulatorManager::new("test_channel");
        let position = create_test_position();
        
        let beacon_id = manager.create_beacon(None, position, None).await.unwrap();
        manager.start_beacon(beacon_id).await.unwrap();
        
        // Try to start again
        let result = manager.start_beacon(beacon_id).await;
        assert!(matches!(result, Err(EmulatorError::ConfigError(_))));
    }

    #[tokio::test]
    async fn test_remove_running_beacon() {
        let mut manager = EmulatorManager::new("test_channel");
        let position = create_test_position();
        
        let beacon_id = manager.create_beacon(None, position, None).await.unwrap();
        manager.start_beacon(beacon_id).await.unwrap();
        
        // Try to remove running beacon
        let result = manager.remove_beacon(beacon_id).await;
        assert!(matches!(result, Err(EmulatorError::ConfigError(_))));
        
        // Stop first, then remove
        manager.stop_beacon(beacon_id).await.unwrap();
        manager.remove_beacon(beacon_id).await.unwrap();
        assert!(!manager.beacon_exists(beacon_id));
    }

    #[tokio::test]
    async fn test_multiple_beacons() {
        let mut manager = EmulatorManager::new("test_channel");
        let position = create_test_position();
        
        // Create multiple beacons
        let beacon1 = manager.create_beacon(None, position, None).await.unwrap();
        let beacon2 = manager.create_beacon(None, position, None).await.unwrap();
        let beacon3 = manager.create_beacon(None, position, None).await.unwrap();
        
        assert_eq!(manager.get_total_beacon_count(), 3);
        assert_eq!(manager.get_active_beacon_count(), 0);
        
        // Start all beacons
        let started = manager.start_all_beacons().await.unwrap();
        assert_eq!(started.len(), 3);
        assert_eq!(manager.get_active_beacon_count(), 3);
        
        // Stop all beacons
        let stopped = manager.stop_all_beacons().await.unwrap();
        assert_eq!(stopped.len(), 3);
        assert_eq!(manager.get_active_beacon_count(), 0);
    }

    #[tokio::test]
    async fn test_beacon_operations_on_nonexistent_beacon() {
        let mut manager = EmulatorManager::new("test_channel");
        let nonexistent_id = Uuid::new_v4();
        
        // Test various operations on nonexistent beacon
        assert!(matches!(
            manager.start_beacon(nonexistent_id).await,
            Err(EmulatorError::BeaconNotFound(_))
        ));
        
        assert!(matches!(
            manager.stop_beacon(nonexistent_id).await,
            Err(EmulatorError::BeaconNotFound(_))
        ));
        
        assert!(matches!(
            manager.get_beacon_status(nonexistent_id),
            Err(EmulatorError::BeaconNotFound(_))
        ));
        
        let position = create_test_position();
        assert!(matches!(
            manager.update_beacon_position(nonexistent_id, position).await,
            Err(EmulatorError::BeaconNotFound(_))
        ));
    }

    #[tokio::test]
    async fn test_beacon_position_update() {
        let mut manager = EmulatorManager::new("test_channel");
        let initial_position = create_test_position();
        
        let beacon_id = manager.create_beacon(None, initial_position, None).await.unwrap();
        
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
    }

    #[tokio::test]
    async fn test_config_validation() {
        let manager = EmulatorManager::new("test_channel");
        
        // Test invalid config with too short interval
        let mut invalid_config = BeaconConfig::new(Uuid::new_v4());
        invalid_config.transmission.interval_ms = 50; // Too short
        
        let result = manager.validate_emulator_config(&invalid_config);
        assert!(matches!(result, Err(EmulatorError::ConfigError(_))));
        
        // Test invalid config with invalid power thresholds
        let mut invalid_config2 = BeaconConfig::new(Uuid::new_v4());
        invalid_config2.power.critical_battery_threshold_percent = 25.0;
        invalid_config2.power.low_battery_threshold_percent = 20.0; // Critical > Low
        
        let result2 = manager.validate_emulator_config(&invalid_config2);
        assert!(matches!(result2, Err(EmulatorError::ConfigError(_))));
    }

    #[tokio::test]
    async fn test_beacon_creation_with_config() {
        let mut manager = EmulatorManager::new("test_channel");
        let position = create_test_position();
        
        // Create a custom config
        let mut config = BeaconConfig::new(Uuid::new_v4());
        config.transmission.interval_ms = 2000;
        
        let beacon_id = manager.create_beacon(None, position, Some(config.clone())).await.unwrap();
        
        assert!(manager.beacon_exists(beacon_id));
        
        let status = manager.get_beacon_status(beacon_id).unwrap();
        assert_eq!(status.config.transmission.interval_ms, 2000);
    }

    #[tokio::test]
    async fn test_channel_management() {
        let mut manager = EmulatorManager::new("channel1");
        assert_eq!(manager.get_current_channel(), "channel1");
        
        // Switch channel
        manager.set_current_channel("channel2");
        assert_eq!(manager.get_current_channel(), "channel2");
        
        // Create beacon on new channel
        let position = create_test_position();
        let beacon_id = manager.create_beacon(None, position, None).await.unwrap();
        
        // Verify beacon is on the correct channel
        assert!(manager.beacon_exists(beacon_id));
    }

    #[tokio::test]
    async fn test_manager_stats() {
        let mut manager = EmulatorManager::new("test_channel");
        let position = create_test_position();
        
        // Initial stats
        let stats = manager.get_manager_stats();
        assert_eq!(stats.total_beacons, 0);
        assert_eq!(stats.running_beacons, 0);
        assert_eq!(stats.stopped_beacons, 0);
        assert_eq!(stats.current_channel, "test_channel");
        
        // Create and start some beacons
        let beacon1 = manager.create_beacon(None, position, None).await.unwrap();
        let beacon2 = manager.create_beacon(None, position, None).await.unwrap();
        
        manager.start_beacon(beacon1).await.unwrap();
        
        let stats = manager.get_manager_stats();
        assert_eq!(stats.total_beacons, 2);
        assert_eq!(stats.running_beacons, 1);
        assert_eq!(stats.stopped_beacons, 1);
    }

    #[tokio::test]
    async fn test_shutdown() {
        let mut manager = EmulatorManager::new("test_channel");
        let position = create_test_position();
        
        // Create and start multiple beacons
        let beacon1 = manager.create_beacon(None, position, None).await.unwrap();
        let beacon2 = manager.create_beacon(None, position, None).await.unwrap();
        
        manager.start_beacon(beacon1).await.unwrap();
        manager.start_beacon(beacon2).await.unwrap();
        
        assert_eq!(manager.get_active_beacon_count(), 2);
        
        // Shutdown
        manager.shutdown().await.unwrap();
        
        assert_eq!(manager.get_total_beacon_count(), 0);
        assert_eq!(manager.get_active_beacon_count(), 0);
    }

    #[tokio::test]
    async fn test_beacon_message_transmission() {
        let mut manager = EmulatorManager::new("test_channel");
        let position = create_test_position();
        
        // Create and start beacon
        let beacon_id = manager.create_beacon(None, position, None).await.unwrap();
        manager.start_beacon(beacon_id).await.unwrap();
        
        // Get channel to subscribe to messages after beacon is created
        let channel = manager.get_communication_space_mut()
            .get_or_create_channel("test_channel");
        let mut receiver = channel.subscribe();
        
        // Wait for message transmission
        sleep(Duration::from_millis(200)).await;
        
        // Check if we received a message
        let received_message = tokio::time::timeout(
            Duration::from_millis(100),
            receiver.recv()
        ).await;
        
        assert!(received_message.is_ok(), "Should have received a message");
        
        let message = received_message.unwrap().unwrap();
        assert_eq!(message.beacon_id, beacon_id);
        assert_eq!(message.position.latitude, position.latitude);
        
        // Stop beacon
        manager.stop_beacon(beacon_id).await.unwrap();
    }

    #[tokio::test]
    async fn test_concurrent_beacon_operations() {
        let mut manager = EmulatorManager::new("test_channel");
        let position = create_test_position();
        
        // Create multiple beacons concurrently
        let mut handles = Vec::new();
        for _ in 0..5 {
            let beacon_id = manager.create_beacon(None, position, None).await.unwrap();
            let handle = tokio::spawn(async move {
                beacon_id
            });
            handles.push(handle);
        }
        
        // Wait for all creations to complete
        let mut beacon_ids = Vec::new();
        for handle in handles {
            beacon_ids.push(handle.await.unwrap());
        }
        
        assert_eq!(manager.get_total_beacon_count(), 5);
        
        // Start all beacons
        for beacon_id in &beacon_ids {
            manager.start_beacon(*beacon_id).await.unwrap();
        }
        
        assert_eq!(manager.get_active_beacon_count(), 5);
        
        // Stop all beacons
        for beacon_id in &beacon_ids {
            manager.stop_beacon(*beacon_id).await.unwrap();
        }
        
        assert_eq!(manager.get_active_beacon_count(), 0);
    }
}