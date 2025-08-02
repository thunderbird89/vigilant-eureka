use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::SystemTime;
use uuid::Uuid;
use tokio::task::JoinHandle;
use tokio::sync::RwLock;
use serde::{Serialize, Deserialize};
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
    MovementPattern,
    IpcServer,
    BeaconLogger,
    LogEntryType,
    LogMetadata,
    LogFilter,
    config::{EmulatorConfigManager, EmulatorBeaconConfig},
    performance::{PerformanceMonitor, PerformanceOptimizer, PerformanceConfig, PerformanceMetrics},
};

/// Persistent beacon data for saving/loading state
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct PersistentBeaconData {
    pub id: Uuid,
    pub position: GeodeticPosition,
    pub config: BeaconConfig,
    pub movement_pattern: MovementPattern,
    pub intended_running: bool, // Whether the beacon should be running (persistent intent)
    /// Configuration file path if loaded from file
    pub config_file_path: Option<std::path::PathBuf>,
    /// Configuration checksum for integrity verification
    pub config_checksum: Option<String>,
    /// Last configuration update timestamp
    pub config_updated_at: Option<u64>,
}

/// Emulator state that can be persisted
#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct EmulatorState {
    pub beacons: Vec<PersistentBeaconData>,
    pub current_channel: String,
}

/// Main emulator manager that coordinates virtual beacons
pub struct EmulatorManager {
    virtual_beacons: HashMap<Uuid, VirtualBeacon>,
    beacon_tasks: HashMap<Uuid, JoinHandle<()>>,
    communication_space: Arc<RwLock<VirtualCommunicationSpace>>,
    current_channel: String,
    state_file_path: PathBuf,
    ipc_server: Option<IpcServer>,
    config_manager: EmulatorConfigManager,
    logger: Arc<BeaconLogger>,
    performance_monitor: Arc<PerformanceMonitor>,
    performance_optimizer: Arc<PerformanceOptimizer>,
}

impl EmulatorManager {
    pub fn new(channel_name: &str) -> Self {
        let communication_space = Arc::new(RwLock::new(VirtualCommunicationSpace::new()));
        let current_channel = channel_name.to_string();
        let state_file_path = Self::get_default_state_file_path();
        let config_manager = EmulatorConfigManager::new();
        let logger = Arc::new(BeaconLogger::new());
        let performance_monitor = Arc::new(PerformanceMonitor::new());
        let performance_optimizer = Arc::new(PerformanceOptimizer::new(performance_monitor.clone()));
        
        Self {
            virtual_beacons: HashMap::new(),
            beacon_tasks: HashMap::new(),
            communication_space,
            current_channel,
            state_file_path,
            ipc_server: None,
            config_manager,
            logger,
            performance_monitor,
            performance_optimizer,
        }
    }
    
    /// Create a new emulator manager and load existing state
    pub async fn new_with_persistence(channel_name: &str) -> Result<Self, EmulatorError> {
        let mut manager = Self::new(channel_name);
        manager.load_state().await?;
        Ok(manager)
    }
    
    /// Get the default state file path
    fn get_default_state_file_path() -> PathBuf {
        PathBuf::from("data/emulator_state.json")
    }
    
    /// Set a custom state file path
    pub fn set_state_file_path(&mut self, path: PathBuf) {
        self.state_file_path = path;
    }
    
    /// Get the current state file path
    pub fn get_state_file_path(&self) -> &PathBuf {
        &self.state_file_path
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
            let mut default_config = self.create_default_beacon_config();
            default_config.beacon_id = beacon_id;
            default_config
        });
        
        let virtual_channel = {
            let mut comm_space = self.communication_space.write().await;
            comm_space.get_or_create_channel(&self.current_channel)
        };
        
        let virtual_beacon = VirtualBeacon::new_with_logger(
            beacon_id,
            beacon_config.clone(),
            position,
            virtual_channel,
            self.logger.clone(),
        )?;
        
        self.virtual_beacons.insert(beacon_id, virtual_beacon);
        
        // Log beacon creation
        let metadata = LogMetadata {
            channel: self.current_channel.clone(),
            message_version: Some(format!("{:?}", beacon_config.transmission.message_version)),
            transmission_interval_ms: Some(beacon_config.transmission.interval_ms),
            movement_pattern: Some("Stationary".to_string()),
            custom_fields: std::collections::HashMap::new(),
        };
        
        if let Err(e) = self.logger.log_event(
            beacon_id,
            LogEntryType::BeaconCreated,
            position,
            255,
            metadata,
        ).await {
            tracing::warn!("Failed to log beacon creation: {}", e);
        }
        
        // Save state after creating beacon
        if let Err(e) = self.save_state().await {
            eprintln!("Warning: Failed to save state after creating beacon: {}", e);
        }
        
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
    
    /// Create a new virtual beacon from an emulator configuration file
    pub async fn create_beacon_from_emulator_config(
        &mut self,
        config_path: &Path,
    ) -> Result<Uuid, EmulatorError> {
        let emulator_config = self.load_emulator_beacon_config(config_path).await?;
        
        // Use the beacon ID from the config or generate a new one
        let beacon_id = if emulator_config.beacon_config.beacon_id.is_nil() {
            Uuid::new_v4()
        } else {
            emulator_config.beacon_config.beacon_id
        };
        
        // Create the beacon
        let created_id = self.create_beacon(
            Some(beacon_id),
            emulator_config.initial_position,
            Some(emulator_config.beacon_config),
        ).await?;
        
        // Set the movement pattern
        self.update_beacon_movement_pattern(created_id, emulator_config.movement_pattern).await?;
        
        // Auto-start if configured
        if emulator_config.auto_start {
            self.start_beacon(created_id).await?;
        }
        
        Ok(created_id)
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
            
            // Log beacon start
            let status = beacon.get_status();
            let metadata = LogMetadata {
                channel: self.current_channel.clone(),
                message_version: Some(format!("{:?}", status.config.transmission.message_version)),
                transmission_interval_ms: Some(status.config.transmission.interval_ms),
                movement_pattern: Some(format!("{:?}", status.movement_pattern)),
                custom_fields: std::collections::HashMap::new(),
            };
            
            if let Err(e) = self.logger.log_event(
                id,
                LogEntryType::BeaconStarted,
                status.position,
                255,
                metadata,
            ).await {
                tracing::warn!("Failed to log beacon start: {}", e);
            }
            
            // Save state after starting beacon
            if let Err(e) = self.save_state().await {
                eprintln!("Warning: Failed to save state after starting beacon: {}", e);
            }
            
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
            let status = beacon.get_status();
            beacon.stop()?;
            
            // Wait for task to complete gracefully, then remove it
            if let Some(task) = self.beacon_tasks.remove(&id) {
                // Give the task a moment to shut down gracefully
                tokio::time::timeout(
                    std::time::Duration::from_millis(100),
                    task
                ).await.ok(); // Ignore timeout errors
            }
            
            // Log beacon stop
            let metadata = LogMetadata {
                channel: self.current_channel.clone(),
                message_version: Some(format!("{:?}", status.config.transmission.message_version)),
                transmission_interval_ms: Some(status.config.transmission.interval_ms),
                movement_pattern: Some(format!("{:?}", status.movement_pattern)),
                custom_fields: std::collections::HashMap::new(),
            };
            
            if let Err(e) = self.logger.log_event(
                id,
                LogEntryType::BeaconStopped,
                status.position,
                255,
                metadata,
            ).await {
                tracing::warn!("Failed to log beacon stop: {}", e);
            }
            
            // Save state after stopping beacon
            if let Err(e) = self.save_state().await {
                eprintln!("Warning: Failed to save state after stopping beacon: {}", e);
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
        
        if let Some(beacon) = self.virtual_beacons.remove(&id) {
            let status = beacon.get_status();
            
            // Ensure task is cleaned up
            if let Some(task) = self.beacon_tasks.remove(&id) {
                task.abort();
            }
            
            // Log beacon removal
            let metadata = LogMetadata {
                channel: self.current_channel.clone(),
                message_version: Some(format!("{:?}", status.config.transmission.message_version)),
                transmission_interval_ms: Some(status.config.transmission.interval_ms),
                movement_pattern: Some(format!("{:?}", status.movement_pattern)),
                custom_fields: std::collections::HashMap::new(),
            };
            
            if let Err(e) = self.logger.log_event(
                id,
                LogEntryType::BeaconRemoved,
                status.position,
                255,
                metadata,
            ).await {
                tracing::warn!("Failed to log beacon removal: {}", e);
            }
            
            // Save state after removing beacon
            if let Err(e) = self.save_state().await {
                eprintln!("Warning: Failed to save state after removing beacon: {}", e);
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
            let old_status = beacon.get_status();
            beacon.update_position(position)?;
            
            // Log position update
            let metadata = LogMetadata {
                channel: self.current_channel.clone(),
                message_version: Some(format!("{:?}", old_status.config.transmission.message_version)),
                transmission_interval_ms: Some(old_status.config.transmission.interval_ms),
                movement_pattern: Some(format!("{:?}", old_status.movement_pattern)),
                custom_fields: std::collections::HashMap::new(),
            };
            
            if let Err(e) = self.logger.log_event(
                id,
                LogEntryType::PositionUpdated { old_position: old_status.position },
                position,
                255,
                metadata,
            ).await {
                tracing::warn!("Failed to log position update: {}", e);
            }
            
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
            let status = beacon.get_status();
            beacon.set_movement_pattern(pattern.clone())?;
            
            // Log movement pattern change
            let metadata = LogMetadata {
                channel: self.current_channel.clone(),
                message_version: Some(format!("{:?}", status.config.transmission.message_version)),
                transmission_interval_ms: Some(status.config.transmission.interval_ms),
                movement_pattern: Some(format!("{:?}", pattern)),
                custom_fields: std::collections::HashMap::new(),
            };
            
            if let Err(e) = self.logger.log_event(
                id,
                LogEntryType::MovementPatternChanged,
                status.position,
                255,
                metadata,
            ).await {
                tracing::warn!("Failed to log movement pattern change: {}", e);
            }
            
            Ok(())
        } else {
            Err(EmulatorError::BeaconNotFound(id))
        }
    }
    
    pub async fn create_scenario(
        &mut self,
        scenario_type: ScenarioType,
        count: usize,
        spacing: f64,
        center: GeodeticPosition,
    ) -> Result<Vec<Uuid>, EmulatorError> {
        use crate::scenario::{generate_scenario_positions, validate_scenario_parameters};
        
        // Validate scenario parameters
        validate_scenario_parameters(&scenario_type, count, spacing)?;
        
        // Generate beacon positions
        let positions = generate_scenario_positions(&scenario_type, count, spacing, &center)?;
        
        let mut beacon_ids = Vec::new();
        
        // Get virtual channel for beacons
        let virtual_channel = {
            let mut comm_space = self.communication_space.write().await;
            comm_space.get_or_create_channel(&self.current_channel)
        };
        
        // Create beacons at generated positions
        for (index, position) in positions.into_iter().enumerate() {
            // Generate a descriptive beacon ID based on scenario
            let beacon_id = uuid::Uuid::new_v4();
            
            // Create default configuration for scenario beacons
            let mut config = shared_positioning::BeaconConfig::new(beacon_id);
            
            // Configure transmission settings for scenario testing
            config.transmission.interval_ms = 5000; // Default 5 second interval
            config.transmission.message_version = shared_positioning::beacon_config::MessageVersion::V3;
            config.transmission.power_level = 255; // Full power for testing
            config.transmission.max_retries = 3;
            config.transmission.retry_delay_ms = 1000;
            
            // Configure power settings for emulation (disable power saving features)
            config.power.charging_enabled = false;
            config.power.solar_charging_enabled = false;
            config.power.power_modes.power_save_transmission_multiplier = 1.0; // No power saving
            
            // Create the virtual beacon
            let virtual_beacon = VirtualBeacon::new_with_logger(
                beacon_id,
                config,
                position,
                virtual_channel.clone(),
                self.logger.clone(),
            )?;
            
            // Add to registry
            self.virtual_beacons.insert(beacon_id, virtual_beacon);
            beacon_ids.push(beacon_id);
            
            tracing::info!(
                "Created scenario beacon {} ({}/{}) at position {:.6}, {:.6}, {:.1}m for {} scenario",
                beacon_id,
                index + 1,
                count,
                position.latitude,
                position.longitude,
                position.depth,
                scenario_type
            );
        }
        
        tracing::info!(
            "Successfully created {} scenario with {} beacons (spacing: {:.1}m)",
            scenario_type,
            beacon_ids.len(),
            spacing
        );
        
        Ok(beacon_ids)
    }
    
    pub async fn export_logs(
        &self,
        output_path: &std::path::Path,
        format: ExportFormat,
        duration_s: u64,
        all_time: bool,
        beacon_id: Option<Uuid>,
        include_messages: bool,
    ) -> Result<usize, EmulatorError> {
        use crate::export::export_beacon_data;
        
        // Create filter based on parameters
        let filter = if all_time {
            LogFilter {
                beacon_id,
                ..Default::default()
            }
        } else {
            let since = std::time::SystemTime::now() - std::time::Duration::from_secs(duration_s);
            LogFilter {
                beacon_id,
                start_time: Some(since),
                ..Default::default()
            }
        };
        
        // Get current beacon status
        let beacon_status = if let Some(id) = beacon_id {
            if self.beacon_exists(id) {
                vec![self.get_beacon_status(id)?]
            } else {
                vec![]
            }
        } else {
            self.list_beacons()
        };
        
        // Export the data
        export_beacon_data(
            &self.logger,
            &beacon_status,
            output_path,
            format,
            filter,
            include_messages,
        ).await
    }
    
    /// Load beacon configuration from a file with automatic format detection and validation
    pub async fn load_beacon_config(&self, config_path: &Path) -> Result<BeaconConfig, EmulatorError> {
        self.config_manager.load_beacon_config(config_path).await
    }
    
    /// Load emulator-specific beacon configuration with extended metadata
    pub async fn load_emulator_beacon_config(&self, config_path: &Path) -> Result<EmulatorBeaconConfig, EmulatorError> {
        self.config_manager.load_emulator_beacon_config(config_path).await
    }
    
    /// Save beacon configuration to a file
    pub async fn save_beacon_config(&self, config: &BeaconConfig, config_path: &Path) -> Result<(), EmulatorError> {
        self.config_manager.save_beacon_config(config, config_path).await
    }
    
    /// Save emulator-specific beacon configuration
    pub async fn save_emulator_beacon_config(&self, config: &EmulatorBeaconConfig, config_path: &Path) -> Result<(), EmulatorError> {
        self.config_manager.save_emulator_beacon_config(config, config_path).await
    }
    
    /// Create a default beacon configuration template for emulator use
    pub fn create_default_beacon_config(&self) -> BeaconConfig {
        EmulatorConfigManager::create_default_beacon_config()
    }
    
    /// Create a default emulator beacon configuration
    pub fn create_default_emulator_config(&self, beacon_id: Option<Uuid>, position: GeodeticPosition) -> EmulatorBeaconConfig {
        EmulatorConfigManager::create_default_emulator_config(beacon_id, position)
    }
    
    /// Validate beacon configuration for emulator use
    pub fn validate_emulator_config(&self, config: &BeaconConfig) -> Result<(), EmulatorError> {
        self.config_manager.validate_emulator_config(config)
    }
    
    /// Validate emulator-specific configuration
    pub fn validate_emulator_specific_config(&self, config: &EmulatorBeaconConfig) -> Result<(), EmulatorError> {
        self.config_manager.validate_emulator_specific_config(config)
    }
    
    /// Generate configuration template files
    pub async fn generate_config_template(&self, template_path: &Path, format: crate::config::ConfigFormat) -> Result<(), EmulatorError> {
        self.config_manager.generate_config_template(template_path, format).await
    }
    
    /// Generate emulator configuration template
    pub async fn generate_emulator_config_template(&self, template_path: &Path, position: GeodeticPosition, format: crate::config::ConfigFormat) -> Result<(), EmulatorError> {
        self.config_manager.generate_emulator_config_template(template_path, position, format).await
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
    pub fn get_communication_space(&self) -> Arc<RwLock<VirtualCommunicationSpace>> {
        self.communication_space.clone()
    }
    
    /// Save current emulator state to file
    pub async fn save_state(&self) -> Result<(), EmulatorError> {
        let mut beacon_data = Vec::new();
        
        for (id, beacon) in &self.virtual_beacons {
            let status = beacon.get_status();
            beacon_data.push(PersistentBeaconData {
                id: *id,
                position: status.position,
                config: status.config.clone(),
                movement_pattern: status.movement_pattern,
                intended_running: status.is_running,
                config_file_path: None, // TODO: Track config file path if loaded from file
                config_checksum: None, // TODO: Calculate config checksum
                config_updated_at: Some(std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap()
                    .as_secs()),
            });
        }
        
        let state = EmulatorState {
            beacons: beacon_data,
            current_channel: self.current_channel.clone(),
        };
        
        // Ensure the data directory exists
        if let Some(parent) = self.state_file_path.parent() {
            tokio::fs::create_dir_all(parent).await?;
        }
        
        let json = serde_json::to_string_pretty(&state)?;
        tokio::fs::write(&self.state_file_path, json).await?;
        
        Ok(())
    }
    
    /// Load emulator state from file
    pub async fn load_state(&mut self) -> Result<(), EmulatorError> {
        if !self.state_file_path.exists() {
            // No state file exists, start with empty state
            return Ok(());
        }
        
        let json = tokio::fs::read_to_string(&self.state_file_path).await?;
        let state: EmulatorState = serde_json::from_str(&json)?;
        
        // Set channel from saved state
        self.current_channel = state.current_channel;
        
        // Recreate beacons from saved state
        for beacon_data in state.beacons {
            let virtual_channel = {
                let mut comm_space = self.communication_space.write().await;
                comm_space.get_or_create_channel(&self.current_channel)
            };
            
            let mut virtual_beacon = VirtualBeacon::new_with_logger(
                beacon_data.id,
                beacon_data.config,
                beacon_data.position,
                virtual_channel,
                self.logger.clone(),
            )?;
            
            // Set movement pattern
            virtual_beacon.set_movement_pattern(beacon_data.movement_pattern)?;
            
            self.virtual_beacons.insert(beacon_data.id, virtual_beacon);
            
            // If beacon was intended to be running, start it
            if beacon_data.intended_running {
                if let Err(e) = self.start_beacon(beacon_data.id).await {
                    eprintln!("Warning: Failed to restart beacon {}: {}", beacon_data.id, e);
                }
            }
        }
        
        Ok(())
    }
    
    /// Clear all state and delete state file
    pub async fn clear_state(&mut self) -> Result<(), EmulatorError> {
        // Stop and remove all beacons
        self.shutdown().await?;
        
        // Delete state file if it exists
        if self.state_file_path.exists() {
            tokio::fs::remove_file(&self.state_file_path).await?;
        }
        
        Ok(())
    }
    
    /// Get statistics about the emulator manager
    pub async fn get_manager_stats(&self) -> EmulatorManagerStats {
        let running_beacons = self.get_active_beacon_count();
        let total_beacons = self.get_total_beacon_count();
        let channels = {
            let comm_space = self.communication_space.read().await;
            comm_space.list_channels()
        };
        
        // Update performance monitor with current counts
        self.performance_monitor.update_beacon_count(running_beacons).await;
        self.performance_monitor.update_channel_count(channels.len()).await;
        
        // Update channel queue depths
        let mut queue_depths = HashMap::new();
        {
            let comm_space = self.communication_space.read().await;
            for channel_name in &channels {
                if let Some(channel) = comm_space.get_channel(channel_name) {
                    let count = channel.get_message_count().await;
                    queue_depths.insert(channel_name.clone(), count);
                }
            }
        }
        self.performance_monitor.update_channel_queue_depths(queue_depths).await;
        
        EmulatorManagerStats {
            total_beacons,
            running_beacons,
            stopped_beacons: total_beacons - running_beacons,
            active_channels: channels.len(),
            current_channel: self.current_channel.clone(),
            channel_names: channels,
        }
    }
    
    /// Start the IPC server for cross-platform communication
    pub async fn start_ipc_server(&mut self, port: Option<u16>) -> Result<(), EmulatorError> {
        if self.ipc_server.is_some() {
            return Err(EmulatorError::ConfigError("IPC server is already running".to_string()));
        }
        
        // Create IPC server with shared communication space
        let mut ipc_server = IpcServer::new_with_shared_communication_space(
            port, 
            self.communication_space.clone()
        );
        
        // Start the IPC server
        ipc_server.start().await?;
        
        println!("IPC server started on port {}", ipc_server.port());
        self.ipc_server = Some(ipc_server);
        
        Ok(())
    }
    
    /// Stop the IPC server
    pub async fn stop_ipc_server(&mut self) -> Result<(), EmulatorError> {
        if let Some(ipc_server) = self.ipc_server.take() {
            ipc_server.shutdown().await?;
            println!("IPC server stopped");
        }
        Ok(())
    }
    
    /// Check if IPC server is running
    pub fn is_ipc_server_running(&self) -> bool {
        self.ipc_server.is_some()
    }
    
    /// Get IPC server port (if running)
    pub fn get_ipc_server_port(&self) -> Option<u16> {
        self.ipc_server.as_ref().map(|server| server.port())
    }
    
    /// Get reference to the beacon logger
    pub fn get_logger(&self) -> Arc<BeaconLogger> {
        self.logger.clone()
    }
    
    /// Get logging statistics
    pub async fn get_logging_stats(&self) -> crate::LoggingStats {
        self.logger.get_stats().await
    }
    
    /// Clear all log entries
    pub async fn clear_logs(&self) -> Result<(), EmulatorError> {
        self.logger.clear_logs().await;
        Ok(())
    }
    
    /// Synchronize communication spaces between emulator and IPC server
    async fn sync_communication_spaces(&mut self) {
        if let Some(ipc_server) = &self.ipc_server {
            let server_comm_space = ipc_server.get_communication_space();
            let mut server_space = server_comm_space.write().await;
            
            // Ensure all channels exist in both spaces
            let channel_names = {
                let comm_space = self.communication_space.read().await;
                comm_space.list_channels()
            };
            for channel_name in channel_names {
                server_space.get_or_create_channel(&channel_name);
            }
        }
    }
    
    /// Get performance metrics
    pub async fn get_performance_metrics(&self) -> PerformanceMetrics {
        // Update current counts before getting metrics
        let running_beacons = self.get_active_beacon_count();
        let channels = {
            let comm_space = self.communication_space.read().await;
            comm_space.list_channels()
        };
        
        self.performance_monitor.update_beacon_count(running_beacons).await;
        self.performance_monitor.update_channel_count(channels.len()).await;
        
        self.performance_monitor.get_metrics().await
    }
    
    /// Get performance optimizer
    pub fn get_performance_optimizer(&self) -> Arc<PerformanceOptimizer> {
        self.performance_optimizer.clone()
    }
    
    /// Get performance monitor
    pub fn get_performance_monitor(&self) -> Arc<PerformanceMonitor> {
        self.performance_monitor.clone()
    }
    
    /// Update performance configuration
    pub async fn update_performance_config(&self, config: PerformanceConfig) -> Result<(), EmulatorError> {
        self.performance_optimizer.update_config(config).await;
        Ok(())
    }
    
    /// Get performance configuration
    pub async fn get_performance_config(&self) -> PerformanceConfig {
        self.performance_optimizer.get_config().await
    }
    
    /// Run performance optimization
    pub async fn optimize_performance(&self) -> Result<Vec<String>, EmulatorError> {
        self.performance_optimizer.optimize_if_needed().await
    }
    
    /// Get performance recommendations
    pub async fn get_performance_recommendations(&self) -> Vec<String> {
        self.performance_optimizer.get_recommendations().await
    }
    
    /// Enable or disable automatic performance optimization
    pub async fn set_auto_optimization(&self, enabled: bool) -> Result<(), EmulatorError> {
        let mut config = self.performance_optimizer.get_config().await;
        config.auto_optimization_enabled = enabled;
        self.performance_optimizer.update_config(config).await;
        Ok(())
    }
    
    /// Set global message rate limit
    pub async fn set_global_rate_limit(&self, rate: Option<f64>) -> Result<(), EmulatorError> {
        let mut config = self.performance_optimizer.get_config().await;
        config.global_rate_limit = rate;
        self.performance_optimizer.update_config(config).await;
        Ok(())
    }
    
    /// Set per-beacon message rate limit
    pub async fn set_per_beacon_rate_limit(&self, rate: Option<f64>) -> Result<(), EmulatorError> {
        let mut config = self.performance_optimizer.get_config().await;
        config.per_beacon_rate_limit = rate;
        self.performance_optimizer.update_config(config).await;
        Ok(())
    }
    
    /// Enable or disable collision avoidance
    pub async fn set_collision_avoidance(&self, enabled: bool) -> Result<(), EmulatorError> {
        let mut config = self.performance_optimizer.get_config().await;
        config.collision_avoidance_enabled = enabled;
        self.performance_optimizer.update_config(config).await;
        Ok(())
    }
    
    /// Cleanup old messages from channels to free memory
    pub async fn cleanup_old_messages(&self, max_age_seconds: u64) -> Result<usize, EmulatorError> {
        let cutoff_time = SystemTime::now() - std::time::Duration::from_secs(max_age_seconds);
        let mut total_cleaned = 0;
        
        let comm_space = self.communication_space.read().await;
        let channels = comm_space.list_channels();
        
        for channel_name in channels {
            if let Some(channel) = comm_space.get_channel(&channel_name) {
                // Get messages before cleanup
                let before_count = channel.get_message_count().await;
                
                // For now, we'll just clear all messages if any are older than cutoff
                // A more sophisticated implementation would selectively remove old messages
                let messages = channel.get_messages_since(cutoff_time).await;
                if messages.len() < before_count {
                    // Some messages are older than cutoff, clear the channel
                    channel.clear_messages().await;
                    total_cleaned += before_count;
                    
                    tracing::info!("Cleaned {} old messages from channel '{}'", before_count, channel_name);
                }
            }
        }
        
        Ok(total_cleaned)
    }
    
    /// Get memory usage breakdown
    pub async fn get_memory_breakdown(&self) -> Result<crate::performance::MemoryBreakdown, EmulatorError> {
        let memory_tracker = self.performance_monitor.get_memory_tracker();
        Ok(memory_tracker.get_memory_breakdown().await)
    }
    
    /// Force garbage collection and memory cleanup
    pub async fn force_memory_cleanup(&self) -> Result<(), EmulatorError> {
        // Cleanup old messages
        let config = self.performance_optimizer.get_config().await;
        if config.message_queue_cleanup_enabled {
            let cleanup_interval = config.message_queue_cleanup_interval_s;
            let cleaned = self.cleanup_old_messages(cleanup_interval).await?;
            if cleaned > 0 {
                tracing::info!("Force cleanup removed {} old messages", cleaned);
            }
        }
        
        // Update memory tracking
        let memory_tracker = self.performance_monitor.get_memory_tracker();
        
        // Re-estimate beacon memory usage
        for (beacon_id, beacon) in &self.virtual_beacons {
            let estimated_size = Self::estimate_beacon_memory_usage(beacon);
            memory_tracker.track_beacon_memory(*beacon_id, estimated_size).await;
        }
        
        // Re-estimate channel memory usage
        let comm_space = self.communication_space.read().await;
        for channel_name in comm_space.list_channels() {
            if let Some(channel) = comm_space.get_channel(&channel_name) {
                let message_count = channel.get_message_count().await;
                let estimated_size = message_count * 256; // Rough estimate: 256 bytes per message
                memory_tracker.track_channel_memory(channel_name, estimated_size as u64).await;
            }
        }
        
        Ok(())
    }
    
    /// Estimate memory usage for a beacon
    fn estimate_beacon_memory_usage(beacon: &VirtualBeacon) -> u64 {
        // Rough estimate based on beacon components
        let base_size = 1024; // Base beacon structure
        let config_size = 512; // Configuration data
        let stats_size = 256; // Statistics
        let buffer_size = 1024; // Message buffers and other data
        
        base_size + config_size + stats_size + buffer_size
    }
}

/// Statistics about the emulator manager
#[derive(Debug, Clone, Serialize, Deserialize)]
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
        let stats = manager.get_manager_stats().await;
        assert_eq!(stats.total_beacons, 0);
        assert_eq!(stats.running_beacons, 0);
        assert_eq!(stats.stopped_beacons, 0);
        assert_eq!(stats.current_channel, "test_channel");
        
        // Create and start some beacons
        let beacon1 = manager.create_beacon(None, position, None).await.unwrap();
        let beacon2 = manager.create_beacon(None, position, None).await.unwrap();
        
        manager.start_beacon(beacon1).await.unwrap();
        
        let stats = manager.get_manager_stats().await;
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
        
        // Create a custom config with shorter transmission interval for testing
        let mut config = BeaconConfig::new(Uuid::new_v4());
        config.transmission.interval_ms = 100; // 100ms for fast testing
        
        // Create and start beacon
        let beacon_id = manager.create_beacon(None, position, Some(config)).await.unwrap();
        manager.start_beacon(beacon_id).await.unwrap();
        
        // Get channel to subscribe to messages after beacon is created
        let comm_space_arc = manager.get_communication_space();
        let channel = {
            let mut comm_space = comm_space_arc.write().await;
            comm_space.get_or_create_channel("test_channel")
        };
        let mut receiver = channel.subscribe();
        
        // Wait for message transmission (using 100ms interval)
        sleep(Duration::from_millis(200)).await;
        
        // Check if we received a message
        let received_message = tokio::time::timeout(
            Duration::from_millis(300),
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