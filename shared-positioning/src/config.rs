use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs;
use std::path::Path;

/// System configuration with validation and bounds checking
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SystemConfig {
    /// Sound speed in water (m/s) - typical range 1450-1550 m/s
    pub sound_speed_m_per_s: f64,
    /// Maximum age of anchor data before considered stale (milliseconds)
    pub max_anchor_age_ms: u32,
    /// Minimum number of anchors required for positioning
    pub min_anchors: u8,
    /// Maximum time to wait for position calculation (milliseconds)
    pub position_timeout_ms: u32,
    /// Accuracy threshold for position validation (meters)
    pub accuracy_threshold_m: f32,
    /// Maximum range for anchor signals (meters)
    pub max_range_m: f32,
    /// Enable/disable regularization for ill-conditioned systems
    pub enable_regularization: bool,
    /// Regularization parameter (lambda) for Tikhonov regularization
    pub regularization_lambda: f64,
    /// Maximum condition number before applying regularization
    pub max_condition_number: f64,
}

impl SystemConfig {
    /// Create a new system configuration with default values
    pub fn new() -> Self {
        Self {
            sound_speed_m_per_s: 1500.0,
            max_anchor_age_ms: 5000,
            min_anchors: 3,
            position_timeout_ms: 200,
            accuracy_threshold_m: 2.0,
            max_range_m: 10000.0,
            enable_regularization: true,
            regularization_lambda: 1e-6,
            max_condition_number: 1000.0,
        }
    }

    /// Create configuration optimized for embedded systems
    pub fn embedded() -> Self {
        Self {
            sound_speed_m_per_s: 1500.0,
            max_anchor_age_ms: 3000,  // Shorter for real-time systems
            min_anchors: 3,
            position_timeout_ms: 100,  // Faster response required
            accuracy_threshold_m: 5.0,  // More lenient for embedded
            max_range_m: 5000.0,  // Shorter range for power efficiency
            enable_regularization: true,
            regularization_lambda: 1e-5,  // Slightly higher for stability
            max_condition_number: 500.0,  // Lower threshold for embedded
        }
    }

    /// Create configuration for high-precision applications
    pub fn high_precision() -> Self {
        Self {
            sound_speed_m_per_s: 1500.0,
            max_anchor_age_ms: 10000,  // Allow older data for precision
            min_anchors: 4,  // Require 4 anchors for 3D positioning
            position_timeout_ms: 500,  // Allow more time for calculations
            accuracy_threshold_m: 0.5,  // High precision requirement
            max_range_m: 20000.0,  // Extended range
            enable_regularization: true,
            regularization_lambda: 1e-8,  // Lower for better precision
            max_condition_number: 10000.0,  // Higher tolerance
        }
    }

    /// Validate configuration parameters and return errors if invalid
    pub fn validate(&self) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();

        // Validate sound speed (typical range for seawater)
        if self.sound_speed_m_per_s < 1400.0 || self.sound_speed_m_per_s > 1600.0 {
            errors.push(format!(
                "Sound speed {:.1} m/s is outside typical range (1400-1600 m/s)",
                self.sound_speed_m_per_s
            ));
        }

        // Validate anchor age limit
        if self.max_anchor_age_ms < 100 {
            errors.push("Max anchor age must be at least 100ms".to_string());
        }
        if self.max_anchor_age_ms > 60000 {
            errors.push("Max anchor age should not exceed 60 seconds".to_string());
        }

        // Validate minimum anchors
        if self.min_anchors < 3 {
            errors.push("Minimum anchors must be at least 3 for positioning".to_string());
        }
        if self.min_anchors > 10 {
            errors.push("Minimum anchors should not exceed 10".to_string());
        }

        // Validate timeout
        if self.position_timeout_ms < 10 {
            errors.push("Position timeout must be at least 10ms".to_string());
        }
        if self.position_timeout_ms > 10000 {
            errors.push("Position timeout should not exceed 10 seconds".to_string());
        }

        // Validate accuracy threshold
        if self.accuracy_threshold_m <= 0.0 {
            errors.push("Accuracy threshold must be positive".to_string());
        }
        if self.accuracy_threshold_m > 1000.0 {
            errors.push("Accuracy threshold should not exceed 1000m".to_string());
        }

        // Validate max range
        if self.max_range_m <= 0.0 {
            errors.push("Max range must be positive".to_string());
        }
        if self.max_range_m > 100000.0 {
            errors.push("Max range should not exceed 100km".to_string());
        }

        // Validate regularization parameters
        if self.regularization_lambda <= 0.0 {
            errors.push("Regularization lambda must be positive".to_string());
        }
        if self.regularization_lambda > 1.0 {
            errors.push("Regularization lambda should not exceed 1.0".to_string());
        }

        if self.max_condition_number <= 1.0 {
            errors.push("Max condition number must be greater than 1.0".to_string());
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }

    /// Get sound speed in mm/ms for embedded systems
    pub fn get_sound_speed_mm_per_ms(&self) -> f32 {
        (self.sound_speed_m_per_s / 1000.0) as f32
    }

    /// Get accuracy threshold in millimeters for embedded systems
    pub fn get_accuracy_threshold_mm(&self) -> u32 {
        (self.accuracy_threshold_m * 1000.0) as u32
    }
}

impl Default for SystemConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// Individual anchor configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnchorConfig {
    /// Unique anchor identifier
    pub id: u16,
    /// Anchor position in geodetic coordinates
    pub position: GeodeticPosition,
    /// Maximum operational range for this anchor (meters)
    pub max_range_m: f32,
    /// Whether this anchor is enabled for positioning
    pub enabled: bool,
    /// Signal quality threshold (0-255, higher is better)
    pub min_signal_quality: u8,
    /// Anchor-specific timeout override (milliseconds, 0 = use system default)
    pub timeout_override_ms: u32,
    /// Human-readable name/description
    pub name: String,
}

/// Geodetic position representation
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct GeodeticPosition {
    /// Latitude in decimal degrees (WGS84)
    pub latitude: f64,
    /// Longitude in decimal degrees (WGS84)
    pub longitude: f64,
    /// Depth in meters (positive downward from surface)
    pub depth: f64,
}

impl AnchorConfig {
    /// Create a new anchor configuration
    pub fn new(id: u16, latitude: f64, longitude: f64, depth: f64) -> Self {
        Self {
            id,
            position: GeodeticPosition {
                latitude,
                longitude,
                depth,
            },
            max_range_m: 5000.0,
            enabled: true,
            min_signal_quality: 100,
            timeout_override_ms: 0,
            name: format!("Anchor_{:03}", id),
        }
    }

    /// Validate anchor configuration
    pub fn validate(&self) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();

        // Validate position
        if self.position.latitude < -90.0 || self.position.latitude > 90.0 {
            errors.push(format!("Invalid latitude: {}", self.position.latitude));
        }
        if self.position.longitude < -180.0 || self.position.longitude > 180.0 {
            errors.push(format!("Invalid longitude: {}", self.position.longitude));
        }
        if self.position.depth < 0.0 {
            errors.push("Depth cannot be negative".to_string());
        }
        if self.position.depth > 11000.0 {  // Deeper than Mariana Trench
            errors.push("Depth exceeds maximum ocean depth".to_string());
        }

        // Validate range
        if self.max_range_m <= 0.0 {
            errors.push("Max range must be positive".to_string());
        }
        if self.max_range_m > 100000.0 {
            errors.push("Max range should not exceed 100km".to_string());
        }

        // Validate timeout override
        if self.timeout_override_ms > 0 && self.timeout_override_ms < 10 {
            errors.push("Timeout override must be at least 10ms if specified".to_string());
        }

        // Validate name
        if self.name.is_empty() {
            errors.push("Anchor name cannot be empty".to_string());
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }

    /// Check if anchor is within operational range of a position
    pub fn is_in_range(&self, target_lat: f64, target_lon: f64, target_depth: f64) -> bool {
        let distance = self.calculate_distance(target_lat, target_lon, target_depth);
        distance <= self.max_range_m as f64
    }

    /// Calculate approximate distance to a target position (meters)
    pub fn calculate_distance(&self, target_lat: f64, target_lon: f64, target_depth: f64) -> f64 {
        // Simple Euclidean distance in local coordinates
        let lat_diff = (target_lat - self.position.latitude) * 111132.0; // meters per degree lat
        let lon_diff = (target_lon - self.position.longitude) * 111320.0 * self.position.latitude.to_radians().cos();
        let depth_diff = target_depth - self.position.depth;
        
        (lat_diff.powi(2) + lon_diff.powi(2) + depth_diff.powi(2)).sqrt()
    }
}

/// Complete configuration container
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PositioningConfig {
    /// System-wide configuration
    pub system: SystemConfig,
    /// Individual anchor configurations
    pub anchors: HashMap<u16, AnchorConfig>,
    /// Configuration metadata
    pub metadata: ConfigMetadata,
}

/// Configuration metadata for tracking and validation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConfigMetadata {
    /// Configuration version for compatibility checking
    pub version: String,
    /// Creation timestamp (Unix timestamp)
    pub created_at: u64,
    /// Last modification timestamp (Unix timestamp)
    pub modified_at: u64,
    /// Human-readable description
    pub description: String,
    /// Configuration author/creator
    pub author: String,
}

impl PositioningConfig {
    /// Create a new positioning configuration
    pub fn new() -> Self {
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs();

        Self {
            system: SystemConfig::new(),
            anchors: HashMap::new(),
            metadata: ConfigMetadata {
                version: "1.0.0".to_string(),
                created_at: now,
                modified_at: now,
                description: "Default positioning configuration".to_string(),
                author: "System".to_string(),
            },
        }
    }

    /// Add an anchor to the configuration
    pub fn add_anchor(&mut self, anchor: AnchorConfig) -> Result<(), String> {
        // Validate anchor before adding
        anchor.validate().map_err(|errors| {
            format!("Anchor validation failed: {}", errors.join(", "))
        })?;

        // Check for duplicate ID
        if self.anchors.contains_key(&anchor.id) {
            return Err(format!("Anchor with ID {} already exists", anchor.id));
        }

        self.anchors.insert(anchor.id, anchor);
        self.update_modified_time();
        Ok(())
    }

    /// Remove an anchor from the configuration
    pub fn remove_anchor(&mut self, anchor_id: u16) -> Result<AnchorConfig, String> {
        self.anchors.remove(&anchor_id)
            .ok_or_else(|| format!("Anchor with ID {} not found", anchor_id))
            .map(|anchor| {
                self.update_modified_time();
                anchor
            })
    }

    /// Get anchor configuration by ID
    pub fn get_anchor(&self, anchor_id: u16) -> Option<&AnchorConfig> {
        self.anchors.get(&anchor_id)
    }

    /// Get mutable anchor configuration by ID
    pub fn get_anchor_mut(&mut self, anchor_id: u16) -> Option<&mut AnchorConfig> {
        self.anchors.get_mut(&anchor_id)
    }

    /// Get all enabled anchors
    pub fn get_enabled_anchors(&self) -> Vec<&AnchorConfig> {
        self.anchors.values().filter(|anchor| anchor.enabled).collect()
    }

    /// Validate the entire configuration
    pub fn validate(&self) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();

        // Validate system configuration
        if let Err(system_errors) = self.system.validate() {
            errors.extend(system_errors.into_iter().map(|e| format!("System: {}", e)));
        }

        // Validate each anchor
        for (id, anchor) in &self.anchors {
            if let Err(anchor_errors) = anchor.validate() {
                errors.extend(anchor_errors.into_iter().map(|e| format!("Anchor {}: {}", id, e)));
            }
        }

        // Check for sufficient enabled anchors
        let enabled_count = self.get_enabled_anchors().len();
        if enabled_count < self.system.min_anchors as usize {
            errors.push(format!(
                "Insufficient enabled anchors: {} enabled, {} required",
                enabled_count, self.system.min_anchors
            ));
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }

    /// Update the modification timestamp
    fn update_modified_time(&mut self) {
        self.metadata.modified_at = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs();
    }

    /// Load configuration from JSON file
    pub fn load_from_file<P: AsRef<Path>>(path: P) -> Result<Self, String> {
        let content = fs::read_to_string(path)
            .map_err(|e| format!("Failed to read config file: {}", e))?;
        
        let config: PositioningConfig = serde_json::from_str(&content)
            .map_err(|e| format!("Failed to parse config JSON: {}", e))?;
        
        // Validate loaded configuration
        config.validate().map_err(|errors| {
            format!("Configuration validation failed: {}", errors.join(", "))
        })?;

        Ok(config)
    }

    /// Save configuration to JSON file
    pub fn save_to_file<P: AsRef<Path>>(&mut self, path: P) -> Result<(), String> {
        // Update modification time before saving
        self.update_modified_time();

        // Validate before saving
        self.validate().map_err(|errors| {
            format!("Cannot save invalid configuration: {}", errors.join(", "))
        })?;

        let json = serde_json::to_string_pretty(self)
            .map_err(|e| format!("Failed to serialize config: {}", e))?;
        
        fs::write(path, json)
            .map_err(|e| format!("Failed to write config file: {}", e))?;

        Ok(())
    }
}

impl Default for PositioningConfig {
    fn default() -> Self {
        Self::new()
    }
}

/// Runtime parameter manager for dynamic configuration updates
#[derive(Debug, Clone)]
pub struct RuntimeParameterManager {
    /// Current active configuration
    config: PositioningConfig,
    /// Backup configuration for rollback
    backup_config: Option<PositioningConfig>,
    /// Change history for auditing
    change_history: Vec<ConfigChange>,
    /// Maximum number of changes to keep in history
    max_history_size: usize,
}

/// Record of a configuration change for auditing and rollback
#[derive(Debug, Clone)]
pub struct ConfigChange {
    /// Timestamp of the change
    pub timestamp: u64,
    /// Type of change made
    pub change_type: ChangeType,
    /// Description of the change
    pub description: String,
    /// Previous value (for rollback)
    pub previous_value: String,
    /// New value
    pub new_value: String,
}

/// Types of configuration changes
#[derive(Debug, Clone)]
pub enum ChangeType {
    SoundSpeedUpdate,
    TimeoutUpdate,
    ThresholdUpdate,
    AnchorEnable,
    AnchorDisable,
    AnchorParameterUpdate,
    SystemParameterUpdate,
}

impl RuntimeParameterManager {
    /// Create a new runtime parameter manager
    pub fn new(config: PositioningConfig) -> Self {
        Self {
            config,
            backup_config: None,
            change_history: Vec::new(),
            max_history_size: 100,
        }
    }

    /// Get the current configuration (read-only)
    pub fn get_config(&self) -> &PositioningConfig {
        &self.config
    }

    /// Create a backup of the current configuration for rollback
    pub fn create_backup(&mut self) {
        self.backup_config = Some(self.config.clone());
    }

    /// Rollback to the backup configuration
    pub fn rollback(&mut self) -> Result<(), String> {
        if let Some(backup) = self.backup_config.take() {
            // Validate backup before rollback
            backup.validate().map_err(|errors| {
                format!("Backup configuration is invalid: {}", errors.join(", "))
            })?;

            self.config = backup;
            self.record_change(ChangeType::SystemParameterUpdate, 
                             "Rollback to backup configuration".to_string(),
                             "current".to_string(),
                             "backup".to_string());
            Ok(())
        } else {
            Err("No backup configuration available".to_string())
        }
    }

    /// Update sound speed parameter with validation
    pub fn update_sound_speed(&mut self, new_speed_m_per_s: f64) -> Result<(), String> {
        let old_speed = self.config.system.sound_speed_m_per_s;
        
        // Validate new sound speed
        if new_speed_m_per_s < 1400.0 || new_speed_m_per_s > 1600.0 {
            return Err(format!(
                "Sound speed {:.1} m/s is outside valid range (1400-1600 m/s)",
                new_speed_m_per_s
            ));
        }

        // Create backup before change
        if self.backup_config.is_none() {
            self.create_backup();
        }

        // Apply change
        self.config.system.sound_speed_m_per_s = new_speed_m_per_s;
        self.config.update_modified_time();

        // Validate configuration after change
        if let Err(errors) = self.config.validate() {
            // Rollback on validation failure
            self.config.system.sound_speed_m_per_s = old_speed;
            return Err(format!("Sound speed update failed validation: {}", errors.join(", ")));
        }

        self.record_change(
            ChangeType::SoundSpeedUpdate,
            format!("Updated sound speed from {:.1} to {:.1} m/s", old_speed, new_speed_m_per_s),
            old_speed.to_string(),
            new_speed_m_per_s.to_string(),
        );

        Ok(())
    }

    /// Record a configuration change
    fn record_change(&mut self, change_type: ChangeType, description: String, previous_value: String, new_value: String) {
        let change = ConfigChange {
            timestamp: std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_secs(),
            change_type,
            description,
            previous_value,
            new_value,
        };

        self.change_history.push(change);

        // Limit history size
        if self.change_history.len() > self.max_history_size {
            self.change_history.remove(0);
        }
    }

    /// Get change history
    pub fn get_change_history(&self) -> &[ConfigChange] {
        &self.change_history
    }
}