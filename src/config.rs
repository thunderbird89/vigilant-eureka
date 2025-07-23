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

        // Check for anchor geometry (basic validation)
        if enabled_count >= 3 {
            let enabled_anchors = self.get_enabled_anchors();
            if self.check_anchor_geometry(&enabled_anchors).is_err() {
                errors.push("Anchor geometry may be degenerate (collinear or coplanar)".to_string());
            }
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }

    /// Check anchor geometry for potential issues
    fn check_anchor_geometry(&self, anchors: &[&AnchorConfig]) -> Result<(), String> {
        if anchors.len() < 3 {
            return Err("Need at least 3 anchors for geometry check".to_string());
        }

        // Check if all anchors are at the same depth (coplanar)
        let first_depth = anchors[0].position.depth;
        let all_same_depth = anchors.iter().all(|a| (a.position.depth - first_depth).abs() < 1e-6);

        if all_same_depth && anchors.len() >= 3 {
            // Check for collinearity in 2D
            let p1 = &anchors[0].position;
            let p2 = &anchors[1].position;
            let p3 = &anchors[2].position;

            // Calculate area of triangle formed by first 3 anchors
            let area = ((p2.latitude - p1.latitude) * (p3.longitude - p1.longitude) - 
                       (p3.latitude - p1.latitude) * (p2.longitude - p1.longitude)).abs() / 2.0;

            if area < 1e-10 {  // Very small area indicates collinearity
                return Err("Anchors appear to be collinear".to_string());
            }
        }

        Ok(())
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

    /// Create a backup of the current configuration
    pub fn create_backup<P: AsRef<Path>>(&mut self, backup_path: P) -> Result<(), String> {
        let timestamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs();
        
        let backup_file = format!("{}.backup.{}", 
            backup_path.as_ref().to_string_lossy(), timestamp);
        
        self.save_to_file(backup_file)
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

    /// Update timeout parameters
    pub fn update_timeout(&mut self, parameter: TimeoutParameter, new_value_ms: u32) -> Result<(), String> {
        let old_value = match parameter {
            TimeoutParameter::MaxAnchorAge => self.config.system.max_anchor_age_ms,
            TimeoutParameter::PositionTimeout => self.config.system.position_timeout_ms,
        };

        // Validate new timeout value
        if new_value_ms < 10 {
            return Err("Timeout value must be at least 10ms".to_string());
        }
        if new_value_ms > 60000 {
            return Err("Timeout value should not exceed 60 seconds".to_string());
        }

        // Create backup before change
        if self.backup_config.is_none() {
            self.create_backup();
        }

        // Apply change
        match parameter {
            TimeoutParameter::MaxAnchorAge => {
                self.config.system.max_anchor_age_ms = new_value_ms;
            }
            TimeoutParameter::PositionTimeout => {
                self.config.system.position_timeout_ms = new_value_ms;
            }
        }
        self.config.update_modified_time();

        // Validate configuration after change
        if let Err(errors) = self.config.validate() {
            // Rollback on validation failure
            match parameter {
                TimeoutParameter::MaxAnchorAge => {
                    self.config.system.max_anchor_age_ms = old_value;
                }
                TimeoutParameter::PositionTimeout => {
                    self.config.system.position_timeout_ms = old_value;
                }
            }
            return Err(format!("Timeout update failed validation: {}", errors.join(", ")));
        }

        self.record_change(
            ChangeType::TimeoutUpdate,
            format!("Updated {:?} from {}ms to {}ms", parameter, old_value, new_value_ms),
            old_value.to_string(),
            new_value_ms.to_string(),
        );

        Ok(())
    }

    /// Update threshold parameters
    pub fn update_threshold(&mut self, parameter: ThresholdParameter, new_value: f32) -> Result<(), String> {
        let old_value = match parameter {
            ThresholdParameter::AccuracyThreshold => self.config.system.accuracy_threshold_m,
            ThresholdParameter::MaxRange => self.config.system.max_range_m,
        };

        // Validate new threshold value
        if new_value <= 0.0 {
            return Err("Threshold value must be positive".to_string());
        }

        match parameter {
            ThresholdParameter::AccuracyThreshold => {
                if new_value > 1000.0 {
                    return Err("Accuracy threshold should not exceed 1000m".to_string());
                }
            }
            ThresholdParameter::MaxRange => {
                if new_value > 100000.0 {
                    return Err("Max range should not exceed 100km".to_string());
                }
            }
        }

        // Create backup before change
        if self.backup_config.is_none() {
            self.create_backup();
        }

        // Apply change
        match parameter {
            ThresholdParameter::AccuracyThreshold => {
                self.config.system.accuracy_threshold_m = new_value;
            }
            ThresholdParameter::MaxRange => {
                self.config.system.max_range_m = new_value;
            }
        }
        self.config.update_modified_time();

        // Validate configuration after change
        if let Err(errors) = self.config.validate() {
            // Rollback on validation failure
            match parameter {
                ThresholdParameter::AccuracyThreshold => {
                    self.config.system.accuracy_threshold_m = old_value;
                }
                ThresholdParameter::MaxRange => {
                    self.config.system.max_range_m = old_value;
                }
            }
            return Err(format!("Threshold update failed validation: {}", errors.join(", ")));
        }

        self.record_change(
            ChangeType::ThresholdUpdate,
            format!("Updated {:?} from {:.2} to {:.2}", parameter, old_value, new_value),
            old_value.to_string(),
            new_value.to_string(),
        );

        Ok(())
    }

    /// Enable an anchor
    pub fn enable_anchor(&mut self, anchor_id: u16) -> Result<(), String> {
        // Check if anchor exists and get current state
        let current_enabled = self.config.anchors.get(&anchor_id)
            .ok_or_else(|| format!("Anchor {} not found", anchor_id))?
            .enabled;

        if current_enabled {
            return Ok(()); // Already enabled
        }

        // Create backup before change
        if self.backup_config.is_none() {
            self.create_backup();
        }

        // Apply change
        if let Some(anchor) = self.config.anchors.get_mut(&anchor_id) {
            anchor.enabled = true;
        }
        self.config.update_modified_time();

        // Validate configuration after change
        if let Err(errors) = self.config.validate() {
            // Rollback on validation failure
            if let Some(anchor) = self.config.anchors.get_mut(&anchor_id) {
                anchor.enabled = false;
            }
            return Err(format!("Enable anchor failed validation: {}", errors.join(", ")));
        }

        self.record_change(
            ChangeType::AnchorEnable,
            format!("Enabled anchor {}", anchor_id),
            "false".to_string(),
            "true".to_string(),
        );

        Ok(())
    }

    /// Disable an anchor
    pub fn disable_anchor(&mut self, anchor_id: u16) -> Result<(), String> {
        // Check if anchor exists and get current state
        let current_enabled = self.config.anchors.get(&anchor_id)
            .ok_or_else(|| format!("Anchor {} not found", anchor_id))?
            .enabled;

        if !current_enabled {
            return Ok(()); // Already disabled
        }

        // Check if disabling this anchor would leave insufficient anchors
        let enabled_count = self.config.get_enabled_anchors().len();
        if enabled_count <= self.config.system.min_anchors as usize {
            return Err(format!(
                "Cannot disable anchor {}: would leave only {} enabled anchors (minimum {})",
                anchor_id, enabled_count - 1, self.config.system.min_anchors
            ));
        }

        // Create backup before change
        if self.backup_config.is_none() {
            self.create_backup();
        }

        // Apply change
        if let Some(anchor) = self.config.anchors.get_mut(&anchor_id) {
            anchor.enabled = false;
        }
        self.config.update_modified_time();

        // Validate configuration after change
        if let Err(errors) = self.config.validate() {
            // Rollback on validation failure
            if let Some(anchor) = self.config.anchors.get_mut(&anchor_id) {
                anchor.enabled = true;
            }
            return Err(format!("Disable anchor failed validation: {}", errors.join(", ")));
        }

        self.record_change(
            ChangeType::AnchorDisable,
            format!("Disabled anchor {}", anchor_id),
            "true".to_string(),
            "false".to_string(),
        );

        Ok(())
    }

    /// Update anchor-specific parameters
    pub fn update_anchor_parameter(&mut self, anchor_id: u16, parameter: AnchorParameter, new_value: AnchorParameterValue) -> Result<(), String> {
        // Get current value for rollback
        let old_value = {
            let anchor = self.config.anchors.get(&anchor_id)
                .ok_or_else(|| format!("Anchor {} not found", anchor_id))?;
            
            match parameter {
                AnchorParameter::MaxRange => AnchorParameterValue::Float(anchor.max_range_m),
                AnchorParameter::MinSignalQuality => AnchorParameterValue::U8(anchor.min_signal_quality),
                AnchorParameter::TimeoutOverride => AnchorParameterValue::U32(anchor.timeout_override_ms),
            }
        };

        // Create backup before change
        if self.backup_config.is_none() {
            self.create_backup();
        }

        // Apply change with validation
        {
            let anchor = self.config.anchors.get_mut(&anchor_id).unwrap();
            match (parameter, new_value) {
                (AnchorParameter::MaxRange, AnchorParameterValue::Float(value)) => {
                    if value <= 0.0 || value > 100000.0 {
                        return Err("Max range must be between 0 and 100000 meters".to_string());
                    }
                    anchor.max_range_m = value;
                }
                (AnchorParameter::MinSignalQuality, AnchorParameterValue::U8(value)) => {
                    anchor.min_signal_quality = value;
                }
                (AnchorParameter::TimeoutOverride, AnchorParameterValue::U32(value)) => {
                    if value > 0 && value < 10 {
                        return Err("Timeout override must be at least 10ms if specified".to_string());
                    }
                    anchor.timeout_override_ms = value;
                }
                _ => return Err("Parameter type mismatch".to_string()),
            }
        }

        self.config.update_modified_time();

        // Validate configuration after change
        if let Err(errors) = self.config.validate() {
            // Rollback on validation failure
            let anchor = self.config.anchors.get_mut(&anchor_id).unwrap();
            match (parameter, old_value) {
                (AnchorParameter::MaxRange, AnchorParameterValue::Float(value)) => {
                    anchor.max_range_m = value;
                }
                (AnchorParameter::MinSignalQuality, AnchorParameterValue::U8(value)) => {
                    anchor.min_signal_quality = value;
                }
                (AnchorParameter::TimeoutOverride, AnchorParameterValue::U32(value)) => {
                    anchor.timeout_override_ms = value;
                }
                _ => {}
            }
            return Err(format!("Anchor parameter update failed validation: {}", errors.join(", ")));
        }

        self.record_change(
            ChangeType::AnchorParameterUpdate,
            format!("Updated anchor {} {:?} from {:?} to {:?}", anchor_id, parameter, old_value, new_value),
            format!("{:?}", old_value),
            format!("{:?}", new_value),
        );

        Ok(())
    }

    /// Get change history
    pub fn get_change_history(&self) -> &[ConfigChange] {
        &self.change_history
    }

    /// Clear change history
    pub fn clear_history(&mut self) {
        self.change_history.clear();
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

    /// Apply multiple parameter changes atomically
    pub fn apply_batch_changes(&mut self, changes: Vec<ParameterChange>) -> Result<(), String> {
        // Create backup before any changes
        self.create_backup();

        let mut applied_changes = Vec::new();
        
        // Apply all changes
        for change in changes {
            let result = match change {
                ParameterChange::SoundSpeed(value) => {
                    self.update_sound_speed(value)
                }
                ParameterChange::Timeout(param, value) => {
                    self.update_timeout(param, value)
                }
                ParameterChange::Threshold(param, value) => {
                    self.update_threshold(param, value)
                }
                ParameterChange::EnableAnchor(id) => {
                    self.enable_anchor(id)
                }
                ParameterChange::DisableAnchor(id) => {
                    self.disable_anchor(id)
                }
                ParameterChange::AnchorParameter(id, param, value) => {
                    self.update_anchor_parameter(id, param, value)
                }
            };

            if let Err(e) = result {
                // Rollback all changes on any failure
                if let Err(rollback_err) = self.rollback() {
                    return Err(format!("Batch change failed: {}. Rollback also failed: {}", e, rollback_err));
                }
                return Err(format!("Batch change failed at step {}: {}", applied_changes.len() + 1, e));
            }

            applied_changes.push(change);
        }

        self.record_change(
            ChangeType::SystemParameterUpdate,
            format!("Applied batch of {} parameter changes", applied_changes.len()),
            "previous_state".to_string(),
            "new_state".to_string(),
        );

        Ok(())
    }
}

/// Parameter types for timeout updates
#[derive(Debug, Clone, Copy)]
pub enum TimeoutParameter {
    MaxAnchorAge,
    PositionTimeout,
}

/// Parameter types for threshold updates
#[derive(Debug, Clone, Copy)]
pub enum ThresholdParameter {
    AccuracyThreshold,
    MaxRange,
}

/// Anchor-specific parameter types
#[derive(Debug, Clone, Copy)]
pub enum AnchorParameter {
    MaxRange,
    MinSignalQuality,
    TimeoutOverride,
}

/// Values for anchor parameters
#[derive(Debug, Clone, Copy)]
pub enum AnchorParameterValue {
    Float(f32),
    U8(u8),
    U32(u32),
}

/// Batch parameter change operations
#[derive(Debug, Clone)]
pub enum ParameterChange {
    SoundSpeed(f64),
    Timeout(TimeoutParameter, u32),
    Threshold(ThresholdParameter, f32),
    EnableAnchor(u16),
    DisableAnchor(u16),
    AnchorParameter(u16, AnchorParameter, AnchorParameterValue),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_system_config_validation() {
        let mut config = SystemConfig::new();
        assert!(config.validate().is_ok());

        // Test invalid sound speed
        config.sound_speed_m_per_s = 1000.0;  // Too low
        assert!(config.validate().is_err());

        config.sound_speed_m_per_s = 2000.0;  // Too high
        assert!(config.validate().is_err());

        config.sound_speed_m_per_s = 1500.0;  // Valid
        assert!(config.validate().is_ok());
    }

    #[test]
    fn test_anchor_config_validation() {
        let mut anchor = AnchorConfig::new(1, 32.123, 45.456, 10.0);
        assert!(anchor.validate().is_ok());

        // Test invalid latitude
        anchor.position.latitude = 95.0;  // Too high
        assert!(anchor.validate().is_err());

        anchor.position.latitude = 32.123;  // Valid
        assert!(anchor.validate().is_ok());

        // Test negative depth
        anchor.position.depth = -5.0;
        assert!(anchor.validate().is_err());
    }

    #[test]
    fn test_positioning_config() {
        let mut config = PositioningConfig::new();
        
        let anchor1 = AnchorConfig::new(1, 32.123, 45.456, 10.0);
        let anchor2 = AnchorConfig::new(2, 32.124, 45.457, 10.0);
        let anchor3 = AnchorConfig::new(3, 32.125, 45.458, 10.0);

        assert!(config.add_anchor(anchor1).is_ok());
        assert!(config.add_anchor(anchor2).is_ok());
        assert!(config.add_anchor(anchor3).is_ok());

        assert_eq!(config.anchors.len(), 3);
        assert_eq!(config.get_enabled_anchors().len(), 3);

        // Test duplicate ID
        let duplicate_anchor = AnchorConfig::new(1, 32.126, 45.459, 10.0);
        assert!(config.add_anchor(duplicate_anchor).is_err());
    }

    #[test]
    fn test_runtime_parameter_manager() {
        let mut config = PositioningConfig::new();
        // Use better distributed anchor positions to avoid geometry issues
        config.add_anchor(AnchorConfig::new(1, 32.123, 45.456, 10.0)).unwrap();
        config.add_anchor(AnchorConfig::new(2, 32.133, 45.466, 15.0)).unwrap();
        config.add_anchor(AnchorConfig::new(3, 32.113, 45.476, 5.0)).unwrap();
        config.add_anchor(AnchorConfig::new(4, 32.143, 45.446, 20.0)).unwrap();

        let mut manager = RuntimeParameterManager::new(config);

        // Test sound speed update
        assert!(manager.update_sound_speed(1520.0).is_ok());
        assert_eq!(manager.get_config().system.sound_speed_m_per_s, 1520.0);

        // Test invalid sound speed
        assert!(manager.update_sound_speed(1000.0).is_err());
        assert_eq!(manager.get_config().system.sound_speed_m_per_s, 1520.0); // Should remain unchanged

        // Test timeout update
        assert!(manager.update_timeout(TimeoutParameter::MaxAnchorAge, 3000).is_ok());
        assert_eq!(manager.get_config().system.max_anchor_age_ms, 3000);

        // Test anchor enable/disable
        let disable_result = manager.disable_anchor(1);
        if let Err(e) = &disable_result {
            println!("Disable anchor failed: {}", e);
        }
        assert!(disable_result.is_ok());
        assert!(!manager.get_config().get_anchor(1).unwrap().enabled);

        // Test cannot disable too many anchors (now we have 3 enabled, can't disable another)
        let disable2_result = manager.disable_anchor(2);
        if let Err(e) = &disable2_result {
            println!("Disable anchor 2 failed (expected): {}", e);
        }
        assert!(disable2_result.is_err()); // Should fail - would leave < 3 anchors

        // Test rollback
        assert!(manager.rollback().is_ok());
        assert_eq!(manager.get_config().system.sound_speed_m_per_s, 1500.0); // Back to original
    }

    #[test]
    fn test_batch_parameter_changes() {
        let mut config = PositioningConfig::new();
        // Use better distributed anchor positions to avoid geometry issues
        config.add_anchor(AnchorConfig::new(1, 32.123, 45.456, 10.0)).unwrap();
        config.add_anchor(AnchorConfig::new(2, 32.133, 45.466, 15.0)).unwrap();
        config.add_anchor(AnchorConfig::new(3, 32.113, 45.476, 5.0)).unwrap();
        config.add_anchor(AnchorConfig::new(4, 32.143, 45.446, 20.0)).unwrap();

        let mut manager = RuntimeParameterManager::new(config);

        let changes = vec![
            ParameterChange::SoundSpeed(1520.0),
            ParameterChange::Timeout(TimeoutParameter::MaxAnchorAge, 4000),
            ParameterChange::DisableAnchor(4),
        ];

        assert!(manager.apply_batch_changes(changes).is_ok());
        assert_eq!(manager.get_config().system.sound_speed_m_per_s, 1520.0);
        assert_eq!(manager.get_config().system.max_anchor_age_ms, 4000);
        assert!(!manager.get_config().get_anchor(4).unwrap().enabled);

        // Test batch failure with rollback
        let bad_changes = vec![
            ParameterChange::SoundSpeed(1530.0),
            ParameterChange::SoundSpeed(1000.0), // This should fail
        ];

        assert!(manager.apply_batch_changes(bad_changes).is_err());
        // Should be rolled back to previous state
        assert_eq!(manager.get_config().system.sound_speed_m_per_s, 1520.0);
    }

    #[test]
    fn test_config_file_operations() {
        // Test saving and loading configuration files
        let mut config = PositioningConfig::new();
        config.add_anchor(AnchorConfig::new(1, 32.123456, 45.476789, 10.0)).unwrap();
        config.add_anchor(AnchorConfig::new(2, 32.133456, 45.486789, 15.0)).unwrap();
        config.add_anchor(AnchorConfig::new(3, 32.113456, 45.466789, 5.0)).unwrap();

        // Save to file
        let test_file = "test_config_temp.json";
        assert!(config.save_to_file(test_file).is_ok());

        // Load from file
        let loaded_config = PositioningConfig::load_from_file(test_file);
        assert!(loaded_config.is_ok());
        
        let loaded = loaded_config.unwrap();
        assert_eq!(loaded.anchors.len(), 3);
        assert_eq!(loaded.system.sound_speed_m_per_s, config.system.sound_speed_m_per_s);

        // Clean up
        let _ = std::fs::remove_file(test_file);
    }

    #[test]
    fn test_complete_configuration_workflow() {
        // Test the complete configuration workflow from creation to runtime management
        println!("Testing complete configuration workflow...");
        
        // 1. Create configuration
        let mut config = PositioningConfig::new();
        config.add_anchor(AnchorConfig::new(1, 32.123, 45.456, 10.0)).unwrap();
        config.add_anchor(AnchorConfig::new(2, 32.133, 45.466, 15.0)).unwrap();
        config.add_anchor(AnchorConfig::new(3, 32.113, 45.476, 5.0)).unwrap();
        config.add_anchor(AnchorConfig::new(4, 32.143, 45.446, 20.0)).unwrap();
        
        // 2. Validate configuration
        assert!(config.validate().is_ok());
        
        // 3. Create runtime manager
        let mut manager = RuntimeParameterManager::new(config);
        
        // 4. Test parameter updates
        assert!(manager.update_sound_speed(1520.0).is_ok());
        assert!(manager.update_timeout(TimeoutParameter::MaxAnchorAge, 4000).is_ok());
        assert!(manager.update_threshold(ThresholdParameter::AccuracyThreshold, 1.5).is_ok());
        
        // 5. Test anchor management
        assert!(manager.disable_anchor(4).is_ok());
        assert_eq!(manager.get_config().get_enabled_anchors().len(), 3);
        
        // 6. Test rollback
        manager.create_backup();
        assert!(manager.update_sound_speed(1550.0).is_ok());
        assert!(manager.rollback().is_ok());
        assert_eq!(manager.get_config().system.sound_speed_m_per_s, 1520.0);
        
        println!("Complete configuration workflow test passed!");
    }
}