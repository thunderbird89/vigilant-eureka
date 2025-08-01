use std::path::{Path, PathBuf};
use std::collections::HashMap;
use serde::{Serialize, Deserialize};
use uuid::Uuid;
use shared_positioning::{BeaconConfig, GeodeticPosition};
use crate::{EmulatorError, MovementPattern};

/// Configuration manager for beacon emulator
pub struct EmulatorConfigManager {
    /// Default configuration template
    default_config: BeaconConfig,
    /// Configuration validation rules
    validation_rules: ValidationRules,
    /// Migration handlers for different config versions
    migration_handlers: HashMap<String, Box<dyn ConfigMigration>>,
}

/// Validation rules for emulator-specific configuration requirements
#[derive(Debug, Clone)]
pub struct ValidationRules {
    /// Minimum transmission interval in milliseconds
    pub min_transmission_interval_ms: u32,
    /// Maximum transmission interval in milliseconds
    pub max_transmission_interval_ms: u32,
    /// Minimum power level (0-255)
    pub min_power_level: u8,
    /// Maximum power level (0-255)
    pub max_power_level: u8,
    /// Maximum retry attempts
    pub max_retry_attempts: u32,
    /// Minimum GPS accuracy threshold in meters
    pub min_gps_accuracy_threshold_m: f32,
    /// Maximum GPS accuracy threshold in meters
    pub max_gps_accuracy_threshold_m: f32,
    /// Minimum battery threshold percentage
    pub min_battery_threshold_percent: f32,
    /// Maximum battery threshold percentage
    pub max_battery_threshold_percent: f32,
}

impl Default for ValidationRules {
    fn default() -> Self {
        Self {
            min_transmission_interval_ms: 100,
            max_transmission_interval_ms: 300_000, // 5 minutes
            min_power_level: 1,
            max_power_level: 255,
            max_retry_attempts: 10,
            min_gps_accuracy_threshold_m: 0.1,
            max_gps_accuracy_threshold_m: 100.0,
            min_battery_threshold_percent: 1.0,
            max_battery_threshold_percent: 100.0,
        }
    }
}

/// Configuration migration trait for handling different config versions
pub trait ConfigMigration: Send + Sync {
    fn migrate(&self, config: &mut BeaconConfig) -> Result<(), EmulatorError>;
    fn source_version(&self) -> &str;
    fn target_version(&self) -> &str;
}

/// Emulator-specific beacon configuration with additional metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmulatorBeaconConfig {
    /// Base beacon configuration
    pub beacon_config: BeaconConfig,
    /// Initial position for the virtual beacon
    pub initial_position: GeodeticPosition,
    /// Movement pattern configuration
    pub movement_pattern: MovementPattern,
    /// Whether the beacon should start automatically
    pub auto_start: bool,
    /// Custom name/description for the beacon
    pub name: Option<String>,
    /// Tags for organizing beacons
    pub tags: Vec<String>,
    /// Configuration metadata
    pub metadata: EmulatorConfigMetadata,
}

/// Metadata for emulator configuration tracking
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmulatorConfigMetadata {
    /// Configuration schema version
    pub schema_version: String,
    /// Creation timestamp (Unix timestamp)
    pub created_at: u64,
    /// Last modification timestamp (Unix timestamp)
    pub modified_at: u64,
    /// Configuration description
    pub description: String,
    /// Configuration author
    pub author: String,
    /// Configuration checksum for integrity verification
    pub checksum: String,
    /// Migration history
    pub migration_history: Vec<MigrationRecord>,
}

/// Record of a configuration migration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MigrationRecord {
    /// Source version
    pub from_version: String,
    /// Target version
    pub to_version: String,
    /// Migration timestamp
    pub timestamp: u64,
    /// Migration description
    pub description: String,
}

impl EmulatorConfigManager {
    /// Create a new configuration manager
    pub fn new() -> Self {
        let default_config = Self::create_default_beacon_config();
        let validation_rules = ValidationRules::default();
        let migration_handlers = Self::create_migration_handlers();
        
        Self {
            default_config,
            validation_rules,
            migration_handlers,
        }
    }
    
    /// Load beacon configuration from a file with automatic format detection
    pub async fn load_beacon_config(&self, config_path: &Path) -> Result<BeaconConfig, EmulatorError> {
        if !config_path.exists() {
            return Err(EmulatorError::ConfigError(
                format!("Configuration file not found: {}", config_path.display())
            ));
        }
        
        let config_content = tokio::fs::read_to_string(config_path).await
            .map_err(|e| EmulatorError::ConfigError(
                format!("Failed to read configuration file: {}", e)
            ))?;
        
        // Detect file format and parse accordingly
        let mut config = match config_path.extension().and_then(|s| s.to_str()) {
            Some("toml") => {
                toml::from_str::<BeaconConfig>(&config_content)
                    .map_err(|e| EmulatorError::ConfigError(
                        format!("Failed to parse TOML configuration: {}", e)
                    ))?
            }
            Some("json") => {
                serde_json::from_str::<BeaconConfig>(&config_content)
                    .map_err(|e| EmulatorError::ConfigError(
                        format!("Failed to parse JSON configuration: {}", e)
                    ))?
            }
            Some("yaml") | Some("yml") => {
                serde_yaml::from_str::<BeaconConfig>(&config_content)
                    .map_err(|e| EmulatorError::ConfigError(
                        format!("Failed to parse YAML configuration: {}", e)
                    ))?
            }
            _ => {
                // Default to TOML format
                toml::from_str::<BeaconConfig>(&config_content)
                    .map_err(|e| EmulatorError::ConfigError(
                        format!("Failed to parse configuration (assumed TOML): {}", e)
                    ))?
            }
        };
        
        // Apply migrations if needed
        self.apply_migrations(&mut config)?;
        
        // Validate configuration for emulator use
        self.validate_emulator_config(&config)?;
        
        Ok(config)
    }
    
    /// Load emulator-specific beacon configuration with extended metadata
    pub async fn load_emulator_beacon_config(&self, config_path: &Path) -> Result<EmulatorBeaconConfig, EmulatorError> {
        let config_content = tokio::fs::read_to_string(config_path).await
            .map_err(|e| EmulatorError::ConfigError(
                format!("Failed to read emulator configuration file: {}", e)
            ))?;
        
        let emulator_config = match config_path.extension().and_then(|s| s.to_str()) {
            Some("json") => {
                serde_json::from_str::<EmulatorBeaconConfig>(&config_content)
                    .map_err(|e| EmulatorError::ConfigError(
                        format!("Failed to parse emulator JSON configuration: {}", e)
                    ))?
            }
            Some("toml") => {
                toml::from_str::<EmulatorBeaconConfig>(&config_content)
                    .map_err(|e| EmulatorError::ConfigError(
                        format!("Failed to parse emulator TOML configuration: {}", e)
                    ))?
            }
            _ => {
                return Err(EmulatorError::ConfigError(
                    "Emulator configuration files must be .json or .toml format".to_string()
                ));
            }
        };
        
        // Validate the beacon configuration
        self.validate_emulator_config(&emulator_config.beacon_config)?;
        
        // Validate the emulator-specific configuration
        self.validate_emulator_specific_config(&emulator_config)?;
        
        Ok(emulator_config)
    }
    
    /// Save beacon configuration to a file
    pub async fn save_beacon_config(
        &self,
        config: &BeaconConfig,
        config_path: &Path,
    ) -> Result<(), EmulatorError> {
        // Validate configuration before saving
        self.validate_emulator_config(config)?;
        
        // Create parent directory if it doesn't exist
        if let Some(parent) = config_path.parent() {
            tokio::fs::create_dir_all(parent).await
                .map_err(|e| EmulatorError::ConfigError(
                    format!("Failed to create configuration directory: {}", e)
                ))?;
        }
        
        // Serialize based on file extension
        let config_content = match config_path.extension().and_then(|s| s.to_str()) {
            Some("json") => {
                serde_json::to_string_pretty(config)
                    .map_err(|e| EmulatorError::ConfigError(
                        format!("Failed to serialize configuration to JSON: {}", e)
                    ))?
            }
            Some("yaml") | Some("yml") => {
                serde_yaml::to_string(config)
                    .map_err(|e| EmulatorError::ConfigError(
                        format!("Failed to serialize configuration to YAML: {}", e)
                    ))?
            }
            _ => {
                // Default to TOML
                toml::to_string_pretty(config)
                    .map_err(|e| EmulatorError::ConfigError(
                        format!("Failed to serialize configuration to TOML: {}", e)
                    ))?
            }
        };
        
        tokio::fs::write(config_path, config_content).await
            .map_err(|e| EmulatorError::ConfigError(
                format!("Failed to write configuration file: {}", e)
            ))?;
        
        Ok(())
    }
    
    /// Save emulator-specific beacon configuration
    pub async fn save_emulator_beacon_config(
        &self,
        config: &EmulatorBeaconConfig,
        config_path: &Path,
    ) -> Result<(), EmulatorError> {
        // Validate configuration before saving
        self.validate_emulator_config(&config.beacon_config)?;
        self.validate_emulator_specific_config(config)?;
        
        // Create parent directory if it doesn't exist
        if let Some(parent) = config_path.parent() {
            tokio::fs::create_dir_all(parent).await
                .map_err(|e| EmulatorError::ConfigError(
                    format!("Failed to create configuration directory: {}", e)
                ))?;
        }
        
        // Update metadata before saving
        let mut config_to_save = config.clone();
        config_to_save.metadata.modified_at = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs();
        config_to_save.metadata.checksum = self.calculate_config_checksum(&config_to_save)?;
        
        // Serialize based on file extension
        let config_content = match config_path.extension().and_then(|s| s.to_str()) {
            Some("json") => {
                serde_json::to_string_pretty(&config_to_save)
                    .map_err(|e| EmulatorError::ConfigError(
                        format!("Failed to serialize emulator configuration to JSON: {}", e)
                    ))?
            }
            Some("toml") => {
                toml::to_string_pretty(&config_to_save)
                    .map_err(|e| EmulatorError::ConfigError(
                        format!("Failed to serialize emulator configuration to TOML: {}", e)
                    ))?
            }
            _ => {
                return Err(EmulatorError::ConfigError(
                    "Emulator configuration files must be .json or .toml format".to_string()
                ));
            }
        };
        
        tokio::fs::write(config_path, config_content).await
            .map_err(|e| EmulatorError::ConfigError(
                format!("Failed to write emulator configuration file: {}", e)
            ))?;
        
        Ok(())
    }
    
    /// Create a default beacon configuration template for emulator use
    pub fn create_default_beacon_config() -> BeaconConfig {
        let beacon_id = Uuid::new_v4();
        let mut config = BeaconConfig::new(beacon_id);
        
        // Configure for emulator use with reasonable defaults
        config.transmission.interval_ms = 5000; // 5 second interval
        config.transmission.message_version = shared_positioning::BeaconMessageVersion::V3;
        config.transmission.power_level = 255; // Full power for testing
        config.transmission.max_retries = 3;
        config.transmission.retry_delay_ms = 1000;
        config.transmission.adaptive_power = false; // Disable for consistent testing
        
        // GPS configuration for emulation (more lenient)
        config.gps.acquisition_timeout_s = 30;
        config.gps.update_interval_s = 5;
        config.gps.min_satellite_count = 3;
        config.gps.accuracy_threshold_m = 5.0; // More lenient for testing
        config.gps.cold_start_timeout_s = 60;
        config.gps.enable_dgps = false;
        config.gps.max_fix_age_s = 30;
        
        // Power configuration for emulation (disable power saving features)
        config.power.low_battery_threshold_percent = 20.0;
        config.power.critical_battery_threshold_percent = 10.0;
        config.power.emergency_battery_threshold_percent = 5.0;
        config.power.power_save_threshold_percent = 25.0;
        config.power.charging_enabled = false; // Not relevant for virtual beacons
        config.power.solar_charging_enabled = false;
        config.power.monitoring_interval_s = 30;
        
        // Disable power saving modes for consistent testing
        config.power.power_modes.power_save_transmission_multiplier = 1.0;
        config.power.power_modes.emergency_transmission_multiplier = 1.0;
        
        // Communication configuration (disable for emulation)
        config.communication.connection_interval_hours = 24; // Infrequent
        config.communication.max_retry_attempts = 1;
        config.communication.initial_retry_backoff_ms = 5000;
        config.communication.max_retry_interval_hours = 1;
        config.communication.connection_timeout_s = 30;
        config.communication.compression_enabled = false;
        
        // Emergency configuration
        config.emergency.emergency_mode_enabled = false; // Disable for testing
        config.emergency.emergency_interval_ms = 1000;
        config.emergency.emergency_power_boost_percent = 150;
        config.emergency.max_emergency_duration_minutes = 30;
        config.emergency.auto_recovery_enabled = false;
        
        config
    }
    
    /// Create a default emulator beacon configuration
    pub fn create_default_emulator_config(
        beacon_id: Option<Uuid>,
        position: GeodeticPosition,
    ) -> EmulatorBeaconConfig {
        let mut beacon_config = Self::create_default_beacon_config();
        if let Some(id) = beacon_id {
            beacon_config.beacon_id = id;
        }
        
        let now = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs();
        
        EmulatorBeaconConfig {
            beacon_config,
            initial_position: position,
            movement_pattern: MovementPattern::Stationary,
            auto_start: false,
            name: None,
            tags: Vec::new(),
            metadata: EmulatorConfigMetadata {
                schema_version: "1.0.0".to_string(),
                created_at: now,
                modified_at: now,
                description: "Default emulator beacon configuration".to_string(),
                author: "Beacon Emulator".to_string(),
                checksum: String::new(), // Will be calculated when saved
                migration_history: Vec::new(),
            },
        }
    }
    
    /// Validate beacon configuration for emulator use
    pub fn validate_emulator_config(&self, config: &BeaconConfig) -> Result<(), EmulatorError> {
        let mut errors = Vec::new();
        
        // Validate beacon ID
        if config.beacon_id.is_nil() {
            errors.push("Beacon ID cannot be nil".to_string());
        }
        
        // Validate transmission configuration
        if let Err(e) = self.validate_transmission_config(&config.transmission) {
            errors.extend(e);
        }
        
        // Validate GPS configuration
        if let Err(e) = self.validate_gps_config(&config.gps) {
            errors.extend(e);
        }
        
        // Validate power configuration
        if let Err(e) = self.validate_power_config(&config.power) {
            errors.extend(e);
        }
        
        // Validate communication configuration
        if let Err(e) = self.validate_communication_config(&config.communication) {
            errors.extend(e);
        }
        
        // Validate emergency configuration
        if let Err(e) = self.validate_emergency_config(&config.emergency) {
            errors.extend(e);
        }
        
        if !errors.is_empty() {
            return Err(EmulatorError::ConfigError(
                format!("Configuration validation failed: {}", errors.join(", "))
            ));
        }
        
        Ok(())
    }
    
    /// Validate emulator-specific configuration
    pub fn validate_emulator_specific_config(&self, config: &EmulatorBeaconConfig) -> Result<(), EmulatorError> {
        let mut errors = Vec::new();
        
        // Validate position
        if config.initial_position.latitude < -90.0 || config.initial_position.latitude > 90.0 {
            errors.push(format!("Invalid latitude: {}", config.initial_position.latitude));
        }
        
        if config.initial_position.longitude < -180.0 || config.initial_position.longitude > 180.0 {
            errors.push(format!("Invalid longitude: {}", config.initial_position.longitude));
        }
        
        if config.initial_position.depth < 0.0 || config.initial_position.depth > 11000.0 {
            errors.push(format!("Invalid depth: {} (must be 0-11000m)", config.initial_position.depth));
        }
        
        // Validate movement pattern
        if let Err(e) = crate::movement::MovementPatternValidator::validate_pattern(&config.movement_pattern) {
            errors.push(format!("Invalid movement pattern: {}", e));
        }
        
        // Validate metadata
        if config.metadata.schema_version.is_empty() {
            errors.push("Schema version cannot be empty".to_string());
        }
        
        if config.metadata.description.is_empty() {
            errors.push("Description cannot be empty".to_string());
        }
        
        if !errors.is_empty() {
            return Err(EmulatorError::ConfigError(
                format!("Emulator configuration validation failed: {}", errors.join(", "))
            ));
        }
        
        Ok(())
    }
    
    /// Apply configuration migrations if needed
    fn apply_migrations(&self, config: &mut BeaconConfig) -> Result<(), EmulatorError> {
        let current_version = &config.metadata.schema_version;
        
        // Check if migration is needed
        if current_version == "1.0.0" {
            // Already at current version
            return Ok(());
        }
        
        // Apply migrations in sequence
        for (version, migration) in &self.migration_handlers {
            if migration.source_version() == current_version {
                migration.migrate(config)?;
                
                // Update metadata
                config.metadata.schema_version = migration.target_version().to_string();
                config.metadata.modified_at = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap()
                    .as_secs();
                
                // Add migration record
                config.metadata.migration_history.push(shared_positioning::beacon_config::MigrationRecord {
                    from_version: migration.source_version().to_string(),
                    to_version: migration.target_version().to_string(),
                    migrated_at: config.metadata.modified_at,
                    description: format!("Migrated from {} to {}", 
                                       migration.source_version(), 
                                       migration.target_version()),
                    success: true,
                });
                
                tracing::info!("Applied configuration migration from {} to {}", 
                             migration.source_version(), 
                             migration.target_version());
                break;
            }
        }
        
        Ok(())
    }
    
    /// Create migration handlers for different configuration versions
    fn create_migration_handlers() -> HashMap<String, Box<dyn ConfigMigration>> {
        let handlers: HashMap<String, Box<dyn ConfigMigration>> = HashMap::new();
        
        // Add migration handlers as needed
        // Example: handlers.insert("0.9.0".to_string(), Box::new(Migration_0_9_to_1_0));
        
        handlers
    }
    
    /// Calculate configuration checksum for integrity verification
    fn calculate_config_checksum(&self, config: &EmulatorBeaconConfig) -> Result<String, EmulatorError> {
        // Create a copy without the checksum field for calculation
        let mut config_for_checksum = config.clone();
        config_for_checksum.metadata.checksum = String::new();
        
        let serialized = serde_json::to_string(&config_for_checksum)
            .map_err(|e| EmulatorError::ConfigError(
                format!("Failed to serialize config for checksum: {}", e)
            ))?;
        
        // Simple checksum using hash of serialized data
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};
        
        let mut hasher = DefaultHasher::new();
        serialized.hash(&mut hasher);
        let checksum = hasher.finish();
        
        Ok(format!("{:016x}", checksum))
    }
    
    /// Validate transmission configuration
    fn validate_transmission_config(&self, config: &shared_positioning::BeaconTransmissionConfig) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();
        
        if config.interval_ms < self.validation_rules.min_transmission_interval_ms {
            errors.push(format!(
                "Transmission interval too short: {}ms (minimum: {}ms)",
                config.interval_ms,
                self.validation_rules.min_transmission_interval_ms
            ));
        }
        
        if config.interval_ms > self.validation_rules.max_transmission_interval_ms {
            errors.push(format!(
                "Transmission interval too long: {}ms (maximum: {}ms)",
                config.interval_ms,
                self.validation_rules.max_transmission_interval_ms
            ));
        }
        
        if config.power_level < self.validation_rules.min_power_level {
            errors.push(format!(
                "Power level too low: {} (minimum: {})",
                config.power_level,
                self.validation_rules.min_power_level
            ));
        }
        
        if config.power_level > self.validation_rules.max_power_level {
            errors.push(format!(
                "Power level too high: {} (maximum: {})",
                config.power_level,
                self.validation_rules.max_power_level
            ));
        }
        
        if u32::from(config.max_retries) > self.validation_rules.max_retry_attempts {
            errors.push(format!(
                "Too many retry attempts: {} (maximum: {})",
                config.max_retries,
                self.validation_rules.max_retry_attempts
            ));
        }
        
        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }
    
    /// Validate GPS configuration
    fn validate_gps_config(&self, config: &shared_positioning::BeaconGpsConfig) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();
        
        if config.acquisition_timeout_s < 10 {
            errors.push(format!(
                "GPS acquisition timeout too short: {}s (minimum: 10s)",
                config.acquisition_timeout_s
            ));
        }
        
        if config.acquisition_timeout_s > 300 {
            errors.push(format!(
                "GPS acquisition timeout too long: {}s (maximum: 300s)",
                config.acquisition_timeout_s
            ));
        }
        
        if config.update_interval_s < 1 {
            errors.push(format!(
                "GPS update interval too short: {}s (minimum: 1s)",
                config.update_interval_s
            ));
        }
        
        if config.min_satellite_count < 3 {
            errors.push(format!(
                "Minimum satellite count too low: {} (minimum: 3)",
                config.min_satellite_count
            ));
        }
        
        if config.accuracy_threshold_m < self.validation_rules.min_gps_accuracy_threshold_m {
            errors.push(format!(
                "GPS accuracy threshold too strict: {:.1}m (minimum: {:.1}m)",
                config.accuracy_threshold_m,
                self.validation_rules.min_gps_accuracy_threshold_m
            ));
        }
        
        if config.accuracy_threshold_m > self.validation_rules.max_gps_accuracy_threshold_m {
            errors.push(format!(
                "GPS accuracy threshold too lenient: {:.1}m (maximum: {:.1}m)",
                config.accuracy_threshold_m,
                self.validation_rules.max_gps_accuracy_threshold_m
            ));
        }
        
        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }
    
    /// Validate power configuration
    fn validate_power_config(&self, config: &shared_positioning::BeaconPowerConfig) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();
        
        if config.low_battery_threshold_percent <= config.critical_battery_threshold_percent {
            errors.push(format!(
                "Low battery threshold ({:.1}%) must be higher than critical threshold ({:.1}%)",
                config.low_battery_threshold_percent,
                config.critical_battery_threshold_percent
            ));
        }
        
        if config.critical_battery_threshold_percent <= config.emergency_battery_threshold_percent {
            errors.push(format!(
                "Critical battery threshold ({:.1}%) must be higher than emergency threshold ({:.1}%)",
                config.critical_battery_threshold_percent,
                config.emergency_battery_threshold_percent
            ));
        }
        
        if config.emergency_battery_threshold_percent < self.validation_rules.min_battery_threshold_percent {
            errors.push(format!(
                "Emergency battery threshold too low: {:.1}% (minimum: {:.1}%)",
                config.emergency_battery_threshold_percent,
                self.validation_rules.min_battery_threshold_percent
            ));
        }
        
        if config.low_battery_threshold_percent > self.validation_rules.max_battery_threshold_percent {
            errors.push(format!(
                "Low battery threshold too high: {:.1}% (maximum: {:.1}%)",
                config.low_battery_threshold_percent,
                self.validation_rules.max_battery_threshold_percent
            ));
        }
        
        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }
    
    /// Validate communication configuration
    fn validate_communication_config(&self, config: &shared_positioning::BeaconCommunicationConfig) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();
        
        if config.connection_interval_hours < 1 {
            errors.push(format!(
                "Connection interval too short: {}h (minimum: 1h)",
                config.connection_interval_hours
            ));
        }
        
        if config.connection_interval_hours > 168 {
            errors.push(format!(
                "Connection interval too long: {}h (maximum: 168h/1 week)",
                config.connection_interval_hours
            ));
        }
        
        if config.max_retry_attempts > 10 {
            errors.push(format!(
                "Too many retry attempts: {} (maximum: 10)",
                config.max_retry_attempts
            ));
        }
        
        if config.connection_timeout_s < 10 {
            errors.push(format!(
                "Connection timeout too short: {}s (minimum: 10s)",
                config.connection_timeout_s
            ));
        }
        
        if config.connection_timeout_s > 300 {
            errors.push(format!(
                "Connection timeout too long: {}s (maximum: 300s)",
                config.connection_timeout_s
            ));
        }
        
        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }
    
    /// Validate emergency configuration
    fn validate_emergency_config(&self, config: &shared_positioning::EmergencyConfig) -> Result<(), Vec<String>> {
        let mut errors = Vec::new();
        
        if config.emergency_interval_ms < 500 {
            errors.push(format!(
                "Emergency transmission interval too short: {}ms (minimum: 500ms)",
                config.emergency_interval_ms
            ));
        }
        
        if config.emergency_interval_ms > 10000 {
            errors.push(format!(
                "Emergency transmission interval too long: {}ms (maximum: 10000ms)",
                config.emergency_interval_ms
            ));
        }
        
        if config.shutdown_conditions.shutdown_grace_period_s < 5 {
            errors.push(format!(
                "Shutdown grace period too short: {}s (minimum: 5s)",
                config.shutdown_conditions.shutdown_grace_period_s
            ));
        }
        
        if config.shutdown_conditions.shutdown_grace_period_s > 300 {
            errors.push(format!(
                "Shutdown grace period too long: {}s (maximum: 300s)",
                config.shutdown_conditions.shutdown_grace_period_s
            ));
        }
        
        if errors.is_empty() {
            Ok(())
        } else {
            Err(errors)
        }
    }
    
    /// Get validation rules
    pub fn get_validation_rules(&self) -> &ValidationRules {
        &self.validation_rules
    }
    
    /// Update validation rules
    pub fn set_validation_rules(&mut self, rules: ValidationRules) {
        self.validation_rules = rules;
    }
    
    /// Generate configuration template files
    pub async fn generate_config_template(
        &self,
        template_path: &Path,
        format: ConfigFormat,
    ) -> Result<(), EmulatorError> {
        let template_config = Self::create_default_beacon_config();
        
        match format {
            ConfigFormat::Toml => {
                self.save_beacon_config(&template_config, &template_path.with_extension("toml")).await?;
            }
            ConfigFormat::Json => {
                self.save_beacon_config(&template_config, &template_path.with_extension("json")).await?;
            }
            ConfigFormat::Yaml => {
                self.save_beacon_config(&template_config, &template_path.with_extension("yaml")).await?;
            }
        }
        
        Ok(())
    }
    
    /// Generate emulator configuration template
    pub async fn generate_emulator_config_template(
        &self,
        template_path: &Path,
        position: GeodeticPosition,
        format: ConfigFormat,
    ) -> Result<(), EmulatorError> {
        let template_config = Self::create_default_emulator_config(None, position);
        
        match format {
            ConfigFormat::Toml => {
                self.save_emulator_beacon_config(&template_config, &template_path.with_extension("toml")).await?;
            }
            ConfigFormat::Json => {
                self.save_emulator_beacon_config(&template_config, &template_path.with_extension("json")).await?;
            }
            ConfigFormat::Yaml => {
                return Err(EmulatorError::ConfigError(
                    "YAML format not supported for emulator configurations".to_string()
                ));
            }
        }
        
        Ok(())
    }
}

/// Configuration file formats
#[derive(Debug, Clone, Copy)]
pub enum ConfigFormat {
    Toml,
    Json,
    Yaml,
}

impl Default for EmulatorConfigManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::NamedTempFile;
    use std::io::Write;

    #[tokio::test]
    async fn test_config_manager_creation() {
        let manager = EmulatorConfigManager::new();
        let rules = manager.get_validation_rules();
        
        assert_eq!(rules.min_transmission_interval_ms, 100);
        assert_eq!(rules.max_transmission_interval_ms, 300_000);
    }

    #[tokio::test]
    async fn test_default_config_creation() {
        let config = EmulatorConfigManager::create_default_beacon_config();
        
        assert_eq!(config.transmission.interval_ms, 5000);
        assert_eq!(config.transmission.power_level, 255);
        assert!(!config.beacon_id.is_nil());
    }

    #[tokio::test]
    async fn test_config_validation() {
        let manager = EmulatorConfigManager::new();
        let config = EmulatorConfigManager::create_default_beacon_config();
        
        // Valid configuration should pass
        assert!(manager.validate_emulator_config(&config).is_ok());
        
        // Invalid configuration should fail
        let mut invalid_config = config.clone();
        invalid_config.transmission.interval_ms = 50; // Too short
        
        assert!(manager.validate_emulator_config(&invalid_config).is_err());
    }

    #[tokio::test]
    async fn test_config_file_loading_toml() {
        let manager = EmulatorConfigManager::new();
        
        // Create a temporary TOML config file
        let mut temp_file = NamedTempFile::new().unwrap();
        let config_content = r#"
beacon_id = "550e8400-e29b-41d4-a716-446655440000"

[transmission]
interval_ms = 3000
message_version = "V2"
power_level = 200
max_retries = 3
retry_delay_ms = 1000
adaptive_power = false
sequence_rollover = 65535

[gps]
acquisition_timeout_s = 60
update_interval_s = 2
min_satellite_count = 4
accuracy_threshold_m = 3.0
cold_start_timeout_s = 120
enable_dgps = false
max_fix_age_s = 30

[power]
low_battery_threshold_percent = 20.0
critical_battery_threshold_percent = 10.0
emergency_battery_threshold_percent = 5.0
power_save_threshold_percent = 25.0
charging_enabled = true
solar_charging_enabled = false
monitoring_interval_s = 30

[power.temperature_limits]
min_operating_c = -10.0
max_operating_c = 50.0
warning_threshold_c = 55.0
emergency_threshold_c = 65.0

[power.power_modes]
normal_transmission_multiplier = 1.0
power_save_transmission_multiplier = 2.0
emergency_transmission_multiplier = 0.5

[power.power_modes.cpu_scaling]
normal_frequency_percent = 100
power_save_frequency_percent = 60
emergency_frequency_percent = 80

[communication]
connection_interval_hours = 6
max_retry_attempts = 5
initial_retry_backoff_ms = 2000
max_retry_interval_hours = 4
connection_timeout_s = 120
compression_enabled = true

[communication.status_report]
include_position_history = true
max_position_history = 100
include_battery_details = true
include_transmission_stats = true
include_system_health = false

[communication.endpoints]
primary_url = "https://api.example.com/beacon"
api_key = "your-api-key"
device_token = "your-device-token"

[emergency]
emergency_mode_enabled = true
emergency_interval_ms = 1000
emergency_power_boost_percent = 150
max_emergency_duration_minutes = 30
auto_recovery_enabled = false

[emergency.shutdown_conditions]
battery_shutdown_enabled = true
temperature_shutdown_enabled = true
hardware_fault_shutdown_enabled = true
shutdown_grace_period_s = 60

[hardware.gpio_pins]
status_led_pin = 2

[hardware.spi_config]
clock_frequency_hz = 1000000
mode = 0
msb_first = true

[hardware.i2c_config]
clock_frequency_hz = 100000
timeout_ms = 1000

[hardware.memory_config]
max_heap_usage_percent = 70
stack_size_bytes = 4096
memory_monitoring_enabled = false
memory_check_interval_s = 60

[hardware.watchdog_config]
enabled = false
timeout_s = 5
auto_reset_enabled = true

[metadata]
schema_version = "1.0.0"
created_at = 1753468665
modified_at = 1753468665
description = "Test beacon configuration"
author = "Test"
checksum = "9a8c09679ab4b503"
migration_history = []
"#;
        
        temp_file.write_all(config_content.as_bytes()).unwrap();
        let temp_path = temp_file.path().with_extension("toml");
        std::fs::copy(temp_file.path(), &temp_path).unwrap();
        
        // Load the configuration
        let loaded_config = manager.load_beacon_config(&temp_path).await.unwrap();
        
        assert_eq!(loaded_config.transmission.interval_ms, 3000);
        assert_eq!(loaded_config.transmission.power_level, 200);
        
        // Clean up
        std::fs::remove_file(temp_path).ok();
    }

    #[tokio::test]
    async fn test_config_file_saving() {
        let manager = EmulatorConfigManager::new();
        let config = EmulatorConfigManager::create_default_beacon_config();
        
        let temp_file = NamedTempFile::new().unwrap();
        let temp_path = temp_file.path().with_extension("toml");
        
        // Save configuration
        manager.save_beacon_config(&config, &temp_path).await.unwrap();
        
        // Verify file was created
        assert!(temp_path.exists());
        
        // Load it back and verify
        let loaded_config = manager.load_beacon_config(&temp_path).await.unwrap();
        assert_eq!(loaded_config.beacon_id, config.beacon_id);
        assert_eq!(loaded_config.transmission.interval_ms, config.transmission.interval_ms);
        
        // Clean up
        std::fs::remove_file(temp_path).ok();
    }

    #[tokio::test]
    async fn test_emulator_config_creation() {
        let position = GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        };
        
        let emulator_config = EmulatorConfigManager::create_default_emulator_config(None, position);
        
        assert_eq!(emulator_config.initial_position.latitude, position.latitude);
        assert_eq!(emulator_config.initial_position.longitude, position.longitude);
        assert_eq!(emulator_config.initial_position.depth, position.depth);
        assert!(matches!(emulator_config.movement_pattern, MovementPattern::Stationary));
        assert!(!emulator_config.auto_start);
        assert_eq!(emulator_config.metadata.schema_version, "1.0.0");
    }

    #[tokio::test]
    async fn test_emulator_config_validation() {
        let manager = EmulatorConfigManager::new();
        let position = GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        };
        
        let config = EmulatorConfigManager::create_default_emulator_config(None, position);
        
        // Valid configuration should pass
        assert!(manager.validate_emulator_specific_config(&config).is_ok());
        
        // Invalid position should fail
        let mut invalid_config = config.clone();
        invalid_config.initial_position.latitude = 100.0; // Invalid latitude
        
        assert!(manager.validate_emulator_specific_config(&invalid_config).is_err());
    }

    #[tokio::test]
    async fn test_template_generation() {
        let manager = EmulatorConfigManager::new();
        let temp_file = NamedTempFile::new().unwrap();
        let temp_path = temp_file.path().with_extension("toml");
        
        // Generate template
        manager.generate_config_template(&temp_path, ConfigFormat::Toml).await.unwrap();
        
        // Verify template was created
        assert!(temp_path.exists());
        
        // Load and validate template
        let template_config = manager.load_beacon_config(&temp_path).await.unwrap();
        assert!(manager.validate_emulator_config(&template_config).is_ok());
        
        // Clean up
        std::fs::remove_file(temp_path).ok();
    }
}