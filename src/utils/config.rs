use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs;
use std::path::Path;

/// System-wide configuration parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SystemConfig {
    /// Sound speed in water (m/s)
    pub sound_speed_ms: f32,
    /// Maximum age for anchor messages before they're considered stale (milliseconds)
    pub max_anchor_age_ms: u32,
    /// Minimum number of anchors required for positioning
    pub min_anchors: u8,
    /// Maximum time to wait for position calculation (milliseconds)
    pub position_timeout_ms: u32,
    /// Accuracy threshold for position validation (meters)
    pub accuracy_threshold_m: f32,
    /// Maximum computation time allowed per position calculation (milliseconds)
    pub max_computation_time_ms: u32,
    /// Target position update rate (Hz)
    pub target_update_rate_hz: f32,
    /// Memory usage limit (bytes)
    pub memory_limit_bytes: u32,
    /// Enable debug logging
    pub debug_logging: bool,
    /// Coordinate system configuration
    pub coordinate_system: CoordinateSystemConfig,
    /// Environmental correction settings
    pub environmental_corrections: EnvironmentalConfig,
}

/// Individual anchor configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnchorConfig {
    /// Unique anchor identifier
    pub id: u16,
    /// Anchor position in geodetic coordinates
    pub position: GeodeticPosition,
    /// Maximum expected range from this anchor (meters)
    pub max_range_m: f32,
    /// Whether this anchor is enabled for positioning
    pub enabled: bool,
    /// Signal quality weight (0.0 to 1.0)
    pub quality_weight: f32,
    /// Anchor-specific timeout (milliseconds, overrides system default if set)
    pub timeout_override_ms: Option<u32>,
    /// Expected signal strength at reference distance
    pub reference_signal_strength: Option<u8>,
    /// Calibration offset for range measurements (meters)
    pub range_calibration_offset_m: f32,
}

/// Geodetic position representation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeodeticPosition {
    /// Latitude in decimal degrees
    pub latitude: f64,
    /// Longitude in decimal degrees  
    pub longitude: f64,
    /// Depth below surface in meters (positive down)
    pub depth: f64,
}

/// Coordinate system configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CoordinateSystemConfig {
    /// Reference point for local tangent plane conversions
    pub reference_point: GeodeticPosition,
    /// Earth model parameters
    pub earth_model: EarthModel,
    /// Local grid system settings
    pub local_grid: Option<LocalGridConfig>,
}

/// Earth model parameters for coordinate transformations
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EarthModel {
    /// Semi-major axis (meters)
    pub semi_major_axis: f64,
    /// Flattening factor
    pub flattening: f64,
    /// Eccentricity squared
    pub eccentricity_squared: f64,
}

/// Local grid system configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LocalGridConfig {
    /// Grid origin in geodetic coordinates
    pub origin: GeodeticPosition,
    /// Grid rotation angle (radians)
    pub rotation_angle: f64,
    /// Grid scale factor
    pub scale_factor: f64,
}

/// Environmental correction configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnvironmentalConfig {
    /// Enable automatic sound speed corrections
    pub auto_sound_speed_correction: bool,
    /// Default water temperature (Celsius)
    pub default_temperature_c: f32,
    /// Default salinity (PSU - Practical Salinity Units)
    pub default_salinity_psu: f32,
    /// Default pressure (bar)
    pub default_pressure_bar: f32,
    /// Sound speed correction coefficients
    pub sound_speed_coefficients: SoundSpeedCoefficients,
}

/// Coefficients for sound speed calculation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SoundSpeedCoefficients {
    /// Temperature coefficient
    pub temperature: f32,
    /// Salinity coefficient  
    pub salinity: f32,
    /// Pressure coefficient
    pub pressure: f32,
    /// Cross-term coefficients
    pub temperature_salinity: f32,
    pub temperature_pressure: f32,
    pub salinity_pressure: f32,
}

/// Configuration validation errors
#[derive(Debug, Clone)]
pub enum ConfigError {
    /// Invalid parameter value
    InvalidParameter { parameter: String, value: String, reason: String },
    /// Missing required parameter
    MissingParameter { parameter: String },
    /// Configuration file I/O error
    IoError { message: String },
    /// JSON serialization/deserialization error
    SerializationError { message: String },
    /// Anchor configuration conflict
    AnchorConflict { anchor_id: u16, reason: String },
    /// Geometry validation failure
    GeometryValidation { reason: String },
}

/// Configuration validation result
#[derive(Debug)]
pub struct ValidationResult {
    /// Whether configuration is valid
    pub is_valid: bool,
    /// Validation errors
    pub errors: Vec<ConfigError>,
    /// Validation warnings
    pub warnings: Vec<String>,
    /// Suggested corrections
    pub suggestions: Vec<String>,
}

/// Main configuration manager
pub struct ConfigurationManager {
    /// Current system configuration
    system_config: SystemConfig,
    /// Anchor configurations indexed by ID
    anchor_configs: HashMap<u16, AnchorConfig>,
    /// Configuration file path
    config_file_path: Option<String>,
    /// Whether configuration has been modified
    is_modified: bool,
}

impl Default for SystemConfig {
    fn default() -> Self {
        Self {
            sound_speed_ms: 1500.0,
            max_anchor_age_ms: 60000,  // 1 minute
            min_anchors: 3,
            position_timeout_ms: 200,
            accuracy_threshold_m: 10.0,
            max_computation_time_ms: 100,
            target_update_rate_hz: 5.0,
            memory_limit_bytes: 32 * 1024,  // 32KB
            debug_logging: false,
            coordinate_system: CoordinateSystemConfig::default(),
            environmental_corrections: EnvironmentalConfig::default(),
        }
    }
}

impl Default for CoordinateSystemConfig {
    fn default() -> Self {
        Self {
            reference_point: GeodeticPosition {
                latitude: 0.0,
                longitude: 0.0,
                depth: 0.0,
            },
            earth_model: EarthModel::wgs84(),
            local_grid: None,
        }
    }
}

impl EarthModel {
    /// WGS84 Earth model parameters
    pub fn wgs84() -> Self {
        Self {
            semi_major_axis: 6378137.0,
            flattening: 1.0 / 298.257223563,
            eccentricity_squared: 0.00669437999014,
        }
    }
}

impl Default for EnvironmentalConfig {
    fn default() -> Self {
        Self {
            auto_sound_speed_correction: true,
            default_temperature_c: 15.0,
            default_salinity_psu: 35.0,
            default_pressure_bar: 1.0,
            sound_speed_coefficients: SoundSpeedCoefficients::default(),
        }
    }
}

impl Default for SoundSpeedCoefficients {
    fn default() -> Self {
        // Simplified coefficients based on empirical formulas
        Self {
            temperature: 4.0,
            salinity: 1.3,
            pressure: 0.16,
            temperature_salinity: -0.01,
            temperature_pressure: 0.0001,
            salinity_pressure: 0.0001,
        }
    }
}

impl ConfigurationManager {
    /// Create a new configuration manager with default settings
    pub fn new() -> Self {
        Self {
            system_config: SystemConfig::default(),
            anchor_configs: HashMap::new(),
            config_file_path: None,
            is_modified: false,
        }
    }

    /// Create configuration manager and load from file
    pub fn from_file<P: AsRef<Path>>(path: P) -> Result<Self, ConfigError> {
        let mut manager = Self::new();
        manager.load_from_file(path)?;
        Ok(manager)
    }

    /// Get current system configuration
    pub fn get_system_config(&self) -> &SystemConfig {
        &self.system_config
    }

    /// Update system configuration with validation
    pub fn update_system_config(&mut self, config: SystemConfig) -> Result<(), ConfigError> {
        let validation = self.validate_system_config(&config)?;
        if !validation.is_valid {
            return Err(validation.errors.into_iter().next().unwrap_or(
                ConfigError::InvalidParameter {
                    parameter: "system_config".to_string(),
                    value: "invalid".to_string(),
                    reason: "Configuration validation failed".to_string(),
                }
            ));
        }

        self.system_config = config;
        self.is_modified = true;
        Ok(())
    }

    /// Get anchor configuration by ID
    pub fn get_anchor_config(&self, anchor_id: u16) -> Option<&AnchorConfig> {
        self.anchor_configs.get(&anchor_id)
    }

    /// Get all anchor configurations
    pub fn get_all_anchor_configs(&self) -> &HashMap<u16, AnchorConfig> {
        &self.anchor_configs
    }

    /// Add or update anchor configuration
    pub fn set_anchor_config(&mut self, config: AnchorConfig) -> Result<(), ConfigError> {
        let validation = self.validate_anchor_config(&config)?;
        if !validation.is_valid {
            return Err(validation.errors.into_iter().next().unwrap_or(
                ConfigError::AnchorConflict {
                    anchor_id: config.id,
                    reason: "Anchor configuration validation failed".to_string(),
                }
            ));
        }

        self.anchor_configs.insert(config.id, config);
        self.is_modified = true;
        Ok(())
    }

    /// Remove anchor configuration
    pub fn remove_anchor_config(&mut self, anchor_id: u16) -> Option<AnchorConfig> {
        self.is_modified = true;
        self.anchor_configs.remove(&anchor_id)
    }

    /// Get list of enabled anchor IDs
    pub fn get_enabled_anchors(&self) -> Vec<u16> {
        self.anchor_configs
            .values()
            .filter(|config| config.enabled)
            .map(|config| config.id)
            .collect()
    }

    /// Load configuration from JSON file
    pub fn load_from_file<P: AsRef<Path>>(&mut self, path: P) -> Result<(), ConfigError> {
        let path_str = path.as_ref().to_string_lossy().to_string();
        
        let content = fs::read_to_string(&path)
            .map_err(|e| ConfigError::IoError {
                message: format!("Failed to read config file '{}': {}", path_str, e),
            })?;

        let config_data: ConfigFileData = serde_json::from_str(&content)
            .map_err(|e| ConfigError::SerializationError {
                message: format!("Failed to parse config file '{}': {}", path_str, e),
            })?;

        // Validate before applying
        let system_validation = self.validate_system_config(&config_data.system)?;
        if !system_validation.is_valid {
            return Err(system_validation.errors.into_iter().next().unwrap());
        }

        // Validate all anchors
        for anchor in &config_data.anchors {
            let anchor_validation = self.validate_anchor_config(anchor)?;
            if !anchor_validation.is_valid {
                return Err(anchor_validation.errors.into_iter().next().unwrap());
            }
        }

        // Apply configuration
        self.system_config = config_data.system;
        self.anchor_configs.clear();
        for anchor in config_data.anchors {
            self.anchor_configs.insert(anchor.id, anchor);
        }

        self.config_file_path = Some(path_str);
        self.is_modified = false;
        Ok(())
    }

    /// Save configuration to JSON file
    pub fn save_to_file<P: AsRef<Path>>(&mut self, path: P) -> Result<(), ConfigError> {
        let path_str = path.as_ref().to_string_lossy().to_string();
        
        let config_data = ConfigFileData {
            system: self.system_config.clone(),
            anchors: self.anchor_configs.values().cloned().collect(),
        };

        let content = serde_json::to_string_pretty(&config_data)
            .map_err(|e| ConfigError::SerializationError {
                message: format!("Failed to serialize config: {}", e),
            })?;

        fs::write(&path, content)
            .map_err(|e| ConfigError::IoError {
                message: format!("Failed to write config file '{}': {}", path_str, e),
            })?;

        self.config_file_path = Some(path_str);
        self.is_modified = false;
        Ok(())
    }

    /// Save to the currently loaded file path
    pub fn save(&mut self) -> Result<(), ConfigError> {
        if let Some(path) = self.config_file_path.clone() {
            self.save_to_file(path)
        } else {
            Err(ConfigError::IoError {
                message: "No file path set for saving configuration".to_string(),
            })
        }
    }

    /// Check if configuration has been modified since last save
    pub fn is_modified(&self) -> bool {
        self.is_modified
    }

    // Runtime Parameter Adjustment Methods

    /// Update sound speed parameter with validation
    pub fn set_sound_speed(&mut self, sound_speed_ms: f32) -> Result<f32, ConfigError> {
        let old_value = self.system_config.sound_speed_ms;
        
        if sound_speed_ms < 1400.0 || sound_speed_ms > 1600.0 {
            return Err(ConfigError::InvalidParameter {
                parameter: "sound_speed_ms".to_string(),
                value: sound_speed_ms.to_string(),
                reason: "Sound speed must be between 1400-1600 m/s for underwater environments".to_string(),
            });
        }

        self.system_config.sound_speed_ms = sound_speed_ms;
        self.is_modified = true;
        Ok(old_value)
    }

    /// Get current sound speed
    pub fn get_sound_speed(&self) -> f32 {
        self.system_config.sound_speed_ms
    }

    /// Update position timeout with validation
    pub fn set_position_timeout(&mut self, timeout_ms: u32) -> Result<u32, ConfigError> {
        let old_value = self.system_config.position_timeout_ms;
        
        if timeout_ms < 50 {
            return Err(ConfigError::InvalidParameter {
                parameter: "position_timeout_ms".to_string(),
                value: timeout_ms.to_string(),
                reason: "Position timeout too short for realistic computation".to_string(),
            });
        }

        if timeout_ms > 10000 {
            return Err(ConfigError::InvalidParameter {
                parameter: "position_timeout_ms".to_string(),
                value: timeout_ms.to_string(),
                reason: "Position timeout too long for real-time operation".to_string(),
            });
        }

        self.system_config.position_timeout_ms = timeout_ms;
        self.is_modified = true;
        Ok(old_value)
    }

    /// Get current position timeout
    pub fn get_position_timeout(&self) -> u32 {
        self.system_config.position_timeout_ms
    }

    /// Update accuracy threshold with validation
    pub fn set_accuracy_threshold(&mut self, threshold_m: f32) -> Result<f32, ConfigError> {
        let old_value = self.system_config.accuracy_threshold_m;
        
        if threshold_m <= 0.0 {
            return Err(ConfigError::InvalidParameter {
                parameter: "accuracy_threshold_m".to_string(),
                value: threshold_m.to_string(),
                reason: "Accuracy threshold must be positive".to_string(),
            });
        }

        if threshold_m > 1000.0 {
            return Err(ConfigError::InvalidParameter {
                parameter: "accuracy_threshold_m".to_string(),
                value: threshold_m.to_string(),
                reason: "Accuracy threshold too large for practical use".to_string(),
            });
        }

        self.system_config.accuracy_threshold_m = threshold_m;
        self.is_modified = true;
        Ok(old_value)
    }

    /// Get current accuracy threshold
    pub fn get_accuracy_threshold(&self) -> f32 {
        self.system_config.accuracy_threshold_m
    }

    /// Update maximum anchor age with validation
    pub fn set_max_anchor_age(&mut self, age_ms: u32) -> Result<u32, ConfigError> {
        let old_value = self.system_config.max_anchor_age_ms;
        
        if age_ms < 1000 {
            return Err(ConfigError::InvalidParameter {
                parameter: "max_anchor_age_ms".to_string(),
                value: age_ms.to_string(),
                reason: "Anchor age limit too short, may cause frequent data loss".to_string(),
            });
        }

        self.system_config.max_anchor_age_ms = age_ms;
        self.is_modified = true;
        Ok(old_value)
    }

    /// Get current maximum anchor age
    pub fn get_max_anchor_age(&self) -> u32 {
        self.system_config.max_anchor_age_ms
    }

    /// Enable or disable an anchor
    pub fn set_anchor_enabled(&mut self, anchor_id: u16, enabled: bool) -> Result<bool, ConfigError> {
        // Check if anchor exists first
        if !self.anchor_configs.contains_key(&anchor_id) {
            return Err(ConfigError::MissingParameter {
                parameter: format!("anchor_{}", anchor_id),
            });
        }
        
        let old_value = self.anchor_configs[&anchor_id].enabled;
        
        // If we're disabling an anchor, check if we'll have enough left
        if !enabled && old_value {
            let current_enabled_count = self.anchor_configs
                .values()
                .filter(|config| config.enabled)
                .count();
            
            if current_enabled_count <= self.system_config.min_anchors as usize {
                return Err(ConfigError::AnchorConflict {
                    anchor_id,
                    reason: format!(
                        "Cannot disable anchor: would leave only {} enabled anchors, but {} required",
                        current_enabled_count - 1,
                        self.system_config.min_anchors
                    ),
                });
            }
        }
        
        // Apply the change
        self.anchor_configs.get_mut(&anchor_id).unwrap().enabled = enabled;
        self.is_modified = true;
        
        Ok(old_value)
    }

    /// Check if an anchor is enabled
    pub fn is_anchor_enabled(&self, anchor_id: u16) -> Option<bool> {
        self.anchor_configs.get(&anchor_id).map(|config| config.enabled)
    }

    /// Update anchor quality weight
    pub fn set_anchor_quality_weight(&mut self, anchor_id: u16, weight: f32) -> Result<f32, ConfigError> {
        if weight < 0.0 || weight > 1.0 {
            return Err(ConfigError::InvalidParameter {
                parameter: "quality_weight".to_string(),
                value: weight.to_string(),
                reason: "Quality weight must be between 0.0 and 1.0".to_string(),
            });
        }

        if let Some(anchor) = self.anchor_configs.get_mut(&anchor_id) {
            let old_value = anchor.quality_weight;
            anchor.quality_weight = weight;
            self.is_modified = true;
            Ok(old_value)
        } else {
            Err(ConfigError::MissingParameter {
                parameter: format!("anchor_{}", anchor_id),
            })
        }
    }

    /// Get anchor quality weight
    pub fn get_anchor_quality_weight(&self, anchor_id: u16) -> Option<f32> {
        self.anchor_configs.get(&anchor_id).map(|config| config.quality_weight)
    }

    /// Update environmental temperature for sound speed correction
    pub fn set_water_temperature(&mut self, temperature_c: f32) -> Result<f32, ConfigError> {
        let old_value = self.system_config.environmental_corrections.default_temperature_c;
        
        if temperature_c < -5.0 || temperature_c > 40.0 {
            return Err(ConfigError::InvalidParameter {
                parameter: "temperature_c".to_string(),
                value: temperature_c.to_string(),
                reason: "Water temperature must be between -5°C and 40°C".to_string(),
            });
        }

        self.system_config.environmental_corrections.default_temperature_c = temperature_c;
        self.is_modified = true;
        Ok(old_value)
    }

    /// Get current water temperature
    pub fn get_water_temperature(&self) -> f32 {
        self.system_config.environmental_corrections.default_temperature_c
    }

    /// Update water salinity for sound speed correction
    pub fn set_water_salinity(&mut self, salinity_psu: f32) -> Result<f32, ConfigError> {
        let old_value = self.system_config.environmental_corrections.default_salinity_psu;
        
        if salinity_psu < 0.0 || salinity_psu > 50.0 {
            return Err(ConfigError::InvalidParameter {
                parameter: "salinity_psu".to_string(),
                value: salinity_psu.to_string(),
                reason: "Water salinity must be between 0 and 50 PSU".to_string(),
            });
        }

        self.system_config.environmental_corrections.default_salinity_psu = salinity_psu;
        self.is_modified = true;
        Ok(old_value)
    }

    /// Get current water salinity
    pub fn get_water_salinity(&self) -> f32 {
        self.system_config.environmental_corrections.default_salinity_psu
    }

    /// Update water pressure for sound speed correction
    pub fn set_water_pressure(&mut self, pressure_bar: f32) -> Result<f32, ConfigError> {
        let old_value = self.system_config.environmental_corrections.default_pressure_bar;
        
        if pressure_bar < 0.5 || pressure_bar > 1100.0 {
            return Err(ConfigError::InvalidParameter {
                parameter: "pressure_bar".to_string(),
                value: pressure_bar.to_string(),
                reason: "Water pressure must be between 0.5 and 1100 bar (surface to deepest ocean)".to_string(),
            });
        }

        self.system_config.environmental_corrections.default_pressure_bar = pressure_bar;
        self.is_modified = true;
        Ok(old_value)
    }

    /// Get current water pressure
    pub fn get_water_pressure(&self) -> f32 {
        self.system_config.environmental_corrections.default_pressure_bar
    }

    /// Enable or disable automatic sound speed correction
    pub fn set_auto_sound_speed_correction(&mut self, enabled: bool) -> bool {
        let old_value = self.system_config.environmental_corrections.auto_sound_speed_correction;
        self.system_config.environmental_corrections.auto_sound_speed_correction = enabled;
        self.is_modified = true;
        old_value
    }

    /// Check if automatic sound speed correction is enabled
    pub fn is_auto_sound_speed_correction_enabled(&self) -> bool {
        self.system_config.environmental_corrections.auto_sound_speed_correction
    }

    /// Calculate corrected sound speed based on current environmental parameters
    pub fn calculate_corrected_sound_speed(&self) -> f32 {
        let env = &self.system_config.environmental_corrections;
        let coeffs = &env.sound_speed_coefficients;
        
        // Simplified sound speed calculation using empirical formula
        // Based on Mackenzie equation for sound speed in seawater
        let base_speed = self.system_config.sound_speed_ms;
        let temp = env.default_temperature_c;
        let salinity = env.default_salinity_psu;
        let pressure = env.default_pressure_bar;
        
        let temp_correction = coeffs.temperature * temp;
        let salinity_correction = coeffs.salinity * salinity;
        let pressure_correction = coeffs.pressure * pressure;
        let cross_terms = coeffs.temperature_salinity * temp * salinity +
                         coeffs.temperature_pressure * temp * pressure +
                         coeffs.salinity_pressure * salinity * pressure;
        
        base_speed + temp_correction + salinity_correction + pressure_correction + cross_terms
    }

    /// Batch update multiple parameters with validation and rollback capability
    pub fn update_parameters(&mut self, updates: ParameterUpdates) -> Result<ParameterUpdateResult, ConfigError> {
        // Store original values for rollback
        let original_config = self.system_config.clone();
        let original_anchors = self.anchor_configs.clone();
        let original_modified = self.is_modified;
        
        let mut applied_updates = Vec::new();
        let mut failed_updates = Vec::new();
        
        // Apply sound speed update
        if let Some(sound_speed) = updates.sound_speed_ms {
            match self.set_sound_speed(sound_speed) {
                Ok(old_value) => applied_updates.push(format!("sound_speed: {} -> {}", old_value, sound_speed)),
                Err(e) => failed_updates.push(format!("sound_speed: {}", e)),
            }
        }
        
        // Apply timeout update
        if let Some(timeout) = updates.position_timeout_ms {
            match self.set_position_timeout(timeout) {
                Ok(old_value) => applied_updates.push(format!("timeout: {} -> {}", old_value, timeout)),
                Err(e) => failed_updates.push(format!("timeout: {}", e)),
            }
        }
        
        // Apply accuracy threshold update
        if let Some(threshold) = updates.accuracy_threshold_m {
            match self.set_accuracy_threshold(threshold) {
                Ok(old_value) => applied_updates.push(format!("accuracy_threshold: {} -> {}", old_value, threshold)),
                Err(e) => failed_updates.push(format!("accuracy_threshold: {}", e)),
            }
        }
        
        // Apply anchor age update
        if let Some(age) = updates.max_anchor_age_ms {
            match self.set_max_anchor_age(age) {
                Ok(old_value) => applied_updates.push(format!("max_anchor_age: {} -> {}", old_value, age)),
                Err(e) => failed_updates.push(format!("max_anchor_age: {}", e)),
            }
        }
        
        // Apply environmental updates
        if let Some(temp) = updates.water_temperature_c {
            match self.set_water_temperature(temp) {
                Ok(old_value) => applied_updates.push(format!("water_temperature: {} -> {}", old_value, temp)),
                Err(e) => failed_updates.push(format!("water_temperature: {}", e)),
            }
        }
        
        if let Some(salinity) = updates.water_salinity_psu {
            match self.set_water_salinity(salinity) {
                Ok(old_value) => applied_updates.push(format!("water_salinity: {} -> {}", old_value, salinity)),
                Err(e) => failed_updates.push(format!("water_salinity: {}", e)),
            }
        }
        
        if let Some(pressure) = updates.water_pressure_bar {
            match self.set_water_pressure(pressure) {
                Ok(old_value) => applied_updates.push(format!("water_pressure: {} -> {}", old_value, pressure)),
                Err(e) => failed_updates.push(format!("water_pressure: {}", e)),
            }
        }
        
        // Count total updates before consuming the vectors
        let total_updates = updates.count_updates();
        
        // Apply anchor enable/disable updates
        for (anchor_id, enabled) in updates.anchor_enabled {
            match self.set_anchor_enabled(anchor_id, enabled) {
                Ok(old_value) => applied_updates.push(format!("anchor_{}_enabled: {} -> {}", anchor_id, old_value, enabled)),
                Err(e) => failed_updates.push(format!("anchor_{}_enabled: {}", anchor_id, e)),
            }
        }
        
        // Apply anchor quality weight updates
        for (anchor_id, weight) in updates.anchor_quality_weights {
            match self.set_anchor_quality_weight(anchor_id, weight) {
                Ok(old_value) => applied_updates.push(format!("anchor_{}_weight: {} -> {}", anchor_id, old_value, weight)),
                Err(e) => failed_updates.push(format!("anchor_{}_weight: {}", anchor_id, e)),
            }
        }
        
        // Check if rollback is requested on any failure
        if !failed_updates.is_empty() && updates.rollback_on_failure {
            // Rollback all changes
            self.system_config = original_config;
            self.anchor_configs = original_anchors;
            self.is_modified = original_modified;
            
            return Err(ConfigError::InvalidParameter {
                parameter: "batch_update".to_string(),
                value: "multiple".to_string(),
                reason: format!("Batch update failed with {} errors, rolled back all changes: {}", 
                               failed_updates.len(), failed_updates.join("; ")),
            });
        }
        
        Ok(ParameterUpdateResult {
            applied_updates,
            failed_updates,
            total_updates,
        })
    }

    /// Create a configuration snapshot for rollback purposes
    pub fn create_snapshot(&self) -> ConfigurationSnapshot {
        ConfigurationSnapshot {
            system_config: self.system_config.clone(),
            anchor_configs: self.anchor_configs.clone(),
            is_modified: self.is_modified,
        }
    }

    /// Restore configuration from a snapshot
    pub fn restore_from_snapshot(&mut self, snapshot: ConfigurationSnapshot) {
        self.system_config = snapshot.system_config;
        self.anchor_configs = snapshot.anchor_configs;
        self.is_modified = snapshot.is_modified;
    }

    /// Get current parameter summary
    pub fn get_parameter_summary(&self) -> ParameterSummary {
        ParameterSummary {
            sound_speed_ms: self.system_config.sound_speed_ms,
            position_timeout_ms: self.system_config.position_timeout_ms,
            accuracy_threshold_m: self.system_config.accuracy_threshold_m,
            max_anchor_age_ms: self.system_config.max_anchor_age_ms,
            water_temperature_c: self.system_config.environmental_corrections.default_temperature_c,
            water_salinity_psu: self.system_config.environmental_corrections.default_salinity_psu,
            water_pressure_bar: self.system_config.environmental_corrections.default_pressure_bar,
            auto_sound_speed_correction: self.system_config.environmental_corrections.auto_sound_speed_correction,
            corrected_sound_speed_ms: self.calculate_corrected_sound_speed(),
            enabled_anchor_count: self.get_enabled_anchors().len(),
            total_anchor_count: self.anchor_configs.len(),
        }
    }

    /// Validate system configuration
    pub fn validate_system_config(&self, config: &SystemConfig) -> Result<ValidationResult, ConfigError> {
        let mut errors = Vec::new();
        let mut warnings = Vec::new();
        let mut suggestions = Vec::new();

        // Validate sound speed
        if config.sound_speed_ms < 1400.0 || config.sound_speed_ms > 1600.0 {
            errors.push(ConfigError::InvalidParameter {
                parameter: "sound_speed_ms".to_string(),
                value: config.sound_speed_ms.to_string(),
                reason: "Sound speed must be between 1400-1600 m/s for underwater environments".to_string(),
            });
        }

        // Validate anchor age limit
        if config.max_anchor_age_ms < 1000 {
            warnings.push("Very short anchor age limit may cause frequent data loss".to_string());
        } else if config.max_anchor_age_ms > 300000 {
            warnings.push("Very long anchor age limit may allow stale data to affect positioning".to_string());
        }

        // Validate minimum anchors
        if config.min_anchors < 3 {
            errors.push(ConfigError::InvalidParameter {
                parameter: "min_anchors".to_string(),
                value: config.min_anchors.to_string(),
                reason: "Minimum 3 anchors required for 2D positioning".to_string(),
            });
        } else if config.min_anchors > 8 {
            warnings.push("Very high minimum anchor count may prevent positioning in sparse networks".to_string());
        }

        // Validate timeout
        if config.position_timeout_ms < 50 {
            errors.push(ConfigError::InvalidParameter {
                parameter: "position_timeout_ms".to_string(),
                value: config.position_timeout_ms.to_string(),
                reason: "Position timeout too short for realistic computation".to_string(),
            });
        } else if config.position_timeout_ms > 5000 {
            warnings.push("Very long position timeout may affect real-time performance".to_string());
        }

        // Validate accuracy threshold
        if config.accuracy_threshold_m <= 0.0 {
            errors.push(ConfigError::InvalidParameter {
                parameter: "accuracy_threshold_m".to_string(),
                value: config.accuracy_threshold_m.to_string(),
                reason: "Accuracy threshold must be positive".to_string(),
            });
        }

        // Validate memory limit
        if config.memory_limit_bytes < 16 * 1024 {
            warnings.push("Very low memory limit may prevent proper operation".to_string());
            suggestions.push("Consider increasing memory limit to at least 32KB".to_string());
        }

        // Validate update rate
        if config.target_update_rate_hz <= 0.0 {
            errors.push(ConfigError::InvalidParameter {
                parameter: "target_update_rate_hz".to_string(),
                value: config.target_update_rate_hz.to_string(),
                reason: "Update rate must be positive".to_string(),
            });
        } else if config.target_update_rate_hz > 20.0 {
            warnings.push("Very high update rate may exceed system capabilities".to_string());
        }

        // Validate coordinate system
        if config.coordinate_system.reference_point.latitude.abs() > 90.0 {
            errors.push(ConfigError::InvalidParameter {
                parameter: "reference_point.latitude".to_string(),
                value: config.coordinate_system.reference_point.latitude.to_string(),
                reason: "Latitude must be between -90 and 90 degrees".to_string(),
            });
        }

        if config.coordinate_system.reference_point.longitude.abs() > 180.0 {
            errors.push(ConfigError::InvalidParameter {
                parameter: "reference_point.longitude".to_string(),
                value: config.coordinate_system.reference_point.longitude.to_string(),
                reason: "Longitude must be between -180 and 180 degrees".to_string(),
            });
        }

        if config.coordinate_system.reference_point.depth < 0.0 {
            errors.push(ConfigError::InvalidParameter {
                parameter: "reference_point.depth".to_string(),
                value: config.coordinate_system.reference_point.depth.to_string(),
                reason: "Depth must be non-negative (positive down)".to_string(),
            });
        }

        Ok(ValidationResult {
            is_valid: errors.is_empty(),
            errors,
            warnings,
            suggestions,
        })
    }

    /// Validate anchor configuration
    pub fn validate_anchor_config(&self, config: &AnchorConfig) -> Result<ValidationResult, ConfigError> {
        let mut errors = Vec::new();
        let mut warnings = Vec::new();
        let mut suggestions = Vec::new();

        // Validate anchor ID
        if config.id == 0 {
            errors.push(ConfigError::InvalidParameter {
                parameter: "id".to_string(),
                value: config.id.to_string(),
                reason: "Anchor ID cannot be zero".to_string(),
            });
        }

        // Check for duplicate IDs (excluding self)
        if let Some(existing) = self.anchor_configs.get(&config.id) {
            if existing.id == config.id {
                // This is an update, not a duplicate
            }
        }

        // Validate position
        if config.position.latitude.abs() > 90.0 {
            errors.push(ConfigError::InvalidParameter {
                parameter: "position.latitude".to_string(),
                value: config.position.latitude.to_string(),
                reason: "Latitude must be between -90 and 90 degrees".to_string(),
            });
        }

        if config.position.longitude.abs() > 180.0 {
            errors.push(ConfigError::InvalidParameter {
                parameter: "position.longitude".to_string(),
                value: config.position.longitude.to_string(),
                reason: "Longitude must be between -180 and 180 degrees".to_string(),
            });
        }

        if config.position.depth < 0.0 {
            errors.push(ConfigError::InvalidParameter {
                parameter: "position.depth".to_string(),
                value: config.position.depth.to_string(),
                reason: "Depth must be non-negative (positive down)".to_string(),
            });
        } else if config.position.depth > 11000.0 {
            warnings.push("Very deep anchor position (>11km) may be unrealistic".to_string());
        }

        // Validate range
        if config.max_range_m <= 0.0 {
            errors.push(ConfigError::InvalidParameter {
                parameter: "max_range_m".to_string(),
                value: config.max_range_m.to_string(),
                reason: "Maximum range must be positive".to_string(),
            });
        } else if config.max_range_m > 50000.0 {
            warnings.push("Very large maximum range may be unrealistic for acoustic positioning".to_string());
        }

        // Validate quality weight
        if config.quality_weight < 0.0 || config.quality_weight > 1.0 {
            errors.push(ConfigError::InvalidParameter {
                parameter: "quality_weight".to_string(),
                value: config.quality_weight.to_string(),
                reason: "Quality weight must be between 0.0 and 1.0".to_string(),
            });
        }

        // Validate timeout override
        if let Some(timeout) = config.timeout_override_ms {
            if timeout < 50 {
                errors.push(ConfigError::InvalidParameter {
                    parameter: "timeout_override_ms".to_string(),
                    value: timeout.to_string(),
                    reason: "Timeout override too short for realistic operation".to_string(),
                });
            }
        }

        // Validate calibration offset
        if config.range_calibration_offset_m.abs() > 100.0 {
            warnings.push("Large range calibration offset may indicate systematic error".to_string());
            suggestions.push("Consider recalibrating anchor position or timing".to_string());
        }

        Ok(ValidationResult {
            is_valid: errors.is_empty(),
            errors,
            warnings,
            suggestions,
        })
    }

    /// Validate overall system geometry with current anchor configuration
    pub fn validate_system_geometry(&self) -> Result<ValidationResult, ConfigError> {
        let mut errors = Vec::new();
        let mut warnings = Vec::new();
        let mut suggestions = Vec::new();

        let enabled_anchors: Vec<&AnchorConfig> = self.anchor_configs
            .values()
            .filter(|config| config.enabled)
            .collect();

        // Check minimum anchor count
        if enabled_anchors.len() < self.system_config.min_anchors as usize {
            errors.push(ConfigError::GeometryValidation {
                reason: format!(
                    "Only {} enabled anchors, but {} required",
                    enabled_anchors.len(),
                    self.system_config.min_anchors
                ),
            });
        }

        // Check for collinear anchors (simplified check)
        if enabled_anchors.len() >= 3 {
            let positions: Vec<_> = enabled_anchors.iter()
                .map(|a| (a.position.latitude, a.position.longitude, a.position.depth))
                .collect();

            if self.check_collinearity(&positions) {
                warnings.push("Anchors may be nearly collinear, which reduces positioning accuracy".to_string());
                suggestions.push("Consider repositioning anchors to form a more distributed geometry".to_string());
            }
        }

        // Check anchor spacing
        if enabled_anchors.len() >= 2 {
            let min_spacing = self.calculate_minimum_anchor_spacing(&enabled_anchors);
            if min_spacing < 10.0 {
                warnings.push("Some anchors are very close together, which may reduce accuracy".to_string());
                suggestions.push("Consider increasing spacing between anchors".to_string());
            }
        }

        Ok(ValidationResult {
            is_valid: errors.is_empty(),
            errors,
            warnings,
            suggestions,
        })
    }

    /// Simple collinearity check (placeholder implementation)
    fn check_collinearity(&self, positions: &[(f64, f64, f64)]) -> bool {
        // Simplified check - in a real implementation, this would use proper geometric analysis
        if positions.len() < 3 {
            return false;
        }

        // Check if all points have very similar latitude/longitude ratios
        let first = positions[0];
        let second = positions[1];
        
        if (second.0 - first.0).abs() < 1e-6 && (second.1 - first.1).abs() < 1e-6 {
            return true; // Same position
        }

        let ratio = (second.0 - first.0) / (second.1 - first.1);
        
        for i in 2..positions.len() {
            let current = positions[i];
            if (current.0 - first.0).abs() < 1e-6 && (current.1 - first.1).abs() < 1e-6 {
                continue; // Same as first point
            }
            
            let current_ratio = (current.0 - first.0) / (current.1 - first.1);
            if (current_ratio - ratio).abs() < 0.001 {
                return true; // Approximately collinear
            }
        }
        
        false
    }

    /// Calculate minimum spacing between anchors
    fn calculate_minimum_anchor_spacing(&self, anchors: &[&AnchorConfig]) -> f64 {
        let mut min_distance = f64::INFINITY;
        
        for i in 0..anchors.len() {
            for j in (i + 1)..anchors.len() {
                let a1 = &anchors[i].position;
                let a2 = &anchors[j].position;
                
                // Simplified distance calculation (should use proper geodetic distance)
                let lat_diff = (a1.latitude - a2.latitude) * 111320.0; // Approximate meters per degree
                let lon_diff = (a1.longitude - a2.longitude) * 111320.0 * a1.latitude.to_radians().cos();
                let depth_diff = a1.depth - a2.depth;
                
                let distance = (lat_diff * lat_diff + lon_diff * lon_diff + depth_diff * depth_diff).sqrt();
                min_distance = min_distance.min(distance);
            }
        }
        
        min_distance
    }
}

/// Configuration file data structure
#[derive(Debug, Serialize, Deserialize)]
struct ConfigFileData {
    /// System configuration
    system: SystemConfig,
    /// Anchor configurations
    anchors: Vec<AnchorConfig>,
}

/// Batch parameter updates structure
#[derive(Debug, Default)]
pub struct ParameterUpdates {
    /// Sound speed update (m/s)
    pub sound_speed_ms: Option<f32>,
    /// Position timeout update (milliseconds)
    pub position_timeout_ms: Option<u32>,
    /// Accuracy threshold update (meters)
    pub accuracy_threshold_m: Option<f32>,
    /// Maximum anchor age update (milliseconds)
    pub max_anchor_age_ms: Option<u32>,
    /// Water temperature update (Celsius)
    pub water_temperature_c: Option<f32>,
    /// Water salinity update (PSU)
    pub water_salinity_psu: Option<f32>,
    /// Water pressure update (bar)
    pub water_pressure_bar: Option<f32>,
    /// Anchor enable/disable updates (anchor_id, enabled)
    pub anchor_enabled: Vec<(u16, bool)>,
    /// Anchor quality weight updates (anchor_id, weight)
    pub anchor_quality_weights: Vec<(u16, f32)>,
    /// Whether to rollback all changes if any update fails
    pub rollback_on_failure: bool,
}

/// Result of batch parameter update
#[derive(Debug)]
pub struct ParameterUpdateResult {
    /// Successfully applied updates
    pub applied_updates: Vec<String>,
    /// Failed updates with error messages
    pub failed_updates: Vec<String>,
    /// Total number of updates attempted
    pub total_updates: usize,
}

/// Configuration snapshot for rollback
#[derive(Debug, Clone)]
pub struct ConfigurationSnapshot {
    /// System configuration at snapshot time
    system_config: SystemConfig,
    /// Anchor configurations at snapshot time
    anchor_configs: HashMap<u16, AnchorConfig>,
    /// Modified flag at snapshot time
    is_modified: bool,
}

/// Summary of current parameter values
#[derive(Debug)]
pub struct ParameterSummary {
    /// Current sound speed (m/s)
    pub sound_speed_ms: f32,
    /// Current position timeout (milliseconds)
    pub position_timeout_ms: u32,
    /// Current accuracy threshold (meters)
    pub accuracy_threshold_m: f32,
    /// Current maximum anchor age (milliseconds)
    pub max_anchor_age_ms: u32,
    /// Current water temperature (Celsius)
    pub water_temperature_c: f32,
    /// Current water salinity (PSU)
    pub water_salinity_psu: f32,
    /// Current water pressure (bar)
    pub water_pressure_bar: f32,
    /// Whether automatic sound speed correction is enabled
    pub auto_sound_speed_correction: bool,
    /// Calculated corrected sound speed (m/s)
    pub corrected_sound_speed_ms: f32,
    /// Number of enabled anchors
    pub enabled_anchor_count: usize,
    /// Total number of configured anchors
    pub total_anchor_count: usize,
}

impl ParameterUpdates {
    /// Create a new empty parameter updates structure
    pub fn new() -> Self {
        Self::default()
    }

    /// Set sound speed update
    pub fn with_sound_speed(mut self, sound_speed_ms: f32) -> Self {
        self.sound_speed_ms = Some(sound_speed_ms);
        self
    }

    /// Set position timeout update
    pub fn with_position_timeout(mut self, timeout_ms: u32) -> Self {
        self.position_timeout_ms = Some(timeout_ms);
        self
    }

    /// Set accuracy threshold update
    pub fn with_accuracy_threshold(mut self, threshold_m: f32) -> Self {
        self.accuracy_threshold_m = Some(threshold_m);
        self
    }

    /// Set maximum anchor age update
    pub fn with_max_anchor_age(mut self, age_ms: u32) -> Self {
        self.max_anchor_age_ms = Some(age_ms);
        self
    }

    /// Set water temperature update
    pub fn with_water_temperature(mut self, temperature_c: f32) -> Self {
        self.water_temperature_c = Some(temperature_c);
        self
    }

    /// Set water salinity update
    pub fn with_water_salinity(mut self, salinity_psu: f32) -> Self {
        self.water_salinity_psu = Some(salinity_psu);
        self
    }

    /// Set water pressure update
    pub fn with_water_pressure(mut self, pressure_bar: f32) -> Self {
        self.water_pressure_bar = Some(pressure_bar);
        self
    }

    /// Add anchor enable/disable update
    pub fn with_anchor_enabled(mut self, anchor_id: u16, enabled: bool) -> Self {
        self.anchor_enabled.push((anchor_id, enabled));
        self
    }

    /// Add anchor quality weight update
    pub fn with_anchor_quality_weight(mut self, anchor_id: u16, weight: f32) -> Self {
        self.anchor_quality_weights.push((anchor_id, weight));
        self
    }

    /// Enable rollback on failure
    pub fn with_rollback_on_failure(mut self) -> Self {
        self.rollback_on_failure = true;
        self
    }

    /// Count total number of updates
    pub fn count_updates(&self) -> usize {
        let mut count = 0;
        if self.sound_speed_ms.is_some() { count += 1; }
        if self.position_timeout_ms.is_some() { count += 1; }
        if self.accuracy_threshold_m.is_some() { count += 1; }
        if self.max_anchor_age_ms.is_some() { count += 1; }
        if self.water_temperature_c.is_some() { count += 1; }
        if self.water_salinity_psu.is_some() { count += 1; }
        if self.water_pressure_bar.is_some() { count += 1; }
        count += self.anchor_enabled.len();
        count += self.anchor_quality_weights.len();
        count
    }
}

impl std::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConfigError::InvalidParameter { parameter, value, reason } => {
                write!(f, "Invalid parameter '{}' = '{}': {}", parameter, value, reason)
            }
            ConfigError::MissingParameter { parameter } => {
                write!(f, "Missing required parameter: {}", parameter)
            }
            ConfigError::IoError { message } => {
                write!(f, "I/O error: {}", message)
            }
            ConfigError::SerializationError { message } => {
                write!(f, "Serialization error: {}", message)
            }
            ConfigError::AnchorConflict { anchor_id, reason } => {
                write!(f, "Anchor {} conflict: {}", anchor_id, reason)
            }
            ConfigError::GeometryValidation { reason } => {
                write!(f, "Geometry validation error: {}", reason)
            }
        }
    }
}

impl std::error::Error for ConfigError {}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;
    use std::path::PathBuf;

    #[test]
    fn test_default_system_config() {
        let config = SystemConfig::default();
        assert_eq!(config.sound_speed_ms, 1500.0);
        assert_eq!(config.min_anchors, 3);
        assert!(config.max_anchor_age_ms > 0);
    }

    #[test]
    fn test_configuration_manager_creation() {
        let manager = ConfigurationManager::new();
        assert_eq!(manager.get_system_config().sound_speed_ms, 1500.0);
        assert_eq!(manager.get_all_anchor_configs().len(), 0);
        assert!(!manager.is_modified());
    }

    #[test]
    fn test_anchor_config_validation() {
        let manager = ConfigurationManager::new();
        
        let valid_anchor = AnchorConfig {
            id: 101,
            position: GeodeticPosition {
                latitude: 32.123456,
                longitude: -117.654321,
                depth: 10.0,
            },
            max_range_m: 1000.0,
            enabled: true,
            quality_weight: 1.0,
            timeout_override_ms: None,
            reference_signal_strength: Some(80),
            range_calibration_offset_m: 0.0,
        };

        let result = manager.validate_anchor_config(&valid_anchor).unwrap();
        assert!(result.is_valid);
        assert!(result.errors.is_empty());
    }

    #[test]
    fn test_invalid_anchor_config() {
        let manager = ConfigurationManager::new();
        
        let invalid_anchor = AnchorConfig {
            id: 0, // Invalid ID
            position: GeodeticPosition {
                latitude: 95.0, // Invalid latitude
                longitude: -200.0, // Invalid longitude
                depth: -5.0, // Invalid depth
            },
            max_range_m: -100.0, // Invalid range
            enabled: true,
            quality_weight: 1.5, // Invalid weight
            timeout_override_ms: Some(10), // Too short
            reference_signal_strength: Some(80),
            range_calibration_offset_m: 0.0,
        };

        let result = manager.validate_anchor_config(&invalid_anchor).unwrap();
        assert!(!result.is_valid);
        assert!(!result.errors.is_empty());
    }

    #[test]
    fn test_config_serialization() {
        let mut manager = ConfigurationManager::new();
        
        let anchor = AnchorConfig {
            id: 101,
            position: GeodeticPosition {
                latitude: 32.123456,
                longitude: -117.654321,
                depth: 10.0,
            },
            max_range_m: 1000.0,
            enabled: true,
            quality_weight: 1.0,
            timeout_override_ms: None,
            reference_signal_strength: Some(80),
            range_calibration_offset_m: 0.0,
        };

        manager.set_anchor_config(anchor).unwrap();

        // Create temporary file for testing
        let temp_path = PathBuf::from("test_config.json");
        
        // Save and load
        manager.save_to_file(&temp_path).unwrap();
        let loaded_manager = ConfigurationManager::from_file(&temp_path).unwrap();
        
        // Verify loaded configuration
        assert_eq!(loaded_manager.get_all_anchor_configs().len(), 1);
        assert!(loaded_manager.get_anchor_config(101).is_some());
        
        // Cleanup
        let _ = fs::remove_file(temp_path);
    }

    #[test]
    fn test_runtime_sound_speed_adjustment() {
        let mut manager = ConfigurationManager::new();
        
        // Test valid sound speed update
        let old_speed = manager.set_sound_speed(1520.0).unwrap();
        assert_eq!(old_speed, 1500.0); // Default value
        assert_eq!(manager.get_sound_speed(), 1520.0);
        assert!(manager.is_modified());
        
        // Test invalid sound speed (too low)
        let result = manager.set_sound_speed(1300.0);
        assert!(result.is_err());
        assert_eq!(manager.get_sound_speed(), 1520.0); // Should remain unchanged
        
        // Test invalid sound speed (too high)
        let result = manager.set_sound_speed(1700.0);
        assert!(result.is_err());
        assert_eq!(manager.get_sound_speed(), 1520.0); // Should remain unchanged
    }

    #[test]
    fn test_runtime_timeout_adjustment() {
        let mut manager = ConfigurationManager::new();
        
        // Test valid timeout update
        let old_timeout = manager.set_position_timeout(300).unwrap();
        assert_eq!(old_timeout, 200); // Default value
        assert_eq!(manager.get_position_timeout(), 300);
        
        // Test invalid timeout (too short)
        let result = manager.set_position_timeout(30);
        assert!(result.is_err());
        assert_eq!(manager.get_position_timeout(), 300); // Should remain unchanged
        
        // Test invalid timeout (too long)
        let result = manager.set_position_timeout(15000);
        assert!(result.is_err());
        assert_eq!(manager.get_position_timeout(), 300); // Should remain unchanged
    }

    #[test]
    fn test_anchor_enable_disable() {
        let mut manager = ConfigurationManager::new();
        
        // Add test anchors
        for i in 1..=4 {
            let anchor = AnchorConfig {
                id: i,
                position: GeodeticPosition {
                    latitude: 32.0 + i as f64 * 0.001,
                    longitude: -117.0 - i as f64 * 0.001,
                    depth: 10.0,
                },
                max_range_m: 1000.0,
                enabled: true,
                quality_weight: 1.0,
                timeout_override_ms: None,
                reference_signal_strength: Some(80),
                range_calibration_offset_m: 0.0,
            };
            manager.set_anchor_config(anchor).unwrap();
        }
        
        // Test disabling an anchor
        let old_enabled = manager.set_anchor_enabled(1, false).unwrap();
        assert!(old_enabled);
        assert!(!manager.is_anchor_enabled(1).unwrap());
        
        // Test that we can't disable too many anchors (would go below minimum)
        // We have 4 anchors, minimum is 3. Disable one more (anchor 1 is already disabled)
        let result = manager.set_anchor_enabled(2, false); // Now only 2 enabled (3 and 4), below minimum
        assert!(result.is_err()); // Should fail
        assert!(manager.is_anchor_enabled(2).unwrap()); // Should remain enabled
        
        // Test enabling an anchor
        let old_enabled = manager.set_anchor_enabled(1, true).unwrap();
        assert!(!old_enabled);
        assert!(manager.is_anchor_enabled(1).unwrap());
    }

    #[test]
    fn test_environmental_parameter_adjustment() {
        let mut manager = ConfigurationManager::new();
        
        // Test temperature adjustment
        let old_temp = manager.set_water_temperature(20.0).unwrap();
        assert_eq!(old_temp, 15.0); // Default value
        assert_eq!(manager.get_water_temperature(), 20.0);
        
        // Test invalid temperature
        let result = manager.set_water_temperature(-10.0);
        assert!(result.is_err());
        assert_eq!(manager.get_water_temperature(), 20.0); // Should remain unchanged
        
        // Test salinity adjustment
        let old_salinity = manager.set_water_salinity(30.0).unwrap();
        assert_eq!(old_salinity, 35.0); // Default value
        assert_eq!(manager.get_water_salinity(), 30.0);
        
        // Test pressure adjustment
        let old_pressure = manager.set_water_pressure(5.0).unwrap();
        assert_eq!(old_pressure, 1.0); // Default value
        assert_eq!(manager.get_water_pressure(), 5.0);
    }

    #[test]
    fn test_corrected_sound_speed_calculation() {
        let mut manager = ConfigurationManager::new();
        
        // Set environmental parameters
        manager.set_water_temperature(20.0).unwrap();
        manager.set_water_salinity(35.0).unwrap();
        manager.set_water_pressure(2.0).unwrap();
        
        let corrected_speed = manager.calculate_corrected_sound_speed();
        
        // Should be different from base speed due to corrections
        assert_ne!(corrected_speed, manager.get_sound_speed());
        
        // Should be reasonable for underwater conditions (allow for wider range due to corrections)
        assert!(corrected_speed > 1400.0);
        assert!(corrected_speed < 1700.0);
    }

    #[test]
    fn test_batch_parameter_updates() {
        let mut manager = ConfigurationManager::new();
        
        // Add test anchor
        let anchor = AnchorConfig {
            id: 101,
            position: GeodeticPosition {
                latitude: 32.123456,
                longitude: -117.654321,
                depth: 10.0,
            },
            max_range_m: 1000.0,
            enabled: true,
            quality_weight: 1.0,
            timeout_override_ms: None,
            reference_signal_strength: Some(80),
            range_calibration_offset_m: 0.0,
        };
        manager.set_anchor_config(anchor).unwrap();
        
        // Create batch update
        let updates = ParameterUpdates::new()
            .with_sound_speed(1520.0)
            .with_position_timeout(300)
            .with_water_temperature(18.0)
            .with_anchor_quality_weight(101, 0.8);
        
        let result = manager.update_parameters(updates).unwrap();
        
        // Verify all updates were applied
        assert_eq!(result.applied_updates.len(), 4);
        assert_eq!(result.failed_updates.len(), 0);
        assert_eq!(result.total_updates, 4);
        
        // Verify parameter values
        assert_eq!(manager.get_sound_speed(), 1520.0);
        assert_eq!(manager.get_position_timeout(), 300);
        assert_eq!(manager.get_water_temperature(), 18.0);
        assert_eq!(manager.get_anchor_quality_weight(101).unwrap(), 0.8);
    }

    #[test]
    fn test_batch_update_with_rollback() {
        let mut manager = ConfigurationManager::new();
        
        // Create batch update with one invalid parameter
        let updates = ParameterUpdates::new()
            .with_sound_speed(1520.0)  // Valid
            .with_position_timeout(30) // Invalid (too short)
            .with_water_temperature(18.0) // Valid
            .with_rollback_on_failure();
        
        let result = manager.update_parameters(updates);
        
        // Should fail due to rollback on failure
        assert!(result.is_err());
        
        // All parameters should remain at default values
        assert_eq!(manager.get_sound_speed(), 1500.0);
        assert_eq!(manager.get_position_timeout(), 200);
        assert_eq!(manager.get_water_temperature(), 15.0);
    }

    #[test]
    fn test_configuration_snapshot_and_restore() {
        let mut manager = ConfigurationManager::new();
        
        // Create snapshot of initial state
        let snapshot = manager.create_snapshot();
        
        // Make changes
        manager.set_sound_speed(1520.0).unwrap();
        manager.set_water_temperature(20.0).unwrap();
        
        // Verify changes
        assert_eq!(manager.get_sound_speed(), 1520.0);
        assert_eq!(manager.get_water_temperature(), 20.0);
        
        // Restore from snapshot
        manager.restore_from_snapshot(snapshot);
        
        // Verify restoration
        assert_eq!(manager.get_sound_speed(), 1500.0);
        assert_eq!(manager.get_water_temperature(), 15.0);
    }

    #[test]
    fn test_parameter_summary() {
        let mut manager = ConfigurationManager::new();
        
        // Add test anchor
        let anchor = AnchorConfig {
            id: 101,
            position: GeodeticPosition {
                latitude: 32.123456,
                longitude: -117.654321,
                depth: 10.0,
            },
            max_range_m: 1000.0,
            enabled: true,
            quality_weight: 1.0,
            timeout_override_ms: None,
            reference_signal_strength: Some(80),
            range_calibration_offset_m: 0.0,
        };
        manager.set_anchor_config(anchor).unwrap();
        
        let summary = manager.get_parameter_summary();
        
        // Verify summary contains expected values
        assert_eq!(summary.sound_speed_ms, 1500.0);
        assert_eq!(summary.position_timeout_ms, 200);
        assert_eq!(summary.accuracy_threshold_m, 10.0);
        assert_eq!(summary.enabled_anchor_count, 1);
        assert_eq!(summary.total_anchor_count, 1);
        assert!(summary.auto_sound_speed_correction);
    }
}