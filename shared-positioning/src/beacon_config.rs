use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs;
use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime, UNIX_EPOCH};
use uuid::Uuid;

use crate::error_handling::{ConfigError, Result};

/// Beacon configuration with all operational parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BeaconConfig {
    /// Unique beacon identifier
    pub beacon_id: Uuid,
    /// Message transmission configuration
    pub transmission: TransmissionConfig,
    /// GPS configuration
    pub gps: GpsConfig,
    /// Power management configuration
    pub power: PowerConfig,
    /// Communication configuration
    pub communication: CommunicationConfig,
    /// Emergency handling configuration
    pub emergency: EmergencyConfig,
    /// Hardware-specific configuration
    pub hardware: HardwareConfig,
    /// Configuration metadata
    pub metadata: BeaconConfigMetadata,
}

/// Message transmission configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransmissionConfig {
    /// Transmission interval in milliseconds (1000-10000)
    pub interval_ms: u32,
    /// Message format version (V1, V2, V3)
    pub message_version: MessageVersion,
    /// Transmission power level (0-255)
    pub power_level: u8,
    /// Maximum transmission retries on failure
    pub max_retries: u8,
    /// Retry delay in milliseconds
    pub retry_delay_ms: u32,
    /// Enable adaptive power control
    pub adaptive_power: bool,
    /// Sequence number rollover threshold
    pub sequence_rollover: u16,
}

/// Message format versions
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum MessageVersion {
    V1,
    V2,
    V3, // New version with UUID support
}

/// GPS configuration parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GpsConfig {
    /// GPS acquisition timeout in seconds (30-300)
    pub acquisition_timeout_s: u32,
    /// Position update interval in seconds (1-30)
    pub update_interval_s: u32,
    /// Minimum satellite count for valid fix (3-12)
    pub min_satellite_count: u8,
    /// Required accuracy threshold in meters (0.5-50.0)
    pub accuracy_threshold_m: f32,
    /// Cold start timeout in seconds (60-600)
    pub cold_start_timeout_s: u32,
    /// Enable DGPS corrections if available
    pub enable_dgps: bool,
    /// Maximum age of GPS fix before considered stale (seconds)
    pub max_fix_age_s: u32,
}

/// Power management configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PowerConfig {
    /// Low battery threshold percentage (10.0-30.0)
    pub low_battery_threshold_percent: f32,
    /// Critical battery threshold percentage (5.0-15.0)
    pub critical_battery_threshold_percent: f32,
    /// Emergency battery threshold percentage (2.0-10.0)
    pub emergency_battery_threshold_percent: f32,
    /// Power save mode threshold percentage (15.0-40.0)
    pub power_save_threshold_percent: f32,
    /// Enable charging system
    pub charging_enabled: bool,
    /// Enable solar charging
    pub solar_charging_enabled: bool,
    /// Battery monitoring interval in seconds (10-300)
    pub monitoring_interval_s: u32,
    /// Temperature monitoring thresholds
    pub temperature_limits: TemperatureLimits,
    /// Power mode configurations
    pub power_modes: PowerModeConfig,
}

/// Temperature monitoring limits
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TemperatureLimits {
    /// Minimum operating temperature in Celsius (-20 to 10)
    pub min_operating_c: f32,
    /// Maximum operating temperature in Celsius (40 to 70)
    pub max_operating_c: f32,
    /// Temperature at which to trigger warnings (35 to 60)
    pub warning_threshold_c: f32,
    /// Temperature at which to trigger emergency shutdown (50 to 80)
    pub emergency_threshold_c: f32,
}

/// Power mode specific configurations
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PowerModeConfig {
    /// Normal mode transmission interval multiplier (1.0)
    pub normal_transmission_multiplier: f32,
    /// Power save mode transmission interval multiplier (2.0-5.0)
    pub power_save_transmission_multiplier: f32,
    /// Emergency mode transmission interval multiplier (0.5-2.0)
    pub emergency_transmission_multiplier: f32,
    /// CPU frequency scaling factors for each mode
    pub cpu_scaling: CpuScalingConfig,
}

/// CPU frequency scaling configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CpuScalingConfig {
    /// Normal mode CPU frequency percentage (80-100)
    pub normal_frequency_percent: u8,
    /// Power save mode CPU frequency percentage (40-80)
    pub power_save_frequency_percent: u8,
    /// Emergency mode CPU frequency percentage (60-100)
    pub emergency_frequency_percent: u8,
}

/// Long-range communication configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommunicationConfig {
    /// Connection attempt interval in hours (1-24)
    pub connection_interval_hours: u32,
    /// Maximum retry attempts per connection (1-10)
    pub max_retry_attempts: u32,
    /// Initial retry backoff in milliseconds (1000-10000)
    pub initial_retry_backoff_ms: u32,
    /// Maximum retry interval in hours (1-12)
    pub max_retry_interval_hours: u32,
    /// Connection timeout in seconds (30-300)
    pub connection_timeout_s: u32,
    /// Enable data compression
    pub compression_enabled: bool,
    /// Status report configuration
    pub status_report: StatusReportConfig,
    /// Communication endpoints
    pub endpoints: CommunicationEndpoints,
}

/// Status report configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StatusReportConfig {
    /// Include position history in reports
    pub include_position_history: bool,
    /// Maximum position history entries (10-1000)
    pub max_position_history: u32,
    /// Include detailed battery statistics
    pub include_battery_details: bool,
    /// Include transmission statistics
    pub include_transmission_stats: bool,
    /// Include system health metrics
    pub include_system_health: bool,
}

/// Communication endpoint configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommunicationEndpoints {
    /// Primary endpoint URL
    pub primary_url: String,
    /// Backup endpoint URL (optional)
    pub backup_url: Option<String>,
    /// API key for authentication
    pub api_key: String,
    /// Device authentication token
    pub device_token: String,
}

/// Emergency handling configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmergencyConfig {
    /// Enable emergency transmission mode
    pub emergency_mode_enabled: bool,
    /// Emergency transmission interval in milliseconds (100-5000)
    pub emergency_interval_ms: u32,
    /// Emergency transmission power boost percentage (100-200)
    pub emergency_power_boost_percent: u8,
    /// Maximum emergency mode duration in minutes (5-60)
    pub max_emergency_duration_minutes: u32,
    /// Auto-recovery from emergency conditions
    pub auto_recovery_enabled: bool,
    /// Emergency shutdown conditions
    pub shutdown_conditions: EmergencyShutdownConfig,
}

/// Emergency shutdown configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmergencyShutdownConfig {
    /// Enable automatic shutdown on critical battery
    pub battery_shutdown_enabled: bool,
    /// Enable automatic shutdown on temperature extremes
    pub temperature_shutdown_enabled: bool,
    /// Enable automatic shutdown on hardware faults
    pub hardware_fault_shutdown_enabled: bool,
    /// Grace period before shutdown in seconds (10-300)
    pub shutdown_grace_period_s: u32,
}

/// Hardware-specific configuration for ESP01-class devices
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HardwareConfig {
    /// GPIO pin assignments
    pub gpio_pins: GpioPinConfig,
    /// SPI configuration
    pub spi_config: SpiConfig,
    /// I2C configuration
    pub i2c_config: I2cConfig,
    /// Memory management settings
    pub memory_config: MemoryConfig,
    /// Watchdog timer configuration
    pub watchdog_config: WatchdogConfig,
}

/// GPIO pin assignments for ESP01-class devices
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GpioPinConfig {
    /// Status LED pin (0-15, optional)
    pub status_led_pin: Option<u8>,
    /// Emergency button pin (0-15, optional)
    pub emergency_button_pin: Option<u8>,
    /// Power control pin (0-15, optional)
    pub power_control_pin: Option<u8>,
    /// External reset pin (0-15, optional)
    pub external_reset_pin: Option<u8>,
}

/// SPI bus configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SpiConfig {
    /// SPI clock frequency in Hz (100000-10000000)
    pub clock_frequency_hz: u32,
    /// SPI mode (0-3)
    pub mode: u8,
    /// Bit order (MSB first = true, LSB first = false)
    pub msb_first: bool,
}

/// I2C bus configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct I2cConfig {
    /// I2C clock frequency in Hz (10000-400000)
    pub clock_frequency_hz: u32,
    /// I2C timeout in milliseconds (100-5000)
    pub timeout_ms: u32,
}

/// Memory management configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MemoryConfig {
    /// Maximum heap usage percentage (50-80)
    pub max_heap_usage_percent: u8,
    /// Stack size in bytes (2048-8192)
    pub stack_size_bytes: u32,
    /// Enable memory monitoring
    pub memory_monitoring_enabled: bool,
    /// Memory check interval in seconds (10-300)
    pub memory_check_interval_s: u32,
}

/// Watchdog timer configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WatchdogConfig {
    /// Enable watchdog timer
    pub enabled: bool,
    /// Watchdog timeout in seconds (1-30)
    pub timeout_s: u32,
    /// Enable automatic reset on timeout
    pub auto_reset_enabled: bool,
}

/// Configuration metadata for versioning and tracking
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BeaconConfigMetadata {
    /// Configuration schema version
    pub schema_version: String,
    /// Configuration creation timestamp
    pub created_at: u64,
    /// Last modification timestamp
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

/// Record of configuration migrations
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MigrationRecord {
    /// Source schema version
    pub from_version: String,
    /// Target schema version
    pub to_version: String,
    /// Migration timestamp
    pub migrated_at: u64,
    /// Migration description
    pub description: String,
    /// Success status
    pub success: bool,
}

impl BeaconConfig {
    /// Create a new beacon configuration with default values
    pub fn new(beacon_id: Uuid) -> Self {
        let now = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs();

        Self {
            beacon_id,
            transmission: TransmissionConfig::default(),
            gps: GpsConfig::default(),
            power: PowerConfig::default(),
            communication: CommunicationConfig::default(),
            emergency: EmergencyConfig::default(),
            hardware: HardwareConfig::default(),
            metadata: BeaconConfigMetadata {
                schema_version: "1.0.0".to_string(),
                created_at: now,
                modified_at: now,
                description: "Default beacon configuration".to_string(),
                author: "System".to_string(),
                checksum: String::new(),
                migration_history: Vec::new(),
            },
        }
    }

    /// Create configuration optimized for low power operation
    pub fn low_power(beacon_id: Uuid) -> Self {
        let mut config = Self::new(beacon_id);
        
        // Optimize for power saving
        config.transmission.interval_ms = 5000; // Longer intervals
        config.transmission.power_level = 128; // Reduced power
        config.transmission.adaptive_power = true;
        
        config.gps.update_interval_s = 4; // Less frequent GPS updates but shorter than transmission
        config.gps.accuracy_threshold_m = 5.0; // More lenient accuracy
        
        config.power.power_save_threshold_percent = 30.0; // Enter power save earlier
        config.power.monitoring_interval_s = 60; // Less frequent monitoring
        
        config.communication.connection_interval_hours = 12; // Less frequent communication
        
        config.hardware.memory_config.max_heap_usage_percent = 60; // Conservative memory usage
        config.hardware.watchdog_config.timeout_s = 10; // Shorter watchdog for reliability
        
        config.metadata.description = "Low power optimized beacon configuration".to_string();
        config
    }

    /// Create configuration optimized for high reliability
    pub fn high_reliability(beacon_id: Uuid) -> Self {
        let mut config = Self::new(beacon_id);
        
        // Optimize for reliability
        config.transmission.interval_ms = 2000; // More frequent transmissions
        config.transmission.max_retries = 5; // More retries
        config.transmission.adaptive_power = true;
        
        config.gps.min_satellite_count = 6; // Require more satellites
        config.gps.accuracy_threshold_m = 2.0; // Higher accuracy requirement
        config.gps.enable_dgps = true; // Use DGPS if available
        
        config.power.monitoring_interval_s = 30; // More frequent monitoring
        config.emergency.auto_recovery_enabled = true; // Enable auto-recovery
        
        config.communication.max_retry_attempts = 8; // More communication retries
        config.communication.status_report.include_system_health = true;
        
        config.hardware.memory_config.memory_monitoring_enabled = true;
        config.hardware.watchdog_config.enabled = true;
        config.hardware.watchdog_config.timeout_s = 15;
        
        config.metadata.description = "High reliability beacon configuration".to_string();
        config
    }
}

// Default implementations for configuration structs
impl Default for TransmissionConfig {
    fn default() -> Self {
        Self {
            interval_ms: 3000,
            message_version: MessageVersion::V2,
            power_level: 200,
            max_retries: 3,
            retry_delay_ms: 1000,
            adaptive_power: false,
            sequence_rollover: 65535,
        }
    }
}

impl Default for GpsConfig {
    fn default() -> Self {
        Self {
            acquisition_timeout_s: 60,
            update_interval_s: 2,
            min_satellite_count: 4,
            accuracy_threshold_m: 3.0,
            cold_start_timeout_s: 120,
            enable_dgps: false,
            max_fix_age_s: 30,
        }
    }
}

impl Default for PowerConfig {
    fn default() -> Self {
        Self {
            low_battery_threshold_percent: 20.0,
            critical_battery_threshold_percent: 10.0,
            emergency_battery_threshold_percent: 5.0,
            power_save_threshold_percent: 25.0,
            charging_enabled: true,
            solar_charging_enabled: false,
            monitoring_interval_s: 30,
            temperature_limits: TemperatureLimits::default(),
            power_modes: PowerModeConfig::default(),
        }
    }
}

impl Default for TemperatureLimits {
    fn default() -> Self {
        Self {
            min_operating_c: -10.0,
            max_operating_c: 50.0,
            warning_threshold_c: 55.0,
            emergency_threshold_c: 65.0,
        }
    }
}

impl Default for PowerModeConfig {
    fn default() -> Self {
        Self {
            normal_transmission_multiplier: 1.0,
            power_save_transmission_multiplier: 2.0,
            emergency_transmission_multiplier: 0.5,
            cpu_scaling: CpuScalingConfig::default(),
        }
    }
}

impl Default for CpuScalingConfig {
    fn default() -> Self {
        Self {
            normal_frequency_percent: 100,
            power_save_frequency_percent: 60,
            emergency_frequency_percent: 80,
        }
    }
}

impl Default for CommunicationConfig {
    fn default() -> Self {
        Self {
            connection_interval_hours: 6,
            max_retry_attempts: 5,
            initial_retry_backoff_ms: 2000,
            max_retry_interval_hours: 4,
            connection_timeout_s: 120,
            compression_enabled: true,
            status_report: StatusReportConfig::default(),
            endpoints: CommunicationEndpoints::default(),
        }
    }
}

impl Default for StatusReportConfig {
    fn default() -> Self {
        Self {
            include_position_history: true,
            max_position_history: 100,
            include_battery_details: true,
            include_transmission_stats: true,
            include_system_health: false,
        }
    }
}

impl Default for CommunicationEndpoints {
    fn default() -> Self {
        Self {
            primary_url: "https://api.example.com/beacon".to_string(),
            backup_url: None,
            api_key: "your-api-key".to_string(),
            device_token: "your-device-token".to_string(),
        }
    }
}

impl Default for EmergencyConfig {
    fn default() -> Self {
        Self {
            emergency_mode_enabled: true,
            emergency_interval_ms: 1000,
            emergency_power_boost_percent: 150,
            max_emergency_duration_minutes: 30,
            auto_recovery_enabled: false,
            shutdown_conditions: EmergencyShutdownConfig::default(),
        }
    }
}

impl Default for EmergencyShutdownConfig {
    fn default() -> Self {
        Self {
            battery_shutdown_enabled: true,
            temperature_shutdown_enabled: true,
            hardware_fault_shutdown_enabled: true,
            shutdown_grace_period_s: 60,
        }
    }
}

impl Default for HardwareConfig {
    fn default() -> Self {
        Self {
            gpio_pins: GpioPinConfig::default(),
            spi_config: SpiConfig::default(),
            i2c_config: I2cConfig::default(),
            memory_config: MemoryConfig::default(),
            watchdog_config: WatchdogConfig::default(),
        }
    }
}

impl Default for GpioPinConfig {
    fn default() -> Self {
        Self {
            status_led_pin: Some(2),
            emergency_button_pin: None,
            power_control_pin: None,
            external_reset_pin: None,
        }
    }
}

impl Default for SpiConfig {
    fn default() -> Self {
        Self {
            clock_frequency_hz: 1000000, // 1 MHz
            mode: 0,
            msb_first: true,
        }
    }
}

impl Default for I2cConfig {
    fn default() -> Self {
        Self {
            clock_frequency_hz: 100000, // 100 kHz
            timeout_ms: 1000,
        }
    }
}

impl Default for MemoryConfig {
    fn default() -> Self {
        Self {
            max_heap_usage_percent: 70,
            stack_size_bytes: 4096,
            memory_monitoring_enabled: false,
            memory_check_interval_s: 60,
        }
    }
}

impl Default for WatchdogConfig {
    fn default() -> Self {
        Self {
            enabled: false,
            timeout_s: 5,
            auto_reset_enabled: true,
        }
    }
}

/// Configuration validation implementation
impl BeaconConfig {
    /// Validate the entire beacon configuration
    pub fn validate(&self) -> Result<()> {
        let mut errors = Vec::new();

        // Validate transmission configuration
        if let Err(e) = self.transmission.validate() {
            errors.push(format!("Transmission config: {}", e));
        }

        // Validate GPS configuration
        if let Err(e) = self.gps.validate() {
            errors.push(format!("GPS config: {}", e));
        }

        // Validate power configuration
        if let Err(e) = self.power.validate() {
            errors.push(format!("Power config: {}", e));
        }

        // Validate communication configuration
        if let Err(e) = self.communication.validate() {
            errors.push(format!("Communication config: {}", e));
        }

        // Validate emergency configuration
        if let Err(e) = self.emergency.validate() {
            errors.push(format!("Emergency config: {}", e));
        }

        // Validate hardware configuration
        if let Err(e) = self.hardware.validate() {
            errors.push(format!("Hardware config: {}", e));
        }

        // Cross-validation checks
        self.validate_cross_dependencies(&mut errors);

        if errors.is_empty() {
            Ok(())
        } else {
            Err(ConfigError::ValidationFailed(errors.join("; ")))
        }
    }

    /// Validate cross-dependencies between configuration sections
    fn validate_cross_dependencies(&self, errors: &mut Vec<String>) {
        // Check power save threshold vs critical threshold
        if self.power.power_save_threshold_percent <= self.power.critical_battery_threshold_percent {
            errors.push("Power save threshold must be higher than critical battery threshold".to_string());
        }

        // Check emergency transmission interval vs normal interval
        if self.emergency.emergency_interval_ms > self.transmission.interval_ms {
            errors.push("Emergency transmission interval should be shorter than normal interval".to_string());
        }

        // Check GPS update interval vs transmission interval
        if (self.gps.update_interval_s * 1000) > self.transmission.interval_ms {
            errors.push("GPS update interval should be shorter than transmission interval".to_string());
        }

        // Check memory configuration vs stack size (assuming 80KB total memory for ESP01)
        let total_memory_bytes = 80 * 1024; // 80KB
        let max_heap_bytes = (total_memory_bytes * self.hardware.memory_config.max_heap_usage_percent as u32) / 100;
        if self.hardware.memory_config.stack_size_bytes > max_heap_bytes {
            errors.push(format!("Stack size {} bytes cannot exceed maximum heap usage {} bytes", 
                self.hardware.memory_config.stack_size_bytes, max_heap_bytes));
        }
    }

    /// Update configuration metadata after changes
    pub fn update_metadata(&mut self, description: Option<String>, author: Option<String>) {
        self.metadata.modified_at = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs();

        if let Some(desc) = description {
            self.metadata.description = desc;
        }

        if let Some(auth) = author {
            self.metadata.author = auth;
        }

        // Update checksum
        self.metadata.checksum = self.calculate_checksum();
    }

    /// Calculate configuration checksum for integrity verification
    fn calculate_checksum(&self) -> String {
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};

        let mut hasher = DefaultHasher::new();
        
        // Hash all configuration fields except metadata
        self.beacon_id.hash(&mut hasher);
        format!("{:?}", self.transmission).hash(&mut hasher);
        format!("{:?}", self.gps).hash(&mut hasher);
        format!("{:?}", self.power).hash(&mut hasher);
        format!("{:?}", self.communication).hash(&mut hasher);
        format!("{:?}", self.emergency).hash(&mut hasher);
        format!("{:?}", self.hardware).hash(&mut hasher);

        format!("{:x}", hasher.finish())
    }

    /// Verify configuration integrity using checksum
    pub fn verify_integrity(&self) -> bool {
        let calculated_checksum = self.calculate_checksum();
        calculated_checksum == self.metadata.checksum
    }
}

/// Validation implementations for individual configuration sections
impl TransmissionConfig {
    pub fn validate(&self) -> Result<()> {
        let mut errors = Vec::new();

        // Validate transmission interval
        if self.interval_ms < 1000 || self.interval_ms > 10000 {
            errors.push(format!("Transmission interval {}ms outside valid range (1000-10000ms)", self.interval_ms));
        }

        // Validate power level
        if self.power_level == 0 {
            errors.push("Transmission power level cannot be zero".to_string());
        }

        // Validate retry configuration
        if self.max_retries > 10 {
            errors.push("Maximum retries should not exceed 10".to_string());
        }

        if self.retry_delay_ms < 100 || self.retry_delay_ms > 10000 {
            errors.push(format!("Retry delay {}ms outside valid range (100-10000ms)", self.retry_delay_ms));
        }

        // Validate sequence rollover
        if self.sequence_rollover < 1000 {
            errors.push("Sequence rollover threshold too low (minimum 1000)".to_string());
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(ConfigError::ValidationFailed(errors.join("; ")))
        }
    }
}

impl GpsConfig {
    pub fn validate(&self) -> Result<()> {
        let mut errors = Vec::new();

        // Validate acquisition timeout
        if self.acquisition_timeout_s < 30 || self.acquisition_timeout_s > 300 {
            errors.push(format!("GPS acquisition timeout {}s outside valid range (30-300s)", self.acquisition_timeout_s));
        }

        // Validate update interval
        if self.update_interval_s < 1 || self.update_interval_s > 30 {
            errors.push(format!("GPS update interval {}s outside valid range (1-30s)", self.update_interval_s));
        }

        // Validate satellite count
        if self.min_satellite_count < 3 || self.min_satellite_count > 12 {
            errors.push(format!("Minimum satellite count {} outside valid range (3-12)", self.min_satellite_count));
        }

        // Validate accuracy threshold
        if self.accuracy_threshold_m < 0.5 || self.accuracy_threshold_m > 50.0 {
            errors.push(format!("Accuracy threshold {:.1}m outside valid range (0.5-50.0m)", self.accuracy_threshold_m));
        }

        // Validate cold start timeout
        if self.cold_start_timeout_s < 60 || self.cold_start_timeout_s > 600 {
            errors.push(format!("Cold start timeout {}s outside valid range (60-600s)", self.cold_start_timeout_s));
        }

        // Validate max fix age
        if self.max_fix_age_s < 5 || self.max_fix_age_s > 300 {
            errors.push(format!("Max fix age {}s outside valid range (5-300s)", self.max_fix_age_s));
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(ConfigError::ValidationFailed(errors.join("; ")))
        }
    }
}

impl PowerConfig {
    pub fn validate(&self) -> Result<()> {
        let mut errors = Vec::new();

        // Validate battery thresholds
        if self.low_battery_threshold_percent < 10.0 || self.low_battery_threshold_percent > 30.0 {
            errors.push(format!("Low battery threshold {:.1}% outside valid range (10.0-30.0%)", self.low_battery_threshold_percent));
        }

        if self.critical_battery_threshold_percent < 5.0 || self.critical_battery_threshold_percent > 15.0 {
            errors.push(format!("Critical battery threshold {:.1}% outside valid range (5.0-15.0%)", self.critical_battery_threshold_percent));
        }

        if self.emergency_battery_threshold_percent < 2.0 || self.emergency_battery_threshold_percent > 10.0 {
            errors.push(format!("Emergency battery threshold {:.1}% outside valid range (2.0-10.0%)", self.emergency_battery_threshold_percent));
        }

        if self.power_save_threshold_percent < 15.0 || self.power_save_threshold_percent > 40.0 {
            errors.push(format!("Power save threshold {:.1}% outside valid range (15.0-40.0%)", self.power_save_threshold_percent));
        }

        // Validate threshold ordering
        if self.emergency_battery_threshold_percent >= self.critical_battery_threshold_percent {
            errors.push("Emergency threshold must be lower than critical threshold".to_string());
        }

        if self.critical_battery_threshold_percent >= self.low_battery_threshold_percent {
            errors.push("Critical threshold must be lower than low battery threshold".to_string());
        }

        if self.low_battery_threshold_percent >= self.power_save_threshold_percent {
            errors.push("Low battery threshold must be lower than power save threshold".to_string());
        }

        // Validate monitoring interval
        if self.monitoring_interval_s < 10 || self.monitoring_interval_s > 300 {
            errors.push(format!("Power monitoring interval {}s outside valid range (10-300s)", self.monitoring_interval_s));
        }

        // Validate temperature limits
        if let Err(e) = self.temperature_limits.validate() {
            errors.push(format!("Temperature limits: {}", e));
        }

        // Validate power modes
        if let Err(e) = self.power_modes.validate() {
            errors.push(format!("Power modes: {}", e));
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(ConfigError::ValidationFailed(errors.join("; ")))
        }
    }
}

impl TemperatureLimits {
    pub fn validate(&self) -> Result<()> {
        let mut errors = Vec::new();

        // Validate temperature ranges
        if self.min_operating_c < -20.0 || self.min_operating_c > 10.0 {
            errors.push(format!("Minimum operating temperature {:.1}°C outside valid range (-20.0 to 10.0°C)", self.min_operating_c));
        }

        if self.max_operating_c < 40.0 || self.max_operating_c > 70.0 {
            errors.push(format!("Maximum operating temperature {:.1}°C outside valid range (40.0 to 70.0°C)", self.max_operating_c));
        }

        if self.warning_threshold_c < 35.0 || self.warning_threshold_c > 60.0 {
            errors.push(format!("Warning threshold {:.1}°C outside valid range (35.0 to 60.0°C)", self.warning_threshold_c));
        }

        if self.emergency_threshold_c < 50.0 || self.emergency_threshold_c > 80.0 {
            errors.push(format!("Emergency threshold {:.1}°C outside valid range (50.0 to 80.0°C)", self.emergency_threshold_c));
        }

        // Validate threshold ordering
        if self.min_operating_c >= self.max_operating_c {
            errors.push("Minimum operating temperature must be lower than maximum".to_string());
        }

        if self.warning_threshold_c >= self.emergency_threshold_c {
            errors.push("Warning threshold must be lower than emergency threshold".to_string());
        }

        if self.warning_threshold_c <= self.max_operating_c {
            errors.push("Warning threshold should be higher than maximum operating temperature".to_string());
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(ConfigError::ValidationFailed(errors.join("; ")))
        }
    }
}

impl PowerModeConfig {
    pub fn validate(&self) -> Result<()> {
        let mut errors = Vec::new();

        // Validate transmission multipliers
        if self.normal_transmission_multiplier != 1.0 {
            errors.push("Normal transmission multiplier must be 1.0".to_string());
        }

        if self.power_save_transmission_multiplier < 2.0 || self.power_save_transmission_multiplier > 5.0 {
            errors.push(format!("Power save transmission multiplier {:.1} outside valid range (2.0-5.0)", self.power_save_transmission_multiplier));
        }

        if self.emergency_transmission_multiplier < 0.5 || self.emergency_transmission_multiplier > 2.0 {
            errors.push(format!("Emergency transmission multiplier {:.1} outside valid range (0.5-2.0)", self.emergency_transmission_multiplier));
        }

        // Validate CPU scaling
        if let Err(e) = self.cpu_scaling.validate() {
            errors.push(format!("CPU scaling: {}", e));
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(ConfigError::ValidationFailed(errors.join("; ")))
        }
    }
}

impl CpuScalingConfig {
    pub fn validate(&self) -> Result<()> {
        let mut errors = Vec::new();

        // Validate frequency percentages
        if self.normal_frequency_percent < 80 || self.normal_frequency_percent > 100 {
            errors.push(format!("Normal frequency {}% outside valid range (80-100%)", self.normal_frequency_percent));
        }

        if self.power_save_frequency_percent < 40 || self.power_save_frequency_percent > 80 {
            errors.push(format!("Power save frequency {}% outside valid range (40-80%)", self.power_save_frequency_percent));
        }

        if self.emergency_frequency_percent < 60 || self.emergency_frequency_percent > 100 {
            errors.push(format!("Emergency frequency {}% outside valid range (60-100%)", self.emergency_frequency_percent));
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(ConfigError::ValidationFailed(errors.join("; ")))
        }
    }
}

impl CommunicationConfig {
    pub fn validate(&self) -> Result<()> {
        let mut errors = Vec::new();

        // Validate connection interval
        if self.connection_interval_hours < 1 || self.connection_interval_hours > 24 {
            errors.push(format!("Connection interval {}h outside valid range (1-24h)", self.connection_interval_hours));
        }

        // Validate retry configuration
        if self.max_retry_attempts < 1 || self.max_retry_attempts > 10 {
            errors.push(format!("Max retry attempts {} outside valid range (1-10)", self.max_retry_attempts));
        }

        if self.initial_retry_backoff_ms < 1000 || self.initial_retry_backoff_ms > 10000 {
            errors.push(format!("Initial retry backoff {}ms outside valid range (1000-10000ms)", self.initial_retry_backoff_ms));
        }

        if self.max_retry_interval_hours < 1 || self.max_retry_interval_hours > 12 {
            errors.push(format!("Max retry interval {}h outside valid range (1-12h)", self.max_retry_interval_hours));
        }

        // Validate connection timeout
        if self.connection_timeout_s < 30 || self.connection_timeout_s > 300 {
            errors.push(format!("Connection timeout {}s outside valid range (30-300s)", self.connection_timeout_s));
        }

        // Validate status report configuration
        if let Err(e) = self.status_report.validate() {
            errors.push(format!("Status report: {}", e));
        }

        // Validate endpoints
        if let Err(e) = self.endpoints.validate() {
            errors.push(format!("Endpoints: {}", e));
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(ConfigError::ValidationFailed(errors.join("; ")))
        }
    }
}

impl StatusReportConfig {
    pub fn validate(&self) -> Result<()> {
        let mut errors = Vec::new();

        // Validate position history settings
        if self.include_position_history && (self.max_position_history < 10 || self.max_position_history > 1000) {
            errors.push(format!("Max position history {} outside valid range (10-1000)", self.max_position_history));
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(ConfigError::ValidationFailed(errors.join("; ")))
        }
    }
}

impl CommunicationEndpoints {
    pub fn validate(&self) -> Result<()> {
        let mut errors = Vec::new();

        // Validate URLs
        if self.primary_url.is_empty() {
            errors.push("Primary URL cannot be empty".to_string());
        } else if !self.primary_url.starts_with("http://") && !self.primary_url.starts_with("https://") {
            errors.push("Primary URL must start with http:// or https://".to_string());
        }

        if let Some(ref backup_url) = self.backup_url {
            if !backup_url.starts_with("http://") && !backup_url.starts_with("https://") {
                errors.push("Backup URL must start with http:// or https://".to_string());
            }
        }

        // Validate authentication
        if self.api_key.is_empty() {
            errors.push("API key cannot be empty".to_string());
        }

        if self.device_token.is_empty() {
            errors.push("Device token cannot be empty".to_string());
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(ConfigError::ValidationFailed(errors.join("; ")))
        }
    }
}

impl EmergencyConfig {
    pub fn validate(&self) -> Result<()> {
        let mut errors = Vec::new();

        // Validate emergency interval
        if self.emergency_interval_ms < 100 || self.emergency_interval_ms > 5000 {
            errors.push(format!("Emergency interval {}ms outside valid range (100-5000ms)", self.emergency_interval_ms));
        }

        // Validate power boost
        if self.emergency_power_boost_percent < 100 || self.emergency_power_boost_percent > 200 {
            errors.push(format!("Emergency power boost {}% outside valid range (100-200%)", self.emergency_power_boost_percent));
        }

        // Validate emergency duration
        if self.max_emergency_duration_minutes < 5 || self.max_emergency_duration_minutes > 60 {
            errors.push(format!("Max emergency duration {}min outside valid range (5-60min)", self.max_emergency_duration_minutes));
        }

        // Validate shutdown conditions
        if let Err(e) = self.shutdown_conditions.validate() {
            errors.push(format!("Shutdown conditions: {}", e));
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(ConfigError::ValidationFailed(errors.join("; ")))
        }
    }
}

impl EmergencyShutdownConfig {
    pub fn validate(&self) -> Result<()> {
        let mut errors = Vec::new();

        // Validate grace period
        if self.shutdown_grace_period_s < 10 || self.shutdown_grace_period_s > 300 {
            errors.push(format!("Shutdown grace period {}s outside valid range (10-300s)", self.shutdown_grace_period_s));
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(ConfigError::ValidationFailed(errors.join("; ")))
        }
    }
}

impl HardwareConfig {
    pub fn validate(&self) -> Result<()> {
        let mut errors = Vec::new();

        // Validate GPIO pins
        if let Err(e) = self.gpio_pins.validate() {
            errors.push(format!("GPIO pins: {}", e));
        }

        // Validate SPI configuration
        if let Err(e) = self.spi_config.validate() {
            errors.push(format!("SPI config: {}", e));
        }

        // Validate I2C configuration
        if let Err(e) = self.i2c_config.validate() {
            errors.push(format!("I2C config: {}", e));
        }

        // Validate memory configuration
        if let Err(e) = self.memory_config.validate() {
            errors.push(format!("Memory config: {}", e));
        }

        // Validate watchdog configuration
        if let Err(e) = self.watchdog_config.validate() {
            errors.push(format!("Watchdog config: {}", e));
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(ConfigError::ValidationFailed(errors.join("; ")))
        }
    }
}

impl GpioPinConfig {
    pub fn validate(&self) -> Result<()> {
        let mut errors = Vec::new();
        let mut used_pins = std::collections::HashSet::new();

        // Validate pin numbers and check for conflicts
        if let Some(pin) = self.status_led_pin {
            if pin > 15 {
                errors.push(format!("Status LED pin {} exceeds maximum (15)", pin));
            }
            used_pins.insert(pin);
        }

        if let Some(pin) = self.emergency_button_pin {
            if pin > 15 {
                errors.push(format!("Emergency button pin {} exceeds maximum (15)", pin));
            }
            if used_pins.contains(&pin) {
                errors.push(format!("Pin {} is used by multiple functions", pin));
            }
            used_pins.insert(pin);
        }

        if let Some(pin) = self.power_control_pin {
            if pin > 15 {
                errors.push(format!("Power control pin {} exceeds maximum (15)", pin));
            }
            if used_pins.contains(&pin) {
                errors.push(format!("Pin {} is used by multiple functions", pin));
            }
            used_pins.insert(pin);
        }

        if let Some(pin) = self.external_reset_pin {
            if pin > 15 {
                errors.push(format!("External reset pin {} exceeds maximum (15)", pin));
            }
            if used_pins.contains(&pin) {
                errors.push(format!("Pin {} is used by multiple functions", pin));
            }
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(ConfigError::ValidationFailed(errors.join("; ")))
        }
    }
}

impl SpiConfig {
    pub fn validate(&self) -> Result<()> {
        let mut errors = Vec::new();

        // Validate clock frequency
        if self.clock_frequency_hz < 100000 || self.clock_frequency_hz > 10000000 {
            errors.push(format!("SPI clock frequency {}Hz outside valid range (100kHz-10MHz)", self.clock_frequency_hz));
        }

        // Validate SPI mode
        if self.mode > 3 {
            errors.push(format!("SPI mode {} invalid (must be 0-3)", self.mode));
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(ConfigError::ValidationFailed(errors.join("; ")))
        }
    }
}

impl I2cConfig {
    pub fn validate(&self) -> Result<()> {
        let mut errors = Vec::new();

        // Validate clock frequency
        if self.clock_frequency_hz < 10000 || self.clock_frequency_hz > 400000 {
            errors.push(format!("I2C clock frequency {}Hz outside valid range (10kHz-400kHz)", self.clock_frequency_hz));
        }

        // Validate timeout
        if self.timeout_ms < 100 || self.timeout_ms > 5000 {
            errors.push(format!("I2C timeout {}ms outside valid range (100-5000ms)", self.timeout_ms));
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(ConfigError::ValidationFailed(errors.join("; ")))
        }
    }
}

impl MemoryConfig {
    pub fn validate(&self) -> Result<()> {
        let mut errors = Vec::new();

        // Validate heap usage percentage
        if self.max_heap_usage_percent < 50 || self.max_heap_usage_percent > 80 {
            errors.push(format!("Max heap usage {}% outside valid range (50-80%)", self.max_heap_usage_percent));
        }

        // Validate stack size
        if self.stack_size_bytes < 2048 || self.stack_size_bytes > 8192 {
            errors.push(format!("Stack size {} bytes outside valid range (2048-8192 bytes)", self.stack_size_bytes));
        }

        // Validate memory check interval
        if self.memory_monitoring_enabled && (self.memory_check_interval_s < 10 || self.memory_check_interval_s > 300) {
            errors.push(format!("Memory check interval {}s outside valid range (10-300s)", self.memory_check_interval_s));
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(ConfigError::ValidationFailed(errors.join("; ")))
        }
    }
}

impl WatchdogConfig {
    pub fn validate(&self) -> Result<()> {
        let mut errors = Vec::new();

        // Validate watchdog timeout
        if self.enabled && (self.timeout_s < 1 || self.timeout_s > 30) {
            errors.push(format!("Watchdog timeout {}s outside valid range (1-30s)", self.timeout_s));
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(ConfigError::ValidationFailed(errors.join("; ")))
        }
    }
}

/// Configuration persistence and backup/restore functionality
impl BeaconConfig {
    /// Load configuration from file with validation
    pub fn load_from_file<P: AsRef<Path>>(path: P) -> Result<Self> {
        let content = fs::read_to_string(&path)
            .map_err(|e| ConfigError::IoError(format!("Failed to read config file '{}': {}", path.as_ref().display(), e)))?;
        
        let mut config: BeaconConfig = serde_json::from_str(&content)
            .map_err(|e| ConfigError::ParseError(format!("Failed to parse config JSON: {}", e)))?;
        
        // Validate loaded configuration
        config.validate()?;

        // Verify integrity if checksum is present
        if !config.metadata.checksum.is_empty() && !config.verify_integrity() {
            return Err(ConfigError::IntegrityError("Configuration checksum verification failed".to_string()));
        }

        // Update checksum if it was empty
        if config.metadata.checksum.is_empty() {
            config.metadata.checksum = config.calculate_checksum();
        }

        Ok(config)
    }

    /// Save configuration to file with backup
    pub fn save_to_file<P: AsRef<Path>>(&mut self, path: P) -> Result<()> {
        // Update metadata before saving
        self.update_metadata(None, None);

        // Validate before saving
        self.validate()?;

        // Create backup of existing file if it exists
        let path_ref = path.as_ref();
        if path_ref.exists() {
            let backup_path = self.create_backup_path(path_ref);
            if let Err(e) = fs::copy(path_ref, &backup_path) {
                eprintln!("Warning: Failed to create backup at '{}': {}", backup_path.display(), e);
            }
        }

        // Serialize configuration
        let json = serde_json::to_string_pretty(self)
            .map_err(|e| ConfigError::SerializationError(format!("Failed to serialize config: {}", e)))?;
        
        // Write to temporary file first, then rename for atomic operation
        let temp_path = path_ref.with_extension("tmp");
        fs::write(&temp_path, json)
            .map_err(|e| ConfigError::IoError(format!("Failed to write temp config file: {}", e)))?;

        // Atomic rename
        fs::rename(&temp_path, path_ref)
            .map_err(|e| ConfigError::IoError(format!("Failed to rename temp config file: {}", e)))?;

        Ok(())
    }

    /// Create backup file path
    fn create_backup_path(&self, original_path: &Path) -> PathBuf {
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs();
        
        let backup_name = format!("{}.backup.{}", 
            original_path.file_stem().unwrap().to_string_lossy(),
            timestamp);
        
        original_path.with_file_name(backup_name)
    }

    /// Create a backup copy of the current configuration
    pub fn create_backup(&self) -> BeaconConfigBackup {
        let mut backup_config = self.clone();
        // Ensure checksum is calculated for backup
        if backup_config.metadata.checksum.is_empty() {
            backup_config.metadata.checksum = backup_config.calculate_checksum();
        }
        
        BeaconConfigBackup {
            config: backup_config,
            backup_timestamp: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_secs(),
            original_checksum: self.metadata.checksum.clone(),
        }
    }

    /// Restore configuration from backup
    pub fn restore_from_backup(&mut self, backup: BeaconConfigBackup) -> Result<()> {
        // Validate backup configuration
        backup.config.validate()?;

        // Verify backup integrity
        if !backup.config.verify_integrity() {
            return Err(ConfigError::IntegrityError("Backup configuration integrity check failed".to_string()));
        }

        // Record migration for rollback
        self.metadata.migration_history.push(MigrationRecord {
            from_version: self.metadata.schema_version.clone(),
            to_version: backup.config.metadata.schema_version.clone(),
            migrated_at: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_secs(),
            description: "Restored from backup".to_string(),
            success: true,
        });

        // Restore configuration
        *self = backup.config;
        self.update_metadata(Some("Restored from backup".to_string()), None);

        Ok(())
    }

    /// Export configuration to different formats
    pub fn export_to_format(&self, format: ConfigExportFormat) -> Result<String> {
        match format {
            ConfigExportFormat::Json => {
                serde_json::to_string_pretty(self)
                    .map_err(|e| ConfigError::SerializationError(format!("JSON export failed: {}", e)))
            }
            ConfigExportFormat::Toml => {
                toml::to_string_pretty(self)
                    .map_err(|e| ConfigError::SerializationError(format!("TOML export failed: {}", e)))
            }
            ConfigExportFormat::Yaml => {
                serde_yaml::to_string(self)
                    .map_err(|e| ConfigError::SerializationError(format!("YAML export failed: {}", e)))
            }
        }
    }

    /// Import configuration from different formats
    pub fn import_from_format(data: &str, format: ConfigExportFormat) -> Result<Self> {
        let mut config: BeaconConfig = match format {
            ConfigExportFormat::Json => {
                serde_json::from_str(data)
                    .map_err(|e| ConfigError::ParseError(format!("JSON import failed: {}", e)))?
            }
            ConfigExportFormat::Toml => {
                toml::from_str(data)
                    .map_err(|e| ConfigError::ParseError(format!("TOML import failed: {}", e)))?
            }
            ConfigExportFormat::Yaml => {
                serde_yaml::from_str(data)
                    .map_err(|e| ConfigError::ParseError(format!("YAML import failed: {}", e)))?
            }
        };

        // Validate imported configuration
        config.validate()?;

        // Update checksum
        config.metadata.checksum = config.calculate_checksum();

        Ok(config)
    }
}

/// Configuration backup structure
#[derive(Debug, Clone)]
pub struct BeaconConfigBackup {
    /// Backed up configuration
    pub config: BeaconConfig,
    /// Backup creation timestamp
    pub backup_timestamp: u64,
    /// Original configuration checksum
    pub original_checksum: String,
}

/// Configuration export formats
#[derive(Debug, Clone, Copy)]
pub enum ConfigExportFormat {
    Json,
    Toml,
    Yaml,
}

/// Runtime configuration manager with validation and persistence
#[derive(Debug)]
pub struct BeaconConfigManager {
    /// Current active configuration
    current_config: BeaconConfig,
    /// Configuration file path
    config_path: PathBuf,
    /// Configuration backups (limited to last N backups)
    backups: Vec<BeaconConfigBackup>,
    /// Maximum number of backups to keep
    max_backups: usize,
    /// Configuration change history
    change_history: Vec<ConfigurationChange>,
    /// Maximum change history entries
    max_history: usize,
}

/// Record of configuration changes for auditing
#[derive(Debug, Clone)]
pub struct ConfigurationChange {
    /// Change timestamp
    pub timestamp: u64,
    /// Type of change
    pub change_type: ConfigurationChangeType,
    /// Field that was changed
    pub field_path: String,
    /// Previous value (serialized)
    pub previous_value: String,
    /// New value (serialized)
    pub new_value: String,
    /// Change description
    pub description: String,
    /// User/system that made the change
    pub changed_by: String,
}

/// Types of configuration changes
#[derive(Debug, Clone)]
pub enum ConfigurationChangeType {
    FieldUpdate,
    SectionUpdate,
    FullReload,
    BackupRestore,
    Migration,
    Validation,
}

impl BeaconConfigManager {
    /// Create a new configuration manager
    pub fn new<P: AsRef<Path>>(config_path: P, initial_config: BeaconConfig) -> Self {
        Self {
            current_config: initial_config,
            config_path: config_path.as_ref().to_path_buf(),
            backups: Vec::new(),
            max_backups: 10,
            change_history: Vec::new(),
            max_history: 100,
        }
    }

    /// Load configuration manager from file
    pub fn load_from_file<P: AsRef<Path>>(config_path: P) -> Result<Self> {
        let config = BeaconConfig::load_from_file(&config_path)?;
        Ok(Self::new(config_path, config))
    }

    /// Get current configuration (read-only)
    pub fn get_config(&self) -> &BeaconConfig {
        &self.current_config
    }

    /// Update configuration with validation
    pub fn update_config(&mut self, new_config: BeaconConfig, changed_by: String) -> Result<()> {
        // Validate new configuration
        new_config.validate()?;

        // Create backup of current configuration
        let backup = self.current_config.create_backup();
        self.add_backup(backup);

        // Record change
        self.record_change(
            ConfigurationChangeType::FullReload,
            "config".to_string(),
            self.current_config.metadata.checksum.clone(),
            new_config.metadata.checksum.clone(),
            "Full configuration update".to_string(),
            changed_by,
        );

        // Apply new configuration
        self.current_config = new_config;

        // Save to file
        self.save_config()?;

        Ok(())
    }

    /// Update specific configuration field with validation
    pub fn update_field<T>(&mut self, field_path: &str, new_value: T, changed_by: String) -> Result<()>
    where
        T: serde::Serialize + Clone,
    {
        // Create backup before change
        let backup = self.current_config.create_backup();
        
        // Get previous value for history
        let previous_value = serde_json::to_string(&self.get_field_value(field_path)?)
            .unwrap_or_else(|_| "unknown".to_string());
        
        // Apply field update (this would need specific implementation for each field)
        self.apply_field_update(field_path, new_value.clone())?;

        // Validate configuration after change
        if let Err(e) = self.current_config.validate() {
            // Rollback on validation failure
            self.current_config.restore_from_backup(backup)?;
            return Err(e);
        }

        // Add backup to history
        self.add_backup(backup);

        // Record change
        let new_value_str = serde_json::to_string(&new_value)
            .unwrap_or_else(|_| "unknown".to_string());
        
        self.record_change(
            ConfigurationChangeType::FieldUpdate,
            field_path.to_string(),
            previous_value,
            new_value_str,
            format!("Updated field: {}", field_path),
            changed_by,
        );

        // Save configuration
        self.save_config()?;

        Ok(())
    }

    /// Rollback to previous backup
    pub fn rollback_to_backup(&mut self, backup_index: usize, changed_by: String) -> Result<()> {
        if backup_index >= self.backups.len() {
            return Err(ConfigError::ValidationFailed("Backup index out of range".to_string()));
        }

        let backup = self.backups[backup_index].clone();
        
        // Record rollback
        self.record_change(
            ConfigurationChangeType::BackupRestore,
            "config".to_string(),
            self.current_config.metadata.checksum.clone(),
            backup.original_checksum.clone(),
            format!("Rollback to backup from timestamp {}", backup.backup_timestamp),
            changed_by,
        );

        // Restore from backup
        self.current_config.restore_from_backup(backup)?;

        // Save configuration
        self.save_config()?;

        Ok(())
    }

    /// Save current configuration to file
    pub fn save_config(&mut self) -> Result<()> {
        self.current_config.save_to_file(&self.config_path)
    }

    /// Get configuration change history
    pub fn get_change_history(&self) -> &[ConfigurationChange] {
        &self.change_history
    }

    /// Get available backups
    pub fn get_backups(&self) -> &[BeaconConfigBackup] {
        &self.backups
    }

    /// Add backup to history with size limit
    fn add_backup(&mut self, backup: BeaconConfigBackup) {
        self.backups.push(backup);
        
        // Limit backup history
        if self.backups.len() > self.max_backups {
            self.backups.remove(0);
        }
    }

    /// Record configuration change
    fn record_change(
        &mut self,
        change_type: ConfigurationChangeType,
        field_path: String,
        previous_value: String,
        new_value: String,
        description: String,
        changed_by: String,
    ) {
        let change = ConfigurationChange {
            timestamp: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_secs(),
            change_type,
            field_path,
            previous_value,
            new_value,
            description,
            changed_by,
        };

        self.change_history.push(change);

        // Limit history size
        if self.change_history.len() > self.max_history {
            self.change_history.remove(0);
        }
    }

    /// Get field value by path (simplified implementation)
    fn get_field_value(&self, field_path: &str) -> Result<serde_json::Value> {
        // This would need a more sophisticated implementation for real field access
        // For now, return the entire config serialized
        serde_json::to_value(&self.current_config)
            .map_err(|e| ConfigError::SerializationError(format!("Failed to serialize config: {}", e)))
    }

    /// Apply field update by path (simplified implementation)
    fn apply_field_update<T>(&mut self, field_path: &str, new_value: T) -> Result<()>
    where
        T: serde::Serialize,
    {
        // This would need specific implementation for each field path
        // For now, just update the modification timestamp
        self.current_config.update_metadata(
            Some(format!("Updated field: {}", field_path)),
            None,
        );
        Ok(())
    }
}

/// Configuration validation utilities
pub struct ConfigValidator;

impl ConfigValidator {
    /// Validate configuration against hardware constraints
    pub fn validate_hardware_constraints(config: &BeaconConfig) -> Result<()> {
        let mut errors = Vec::new();

        // Check memory constraints for ESP01-class devices
        let estimated_memory_usage = Self::estimate_memory_usage(config);
        let max_memory = 80 * 1024; // 80KB typical for ESP01
        
        if estimated_memory_usage > max_memory {
            errors.push(format!(
                "Estimated memory usage {} bytes exceeds ESP01 limit {} bytes",
                estimated_memory_usage, max_memory
            ));
        }

        // Check GPIO pin availability
        let used_pins = Self::count_used_gpio_pins(config);
        if used_pins > 4 { // ESP01 has only 4 usable GPIO pins
            errors.push(format!(
                "Configuration uses {} GPIO pins, but ESP01 only has 4 available",
                used_pins
            ));
        }

        // Check power consumption estimates
        let estimated_power = Self::estimate_power_consumption(config);
        if estimated_power > 1000.0 { // 1W typical limit for battery operation
            errors.push(format!(
                "Estimated power consumption {:.1}mW exceeds recommended limit for battery operation",
                estimated_power
            ));
        }

        if errors.is_empty() {
            Ok(())
        } else {
            Err(ConfigError::ValidationFailed(errors.join("; ")))
        }
    }

    /// Estimate memory usage based on configuration
    fn estimate_memory_usage(config: &BeaconConfig) -> u32 {
        let mut usage = 8192; // Base system usage
        
        // Add memory for position history
        if config.communication.status_report.include_position_history {
            usage += config.communication.status_report.max_position_history * 32; // ~32 bytes per position
        }
        
        // Add memory for transmission buffer
        usage += (config.transmission.interval_ms / 100) * 64; // Buffer based on interval
        
        // Add stack size
        usage += config.hardware.memory_config.stack_size_bytes;
        
        usage
    }

    /// Count used GPIO pins
    fn count_used_gpio_pins(config: &BeaconConfig) -> u32 {
        let mut count = 0;
        
        if config.hardware.gpio_pins.status_led_pin.is_some() { count += 1; }
        if config.hardware.gpio_pins.emergency_button_pin.is_some() { count += 1; }
        if config.hardware.gpio_pins.power_control_pin.is_some() { count += 1; }
        if config.hardware.gpio_pins.external_reset_pin.is_some() { count += 1; }
        
        count
    }

    /// Estimate power consumption in milliwatts
    fn estimate_power_consumption(config: &BeaconConfig) -> f32 {
        let mut power = 100.0; // Base system power
        
        // GPS power consumption
        let gps_duty_cycle = 1.0 / config.gps.update_interval_s as f32;
        power += gps_duty_cycle * 200.0; // GPS active power
        
        // Transmission power consumption
        let tx_duty_cycle = 0.1 / (config.transmission.interval_ms as f32 / 1000.0); // 100ms transmission
        power += tx_duty_cycle * (config.transmission.power_level as f32 * 2.0); // Power based on level
        
        // Communication power consumption
        let comm_duty_cycle = 1.0 / (config.communication.connection_interval_hours as f32 * 3600.0);
        power += comm_duty_cycle * 500.0; // Cellular/WiFi power
        
        power
    }
}

/// Configuration migration system for version updates
pub struct ConfigMigrator;

impl ConfigMigrator {
    /// Migrate configuration from one version to another
    pub fn migrate_config(config_data: &str, from_version: &str, to_version: &str) -> Result<BeaconConfig> {
        let migration_path = Self::get_migration_path(from_version, to_version)?;
        
        let mut current_data = config_data.to_string();
        let mut current_version = from_version.to_string();
        
        // Apply migrations in sequence
        for (from_ver, to_ver) in migration_path {
            current_data = Self::apply_migration(&current_data, &from_ver, &to_ver)?;
            current_version = to_ver;
        }
        
        // Parse final configuration
        let mut config: BeaconConfig = serde_json::from_str(&current_data)
            .map_err(|e| ConfigError::MigrationError(format!("Failed to parse migrated config: {}", e)))?;
        
        // Update migration history
        config.metadata.migration_history.push(MigrationRecord {
            from_version: from_version.to_string(),
            to_version: to_version.to_string(),
            migrated_at: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_secs(),
            description: format!("Migrated from {} to {}", from_version, to_version),
            success: true,
        });
        
        // Update schema version
        config.metadata.schema_version = to_version.to_string();
        config.update_metadata(Some("Configuration migrated".to_string()), None);
        
        // Validate migrated configuration
        config.validate()?;
        
        Ok(config)
    }

    /// Get migration path between versions
    fn get_migration_path(from_version: &str, to_version: &str) -> Result<Vec<(String, String)>> {
        let supported_versions = vec!["1.0.0", "1.1.0", "1.2.0", "2.0.0"];
        
        let from_index = supported_versions.iter().position(|&v| v == from_version)
            .ok_or_else(|| ConfigError::MigrationError(format!("Unsupported source version: {}", from_version)))?;
        
        let to_index = supported_versions.iter().position(|&v| v == to_version)
            .ok_or_else(|| ConfigError::MigrationError(format!("Unsupported target version: {}", to_version)))?;
        
        if from_index >= to_index {
            return Err(ConfigError::MigrationError("Cannot migrate to older or same version".to_string()));
        }
        
        let mut path = Vec::new();
        for i in from_index..to_index {
            path.push((
                supported_versions[i].to_string(),
                supported_versions[i + 1].to_string(),
            ));
        }
        
        Ok(path)
    }

    /// Apply single migration step
    fn apply_migration(config_data: &str, from_version: &str, to_version: &str) -> Result<String> {
        match (from_version, to_version) {
            ("1.0.0", "1.1.0") => Self::migrate_1_0_to_1_1(config_data),
            ("1.1.0", "1.2.0") => Self::migrate_1_1_to_1_2(config_data),
            ("1.2.0", "2.0.0") => Self::migrate_1_2_to_2_0(config_data),
            _ => Err(ConfigError::MigrationError(format!(
                "No migration available from {} to {}",
                from_version, to_version
            ))),
        }
    }

    /// Migrate from version 1.0.0 to 1.1.0
    /// Added: Hardware configuration section
    fn migrate_1_0_to_1_1(config_data: &str) -> Result<String> {
        let mut config: serde_json::Value = serde_json::from_str(config_data)
            .map_err(|e| ConfigError::MigrationError(format!("Failed to parse v1.0.0 config: {}", e)))?;
        
        // Add hardware configuration with defaults
        let hardware_config = serde_json::json!({
            "gpio_pins": {
                "status_led_pin": 2,
                "emergency_button_pin": null,
                "power_control_pin": null,
                "external_reset_pin": null
            },
            "spi_config": {
                "clock_frequency_hz": 1000000,
                "mode": 0,
                "msb_first": true
            },
            "i2c_config": {
                "clock_frequency_hz": 100000,
                "timeout_ms": 1000
            },
            "memory_config": {
                "max_heap_usage_percent": 70,
                "stack_size_bytes": 4096,
                "memory_monitoring_enabled": false,
                "memory_check_interval_s": 60
            },
            "watchdog_config": {
                "enabled": false,
                "timeout_s": 5,
                "auto_reset_enabled": true
            }
        });
        
        config["hardware"] = hardware_config;
        
        // Update schema version in metadata
        if let Some(metadata) = config.get_mut("metadata") {
            metadata["schema_version"] = serde_json::Value::String("1.1.0".to_string());
        }
        
        serde_json::to_string(&config)
            .map_err(|e| ConfigError::MigrationError(format!("Failed to serialize v1.1.0 config: {}", e)))
    }

    /// Migrate from version 1.1.0 to 1.2.0
    /// Added: Enhanced power management with CPU scaling
    fn migrate_1_1_to_1_2(config_data: &str) -> Result<String> {
        let mut config: serde_json::Value = serde_json::from_str(config_data)
            .map_err(|e| ConfigError::MigrationError(format!("Failed to parse v1.1.0 config: {}", e)))?;
        
        // Add CPU scaling to power configuration
        if let Some(power) = config.get_mut("power") {
            let cpu_scaling = serde_json::json!({
                "normal_frequency_percent": 100,
                "power_save_frequency_percent": 60,
                "emergency_frequency_percent": 80
            });
            
            if let Some(power_modes) = power.get_mut("power_modes") {
                power_modes["cpu_scaling"] = cpu_scaling;
            } else {
                power["power_modes"] = serde_json::json!({
                    "normal_transmission_multiplier": 1.0,
                    "power_save_transmission_multiplier": 2.0,
                    "emergency_transmission_multiplier": 0.5,
                    "cpu_scaling": cpu_scaling
                });
            }
        }
        
        // Update schema version
        if let Some(metadata) = config.get_mut("metadata") {
            metadata["schema_version"] = serde_json::Value::String("1.2.0".to_string());
        }
        
        serde_json::to_string(&config)
            .map_err(|e| ConfigError::MigrationError(format!("Failed to serialize v1.2.0 config: {}", e)))
    }

    /// Migrate from version 1.2.0 to 2.0.0
    /// Added: Message version V3 with UUID support, enhanced communication endpoints
    fn migrate_1_2_to_2_0(config_data: &str) -> Result<String> {
        let mut config: serde_json::Value = serde_json::from_str(config_data)
            .map_err(|e| ConfigError::MigrationError(format!("Failed to parse v1.2.0 config: {}", e)))?;
        
        // Update transmission configuration for V3 message support
        if let Some(transmission) = config.get_mut("transmission") {
            // Change default message version to V3 if it was V2
            if transmission.get("message_version").and_then(|v| v.as_str()) == Some("V2") {
                transmission["message_version"] = serde_json::Value::String("V3".to_string());
            }
            
            // Add sequence rollover if not present
            if transmission.get("sequence_rollover").is_none() {
                transmission["sequence_rollover"] = serde_json::Value::Number(serde_json::Number::from(65535));
            }
        }
        
        // Enhance communication endpoints
        if let Some(communication) = config.get_mut("communication") {
            if let Some(endpoints) = communication.get_mut("endpoints") {
                // Add device token if not present
                if endpoints.get("device_token").is_none() {
                    endpoints["device_token"] = serde_json::Value::String("your-device-token".to_string());
                }
            }
        }
        
        // Update schema version
        if let Some(metadata) = config.get_mut("metadata") {
            metadata["schema_version"] = serde_json::Value::String("2.0.0".to_string());
        }
        
        serde_json::to_string(&config)
            .map_err(|e| ConfigError::MigrationError(format!("Failed to serialize v2.0.0 config: {}", e)))
    }

    /// Check if migration is needed
    pub fn needs_migration(config_data: &str, target_version: &str) -> Result<bool> {
        let config: serde_json::Value = serde_json::from_str(config_data)
            .map_err(|e| ConfigError::ParseError(format!("Failed to parse config for migration check: {}", e)))?;
        
        let current_version = config
            .get("metadata")
            .and_then(|m| m.get("schema_version"))
            .and_then(|v| v.as_str())
            .unwrap_or("1.0.0");
        
        Ok(current_version != target_version)
    }

    /// Get current schema version from config data
    pub fn get_schema_version(config_data: &str) -> Result<String> {
        let config: serde_json::Value = serde_json::from_str(config_data)
            .map_err(|e| ConfigError::ParseError(format!("Failed to parse config: {}", e)))?;
        
        let version = config
            .get("metadata")
            .and_then(|m| m.get("schema_version"))
            .and_then(|v| v.as_str())
            .unwrap_or("1.0.0");
        
        Ok(version.to_string())
    }

    /// Validate migration compatibility
    pub fn validate_migration_compatibility(from_version: &str, to_version: &str) -> Result<()> {
        let supported_versions = vec!["1.0.0", "1.1.0", "1.2.0", "2.0.0"];
        
        if !supported_versions.contains(&from_version) {
            return Err(ConfigError::MigrationError(format!("Unsupported source version: {}", from_version)));
        }
        
        if !supported_versions.contains(&to_version) {
            return Err(ConfigError::MigrationError(format!("Unsupported target version: {}", to_version)));
        }
        
        // Check version ordering
        let from_index = supported_versions.iter().position(|&v| v == from_version).unwrap();
        let to_index = supported_versions.iter().position(|&v| v == to_version).unwrap();
        
        if from_index >= to_index {
            return Err(ConfigError::MigrationError("Cannot migrate to older or same version".to_string()));
        }
        
        Ok(())
    }
}

/// Configuration template system for common use cases
pub struct ConfigTemplates;

impl ConfigTemplates {
    /// Create configuration template for marine research deployment
    pub fn marine_research(beacon_id: Uuid) -> BeaconConfig {
        let mut config = BeaconConfig::high_reliability(beacon_id);
        
        // Optimize for research requirements
        config.transmission.interval_ms = 2000; // Frequent updates for tracking
        config.gps.accuracy_threshold_m = 1.0; // High accuracy for research
        config.gps.min_satellite_count = 6; // Require good GPS fix
        
        config.communication.connection_interval_hours = 4; // Regular data uploads
        config.communication.status_report.include_position_history = true;
        config.communication.status_report.max_position_history = 500;
        
        config.metadata.description = "Marine research optimized configuration".to_string();
        config
    }

    /// Create configuration template for commercial fishing operations
    pub fn commercial_fishing(beacon_id: Uuid) -> BeaconConfig {
        let mut config = BeaconConfig::low_power(beacon_id);
        
        // Optimize for long deployment periods
        config.transmission.interval_ms = 5000; // Longer intervals for battery life
        config.gps.update_interval_s = 4; // GPS update interval shorter than transmission
        config.power.power_save_threshold_percent = 40.0; // Conservative power management
        
        config.communication.connection_interval_hours = 12; // Less frequent communication
        config.emergency.emergency_mode_enabled = true; // Important for safety
        
        config.metadata.description = "Commercial fishing optimized configuration".to_string();
        config
    }

    /// Create configuration template for search and rescue operations
    pub fn search_rescue(beacon_id: Uuid) -> BeaconConfig {
        let mut config = BeaconConfig::high_reliability(beacon_id);
        
        // Optimize for emergency response
        config.transmission.interval_ms = 1000; // Very frequent updates
        config.transmission.power_level = 255; // Maximum power
        
        config.emergency.emergency_interval_ms = 500; // Rapid emergency updates
        config.emergency.emergency_power_boost_percent = 200; // Maximum emergency power
        
        config.communication.connection_interval_hours = 1; // Frequent status updates
        
        config.metadata.description = "Search and rescue optimized configuration".to_string();
        config
    }

    /// Create configuration template for environmental monitoring
    pub fn environmental_monitoring(beacon_id: Uuid) -> BeaconConfig {
        let mut config = BeaconConfig::low_power(beacon_id);
        
        // Optimize for long-term monitoring
        config.transmission.interval_ms = 10000; // Infrequent updates for battery life
        config.gps.update_interval_s = 8; // Less frequent GPS updates but shorter than transmission
        
        config.communication.connection_interval_hours = 24; // Daily data uploads
        config.communication.status_report.include_system_health = true; // Monitor system health
        
        config.power.solar_charging_enabled = true; // Enable solar charging for long deployments
        
        config.metadata.description = "Environmental monitoring optimized configuration".to_string();
        config
    }

    /// Get all available templates
    pub fn get_available_templates() -> Vec<(&'static str, &'static str)> {
        vec![
            ("marine_research", "Marine Research - High accuracy and reliability for scientific applications"),
            ("commercial_fishing", "Commercial Fishing - Power optimized for long deployments"),
            ("search_rescue", "Search & Rescue - Maximum performance for emergency response"),
            ("environmental_monitoring", "Environmental Monitoring - Ultra low power for long-term studies"),
        ]
    }

    /// Create configuration from template name
    pub fn create_from_template(template_name: &str, beacon_id: Uuid) -> Result<BeaconConfig> {
        match template_name {
            "marine_research" => Ok(Self::marine_research(beacon_id)),
            "commercial_fishing" => Ok(Self::commercial_fishing(beacon_id)),
            "search_rescue" => Ok(Self::search_rescue(beacon_id)),
            "environmental_monitoring" => Ok(Self::environmental_monitoring(beacon_id)),
            _ => Err(ConfigError::ValidationFailed(format!("Unknown template: {}", template_name))),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::NamedTempFile;

    #[test]
    fn test_beacon_config_creation() {
        let beacon_id = Uuid::new_v4();
        let config = BeaconConfig::new(beacon_id);
        
        assert_eq!(config.beacon_id, beacon_id);
        if let Err(e) = config.validate() {
            panic!("Validation failed: {}", e);
        }
    }

    #[test]
    fn test_config_validation() {
        let beacon_id = Uuid::new_v4();
        let mut config = BeaconConfig::new(beacon_id);
        
        // Test invalid transmission interval
        config.transmission.interval_ms = 500; // Too short
        assert!(config.validate().is_err());
        
        // Fix and test again
        config.transmission.interval_ms = 3000;
        assert!(config.validate().is_ok());
    }

    #[test]
    fn test_config_persistence() {
        let beacon_id = Uuid::new_v4();
        let mut config = BeaconConfig::new(beacon_id);
        
        let temp_file = NamedTempFile::new().unwrap();
        let path = temp_file.path();
        
        // Save configuration
        assert!(config.save_to_file(path).is_ok());
        
        // Load configuration
        let loaded_config = BeaconConfig::load_from_file(path).unwrap();
        assert_eq!(config.beacon_id, loaded_config.beacon_id);
    }

    #[test]
    fn test_config_migration() {
        let config_v1 = r#"{
            "beacon_id": "550e8400-e29b-41d4-a716-446655440000",
            "transmission": {
                "interval_ms": 3000,
                "message_version": "V2",
                "power_level": 200,
                "max_retries": 3,
                "retry_delay_ms": 1000,
                "adaptive_power": false,
                "sequence_rollover": 65535
            },
            "gps": {
                "acquisition_timeout_s": 60,
                "update_interval_s": 2,
                "min_satellite_count": 4,
                "accuracy_threshold_m": 3.0,
                "cold_start_timeout_s": 120,
                "enable_dgps": false,
                "max_fix_age_s": 30
            },
            "power": {
                "low_battery_threshold_percent": 20.0,
                "critical_battery_threshold_percent": 10.0,
                "emergency_battery_threshold_percent": 5.0,
                "power_save_threshold_percent": 25.0,
                "charging_enabled": true,
                "solar_charging_enabled": false,
                "monitoring_interval_s": 30,
                "temperature_limits": {
                    "min_operating_c": -10.0,
                    "max_operating_c": 50.0,
                    "warning_threshold_c": 55.0,
                    "emergency_threshold_c": 65.0
                },
                "power_modes": {
                    "normal_transmission_multiplier": 1.0,
                    "power_save_transmission_multiplier": 2.0,
                    "emergency_transmission_multiplier": 0.5,
                    "cpu_scaling": {
                        "normal_frequency_percent": 100,
                        "power_save_frequency_percent": 60,
                        "emergency_frequency_percent": 80
                    }
                }
            },
            "communication": {
                "connection_interval_hours": 6,
                "max_retry_attempts": 5,
                "initial_retry_backoff_ms": 2000,
                "max_retry_interval_hours": 4,
                "connection_timeout_s": 120,
                "compression_enabled": true,
                "status_report": {
                    "include_position_history": true,
                    "max_position_history": 100,
                    "include_battery_details": true,
                    "include_transmission_stats": true,
                    "include_system_health": false
                },
                "endpoints": {
                    "primary_url": "https://api.example.com/beacon",
                    "backup_url": null,
                    "api_key": "test-api-key",
                    "device_token": "test-device-token"
                }
            },
            "emergency": {
                "emergency_mode_enabled": true,
                "emergency_interval_ms": 1000,
                "emergency_power_boost_percent": 150,
                "max_emergency_duration_minutes": 30,
                "auto_recovery_enabled": false,
                "shutdown_conditions": {
                    "battery_shutdown_enabled": true,
                    "temperature_shutdown_enabled": true,
                    "hardware_fault_shutdown_enabled": true,
                    "shutdown_grace_period_s": 60
                }
            },
            "hardware": {
                "gpio_pins": {
                    "status_led_pin": 2,
                    "emergency_button_pin": null,
                    "power_control_pin": null,
                    "external_reset_pin": null
                },
                "spi_config": {
                    "clock_frequency_hz": 1000000,
                    "mode": 0,
                    "msb_first": true
                },
                "i2c_config": {
                    "clock_frequency_hz": 100000,
                    "timeout_ms": 1000
                },
                "memory_config": {
                    "max_heap_usage_percent": 70,
                    "stack_size_bytes": 4096,
                    "memory_monitoring_enabled": false,
                    "memory_check_interval_s": 60
                },
                "watchdog_config": {
                    "enabled": false,
                    "timeout_s": 5,
                    "auto_reset_enabled": true
                }
            },
            "metadata": {
                "schema_version": "1.0.0",
                "created_at": 1640995200,
                "modified_at": 1640995200,
                "description": "Test config",
                "author": "Test",
                "checksum": "",
                "migration_history": []
            }
        }"#;
        
        let migrated = ConfigMigrator::migrate_config(config_v1, "1.0.0", "1.1.0").unwrap();
        assert_eq!(migrated.metadata.schema_version, "1.1.0");
        assert!(migrated.validate().is_ok());
    }

    #[test]
    fn test_config_templates() {
        let beacon_id = Uuid::new_v4();
        
        let marine_config = ConfigTemplates::marine_research(beacon_id);
        assert!(marine_config.validate().is_ok());
        assert_eq!(marine_config.transmission.interval_ms, 2000);
        
        let fishing_config = ConfigTemplates::commercial_fishing(beacon_id);
        assert!(fishing_config.validate().is_ok());
        assert_eq!(fishing_config.transmission.interval_ms, 5000);
    }

    #[test]
    fn test_config_manager() {
        let beacon_id = Uuid::new_v4();
        let config = BeaconConfig::new(beacon_id);
        let temp_file = NamedTempFile::new().unwrap();
        
        let mut manager = BeaconConfigManager::new(temp_file.path(), config);
        
        // Test configuration update
        let new_config = BeaconConfig::low_power(beacon_id);
        assert!(manager.update_config(new_config, "test_user".to_string()).is_ok());
        
        // Check that backup was created
        assert!(!manager.get_backups().is_empty());
        
        // Test rollback
        if let Err(e) = manager.rollback_to_backup(0, "test_user".to_string()) {
            panic!("Rollback failed: {}", e);
        }
        
        // Check history
        assert!(!manager.get_change_history().is_empty());
    }
}