use std::time::{Duration, SystemTime};
use serde::{Deserialize, Serialize};

/// Power management errors
#[derive(Debug, Clone, PartialEq)]
pub enum PowerError {
    BatteryDepleted,
    ChargingFault,
    TemperatureExtreme { temperature_c: f32 },
    VoltageOutOfRange { voltage_v: f32 },
    CurrentOverload { current_ma: f32 },
    HardwareFault(String),
    ConfigurationInvalid(String),
    ThresholdViolation { threshold_type: String, value: f32 },
}

impl std::fmt::Display for PowerError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PowerError::BatteryDepleted => write!(f, "Battery depleted"),
            PowerError::ChargingFault => write!(f, "Charging system fault"),
            PowerError::TemperatureExtreme { temperature_c } => {
                write!(f, "Temperature extreme: {}°C", temperature_c)
            }
            PowerError::VoltageOutOfRange { voltage_v } => {
                write!(f, "Voltage out of range: {}V", voltage_v)
            }
            PowerError::CurrentOverload { current_ma } => {
                write!(f, "Current overload: {}mA", current_ma)
            }
            PowerError::HardwareFault(msg) => write!(f, "Hardware fault: {}", msg),
            PowerError::ConfigurationInvalid(msg) => write!(f, "Invalid configuration: {}", msg),
            PowerError::ThresholdViolation { threshold_type, value } => {
                write!(f, "Threshold violation - {}: {}", threshold_type, value)
            }
        }
    }
}

impl std::error::Error for PowerError {}

/// Battery health status
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum BatteryHealth {
    Good,
    Fair,
    Poor,
    Critical,
    Unknown,
}

/// Charging status information
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ChargingStatus {
    NotCharging,
    Charging { rate_ma: f32 },
    ChargingComplete,
    ChargingFault(String),
    SolarCharging { rate_ma: f32, solar_voltage_v: f32 },
}

/// Power operation modes
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PowerOperationMode {
    Normal,
    PowerSave,
    Emergency,
    Shutdown,
}

/// Battery status information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BatteryStatus {
    pub voltage_v: f32,
    pub current_ma: f32,
    pub capacity_percent: f32,
    pub temperature_c: f32,
    pub health: BatteryHealth,
    pub timestamp: SystemTime,
    pub cycles: u32,
    pub time_to_empty: Option<Duration>,
    pub time_to_full: Option<Duration>,
}

impl BatteryStatus {
    pub fn new(voltage_v: f32, current_ma: f32, capacity_percent: f32, temperature_c: f32) -> Self {
        Self {
            voltage_v,
            current_ma,
            capacity_percent,
            temperature_c,
            health: BatteryHealth::Unknown,
            timestamp: SystemTime::now(),
            cycles: 0,
            time_to_empty: None,
            time_to_full: None,
        }
    }

    /// Check if battery is in critical state
    pub fn is_critical(&self) -> bool {
        self.capacity_percent < 5.0 || 
        self.health == BatteryHealth::Critical ||
        self.temperature_c < -20.0 || 
        self.temperature_c > 60.0
    }

    /// Check if battery is low
    pub fn is_low(&self, threshold: f32) -> bool {
        self.capacity_percent < threshold
    }
}

/// Power configuration parameters
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct PowerConfig {
    pub low_battery_threshold_percent: f32,
    pub critical_battery_threshold_percent: f32,
    pub emergency_battery_threshold_percent: f32,
    pub power_save_mode_threshold_percent: f32,
    pub charging_enabled: bool,
    pub solar_charging_enabled: bool,
    pub temperature_min_c: f32,
    pub temperature_max_c: f32,
    pub voltage_min_v: f32,
    pub voltage_max_v: f32,
    pub current_max_ma: f32,
    pub monitoring_interval_ms: u32,
}

impl Default for PowerConfig {
    fn default() -> Self {
        Self {
            low_battery_threshold_percent: 20.0,
            critical_battery_threshold_percent: 10.0,
            emergency_battery_threshold_percent: 5.0,
            power_save_mode_threshold_percent: 30.0,
            charging_enabled: true,
            solar_charging_enabled: true,
            temperature_min_c: -20.0,
            temperature_max_c: 60.0,
            voltage_min_v: 3.0,
            voltage_max_v: 4.2,
            current_max_ma: 2000.0,
            monitoring_interval_ms: 1000,
        }
    }
}

impl PowerConfig {
    /// Validate power configuration parameters
    pub fn validate(&self) -> Result<(), PowerError> {
        if self.low_battery_threshold_percent <= self.critical_battery_threshold_percent {
            return Err(PowerError::ConfigurationInvalid(
                "Low battery threshold must be higher than critical threshold".to_string()
            ));
        }

        if self.critical_battery_threshold_percent <= self.emergency_battery_threshold_percent {
            return Err(PowerError::ConfigurationInvalid(
                "Critical battery threshold must be higher than emergency threshold".to_string()
            ));
        }

        if self.temperature_min_c >= self.temperature_max_c {
            return Err(PowerError::ConfigurationInvalid(
                "Minimum temperature must be less than maximum temperature".to_string()
            ));
        }

        if self.voltage_min_v >= self.voltage_max_v {
            return Err(PowerError::ConfigurationInvalid(
                "Minimum voltage must be less than maximum voltage".to_string()
            ));
        }

        if self.current_max_ma <= 0.0 {
            return Err(PowerError::ConfigurationInvalid(
                "Maximum current must be positive".to_string()
            ));
        }

        Ok(())
    }
}

/// Power management trait for battery monitoring and power control
pub trait PowerManager {
    /// Get current battery status
    fn get_battery_status(&self) -> Result<BatteryStatus, PowerError>;

    /// Get current charging status
    fn get_charging_status(&self) -> Result<ChargingStatus, PowerError>;

    /// Set power operation mode
    fn set_power_mode(&mut self, mode: PowerOperationMode) -> Result<(), PowerError>;

    /// Get current power mode
    fn get_power_mode(&self) -> PowerOperationMode;

    /// Estimate remaining battery time based on current usage
    fn estimate_remaining_time(&self) -> Result<Duration, PowerError>;

    /// Configure power management parameters
    fn configure_power_thresholds(&mut self, config: PowerConfig) -> Result<(), PowerError>;

    /// Get current power configuration
    fn get_power_config(&self) -> &PowerConfig;

    /// Check if any power thresholds are violated
    fn check_thresholds(&self) -> Result<Vec<PowerError>, PowerError>;

    /// Enable or disable charging
    fn set_charging_enabled(&mut self, enabled: bool) -> Result<(), PowerError>;

    /// Enable or disable solar charging
    fn set_solar_charging_enabled(&mut self, enabled: bool) -> Result<(), PowerError>;

    /// Get power consumption statistics
    fn get_power_stats(&self) -> PowerStats;

    /// Reset power statistics
    fn reset_power_stats(&mut self);

    /// Perform emergency shutdown preparation
    fn prepare_emergency_shutdown(&mut self) -> Result<(), PowerError>;
}

/// Power consumption and usage statistics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PowerStats {
    pub average_current_ma: f32,
    pub peak_current_ma: f32,
    pub total_energy_consumed_mah: f32,
    pub uptime: Duration,
    pub charging_cycles: u32,
    pub last_full_charge: Option<SystemTime>,
    pub power_mode_history: Vec<(PowerOperationMode, Duration)>,
}

impl Default for PowerStats {
    fn default() -> Self {
        Self {
            average_current_ma: 0.0,
            peak_current_ma: 0.0,
            total_energy_consumed_mah: 0.0,
            uptime: Duration::from_secs(0),
            charging_cycles: 0,
            last_full_charge: None,
            power_mode_history: Vec::new(),
        }
    }
}
/// Basic power manager implementation for testing and development
pub struct BasicPowerManager {
    config: PowerConfig,
    current_mode: PowerOperationMode,
    battery_status: BatteryStatus,
    charging_status: ChargingStatus,
    stats: PowerStats,
    start_time: SystemTime,
    last_update: SystemTime,
    charging_enabled: bool,
    solar_charging_enabled: bool,
}

impl BasicPowerManager {
    pub fn new(config: PowerConfig) -> Result<Self, PowerError> {
        config.validate()?;
        
        let now = SystemTime::now();
        Ok(Self {
            config,
            current_mode: PowerOperationMode::Normal,
            battery_status: BatteryStatus::new(3.7, 100.0, 75.0, 25.0),
            charging_status: ChargingStatus::NotCharging,
            stats: PowerStats::default(),
            start_time: now,
            last_update: now,
            charging_enabled: true,
            solar_charging_enabled: true,
        })
    }

    /// Simulate battery discharge based on current consumption and time elapsed
    fn update_battery_simulation(&mut self) {
        let now = SystemTime::now();
        let elapsed = now.duration_since(self.last_update).unwrap_or(Duration::from_secs(0));
        self.last_update = now;

        // Simulate discharge based on power mode
        let discharge_rate = match self.current_mode {
            PowerOperationMode::Normal => 150.0, // mA
            PowerOperationMode::PowerSave => 50.0, // mA
            PowerOperationMode::Emergency => 25.0, // mA
            PowerOperationMode::Shutdown => 5.0, // mA
        };

        // Calculate capacity decrease
        let hours_elapsed = elapsed.as_secs_f32() / 3600.0;
        let capacity_decrease = (discharge_rate * hours_elapsed) / 2000.0; // Assuming 2000mAh battery
        
        self.battery_status.capacity_percent = (self.battery_status.capacity_percent - capacity_decrease).max(0.0);
        self.battery_status.current_ma = discharge_rate;
        self.battery_status.timestamp = now;

        // Update voltage based on capacity (simplified model)
        self.battery_status.voltage_v = 3.0 + (self.battery_status.capacity_percent / 100.0) * 1.2;

        // Update battery health based on capacity and temperature
        self.battery_status.health = if self.battery_status.capacity_percent < 5.0 {
            BatteryHealth::Critical
        } else if self.battery_status.capacity_percent < 20.0 {
            BatteryHealth::Poor
        } else if self.battery_status.capacity_percent < 50.0 {
            BatteryHealth::Fair
        } else {
            BatteryHealth::Good
        };

        // Estimate time to empty
        if discharge_rate > 0.0 {
            let remaining_mah = (self.battery_status.capacity_percent / 100.0) * 2000.0;
            let hours_remaining = remaining_mah / discharge_rate;
            self.battery_status.time_to_empty = Some(Duration::from_secs((hours_remaining * 3600.0) as u64));
        }

        // Update statistics
        self.stats.uptime = now.duration_since(self.start_time).unwrap_or(Duration::from_secs(0));
        self.stats.total_energy_consumed_mah += discharge_rate * hours_elapsed;
        self.stats.average_current_ma = self.stats.total_energy_consumed_mah / (self.stats.uptime.as_secs_f32() / 3600.0);
        self.stats.peak_current_ma = self.stats.peak_current_ma.max(discharge_rate);
    }

    /// Check if charging should be active based on configuration and conditions
    fn update_charging_status(&mut self) {
        if !self.charging_enabled {
            self.charging_status = ChargingStatus::NotCharging;
            return;
        }

        // Simulate charging logic
        if self.battery_status.capacity_percent < 95.0 {
            if self.solar_charging_enabled && self.battery_status.capacity_percent < 80.0 {
                // Simulate solar charging
                self.charging_status = ChargingStatus::SolarCharging {
                    rate_ma: 200.0,
                    solar_voltage_v: 5.0,
                };
                
                // Increase battery capacity during charging
                let now = SystemTime::now();
                let elapsed = now.duration_since(self.last_update).unwrap_or(Duration::from_secs(0));
                let hours_elapsed = elapsed.as_secs_f32() / 3600.0;
                let capacity_increase = (200.0 * hours_elapsed) / 2000.0;
                self.battery_status.capacity_percent = (self.battery_status.capacity_percent + capacity_increase).min(100.0);
            } else {
                self.charging_status = ChargingStatus::Charging { rate_ma: 500.0 };
            }
        } else {
            self.charging_status = ChargingStatus::ChargingComplete;
        }
    }
}

impl PowerManager for BasicPowerManager {
    fn get_battery_status(&self) -> Result<BatteryStatus, PowerError> {
        Ok(self.battery_status.clone())
    }

    fn get_charging_status(&self) -> Result<ChargingStatus, PowerError> {
        Ok(self.charging_status.clone())
    }

    fn set_power_mode(&mut self, mode: PowerOperationMode) -> Result<(), PowerError> {
        // Update simulation before changing mode
        self.update_battery_simulation();
        
        // Record mode change in history
        let duration_in_mode = SystemTime::now()
            .duration_since(self.last_update)
            .unwrap_or(Duration::from_secs(0));
        
        self.stats.power_mode_history.push((self.current_mode.clone(), duration_in_mode));
        
        self.current_mode = mode;
        Ok(())
    }

    fn get_power_mode(&self) -> PowerOperationMode {
        self.current_mode.clone()
    }

    fn estimate_remaining_time(&self) -> Result<Duration, PowerError> {
        let current_draw = match self.current_mode {
            PowerOperationMode::Normal => 150.0,
            PowerOperationMode::PowerSave => 50.0,
            PowerOperationMode::Emergency => 25.0,
            PowerOperationMode::Shutdown => 5.0,
        };

        if current_draw <= 0.0 {
            return Ok(Duration::from_secs(u64::MAX));
        }

        let remaining_capacity_mah = (self.battery_status.capacity_percent / 100.0) * 2000.0;
        let hours_remaining = remaining_capacity_mah / current_draw;
        
        Ok(Duration::from_secs((hours_remaining * 3600.0) as u64))
    }

    fn configure_power_thresholds(&mut self, config: PowerConfig) -> Result<(), PowerError> {
        config.validate()?;
        self.config = config;
        Ok(())
    }

    fn get_power_config(&self) -> &PowerConfig {
        &self.config
    }

    fn check_thresholds(&self) -> Result<Vec<PowerError>, PowerError> {
        let mut violations = Vec::new();

        // Check battery capacity thresholds
        if self.battery_status.capacity_percent <= self.config.emergency_battery_threshold_percent {
            violations.push(PowerError::ThresholdViolation {
                threshold_type: "Emergency battery".to_string(),
                value: self.battery_status.capacity_percent,
            });
        } else if self.battery_status.capacity_percent <= self.config.critical_battery_threshold_percent {
            violations.push(PowerError::ThresholdViolation {
                threshold_type: "Critical battery".to_string(),
                value: self.battery_status.capacity_percent,
            });
        } else if self.battery_status.capacity_percent <= self.config.low_battery_threshold_percent {
            violations.push(PowerError::ThresholdViolation {
                threshold_type: "Low battery".to_string(),
                value: self.battery_status.capacity_percent,
            });
        }

        // Check temperature thresholds
        if self.battery_status.temperature_c < self.config.temperature_min_c {
            violations.push(PowerError::TemperatureExtreme {
                temperature_c: self.battery_status.temperature_c,
            });
        } else if self.battery_status.temperature_c > self.config.temperature_max_c {
            violations.push(PowerError::TemperatureExtreme {
                temperature_c: self.battery_status.temperature_c,
            });
        }

        // Check voltage thresholds
        if self.battery_status.voltage_v < self.config.voltage_min_v {
            violations.push(PowerError::VoltageOutOfRange {
                voltage_v: self.battery_status.voltage_v,
            });
        } else if self.battery_status.voltage_v > self.config.voltage_max_v {
            violations.push(PowerError::VoltageOutOfRange {
                voltage_v: self.battery_status.voltage_v,
            });
        }

        // Check current thresholds
        if self.battery_status.current_ma > self.config.current_max_ma {
            violations.push(PowerError::CurrentOverload {
                current_ma: self.battery_status.current_ma,
            });
        }

        Ok(violations)
    }

    fn set_charging_enabled(&mut self, enabled: bool) -> Result<(), PowerError> {
        self.charging_enabled = enabled;
        if !enabled {
            self.charging_status = ChargingStatus::NotCharging;
        }
        Ok(())
    }

    fn set_solar_charging_enabled(&mut self, enabled: bool) -> Result<(), PowerError> {
        self.solar_charging_enabled = enabled;
        Ok(())
    }

    fn get_power_stats(&self) -> PowerStats {
        self.stats.clone()
    }

    fn reset_power_stats(&mut self) {
        self.stats = PowerStats::default();
        self.start_time = SystemTime::now();
    }

    fn prepare_emergency_shutdown(&mut self) -> Result<(), PowerError> {
        // Set to emergency mode
        self.set_power_mode(PowerOperationMode::Emergency)?;
        
        // Disable charging to prevent any power draw
        self.set_charging_enabled(false)?;
        
        // Log emergency shutdown preparation
        println!("Emergency shutdown prepared - battery: {:.1}%", self.battery_status.capacity_percent);
        
        Ok(())
    }
}

/// Mock power manager for testing and simulation
pub struct MockPowerManager {
    config: PowerConfig,
    current_mode: PowerOperationMode,
    battery_status: BatteryStatus,
    charging_status: ChargingStatus,
    stats: PowerStats,
    charging_enabled: bool,
    solar_charging_enabled: bool,
    simulate_faults: bool,
    threshold_violations: Vec<PowerError>,
}

impl MockPowerManager {
    pub fn new() -> Self {
        Self {
            config: PowerConfig::default(),
            current_mode: PowerOperationMode::Normal,
            battery_status: BatteryStatus::new(3.7, 100.0, 85.0, 25.0),
            charging_status: ChargingStatus::NotCharging,
            stats: PowerStats::default(),
            charging_enabled: true,
            solar_charging_enabled: true,
            simulate_faults: false,
            threshold_violations: Vec::new(),
        }
    }

    pub fn with_config(config: PowerConfig) -> Result<Self, PowerError> {
        config.validate()?;
        Ok(Self {
            config,
            ..Self::new()
        })
    }

    /// Set simulated battery status for testing
    pub fn set_battery_status(&mut self, status: BatteryStatus) {
        self.battery_status = status;
    }

    /// Set simulated charging status for testing
    pub fn set_charging_status(&mut self, status: ChargingStatus) {
        self.charging_status = status;
    }

    /// Enable fault simulation for testing error handling
    pub fn set_simulate_faults(&mut self, simulate: bool) {
        self.simulate_faults = simulate;
    }

    /// Add simulated threshold violations for testing
    pub fn add_threshold_violation(&mut self, violation: PowerError) {
        self.threshold_violations.push(violation);
    }

    /// Clear simulated threshold violations
    pub fn clear_threshold_violations(&mut self) {
        self.threshold_violations.clear();
    }

    /// Simulate battery discharge for testing
    pub fn simulate_discharge(&mut self, amount_percent: f32) {
        self.battery_status.capacity_percent = (self.battery_status.capacity_percent - amount_percent).max(0.0);
        self.battery_status.voltage_v = 3.0 + (self.battery_status.capacity_percent / 100.0) * 1.2;
        self.battery_status.timestamp = SystemTime::now();
    }

    /// Simulate battery charging for testing
    pub fn simulate_charge(&mut self, amount_percent: f32) {
        self.battery_status.capacity_percent = (self.battery_status.capacity_percent + amount_percent).min(100.0);
        self.battery_status.voltage_v = 3.0 + (self.battery_status.capacity_percent / 100.0) * 1.2;
        self.battery_status.timestamp = SystemTime::now();
    }

    /// Simulate temperature change for testing
    pub fn simulate_temperature_change(&mut self, temperature_c: f32) {
        self.battery_status.temperature_c = temperature_c;
    }
}

impl PowerManager for MockPowerManager {
    fn get_battery_status(&self) -> Result<BatteryStatus, PowerError> {
        if self.simulate_faults {
            return Err(PowerError::HardwareFault("Simulated battery sensor fault".to_string()));
        }
        Ok(self.battery_status.clone())
    }

    fn get_charging_status(&self) -> Result<ChargingStatus, PowerError> {
        if self.simulate_faults {
            return Err(PowerError::ChargingFault);
        }
        Ok(self.charging_status.clone())
    }

    fn set_power_mode(&mut self, mode: PowerOperationMode) -> Result<(), PowerError> {
        if self.simulate_faults && mode == PowerOperationMode::Emergency {
            return Err(PowerError::HardwareFault("Cannot enter emergency mode".to_string()));
        }
        self.current_mode = mode;
        Ok(())
    }

    fn get_power_mode(&self) -> PowerOperationMode {
        self.current_mode.clone()
    }

    fn estimate_remaining_time(&self) -> Result<Duration, PowerError> {
        if self.simulate_faults {
            return Err(PowerError::HardwareFault("Cannot estimate remaining time".to_string()));
        }

        let current_draw = match self.current_mode {
            PowerOperationMode::Normal => 150.0,
            PowerOperationMode::PowerSave => 50.0,
            PowerOperationMode::Emergency => 25.0,
            PowerOperationMode::Shutdown => 5.0,
        };

        let remaining_capacity_mah = (self.battery_status.capacity_percent / 100.0) * 2000.0;
        let hours_remaining = remaining_capacity_mah / current_draw;
        
        Ok(Duration::from_secs((hours_remaining * 3600.0) as u64))
    }

    fn configure_power_thresholds(&mut self, config: PowerConfig) -> Result<(), PowerError> {
        config.validate()?;
        self.config = config;
        Ok(())
    }

    fn get_power_config(&self) -> &PowerConfig {
        &self.config
    }

    fn check_thresholds(&self) -> Result<Vec<PowerError>, PowerError> {
        if self.simulate_faults {
            return Err(PowerError::HardwareFault("Cannot check thresholds".to_string()));
        }

        // Return simulated violations for testing
        if !self.threshold_violations.is_empty() {
            return Ok(self.threshold_violations.clone());
        }

        // Normal threshold checking logic (same as BasicPowerManager)
        let mut violations = Vec::new();

        if self.battery_status.capacity_percent <= self.config.emergency_battery_threshold_percent {
            violations.push(PowerError::ThresholdViolation {
                threshold_type: "Emergency battery".to_string(),
                value: self.battery_status.capacity_percent,
            });
        } else if self.battery_status.capacity_percent <= self.config.critical_battery_threshold_percent {
            violations.push(PowerError::ThresholdViolation {
                threshold_type: "Critical battery".to_string(),
                value: self.battery_status.capacity_percent,
            });
        } else if self.battery_status.capacity_percent <= self.config.low_battery_threshold_percent {
            violations.push(PowerError::ThresholdViolation {
                threshold_type: "Low battery".to_string(),
                value: self.battery_status.capacity_percent,
            });
        }

        if self.battery_status.temperature_c < self.config.temperature_min_c ||
           self.battery_status.temperature_c > self.config.temperature_max_c {
            violations.push(PowerError::TemperatureExtreme {
                temperature_c: self.battery_status.temperature_c,
            });
        }

        if self.battery_status.voltage_v < self.config.voltage_min_v ||
           self.battery_status.voltage_v > self.config.voltage_max_v {
            violations.push(PowerError::VoltageOutOfRange {
                voltage_v: self.battery_status.voltage_v,
            });
        }

        if self.battery_status.current_ma > self.config.current_max_ma {
            violations.push(PowerError::CurrentOverload {
                current_ma: self.battery_status.current_ma,
            });
        }

        Ok(violations)
    }

    fn set_charging_enabled(&mut self, enabled: bool) -> Result<(), PowerError> {
        if self.simulate_faults && enabled {
            return Err(PowerError::ChargingFault);
        }
        self.charging_enabled = enabled;
        if !enabled {
            self.charging_status = ChargingStatus::NotCharging;
        }
        Ok(())
    }

    fn set_solar_charging_enabled(&mut self, enabled: bool) -> Result<(), PowerError> {
        self.solar_charging_enabled = enabled;
        Ok(())
    }

    fn get_power_stats(&self) -> PowerStats {
        self.stats.clone()
    }

    fn reset_power_stats(&mut self) {
        self.stats = PowerStats::default();
    }

    fn prepare_emergency_shutdown(&mut self) -> Result<(), PowerError> {
        if self.simulate_faults {
            return Err(PowerError::HardwareFault("Emergency shutdown failed".to_string()));
        }
        
        self.set_power_mode(PowerOperationMode::Emergency)?;
        self.set_charging_enabled(false)?;
        Ok(())
    }
}

impl Default for MockPowerManager {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_power_config_validation() {
        let mut config = PowerConfig::default();
        assert!(config.validate().is_ok());

        // Test invalid threshold ordering
        config.low_battery_threshold_percent = 5.0;
        config.critical_battery_threshold_percent = 10.0;
        assert!(config.validate().is_err());

        // Test invalid temperature range
        config = PowerConfig::default();
        config.temperature_min_c = 30.0;
        config.temperature_max_c = 20.0;
        assert!(config.validate().is_err());
    }

    #[test]
    fn test_battery_status_critical_check() {
        let mut status = BatteryStatus::new(3.0, 100.0, 3.0, 25.0);
        assert!(status.is_critical());

        status.capacity_percent = 50.0;
        status.temperature_c = -25.0;
        assert!(status.is_critical());

        status.temperature_c = 25.0;
        status.health = BatteryHealth::Critical;
        assert!(status.is_critical());
    }

    #[test]
    fn test_basic_power_manager_creation() {
        let config = PowerConfig::default();
        let manager = BasicPowerManager::new(config);
        assert!(manager.is_ok());

        let mut invalid_config = PowerConfig::default();
        invalid_config.low_battery_threshold_percent = 5.0;
        let manager = BasicPowerManager::new(invalid_config);
        assert!(manager.is_err());
    }

    #[test]
    fn test_mock_power_manager_simulation() {
        let mut mock = MockPowerManager::new();
        
        // Test battery discharge simulation
        let initial_capacity = mock.battery_status.capacity_percent;
        mock.simulate_discharge(10.0);
        assert!(mock.battery_status.capacity_percent < initial_capacity);

        // Test charging simulation
        let capacity_after_discharge = mock.battery_status.capacity_percent;
        mock.simulate_charge(5.0);
        assert!(mock.battery_status.capacity_percent > capacity_after_discharge);

        // Test fault simulation
        mock.set_simulate_faults(true);
        assert!(mock.get_battery_status().is_err());
    }

    #[test]
    fn test_power_mode_transitions() {
        let mut mock = MockPowerManager::new();
        
        assert_eq!(mock.get_power_mode(), PowerOperationMode::Normal);
        
        assert!(mock.set_power_mode(PowerOperationMode::PowerSave).is_ok());
        assert_eq!(mock.get_power_mode(), PowerOperationMode::PowerSave);
        
        assert!(mock.set_power_mode(PowerOperationMode::Emergency).is_ok());
        assert_eq!(mock.get_power_mode(), PowerOperationMode::Emergency);
    }

    #[test]
    fn test_threshold_violations() {
        let mut mock = MockPowerManager::new();
        
        // Simulate low battery
        mock.simulate_discharge(70.0); // Should trigger low battery threshold
        let violations = mock.check_thresholds().unwrap();
        assert!(!violations.is_empty());

        // Test custom threshold violations
        mock.clear_threshold_violations();
        mock.add_threshold_violation(PowerError::TemperatureExtreme { temperature_c: -30.0 });
        let violations = mock.check_thresholds().unwrap();
        assert_eq!(violations.len(), 1);
    }

    #[test]
    fn test_emergency_shutdown() {
        let mut mock = MockPowerManager::new();
        
        assert!(mock.prepare_emergency_shutdown().is_ok());
        assert_eq!(mock.get_power_mode(), PowerOperationMode::Emergency);
        
        // Test with fault simulation
        mock.set_simulate_faults(true);
        assert!(mock.prepare_emergency_shutdown().is_err());
    }

    #[test]
    fn test_charging_control() {
        let mut mock = MockPowerManager::new();
        
        assert!(mock.set_charging_enabled(false).is_ok());
        assert_eq!(mock.charging_status, ChargingStatus::NotCharging);
        
        assert!(mock.set_solar_charging_enabled(false).is_ok());
        
        // Test with fault simulation
        mock.set_simulate_faults(true);
        assert!(mock.set_charging_enabled(true).is_err());
    }
}