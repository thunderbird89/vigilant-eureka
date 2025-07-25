// Environmental monitoring and adaptation system for beacon reliability
// Implements comprehensive environmental condition monitoring, thermal management,
// and adaptive system behavior based on environmental factors

use std::time::{Duration, SystemTime, Instant};
use std::collections::{HashMap, VecDeque};
use serde::{Serialize, Deserialize};

use crate::{
    EnvironmentalConditions, PowerManager,
    BeaconError, ErrorSeverity
};
use crate::error_handling::EnvironmentalCondition;

/// Environmental monitoring errors
#[derive(Debug, Clone, PartialEq)]
pub enum EnvironmentalError {
    SensorFault { sensor_type: String, details: String },
    ReadingOutOfRange { sensor: String, value: f64, range: (f64, f64) },
    CalibrationRequired { sensor: String, drift_amount: f64 },
    EnvironmentalExtreme { condition: String, severity: ExtremeSeverity },
    AdaptationFailed { reason: String },
    ThermalShutdownRequired { temperature_c: f32 },
}

impl std::fmt::Display for EnvironmentalError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            EnvironmentalError::SensorFault { sensor_type, details } => {
                write!(f, "Sensor fault in {}: {}", sensor_type, details)
            }
            EnvironmentalError::ReadingOutOfRange { sensor, value, range } => {
                write!(f, "Reading out of range for {}: {} (expected: {:.1}-{:.1})", 
                       sensor, value, range.0, range.1)
            }
            EnvironmentalError::CalibrationRequired { sensor, drift_amount } => {
                write!(f, "Calibration required for {}: drift {:.2}", sensor, drift_amount)
            }
            EnvironmentalError::EnvironmentalExtreme { condition, severity } => {
                write!(f, "Environmental extreme ({:?}): {}", severity, condition)
            }
            EnvironmentalError::AdaptationFailed { reason } => {
                write!(f, "Environmental adaptation failed: {}", reason)
            }
            EnvironmentalError::ThermalShutdownRequired { temperature_c } => {
                write!(f, "Thermal shutdown required: temperature {:.1}°C", temperature_c)
            }
        }
    }
}

impl std::error::Error for EnvironmentalError {}

/// Severity levels for environmental extremes
#[derive(Debug, Clone, PartialEq)]
pub enum ExtremeSeverity {
    Warning,    // Approaching limits, monitoring required
    Critical,   // At limits, adaptation required
    Emergency,  // Beyond safe limits, emergency action required
}

/// Extended environmental conditions with monitoring metadata
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ExtendedEnvironmentalConditions {
    pub base_conditions: EnvironmentalConditions,
    pub air_temperature_c: Option<f32>,
    pub humidity_percent: Option<f32>,
    pub atmospheric_pressure_hpa: Option<f32>,
    pub wind_speed_ms: Option<f32>,
    pub solar_irradiance_wm2: Option<f32>,
    pub internal_temperature_c: Option<f32>,
    pub cpu_temperature_c: Option<f32>,
    pub battery_temperature_c: Option<f32>,
    pub enclosure_humidity_percent: Option<f32>,
    pub vibration_level_g: Option<f32>,
    pub magnetic_field_strength_ut: Option<f32>,
    pub timestamp: SystemTime,
    pub measurement_quality: MeasurementQuality,
}

/// Quality assessment of environmental measurements
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MeasurementQuality {
    pub overall_quality: f32,  // 0.0 to 1.0
    pub sensor_health: HashMap<String, f32>,  // Per-sensor health scores
    pub calibration_status: HashMap<String, CalibrationStatus>,
    pub last_calibration: HashMap<String, SystemTime>,
}

/// Calibration status for sensors
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum CalibrationStatus {
    Good,
    DriftDetected { amount: f64 },
    CalibrationRequired,
    CalibrationFailed,
    SensorFault,
}

/// Environmental thresholds for different operational modes
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnvironmentalThresholds {
    // Temperature thresholds (°C)
    pub air_temp_min: f32,
    pub air_temp_max: f32,
    pub water_temp_min: f32,
    pub water_temp_max: f32,
    pub internal_temp_max: f32,
    pub cpu_temp_max: f32,
    pub battery_temp_min: f32,
    pub battery_temp_max: f32,
    
    // Other environmental thresholds
    pub humidity_max: f32,
    pub pressure_min: f32,
    pub pressure_max: f32,
    pub wind_speed_max: f32,
    pub vibration_max: f32,
    pub noise_level_max: f32,
    
    // Adaptation trigger thresholds
    pub power_adaptation_temp_threshold: f32,
    pub transmission_adaptation_noise_threshold: f32,
    pub emergency_shutdown_temp_threshold: f32,
}

impl Default for EnvironmentalThresholds {
    fn default() -> Self {
        Self {
            // Temperature thresholds based on marine equipment standards
            air_temp_min: -20.0,
            air_temp_max: 60.0,
            water_temp_min: -2.0,
            water_temp_max: 40.0,
            internal_temp_max: 70.0,
            cpu_temp_max: 85.0,
            battery_temp_min: -10.0,
            battery_temp_max: 50.0,
            
            // Environmental limits
            humidity_max: 95.0,
            pressure_min: 950.0,
            pressure_max: 1050.0,
            wind_speed_max: 25.0,  // m/s (about 50 knots)
            vibration_max: 2.0,    // g
            noise_level_max: 100.0, // dB
            
            // Adaptation thresholds
            power_adaptation_temp_threshold: 40.0,
            transmission_adaptation_noise_threshold: 80.0,
            emergency_shutdown_temp_threshold: 75.0,
        }
    }
}

/// Environmental adaptation actions
#[derive(Debug, Clone)]
pub enum AdaptationAction {
    ReduceTransmissionPower { from: u8, to: u8 },
    IncreaseTransmissionPower { from: u8, to: u8 },
    ReduceTransmissionFrequency { from_ms: u32, to_ms: u32 },
    EnablePowerSaveMode,
    DisableNonEssentialSystems,
    ActivateThermalManagement,
    RequestEmergencyShutdown { reason: String },
    AdjustSensorSampling { sensor: String, from_ms: u32, to_ms: u32 },
}

/// Environmental monitoring statistics
#[derive(Debug, Clone, Default)]
pub struct EnvironmentalStats {
    pub total_measurements: u64,
    pub adaptation_actions: u64,
    pub threshold_violations: u64,
    pub sensor_faults: u32,
    pub calibration_events: u32,
    pub extreme_conditions_detected: u32,
    pub thermal_events: u32,
    pub measurement_history_size: usize,
    pub average_measurement_quality: f32,
    pub uptime_in_extreme_conditions: Duration,
}

/// Main environmental monitoring system
pub struct EnvironmentalMonitor {
    config: EnvironmentalThresholds,
    current_conditions: ExtendedEnvironmentalConditions,
    measurement_history: VecDeque<ExtendedEnvironmentalConditions>,
    adaptation_history: VecDeque<(SystemTime, AdaptationAction)>,
    statistics: EnvironmentalStats,
    sensor_calibration_data: HashMap<String, SensorCalibration>,
    last_measurement_time: Option<SystemTime>,
    measurement_interval: Duration,
    adaptation_enabled: bool,
    thermal_management_active: bool,
    extreme_condition_start: Option<SystemTime>,
}

/// Sensor calibration data
#[derive(Debug, Clone)]
struct SensorCalibration {
    offset: f64,
    scale: f64,
    last_calibration: SystemTime,
    drift_rate: f64,  // units per day
    reference_readings: VecDeque<(SystemTime, f64)>,
}

impl EnvironmentalMonitor {
    /// Create new environmental monitor
    pub fn new(config: EnvironmentalThresholds) -> Self {
        Self {
            config,
            current_conditions: ExtendedEnvironmentalConditions {
                base_conditions: EnvironmentalConditions::default(),
                air_temperature_c: None,
                humidity_percent: None,
                atmospheric_pressure_hpa: None,
                wind_speed_ms: None,
                solar_irradiance_wm2: None,
                internal_temperature_c: None,
                cpu_temperature_c: None,
                battery_temperature_c: None,
                enclosure_humidity_percent: None,
                vibration_level_g: None,
                magnetic_field_strength_ut: None,
                timestamp: SystemTime::now(),
                measurement_quality: MeasurementQuality {
                    overall_quality: 1.0,
                    sensor_health: HashMap::new(),
                    calibration_status: HashMap::new(),
                    last_calibration: HashMap::new(),
                },
            },
            measurement_history: VecDeque::with_capacity(1000),
            adaptation_history: VecDeque::with_capacity(100),
            statistics: EnvironmentalStats::default(),
            sensor_calibration_data: HashMap::new(),
            last_measurement_time: None,
            measurement_interval: Duration::from_secs(30), // 30 second intervals
            adaptation_enabled: true,
            thermal_management_active: false,
            extreme_condition_start: None,
        }
    }

    /// Update environmental conditions with new sensor readings
    pub fn update_conditions(&mut self, conditions: ExtendedEnvironmentalConditions) -> Result<Vec<AdaptationAction>, EnvironmentalError> {
        let now = SystemTime::now();
        let mut actions = Vec::new();

        // Validate sensor readings
        self.validate_sensor_readings(&conditions)?;

        // Apply sensor calibration
        let calibrated_conditions = self.apply_calibration(conditions)?;

        // Check for threshold violations
        let violations = self.check_thresholds(&calibrated_conditions);
        
        // Update statistics
        self.statistics.total_measurements += 1;
        self.statistics.threshold_violations += violations.len() as u64;

        // Store measurement in history
        self.measurement_history.push_back(calibrated_conditions.clone());
        if self.measurement_history.len() > 1000 {
            self.measurement_history.pop_front();
        }

        // Update current conditions
        self.current_conditions = calibrated_conditions;
        self.last_measurement_time = Some(now);

        // Process threshold violations and generate adaptation actions
        for violation in violations {
            if let Some(action) = self.process_threshold_violation(violation)? {
                actions.push(action);
            }
        }

        // Update extreme condition tracking
        self.update_extreme_condition_tracking(&actions);

        // Record adaptation actions
        for action in &actions {
            self.adaptation_history.push_back((now, action.clone()));
            if self.adaptation_history.len() > 100 {
                self.adaptation_history.pop_front();
            }
        }

        self.statistics.adaptation_actions += actions.len() as u64;

        Ok(actions)
    }

    /// Validate sensor readings for plausibility
    fn validate_sensor_readings(&self, conditions: &ExtendedEnvironmentalConditions) -> Result<(), EnvironmentalError> {
        // Temperature validation
        if let Some(temp) = conditions.air_temperature_c {
            if temp < -50.0 || temp > 80.0 {
                return Err(EnvironmentalError::ReadingOutOfRange {
                    sensor: "air_temperature".to_string(),
                    value: temp as f64,
                    range: (-50.0, 80.0),
                });
            }
        }

        if let Some(temp) = conditions.internal_temperature_c {
            if temp < -20.0 || temp > 100.0 {
                return Err(EnvironmentalError::ReadingOutOfRange {
                    sensor: "internal_temperature".to_string(),
                    value: temp as f64,
                    range: (-20.0, 100.0),
                });
            }
        }

        if let Some(temp) = conditions.cpu_temperature_c {
            if temp < 0.0 || temp > 120.0 {
                return Err(EnvironmentalError::ReadingOutOfRange {
                    sensor: "cpu_temperature".to_string(),
                    value: temp as f64,
                    range: (0.0, 120.0),
                });
            }
        }

        // Humidity validation
        if let Some(humidity) = conditions.humidity_percent {
            if humidity < 0.0 || humidity > 100.0 {
                return Err(EnvironmentalError::ReadingOutOfRange {
                    sensor: "humidity".to_string(),
                    value: humidity as f64,
                    range: (0.0, 100.0),
                });
            }
        }

        // Pressure validation
        if let Some(pressure) = conditions.atmospheric_pressure_hpa {
            if pressure < 800.0 || pressure > 1200.0 {
                return Err(EnvironmentalError::ReadingOutOfRange {
                    sensor: "atmospheric_pressure".to_string(),
                    value: pressure as f64,
                    range: (800.0, 1200.0),
                });
            }
        }

        Ok(())
    }

    /// Apply sensor calibration to raw readings
    fn apply_calibration(&self, mut conditions: ExtendedEnvironmentalConditions) -> Result<ExtendedEnvironmentalConditions, EnvironmentalError> {
        // Apply temperature sensor calibrations
        if let Some(temp) = conditions.air_temperature_c {
            if let Some(cal) = self.sensor_calibration_data.get("air_temperature") {
                conditions.air_temperature_c = Some((temp as f64 * cal.scale + cal.offset) as f32);
            }
        }

        if let Some(temp) = conditions.internal_temperature_c {
            if let Some(cal) = self.sensor_calibration_data.get("internal_temperature") {
                conditions.internal_temperature_c = Some((temp as f64 * cal.scale + cal.offset) as f32);
            }
        }

        // Apply other sensor calibrations as needed
        // ... (similar pattern for other sensors)

        Ok(conditions)
    }

    /// Check environmental thresholds and return violations
    fn check_thresholds(&self, conditions: &ExtendedEnvironmentalConditions) -> Vec<ThresholdViolation> {
        let mut violations = Vec::new();

        // Temperature threshold checks
        if let Some(temp) = conditions.air_temperature_c {
            if temp < self.config.air_temp_min {
                violations.push(ThresholdViolation {
                    parameter: "air_temperature".to_string(),
                    value: temp as f64,
                    threshold: self.config.air_temp_min as f64,
                    violation_type: ViolationType::BelowMinimum,
                    severity: if temp < self.config.air_temp_min - 10.0 { ExtremeSeverity::Emergency } else { ExtremeSeverity::Warning },
                });
            } else if temp > self.config.air_temp_max {
                violations.push(ThresholdViolation {
                    parameter: "air_temperature".to_string(),
                    value: temp as f64,
                    threshold: self.config.air_temp_max as f64,
                    violation_type: ViolationType::AboveMaximum,
                    severity: if temp > self.config.air_temp_max + 10.0 { ExtremeSeverity::Emergency } else { ExtremeSeverity::Warning },
                });
            }
        }

        if let Some(temp) = conditions.internal_temperature_c {
            if temp > self.config.internal_temp_max {
                violations.push(ThresholdViolation {
                    parameter: "internal_temperature".to_string(),
                    value: temp as f64,
                    threshold: self.config.internal_temp_max as f64,
                    violation_type: ViolationType::AboveMaximum,
                    severity: if temp > self.config.emergency_shutdown_temp_threshold { ExtremeSeverity::Emergency } else { ExtremeSeverity::Critical },
                });
            }
        }

        if let Some(temp) = conditions.cpu_temperature_c {
            if temp > self.config.cpu_temp_max {
                violations.push(ThresholdViolation {
                    parameter: "cpu_temperature".to_string(),
                    value: temp as f64,
                    threshold: self.config.cpu_temp_max as f64,
                    violation_type: ViolationType::AboveMaximum,
                    severity: ExtremeSeverity::Emergency,
                });
            }
        }

        if let Some(temp) = conditions.battery_temperature_c {
            if temp < self.config.battery_temp_min || temp > self.config.battery_temp_max {
                violations.push(ThresholdViolation {
                    parameter: "battery_temperature".to_string(),
                    value: temp as f64,
                    threshold: if temp < self.config.battery_temp_min { self.config.battery_temp_min as f64 } else { self.config.battery_temp_max as f64 },
                    violation_type: if temp < self.config.battery_temp_min { ViolationType::BelowMinimum } else { ViolationType::AboveMaximum },
                    severity: ExtremeSeverity::Critical,
                });
            }
        }

        // Other environmental checks
        if let Some(humidity) = conditions.humidity_percent {
            if humidity > self.config.humidity_max {
                violations.push(ThresholdViolation {
                    parameter: "humidity".to_string(),
                    value: humidity as f64,
                    threshold: self.config.humidity_max as f64,
                    violation_type: ViolationType::AboveMaximum,
                    severity: ExtremeSeverity::Warning,
                });
            }
        }

        if let Some(vibration) = conditions.vibration_level_g {
            if vibration > self.config.vibration_max {
                violations.push(ThresholdViolation {
                    parameter: "vibration".to_string(),
                    value: vibration as f64,
                    threshold: self.config.vibration_max as f64,
                    violation_type: ViolationType::AboveMaximum,
                    severity: ExtremeSeverity::Critical,
                });
            }
        }

        violations
    }

    /// Process threshold violation and determine adaptation action
    fn process_threshold_violation(&mut self, violation: ThresholdViolation) -> Result<Option<AdaptationAction>, EnvironmentalError> {
        if !self.adaptation_enabled {
            return Ok(None);
        }

        match violation.parameter.as_str() {
            "internal_temperature" | "cpu_temperature" => {
                match violation.severity {
                    ExtremeSeverity::Emergency => {
                        self.thermal_management_active = true;
                        self.statistics.thermal_events += 1;
                        Ok(Some(AdaptationAction::RequestEmergencyShutdown {
                            reason: format!("Critical temperature: {:.1}°C", violation.value)
                        }))
                    }
                    ExtremeSeverity::Critical => {
                        self.thermal_management_active = true;
                        Ok(Some(AdaptationAction::ActivateThermalManagement))
                    }
                    ExtremeSeverity::Warning => {
                        Ok(Some(AdaptationAction::EnablePowerSaveMode))
                    }
                }
            }
            "battery_temperature" => {
                Ok(Some(AdaptationAction::EnablePowerSaveMode))
            }
            "vibration" => {
                Ok(Some(AdaptationAction::AdjustSensorSampling {
                    sensor: "vibration".to_string(),
                    from_ms: 30000,
                    to_ms: 10000, // Increase sampling frequency during high vibration
                }))
            }
            _ => Ok(None)
        }
    }

    /// Update extreme condition tracking
    fn update_extreme_condition_tracking(&mut self, actions: &[AdaptationAction]) {
        let has_extreme_conditions = actions.iter().any(|action| {
            matches!(action, 
                AdaptationAction::RequestEmergencyShutdown { .. } |
                AdaptationAction::ActivateThermalManagement |
                AdaptationAction::DisableNonEssentialSystems
            )
        });

        if has_extreme_conditions && self.extreme_condition_start.is_none() {
            self.extreme_condition_start = Some(SystemTime::now());
            self.statistics.extreme_conditions_detected += 1;
        } else if !has_extreme_conditions && self.extreme_condition_start.is_some() {
            if let Some(start_time) = self.extreme_condition_start.take() {
                if let Ok(duration) = SystemTime::now().duration_since(start_time) {
                    self.statistics.uptime_in_extreme_conditions += duration;
                }
            }
        }
    }

    /// Get current environmental conditions
    pub fn get_current_conditions(&self) -> &ExtendedEnvironmentalConditions {
        &self.current_conditions
    }

    /// Get environmental statistics
    pub fn get_statistics(&self) -> &EnvironmentalStats {
        &self.statistics
    }

    /// Get recent adaptation history
    pub fn get_adaptation_history(&self, limit: usize) -> Vec<(SystemTime, AdaptationAction)> {
        self.adaptation_history.iter()
            .rev()
            .take(limit)
            .cloned()
            .collect()
    }

    /// Enable or disable environmental adaptation
    pub fn set_adaptation_enabled(&mut self, enabled: bool) {
        self.adaptation_enabled = enabled;
    }

    /// Check if thermal management is currently active
    pub fn is_thermal_management_active(&self) -> bool {
        self.thermal_management_active
    }

    /// Reset thermal management state
    pub fn reset_thermal_management(&mut self) {
        self.thermal_management_active = false;
    }

    /// Update sensor calibration
    pub fn update_sensor_calibration(&mut self, sensor: String, offset: f64, scale: f64) {
        let calibration = SensorCalibration {
            offset,
            scale,
            last_calibration: SystemTime::now(),
            drift_rate: 0.0,
            reference_readings: VecDeque::with_capacity(100),
        };
        
        self.sensor_calibration_data.insert(sensor, calibration);
        self.statistics.calibration_events += 1;
    }
}

/// Threshold violation information
#[derive(Debug, Clone)]
struct ThresholdViolation {
    parameter: String,
    value: f64,
    threshold: f64,
    violation_type: ViolationType,
    severity: ExtremeSeverity,
}

/// Types of threshold violations
#[derive(Debug, Clone, PartialEq)]
enum ViolationType {
    AboveMaximum,
    BelowMinimum,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_environmental_monitor_creation() {
        let config = EnvironmentalThresholds::default();
        let monitor = EnvironmentalMonitor::new(config);
        
        assert!(!monitor.is_thermal_management_active());
        assert_eq!(monitor.get_statistics().total_measurements, 0);
    }

    #[test]
    fn test_temperature_threshold_violation() {
        let config = EnvironmentalThresholds::default();
        let mut monitor = EnvironmentalMonitor::new(config);
        
        let mut conditions = ExtendedEnvironmentalConditions {
            base_conditions: EnvironmentalConditions::default(),
            air_temperature_c: Some(70.0), // Above max threshold
            internal_temperature_c: Some(80.0), // Above max threshold
            cpu_temperature_c: None,
            battery_temperature_c: None,
            humidity_percent: None,
            atmospheric_pressure_hpa: None,
            wind_speed_ms: None,
            solar_irradiance_wm2: None,
            enclosure_humidity_percent: None,
            vibration_level_g: None,
            magnetic_field_strength_ut: None,
            timestamp: SystemTime::now(),
            measurement_quality: MeasurementQuality {
                overall_quality: 1.0,
                sensor_health: HashMap::new(),
                calibration_status: HashMap::new(),
                last_calibration: HashMap::new(),
            },
        };
        
        let actions = monitor.update_conditions(conditions).unwrap();
        assert!(!actions.is_empty());
        assert!(monitor.is_thermal_management_active());
    }

    #[test]
    fn test_sensor_validation() {
        let config = EnvironmentalThresholds::default();
        let monitor = EnvironmentalMonitor::new(config);
        
        let conditions = ExtendedEnvironmentalConditions {
            base_conditions: EnvironmentalConditions::default(),
            air_temperature_c: Some(150.0), // Invalid temperature
            internal_temperature_c: None,
            cpu_temperature_c: None,
            battery_temperature_c: None,
            humidity_percent: None,
            atmospheric_pressure_hpa: None,
            wind_speed_ms: None,
            solar_irradiance_wm2: None,
            enclosure_humidity_percent: None,
            vibration_level_g: None,
            magnetic_field_strength_ut: None,
            timestamp: SystemTime::now(),
            measurement_quality: MeasurementQuality {
                overall_quality: 1.0,
                sensor_health: HashMap::new(),
                calibration_status: HashMap::new(),
                last_calibration: HashMap::new(),
            },
        };
        
        assert!(monitor.validate_sensor_readings(&conditions).is_err());
    }
}