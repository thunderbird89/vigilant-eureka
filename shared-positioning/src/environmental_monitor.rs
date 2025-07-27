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
    // Enhanced environmental parameters for rough sea conditions
    pub wave_height_m: Option<f32>,
    pub wave_period_s: Option<f32>,
    pub sea_state: Option<SeaState>,
    pub tilt_angle_degrees: Option<f32>,
    pub acceleration_g: Option<f32>,
    // Enhanced thermal monitoring
    pub thermal_gradient_c_per_m: Option<f32>,
    pub heat_dissipation_rate_w: Option<f32>,
    pub cooling_efficiency_percent: Option<f32>,
    pub timestamp: SystemTime,
    pub measurement_quality: MeasurementQuality,
}

/// Sea state classification for rough sea condition detection
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum SeaState {
    Calm = 0,        // 0-0.1m waves
    Smooth = 1,      // 0.1-0.5m waves
    Slight = 2,      // 0.5-1.25m waves
    Moderate = 3,    // 1.25-2.5m waves
    Rough = 4,       // 2.5-4m waves
    VeryRough = 5,   // 4-6m waves
    High = 6,        // 6-9m waves
    VeryHigh = 7,    // 9-14m waves
    Phenomenal = 8,  // >14m waves
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
    
    // Enhanced thresholds for rough sea conditions
    pub rough_sea_wave_height_threshold: f32,
    pub rough_sea_tilt_threshold: f32,
    pub rough_sea_acceleration_threshold: f32,
    pub stabilization_activation_threshold: f32,
    
    // Enhanced thermal management thresholds
    pub thermal_throttling_threshold: f32,
    pub cooling_activation_threshold: f32,
    pub cpu_frequency_reduction_threshold: f32,
    pub thermal_gradient_threshold: f32,
    
    // Environmental data logging thresholds
    pub logging_frequency_increase_threshold: f32,
    pub environmental_snapshot_threshold: f32,
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
            
            // Enhanced thresholds for rough sea conditions
            rough_sea_wave_height_threshold: 2.5,  // Moderate sea state (Sea State 3)
            rough_sea_tilt_threshold: 15.0,        // degrees
            rough_sea_acceleration_threshold: 1.5, // g
            stabilization_activation_threshold: 10.0, // degrees
            
            // Enhanced thermal management thresholds
            thermal_throttling_threshold: 65.0,    // °C
            cooling_activation_threshold: 60.0,    // °C
            cpu_frequency_reduction_threshold: 70.0, // °C
            thermal_gradient_threshold: 5.0,       // °C/m
            
            // Environmental data logging thresholds
            logging_frequency_increase_threshold: 50.0, // Generic threshold for increased logging
            environmental_snapshot_threshold: 75.0,     // Trigger snapshot at 75% of critical thresholds
        }
    }
}

/// Environmental adaptation actions
#[derive(Debug, Clone)]
pub enum AdaptationAction {
    ReduceTransmissionPower { from: u8, to: u8 },
    IncreaseTransmissionPower { from: u8, to: u8 },
    ReduceTransmissionFrequency { from_ms: u32, to_ms: u32 },
    IncreaseTransmissionFrequency { from_ms: u32, to_ms: u32 },
    EnablePowerSaveMode,
    DisableNonEssentialSystems,
    ActivateThermalManagement,
    RequestEmergencyShutdown { reason: String },
    AdjustSensorSampling { sensor: String, from_ms: u32, to_ms: u32 },
    // Enhanced adaptation actions for rough sea conditions
    EnableRoughSeaMode { sea_state: SeaState },
    AdjustTransmissionTiming { delay_ms: u32, reason: String },
    ActivateStabilization { target_angle_degrees: f32 },
    // Enhanced thermal management actions
    ReduceCpuFrequency { from_mhz: u32, to_mhz: u32 },
    ActivateCooling { cooling_level: u8 },
    ThrottleOperations { throttle_percent: u8 },
    // Environmental data logging actions
    IncreaseLoggingFrequency { sensor: String, new_interval_ms: u32 },
    TriggerEnvironmentalSnapshot { reason: String },
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
    // Enhanced statistics for rough sea conditions
    pub rough_sea_events: u32,
    pub stabilization_activations: u32,
    pub transmission_timing_adjustments: u32,
    pub max_wave_height_m: f32,
    pub max_tilt_angle_degrees: f32,
    // Enhanced thermal statistics
    pub thermal_throttling_events: u32,
    pub cooling_activations: u32,
    pub cpu_frequency_reductions: u32,
    pub max_internal_temperature_c: f32,
    pub thermal_gradient_events: u32,
    // Environmental data logging statistics
    pub environmental_snapshots: u32,
    pub logging_frequency_adjustments: u32,
    pub data_logging_errors: u32,
    pub trend_analysis_runs: u32,
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
    // Enhanced monitoring state
    rough_sea_mode_active: bool,
    current_sea_state: SeaState,
    stabilization_active: bool,
    cooling_system_active: bool,
    cpu_throttling_active: bool,
    environmental_data_log: VecDeque<EnvironmentalDataPoint>,
    trend_analysis_cache: HashMap<String, TrendData>,
    last_trend_analysis: Option<SystemTime>,
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

/// Environmental data point for logging and trend analysis
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnvironmentalDataPoint {
    pub timestamp: SystemTime,
    pub parameter_name: String,
    pub value: f64,
    pub unit: String,
    pub quality_score: f32,
    pub trend_direction: TrendDirection,
    pub rate_of_change: f64,
}

/// Trend direction for environmental parameters
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum TrendDirection {
    Increasing,
    Decreasing,
    Stable,
    Oscillating,
    Unknown,
}

/// Trend analysis data for environmental parameters
#[derive(Debug, Clone)]
struct TrendData {
    parameter_name: String,
    recent_values: VecDeque<(SystemTime, f64)>,
    trend_direction: TrendDirection,
    rate_of_change: f64,
    variance: f64,
    last_analysis: SystemTime,
    prediction_accuracy: f32,
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
                wave_height_m: None,
                wave_period_s: None,
                sea_state: None,
                tilt_angle_degrees: None,
                acceleration_g: None,
                thermal_gradient_c_per_m: None,
                heat_dissipation_rate_w: None,
                cooling_efficiency_percent: None,
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
            // Initialize enhanced monitoring state
            rough_sea_mode_active: false,
            current_sea_state: SeaState::Calm,
            stabilization_active: false,
            cooling_system_active: false,
            cpu_throttling_active: false,
            environmental_data_log: VecDeque::with_capacity(10000),
            trend_analysis_cache: HashMap::new(),
            last_trend_analysis: None,
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

        // Enhanced threshold checks for rough sea conditions
        if let Some(wave_height) = conditions.wave_height_m {
            if wave_height > self.config.rough_sea_wave_height_threshold {
                violations.push(ThresholdViolation {
                    parameter: "wave_height".to_string(),
                    value: wave_height as f64,
                    threshold: self.config.rough_sea_wave_height_threshold as f64,
                    violation_type: ViolationType::AboveMaximum,
                    severity: if wave_height > 6.0 { ExtremeSeverity::Emergency } else { ExtremeSeverity::Warning },
                });
            }
        }

        if let Some(tilt_angle) = conditions.tilt_angle_degrees {
            if tilt_angle.abs() > self.config.rough_sea_tilt_threshold {
                violations.push(ThresholdViolation {
                    parameter: "tilt_angle".to_string(),
                    value: tilt_angle.abs() as f64,
                    threshold: self.config.rough_sea_tilt_threshold as f64,
                    violation_type: ViolationType::AboveMaximum,
                    severity: if tilt_angle.abs() > 30.0 { ExtremeSeverity::Critical } else { ExtremeSeverity::Warning },
                });
            }
        }

        if let Some(acceleration) = conditions.acceleration_g {
            if acceleration > self.config.rough_sea_acceleration_threshold {
                violations.push(ThresholdViolation {
                    parameter: "acceleration".to_string(),
                    value: acceleration as f64,
                    threshold: self.config.rough_sea_acceleration_threshold as f64,
                    violation_type: ViolationType::AboveMaximum,
                    severity: if acceleration > 3.0 { ExtremeSeverity::Critical } else { ExtremeSeverity::Warning },
                });
            }
        }

        // Enhanced thermal threshold checks
        if let Some(thermal_gradient) = conditions.thermal_gradient_c_per_m {
            if thermal_gradient.abs() > self.config.thermal_gradient_threshold {
                violations.push(ThresholdViolation {
                    parameter: "thermal_gradient".to_string(),
                    value: thermal_gradient.abs() as f64,
                    threshold: self.config.thermal_gradient_threshold as f64,
                    violation_type: ViolationType::AboveMaximum,
                    severity: ExtremeSeverity::Warning,
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
                        self.cpu_throttling_active = true;
                        self.statistics.cpu_frequency_reductions += 1;
                        Ok(Some(AdaptationAction::RequestEmergencyShutdown {
                            reason: format!("Critical temperature: {:.1}°C", violation.value)
                        }))
                    }
                    ExtremeSeverity::Critical => {
                        self.thermal_management_active = true;
                        self.cooling_system_active = true;
                        self.statistics.cooling_activations += 1;
                        
                        // Update max temperature tracking
                        if violation.value > self.statistics.max_internal_temperature_c as f64 {
                            self.statistics.max_internal_temperature_c = violation.value as f32;
                        }
                        
                        Ok(Some(AdaptationAction::ActivateCooling { cooling_level: 255 }))
                    }
                    ExtremeSeverity::Warning => {
                        if violation.value > self.config.thermal_throttling_threshold as f64 {
                            self.statistics.thermal_throttling_events += 1;
                            Ok(Some(AdaptationAction::ThrottleOperations { throttle_percent: 25 }))
                        } else {
                            Ok(Some(AdaptationAction::EnablePowerSaveMode))
                        }
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
            // Enhanced adaptation for rough sea conditions
            "wave_height" => {
                let sea_state = self.classify_sea_state(violation.value as f32);
                self.current_sea_state = sea_state.clone();
                
                match violation.severity {
                    ExtremeSeverity::Emergency => {
                        self.rough_sea_mode_active = true;
                        self.statistics.rough_sea_events += 1;
                        Ok(Some(AdaptationAction::EnableRoughSeaMode { sea_state }))
                    }
                    ExtremeSeverity::Critical | ExtremeSeverity::Warning => {
                        self.statistics.transmission_timing_adjustments += 1;
                        Ok(Some(AdaptationAction::AdjustTransmissionTiming {
                            delay_ms: (violation.value * 100.0) as u32, // Delay based on wave height
                            reason: format!("Rough sea conditions: {:.1}m waves", violation.value)
                        }))
                    }
                }
            }
            "tilt_angle" => {
                if violation.value > self.config.stabilization_activation_threshold as f64 {
                    self.stabilization_active = true;
                    self.statistics.stabilization_activations += 1;
                    
                    // Update max tilt tracking
                    if violation.value > self.statistics.max_tilt_angle_degrees as f64 {
                        self.statistics.max_tilt_angle_degrees = violation.value as f32;
                    }
                    
                    Ok(Some(AdaptationAction::ActivateStabilization {
                        target_angle_degrees: 0.0
                    }))
                } else {
                    Ok(Some(AdaptationAction::AdjustTransmissionTiming {
                        delay_ms: (violation.value * 50.0) as u32,
                        reason: format!("Platform tilt: {:.1}°", violation.value)
                    }))
                }
            }
            "acceleration" => {
                self.statistics.transmission_timing_adjustments += 1;
                Ok(Some(AdaptationAction::AdjustTransmissionTiming {
                    delay_ms: (violation.value * 200.0) as u32,
                    reason: format!("High acceleration: {:.1}g", violation.value)
                }))
            }
            "thermal_gradient" => {
                self.statistics.thermal_gradient_events += 1;
                Ok(Some(AdaptationAction::IncreaseLoggingFrequency {
                    sensor: "thermal_gradient".to_string(),
                    new_interval_ms: 5000, // Increase thermal monitoring frequency
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

    /// Classify sea state based on wave height
    fn classify_sea_state(&self, wave_height_m: f32) -> SeaState {
        match wave_height_m {
            h if h <= 0.1 => SeaState::Calm,
            h if h <= 0.5 => SeaState::Smooth,
            h if h <= 1.25 => SeaState::Slight,
            h if h <= 2.5 => SeaState::Moderate,
            h if h <= 4.0 => SeaState::Rough,
            h if h <= 6.0 => SeaState::VeryRough,
            h if h <= 9.0 => SeaState::High,
            h if h <= 14.0 => SeaState::VeryHigh,
            _ => SeaState::Phenomenal,
        }
    }

    /// Log environmental data point for trend analysis
    pub fn log_environmental_data(&mut self, parameter_name: String, value: f64, unit: String) {
        let now = SystemTime::now();
        
        // Calculate trend direction and rate of change
        let (trend_direction, rate_of_change) = self.calculate_trend(&parameter_name, value, now);
        
        let data_point = EnvironmentalDataPoint {
            timestamp: now,
            parameter_name: parameter_name.clone(),
            value,
            unit,
            quality_score: self.current_conditions.measurement_quality.overall_quality,
            trend_direction,
            rate_of_change,
        };
        
        self.environmental_data_log.push_back(data_point);
        
        // Maintain log size limit
        if self.environmental_data_log.len() > 10000 {
            self.environmental_data_log.pop_front();
        }
        
        // Update trend analysis cache
        self.update_trend_cache(parameter_name, value, now);
    }

    /// Calculate trend direction and rate of change for a parameter
    fn calculate_trend(&self, parameter_name: &str, current_value: f64, timestamp: SystemTime) -> (TrendDirection, f64) {
        if let Some(trend_data) = self.trend_analysis_cache.get(parameter_name) {
            if let Some((last_time, last_value)) = trend_data.recent_values.back() {
                if let Ok(time_diff) = timestamp.duration_since(*last_time) {
                    let time_diff_secs = time_diff.as_secs_f64();
                    if time_diff_secs > 0.0 {
                        let rate_of_change = (current_value - last_value) / time_diff_secs;
                        let trend_direction = if rate_of_change.abs() < 0.001 {
                            TrendDirection::Stable
                        } else if rate_of_change > 0.0 {
                            TrendDirection::Increasing
                        } else {
                            TrendDirection::Decreasing
                        };
                        return (trend_direction, rate_of_change);
                    }
                }
            }
        }
        (TrendDirection::Unknown, 0.0)
    }

    /// Update trend analysis cache
    fn update_trend_cache(&mut self, parameter_name: String, value: f64, timestamp: SystemTime) {
        let trend_data = self.trend_analysis_cache.entry(parameter_name.clone()).or_insert_with(|| {
            TrendData {
                parameter_name: parameter_name.clone(),
                recent_values: VecDeque::with_capacity(100),
                trend_direction: TrendDirection::Unknown,
                rate_of_change: 0.0,
                variance: 0.0,
                last_analysis: timestamp,
                prediction_accuracy: 0.0,
            }
        });
        
        trend_data.recent_values.push_back((timestamp, value));
        if trend_data.recent_values.len() > 100 {
            trend_data.recent_values.pop_front();
        }
        
        // Update trend analysis if enough data points
        if trend_data.recent_values.len() >= 5 {
            self.analyze_trend(parameter_name);
        }
    }

    /// Perform trend analysis for a parameter
    fn analyze_trend(&mut self, parameter_name: String) {
        if let Some(trend_data) = self.trend_analysis_cache.get_mut(&parameter_name) {
            let values: Vec<f64> = trend_data.recent_values.iter().map(|(_, v)| *v).collect();
            
            // Calculate variance
            let mean = values.iter().sum::<f64>() / values.len() as f64;
            let variance = values.iter().map(|v| (v - mean).powi(2)).sum::<f64>() / values.len() as f64;
            trend_data.variance = variance;
            
            // Calculate trend direction based on linear regression
            let n = values.len() as f64;
            let x_values: Vec<f64> = (0..values.len()).map(|i| i as f64).collect();
            let x_mean = x_values.iter().sum::<f64>() / n;
            let y_mean = mean;
            
            let numerator: f64 = x_values.iter().zip(values.iter())
                .map(|(x, y)| (x - x_mean) * (y - y_mean))
                .sum();
            let denominator: f64 = x_values.iter()
                .map(|x| (x - x_mean).powi(2))
                .sum();
            
            if denominator != 0.0 {
                let slope = numerator / denominator;
                trend_data.rate_of_change = slope;
                
                trend_data.trend_direction = if slope.abs() < variance.sqrt() * 0.1 {
                    TrendDirection::Stable
                } else if slope > 0.0 {
                    TrendDirection::Increasing
                } else {
                    TrendDirection::Decreasing
                };
            }
            
            trend_data.last_analysis = SystemTime::now();
        }
        
        self.statistics.trend_analysis_runs += 1;
    }

    /// Run comprehensive trend analysis on all parameters
    pub fn run_trend_analysis(&mut self) -> Result<HashMap<String, TrendDirection>, EnvironmentalError> {
        let mut results = HashMap::new();
        
        for parameter_name in self.trend_analysis_cache.keys().cloned().collect::<Vec<_>>() {
            self.analyze_trend(parameter_name.clone());
            if let Some(trend_data) = self.trend_analysis_cache.get(&parameter_name) {
                results.insert(parameter_name, trend_data.trend_direction.clone());
            }
        }
        
        self.last_trend_analysis = Some(SystemTime::now());
        Ok(results)
    }

    /// Trigger environmental snapshot for detailed logging
    pub fn trigger_environmental_snapshot(&mut self, reason: String) -> Result<(), EnvironmentalError> {
        self.statistics.environmental_snapshots += 1;
        
        // Log all current environmental parameters (clone to avoid borrowing issues)
        let conditions = self.current_conditions.clone();
        
        if let Some(temp) = conditions.air_temperature_c {
            self.log_environmental_data("air_temperature".to_string(), temp as f64, "°C".to_string());
        }
        if let Some(humidity) = conditions.humidity_percent {
            self.log_environmental_data("humidity".to_string(), humidity as f64, "%".to_string());
        }
        if let Some(pressure) = conditions.atmospheric_pressure_hpa {
            self.log_environmental_data("atmospheric_pressure".to_string(), pressure as f64, "hPa".to_string());
        }
        if let Some(wave_height) = conditions.wave_height_m {
            self.log_environmental_data("wave_height".to_string(), wave_height as f64, "m".to_string());
        }
        if let Some(tilt) = conditions.tilt_angle_degrees {
            self.log_environmental_data("tilt_angle".to_string(), tilt as f64, "°".to_string());
        }
        if let Some(accel) = conditions.acceleration_g {
            self.log_environmental_data("acceleration".to_string(), accel as f64, "g".to_string());
        }
        if let Some(thermal_grad) = conditions.thermal_gradient_c_per_m {
            self.log_environmental_data("thermal_gradient".to_string(), thermal_grad as f64, "°C/m".to_string());
        }
        
        // Create snapshot action for processing
        let snapshot_action = AdaptationAction::TriggerEnvironmentalSnapshot { reason };
        self.adaptation_history.push_back((SystemTime::now(), snapshot_action));
        
        Ok(())
    }

    /// Get environmental data log for a specific parameter
    pub fn get_environmental_data_log(&self, parameter_name: &str, limit: Option<usize>) -> Vec<EnvironmentalDataPoint> {
        let filtered: Vec<EnvironmentalDataPoint> = self.environmental_data_log
            .iter()
            .filter(|point| point.parameter_name == parameter_name)
            .cloned()
            .collect();
        
        if let Some(limit) = limit {
            filtered.into_iter().rev().take(limit).collect()
        } else {
            filtered
        }
    }

    /// Get current sea state
    pub fn get_current_sea_state(&self) -> &SeaState {
        &self.current_sea_state
    }

    /// Check if rough sea mode is active
    pub fn is_rough_sea_mode_active(&self) -> bool {
        self.rough_sea_mode_active
    }

    /// Check if stabilization is active
    pub fn is_stabilization_active(&self) -> bool {
        self.stabilization_active
    }

    /// Check if cooling system is active
    pub fn is_cooling_system_active(&self) -> bool {
        self.cooling_system_active
    }

    /// Check if CPU throttling is active
    pub fn is_cpu_throttling_active(&self) -> bool {
        self.cpu_throttling_active
    }

    /// Reset rough sea mode
    pub fn reset_rough_sea_mode(&mut self) {
        self.rough_sea_mode_active = false;
        self.current_sea_state = SeaState::Calm;
    }

    /// Reset stabilization
    pub fn reset_stabilization(&mut self) {
        self.stabilization_active = false;
    }

    /// Reset cooling system
    pub fn reset_cooling_system(&mut self) {
        self.cooling_system_active = false;
    }

    /// Reset CPU throttling
    pub fn reset_cpu_throttling(&mut self) {
        self.cpu_throttling_active = false;
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
            wave_height_m: None,
            wave_period_s: None,
            sea_state: None,
            tilt_angle_degrees: None,
            acceleration_g: None,
            thermal_gradient_c_per_m: None,
            heat_dissipation_rate_w: None,
            cooling_efficiency_percent: None,
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
            wave_height_m: None,
            wave_period_s: None,
            sea_state: None,
            tilt_angle_degrees: None,
            acceleration_g: None,
            thermal_gradient_c_per_m: None,
            heat_dissipation_rate_w: None,
            cooling_efficiency_percent: None,
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

    #[test]
    fn test_sea_state_classification() {
        let config = EnvironmentalThresholds::default();
        let monitor = EnvironmentalMonitor::new(config);
        
        assert_eq!(monitor.classify_sea_state(0.05), SeaState::Calm);
        assert_eq!(monitor.classify_sea_state(0.3), SeaState::Smooth);
        assert_eq!(monitor.classify_sea_state(1.0), SeaState::Slight);
        assert_eq!(monitor.classify_sea_state(2.0), SeaState::Moderate);
        assert_eq!(monitor.classify_sea_state(3.0), SeaState::Rough);
        assert_eq!(monitor.classify_sea_state(5.0), SeaState::VeryRough);
        assert_eq!(monitor.classify_sea_state(7.0), SeaState::High);
        assert_eq!(monitor.classify_sea_state(12.0), SeaState::VeryHigh);
        assert_eq!(monitor.classify_sea_state(20.0), SeaState::Phenomenal);
    }

    #[test]
    fn test_rough_sea_adaptation() {
        let config = EnvironmentalThresholds::default();
        let mut monitor = EnvironmentalMonitor::new(config);
        
        let conditions = ExtendedEnvironmentalConditions {
            base_conditions: EnvironmentalConditions::default(),
            air_temperature_c: None,
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
            wave_height_m: Some(3.5), // Rough sea conditions
            wave_period_s: Some(8.0),
            sea_state: Some(SeaState::Rough),
            tilt_angle_degrees: Some(20.0), // Significant tilt
            acceleration_g: Some(2.0), // High acceleration
            thermal_gradient_c_per_m: None,
            heat_dissipation_rate_w: None,
            cooling_efficiency_percent: None,
            timestamp: SystemTime::now(),
            measurement_quality: MeasurementQuality {
                overall_quality: 0.8,
                sensor_health: HashMap::new(),
                calibration_status: HashMap::new(),
                last_calibration: HashMap::new(),
            },
        };
        
        let actions = monitor.update_conditions(conditions).unwrap();
        assert!(!actions.is_empty());
        assert!(monitor.is_stabilization_active());
        assert_eq!(monitor.get_current_sea_state(), &SeaState::Rough);
    }

    #[test]
    fn test_environmental_data_logging() {
        let config = EnvironmentalThresholds::default();
        let mut monitor = EnvironmentalMonitor::new(config);
        
        // Log some test data
        monitor.log_environmental_data("temperature".to_string(), 25.5, "°C".to_string());
        monitor.log_environmental_data("temperature".to_string(), 26.0, "°C".to_string());
        monitor.log_environmental_data("temperature".to_string(), 26.5, "°C".to_string());
        
        let temp_data = monitor.get_environmental_data_log("temperature", Some(5));
        assert_eq!(temp_data.len(), 3);
        assert_eq!(temp_data[0].parameter_name, "temperature");
        assert_eq!(temp_data[0].value, 26.5); // Most recent first
    }

    #[test]
    fn test_trend_analysis() {
        let config = EnvironmentalThresholds::default();
        let mut monitor = EnvironmentalMonitor::new(config);
        
        // Log increasing temperature trend
        for i in 0..10 {
            monitor.log_environmental_data("temperature".to_string(), 20.0 + i as f64, "°C".to_string());
        }
        
        let trends = monitor.run_trend_analysis().unwrap();
        assert!(trends.contains_key("temperature"));
        assert_eq!(trends["temperature"], TrendDirection::Increasing);
    }
}