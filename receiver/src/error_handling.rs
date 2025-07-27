// Comprehensive error handling and diagnostics system for underwater positioning
// Implements detailed error classification, reporting, and recovery strategies

use std::collections::HashMap;
use std::fmt;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use shared_positioning::CommError;
use shared_positioning::MessageParseError;

/// Comprehensive error classification for positioning system
#[derive(Debug, Clone, PartialEq)]
pub enum PositioningError {
    /// Insufficient anchor nodes for positioning
    InsufficientAnchors { 
        available: u8, 
        required: u8,
        anchor_ids: Vec<u16>,
    },
    
    /// Anchor geometry is degenerate or poor quality
    DegenerateGeometry { 
        condition_number: f64,
        volume: f64,
        geometry_type: GeometryIssue,
        anchor_positions: Vec<(u16, f64, f64, f64)>, // (id, lat, lon, depth)
    },
    
    /// Anchor data is too old to be reliable
    StaleData { 
        oldest_anchor_age_ms: u32,
        max_allowed_age_ms: u32,
        stale_anchor_ids: Vec<u16>,
    },
    
    /// Mathematical computation failed
    ComputationFailure { 
        operation: String,
        details: String,
        input_data_summary: String,
    },
    
    /// Transceiver communication error
    TransceiverError { 
        transceiver_id: u8,
        error: CommError,
        recovery_attempted: bool,
    },
    
    /// Message parsing or validation error
    MessageError {
        anchor_id: Option<u16>,
        error: MessageParseError,
        raw_data_length: usize,
    },
    
    /// Position validation failed
    PositionValidationError {
        position: (f64, f64, f64), // lat, lon, depth
        validation_issue: ValidationIssue,
        previous_position: Option<(f64, f64, f64)>,
    },
    
    /// System configuration error
    ConfigurationError {
        parameter: String,
        current_value: String,
        expected_range: String,
        impact: ConfigurationImpact,
    },
    
    /// Timing or synchronization error
    TimingError {
        error_type: TimingIssue,
        time_offset_ms: i64,
        affected_anchors: Vec<u16>,
    },
    
    /// Environmental conditions affecting positioning
    EnvironmentalError {
        condition: EnvironmentalCondition,
        severity: ErrorSeverity,
        mitigation_applied: bool,
    },
    
    /// System resource constraints
    ResourceError {
        resource: SystemResource,
        current_usage: f64,
        limit: f64,
        impact: ResourceImpact,
    },
    
    /// Critical system failure requiring shutdown
    CriticalFailure {
        failure_type: CriticalFailureType,
        system_state: String,
        recovery_possible: bool,
    },
}

/// Types of geometry issues
#[derive(Debug, Clone, PartialEq)]
pub enum GeometryIssue {
    Collinear,
    Coplanar,
    NearlyCollinear { min_angle_deg: f64 },
    NearlyCoplanar { min_volume: f64 },
    PoorDilutionOfPrecision { gdop: f64 },
    InsufficientSpread { max_distance: f64 },
}

/// Position validation issues
#[derive(Debug, Clone, PartialEq)]
pub enum ValidationIssue {
    OutOfBounds { bounds: String },
    UnrealisticJump { distance_m: f64, time_ms: u64 },
    DepthInconsistent { expected_range: (f64, f64) },
    CoordinateInvalid { coordinate: String, value: f64 },
    AccuracyTooLow { estimated_error_m: f64, threshold_m: f64 },
}

/// Configuration impact levels
#[derive(Debug, Clone, PartialEq)]
pub enum ConfigurationImpact {
    Minor,      // System can continue with reduced performance
    Moderate,   // Significant performance degradation
    Severe,     // System may fail intermittently
    Critical,   // System cannot operate reliably
}

/// Timing error types
#[derive(Debug, Clone, PartialEq)]
pub enum TimingIssue {
    ClockDrift,
    SynchronizationLoss,
    TimestampInvalid,
    PropagationDelayError,
    MessageOrderingError,
}

/// Environmental conditions
#[derive(Debug, Clone, PartialEq)]
pub enum EnvironmentalCondition {
    SoundSpeedVariation { measured: f64, expected: f64 },
    TemperatureExtreme { temperature_c: f32 },
    SignalAttenuation { loss_db: f32 },
    Multipath { delay_ms: f64 },
    NoiseLevel { snr_db: f32 },
}

/// System resources
#[derive(Debug, Clone, PartialEq)]
pub enum SystemResource {
    Memory,
    ComputationTime,
    BatteryPower,
    NetworkBandwidth,
    StorageSpace,
}

/// Resource impact
#[derive(Debug, Clone, PartialEq)]
pub enum ResourceImpact {
    PerformanceDegradation,
    FeatureDisabled,
    SystemSlowdown,
    OperationFailed,
}

/// Critical failure types
#[derive(Debug, Clone, PartialEq)]
pub enum CriticalFailureType {
    HardwareFault,
    MemoryCorruption,
    SystemOverload,
    SecurityBreach,
    DataCorruption,
}

/// Error severity levels
#[derive(Debug, Clone, PartialEq, PartialOrd, Ord, Eq, Hash)]
pub enum ErrorSeverity {
    Info,       // Informational, no action needed
    Warning,    // Potential issue, monitoring recommended
    Error,      // Error occurred, system can continue with degraded performance
    Critical,   // Critical error, immediate attention required
    Fatal,      // System cannot continue, shutdown required
}

/// Error context information for diagnostics
#[derive(Debug, Clone)]
pub struct ErrorContext {
    pub timestamp: SystemTime,
    pub system_state: SystemState,
    pub anchor_count: u8,
    pub last_successful_position: Option<(f64, f64, f64, SystemTime)>,
    pub computation_time_ms: f64,
    pub memory_usage_bytes: usize,
    pub active_transceivers: Vec<u8>,
    pub environmental_conditions: HashMap<String, f64>,
}

/// System state snapshot
#[derive(Debug, Clone)]
pub struct SystemState {
    pub uptime_ms: u64,
    pub positioning_attempts: u64,
    pub successful_positions: u64,
    pub error_count: u32,
    pub last_position_time: Option<SystemTime>,
    pub configuration_version: u32,
    pub active_features: Vec<String>,
}

impl Default for SystemState {
    fn default() -> Self {
        Self {
            uptime_ms: 0,
            positioning_attempts: 0,
            successful_positions: 0,
            error_count: 0,
            last_position_time: None,
            configuration_version: 1,
            active_features: vec!["trilateration".to_string()],
        }
    }
}

impl Default for ErrorContext {
    fn default() -> Self {
        Self {
            timestamp: SystemTime::now(),
            system_state: SystemState::default(),
            anchor_count: 0,
            last_successful_position: None,
            computation_time_ms: 0.0,
            memory_usage_bytes: 0,
            active_transceivers: Vec::new(),
            environmental_conditions: HashMap::new(),
        }
    }
}

impl fmt::Display for PositioningError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PositioningError::InsufficientAnchors { available, required, anchor_ids } => {
                write!(f, "Insufficient anchors: {} available, {} required. Active anchors: {:?}", 
                       available, required, anchor_ids)
            }
            PositioningError::DegenerateGeometry { condition_number, volume, geometry_type, .. } => {
                write!(f, "Degenerate geometry: condition number {:.2}, volume {:.6} m³, issue: {:?}", 
                       condition_number, volume, geometry_type)
            }
            PositioningError::StaleData { oldest_anchor_age_ms, max_allowed_age_ms, stale_anchor_ids } => {
                write!(f, "Stale anchor data: oldest {} ms (max {} ms), stale anchors: {:?}", 
                       oldest_anchor_age_ms, max_allowed_age_ms, stale_anchor_ids)
            }
            PositioningError::ComputationFailure { operation, details, .. } => {
                write!(f, "Computation failed in {}: {}", operation, details)
            }
            PositioningError::TransceiverError { transceiver_id, error, recovery_attempted } => {
                write!(f, "Transceiver {} error: {} (recovery attempted: {})", 
                       transceiver_id, error, recovery_attempted)
            }
            PositioningError::MessageError { anchor_id, error, raw_data_length } => {
                write!(f, "Message error from anchor {:?}: {} (data length: {} bytes)", 
                       anchor_id, error, raw_data_length)
            }
            PositioningError::PositionValidationError { position, validation_issue, .. } => {
                write!(f, "Position validation failed at ({:.6}, {:.6}, {:.2}): {:?}", 
                       position.0, position.1, position.2, validation_issue)
            }
            PositioningError::ConfigurationError { parameter, current_value, expected_range, impact } => {
                write!(f, "Configuration error: {} = {} (expected: {}), impact: {:?}", 
                       parameter, current_value, expected_range, impact)
            }
            PositioningError::TimingError { error_type, time_offset_ms, affected_anchors } => {
                write!(f, "Timing error: {:?}, offset {} ms, affected anchors: {:?}", 
                       error_type, time_offset_ms, affected_anchors)
            }
            PositioningError::EnvironmentalError { condition, severity, mitigation_applied } => {
                write!(f, "Environmental error: {:?}, severity: {:?}, mitigation: {}", 
                       condition, severity, mitigation_applied)
            }
            PositioningError::ResourceError { resource, current_usage, limit, impact } => {
                write!(f, "Resource error: {:?} usage {:.1}% of limit {:.1}, impact: {:?}", 
                       resource, current_usage * 100.0 / limit, limit, impact)
            }
            PositioningError::CriticalFailure { failure_type, system_state, recovery_possible } => {
                write!(f, "CRITICAL FAILURE: {:?}, state: {}, recovery possible: {}", 
                       failure_type, system_state, recovery_possible)
            }
        }
    }
}

impl std::error::Error for PositioningError {}

/// Error recovery strategies
#[derive(Debug, Clone)]
pub enum RecoveryStrategy {
    /// Retry operation with modified parameters
    Retry {
        max_attempts: u32,
        delay_ms: u64,
        parameter_adjustments: HashMap<String, String>,
    },
    
    /// Fallback to alternative algorithm or mode
    Fallback {
        fallback_mode: String,
        expected_accuracy_degradation: f64,
    },
    
    /// Ignore error and continue with warning
    IgnoreWithWarning {
        warning_message: String,
        monitoring_required: bool,
    },
    
    /// Reset subsystem
    Reset {
        subsystem: String,
        preserve_configuration: bool,
    },
    
    /// Graceful degradation
    Degrade {
        disabled_features: Vec<String>,
        performance_impact: f64,
    },
    
    /// Request user intervention
    UserIntervention {
        required_action: String,
        urgency: ErrorSeverity,
    },
    
    /// Shutdown system safely
    Shutdown {
        reason: String,
        save_state: bool,
    },
}

/// Error recovery manager
pub struct ErrorRecoveryManager {
    recovery_strategies: HashMap<String, RecoveryStrategy>,
    recovery_attempts: HashMap<String, u32>,
    last_recovery: HashMap<String, Instant>,
    recovery_history: Vec<RecoveryAttempt>,
    max_history_size: usize,
}

/// Recovery attempt record
#[derive(Debug, Clone)]
pub struct RecoveryAttempt {
    pub timestamp: Instant,
    pub error_type: String,
    pub strategy: RecoveryStrategy,
    pub success: bool,
    pub duration: Duration,
    pub side_effects: Vec<String>,
}

impl ErrorRecoveryManager {
    pub fn new() -> Self {
        let mut strategies = HashMap::new();
        
        // Default recovery strategies for different error types
        strategies.insert("InsufficientAnchors".to_string(), RecoveryStrategy::Retry {
            max_attempts: 5,
            delay_ms: 500,
            parameter_adjustments: HashMap::from([
                ("timeout".to_string(), "increase".to_string()),
                ("min_anchors".to_string(), "decrease".to_string()),
            ]),
        });
        
        strategies.insert("DegenerateGeometry".to_string(), RecoveryStrategy::Fallback {
            fallback_mode: "2D_positioning".to_string(),
            expected_accuracy_degradation: 2.0,
        });
        
        strategies.insert("StaleData".to_string(), RecoveryStrategy::IgnoreWithWarning {
            warning_message: "Using stale anchor data, accuracy may be reduced".to_string(),
            monitoring_required: true,
        });
        
        strategies.insert("ComputationFailure".to_string(), RecoveryStrategy::Fallback {
            fallback_mode: "simple_trilateration".to_string(),
            expected_accuracy_degradation: 1.5,
        });
        
        strategies.insert("TransceiverError".to_string(), RecoveryStrategy::Reset {
            subsystem: "transceiver".to_string(),
            preserve_configuration: true,
        });
        
        strategies.insert("CriticalFailure".to_string(), RecoveryStrategy::Shutdown {
            reason: "Critical system failure detected".to_string(),
            save_state: true,
        });
        
        Self {
            recovery_strategies: strategies,
            recovery_attempts: HashMap::new(),
            last_recovery: HashMap::new(),
            recovery_history: Vec::new(),
            max_history_size: 100,
        }
    }
    
    /// Handle error and determine recovery strategy
    pub fn handle_error(&mut self, error: &PositioningError, context: &ErrorContext) -> Option<RecoveryStrategy> {
        let error_type = self.classify_error(error);
        let now = Instant::now();
        
        // Check if we should apply recovery strategy based on timing and attempt count
        if let Some(last_time) = self.last_recovery.get(&error_type) {
            if now.duration_since(*last_time) < Duration::from_millis(1000) {
                // Too soon since last recovery attempt
                return None;
            }
        }
        
        let attempts = self.recovery_attempts.entry(error_type.clone()).or_insert(0);
        
        if let Some(strategy) = self.recovery_strategies.get(&error_type) {
            match strategy {
                RecoveryStrategy::Retry { max_attempts, .. } => {
                    if *attempts < *max_attempts {
                        *attempts += 1;
                        self.last_recovery.insert(error_type.clone(), now);
                        return Some(strategy.clone());
                    } else {
                        // Max retries exceeded, try fallback strategy
                        return self.get_fallback_strategy(&error_type);
                    }
                }
                _ => {
                    *attempts += 1;
                    self.last_recovery.insert(error_type.clone(), now);
                    return Some(strategy.clone());
                }
            }
        }
        
        // No specific strategy found, use default based on severity
        self.get_default_strategy(error, context)
    }
    
    /// Record recovery attempt result
    pub fn record_recovery_attempt(&mut self, error_type: String, strategy: RecoveryStrategy, 
                                   success: bool, duration: Duration, side_effects: Vec<String>) {
        let attempt = RecoveryAttempt {
            timestamp: Instant::now(),
            error_type: error_type.clone(),
            strategy,
            success,
            duration,
            side_effects,
        };
        
        self.recovery_history.push(attempt);
        
        // Limit history size
        if self.recovery_history.len() > self.max_history_size {
            self.recovery_history.remove(0);
        }
        
        // Reset attempt counter on success
        if success {
            self.recovery_attempts.remove(&error_type);
        }
    }
    
    /// Get recovery statistics
    pub fn get_recovery_statistics(&self) -> RecoveryStatistics {
        let total_attempts = self.recovery_history.len();
        let successful_attempts = self.recovery_history.iter().filter(|a| a.success).count();
        
        let mut strategy_success_rates = HashMap::new();
        let mut error_type_counts = HashMap::new();
        
        for attempt in &self.recovery_history {
            let strategy_name = format!("{:?}", attempt.strategy).split('{').next().unwrap_or("Unknown").to_string();
            let entry = strategy_success_rates.entry(strategy_name).or_insert((0, 0));
            entry.0 += 1;
            if attempt.success {
                entry.1 += 1;
            }
            
            *error_type_counts.entry(attempt.error_type.clone()).or_insert(0) += 1;
        }
        
        RecoveryStatistics {
            total_attempts,
            successful_attempts,
            success_rate: if total_attempts > 0 { successful_attempts as f64 / total_attempts as f64 } else { 0.0 },
            strategy_success_rates,
            error_type_counts,
            average_recovery_time: if !self.recovery_history.is_empty() {
                self.recovery_history.iter().map(|a| a.duration.as_millis() as f64).sum::<f64>() / self.recovery_history.len() as f64
            } else { 0.0 },
        }
    }
    
    /// Classify error for recovery strategy selection
    fn classify_error(&self, error: &PositioningError) -> String {
        match error {
            PositioningError::InsufficientAnchors { .. } => "InsufficientAnchors".to_string(),
            PositioningError::DegenerateGeometry { .. } => "DegenerateGeometry".to_string(),
            PositioningError::StaleData { .. } => "StaleData".to_string(),
            PositioningError::ComputationFailure { .. } => "ComputationFailure".to_string(),
            PositioningError::TransceiverError { .. } => "TransceiverError".to_string(),
            PositioningError::MessageError { .. } => "MessageError".to_string(),
            PositioningError::PositionValidationError { .. } => "PositionValidationError".to_string(),
            PositioningError::ConfigurationError { .. } => "ConfigurationError".to_string(),
            PositioningError::TimingError { .. } => "TimingError".to_string(),
            PositioningError::EnvironmentalError { .. } => "EnvironmentalError".to_string(),
            PositioningError::ResourceError { .. } => "ResourceError".to_string(),
            PositioningError::CriticalFailure { .. } => "CriticalFailure".to_string(),
        }
    }
    
    /// Get fallback strategy when primary strategy fails
    fn get_fallback_strategy(&self, error_type: &str) -> Option<RecoveryStrategy> {
        match error_type {
            "InsufficientAnchors" => Some(RecoveryStrategy::Fallback {
                fallback_mode: "last_known_position".to_string(),
                expected_accuracy_degradation: 10.0,
            }),
            "DegenerateGeometry" => Some(RecoveryStrategy::IgnoreWithWarning {
                warning_message: "Poor geometry detected, accuracy reduced".to_string(),
                monitoring_required: true,
            }),
            "ComputationFailure" => Some(RecoveryStrategy::Reset {
                subsystem: "positioning_engine".to_string(),
                preserve_configuration: true,
            }),
            _ => None,
        }
    }
    
    /// Get default strategy based on error severity
    fn get_default_strategy(&self, error: &PositioningError, _context: &ErrorContext) -> Option<RecoveryStrategy> {
        let severity = self.assess_error_severity(error);
        
        match severity {
            ErrorSeverity::Info | ErrorSeverity::Warning => Some(RecoveryStrategy::IgnoreWithWarning {
                warning_message: "Minor issue detected, monitoring".to_string(),
                monitoring_required: true,
            }),
            ErrorSeverity::Error => Some(RecoveryStrategy::Retry {
                max_attempts: 3,
                delay_ms: 100,
                parameter_adjustments: HashMap::new(),
            }),
            ErrorSeverity::Critical => Some(RecoveryStrategy::Reset {
                subsystem: "positioning_system".to_string(),
                preserve_configuration: true,
            }),
            ErrorSeverity::Fatal => Some(RecoveryStrategy::Shutdown {
                reason: "Fatal error detected".to_string(),
                save_state: true,
            }),
        }
    }
    
    /// Assess error severity
    fn assess_error_severity(&self, error: &PositioningError) -> ErrorSeverity {
        match error {
            PositioningError::InsufficientAnchors { available, required, .. } => {
                if *available == 0 { ErrorSeverity::Critical } 
                else if *available < *required { ErrorSeverity::Error }
                else { ErrorSeverity::Warning }
            }
            PositioningError::DegenerateGeometry { condition_number, .. } => {
                if *condition_number > 10000.0 { ErrorSeverity::Critical }
                else if *condition_number > 1000.0 { ErrorSeverity::Error }
                else { ErrorSeverity::Warning }
            }
            PositioningError::StaleData { oldest_anchor_age_ms, max_allowed_age_ms, .. } => {
                let age_ratio = *oldest_anchor_age_ms as f64 / *max_allowed_age_ms as f64;
                if age_ratio > 5.0 { ErrorSeverity::Critical }
                else if age_ratio > 2.0 { ErrorSeverity::Error }
                else { ErrorSeverity::Warning }
            }
            PositioningError::ComputationFailure { .. } => ErrorSeverity::Error,
            PositioningError::TransceiverError { .. } => ErrorSeverity::Error,
            PositioningError::MessageError { .. } => ErrorSeverity::Warning,
            PositioningError::PositionValidationError { .. } => ErrorSeverity::Warning,
            PositioningError::ConfigurationError { impact, .. } => {
                match impact {
                    ConfigurationImpact::Minor => ErrorSeverity::Warning,
                    ConfigurationImpact::Moderate => ErrorSeverity::Error,
                    ConfigurationImpact::Severe => ErrorSeverity::Critical,
                    ConfigurationImpact::Critical => ErrorSeverity::Fatal,
                }
            }
            PositioningError::TimingError { .. } => ErrorSeverity::Error,
            PositioningError::EnvironmentalError { severity, .. } => severity.clone(),
            PositioningError::ResourceError { .. } => ErrorSeverity::Error,
            PositioningError::CriticalFailure { .. } => ErrorSeverity::Fatal,
        }
    }
}

impl Default for ErrorRecoveryManager {
    fn default() -> Self {
        Self::new()
    }
}

/// Recovery statistics
#[derive(Debug, Clone)]
pub struct RecoveryStatistics {
    pub total_attempts: usize,
    pub successful_attempts: usize,
    pub success_rate: f64,
    pub strategy_success_rates: HashMap<String, (usize, usize)>, // (total, successful)
    pub error_type_counts: HashMap<String, usize>,
    pub average_recovery_time: f64, // milliseconds
}

/// Error logging and reporting system
pub struct ErrorLogger {
    log_entries: Vec<ErrorLogEntry>,
    max_entries: usize,
    log_level: ErrorSeverity,
    output_handlers: Vec<Box<dyn LogOutputHandler>>,
}

/// Error log entry
#[derive(Debug, Clone)]
pub struct ErrorLogEntry {
    pub timestamp: SystemTime,
    pub severity: ErrorSeverity,
    pub error: PositioningError,
    pub context: ErrorContext,
    pub recovery_action: Option<RecoveryStrategy>,
    pub resolution: Option<String>,
}

/// Log output handler trait
pub trait LogOutputHandler {
    fn handle_log_entry(&mut self, entry: &ErrorLogEntry);
    fn flush(&mut self);
}

/// Console log output handler
pub struct ConsoleLogHandler {
    min_severity: ErrorSeverity,
}

impl ConsoleLogHandler {
    pub fn new(min_severity: ErrorSeverity) -> Self {
        Self { min_severity }
    }
}

impl LogOutputHandler for ConsoleLogHandler {
    fn handle_log_entry(&mut self, entry: &ErrorLogEntry) {
        if entry.severity >= self.min_severity {
            let timestamp = entry.timestamp
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_millis();
            
            println!("[{}] {:?}: {}", timestamp, entry.severity, entry.error);
            
            if let Some(recovery) = &entry.recovery_action {
                println!("  Recovery: {:?}", recovery);
            }
            
            if let Some(resolution) = &entry.resolution {
                println!("  Resolution: {}", resolution);
            }
        }
    }
    
    fn flush(&mut self) {
        // Console output is immediately flushed
    }
}

impl ErrorLogger {
    pub fn new(max_entries: usize, log_level: ErrorSeverity) -> Self {
        Self {
            log_entries: Vec::new(),
            max_entries,
            log_level,
            output_handlers: Vec::new(),
        }
    }
    
    /// Add output handler
    pub fn add_output_handler(&mut self, handler: Box<dyn LogOutputHandler>) {
        self.output_handlers.push(handler);
    }
    
    /// Log error with context
    pub fn log_error(&mut self, error: PositioningError, context: ErrorContext, 
                     recovery_action: Option<RecoveryStrategy>) {
        let severity = self.assess_error_severity(&error);
        
        if severity >= self.log_level {
            let entry = ErrorLogEntry {
                timestamp: SystemTime::now(),
                severity,
                error,
                context,
                recovery_action,
                resolution: None,
            };
            
            // Send to output handlers
            for handler in &mut self.output_handlers {
                handler.handle_log_entry(&entry);
            }
            
            // Store in memory
            self.log_entries.push(entry);
            
            // Limit memory usage
            if self.log_entries.len() > self.max_entries {
                self.log_entries.remove(0);
            }
        }
    }
    
    /// Update log entry with resolution
    pub fn update_resolution(&mut self, error_id: usize, resolution: String) {
        if let Some(entry) = self.log_entries.get_mut(error_id) {
            entry.resolution = Some(resolution);
        }
    }
    
    /// Get error statistics
    pub fn get_error_statistics(&self) -> ErrorStatistics {
        let mut severity_counts = HashMap::new();
        let mut error_type_counts = HashMap::new();
        let mut hourly_counts = HashMap::new();
        
        let now = SystemTime::now();
        let one_hour = Duration::from_secs(3600);
        
        for entry in &self.log_entries {
            // Count by severity
            *severity_counts.entry(entry.severity.clone()).or_insert(0) += 1;
            
            // Count by error type
            let error_type = format!("{:?}", entry.error).split('{').next().unwrap_or("Unknown").to_string();
            *error_type_counts.entry(error_type).or_insert(0) += 1;
            
            // Count by hour (for recent entries)
            if let Ok(duration) = now.duration_since(entry.timestamp) {
                if duration < Duration::from_secs(24 * 3600) { // Last 24 hours
                    let hour = duration.as_secs() / 3600;
                    *hourly_counts.entry(hour).or_insert(0) += 1;
                }
            }
        }
        
        ErrorStatistics {
            total_errors: self.log_entries.len(),
            severity_counts,
            error_type_counts,
            hourly_counts,
            resolved_errors: self.log_entries.iter().filter(|e| e.resolution.is_some()).count(),
        }
    }
    
    /// Generate diagnostic report
    pub fn generate_diagnostic_report(&self) -> String {
        let mut report = String::new();
        report.push_str("=== ERROR DIAGNOSTIC REPORT ===\n\n");
        
        let stats = self.get_error_statistics();
        
        report.push_str(&format!("Total errors logged: {}\n", stats.total_errors));
        report.push_str(&format!("Resolved errors: {}\n", stats.resolved_errors));
        report.push_str(&format!("Resolution rate: {:.1}%\n\n", 
                                if stats.total_errors > 0 { 
                                    stats.resolved_errors as f64 / stats.total_errors as f64 * 100.0 
                                } else { 0.0 }));
        
        report.push_str("SEVERITY BREAKDOWN:\n");
        for (severity, count) in &stats.severity_counts {
            report.push_str(&format!("  {:?}: {}\n", severity, count));
        }
        
        report.push_str("\nERROR TYPE BREAKDOWN:\n");
        for (error_type, count) in &stats.error_type_counts {
            report.push_str(&format!("  {}: {}\n", error_type, count));
        }
        
        if !self.log_entries.is_empty() {
            report.push_str("\nRECENT ERRORS:\n");
            for entry in self.log_entries.iter().rev().take(5) {
                let timestamp = entry.timestamp
                    .duration_since(UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_millis();
                report.push_str(&format!("  [{}] {:?}: {}\n", timestamp, entry.severity, entry.error));
            }
        }
        
        report
    }
    
    /// Assess error severity (same logic as recovery manager)
    fn assess_error_severity(&self, error: &PositioningError) -> ErrorSeverity {
        match error {
            PositioningError::InsufficientAnchors { available, required, .. } => {
                if *available == 0 { ErrorSeverity::Critical } 
                else if *available < *required { ErrorSeverity::Error }
                else { ErrorSeverity::Warning }
            }
            PositioningError::DegenerateGeometry { condition_number, .. } => {
                if *condition_number > 10000.0 { ErrorSeverity::Critical }
                else if *condition_number > 1000.0 { ErrorSeverity::Error }
                else { ErrorSeverity::Warning }
            }
            PositioningError::CriticalFailure { .. } => ErrorSeverity::Fatal,
            _ => ErrorSeverity::Error,
        }
    }
}

/// Error statistics
#[derive(Debug, Clone)]
pub struct ErrorStatistics {
    pub total_errors: usize,
    pub severity_counts: HashMap<ErrorSeverity, usize>,
    pub error_type_counts: HashMap<String, usize>,
    pub hourly_counts: HashMap<u64, usize>,
    pub resolved_errors: usize,
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_error_classification() {
        let error = PositioningError::InsufficientAnchors {
            available: 2,
            required: 3,
            anchor_ids: vec![1, 2],
        };
        
        let error_str = format!("{}", error);
        assert!(error_str.contains("Insufficient anchors"));
        assert!(error_str.contains("2 available"));
        assert!(error_str.contains("3 required"));
    }
    
    #[test]
    fn test_recovery_manager() {
        let mut manager = ErrorRecoveryManager::new();
        let context = ErrorContext::default();
        
        let error = PositioningError::InsufficientAnchors {
            available: 2,
            required: 3,
            anchor_ids: vec![1, 2],
        };
        
        let strategy = manager.handle_error(&error, &context);
        assert!(strategy.is_some());
        
        if let Some(RecoveryStrategy::Retry { max_attempts, .. }) = strategy {
            assert_eq!(max_attempts, 5);
        }
    }
    
    #[test]
    fn test_error_logger() {
        let mut logger = ErrorLogger::new(100, ErrorSeverity::Warning);
        logger.add_output_handler(Box::new(ConsoleLogHandler::new(ErrorSeverity::Error)));
        
        let error = PositioningError::ComputationFailure {
            operation: "trilateration".to_string(),
            details: "Matrix singular".to_string(),
            input_data_summary: "4 anchors".to_string(),
        };
        
        let context = ErrorContext::default();
        logger.log_error(error, context, None);
        
        let stats = logger.get_error_statistics();
        assert_eq!(stats.total_errors, 1);
    }
}