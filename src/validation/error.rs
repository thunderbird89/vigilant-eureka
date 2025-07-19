use std::fmt;
use std::collections::HashMap;
use std::time::{SystemTime, UNIX_EPOCH};
use serde::{Serialize, Deserialize};

/// Comprehensive error classification for the underwater positioning system
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PositioningError {
    // Anchor-related errors
    InsufficientAnchors { 
        available: u8, 
        required: u8,
        anchor_ids: Vec<u16>,
    },
    AnchorTimeout { 
        anchor_id: u16, 
        last_seen_ms: u64,
        timeout_threshold_ms: u64,
    },
    AnchorSignalLoss { 
        anchor_id: u16, 
        consecutive_failures: u32,
        signal_quality_history: Vec<u8>,
    },
    
    // Geometric errors
    DegenerateGeometry { 
        condition_number: f64,
        anchor_positions: Vec<(u16, f64, f64, f64)>, // (id, lat, lon, depth)
        geometry_type: GeometryIssue,
    },
    PoorGeometry { 
        gdop_value: f64,
        recommended_threshold: f64,
        anchor_distribution: String,
    },
    
    // Data quality errors
    StaleData { 
        oldest_anchor_id: u16,
        age_ms: u64,
        max_allowed_age_ms: u64,
    },
    CorruptedData { 
        anchor_id: u16,
        corruption_type: DataCorruption,
        raw_data_sample: Vec<u8>,
    },
    InconsistentMeasurements { 
        anchor_id: u16,
        measurement_variance: f64,
        expected_variance: f64,
    },
    
    // Computation errors
    ComputationFailure { 
        operation: String,
        error_code: ComputationErrorCode,
        context: String,
    },
    MatrixSingular { 
        matrix_type: String,
        condition_number: f64,
        suggested_action: String,
    },
    ConvergenceFailure { 
        algorithm: String,
        iterations: u32,
        residual_error: f64,
    },
    
    // Communication errors
    TransceiverError { 
        transceiver_id: u8,
        error_type: TransceiverErrorType,
        retry_count: u32,
    },
    MessageParsingError { 
        raw_message_length: usize,
        parse_error: String,
        message_version: Option<u8>,
    },
    
    // Environmental errors
    EnvironmentalConditions { 
        condition_type: EnvironmentalIssue,
        severity: ErrorSeverity,
        mitigation_applied: bool,
    },
    SoundSpeedVariation { 
        measured_speed: f64,
        expected_speed: f64,
        depth: f64,
        temperature: Option<f64>,
    },
    
    // System resource errors
    ResourceExhaustion { 
        resource_type: ResourceType,
        current_usage: u64,
        limit: u64,
    },
    PerformanceConstraintViolation { 
        constraint_type: PerformanceConstraint,
        measured_value: f64,
        threshold: f64,
    },
    
    // Configuration errors
    ConfigurationError { 
        parameter: String,
        invalid_value: String,
        valid_range: String,
    },
    CalibrationRequired { 
        component: String,
        last_calibration: Option<u64>,
        recommended_interval_ms: u64,
    },
}

/// Types of geometric issues that can affect positioning
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum GeometryIssue {
    Collinear,
    Coplanar,
    TooClose,
    TooFar,
    Asymmetric,
}

/// Types of data corruption
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum DataCorruption {
    ChecksumMismatch,
    InvalidFormat,
    OutOfRange,
    SequenceError,
    TimestampError,
}

/// Computation error codes
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ComputationErrorCode {
    DivisionByZero,
    NumericalInstability,
    OverflowError,
    UnderflowError,
    InvalidInput,
    AlgorithmFailure,
}

/// Transceiver error types
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum TransceiverErrorType {
    CommunicationTimeout,
    InvalidResponse,
    HardwareFailure,
    ConfigurationMismatch,
    SignalInterference,
}

/// Environmental issues affecting positioning
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum EnvironmentalIssue {
    MultipathPropagation,
    ThermoclineEffect,
    CurrentVariation,
    NoiseInterference,
    ObstructionDetected,
}

/// Error severity levels
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ErrorSeverity {
    Critical,   // System cannot function
    High,       // Significant accuracy degradation
    Medium,     // Moderate impact on performance
    Low,        // Minor impact, system still functional
    Warning,    // Potential issue, no immediate impact
}

/// System resource types
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ResourceType {
    Memory,
    ComputationTime,
    BatteryPower,
    StorageSpace,
    NetworkBandwidth,
}

/// Performance constraint types
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PerformanceConstraint {
    MaxComputationTime,
    MaxMemoryUsage,
    MinAccuracy,
    MaxLatency,
    MinUpdateRate,
}

/// Error context information for diagnostics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ErrorContext {
    pub timestamp_ms: u64,
    pub system_state: SystemState,
    pub active_anchors: Vec<u16>,
    pub last_successful_position: Option<(f64, f64, f64)>, // lat, lon, depth
    pub performance_metrics: PerformanceMetrics,
    pub environmental_conditions: EnvironmentalConditions,
}

/// Current system state snapshot
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SystemState {
    pub positioning_mode: PositioningMode,
    pub anchor_count: u8,
    pub last_position_update_ms: u64,
    pub computation_load: f32, // 0.0 to 1.0
    pub memory_usage_bytes: u64,
    pub uptime_ms: u64,
}

/// Positioning modes
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PositioningMode {
    FullPrecision3D,
    Reduced2D,
    DeadReckoning,
    Fallback,
    Disabled,
}

/// Performance metrics for diagnostics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerformanceMetrics {
    pub avg_computation_time_ms: f64,
    pub max_computation_time_ms: f64,
    pub position_update_rate_hz: f64,
    pub accuracy_estimate_m: f64,
    pub successful_positions: u64,
    pub failed_positions: u64,
}

/// Environmental conditions affecting system
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnvironmentalConditions {
    pub sound_speed_ms: f64,
    pub estimated_depth_m: f64,
    pub signal_noise_level: f64,
    pub multipath_detected: bool,
    pub temperature_c: Option<f64>,
    pub pressure_bar: Option<f64>,
}

/// Error recovery strategies
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum RecoveryStrategy {
    RetryOperation { max_attempts: u32, delay_ms: u64 },
    FallbackAlgorithm { algorithm_name: String },
    ReducedPrecision { precision_level: u8 },
    IgnoreAnchor { anchor_id: u16, duration_ms: u64 },
    RecalibrateSystem { component: String },
    RestartSubsystem { subsystem: String },
    WaitForConditions { condition: String, timeout_ms: u64 },
    UserIntervention { required_action: String },
}

/// Diagnostic information for troubleshooting
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiagnosticInfo {
    pub error_id: u64,
    pub error: PositioningError,
    pub context: ErrorContext,
    pub recovery_strategy: Option<RecoveryStrategy>,
    pub recovery_attempts: u32,
    pub resolution_status: ResolutionStatus,
    pub related_errors: Vec<u64>, // IDs of related errors
    pub debug_data: HashMap<String, String>,
}

/// Status of error resolution
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ResolutionStatus {
    Unresolved,
    InProgress,
    Resolved,
    Mitigated,
    Ignored,
    RequiresIntervention,
}

/// Error logging and reporting system
pub struct ErrorReporter {
    error_history: Vec<DiagnosticInfo>,
    error_counter: u64,
    max_history_size: usize,
    error_statistics: HashMap<String, ErrorStatistics>,
    recovery_strategies: HashMap<String, RecoveryStrategy>,
    system_start_time: u64,
}

/// Statistics for error types
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
struct ErrorStatistics {
    occurrence_count: u64,
    first_occurrence: u64,
    last_occurrence: u64,
    resolution_success_rate: f64,
    avg_resolution_time_ms: f64,
}

impl ErrorReporter {
    /// Create a new error reporter
    pub fn new() -> Self {
        Self {
            error_history: Vec::new(),
            error_counter: 0,
            max_history_size: 1000,
            error_statistics: HashMap::new(),
            recovery_strategies: Self::default_recovery_strategies(),
            system_start_time: Self::current_time_ms(),
        }
    }

    /// Create error reporter with custom configuration
    pub fn with_config(max_history_size: usize) -> Self {
        Self {
            error_history: Vec::new(),
            error_counter: 0,
            max_history_size,
            error_statistics: HashMap::new(),
            recovery_strategies: Self::default_recovery_strategies(),
            system_start_time: Self::current_time_ms(),
        }
    }

    /// Report a new error with context
    pub fn report_error(&mut self, error: PositioningError, context: ErrorContext) -> u64 {
        self.error_counter += 1;
        let error_id = self.error_counter;
        
        // Determine recovery strategy
        let recovery_strategy = self.determine_recovery_strategy(&error);
        
        // Create diagnostic info
        let diagnostic = DiagnosticInfo {
            error_id,
            error: error.clone(),
            context,
            recovery_strategy,
            recovery_attempts: 0,
            resolution_status: ResolutionStatus::Unresolved,
            related_errors: Vec::new(),
            debug_data: HashMap::new(),
        };
        
        // Update statistics
        self.update_error_statistics(&error);
        
        // Add to history
        self.error_history.push(diagnostic);
        
        // Maintain history size limit
        if self.error_history.len() > self.max_history_size {
            self.error_history.remove(0);
        }
        
        // Log error (in a real system, this would go to a proper logging system)
        self.log_error(&error, error_id);
        
        error_id
    }

    /// Update error resolution status
    pub fn update_resolution_status(&mut self, error_id: u64, status: ResolutionStatus) -> bool {
        // First, find the diagnostic and extract the error type name
        let error_info = self.error_history.iter()
            .find(|d| d.error_id == error_id)
            .map(|d| (self.get_error_type_name(&d.error), d.context.timestamp_ms));
        
        // Then update the resolution status
        let updated = if let Some(diagnostic) = self.error_history.iter_mut().find(|d| d.error_id == error_id) {
            diagnostic.resolution_status = status.clone();
            true
        } else {
            false
        };
        
        // Finally, update statistics if resolved
        if updated && status == ResolutionStatus::Resolved {
            if let Some((error_type, timestamp)) = error_info {
                let resolution_time = Self::current_time_ms() - timestamp;
                if let Some(stats) = self.error_statistics.get_mut(&error_type) {
                    stats.avg_resolution_time_ms = 
                        (stats.avg_resolution_time_ms + resolution_time as f64) / 2.0;
                }
            }
        }
        
        updated
    }

    /// Get error by ID
    pub fn get_error(&self, error_id: u64) -> Option<&DiagnosticInfo> {
        self.error_history.iter().find(|d| d.error_id == error_id)
    }

    /// Get recent errors
    pub fn get_recent_errors(&self, count: usize) -> Vec<&DiagnosticInfo> {
        self.error_history.iter().rev().take(count).collect()
    }

    /// Get errors by type
    pub fn get_errors_by_type(&self, error_type: &str) -> Vec<&DiagnosticInfo> {
        self.error_history.iter()
            .filter(|d| self.get_error_type_name(&d.error) == error_type)
            .collect()
    }

    /// Get unresolved errors
    pub fn get_unresolved_errors(&self) -> Vec<&DiagnosticInfo> {
        self.error_history.iter()
            .filter(|d| d.resolution_status == ResolutionStatus::Unresolved)
            .collect()
    }

    /// Generate error summary report
    pub fn generate_error_report(&self) -> ErrorReport {
        let total_errors = self.error_history.len();
        let unresolved_count = self.get_unresolved_errors().len();
        let critical_errors = self.error_history.iter()
            .filter(|d| self.get_error_severity(&d.error) == ErrorSeverity::Critical)
            .count();
        
        let mut error_type_counts = HashMap::new();
        for diagnostic in &self.error_history {
            let error_type = self.get_error_type_name(&diagnostic.error);
            *error_type_counts.entry(error_type).or_insert(0) += 1;
        }
        
        ErrorReport {
            report_timestamp: Self::current_time_ms(),
            system_uptime_ms: Self::current_time_ms() - self.system_start_time,
            total_errors,
            unresolved_errors: unresolved_count,
            critical_errors,
            error_type_distribution: error_type_counts,
            error_statistics: self.error_statistics.clone(),
            recent_errors: self.get_recent_errors(10).into_iter().cloned().collect(),
        }
    }

    /// Clear error history
    pub fn clear_history(&mut self) {
        self.error_history.clear();
        self.error_statistics.clear();
    }

    /// Add debug data to an error
    pub fn add_debug_data(&mut self, error_id: u64, key: String, value: String) -> bool {
        if let Some(diagnostic) = self.error_history.iter_mut().find(|d| d.error_id == error_id) {
            diagnostic.debug_data.insert(key, value);
            true
        } else {
            false
        }
    }

    /// Link related errors
    pub fn link_errors(&mut self, primary_error_id: u64, related_error_id: u64) -> bool {
        if let Some(diagnostic) = self.error_history.iter_mut().find(|d| d.error_id == primary_error_id) {
            if !diagnostic.related_errors.contains(&related_error_id) {
                diagnostic.related_errors.push(related_error_id);
            }
            true
        } else {
            false
        }
    }

    // Private helper methods
    
    fn determine_recovery_strategy(&self, error: &PositioningError) -> Option<RecoveryStrategy> {
        match error {
            PositioningError::InsufficientAnchors { .. } => {
                Some(RecoveryStrategy::WaitForConditions {
                    condition: "Additional anchor signals".to_string(),
                    timeout_ms: 30000,
                })
            }
            PositioningError::DegenerateGeometry { .. } => {
                Some(RecoveryStrategy::FallbackAlgorithm {
                    algorithm_name: "2D positioning with depth estimation".to_string(),
                })
            }
            PositioningError::TransceiverError { .. } => {
                Some(RecoveryStrategy::RetryOperation {
                    max_attempts: 3,
                    delay_ms: 1000,
                })
            }
            PositioningError::ComputationFailure { .. } => {
                Some(RecoveryStrategy::ReducedPrecision {
                    precision_level: 2,
                })
            }
            _ => None,
        }
    }

    fn update_error_statistics(&mut self, error: &PositioningError) {
        let error_type = self.get_error_type_name(error);
        let current_time = Self::current_time_ms();
        
        let stats = self.error_statistics.entry(error_type).or_insert(ErrorStatistics::default());
        stats.occurrence_count += 1;
        stats.last_occurrence = current_time;
        
        if stats.first_occurrence == 0 {
            stats.first_occurrence = current_time;
        }
    }

    fn get_error_type_name(&self, error: &PositioningError) -> String {
        match error {
            PositioningError::InsufficientAnchors { .. } => "InsufficientAnchors".to_string(),
            PositioningError::AnchorTimeout { .. } => "AnchorTimeout".to_string(),
            PositioningError::AnchorSignalLoss { .. } => "AnchorSignalLoss".to_string(),
            PositioningError::DegenerateGeometry { .. } => "DegenerateGeometry".to_string(),
            PositioningError::PoorGeometry { .. } => "PoorGeometry".to_string(),
            PositioningError::StaleData { .. } => "StaleData".to_string(),
            PositioningError::CorruptedData { .. } => "CorruptedData".to_string(),
            PositioningError::InconsistentMeasurements { .. } => "InconsistentMeasurements".to_string(),
            PositioningError::ComputationFailure { .. } => "ComputationFailure".to_string(),
            PositioningError::MatrixSingular { .. } => "MatrixSingular".to_string(),
            PositioningError::ConvergenceFailure { .. } => "ConvergenceFailure".to_string(),
            PositioningError::TransceiverError { .. } => "TransceiverError".to_string(),
            PositioningError::MessageParsingError { .. } => "MessageParsingError".to_string(),
            PositioningError::EnvironmentalConditions { .. } => "EnvironmentalConditions".to_string(),
            PositioningError::SoundSpeedVariation { .. } => "SoundSpeedVariation".to_string(),
            PositioningError::ResourceExhaustion { .. } => "ResourceExhaustion".to_string(),
            PositioningError::PerformanceConstraintViolation { .. } => "PerformanceConstraintViolation".to_string(),
            PositioningError::ConfigurationError { .. } => "ConfigurationError".to_string(),
            PositioningError::CalibrationRequired { .. } => "CalibrationRequired".to_string(),
        }
    }

    fn get_error_severity(&self, error: &PositioningError) -> ErrorSeverity {
        match error {
            PositioningError::InsufficientAnchors { .. } => ErrorSeverity::High,
            PositioningError::AnchorTimeout { .. } => ErrorSeverity::Medium,
            PositioningError::AnchorSignalLoss { .. } => ErrorSeverity::Medium,
            PositioningError::DegenerateGeometry { .. } => ErrorSeverity::High,
            PositioningError::PoorGeometry { .. } => ErrorSeverity::Medium,
            PositioningError::StaleData { .. } => ErrorSeverity::Medium,
            PositioningError::CorruptedData { .. } => ErrorSeverity::High,
            PositioningError::InconsistentMeasurements { .. } => ErrorSeverity::Medium,
            PositioningError::ComputationFailure { .. } => ErrorSeverity::Critical,
            PositioningError::MatrixSingular { .. } => ErrorSeverity::High,
            PositioningError::ConvergenceFailure { .. } => ErrorSeverity::High,
            PositioningError::TransceiverError { .. } => ErrorSeverity::High,
            PositioningError::MessageParsingError { .. } => ErrorSeverity::Medium,
            PositioningError::EnvironmentalConditions { condition_type: _, severity, .. } => severity.clone(),
            PositioningError::SoundSpeedVariation { .. } => ErrorSeverity::Low,
            PositioningError::ResourceExhaustion { .. } => ErrorSeverity::Critical,
            PositioningError::PerformanceConstraintViolation { .. } => ErrorSeverity::High,
            PositioningError::ConfigurationError { .. } => ErrorSeverity::High,
            PositioningError::CalibrationRequired { .. } => ErrorSeverity::Warning,
        }
    }

    fn log_error(&self, error: &PositioningError, error_id: u64) {
        // In a real system, this would use a proper logging framework
        println!("[ERROR {}] {}", error_id, error);
    }

    fn default_recovery_strategies() -> HashMap<String, RecoveryStrategy> {
        let mut strategies = HashMap::new();
        
        strategies.insert("InsufficientAnchors".to_string(), 
            RecoveryStrategy::WaitForConditions {
                condition: "Additional anchor signals".to_string(),
                timeout_ms: 30000,
            });
        
        strategies.insert("DegenerateGeometry".to_string(),
            RecoveryStrategy::FallbackAlgorithm {
                algorithm_name: "2D positioning".to_string(),
            });
        
        strategies.insert("TransceiverError".to_string(),
            RecoveryStrategy::RetryOperation {
                max_attempts: 3,
                delay_ms: 1000,
            });
        
        strategies
    }

    fn current_time_ms() -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis() as u64
    }
}

/// Error report structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ErrorReport {
    pub report_timestamp: u64,
    pub system_uptime_ms: u64,
    pub total_errors: usize,
    pub unresolved_errors: usize,
    pub critical_errors: usize,
    pub error_type_distribution: HashMap<String, usize>,
    pub error_statistics: HashMap<String, ErrorStatistics>,
    pub recent_errors: Vec<DiagnosticInfo>,
}

impl fmt::Display for PositioningError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PositioningError::InsufficientAnchors { available, required, anchor_ids } => {
                write!(f, "Insufficient anchors: {} available ({}), {} required", 
                       available, 
                       anchor_ids.iter().map(|id| id.to_string()).collect::<Vec<_>>().join(", "),
                       required)
            }
            PositioningError::AnchorTimeout { anchor_id, last_seen_ms, timeout_threshold_ms } => {
                write!(f, "Anchor {} timeout: last seen {} ms ago (threshold: {} ms)", 
                       anchor_id, 
                       SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default().as_millis() as u64 - last_seen_ms,
                       timeout_threshold_ms)
            }
            PositioningError::DegenerateGeometry { condition_number, geometry_type, .. } => {
                write!(f, "Degenerate geometry ({:?}): condition number {:.2}", geometry_type, condition_number)
            }
            PositioningError::ComputationFailure { operation, error_code, context } => {
                write!(f, "Computation failure in {}: {:?} ({})", operation, error_code, context)
            }
            PositioningError::TransceiverError { transceiver_id, error_type, retry_count } => {
                write!(f, "Transceiver {} error: {:?} (retries: {})", transceiver_id, error_type, retry_count)
            }
            // Add other error display implementations as needed
            _ => write!(f, "{:?}", self),
        }
    }
}

impl std::error::Error for PositioningError {}

impl Default for ErrorReporter {
    fn default() -> Self {
        Self::new()
    }
}