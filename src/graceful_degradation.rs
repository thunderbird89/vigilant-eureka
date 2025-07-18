use crate::error_handling::{PositioningError, ErrorReporter, ErrorContext, SystemState, PositioningMode, PerformanceMetrics, EnvironmentalConditions, ErrorSeverity};
use crate::message_parser::{AnchorMessage, GeodeticPosition};
use crate::data_validator::{GeometryQuality};
use std::collections::HashMap;
use std::time::{SystemTime, UNIX_EPOCH};
use serde::{Serialize, Deserialize};
use nalgebra::{Vector3, Matrix3, Matrix4};

/// System health status
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum SystemHealth {
    Optimal,        // All systems functioning normally
    Good,          // Minor issues, full functionality
    Degraded,      // Reduced functionality, acceptable performance
    Poor,          // Significant issues, limited functionality
    Critical,      // System barely functional
    Failed,        // System non-functional
}

/// Positioning capability levels
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PositioningCapability {
    FullPrecision3D { accuracy_estimate_m: f64 },
    Standard3D { accuracy_estimate_m: f64 },
    Reduced2D { accuracy_estimate_m: f64, depth_estimate_m: Option<f64> },
    DeadReckoning { last_known_position: GeodeticPosition, confidence: f64 },
    RangeOnly { nearest_anchor_id: u16, range_m: f64 },
    Unavailable,
}

/// Algorithm selection criteria
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlgorithmSelectionCriteria {
    pub anchor_count: u8,
    pub geometry_quality: GeometryQuality,
    pub signal_quality_avg: f64,
    pub computation_time_budget_ms: f64,
    pub accuracy_requirement_m: f64,
    pub power_budget: PowerBudget,
}

/// Power budget levels
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PowerBudget {
    Unlimited,
    High,
    Medium,
    Low,
    Critical,
}

/// Fallback positioning algorithms
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum PositioningAlgorithm {
    AdvancedTrilateration,      // Full precision with Kalman filtering
    StandardTrilateration,      // Basic least squares
    WeightedTrilateration,      // Quality-weighted least squares
    Trilateration2D,           // 2D positioning with depth estimation
    RangeBasedEstimation,      // Single anchor range + dead reckoning
    DeadReckoning,             // Pure inertial/time-based estimation
    LastKnownPosition,         // Hold last valid position
}

/// System health monitor
pub struct SystemHealthMonitor {
    error_reporter: ErrorReporter,
    health_history: Vec<HealthSnapshot>,
    performance_metrics: PerformanceMetrics,
    anchor_health: HashMap<u16, AnchorHealth>,
    system_start_time: u64,
    last_health_check: u64,
    health_check_interval_ms: u64,
}

/// Health snapshot at a point in time
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HealthSnapshot {
    pub timestamp_ms: u64,
    pub overall_health: SystemHealth,
    pub positioning_capability: PositioningCapability,
    pub active_anchors: u8,
    pub error_count_last_minute: u32,
    pub avg_computation_time_ms: f64,
    pub memory_usage_percent: f64,
    pub power_level_percent: Option<f64>,
}

/// Individual anchor health tracking
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnchorHealth {
    pub anchor_id: u16,
    pub status: AnchorStatus,
    pub signal_quality_history: Vec<u8>,
    pub last_message_time: u64,
    pub consecutive_failures: u32,
    pub total_messages: u64,
    pub reliability_score: f64, // 0.0 to 1.0
}

/// Anchor status
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum AnchorStatus {
    Active,
    Degraded,
    Intermittent,
    Failed,
    Disabled,
}

/// Graceful degradation manager
pub struct GracefulDegradationManager {
    health_monitor: SystemHealthMonitor,
    current_algorithm: PositioningAlgorithm,
    fallback_sequence: Vec<PositioningAlgorithm>,
    last_known_position: Option<GeodeticPosition>,
    position_history: Vec<(u64, GeodeticPosition, f64)>, // timestamp, position, accuracy
    degradation_triggers: DegradationTriggers,
    recovery_conditions: RecoveryConditions,
}

/// Conditions that trigger degradation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DegradationTriggers {
    pub min_anchor_count: u8,
    pub max_computation_time_ms: f64,
    pub min_geometry_quality: GeometryQuality,
    pub max_error_rate_per_minute: u32,
    pub min_signal_quality: u8,
    pub max_position_uncertainty_m: f64,
}

/// Conditions for recovery to higher capability
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecoveryConditions {
    pub stable_period_ms: u64,
    pub min_anchor_count_for_recovery: u8,
    pub min_signal_quality_for_recovery: u8,
    pub max_error_rate_for_recovery: u32,
}

impl SystemHealthMonitor {
    /// Create a new system health monitor
    pub fn new() -> Self {
        Self {
            error_reporter: ErrorReporter::new(),
            health_history: Vec::new(),
            performance_metrics: PerformanceMetrics {
                avg_computation_time_ms: 0.0,
                max_computation_time_ms: 0.0,
                position_update_rate_hz: 0.0,
                accuracy_estimate_m: 0.0,
                successful_positions: 0,
                failed_positions: 0,
            },
            anchor_health: HashMap::new(),
            system_start_time: Self::current_time_ms(),
            last_health_check: Self::current_time_ms(),
            health_check_interval_ms: 1000, // 1 second
        }
    }

    /// Update anchor health based on message
    pub fn update_anchor_health(&mut self, message: &AnchorMessage) {
        let current_time = Self::current_time_ms();
        
        let anchor_health = self.anchor_health.entry(message.anchor_id).or_insert(AnchorHealth {
            anchor_id: message.anchor_id,
            status: AnchorStatus::Active,
            signal_quality_history: Vec::new(),
            last_message_time: current_time,
            consecutive_failures: 0,
            total_messages: 0,
            reliability_score: 1.0,
        });

        // Update message statistics
        anchor_health.last_message_time = current_time;
        anchor_health.total_messages += 1;
        anchor_health.consecutive_failures = 0; // Reset on successful message

        // Update signal quality history
        anchor_health.signal_quality_history.push(message.signal_quality);
        if anchor_health.signal_quality_history.len() > 20 {
            anchor_health.signal_quality_history.remove(0);
        }

        // Calculate reliability score and status using cloned data
        let reliability_score = Self::calculate_reliability_score_static(anchor_health);
        let status = Self::determine_anchor_status_static(anchor_health, current_time);
        
        // Update anchor health
        anchor_health.reliability_score = reliability_score;
        anchor_health.status = status;
    }

    /// Report anchor failure
    pub fn report_anchor_failure(&mut self, anchor_id: u16, error: PositioningError) {
        let current_time = Self::current_time_ms();
        
        if let Some(anchor_health) = self.anchor_health.get_mut(&anchor_id) {
            anchor_health.consecutive_failures += 1;
            let reliability_score = Self::calculate_reliability_score_static(anchor_health);
            let status = Self::determine_anchor_status_static(anchor_health, current_time);
            anchor_health.reliability_score = reliability_score;
            anchor_health.status = status;
        }

        // Report to error system
        let context = self.create_error_context();
        self.error_reporter.report_error(error, context);
    }

    /// Perform periodic health check
    pub fn perform_health_check(&mut self) -> SystemHealth {
        let current_time = Self::current_time_ms();
        
        if current_time - self.last_health_check < self.health_check_interval_ms {
            return self.get_current_health();
        }

        self.last_health_check = current_time;

        // Check anchor timeouts
        self.check_anchor_timeouts(current_time);

        // Calculate overall system health
        let overall_health = self.calculate_overall_health();

        // Create health snapshot
        let snapshot = HealthSnapshot {
            timestamp_ms: current_time,
            overall_health: overall_health.clone(),
            positioning_capability: self.assess_positioning_capability(),
            active_anchors: self.count_active_anchors(),
            error_count_last_minute: self.count_recent_errors(60000),
            avg_computation_time_ms: self.performance_metrics.avg_computation_time_ms,
            memory_usage_percent: self.estimate_memory_usage(),
            power_level_percent: None, // Would be provided by hardware interface
        };

        // Store snapshot
        self.health_history.push(snapshot);
        if self.health_history.len() > 100 {
            self.health_history.remove(0);
        }

        overall_health
    }

    /// Get current system health without full check
    pub fn get_current_health(&self) -> SystemHealth {
        if let Some(latest) = self.health_history.last() {
            latest.overall_health.clone()
        } else {
            SystemHealth::Good // Default for new system
        }
    }

    /// Get health report
    pub fn get_health_report(&self) -> HealthReport {
        let current_time = Self::current_time_ms();
        let uptime = current_time - self.system_start_time;
        
        HealthReport {
            timestamp_ms: current_time,
            system_uptime_ms: uptime,
            overall_health: self.get_current_health(),
            positioning_capability: self.assess_positioning_capability(),
            anchor_health: self.anchor_health.clone(),
            recent_snapshots: self.health_history.iter().rev().take(10).cloned().collect(),
            error_summary: self.error_reporter.generate_error_report(),
            performance_metrics: self.performance_metrics.clone(),
        }
    }

    // Private helper methods

    fn calculate_reliability_score_static(anchor_health: &AnchorHealth) -> f64 {
        if anchor_health.total_messages == 0 {
            return 0.0;
        }

        // Base score from success rate
        let success_rate = 1.0 - (anchor_health.consecutive_failures as f64 / anchor_health.total_messages as f64);
        
        // Adjust for signal quality
        let avg_quality = if anchor_health.signal_quality_history.is_empty() {
            0.0
        } else {
            anchor_health.signal_quality_history.iter().map(|&q| q as f64).sum::<f64>() 
                / anchor_health.signal_quality_history.len() as f64
        };
        let quality_factor = avg_quality / 255.0;

        // Adjust for recency
        let current_time = Self::current_time_ms();
        let time_since_last_message = current_time - anchor_health.last_message_time;
        let recency_factor = if time_since_last_message > 30000 { // 30 seconds
            0.5
        } else {
            1.0
        };

        (success_rate * quality_factor * recency_factor).min(1.0).max(0.0)
    }

    fn calculate_reliability_score(&self, anchor_health: &AnchorHealth) -> f64 {
        Self::calculate_reliability_score_static(anchor_health)
    }

    fn determine_anchor_status(&self, anchor_health: &AnchorHealth) -> AnchorStatus {
        let current_time = Self::current_time_ms();
        Self::determine_anchor_status_static(anchor_health, current_time)
    }

    fn determine_anchor_status_static(anchor_health: &AnchorHealth, current_time: u64) -> AnchorStatus {
        let time_since_last = current_time - anchor_health.last_message_time;

        if anchor_health.consecutive_failures > 10 {
            AnchorStatus::Failed
        } else if time_since_last > 60000 { // 1 minute
            AnchorStatus::Failed
        } else if anchor_health.consecutive_failures > 5 || time_since_last > 30000 {
            AnchorStatus::Intermittent
        } else if anchor_health.reliability_score < 0.7 {
            AnchorStatus::Degraded
        } else {
            AnchorStatus::Active
        }
    }

    fn check_anchor_timeouts(&mut self, current_time: u64) {
        let timeout_threshold = 30000; // 30 seconds
        let mut timeout_errors = Vec::new();
        
        for (anchor_id, anchor_health) in self.anchor_health.iter_mut() {
            let time_since_last = current_time - anchor_health.last_message_time;
            
            if time_since_last > timeout_threshold && anchor_health.status == AnchorStatus::Active {
                anchor_health.status = AnchorStatus::Intermittent;
                
                // Collect timeout error for later reporting
                timeout_errors.push(PositioningError::AnchorTimeout {
                    anchor_id: *anchor_id,
                    last_seen_ms: anchor_health.last_message_time,
                    timeout_threshold_ms: timeout_threshold,
                });
            }
        }
        
        // Report timeout errors
        for error in timeout_errors {
            let context = self.create_error_context();
            self.error_reporter.report_error(error, context);
        }
    }

    fn calculate_overall_health(&self) -> SystemHealth {
        let active_anchors = self.count_active_anchors();
        let recent_errors = self.count_recent_errors(60000);
        let avg_reliability = self.calculate_average_reliability();

        // Determine health based on multiple factors
        if active_anchors == 0 {
            SystemHealth::Failed
        } else if active_anchors < 3 || recent_errors > 20 || avg_reliability < 0.3 {
            SystemHealth::Critical
        } else if active_anchors < 4 || recent_errors > 10 || avg_reliability < 0.6 {
            SystemHealth::Poor
        } else if recent_errors > 5 || avg_reliability < 0.8 {
            SystemHealth::Degraded
        } else if recent_errors > 2 || avg_reliability < 0.9 {
            SystemHealth::Good
        } else {
            SystemHealth::Optimal
        }
    }

    fn assess_positioning_capability(&self) -> PositioningCapability {
        let active_anchors = self.count_active_anchors();
        let avg_reliability = self.calculate_average_reliability();
        
        match active_anchors {
            0 => PositioningCapability::Unavailable,
            1 => {
                if let Some((anchor_id, _)) = self.anchor_health.iter()
                    .find(|(_, health)| health.status == AnchorStatus::Active) {
                    PositioningCapability::RangeOnly {
                        nearest_anchor_id: *anchor_id,
                        range_m: 0.0, // Would be calculated from actual range measurement
                    }
                } else {
                    PositioningCapability::Unavailable
                }
            }
            2 => PositioningCapability::DeadReckoning {
                last_known_position: GeodeticPosition {
                    latitude: 0.0,
                    longitude: 0.0,
                    depth: 0.0,
                },
                confidence: avg_reliability * 0.5, // Reduced confidence with only 2 anchors
            },
            3 => PositioningCapability::Reduced2D {
                accuracy_estimate_m: 5.0 / avg_reliability, // Worse accuracy with poor reliability
                depth_estimate_m: Some(10.0),
            },
            _ => {
                if avg_reliability > 0.9 {
                    PositioningCapability::FullPrecision3D {
                        accuracy_estimate_m: 1.0,
                    }
                } else {
                    PositioningCapability::Standard3D {
                        accuracy_estimate_m: 2.0 / avg_reliability,
                    }
                }
            }
        }
    }

    fn count_active_anchors(&self) -> u8 {
        self.anchor_health.values()
            .filter(|health| health.status == AnchorStatus::Active)
            .count() as u8
    }

    fn count_recent_errors(&self, time_window_ms: u64) -> u32 {
        let current_time = Self::current_time_ms();
        let cutoff_time = current_time - time_window_ms;
        
        self.error_reporter.get_recent_errors(100)
            .iter()
            .filter(|error| error.context.timestamp_ms > cutoff_time)
            .count() as u32
    }

    fn calculate_average_reliability(&self) -> f64 {
        if self.anchor_health.is_empty() {
            return 0.0;
        }

        let total_reliability: f64 = self.anchor_health.values()
            .map(|health| health.reliability_score)
            .sum();
        
        total_reliability / self.anchor_health.len() as f64
    }

    fn estimate_memory_usage(&self) -> f64 {
        // Simplified memory usage estimation
        // In a real system, this would query actual memory usage
        let base_usage = 1024 * 1024; // 1MB base
        let history_usage = self.health_history.len() * 1024; // ~1KB per snapshot
        let anchor_usage = self.anchor_health.len() * 512; // ~512B per anchor
        
        let total_usage = base_usage + history_usage + anchor_usage;
        let total_available = 32 * 1024 * 1024; // 32MB assumed available
        
        (total_usage as f64 / total_available as f64) * 100.0
    }

    fn create_error_context(&self) -> ErrorContext {
        ErrorContext {
            timestamp_ms: Self::current_time_ms(),
            system_state: SystemState {
                positioning_mode: PositioningMode::FullPrecision3D, // Would be determined by current state
                anchor_count: self.count_active_anchors(),
                last_position_update_ms: Self::current_time_ms(),
                computation_load: 0.5, // Would be measured
                memory_usage_bytes: 1024 * 1024, // Would be measured
                uptime_ms: Self::current_time_ms() - self.system_start_time,
            },
            active_anchors: self.anchor_health.keys().cloned().collect(),
            last_successful_position: None, // Would be tracked
            performance_metrics: self.performance_metrics.clone(),
            environmental_conditions: EnvironmentalConditions {
                sound_speed_ms: 1500.0,
                estimated_depth_m: 50.0,
                signal_noise_level: 0.1,
                multipath_detected: false,
                temperature_c: Some(15.0),
                pressure_bar: Some(5.0),
            },
        }
    }

    fn current_time_ms() -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis() as u64
    }
}

impl GracefulDegradationManager {
    /// Create a new graceful degradation manager
    pub fn new() -> Self {
        Self {
            health_monitor: SystemHealthMonitor::new(),
            current_algorithm: PositioningAlgorithm::AdvancedTrilateration,
            fallback_sequence: vec![
                PositioningAlgorithm::AdvancedTrilateration,
                PositioningAlgorithm::StandardTrilateration,
                PositioningAlgorithm::WeightedTrilateration,
                PositioningAlgorithm::Trilateration2D,
                PositioningAlgorithm::RangeBasedEstimation,
                PositioningAlgorithm::DeadReckoning,
                PositioningAlgorithm::LastKnownPosition,
            ],
            last_known_position: None,
            position_history: Vec::new(),
            degradation_triggers: DegradationTriggers {
                min_anchor_count: 3,
                max_computation_time_ms: 100.0,
                min_geometry_quality: GeometryQuality::Acceptable,
                max_error_rate_per_minute: 10,
                min_signal_quality: 60,
                max_position_uncertainty_m: 10.0,
            },
            recovery_conditions: RecoveryConditions {
                stable_period_ms: 30000,
                min_anchor_count_for_recovery: 4,
                min_signal_quality_for_recovery: 80,
                max_error_rate_for_recovery: 2,
            },
        }
    }

    /// Update system with new anchor messages and check for degradation
    pub fn update(&mut self, messages: &[AnchorMessage]) -> DegradationDecision {
        // Update health monitor
        for message in messages {
            self.health_monitor.update_anchor_health(message);
        }

        // Perform health check
        let system_health = self.health_monitor.perform_health_check();

        // Determine if algorithm change is needed
        let selection_criteria = self.create_selection_criteria(messages);
        let recommended_algorithm = self.select_optimal_algorithm(&selection_criteria);

        if recommended_algorithm != self.current_algorithm {
            let decision = DegradationDecision {
                action: DegradationAction::ChangeAlgorithm {
                    from: self.current_algorithm.clone(),
                    to: recommended_algorithm.clone(),
                    reason: self.determine_change_reason(&selection_criteria),
                },
                system_health,
                positioning_capability: self.health_monitor.assess_positioning_capability(),
                confidence: self.calculate_decision_confidence(&selection_criteria),
            };

            self.current_algorithm = recommended_algorithm;
            decision
        } else {
            DegradationDecision {
                action: DegradationAction::NoChange,
                system_health,
                positioning_capability: self.health_monitor.assess_positioning_capability(),
                confidence: 1.0,
            }
        }
    }

    /// Get current system status
    pub fn get_status(&self) -> SystemStatus {
        SystemStatus {
            current_algorithm: self.current_algorithm.clone(),
            system_health: self.health_monitor.get_current_health(),
            positioning_capability: self.health_monitor.assess_positioning_capability(),
            active_anchors: self.health_monitor.count_active_anchors(),
            last_known_position: self.last_known_position.clone(),
            health_report: self.health_monitor.get_health_report(),
        }
    }

    /// Force algorithm change (for testing or manual override)
    pub fn force_algorithm_change(&mut self, algorithm: PositioningAlgorithm) {
        self.current_algorithm = algorithm;
    }

    /// Update position history
    pub fn update_position_history(&mut self, position: GeodeticPosition, accuracy: f64) {
        let current_time = SystemHealthMonitor::current_time_ms();
        self.position_history.push((current_time, position.clone(), accuracy));
        self.last_known_position = Some(position);

        // Keep only recent history
        if self.position_history.len() > 100 {
            self.position_history.remove(0);
        }
    }

    // Private helper methods

    fn create_selection_criteria(&self, messages: &[AnchorMessage]) -> AlgorithmSelectionCriteria {
        let anchor_count = messages.len() as u8;
        let signal_quality_avg = if messages.is_empty() {
            0.0
        } else {
            messages.iter().map(|m| m.signal_quality as f64).sum::<f64>() / messages.len() as f64
        };

        AlgorithmSelectionCriteria {
            anchor_count,
            geometry_quality: self.assess_geometry_quality(messages),
            signal_quality_avg,
            computation_time_budget_ms: 100.0, // Default budget
            accuracy_requirement_m: 2.0, // Default requirement
            power_budget: PowerBudget::Medium, // Default budget
        }
    }

    fn assess_geometry_quality(&self, messages: &[AnchorMessage]) -> GeometryQuality {
        if messages.len() < 3 {
            return GeometryQuality::Degenerate;
        }

        // Simplified geometry assessment
        // In a real implementation, this would calculate GDOP
        match messages.len() {
            3 => GeometryQuality::Poor,
            4 => GeometryQuality::Acceptable,
            5..=6 => GeometryQuality::Good,
            _ => GeometryQuality::Excellent,
        }
    }

    fn select_optimal_algorithm(&self, criteria: &AlgorithmSelectionCriteria) -> PositioningAlgorithm {
        // Algorithm selection logic based on criteria
        match criteria.anchor_count {
            0 => PositioningAlgorithm::LastKnownPosition,
            1 => PositioningAlgorithm::RangeBasedEstimation,
            2 => PositioningAlgorithm::DeadReckoning,
            3 => {
                if criteria.geometry_quality == GeometryQuality::Poor {
                    PositioningAlgorithm::Trilateration2D
                } else {
                    PositioningAlgorithm::WeightedTrilateration
                }
            }
            _ => {
                if criteria.signal_quality_avg > 200.0 && 
                   criteria.geometry_quality == GeometryQuality::Excellent &&
                   criteria.power_budget != PowerBudget::Critical {
                    PositioningAlgorithm::AdvancedTrilateration
                } else if criteria.geometry_quality >= GeometryQuality::Good {
                    PositioningAlgorithm::StandardTrilateration
                } else {
                    PositioningAlgorithm::WeightedTrilateration
                }
            }
        }
    }

    fn determine_change_reason(&self, criteria: &AlgorithmSelectionCriteria) -> String {
        if criteria.anchor_count < self.degradation_triggers.min_anchor_count {
            format!("Insufficient anchors: {} < {}", criteria.anchor_count, self.degradation_triggers.min_anchor_count)
        } else if criteria.signal_quality_avg < self.degradation_triggers.min_signal_quality as f64 {
            format!("Poor signal quality: {:.1} < {}", criteria.signal_quality_avg, self.degradation_triggers.min_signal_quality)
        } else if criteria.geometry_quality < self.degradation_triggers.min_geometry_quality {
            format!("Poor geometry: {:?} < {:?}", criteria.geometry_quality, self.degradation_triggers.min_geometry_quality)
        } else {
            "Optimization for current conditions".to_string()
        }
    }

    fn calculate_decision_confidence(&self, criteria: &AlgorithmSelectionCriteria) -> f64 {
        let mut confidence: f64 = 1.0;

        // Reduce confidence based on various factors
        if criteria.anchor_count < 4 {
            confidence *= 0.8;
        }
        if criteria.signal_quality_avg < 150.0 {
            confidence *= 0.9;
        }
        if criteria.geometry_quality < GeometryQuality::Good {
            confidence *= 0.7;
        }

        confidence.max(0.1).min(1.0)
    }
}

/// Decision made by degradation manager
#[derive(Debug, Clone)]
pub struct DegradationDecision {
    pub action: DegradationAction,
    pub system_health: SystemHealth,
    pub positioning_capability: PositioningCapability,
    pub confidence: f64,
}

/// Actions that can be taken for degradation
#[derive(Debug, Clone)]
pub enum DegradationAction {
    NoChange,
    ChangeAlgorithm {
        from: PositioningAlgorithm,
        to: PositioningAlgorithm,
        reason: String,
    },
    DisableAnchor {
        anchor_id: u16,
        reason: String,
    },
    EnableFallbackMode {
        mode: PositioningMode,
        reason: String,
    },
}

/// Current system status
#[derive(Debug, Clone)]
pub struct SystemStatus {
    pub current_algorithm: PositioningAlgorithm,
    pub system_health: SystemHealth,
    pub positioning_capability: PositioningCapability,
    pub active_anchors: u8,
    pub last_known_position: Option<GeodeticPosition>,
    pub health_report: HealthReport,
}

/// Comprehensive health report
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HealthReport {
    pub timestamp_ms: u64,
    pub system_uptime_ms: u64,
    pub overall_health: SystemHealth,
    pub positioning_capability: PositioningCapability,
    pub anchor_health: HashMap<u16, AnchorHealth>,
    pub recent_snapshots: Vec<HealthSnapshot>,
    pub error_summary: crate::error_handling::ErrorReport,
    pub performance_metrics: PerformanceMetrics,
}

impl Default for SystemHealthMonitor {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for GracefulDegradationManager {
    fn default() -> Self {
        Self::new()
    }
}