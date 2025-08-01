// System reliability monitoring and reporting for beacon systems
// Implements comprehensive reliability metrics, trend analysis, and predictive maintenance

use std::time::{Duration, SystemTime, Instant};
use std::collections::{HashMap, VecDeque};
use serde::{Serialize, Deserialize};

use crate::{
    BeaconError, HardwareComponent, ErrorSeverity,
    EnvironmentalStats, HardwareMonitorStats
};

/// Reliability monitoring errors
#[derive(Debug, Clone, PartialEq)]
pub enum ReliabilityError {
    InsufficientData { metric: String, required_samples: usize, available: usize },
    CalculationFailed { metric: String, reason: String },
    ThresholdViolation { metric: String, value: f64, threshold: f64 },
    PredictionFailed { reason: String },
    ReportGenerationFailed { reason: String },
}

impl std::fmt::Display for ReliabilityError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ReliabilityError::InsufficientData { metric, required_samples, available } => {
                write!(f, "Insufficient data for {}: need {} samples, have {}", metric, required_samples, available)
            }
            ReliabilityError::CalculationFailed { metric, reason } => {
                write!(f, "Calculation failed for {}: {}", metric, reason)
            }
            ReliabilityError::ThresholdViolation { metric, value, threshold } => {
                write!(f, "Threshold violation for {}: {:.3} exceeds {:.3}", metric, value, threshold)
            }
            ReliabilityError::PredictionFailed { reason } => {
                write!(f, "Prediction failed: {}", reason)
            }
            ReliabilityError::ReportGenerationFailed { reason } => {
                write!(f, "Report generation failed: {}", reason)
            }
        }
    }
}

impl std::error::Error for ReliabilityError {}

/// System reliability metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReliabilityMetrics {
    pub overall_availability: f64,        // Percentage of time system is operational
    pub mean_time_between_failures: Duration,  // MTBF
    pub mean_time_to_repair: Duration,     // MTTR
    pub failure_rate: f64,                 // Failures per hour
    pub component_reliability: HashMap<String, ComponentReliability>,
    pub system_uptime: Duration,
    pub total_failures: u32,
    pub critical_failures: u32,
    pub recovery_success_rate: f64,
    pub performance_degradation_events: u32,
    pub timestamp: SystemTime,
}

/// Component-specific reliability metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComponentReliability {
    pub availability: f64,
    pub failure_rate: f64,
    pub mtbf: Duration,
    pub mttr: Duration,
    pub total_failures: u32,
    pub successful_recoveries: u32,
    pub performance_score: f64,  // 0.0 to 1.0
    pub health_trend: HealthTrend,
}

/// Health trend analysis
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum HealthTrend {
    Improving,
    Stable,
    Degrading,
    Critical,
    Unknown,
}

/// Reliability thresholds for alerting
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReliabilityThresholds {
    pub min_availability: f64,
    pub max_failure_rate: f64,
    pub max_mttr_hours: f64,
    pub min_recovery_success_rate: f64,
    pub max_performance_degradation_events: u32,
    pub component_thresholds: HashMap<String, ComponentThresholds>,
}

/// Component-specific thresholds
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComponentThresholds {
    pub min_availability: f64,
    pub max_failure_rate: f64,
    pub max_mttr_hours: f64,
    pub min_performance_score: f64,
}

impl Default for ReliabilityThresholds {
    fn default() -> Self {
        let mut component_thresholds = HashMap::new();
        
        // GPS thresholds
        component_thresholds.insert("gps".to_string(), ComponentThresholds {
            min_availability: 0.95,  // 95% availability
            max_failure_rate: 0.1,   // 0.1 failures per hour
            max_mttr_hours: 0.5,     // 30 minutes max repair time
            min_performance_score: 0.8,
        });

        // Transceiver thresholds
        component_thresholds.insert("transceiver".to_string(), ComponentThresholds {
            min_availability: 0.98,  // 98% availability
            max_failure_rate: 0.05,  // 0.05 failures per hour
            max_mttr_hours: 0.25,    // 15 minutes max repair time
            min_performance_score: 0.9,
        });

        // Power management thresholds
        component_thresholds.insert("power".to_string(), ComponentThresholds {
            min_availability: 0.99,  // 99% availability
            max_failure_rate: 0.02,  // 0.02 failures per hour
            max_mttr_hours: 0.1,     // 6 minutes max repair time
            min_performance_score: 0.95,
        });

        // Communication thresholds
        component_thresholds.insert("communication".to_string(), ComponentThresholds {
            min_availability: 0.85,  // 85% availability (more tolerant due to network issues)
            max_failure_rate: 0.5,   // 0.5 failures per hour
            max_mttr_hours: 2.0,     // 2 hours max repair time
            min_performance_score: 0.7,
        });

        Self {
            min_availability: 0.95,
            max_failure_rate: 0.2,
            max_mttr_hours: 1.0,
            min_recovery_success_rate: 0.8,
            max_performance_degradation_events: 10,
            component_thresholds,
        }
    }
}

/// Failure event record
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FailureEvent {
    pub timestamp: SystemTime,
    pub component: String,
    pub failure_type: String,
    pub severity: ErrorSeverity,
    pub duration: Option<Duration>,
    pub recovery_successful: bool,
    pub recovery_time: Option<Duration>,
    pub impact_description: String,
    pub root_cause: Option<String>,
}

/// Performance degradation event
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DegradationEvent {
    pub timestamp: SystemTime,
    pub component: String,
    pub metric: String,
    pub baseline_value: f64,
    pub degraded_value: f64,
    pub degradation_percentage: f64,
    pub duration: Option<Duration>,
    pub recovery_time: Option<Duration>,
}

/// Predictive maintenance recommendation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MaintenanceRecommendation {
    pub component: String,
    pub recommendation_type: MaintenanceType,
    pub urgency: MaintenanceUrgency,
    pub predicted_failure_time: Option<SystemTime>,
    pub confidence: f64,  // 0.0 to 1.0
    pub description: String,
    pub estimated_downtime: Duration,
}

/// Types of maintenance recommendations
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum MaintenanceType {
    Calibration,
    ComponentReplacement,
    SoftwareUpdate,
    ConfigurationAdjustment,
    PreventiveMaintenance,
    EmergencyRepair,
}

/// Urgency levels for maintenance
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum MaintenanceUrgency {
    Low,      // Can wait for scheduled maintenance
    Medium,   // Should be addressed within a week
    High,     // Should be addressed within 24 hours
    Critical, // Immediate attention required
}

/// Comprehensive reliability report
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReliabilityReport {
    pub report_id: String,
    pub generation_time: SystemTime,
    pub reporting_period: Duration,
    pub overall_metrics: ReliabilityMetrics,
    pub component_analysis: HashMap<String, ComponentAnalysis>,
    pub failure_analysis: FailureAnalysis,
    pub trend_analysis: TrendAnalysis,
    pub maintenance_recommendations: Vec<MaintenanceRecommendation>,
    pub threshold_violations: Vec<ThresholdViolation>,
    pub executive_summary: String,
}

/// Component analysis details
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ComponentAnalysis {
    pub reliability: ComponentReliability,
    pub recent_failures: Vec<FailureEvent>,
    pub performance_trends: Vec<(SystemTime, f64)>,
    pub maintenance_history: Vec<MaintenanceEvent>,
    pub health_assessment: String,
    pub recommendations: Vec<String>,
}

/// Maintenance event record
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MaintenanceEvent {
    pub timestamp: SystemTime,
    pub maintenance_type: MaintenanceType,
    pub duration: Duration,
    pub success: bool,
    pub notes: String,
}

/// Failure analysis summary
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FailureAnalysis {
    pub total_failures: u32,
    pub failure_categories: HashMap<String, u32>,
    pub most_common_failures: Vec<(String, u32)>,
    pub failure_rate_trend: Vec<(SystemTime, f64)>,
    pub root_cause_analysis: HashMap<String, u32>,
}

/// Trend analysis results
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrendAnalysis {
    pub availability_trend: Vec<(SystemTime, f64)>,
    pub performance_trend: Vec<(SystemTime, f64)>,
    pub failure_rate_trend: Vec<(SystemTime, f64)>,
    pub recovery_time_trend: Vec<(SystemTime, f64)>,
    pub predicted_metrics: PredictedMetrics,
}

/// Predicted future metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PredictedMetrics {
    pub predicted_availability_7d: f64,
    pub predicted_failure_rate_7d: f64,
    pub predicted_mtbf_7d: Duration,
    pub confidence_interval: (f64, f64),
}

/// Threshold violation record
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ThresholdViolation {
    pub metric: String,
    pub component: Option<String>,
    pub current_value: f64,
    pub threshold_value: f64,
    pub violation_severity: ErrorSeverity,
    pub timestamp: SystemTime,
    pub duration: Option<Duration>,
}

/// Main reliability monitoring system
pub struct ReliabilityMonitor {
    thresholds: ReliabilityThresholds,
    failure_history: VecDeque<FailureEvent>,
    degradation_history: VecDeque<DegradationEvent>,
    maintenance_history: VecDeque<MaintenanceEvent>,
    metrics_history: VecDeque<(SystemTime, ReliabilityMetrics)>,
    component_performance: HashMap<String, VecDeque<(SystemTime, f64)>>,
    system_start_time: SystemTime,
    last_report_time: Option<SystemTime>,
    monitoring_active: bool,
}

impl ReliabilityMonitor {
    /// Create new reliability monitor
    pub fn new(thresholds: ReliabilityThresholds) -> Self {
        Self {
            thresholds,
            failure_history: VecDeque::with_capacity(1000),
            degradation_history: VecDeque::with_capacity(500),
            maintenance_history: VecDeque::with_capacity(200),
            metrics_history: VecDeque::with_capacity(1000),
            component_performance: HashMap::new(),
            system_start_time: SystemTime::now(),
            last_report_time: None,
            monitoring_active: false,
        }
    }

    /// Start reliability monitoring
    pub fn start_monitoring(&mut self) {
        self.system_start_time = SystemTime::now();
        self.monitoring_active = true;
    }

    /// Stop reliability monitoring
    pub fn stop_monitoring(&mut self) {
        self.monitoring_active = false;
    }

    /// Record a failure event
    pub fn record_failure(&mut self, event: FailureEvent) {
        if !self.monitoring_active {
            return;
        }

        self.failure_history.push_back(event);
        if self.failure_history.len() > 1000 {
            self.failure_history.pop_front();
        }
    }

    /// Record a performance degradation event
    pub fn record_degradation(&mut self, event: DegradationEvent) {
        if !self.monitoring_active {
            return;
        }

        self.degradation_history.push_back(event);
        if self.degradation_history.len() > 500 {
            self.degradation_history.pop_front();
        }
    }

    /// Record a maintenance event
    pub fn record_maintenance(&mut self, event: MaintenanceEvent) {
        self.maintenance_history.push_back(event);
        if self.maintenance_history.len() > 200 {
            self.maintenance_history.pop_front();
        }
    }

    /// Update component performance metrics
    pub fn update_component_performance(&mut self, component: String, performance_score: f64) {
        if !self.monitoring_active {
            return;
        }

        let performance_history = self.component_performance
            .entry(component)
            .or_insert_with(|| VecDeque::with_capacity(100));

        performance_history.push_back((SystemTime::now(), performance_score));
        if performance_history.len() > 100 {
            performance_history.pop_front();
        }
    }

    /// Calculate current reliability metrics
    pub fn calculate_reliability_metrics(&self) -> Result<ReliabilityMetrics, ReliabilityError> {
        if !self.monitoring_active {
            return Err(ReliabilityError::CalculationFailed {
                metric: "overall".to_string(),
                reason: "Monitoring not active".to_string(),
            });
        }

        let now = SystemTime::now();
        let system_uptime = now.duration_since(self.system_start_time)
            .unwrap_or(Duration::from_secs(0));

        // Calculate overall availability
        let total_downtime = self.calculate_total_downtime();
        let overall_availability = if system_uptime.as_secs() > 0 {
            1.0 - (total_downtime.as_secs_f64() / system_uptime.as_secs_f64())
        } else {
            1.0
        };

        // Calculate MTBF and MTTR
        let total_failures = self.failure_history.len() as u32;
        let mtbf = if total_failures > 0 {
            Duration::from_secs_f64(system_uptime.as_secs_f64() / total_failures as f64)
        } else {
            Duration::from_secs(u64::MAX)
        };

        let total_repair_time: Duration = self.failure_history.iter()
            .filter_map(|f| f.recovery_time)
            .sum();
        
        let mttr = if total_failures > 0 {
            Duration::from_secs_f64(total_repair_time.as_secs_f64() / total_failures as f64)
        } else {
            Duration::from_secs(0)
        };

        // Calculate failure rate (failures per hour)
        let failure_rate = if system_uptime.as_secs() > 0 {
            (total_failures as f64) / (system_uptime.as_secs_f64() / 3600.0)
        } else {
            0.0
        };

        // Calculate component reliability
        let component_reliability = self.calculate_component_reliability()?;

        // Calculate recovery success rate
        let successful_recoveries = self.failure_history.iter()
            .filter(|f| f.recovery_successful)
            .count();
        
        let recovery_success_rate = if total_failures > 0 {
            successful_recoveries as f64 / total_failures as f64
        } else {
            1.0
        };

        // Count critical failures
        let critical_failures = self.failure_history.iter()
            .filter(|f| matches!(f.severity, ErrorSeverity::Critical | ErrorSeverity::Fatal))
            .count() as u32;

        // Count performance degradation events
        let performance_degradation_events = self.degradation_history.len() as u32;

        Ok(ReliabilityMetrics {
            overall_availability,
            mean_time_between_failures: mtbf,
            mean_time_to_repair: mttr,
            failure_rate,
            component_reliability,
            system_uptime,
            total_failures,
            critical_failures,
            recovery_success_rate,
            performance_degradation_events,
            timestamp: now,
        })
    }

    /// Calculate component-specific reliability metrics
    fn calculate_component_reliability(&self) -> Result<HashMap<String, ComponentReliability>, ReliabilityError> {
        let mut component_reliability = HashMap::new();
        
        // Get unique components from failure history
        let components: std::collections::HashSet<String> = self.failure_history.iter()
            .map(|f| f.component.clone())
            .collect();

        for component in components {
            let component_failures: Vec<&FailureEvent> = self.failure_history.iter()
                .filter(|f| f.component == component)
                .collect();

            let total_failures = component_failures.len() as u32;
            
            // Calculate component uptime (simplified)
            let component_downtime: Duration = component_failures.iter()
                .filter_map(|f| f.duration)
                .sum();
            
            let system_uptime = SystemTime::now().duration_since(self.system_start_time)
                .unwrap_or(Duration::from_secs(0));
            
            let availability = if system_uptime.as_secs() > 0 {
                1.0 - (component_downtime.as_secs_f64() / system_uptime.as_secs_f64())
            } else {
                1.0
            };

            // Calculate component MTBF and MTTR
            let mtbf = if total_failures > 0 {
                Duration::from_secs_f64(system_uptime.as_secs_f64() / total_failures as f64)
            } else {
                Duration::from_secs(u64::MAX)
            };

            let total_repair_time: Duration = component_failures.iter()
                .filter_map(|f| f.recovery_time)
                .sum();
            
            let mttr = if total_failures > 0 {
                Duration::from_secs_f64(total_repair_time.as_secs_f64() / total_failures as f64)
            } else {
                Duration::from_secs(0)
            };

            let failure_rate = if system_uptime.as_secs() > 0 {
                (total_failures as f64) / (system_uptime.as_secs_f64() / 3600.0)
            } else {
                0.0
            };

            let successful_recoveries = component_failures.iter()
                .filter(|f| f.recovery_successful)
                .count() as u32;

            // Get performance score from recent performance data
            let performance_score = self.component_performance.get(&component)
                .and_then(|history| history.back())
                .map(|(_, score)| *score)
                .unwrap_or(1.0);

            // Determine health trend
            let health_trend = self.calculate_health_trend(&component);

            component_reliability.insert(component, ComponentReliability {
                availability,
                failure_rate,
                mtbf,
                mttr,
                total_failures,
                successful_recoveries,
                performance_score,
                health_trend,
            });
        }

        Ok(component_reliability)
    }

    /// Calculate health trend for a component
    fn calculate_health_trend(&self, component: &str) -> HealthTrend {
        if let Some(performance_history) = self.component_performance.get(component) {
            if performance_history.len() < 3 {
                return HealthTrend::Unknown;
            }

            let recent_scores: Vec<f64> = performance_history.iter()
                .rev()
                .take(5)
                .map(|(_, score)| *score)
                .collect();

            let avg_recent = recent_scores.iter().sum::<f64>() / recent_scores.len() as f64;
            
            let older_scores: Vec<f64> = performance_history.iter()
                .rev()
                .skip(5)
                .take(5)
                .map(|(_, score)| *score)
                .collect();

            if older_scores.is_empty() {
                return HealthTrend::Stable;
            }

            let avg_older = older_scores.iter().sum::<f64>() / older_scores.len() as f64;
            let trend_change = avg_recent - avg_older;

            if trend_change > 0.1 {
                HealthTrend::Improving
            } else if trend_change < -0.1 {
                if avg_recent < 0.5 {
                    HealthTrend::Critical
                } else {
                    HealthTrend::Degrading
                }
            } else {
                HealthTrend::Stable
            }
        } else {
            HealthTrend::Unknown
        }
    }

    /// Calculate total system downtime
    fn calculate_total_downtime(&self) -> Duration {
        self.failure_history.iter()
            .filter_map(|f| f.duration)
            .sum()
    }

    /// Generate comprehensive reliability report
    pub fn generate_reliability_report(&mut self) -> Result<ReliabilityReport, ReliabilityError> {
        let report_id = format!("REL_{}", SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .unwrap_or(Duration::from_secs(0))
            .as_secs());

        let generation_time = SystemTime::now();
        let reporting_period = self.last_report_time
            .map(|last| generation_time.duration_since(last).unwrap_or(Duration::from_secs(0)))
            .unwrap_or_else(|| generation_time.duration_since(self.system_start_time).unwrap_or(Duration::from_secs(0)));

        let overall_metrics = self.calculate_reliability_metrics()?;
        
        // Generate component analysis
        let mut component_analysis = HashMap::new();
        for (component, reliability) in &overall_metrics.component_reliability {
            let recent_failures: Vec<FailureEvent> = self.failure_history.iter()
                .filter(|f| f.component == *component)
                .rev()
                .take(10)
                .cloned()
                .collect();

            let performance_trends: Vec<(SystemTime, f64)> = self.component_performance.get(component)
                .map(|history| history.iter().cloned().collect())
                .unwrap_or_default();

            let maintenance_history: Vec<MaintenanceEvent> = self.maintenance_history.iter()
                .filter(|m| {
                    // Simple matching - in real implementation, would need better component tracking
                    true
                })
                .cloned()
                .collect();

            let health_assessment = self.generate_health_assessment(component, reliability);
            let recommendations = self.generate_component_recommendations(component, reliability);

            component_analysis.insert(component.clone(), ComponentAnalysis {
                reliability: reliability.clone(),
                recent_failures,
                performance_trends,
                maintenance_history,
                health_assessment,
                recommendations,
            });
        }

        // Generate failure analysis
        let failure_analysis = self.generate_failure_analysis();

        // Generate trend analysis
        let trend_analysis = self.generate_trend_analysis()?;

        // Generate maintenance recommendations
        let maintenance_recommendations = self.generate_maintenance_recommendations(&overall_metrics)?;

        // Check for threshold violations
        let threshold_violations = self.check_threshold_violations(&overall_metrics)?;

        // Generate executive summary
        let executive_summary = self.generate_executive_summary(&overall_metrics, &threshold_violations);

        self.last_report_time = Some(generation_time);

        Ok(ReliabilityReport {
            report_id,
            generation_time,
            reporting_period,
            overall_metrics,
            component_analysis,
            failure_analysis,
            trend_analysis,
            maintenance_recommendations,
            threshold_violations,
            executive_summary,
        })
    }

    /// Generate health assessment for a component
    fn generate_health_assessment(&self, component: &str, reliability: &ComponentReliability) -> String {
        match reliability.health_trend {
            HealthTrend::Improving => format!("Component {} is showing improvement with {:.1}% availability", component, reliability.availability * 100.0),
            HealthTrend::Stable => format!("Component {} is operating stably with {:.1}% availability", component, reliability.availability * 100.0),
            HealthTrend::Degrading => format!("Component {} is showing degradation with {:.1}% availability - monitoring recommended", component, reliability.availability * 100.0),
            HealthTrend::Critical => format!("Component {} is in critical condition with {:.1}% availability - immediate attention required", component, reliability.availability * 100.0),
            HealthTrend::Unknown => format!("Component {} health trend unknown - insufficient data", component),
        }
    }

    /// Generate component-specific recommendations
    fn generate_component_recommendations(&self, component: &str, reliability: &ComponentReliability) -> Vec<String> {
        let mut recommendations = Vec::new();

        if reliability.availability < 0.95 {
            recommendations.push(format!("Investigate causes of low availability ({:.1}%)", reliability.availability * 100.0));
        }

        if reliability.failure_rate > 0.1 {
            recommendations.push(format!("High failure rate ({:.3}/hour) - consider preventive maintenance", reliability.failure_rate));
        }

        if reliability.mttr.as_secs() > 3600 {
            recommendations.push("Long repair times - review recovery procedures".to_string());
        }

        if reliability.performance_score < 0.8 {
            recommendations.push(format!("Performance degraded ({:.1}%) - consider calibration", reliability.performance_score * 100.0));
        }

        match reliability.health_trend {
            HealthTrend::Degrading => recommendations.push("Increasing monitoring frequency recommended".to_string()),
            HealthTrend::Critical => recommendations.push("Immediate maintenance required".to_string()),
            _ => {}
        }

        if recommendations.is_empty() {
            recommendations.push("Component operating within normal parameters".to_string());
        }

        recommendations
    }

    /// Generate failure analysis
    fn generate_failure_analysis(&self) -> FailureAnalysis {
        let total_failures = self.failure_history.len() as u32;
        
        let mut failure_categories = HashMap::new();
        let mut root_cause_analysis = HashMap::new();

        for failure in &self.failure_history {
            *failure_categories.entry(failure.failure_type.clone()).or_insert(0) += 1;
            
            if let Some(root_cause) = &failure.root_cause {
                *root_cause_analysis.entry(root_cause.clone()).or_insert(0) += 1;
            }
        }

        let mut most_common_failures: Vec<(String, u32)> = failure_categories.iter()
            .map(|(k, v)| (k.clone(), *v))
            .collect();
        most_common_failures.sort_by(|a, b| b.1.cmp(&a.1));
        most_common_failures.truncate(5);

        // Generate failure rate trend (simplified)
        let failure_rate_trend = vec![(SystemTime::now(), total_failures as f64)];

        FailureAnalysis {
            total_failures,
            failure_categories,
            most_common_failures,
            failure_rate_trend,
            root_cause_analysis,
        }
    }

    /// Generate trend analysis
    fn generate_trend_analysis(&self) -> Result<TrendAnalysis, ReliabilityError> {
        // Simplified trend analysis - in real implementation would use more sophisticated algorithms
        let current_metrics = self.calculate_reliability_metrics()?;
        
        let availability_trend = vec![(SystemTime::now(), current_metrics.overall_availability)];
        let performance_trend = vec![(SystemTime::now(), 0.9)]; // Placeholder
        let failure_rate_trend = vec![(SystemTime::now(), current_metrics.failure_rate)];
        let recovery_time_trend = vec![(SystemTime::now(), current_metrics.mean_time_to_repair.as_secs_f64())];

        let predicted_metrics = PredictedMetrics {
            predicted_availability_7d: current_metrics.overall_availability * 0.98, // Slight degradation prediction
            predicted_failure_rate_7d: current_metrics.failure_rate * 1.1, // Slight increase prediction
            predicted_mtbf_7d: Duration::from_secs((current_metrics.mean_time_between_failures.as_secs() as f64 * 0.95) as u64),
            confidence_interval: (0.8, 0.95),
        };

        Ok(TrendAnalysis {
            availability_trend,
            performance_trend,
            failure_rate_trend,
            recovery_time_trend,
            predicted_metrics,
        })
    }

    /// Generate maintenance recommendations
    fn generate_maintenance_recommendations(&self, metrics: &ReliabilityMetrics) -> Result<Vec<MaintenanceRecommendation>, ReliabilityError> {
        let mut recommendations = Vec::new();

        for (component, reliability) in &metrics.component_reliability {
            if reliability.availability < 0.9 {
                recommendations.push(MaintenanceRecommendation {
                    component: component.clone(),
                    recommendation_type: MaintenanceType::PreventiveMaintenance,
                    urgency: MaintenanceUrgency::High,
                    predicted_failure_time: Some(SystemTime::now() + Duration::from_secs(86400)), // 24 hours
                    confidence: 0.8,
                    description: format!("Low availability ({:.1}%) requires preventive maintenance", reliability.availability * 100.0),
                    estimated_downtime: Duration::from_secs(3600), // 1 hour
                });
            }

            if reliability.performance_score < 0.7 {
                recommendations.push(MaintenanceRecommendation {
                    component: component.clone(),
                    recommendation_type: MaintenanceType::Calibration,
                    urgency: MaintenanceUrgency::Medium,
                    predicted_failure_time: None,
                    confidence: 0.9,
                    description: format!("Performance degraded ({:.1}%) - calibration recommended", reliability.performance_score * 100.0),
                    estimated_downtime: Duration::from_secs(1800), // 30 minutes
                });
            }

            if matches!(reliability.health_trend, HealthTrend::Critical) {
                recommendations.push(MaintenanceRecommendation {
                    component: component.clone(),
                    recommendation_type: MaintenanceType::EmergencyRepair,
                    urgency: MaintenanceUrgency::Critical,
                    predicted_failure_time: Some(SystemTime::now() + Duration::from_secs(3600)), // 1 hour
                    confidence: 0.95,
                    description: "Critical health trend detected - immediate repair required".to_string(),
                    estimated_downtime: Duration::from_secs(7200), // 2 hours
                });
            }
        }

        Ok(recommendations)
    }

    /// Check for threshold violations
    fn check_threshold_violations(&self, metrics: &ReliabilityMetrics) -> Result<Vec<ThresholdViolation>, ReliabilityError> {
        let mut violations = Vec::new();

        // Check overall thresholds
        if metrics.overall_availability < self.thresholds.min_availability {
            violations.push(ThresholdViolation {
                metric: "overall_availability".to_string(),
                component: None,
                current_value: metrics.overall_availability,
                threshold_value: self.thresholds.min_availability,
                violation_severity: ErrorSeverity::Critical,
                timestamp: SystemTime::now(),
                duration: None,
            });
        }

        if metrics.failure_rate > self.thresholds.max_failure_rate {
            violations.push(ThresholdViolation {
                metric: "failure_rate".to_string(),
                component: None,
                current_value: metrics.failure_rate,
                threshold_value: self.thresholds.max_failure_rate,
                violation_severity: ErrorSeverity::Warning,
                timestamp: SystemTime::now(),
                duration: None,
            });
        }

        // Check component thresholds
        for (component, reliability) in &metrics.component_reliability {
            if let Some(component_thresholds) = self.thresholds.component_thresholds.get(component) {
                if reliability.availability < component_thresholds.min_availability {
                    violations.push(ThresholdViolation {
                        metric: "availability".to_string(),
                        component: Some(component.clone()),
                        current_value: reliability.availability,
                        threshold_value: component_thresholds.min_availability,
                        violation_severity: ErrorSeverity::Error,
                        timestamp: SystemTime::now(),
                        duration: None,
                    });
                }

                if reliability.performance_score < component_thresholds.min_performance_score {
                    violations.push(ThresholdViolation {
                        metric: "performance_score".to_string(),
                        component: Some(component.clone()),
                        current_value: reliability.performance_score,
                        threshold_value: component_thresholds.min_performance_score,
                        violation_severity: ErrorSeverity::Warning,
                        timestamp: SystemTime::now(),
                        duration: None,
                    });
                }
            }
        }

        Ok(violations)
    }

    /// Generate executive summary
    fn generate_executive_summary(&self, metrics: &ReliabilityMetrics, violations: &[ThresholdViolation]) -> String {
        let mut summary = String::new();
        
        summary.push_str(&format!("System Reliability Summary:\n"));
        summary.push_str(&format!("- Overall Availability: {:.1}%\n", metrics.overall_availability * 100.0));
        summary.push_str(&format!("- Total Failures: {}\n", metrics.total_failures));
        summary.push_str(&format!("- Critical Failures: {}\n", metrics.critical_failures));
        summary.push_str(&format!("- Recovery Success Rate: {:.1}%\n", metrics.recovery_success_rate * 100.0));
        
        if !violations.is_empty() {
            summary.push_str(&format!("\nThreshold Violations: {}\n", violations.len()));
            for violation in violations.iter().take(3) {
                summary.push_str(&format!("- {}: {:.3} (threshold: {:.3})\n", 
                    violation.metric, violation.current_value, violation.threshold_value));
            }
        }

        if metrics.overall_availability > 0.95 && violations.is_empty() {
            summary.push_str("\nSystem is operating within acceptable parameters.");
        } else {
            summary.push_str("\nSystem requires attention - see detailed analysis for recommendations.");
        }

        summary
    }

    /// Get reliability statistics
    pub fn get_statistics(&self) -> (usize, usize, usize) {
        (
            self.failure_history.len(),
            self.degradation_history.len(),
            self.maintenance_history.len(),
        )
    }

    /// Check if monitoring is active
    pub fn is_monitoring_active(&self) -> bool {
        self.monitoring_active
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_reliability_monitor_creation() {
        let thresholds = ReliabilityThresholds::default();
        let monitor = ReliabilityMonitor::new(thresholds);
        
        assert!(!monitor.is_monitoring_active());
        let (failures, degradations, maintenance) = monitor.get_statistics();
        assert_eq!(failures, 0);
        assert_eq!(degradations, 0);
        assert_eq!(maintenance, 0);
    }

    #[test]
    fn test_failure_recording() {
        let thresholds = ReliabilityThresholds::default();
        let mut monitor = ReliabilityMonitor::new(thresholds);
        monitor.start_monitoring();

        let failure_event = FailureEvent {
            timestamp: SystemTime::now(),
            component: "gps".to_string(),
            failure_type: "signal_lost".to_string(),
            severity: ErrorSeverity::Warning,
            duration: Some(Duration::from_secs(300)),
            recovery_successful: true,
            recovery_time: Some(Duration::from_secs(60)),
            impact_description: "GPS signal lost for 5 minutes".to_string(),
            root_cause: Some("antenna_obstruction".to_string()),
        };

        monitor.record_failure(failure_event);
        
        let (failures, _, _) = monitor.get_statistics();
        assert_eq!(failures, 1);
    }

    #[test]
    fn test_reliability_metrics_calculation() {
        let thresholds = ReliabilityThresholds::default();
        let mut monitor = ReliabilityMonitor::new(thresholds);
        monitor.start_monitoring();

        // Wait a bit to have some uptime
        std::thread::sleep(Duration::from_millis(10));

        let metrics = monitor.calculate_reliability_metrics().unwrap();
        assert!(metrics.overall_availability > 0.0);
        assert!(metrics.system_uptime.as_millis() > 0);
    }
}