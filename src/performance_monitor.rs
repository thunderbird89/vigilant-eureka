use nalgebra::Vector3;
use crate::accuracy_validation::{AccuracyValidator, PositionError, AccuracyStatistics};
use std::collections::VecDeque;
use std::time::{Duration, Instant};

/// Performance monitoring for underwater positioning system
pub struct PerformanceMonitor {
    /// Accuracy validator for error calculations
    pub validator: AccuracyValidator,
    /// History of position errors
    error_history: VecDeque<PositionError>,
    /// History of computation times
    timing_history: VecDeque<(String, Duration)>,
    /// Maximum history size
    max_history_size: usize,
    /// Current performance metrics
    current_metrics: PerformanceMetrics,
    /// Performance thresholds for alerts
    thresholds: PerformanceThresholds,
    /// Last update time
    last_update: Instant,
}

/// Performance metrics for underwater positioning
#[derive(Debug, Clone)]
pub struct PerformanceMetrics {
    /// Accuracy statistics
    pub accuracy: AccuracyStatistics,
    /// Sub-meter accuracy rate (0-1)
    pub submeter_accuracy_rate: f64,
    /// Average computation time (milliseconds)
    pub avg_computation_time_ms: f64,
    /// Maximum computation time (milliseconds)
    pub max_computation_time_ms: f64,
    /// Average GDOP value
    pub avg_gdop: f64,
    /// Average number of anchors used
    pub avg_anchor_count: f64,
    /// System uptime (seconds)
    pub uptime_seconds: u64,
    /// Number of measurements processed
    pub measurement_count: usize,
}

/// Performance thresholds for alerts
#[derive(Debug, Clone)]
pub struct PerformanceThresholds {
    /// Minimum acceptable sub-meter accuracy rate (0-1)
    pub min_submeter_accuracy_rate: f64,
    /// Maximum acceptable mean error (meters)
    pub max_mean_error: f64,
    /// Maximum acceptable computation time (milliseconds)
    pub max_computation_time_ms: f64,
    /// Maximum acceptable GDOP value
    pub max_gdop: f64,
}

/// Performance alert with severity level
#[derive(Debug, Clone)]
pub struct PerformanceAlert {
    /// Alert message
    pub message: String,
    /// Alert severity level
    pub severity: AlertSeverity,
    /// Timestamp when alert was generated
    pub timestamp: Instant,
}

/// Alert severity levels
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AlertSeverity {
    /// Informational alert
    Info,
    /// Warning alert
    Warning,
    /// Critical alert
    Critical,
}

impl Default for PerformanceMonitor {
    fn default() -> Self {
        Self {
            validator: AccuracyValidator::new(),
            error_history: VecDeque::new(),
            timing_history: VecDeque::new(),
            max_history_size: 100,
            current_metrics: PerformanceMetrics {
                accuracy: AccuracyStatistics {
                    mean_error: 0.0,
                    std_dev_error: 0.0,
                    error_95_percentile: 0.0,
                    rmse: 0.0,
                    mean_horizontal_error: 0.0,
                    mean_vertical_error: 0.0,
                    max_error: 0.0,
                    min_error: 0.0,
                    sample_count: 0,
                },
                submeter_accuracy_rate: 0.0,
                avg_computation_time_ms: 0.0,
                max_computation_time_ms: 0.0,
                avg_gdop: 0.0,
                avg_anchor_count: 0.0,
                uptime_seconds: 0,
                measurement_count: 0,
            },
            thresholds: PerformanceThresholds {
                min_submeter_accuracy_rate: 0.95,
                max_mean_error: 0.5,
                max_computation_time_ms: 100.0,
                max_gdop: 5.0,
            },
            last_update: Instant::now(),
        }
    }
}

impl PerformanceMonitor {
    /// Create a new performance monitor
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a new performance monitor with custom thresholds
    pub fn with_thresholds(
        min_submeter_accuracy_rate: f64,
        max_mean_error: f64,
        max_computation_time_ms: f64,
        max_gdop: f64,
    ) -> Self {
        let mut monitor = Self::default();
        monitor.thresholds = PerformanceThresholds {
            min_submeter_accuracy_rate,
            max_mean_error,
            max_computation_time_ms,
            max_gdop,
        };
        monitor
    }

    /// Record position error
    pub fn record_position_error(&mut self, error: PositionError) {
        self.error_history.push_back(error);
        
        // Trim history if needed
        while self.error_history.len() > self.max_history_size {
            self.error_history.pop_front();
        }
        
        // Update metrics
        self.update_metrics();
    }

    /// Record computation time
    pub fn record_computation_time(&mut self, operation: &str, duration: Duration) {
        self.timing_history.push_back((operation.to_string(), duration));
        
        // Trim history if needed
        while self.timing_history.len() > self.max_history_size {
            self.timing_history.pop_front();
        }
        
        // Update timing metrics immediately
        let (avg_time_ms, max_time_ms) = self.calculate_timing_statistics();
        self.current_metrics.avg_computation_time_ms = avg_time_ms;
        self.current_metrics.max_computation_time_ms = max_time_ms;
    }

    /// Update performance metrics
    fn update_metrics(&mut self) {
        if self.error_history.is_empty() {
            return;
        }
        
        // Calculate accuracy statistics
        let accuracy = self.calculate_accuracy_statistics();
        
        // Calculate sub-meter accuracy rate
        let submeter_count = self.error_history.iter()
            .filter(|e| e.error_magnitude < 1.0)
            .count();
        
        let submeter_accuracy_rate = submeter_count as f64 / self.error_history.len() as f64;
        
        // Calculate timing statistics
        let (avg_time_ms, max_time_ms) = self.calculate_timing_statistics();
        
        // Calculate GDOP and anchor count statistics
        let (avg_gdop, avg_anchor_count) = self.calculate_gdop_statistics();
        
        // Update metrics
        self.current_metrics = PerformanceMetrics {
            accuracy,
            submeter_accuracy_rate,
            avg_computation_time_ms: avg_time_ms,
            max_computation_time_ms: max_time_ms,
            avg_gdop,
            avg_anchor_count,
            uptime_seconds: self.last_update.elapsed().as_secs(),
            measurement_count: self.error_history.len(),
        };
    }

    /// Calculate accuracy statistics
    fn calculate_accuracy_statistics(&self) -> AccuracyStatistics {
        if self.error_history.is_empty() {
            return AccuracyStatistics {
                mean_error: 0.0,
                std_dev_error: 0.0,
                error_95_percentile: 0.0,
                rmse: 0.0,
                mean_horizontal_error: 0.0,
                mean_vertical_error: 0.0,
                max_error: 0.0,
                min_error: 0.0,
                sample_count: 0,
            };
        }
        
        let n = self.error_history.len();
        
        // Calculate mean error
        let mean_error: f64 = self.error_history.iter()
            .map(|e| e.error_magnitude)
            .sum::<f64>() / n as f64;
        
        // Calculate RMSE
        let rmse: f64 = (self.error_history.iter()
            .map(|e| e.error_magnitude.powi(2))
            .sum::<f64>() / n as f64)
            .sqrt();
        
        // Calculate standard deviation
        let variance: f64 = self.error_history.iter()
            .map(|e| (e.error_magnitude - mean_error).powi(2))
            .sum::<f64>() / n as f64;
        let std_dev_error = variance.sqrt();
        
        // Calculate mean horizontal and vertical errors
        let mean_horizontal_error: f64 = self.error_history.iter()
            .map(|e| e.horizontal_error)
            .sum::<f64>() / n as f64;
        
        let mean_vertical_error: f64 = self.error_history.iter()
            .map(|e| e.vertical_error)
            .sum::<f64>() / n as f64;
        
        // Find min and max errors
        let max_error = self.error_history.iter()
            .map(|e| e.error_magnitude)
            .fold(0.0, f64::max);
        
        let min_error = self.error_history.iter()
            .map(|e| e.error_magnitude)
            .fold(f64::INFINITY, f64::min);
        
        // Calculate 95th percentile error
        let mut sorted_errors: Vec<f64> = self.error_history.iter()
            .map(|e| e.error_magnitude)
            .collect();
        sorted_errors.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        
        let index_95 = (n as f64 * 0.95).ceil() as usize - 1;
        let error_95_percentile = sorted_errors.get(index_95.min(n - 1)).copied().unwrap_or(0.0);
        
        AccuracyStatistics {
            mean_error,
            std_dev_error,
            error_95_percentile,
            rmse,
            mean_horizontal_error,
            mean_vertical_error,
            max_error,
            min_error,
            sample_count: n,
        }
    }

    /// Calculate timing statistics
    fn calculate_timing_statistics(&self) -> (f64, f64) {
        if self.timing_history.is_empty() {
            return (0.0, 0.0);
        }
        
        let n = self.timing_history.len();
        
        // Calculate average time
        let avg_time_ms = self.timing_history.iter()
            .map(|(_, duration)| duration.as_secs_f64() * 1000.0)
            .sum::<f64>() / n as f64;
        
        // Calculate maximum time
        let max_time_ms = self.timing_history.iter()
            .map(|(_, duration)| duration.as_secs_f64() * 1000.0)
            .fold(0.0, f64::max);
        
        (avg_time_ms, max_time_ms)
    }

    /// Calculate GDOP and anchor count statistics
    fn calculate_gdop_statistics(&self) -> (f64, f64) {
        if self.error_history.is_empty() {
            return (0.0, 0.0);
        }
        
        let n = self.error_history.len();
        
        // Calculate average GDOP
        let avg_gdop = self.error_history.iter()
            .map(|e| e.gdop)
            .sum::<f64>() / n as f64;
        
        // Calculate average anchor count
        let avg_anchor_count = self.error_history.iter()
            .map(|e| e.num_anchors as f64)
            .sum::<f64>() / n as f64;
        
        (avg_gdop, avg_anchor_count)
    }

    /// Get current performance metrics
    pub fn get_metrics(&self) -> &PerformanceMetrics {
        &self.current_metrics
    }

    /// Check for performance alerts
    pub fn check_alerts(&self) -> Vec<PerformanceAlert> {
        let mut alerts = Vec::new();
        
        // Check sub-meter accuracy rate
        if self.current_metrics.submeter_accuracy_rate < self.thresholds.min_submeter_accuracy_rate {
            alerts.push(PerformanceAlert {
                message: format!(
                    "Sub-meter accuracy rate ({:.1}%) below threshold ({:.1}%)",
                    self.current_metrics.submeter_accuracy_rate * 100.0,
                    self.thresholds.min_submeter_accuracy_rate * 100.0
                ),
                severity: AlertSeverity::Critical,
                timestamp: Instant::now(),
            });
        }
        
        // Check mean error
        if self.current_metrics.accuracy.mean_error > self.thresholds.max_mean_error {
            alerts.push(PerformanceAlert {
                message: format!(
                    "Mean error ({:.2} m) exceeds threshold ({:.2} m)",
                    self.current_metrics.accuracy.mean_error,
                    self.thresholds.max_mean_error
                ),
                severity: AlertSeverity::Warning,
                timestamp: Instant::now(),
            });
        }
        
        // Check computation time
        if self.current_metrics.avg_computation_time_ms > self.thresholds.max_computation_time_ms {
            alerts.push(PerformanceAlert {
                message: format!(
                    "Average computation time ({:.2} ms) exceeds threshold ({:.2} ms)",
                    self.current_metrics.avg_computation_time_ms,
                    self.thresholds.max_computation_time_ms
                ),
                severity: AlertSeverity::Warning,
                timestamp: Instant::now(),
            });
        }
        
        // Check GDOP
        if self.current_metrics.avg_gdop > self.thresholds.max_gdop {
            alerts.push(PerformanceAlert {
                message: format!(
                    "Average GDOP ({:.2}) exceeds threshold ({:.2})",
                    self.current_metrics.avg_gdop,
                    self.thresholds.max_gdop
                ),
                severity: AlertSeverity::Warning,
                timestamp: Instant::now(),
            });
        }
        
        alerts
    }

    /// Generate performance report
    pub fn generate_report(&self) -> String {
        let metrics = &self.current_metrics;
        
        let mut report = String::new();
        
        report.push_str("=== UNDERWATER POSITIONING PERFORMANCE REPORT ===\n\n");
        
        // System information
        report.push_str(&format!("Uptime: {} seconds\n", metrics.uptime_seconds));
        report.push_str(&format!("Measurements: {}\n", metrics.measurement_count));
        report.push_str("\n");
        
        // Accuracy metrics
        report.push_str("ACCURACY METRICS:\n");
        report.push_str(&format!("Mean Error: {:.3} m\n", metrics.accuracy.mean_error));
        report.push_str(&format!("RMSE: {:.3} m\n", metrics.accuracy.rmse));
        report.push_str(&format!("95% Error: {:.3} m\n", metrics.accuracy.error_95_percentile));
        report.push_str(&format!("Horizontal Error: {:.3} m\n", metrics.accuracy.mean_horizontal_error));
        report.push_str(&format!("Vertical Error: {:.3} m\n", metrics.accuracy.mean_vertical_error));
        report.push_str(&format!("Sub-meter Accuracy Rate: {:.1}%\n", metrics.submeter_accuracy_rate * 100.0));
        report.push_str("\n");
        
        // System metrics
        report.push_str("SYSTEM METRICS:\n");
        report.push_str(&format!("Average GDOP: {:.2}\n", metrics.avg_gdop));
        report.push_str(&format!("Average Anchor Count: {:.1}\n", metrics.avg_anchor_count));
        report.push_str(&format!("Average Computation Time: {:.2} ms\n", metrics.avg_computation_time_ms));
        report.push_str(&format!("Maximum Computation Time: {:.2} ms\n", metrics.max_computation_time_ms));
        report.push_str("\n");
        
        // Alerts
        let alerts = self.check_alerts();
        if !alerts.is_empty() {
            report.push_str("ALERTS:\n");
            for alert in &alerts {
                report.push_str(&format!("[{}] {}\n", 
                                       match alert.severity {
                                           AlertSeverity::Info => "INFO",
                                           AlertSeverity::Warning => "WARNING",
                                           AlertSeverity::Critical => "CRITICAL",
                                       },
                                       alert.message));
            }
            report.push_str("\n");
        }
        
        // Recommendations
        report.push_str("RECOMMENDATIONS:\n");
        
        if metrics.submeter_accuracy_rate < 0.95 {
            report.push_str("- Improve anchor geometry for better GDOP\n");
            report.push_str("- Deploy anchors in tetrahedral configuration when possible\n");
        }
        
        if metrics.avg_gdop > 5.0 {
            report.push_str("- Optimize anchor placement to reduce GDOP\n");
            report.push_str("- Add more anchors for better geometry\n");
        }
        
        if metrics.accuracy.mean_horizontal_error > metrics.accuracy.mean_vertical_error {
            report.push_str("- Improve horizontal anchor distribution\n");
        } else {
            report.push_str("- Add anchors at different depths for better vertical accuracy\n");
        }
        
        if metrics.avg_computation_time_ms > 50.0 {
            report.push_str("- Optimize algorithms for faster computation\n");
            report.push_str("- Consider using optimization cache for repeated calculations\n");
        }
        
        report
    }

    /// Reset performance monitor
    pub fn reset(&mut self) {
        self.error_history.clear();
        self.timing_history.clear();
        self.last_update = Instant::now();
        
        // Reset metrics
        self.current_metrics = PerformanceMetrics {
            accuracy: AccuracyStatistics {
                mean_error: 0.0,
                std_dev_error: 0.0,
                error_95_percentile: 0.0,
                rmse: 0.0,
                mean_horizontal_error: 0.0,
                mean_vertical_error: 0.0,
                max_error: 0.0,
                min_error: 0.0,
                sample_count: 0,
            },
            submeter_accuracy_rate: 0.0,
            avg_computation_time_ms: 0.0,
            max_computation_time_ms: 0.0,
            avg_gdop: 0.0,
            avg_anchor_count: 0.0,
            uptime_seconds: 0,
            measurement_count: 0,
        };
    }

    /// Set performance thresholds
    pub fn set_thresholds(
        &mut self,
        min_submeter_accuracy_rate: f64,
        max_mean_error: f64,
        max_computation_time_ms: f64,
        max_gdop: f64,
    ) {
        self.thresholds = PerformanceThresholds {
            min_submeter_accuracy_rate,
            max_mean_error,
            max_computation_time_ms,
            max_gdop,
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_record_position_error() {
        let mut monitor = PerformanceMonitor::new();
        
        let true_pos = Vector3::new(0.0, 0.0, 0.0);
        let estimated_pos = Vector3::new(0.5, 0.0, 0.0);
        
        let error = PositionError {
            true_position: true_pos,
            estimated_position: estimated_pos,
            error_vector: estimated_pos - true_pos,
            error_magnitude: 0.5,
            horizontal_error: 0.5,
            vertical_error: 0.0,
            gdop: 2.0,
            num_anchors: 4,
        };
        
        monitor.record_position_error(error);
        
        let metrics = monitor.get_metrics();
        assert_eq!(metrics.measurement_count, 1);
        assert_eq!(metrics.accuracy.mean_error, 0.5);
        assert_eq!(metrics.submeter_accuracy_rate, 1.0); // 100% sub-meter
    }
    
    #[test]
    fn test_record_computation_time() {
        let mut monitor = PerformanceMonitor::new();
        
        let duration = Duration::from_millis(50);
        monitor.record_computation_time("test_operation", duration);
        
        let metrics = monitor.get_metrics();
        assert!(metrics.avg_computation_time_ms > 0.0);
        assert!(metrics.max_computation_time_ms > 0.0);
    }
    
    #[test]
    fn test_check_alerts() {
        let mut monitor = PerformanceMonitor::with_thresholds(
            0.95, // min_submeter_accuracy_rate
            0.3,  // max_mean_error
            50.0, // max_computation_time_ms
            3.0,  // max_gdop
        );
        
        // Add error that exceeds thresholds
        let error = PositionError {
            true_position: Vector3::new(0.0, 0.0, 0.0),
            estimated_position: Vector3::new(0.5, 0.0, 0.0),
            error_vector: Vector3::new(0.5, 0.0, 0.0),
            error_magnitude: 0.5, // Exceeds max_mean_error
            horizontal_error: 0.5,
            vertical_error: 0.0,
            gdop: 4.0, // Exceeds max_gdop
            num_anchors: 4,
        };
        
        monitor.record_position_error(error);
        
        // Add computation time that exceeds threshold
        monitor.record_computation_time("test", Duration::from_millis(60)); // Exceeds max_computation_time_ms
        
        // Check alerts
        let alerts = monitor.check_alerts();
        
        // Should have alerts for mean error and GDOP (computation time might not trigger)
        assert!(alerts.len() >= 2);
    }
    
    #[test]
    fn test_reset() {
        let mut monitor = PerformanceMonitor::new();
        
        // Add some data
        let error = PositionError {
            true_position: Vector3::new(0.0, 0.0, 0.0),
            estimated_position: Vector3::new(0.5, 0.0, 0.0),
            error_vector: Vector3::new(0.5, 0.0, 0.0),
            error_magnitude: 0.5,
            horizontal_error: 0.5,
            vertical_error: 0.0,
            gdop: 2.0,
            num_anchors: 4,
        };
        
        monitor.record_position_error(error);
        monitor.record_computation_time("test", Duration::from_millis(50));
        
        // Reset
        monitor.reset();
        
        // Check that metrics are reset
        let metrics = monitor.get_metrics();
        assert_eq!(metrics.measurement_count, 0);
        assert_eq!(metrics.accuracy.mean_error, 0.0);
        assert_eq!(metrics.avg_computation_time_ms, 0.0);
    }
}