use std::time::{Duration, Instant};
use std::collections::HashMap;

/// Performance metrics for timing and resource usage monitoring
#[derive(Debug, Clone)]
pub struct PerformanceMetrics {
    pub computation_time_ms: f64,
    pub memory_usage_bytes: usize,
    pub operation_count: u32,
    pub timestamp: Instant,
}

impl PerformanceMetrics {
    pub fn new() -> Self {
        Self {
            computation_time_ms: 0.0,
            memory_usage_bytes: 0,
            operation_count: 0,
            timestamp: Instant::now(),
        }
    }
}

/// Real-time constraint validation thresholds
#[derive(Debug, Clone)]
pub struct PerformanceConstraints {
    pub max_computation_time_ms: f64,
    pub max_memory_usage_bytes: usize,
    pub max_latency_ms: f64,
    pub min_update_rate_hz: f64,
}

impl PerformanceConstraints {
    pub fn embedded_system() -> Self {
        Self {
            max_computation_time_ms: 100.0,  // 100ms for embedded systems
            max_memory_usage_bytes: 32 * 1024, // 32KB RAM limit
            max_latency_ms: 200.0,           // 200ms total latency requirement
            min_update_rate_hz: 5.0,         // 5Hz minimum update rate
        }
    }

    pub fn real_time_system() -> Self {
        Self {
            max_computation_time_ms: 50.0,   // 50ms for real-time systems
            max_memory_usage_bytes: 16 * 1024, // 16KB RAM limit
            max_latency_ms: 100.0,           // 100ms total latency requirement
            min_update_rate_hz: 10.0,        // 10Hz minimum update rate
        }
    }
}

/// Performance monitoring and profiling system
pub struct PerformanceMonitor {
    pub constraints: PerformanceConstraints,
    metrics_history: Vec<PerformanceMetrics>,
    operation_timers: HashMap<String, Instant>,
    violation_count: u32,
    total_operations: u32,
}

impl PerformanceMonitor {
    pub fn new(constraints: PerformanceConstraints) -> Self {
        Self {
            constraints,
            metrics_history: Vec::new(),
            operation_timers: HashMap::new(),
            violation_count: 0,
            total_operations: 0,
        }
    }

    /// Start timing an operation
    pub fn start_operation(&mut self, operation_name: &str) {
        self.operation_timers.insert(operation_name.to_string(), Instant::now());
    }

    /// End timing an operation and return the duration
    pub fn end_operation(&mut self, operation_name: &str) -> Option<Duration> {
        if let Some(start_time) = self.operation_timers.remove(operation_name) {
            let duration = start_time.elapsed();
            self.total_operations += 1;
            Some(duration)
        } else {
            None
        }
    }

    /// Record performance metrics for an operation
    pub fn record_metrics(&mut self, metrics: PerformanceMetrics) {
        // Update total operations count
        self.total_operations += metrics.operation_count;
        
        // Check for constraint violations
        if metrics.computation_time_ms > self.constraints.max_computation_time_ms {
            self.violation_count += 1;
            eprintln!("WARNING: Computation time violation: {:.2}ms > {:.2}ms", 
                     metrics.computation_time_ms, self.constraints.max_computation_time_ms);
        }

        if metrics.memory_usage_bytes > self.constraints.max_memory_usage_bytes {
            self.violation_count += 1;
            eprintln!("WARNING: Memory usage violation: {} bytes > {} bytes", 
                     metrics.memory_usage_bytes, self.constraints.max_memory_usage_bytes);
        }

        self.metrics_history.push(metrics);

        // Keep only recent metrics to prevent unbounded growth
        if self.metrics_history.len() > 1000 {
            self.metrics_history.drain(0..500); // Remove oldest half
        }
    }

    /// Get current performance statistics
    pub fn get_statistics(&self) -> PerformanceStatistics {
        if self.metrics_history.is_empty() {
            return PerformanceStatistics::default();
        }

        let computation_times: Vec<f64> = self.metrics_history.iter()
            .map(|m| m.computation_time_ms)
            .collect();
        
        let memory_usages: Vec<usize> = self.metrics_history.iter()
            .map(|m| m.memory_usage_bytes)
            .collect();

        let mean_computation_time = computation_times.iter().sum::<f64>() / computation_times.len() as f64;
        let max_computation_time = computation_times.iter().fold(0.0f64, |a, &b| a.max(b));
        let min_computation_time = computation_times.iter().fold(f64::INFINITY, |a, &b| a.min(b));

        let mean_memory_usage = memory_usages.iter().sum::<usize>() / memory_usages.len();
        let max_memory_usage = *memory_usages.iter().max().unwrap_or(&0);
        let min_memory_usage = *memory_usages.iter().min().unwrap_or(&0);

        // Calculate percentiles
        let mut sorted_times = computation_times.clone();
        sorted_times.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let p95_computation_time = if sorted_times.len() > 0 {
            sorted_times[(sorted_times.len() as f64 * 0.95) as usize]
        } else {
            0.0
        };

        PerformanceStatistics {
            total_operations: self.total_operations,
            violation_count: self.violation_count,
            violation_rate: self.violation_count as f64 / self.total_operations.max(1) as f64,
            mean_computation_time_ms: mean_computation_time,
            max_computation_time_ms: max_computation_time,
            min_computation_time_ms: min_computation_time,
            p95_computation_time_ms: p95_computation_time,
            mean_memory_usage_bytes: mean_memory_usage,
            max_memory_usage_bytes: max_memory_usage,
            min_memory_usage_bytes: min_memory_usage,
            constraints: self.constraints.clone(),
        }
    }

    /// Check if current performance meets real-time constraints
    pub fn validate_real_time_constraints(&self) -> ConstraintValidationResult {
        let stats = self.get_statistics();
        let mut violations = Vec::new();

        if stats.max_computation_time_ms > self.constraints.max_computation_time_ms {
            violations.push(format!("Computation time: {:.2}ms > {:.2}ms", 
                                   stats.max_computation_time_ms, self.constraints.max_computation_time_ms));
        }

        if stats.max_memory_usage_bytes > self.constraints.max_memory_usage_bytes {
            violations.push(format!("Memory usage: {} bytes > {} bytes", 
                                   stats.max_memory_usage_bytes, self.constraints.max_memory_usage_bytes));
        }

        // Check update rate (based on recent operations)
        let recent_operations = self.metrics_history.iter()
            .rev()
            .take(10)
            .collect::<Vec<_>>();

        if recent_operations.len() >= 2 {
            let time_span = recent_operations.first().unwrap().timestamp
                .duration_since(recent_operations.last().unwrap().timestamp);
            let actual_rate_hz = (recent_operations.len() - 1) as f64 / time_span.as_secs_f64();
            
            if actual_rate_hz < self.constraints.min_update_rate_hz {
                violations.push(format!("Update rate: {:.2}Hz < {:.2}Hz", 
                                       actual_rate_hz, self.constraints.min_update_rate_hz));
            }
        }

        ConstraintValidationResult {
            is_valid: violations.is_empty(),
            violations,
            statistics: stats,
        }
    }

    /// Generate performance report for embedded deployment
    pub fn generate_embedded_report(&self) -> String {
        let stats = self.get_statistics();
        let validation = self.validate_real_time_constraints();

        let mut report = String::new();
        report.push_str("=== EMBEDDED SYSTEM PERFORMANCE REPORT ===\n\n");

        // Overall status
        if validation.is_valid {
            report.push_str("✓ SYSTEM MEETS ALL REAL-TIME CONSTRAINTS\n\n");
        } else {
            report.push_str("⚠ CONSTRAINT VIOLATIONS DETECTED\n\n");
            for violation in &validation.violations {
                report.push_str(&format!("  - {}\n", violation));
            }
            report.push_str("\n");
        }

        // Performance statistics
        report.push_str("PERFORMANCE STATISTICS:\n");
        report.push_str(&format!("  Total operations: {}\n", stats.total_operations));
        report.push_str(&format!("  Violation rate: {:.2}%\n", stats.violation_rate * 100.0));
        report.push_str(&format!("  Mean computation time: {:.2} ms\n", stats.mean_computation_time_ms));
        report.push_str(&format!("  Max computation time: {:.2} ms\n", stats.max_computation_time_ms));
        report.push_str(&format!("  95th percentile time: {:.2} ms\n", stats.p95_computation_time_ms));
        report.push_str(&format!("  Mean memory usage: {} bytes ({:.1} KB)\n", 
                                stats.mean_memory_usage_bytes, stats.mean_memory_usage_bytes as f64 / 1024.0));
        report.push_str(&format!("  Max memory usage: {} bytes ({:.1} KB)\n", 
                                stats.max_memory_usage_bytes, stats.max_memory_usage_bytes as f64 / 1024.0));

        // Constraint limits
        report.push_str("\nCONSTRAINT LIMITS:\n");
        report.push_str(&format!("  Max computation time: {:.2} ms\n", stats.constraints.max_computation_time_ms));
        report.push_str(&format!("  Max memory usage: {} bytes ({:.1} KB)\n", 
                                stats.constraints.max_memory_usage_bytes, 
                                stats.constraints.max_memory_usage_bytes as f64 / 1024.0));
        report.push_str(&format!("  Max latency: {:.2} ms\n", stats.constraints.max_latency_ms));
        report.push_str(&format!("  Min update rate: {:.2} Hz\n", stats.constraints.min_update_rate_hz));

        // Recommendations
        report.push_str("\nRECOMMENDATIONS:\n");
        if stats.max_computation_time_ms > stats.constraints.max_computation_time_ms * 0.8 {
            report.push_str("  - Consider algorithm optimization to reduce computation time\n");
        }
        if stats.max_memory_usage_bytes > stats.constraints.max_memory_usage_bytes / 2 {
            report.push_str("  - Monitor memory usage - approaching limit\n");
        }
        if stats.violation_rate > 0.05 {
            report.push_str("  - High violation rate - review system configuration\n");
        }
        if validation.is_valid && stats.violation_rate == 0.0 {
            report.push_str("  - System performance is optimal for embedded deployment\n");
        }

        report
    }

    /// Clear all metrics history (useful for testing)
    pub fn clear_history(&mut self) {
        self.metrics_history.clear();
        self.operation_timers.clear();
        self.violation_count = 0;
        self.total_operations = 0;
    }
}

/// Performance statistics summary
#[derive(Debug, Clone)]
pub struct PerformanceStatistics {
    pub total_operations: u32,
    pub violation_count: u32,
    pub violation_rate: f64,
    pub mean_computation_time_ms: f64,
    pub max_computation_time_ms: f64,
    pub min_computation_time_ms: f64,
    pub p95_computation_time_ms: f64,
    pub mean_memory_usage_bytes: usize,
    pub max_memory_usage_bytes: usize,
    pub min_memory_usage_bytes: usize,
    pub constraints: PerformanceConstraints,
}

impl Default for PerformanceStatistics {
    fn default() -> Self {
        Self {
            total_operations: 0,
            violation_count: 0,
            violation_rate: 0.0,
            mean_computation_time_ms: 0.0,
            max_computation_time_ms: 0.0,
            min_computation_time_ms: 0.0,
            p95_computation_time_ms: 0.0,
            mean_memory_usage_bytes: 0,
            max_memory_usage_bytes: 0,
            min_memory_usage_bytes: 0,
            constraints: PerformanceConstraints::embedded_system(),
        }
    }
}

/// Result of constraint validation
#[derive(Debug)]
pub struct ConstraintValidationResult {
    pub is_valid: bool,
    pub violations: Vec<String>,
    pub statistics: PerformanceStatistics,
}

/// Memory usage tracker for embedded systems
pub struct MemoryTracker {
    baseline_usage: usize,
    peak_usage: usize,
    current_allocations: HashMap<String, usize>,
}

impl MemoryTracker {
    pub fn new() -> Self {
        Self {
            baseline_usage: Self::get_current_memory_usage(),
            peak_usage: 0,
            current_allocations: HashMap::new(),
        }
    }

    /// Track memory allocation for a specific component
    pub fn track_allocation(&mut self, component: &str, size_bytes: usize) {
        self.current_allocations.insert(component.to_string(), size_bytes);
        self.update_peak_usage();
    }

    /// Remove tracking for a component
    pub fn untrack_allocation(&mut self, component: &str) {
        self.current_allocations.remove(component);
    }

    /// Get current total tracked memory usage
    pub fn get_tracked_usage(&self) -> usize {
        self.current_allocations.values().sum::<usize>() + self.baseline_usage
    }

    /// Get peak memory usage
    pub fn get_peak_usage(&self) -> usize {
        self.peak_usage
    }

    /// Update peak usage tracking
    fn update_peak_usage(&mut self) {
        let current = self.get_tracked_usage();
        if current > self.peak_usage {
            self.peak_usage = current;
        }
    }

    /// Get current system memory usage (simplified for embedded systems)
    fn get_current_memory_usage() -> usize {
        // In a real embedded system, this would query the heap allocator
        // For now, we'll use a simple approximation
        use std::mem;
        mem::size_of::<MemoryTracker>() + 1024 // Base overhead
    }

    /// Generate memory usage report
    pub fn generate_report(&self) -> String {
        let mut report = String::new();
        report.push_str("=== MEMORY USAGE REPORT ===\n");
        report.push_str(&format!("Baseline usage: {} bytes\n", self.baseline_usage));
        report.push_str(&format!("Current usage: {} bytes\n", self.get_tracked_usage()));
        report.push_str(&format!("Peak usage: {} bytes\n", self.peak_usage));
        report.push_str("\nComponent breakdown:\n");
        
        for (component, size) in &self.current_allocations {
            report.push_str(&format!("  {}: {} bytes\n", component, size));
        }
        
        report
    }
}

/// Timing utilities for performance measurement
pub struct TimingProfiler {
    start_time: Option<Instant>,
    measurements: Vec<(String, Duration)>,
}

impl TimingProfiler {
    pub fn new() -> Self {
        Self {
            start_time: None,
            measurements: Vec::new(),
        }
    }

    /// Start timing
    pub fn start(&mut self) {
        self.start_time = Some(Instant::now());
    }

    /// Record a checkpoint with a label
    pub fn checkpoint(&mut self, label: &str) {
        if let Some(start) = self.start_time {
            let duration = start.elapsed();
            self.measurements.push((label.to_string(), duration));
        }
    }

    /// Get all measurements
    pub fn get_measurements(&self) -> &[(String, Duration)] {
        &self.measurements
    }

    /// Get total elapsed time
    pub fn total_elapsed(&self) -> Option<Duration> {
        self.start_time.map(|start| start.elapsed())
    }

    /// Clear all measurements
    pub fn clear(&mut self) {
        self.start_time = None;
        self.measurements.clear();
    }

    /// Generate timing report
    pub fn generate_report(&self) -> String {
        let mut report = String::new();
        report.push_str("=== TIMING PROFILE REPORT ===\n");
        
        if let Some(total) = self.total_elapsed() {
            report.push_str(&format!("Total elapsed: {:.2} ms\n\n", total.as_secs_f64() * 1000.0));
        }
        
        report.push_str("Checkpoints:\n");
        for (label, duration) in &self.measurements {
            report.push_str(&format!("  {}: {:.2} ms\n", label, duration.as_secs_f64() * 1000.0));
        }
        
        report
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread;

    #[test]
    fn test_performance_monitor_basic() {
        let constraints = PerformanceConstraints::embedded_system();
        let mut monitor = PerformanceMonitor::new(constraints);

        // Record some metrics
        let metrics1 = PerformanceMetrics {
            computation_time_ms: 50.0,
            memory_usage_bytes: 1024,
            operation_count: 1,
            timestamp: Instant::now(),
        };
        monitor.record_metrics(metrics1);

        let stats = monitor.get_statistics();
        assert_eq!(stats.total_operations, 1);
        assert_eq!(stats.violation_count, 0);
        assert!((stats.mean_computation_time_ms - 50.0).abs() < 1e-6);
    }

    #[test]
    fn test_constraint_violations() {
        let constraints = PerformanceConstraints::embedded_system();
        let mut monitor = PerformanceMonitor::new(constraints);

        // Record metrics that violate constraints
        let metrics = PerformanceMetrics {
            computation_time_ms: 150.0, // Exceeds 100ms limit
            memory_usage_bytes: 64 * 1024, // Exceeds 32KB limit
            operation_count: 1,
            timestamp: Instant::now(),
        };
        monitor.record_metrics(metrics);

        let stats = monitor.get_statistics();
        assert_eq!(stats.violation_count, 2); // Both time and memory violations
        assert!(stats.violation_rate > 0.0);
    }

    #[test]
    fn test_operation_timing() {
        let constraints = PerformanceConstraints::embedded_system();
        let mut monitor = PerformanceMonitor::new(constraints);

        monitor.start_operation("test_op");
        thread::sleep(Duration::from_millis(10));
        let duration = monitor.end_operation("test_op");

        assert!(duration.is_some());
        assert!(duration.unwrap().as_millis() >= 10);
    }

    #[test]
    fn test_memory_tracker() {
        let mut tracker = MemoryTracker::new();
        
        tracker.track_allocation("component1", 1024);
        tracker.track_allocation("component2", 2048);
        
        let usage = tracker.get_tracked_usage();
        assert!(usage >= 3072); // At least the tracked allocations
        
        tracker.untrack_allocation("component1");
        let new_usage = tracker.get_tracked_usage();
        assert!(new_usage < usage);
    }

    #[test]
    fn test_timing_profiler() {
        let mut profiler = TimingProfiler::new();
        
        profiler.start();
        thread::sleep(Duration::from_millis(5));
        profiler.checkpoint("step1");
        thread::sleep(Duration::from_millis(5));
        profiler.checkpoint("step2");
        
        let measurements = profiler.get_measurements();
        assert_eq!(measurements.len(), 2);
        assert!(measurements[0].1.as_millis() >= 5);
        assert!(measurements[1].1.as_millis() >= 10);
    }

    #[test]
    fn test_performance_report_generation() {
        let constraints = PerformanceConstraints::embedded_system();
        let mut monitor = PerformanceMonitor::new(constraints);

        // Add some test metrics
        for i in 0..10 {
            let metrics = PerformanceMetrics {
                computation_time_ms: 30.0 + i as f64,
                memory_usage_bytes: 1000 + i * 100,
                operation_count: 1,
                timestamp: Instant::now(),
            };
            monitor.record_metrics(metrics);
        }

        let report = monitor.generate_embedded_report();
        assert!(report.contains("PERFORMANCE STATISTICS"));
        assert!(report.contains("Total operations: 10"));
        assert!(report.contains("CONSTRAINT LIMITS"));
    }
}