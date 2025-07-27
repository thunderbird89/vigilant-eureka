//! Performance monitoring and optimization utilities for the beacon emulator
//! 
//! This module provides tools for monitoring system performance, resource usage,
//! and implementing optimizations for high-scale beacon deployments.

use std::sync::Arc;
use std::sync::atomic::{AtomicU64, AtomicUsize, Ordering};
use std::time::{Duration, Instant, SystemTime};
use std::collections::HashMap;
use tokio::sync::{RwLock, Mutex};
use serde::{Serialize, Deserialize};
use uuid::Uuid;
use crate::EmulatorError;

/// System performance metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerformanceMetrics {
    /// Total number of messages transmitted across all beacons
    pub total_messages_transmitted: u64,
    /// Total number of message transmission failures
    pub total_transmission_failures: u64,
    /// Average message transmission rate (messages per second)
    pub avg_transmission_rate: f64,
    /// Peak message transmission rate
    pub peak_transmission_rate: f64,
    /// Current memory usage estimate (bytes)
    pub estimated_memory_usage: u64,
    /// Peak memory usage estimate (bytes)
    pub peak_memory_usage: u64,
    /// Number of active beacons
    pub active_beacon_count: usize,
    /// Number of active channels
    pub active_channel_count: usize,
    /// System uptime
    pub uptime: Duration,
    /// CPU usage estimate (0.0 to 1.0)
    pub estimated_cpu_usage: f64,
    /// Message queue depths across all channels
    pub channel_queue_depths: HashMap<String, usize>,
    /// Rate limiting statistics
    pub rate_limiting_stats: RateLimitingStats,
}

/// Rate limiting statistics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RateLimitingStats {
    /// Number of messages rate limited
    pub messages_rate_limited: u64,
    /// Number of collision avoidance interventions
    pub collision_avoidance_count: u64,
    /// Average message delay due to rate limiting (milliseconds)
    pub avg_rate_limit_delay_ms: f64,
}

/// Performance monitor that tracks system metrics
pub struct PerformanceMonitor {
    start_time: Instant,
    metrics: Arc<RwLock<PerformanceMetrics>>,
    message_counter: Arc<AtomicU64>,
    failure_counter: Arc<AtomicU64>,
    rate_samples: Arc<Mutex<Vec<(Instant, u64)>>>,
    memory_tracker: Arc<MemoryTracker>,
    rate_limiter: Arc<RateLimiter>,
}

impl PerformanceMonitor {
    pub fn new() -> Self {
        let start_time = Instant::now();
        let initial_metrics = PerformanceMetrics {
            total_messages_transmitted: 0,
            total_transmission_failures: 0,
            avg_transmission_rate: 0.0,
            peak_transmission_rate: 0.0,
            estimated_memory_usage: 0,
            peak_memory_usage: 0,
            active_beacon_count: 0,
            active_channel_count: 0,
            uptime: Duration::new(0, 0),
            estimated_cpu_usage: 0.0,
            channel_queue_depths: HashMap::new(),
            rate_limiting_stats: RateLimitingStats {
                messages_rate_limited: 0,
                collision_avoidance_count: 0,
                avg_rate_limit_delay_ms: 0.0,
            },
        };

        Self {
            start_time,
            metrics: Arc::new(RwLock::new(initial_metrics)),
            message_counter: Arc::new(AtomicU64::new(0)),
            failure_counter: Arc::new(AtomicU64::new(0)),
            rate_samples: Arc::new(Mutex::new(Vec::new())),
            memory_tracker: Arc::new(MemoryTracker::new()),
            rate_limiter: Arc::new(RateLimiter::new()),
        }
    }

    /// Record a successful message transmission
    pub async fn record_message_transmission(&self) {
        self.message_counter.fetch_add(1, Ordering::Relaxed);
        self.update_transmission_rate().await;
    }

    /// Record a failed message transmission
    pub async fn record_transmission_failure(&self) {
        self.failure_counter.fetch_add(1, Ordering::Relaxed);
    }

    /// Update beacon count
    pub async fn update_beacon_count(&self, active_count: usize) {
        let mut metrics = self.metrics.write().await;
        metrics.active_beacon_count = active_count;
    }

    /// Update channel count
    pub async fn update_channel_count(&self, channel_count: usize) {
        let mut metrics = self.metrics.write().await;
        metrics.active_channel_count = channel_count;
    }

    /// Update channel queue depths
    pub async fn update_channel_queue_depths(&self, queue_depths: HashMap<String, usize>) {
        let mut metrics = self.metrics.write().await;
        metrics.channel_queue_depths = queue_depths;
    }

    /// Get current performance metrics
    pub async fn get_metrics(&self) -> PerformanceMetrics {
        let mut metrics = self.metrics.write().await;
        
        // Update counters
        metrics.total_messages_transmitted = self.message_counter.load(Ordering::Relaxed);
        metrics.total_transmission_failures = self.failure_counter.load(Ordering::Relaxed);
        metrics.uptime = self.start_time.elapsed();
        
        // Update memory usage
        metrics.estimated_memory_usage = self.memory_tracker.get_current_usage().await;
        metrics.peak_memory_usage = self.memory_tracker.get_peak_usage().await;
        
        // Update rate limiting stats
        metrics.rate_limiting_stats = self.rate_limiter.get_stats().await;
        
        // Estimate CPU usage based on message rate and beacon count
        metrics.estimated_cpu_usage = self.estimate_cpu_usage(&metrics).await;
        
        metrics.clone()
    }

    /// Update transmission rate calculations
    async fn update_transmission_rate(&self) {
        let now = Instant::now();
        let current_count = self.message_counter.load(Ordering::Relaxed);
        
        let mut samples = self.rate_samples.lock().await;
        samples.push((now, current_count));
        
        // Keep only samples from the last 60 seconds
        let cutoff = now - Duration::from_secs(60);
        samples.retain(|(time, _)| *time > cutoff);
        
        if samples.len() >= 2 {
            let (oldest_time, oldest_count) = samples[0];
            let (newest_time, newest_count) = samples[samples.len() - 1];
            
            let time_diff = newest_time.duration_since(oldest_time).as_secs_f64();
            let count_diff = newest_count - oldest_count;
            
            if time_diff > 0.0 {
                let current_rate = count_diff as f64 / time_diff;
                
                let mut metrics = self.metrics.write().await;
                metrics.avg_transmission_rate = current_rate;
                
                if current_rate > metrics.peak_transmission_rate {
                    metrics.peak_transmission_rate = current_rate;
                }
            }
        }
    }

    /// Estimate CPU usage based on system activity
    async fn estimate_cpu_usage(&self, metrics: &PerformanceMetrics) -> f64 {
        // Simple heuristic: base CPU usage on message rate and beacon count
        let base_usage = (metrics.active_beacon_count as f64) * 0.01; // 1% per beacon
        let rate_usage = metrics.avg_transmission_rate * 0.001; // 0.1% per msg/sec
        
        (base_usage + rate_usage).min(1.0)
    }

    /// Get rate limiter reference
    pub fn get_rate_limiter(&self) -> Arc<RateLimiter> {
        self.rate_limiter.clone()
    }

    /// Get memory tracker reference
    pub fn get_memory_tracker(&self) -> Arc<MemoryTracker> {
        self.memory_tracker.clone()
    }

    /// Reset all metrics
    pub async fn reset_metrics(&self) {
        self.message_counter.store(0, Ordering::Relaxed);
        self.failure_counter.store(0, Ordering::Relaxed);
        
        let mut samples = self.rate_samples.lock().await;
        samples.clear();
        
        self.memory_tracker.reset().await;
        self.rate_limiter.reset().await;
        
        let mut metrics = self.metrics.write().await;
        *metrics = PerformanceMetrics {
            total_messages_transmitted: 0,
            total_transmission_failures: 0,
            avg_transmission_rate: 0.0,
            peak_transmission_rate: 0.0,
            estimated_memory_usage: 0,
            peak_memory_usage: 0,
            active_beacon_count: 0,
            active_channel_count: 0,
            uptime: Duration::new(0, 0),
            estimated_cpu_usage: 0.0,
            channel_queue_depths: HashMap::new(),
            rate_limiting_stats: RateLimitingStats {
                messages_rate_limited: 0,
                collision_avoidance_count: 0,
                avg_rate_limit_delay_ms: 0.0,
            },
        };
    }
}

/// Memory usage tracker
pub struct MemoryTracker {
    current_usage: Arc<AtomicU64>,
    peak_usage: Arc<AtomicU64>,
    beacon_memory_estimates: Arc<RwLock<HashMap<Uuid, u64>>>,
    channel_memory_estimates: Arc<RwLock<HashMap<String, u64>>>,
}

impl MemoryTracker {
    pub fn new() -> Self {
        Self {
            current_usage: Arc::new(AtomicU64::new(0)),
            peak_usage: Arc::new(AtomicU64::new(0)),
            beacon_memory_estimates: Arc::new(RwLock::new(HashMap::new())),
            channel_memory_estimates: Arc::new(RwLock::new(HashMap::new())),
        }
    }

    /// Estimate memory usage for a beacon
    pub async fn track_beacon_memory(&self, beacon_id: Uuid, estimated_bytes: u64) {
        let mut estimates = self.beacon_memory_estimates.write().await;
        estimates.insert(beacon_id, estimated_bytes);
        self.update_total_usage().await;
    }

    /// Remove beacon from memory tracking
    pub async fn untrack_beacon_memory(&self, beacon_id: Uuid) {
        let mut estimates = self.beacon_memory_estimates.write().await;
        estimates.remove(&beacon_id);
        self.update_total_usage().await;
    }

    /// Estimate memory usage for a channel
    pub async fn track_channel_memory(&self, channel_name: String, estimated_bytes: u64) {
        let mut estimates = self.channel_memory_estimates.write().await;
        estimates.insert(channel_name, estimated_bytes);
        self.update_total_usage().await;
    }

    /// Remove channel from memory tracking
    pub async fn untrack_channel_memory(&self, channel_name: &str) {
        let mut estimates = self.channel_memory_estimates.write().await;
        estimates.remove(channel_name);
        self.update_total_usage().await;
    }

    /// Update total memory usage calculation
    async fn update_total_usage(&self) {
        let beacon_estimates = self.beacon_memory_estimates.read().await;
        let channel_estimates = self.channel_memory_estimates.read().await;
        
        let beacon_total: u64 = beacon_estimates.values().sum();
        let channel_total: u64 = channel_estimates.values().sum();
        let total = beacon_total + channel_total;
        
        self.current_usage.store(total, Ordering::Relaxed);
        
        // Update peak usage
        let current_peak = self.peak_usage.load(Ordering::Relaxed);
        if total > current_peak {
            self.peak_usage.store(total, Ordering::Relaxed);
        }
    }

    /// Get current memory usage estimate
    pub async fn get_current_usage(&self) -> u64 {
        self.current_usage.load(Ordering::Relaxed)
    }

    /// Get peak memory usage estimate
    pub async fn get_peak_usage(&self) -> u64 {
        self.peak_usage.load(Ordering::Relaxed)
    }

    /// Reset memory tracking
    pub async fn reset(&self) {
        self.current_usage.store(0, Ordering::Relaxed);
        self.peak_usage.store(0, Ordering::Relaxed);
        
        let mut beacon_estimates = self.beacon_memory_estimates.write().await;
        beacon_estimates.clear();
        
        let mut channel_estimates = self.channel_memory_estimates.write().await;
        channel_estimates.clear();
    }

    /// Get memory breakdown by component
    pub async fn get_memory_breakdown(&self) -> MemoryBreakdown {
        let beacon_estimates = self.beacon_memory_estimates.read().await;
        let channel_estimates = self.channel_memory_estimates.read().await;
        
        MemoryBreakdown {
            total_usage: self.current_usage.load(Ordering::Relaxed),
            beacon_usage: beacon_estimates.values().sum(),
            channel_usage: channel_estimates.values().sum(),
            beacon_count: beacon_estimates.len(),
            channel_count: channel_estimates.len(),
        }
    }
}

/// Memory usage breakdown
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct MemoryBreakdown {
    pub total_usage: u64,
    pub beacon_usage: u64,
    pub channel_usage: u64,
    pub beacon_count: usize,
    pub channel_count: usize,
}

/// Rate limiter for message transmission
pub struct RateLimiter {
    global_rate_limit: Arc<RwLock<Option<f64>>>, // messages per second
    per_beacon_rate_limit: Arc<RwLock<Option<f64>>>, // messages per second per beacon
    beacon_last_transmission: Arc<RwLock<HashMap<Uuid, Instant>>>,
    channel_last_transmission: Arc<RwLock<HashMap<String, Instant>>>,
    stats: Arc<RwLock<RateLimitingStats>>,
    collision_avoidance_enabled: Arc<RwLock<bool>>,
    collision_window_ms: Arc<RwLock<u64>>,
}

impl RateLimiter {
    pub fn new() -> Self {
        Self {
            global_rate_limit: Arc::new(RwLock::new(None)),
            per_beacon_rate_limit: Arc::new(RwLock::new(None)),
            beacon_last_transmission: Arc::new(RwLock::new(HashMap::new())),
            channel_last_transmission: Arc::new(RwLock::new(HashMap::new())),
            stats: Arc::new(RwLock::new(RateLimitingStats {
                messages_rate_limited: 0,
                collision_avoidance_count: 0,
                avg_rate_limit_delay_ms: 0.0,
            })),
            collision_avoidance_enabled: Arc::new(RwLock::new(true)),
            collision_window_ms: Arc::new(RwLock::new(100)), // 100ms collision window
        }
    }

    /// Set global rate limit (messages per second across all beacons)
    pub async fn set_global_rate_limit(&self, rate: Option<f64>) {
        let mut limit = self.global_rate_limit.write().await;
        *limit = rate;
    }

    /// Set per-beacon rate limit (messages per second per beacon)
    pub async fn set_per_beacon_rate_limit(&self, rate: Option<f64>) {
        let mut limit = self.per_beacon_rate_limit.write().await;
        *limit = rate;
    }

    /// Enable or disable collision avoidance
    pub async fn set_collision_avoidance(&self, enabled: bool) {
        let mut ca_enabled = self.collision_avoidance_enabled.write().await;
        *ca_enabled = enabled;
    }

    /// Set collision avoidance window (milliseconds)
    pub async fn set_collision_window(&self, window_ms: u64) {
        let mut window = self.collision_window_ms.write().await;
        *window = window_ms;
    }

    /// Check if a beacon can transmit now, applying rate limiting and collision avoidance
    pub async fn can_transmit(&self, beacon_id: Uuid, channel_name: &str) -> Result<bool, Duration> {
        let now = Instant::now();
        
        // Check per-beacon rate limit
        if let Some(per_beacon_limit) = *self.per_beacon_rate_limit.read().await {
            let mut beacon_times = self.beacon_last_transmission.write().await;
            if let Some(last_time) = beacon_times.get(&beacon_id) {
                let min_interval = Duration::from_secs_f64(1.0 / per_beacon_limit);
                let elapsed = now.duration_since(*last_time);
                if elapsed < min_interval {
                    let delay = min_interval - elapsed;
                    self.record_rate_limit(delay).await;
                    return Err(delay);
                }
            }
            beacon_times.insert(beacon_id, now);
        }

        // Check global rate limit (simplified - would need more sophisticated tracking)
        if let Some(_global_limit) = *self.global_rate_limit.read().await {
            // For now, just track channel-level timing as a proxy
            let mut channel_times = self.channel_last_transmission.write().await;
            if let Some(last_time) = channel_times.get(channel_name) {
                let min_interval = Duration::from_millis(10); // Minimum 10ms between any transmissions
                let elapsed = now.duration_since(*last_time);
                if elapsed < min_interval {
                    let delay = min_interval - elapsed;
                    self.record_rate_limit(delay).await;
                    return Err(delay);
                }
            }
            channel_times.insert(channel_name.to_string(), now);
        }

        // Check collision avoidance
        if *self.collision_avoidance_enabled.read().await {
            let collision_window = Duration::from_millis(*self.collision_window_ms.read().await);
            let mut channel_times = self.channel_last_transmission.write().await;
            
            if let Some(last_time) = channel_times.get(channel_name) {
                let elapsed = now.duration_since(*last_time);
                if elapsed < collision_window {
                    // Add small random delay to avoid synchronized retries
                    let random_delay = Duration::from_millis(rand::random::<u64>() % 50);
                    let total_delay = collision_window - elapsed + random_delay;
                    self.record_collision_avoidance().await;
                    return Err(total_delay);
                }
            }
            channel_times.insert(channel_name.to_string(), now);
        }

        Ok(true)
    }

    /// Record a rate limiting event
    async fn record_rate_limit(&self, delay: Duration) {
        let mut stats = self.stats.write().await;
        stats.messages_rate_limited += 1;
        
        // Update average delay (simple moving average)
        let new_delay_ms = delay.as_millis() as f64;
        if stats.avg_rate_limit_delay_ms == 0.0 {
            stats.avg_rate_limit_delay_ms = new_delay_ms;
        } else {
            stats.avg_rate_limit_delay_ms = (stats.avg_rate_limit_delay_ms * 0.9) + (new_delay_ms * 0.1);
        }
    }

    /// Record a collision avoidance event
    async fn record_collision_avoidance(&self) {
        let mut stats = self.stats.write().await;
        stats.collision_avoidance_count += 1;
    }

    /// Get rate limiting statistics
    pub async fn get_stats(&self) -> RateLimitingStats {
        self.stats.read().await.clone()
    }

    /// Reset rate limiting statistics
    pub async fn reset(&self) {
        let mut stats = self.stats.write().await;
        *stats = RateLimitingStats {
            messages_rate_limited: 0,
            collision_avoidance_count: 0,
            avg_rate_limit_delay_ms: 0.0,
        };
        
        let mut beacon_times = self.beacon_last_transmission.write().await;
        beacon_times.clear();
        
        let mut channel_times = self.channel_last_transmission.write().await;
        channel_times.clear();
    }
}

/// Performance optimization configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerformanceConfig {
    /// Maximum number of concurrent beacons before warnings
    pub max_concurrent_beacons: usize,
    /// Global message rate limit (messages per second)
    pub global_rate_limit: Option<f64>,
    /// Per-beacon message rate limit (messages per second)
    pub per_beacon_rate_limit: Option<f64>,
    /// Enable collision avoidance
    pub collision_avoidance_enabled: bool,
    /// Collision avoidance window (milliseconds)
    pub collision_window_ms: u64,
    /// Memory usage warning threshold (bytes)
    pub memory_warning_threshold: u64,
    /// CPU usage warning threshold (0.0 to 1.0)
    pub cpu_warning_threshold: f64,
    /// Enable automatic performance optimizations
    pub auto_optimization_enabled: bool,
    /// Message queue size limit per channel
    pub max_message_queue_size: usize,
    /// Enable message queue cleanup
    pub message_queue_cleanup_enabled: bool,
    /// Message queue cleanup interval (seconds)
    pub message_queue_cleanup_interval_s: u64,
}

impl Default for PerformanceConfig {
    fn default() -> Self {
        Self {
            max_concurrent_beacons: 100,
            global_rate_limit: Some(1000.0), // 1000 messages/sec total
            per_beacon_rate_limit: Some(10.0), // 10 messages/sec per beacon
            collision_avoidance_enabled: true,
            collision_window_ms: 100,
            memory_warning_threshold: 100 * 1024 * 1024, // 100MB
            cpu_warning_threshold: 0.8, // 80%
            auto_optimization_enabled: true,
            max_message_queue_size: 10000,
            message_queue_cleanup_enabled: true,
            message_queue_cleanup_interval_s: 300, // 5 minutes
        }
    }
}

/// Performance optimizer that automatically adjusts system parameters
pub struct PerformanceOptimizer {
    config: Arc<RwLock<PerformanceConfig>>,
    monitor: Arc<PerformanceMonitor>,
    last_optimization: Arc<RwLock<Instant>>,
    optimization_interval: Duration,
}

impl PerformanceOptimizer {
    pub fn new(monitor: Arc<PerformanceMonitor>) -> Self {
        Self {
            config: Arc::new(RwLock::new(PerformanceConfig::default())),
            monitor,
            last_optimization: Arc::new(RwLock::new(Instant::now())),
            optimization_interval: Duration::from_secs(30), // Optimize every 30 seconds
        }
    }

    /// Update performance configuration
    pub async fn update_config(&self, config: PerformanceConfig) {
        let mut current_config = self.config.write().await;
        *current_config = config;
        
        // Apply rate limiting configuration
        let rate_limiter = self.monitor.get_rate_limiter();
        rate_limiter.set_global_rate_limit(current_config.global_rate_limit).await;
        rate_limiter.set_per_beacon_rate_limit(current_config.per_beacon_rate_limit).await;
        rate_limiter.set_collision_avoidance(current_config.collision_avoidance_enabled).await;
        rate_limiter.set_collision_window(current_config.collision_window_ms).await;
    }

    /// Get current performance configuration
    pub async fn get_config(&self) -> PerformanceConfig {
        self.config.read().await.clone()
    }

    /// Run automatic performance optimization
    pub async fn optimize_if_needed(&self) -> Result<Vec<String>, EmulatorError> {
        let now = Instant::now();
        let last_opt = *self.last_optimization.read().await;
        
        if now.duration_since(last_opt) < self.optimization_interval {
            return Ok(vec![]);
        }

        let config = self.config.read().await;
        if !config.auto_optimization_enabled {
            return Ok(vec![]);
        }

        let metrics = self.monitor.get_metrics().await;
        let mut optimizations = Vec::new();

        // Check beacon count
        if metrics.active_beacon_count > config.max_concurrent_beacons {
            optimizations.push(format!(
                "Warning: {} active beacons exceeds recommended maximum of {}",
                metrics.active_beacon_count, config.max_concurrent_beacons
            ));
        }

        // Check memory usage
        if metrics.estimated_memory_usage > config.memory_warning_threshold {
            optimizations.push(format!(
                "Warning: Memory usage ({} MB) exceeds threshold ({} MB)",
                metrics.estimated_memory_usage / (1024 * 1024),
                config.memory_warning_threshold / (1024 * 1024)
            ));
        }

        // Check CPU usage
        if metrics.estimated_cpu_usage > config.cpu_warning_threshold {
            optimizations.push(format!(
                "Warning: CPU usage ({:.1}%) exceeds threshold ({:.1}%)",
                metrics.estimated_cpu_usage * 100.0,
                config.cpu_warning_threshold * 100.0
            ));
        }

        // Check transmission rate
        if let Some(global_limit) = config.global_rate_limit {
            if metrics.avg_transmission_rate > global_limit * 0.9 {
                optimizations.push(format!(
                    "Warning: Transmission rate ({:.1} msg/s) approaching global limit ({:.1} msg/s)",
                    metrics.avg_transmission_rate, global_limit
                ));
            }
        }

        // Update last optimization time
        let mut last_opt = self.last_optimization.write().await;
        *last_opt = now;

        Ok(optimizations)
    }

    /// Get performance recommendations
    pub async fn get_recommendations(&self) -> Vec<String> {
        let metrics = self.monitor.get_metrics().await;
        let config = self.config.read().await;
        let mut recommendations = Vec::new();

        // Beacon count recommendations
        if metrics.active_beacon_count > 50 {
            recommendations.push(
                "Consider reducing transmission frequency for better performance with many beacons".to_string()
            );
        }

        // Memory recommendations
        if metrics.estimated_memory_usage > config.memory_warning_threshold / 2 {
            recommendations.push(
                "Consider enabling message queue cleanup to reduce memory usage".to_string()
            );
        }

        // Rate limiting recommendations
        if metrics.rate_limiting_stats.messages_rate_limited > 100 {
            recommendations.push(
                "High rate limiting activity detected - consider adjusting transmission intervals".to_string()
            );
        }

        // Collision avoidance recommendations
        if metrics.rate_limiting_stats.collision_avoidance_count > 50 {
            recommendations.push(
                "High collision avoidance activity - consider spreading beacon transmission times".to_string()
            );
        }

        recommendations
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio::time::{sleep, Duration as TokioDuration};

    #[tokio::test]
    async fn test_performance_monitor_creation() {
        let monitor = PerformanceMonitor::new();
        let metrics = monitor.get_metrics().await;
        
        assert_eq!(metrics.total_messages_transmitted, 0);
        assert_eq!(metrics.total_transmission_failures, 0);
        assert_eq!(metrics.active_beacon_count, 0);
        assert_eq!(metrics.active_channel_count, 0);
    }

    #[tokio::test]
    async fn test_message_transmission_recording() {
        let monitor = PerformanceMonitor::new();
        
        // Record some transmissions
        for _ in 0..10 {
            monitor.record_message_transmission().await;
        }
        
        let metrics = monitor.get_metrics().await;
        assert_eq!(metrics.total_messages_transmitted, 10);
        assert_eq!(metrics.total_transmission_failures, 0);
    }

    #[tokio::test]
    async fn test_transmission_failure_recording() {
        let monitor = PerformanceMonitor::new();
        
        // Record some failures
        for _ in 0..5 {
            monitor.record_transmission_failure().await;
        }
        
        let metrics = monitor.get_metrics().await;
        assert_eq!(metrics.total_transmission_failures, 5);
    }

    #[tokio::test]
    async fn test_beacon_count_tracking() {
        let monitor = PerformanceMonitor::new();
        
        monitor.update_beacon_count(25).await;
        let metrics = monitor.get_metrics().await;
        assert_eq!(metrics.active_beacon_count, 25);
    }

    #[tokio::test]
    async fn test_memory_tracker() {
        let tracker = MemoryTracker::new();
        let beacon_id = Uuid::new_v4();
        
        // Track beacon memory
        tracker.track_beacon_memory(beacon_id, 1024).await;
        assert_eq!(tracker.get_current_usage().await, 1024);
        
        // Track channel memory
        tracker.track_channel_memory("test_channel".to_string(), 2048).await;
        assert_eq!(tracker.get_current_usage().await, 3072);
        
        // Remove beacon
        tracker.untrack_beacon_memory(beacon_id).await;
        assert_eq!(tracker.get_current_usage().await, 2048);
        
        // Get breakdown
        let breakdown = tracker.get_memory_breakdown().await;
        assert_eq!(breakdown.total_usage, 2048);
        assert_eq!(breakdown.beacon_usage, 0);
        assert_eq!(breakdown.channel_usage, 2048);
    }

    #[tokio::test]
    async fn test_rate_limiter() {
        let limiter = RateLimiter::new();
        let beacon_id = Uuid::new_v4();
        
        // Set per-beacon rate limit
        limiter.set_per_beacon_rate_limit(Some(2.0)).await; // 2 messages per second
        
        // First transmission should be allowed
        assert!(limiter.can_transmit(beacon_id, "test_channel").await.is_ok());
        
        // Second transmission immediately should be rate limited
        match limiter.can_transmit(beacon_id, "test_channel").await {
            Err(delay) => {
                assert!(delay > Duration::from_millis(400)); // Should be ~500ms delay
            }
            Ok(_) => panic!("Expected rate limiting"),
        }
    }

    #[tokio::test]
    async fn test_collision_avoidance() {
        let limiter = RateLimiter::new();
        let beacon1 = Uuid::new_v4();
        let beacon2 = Uuid::new_v4();
        
        limiter.set_collision_avoidance(true).await;
        limiter.set_collision_window(50).await; // 50ms window
        
        // First beacon transmits
        assert!(limiter.can_transmit(beacon1, "test_channel").await.is_ok());
        
        // Second beacon should be delayed due to collision avoidance
        match limiter.can_transmit(beacon2, "test_channel").await {
            Err(delay) => {
                assert!(delay > Duration::from_millis(40)); // Should be delayed
            }
            Ok(_) => panic!("Expected collision avoidance delay"),
        }
    }

    #[tokio::test]
    async fn test_performance_optimizer() {
        let monitor = Arc::new(PerformanceMonitor::new());
        let optimizer = PerformanceOptimizer::new(monitor.clone());
        
        // Update beacon count to trigger warning
        monitor.update_beacon_count(150).await; // Above default threshold of 100
        
        // Force optimization by setting last optimization time to past
        {
            let mut last_opt = optimizer.last_optimization.write().await;
            *last_opt = std::time::Instant::now() - std::time::Duration::from_secs(60);
        }
        
        let optimizations = optimizer.optimize_if_needed().await.unwrap();
        assert!(!optimizations.is_empty());
        assert!(optimizations[0].contains("exceeds recommended maximum"));
    }

    #[tokio::test]
    async fn test_performance_config() {
        let monitor = Arc::new(PerformanceMonitor::new());
        let optimizer = PerformanceOptimizer::new(monitor);
        
        let mut config = PerformanceConfig::default();
        config.max_concurrent_beacons = 200;
        config.global_rate_limit = Some(500.0);
        
        optimizer.update_config(config.clone()).await;
        let retrieved_config = optimizer.get_config().await;
        
        assert_eq!(retrieved_config.max_concurrent_beacons, 200);
        assert_eq!(retrieved_config.global_rate_limit, Some(500.0));
    }

    #[tokio::test]
    async fn test_metrics_reset() {
        let monitor = PerformanceMonitor::new();
        
        // Generate some metrics
        for _ in 0..10 {
            monitor.record_message_transmission().await;
        }
        monitor.update_beacon_count(5).await;
        
        let metrics_before = monitor.get_metrics().await;
        assert_eq!(metrics_before.total_messages_transmitted, 10);
        assert_eq!(metrics_before.active_beacon_count, 5);
        
        // Reset metrics
        monitor.reset_metrics().await;
        
        let metrics_after = monitor.get_metrics().await;
        assert_eq!(metrics_after.total_messages_transmitted, 0);
        assert_eq!(metrics_after.active_beacon_count, 0);
    }

    #[tokio::test]
    async fn test_transmission_rate_calculation() {
        let monitor = PerformanceMonitor::new();
        
        // Record transmissions over time
        for _ in 0..10 {
            monitor.record_message_transmission().await;
            sleep(TokioDuration::from_millis(10)).await;
        }
        
        let metrics = monitor.get_metrics().await;
        assert!(metrics.avg_transmission_rate > 0.0);
        assert!(metrics.peak_transmission_rate >= metrics.avg_transmission_rate);
    }
}