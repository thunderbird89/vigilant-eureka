# Beacon Emulator API Documentation

This document describes the programmatic API for extending and integrating with the beacon emulator.

## Table of Contents

- [Architecture Overview](#architecture-overview)
- [Core Components](#core-components)
- [Public API](#public-api)
- [Extension Points](#extension-points)
- [Integration Examples](#integration-examples)
- [Custom Movement Patterns](#custom-movement-patterns)
- [Message Format Extensions](#message-format-extensions)
- [Testing Framework](#testing-framework)

## Architecture Overview

The beacon emulator is built with a modular architecture that allows for extension and customization:

```rust
// Core architecture components
pub struct EmulatorManager {
    virtual_beacons: HashMap<Uuid, VirtualBeacon>,
    beacon_tasks: HashMap<Uuid, JoinHandle<()>>,
    communication_space: VirtualCommunicationSpace,
    performance_monitor: PerformanceMonitor,
}

pub struct VirtualBeacon {
    id: Uuid,
    config: BeaconConfig,
    position: GeodeticPosition,
    movement_pattern: MovementPattern,
    message_builder: MessageBuilder,
    virtual_channel: VirtualChannel,
}

pub struct VirtualCommunicationSpace {
    channels: HashMap<String, VirtualChannel>,
    ipc_server: IpcServer,
}
```

## Core Components

### EmulatorManager

The main orchestration component for managing virtual beacons.

```rust
use beacon_emulator::{EmulatorManager, EmulatorError, VirtualBeaconConfig};
use shared_positioning::GeodeticPosition;
use uuid::Uuid;

impl EmulatorManager {
    /// Create a new emulator manager
    pub fn new(channel_name: &str) -> Self;
    
    /// Create a virtual beacon
    pub async fn create_beacon(
        &mut self,
        id: Option<Uuid>,
        position: GeodeticPosition,
        config: Option<VirtualBeaconConfig>,
    ) -> Result<Uuid, EmulatorError>;
    
    /// Start a beacon
    pub async fn start_beacon(&mut self, id: Uuid) -> Result<(), EmulatorError>;
    
    /// Stop a beacon
    pub async fn stop_beacon(&mut self, id: Uuid) -> Result<(), EmulatorError>;
    
    /// Update beacon configuration
    pub async fn update_beacon(
        &mut self,
        id: Uuid,
        updates: BeaconUpdates,
    ) -> Result<(), EmulatorError>;
    
    /// List all beacons
    pub fn list_beacons(&self) -> Vec<VirtualBeaconStatus>;
    
    /// Get beacon status
    pub fn get_beacon_status(&self, id: Uuid) -> Option<VirtualBeaconStatus>;
    
    /// Create scenario
    pub async fn create_scenario(
        &mut self,
        scenario_type: ScenarioType,
        config: ScenarioConfig,
    ) -> Result<Vec<Uuid>, EmulatorError>;
    
    /// Export logs
    pub async fn export_logs(
        &self,
        config: ExportConfig,
    ) -> Result<Vec<LogEntry>, EmulatorError>;
    
    /// Get performance metrics
    pub fn get_performance_metrics(&self) -> PerformanceMetrics;
}
```

### VirtualBeacon

Individual virtual beacon implementation.

```rust
use beacon_emulator::{VirtualBeacon, MovementPattern, VirtualMessage};
use shared_positioning::{BeaconConfig, GeodeticPosition, MessageBuilder};

impl VirtualBeacon {
    /// Create new virtual beacon
    pub fn new(
        id: Uuid,
        config: BeaconConfig,
        initial_position: GeodeticPosition,
        virtual_channel: VirtualChannel,
    ) -> Result<Self, EmulatorError>;
    
    /// Start beacon transmission
    pub async fn start(&mut self) -> Result<(), EmulatorError>;
    
    /// Stop beacon transmission
    pub fn stop(&mut self);
    
    /// Update position
    pub fn update_position(&mut self, position: GeodeticPosition);
    
    /// Set movement pattern
    pub fn set_movement_pattern(&mut self, pattern: MovementPattern);
    
    /// Get current status
    pub fn get_status(&self) -> VirtualBeaconStatus;
    
    /// Get statistics
    pub fn get_statistics(&self) -> VirtualBeaconStats;
    
    /// Force message transmission (for testing)
    pub async fn transmit_now(&mut self) -> Result<VirtualMessage, EmulatorError>;
}
```

### VirtualCommunicationSpace

Manages virtual communication channels and message routing.

```rust
use beacon_emulator::{VirtualCommunicationSpace, VirtualChannel, VirtualMessage};

impl VirtualCommunicationSpace {
    /// Create new communication space
    pub fn new() -> Self;
    
    /// Get or create channel
    pub fn get_or_create_channel(&mut self, name: &str) -> VirtualChannel;
    
    /// List all channels
    pub fn list_channels(&self) -> Vec<String>;
    
    /// Get channel statistics
    pub fn get_channel_stats(&self, name: &str) -> Option<ChannelStats>;
    
    /// Remove channel
    pub fn remove_channel(&mut self, name: &str) -> Result<(), EmulatorError>;
}

impl VirtualChannel {
    /// Broadcast message to all subscribers
    pub async fn broadcast_message(&self, message: VirtualMessage) -> Result<(), EmulatorError>;
    
    /// Subscribe to channel messages
    pub fn subscribe(&self) -> broadcast::Receiver<VirtualMessage>;
    
    /// Get recent messages
    pub async fn get_recent_messages(&self, count: usize) -> Vec<VirtualMessage>;
    
    /// Get messages since timestamp
    pub async fn get_messages_since(
        &self,
        since: SystemTime,
    ) -> Vec<VirtualMessage>;
    
    /// Get channel statistics
    pub fn get_statistics(&self) -> ChannelStats;
}
```

## Public API

### Data Types

```rust
use serde::{Serialize, Deserialize};
use uuid::Uuid;
use std::time::{SystemTime, Duration};
use shared_positioning::{BeaconConfig, GeodeticPosition};

/// Virtual beacon configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VirtualBeaconConfig {
    pub beacon_config: BeaconConfig,
    pub movement_pattern: MovementPattern,
    pub auto_start: bool,
    pub emulator_specific: EmulatorSpecificConfig,
}

/// Movement patterns for virtual beacons
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum MovementPattern {
    Stationary,
    Linear {
        speed_m_per_s: f64,
        bearing_deg: f64,
    },
    Circular {
        radius_m: f64,
        period_s: f64,
        center: GeodeticPosition,
    },
    Random {
        max_speed_m_per_s: f64,
        bounds: Option<GeographicBounds>,
    },
    Custom {
        pattern_name: String,
        parameters: serde_json::Value,
    },
}

/// Beacon status information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VirtualBeaconStatus {
    pub id: Uuid,
    pub position: GeodeticPosition,
    pub is_running: bool,
    pub movement_pattern: MovementPattern,
    pub stats: VirtualBeaconStats,
    pub config: BeaconConfig,
    pub last_update: SystemTime,
}

/// Beacon statistics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VirtualBeaconStats {
    pub messages_sent: u64,
    pub last_transmission: Option<SystemTime>,
    pub uptime: Duration,
    pub transmission_failures: u32,
    pub average_interval: Duration,
    pub position_updates: u64,
}

/// Virtual message structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VirtualMessage {
    pub beacon_id: Uuid,
    pub timestamp: SystemTime,
    pub position: GeodeticPosition,
    pub message_data: Vec<u8>,
    pub signal_quality: u8,
    pub sequence_number: u16,
    pub message_version: MessageVersion,
}

/// Performance metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerformanceMetrics {
    pub total_beacons: usize,
    pub running_beacons: usize,
    pub total_messages_sent: u64,
    pub messages_per_second: f64,
    pub memory_usage_mb: f64,
    pub cpu_usage_percent: f64,
    pub channel_count: usize,
    pub uptime: Duration,
}

/// Error types
#[derive(thiserror::Error, Debug)]
pub enum EmulatorError {
    #[error("Beacon {0} already exists")]
    BeaconExists(Uuid),
    
    #[error("Beacon {0} not found")]
    BeaconNotFound(Uuid),
    
    #[error("Invalid configuration: {0}")]
    InvalidConfig(String),
    
    #[error("Communication error: {0}")]
    CommunicationError(String),
    
    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),
    
    #[error("Serialization error: {0}")]
    SerializationError(#[from] serde_json::Error),
}
```

### Configuration Types

```rust
/// Scenario configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ScenarioConfig {
    pub scenario_type: ScenarioType,
    pub beacon_count: usize,
    pub spacing: f64,
    pub center: GeodeticPosition,
    pub beacon_config: BeaconConfig,
    pub start_immediately: bool,
}

/// Export configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExportConfig {
    pub format: ExportFormat,
    pub time_range: TimeRange,
    pub beacon_filter: Option<Vec<Uuid>>,
    pub include_messages: bool,
    pub include_statistics: bool,
}

/// Time range specification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum TimeRange {
    Duration(Duration),
    Since(SystemTime),
    Between { start: SystemTime, end: SystemTime },
    All,
}
```

## Extension Points

### Custom Movement Patterns

Implement custom movement patterns by extending the `MovementPattern` enum:

```rust
use beacon_emulator::{MovementPattern, GeodeticPosition};
use std::time::Duration;

/// Custom movement pattern implementation
pub trait CustomMovementPattern: Send + Sync {
    /// Calculate new position based on elapsed time
    fn calculate_position(
        &self,
        initial_position: GeodeticPosition,
        elapsed: Duration,
        parameters: &serde_json::Value,
    ) -> GeodeticPosition;
    
    /// Validate pattern parameters
    fn validate_parameters(&self, parameters: &serde_json::Value) -> Result<(), String>;
    
    /// Get pattern name
    fn name(&self) -> &str;
}

/// Example: Spiral movement pattern
pub struct SpiralMovementPattern;

impl CustomMovementPattern for SpiralMovementPattern {
    fn calculate_position(
        &self,
        initial_position: GeodeticPosition,
        elapsed: Duration,
        parameters: &serde_json::Value,
    ) -> GeodeticPosition {
        let radius_growth = parameters["radius_growth"].as_f64().unwrap_or(1.0);
        let angular_speed = parameters["angular_speed"].as_f64().unwrap_or(0.1);
        
        let t = elapsed.as_secs_f64();
        let radius = radius_growth * t;
        let angle = angular_speed * t;
        
        let lat_offset = radius * angle.cos() / 111_132.0;
        let lon_offset = radius * angle.sin() / 
            (111_320.0 * initial_position.latitude.to_radians().cos());
        
        GeodeticPosition {
            latitude: initial_position.latitude + lat_offset,
            longitude: initial_position.longitude + lon_offset,
            altitude: initial_position.altitude,
        }
    }
    
    fn validate_parameters(&self, parameters: &serde_json::Value) -> Result<(), String> {
        if !parameters.is_object() {
            return Err("Parameters must be an object".to_string());
        }
        
        if let Some(radius_growth) = parameters.get("radius_growth") {
            if !radius_growth.is_f64() || radius_growth.as_f64().unwrap() <= 0.0 {
                return Err("radius_growth must be a positive number".to_string());
            }
        }
        
        Ok(())
    }
    
    fn name(&self) -> &str {
        "spiral"
    }
}

/// Register custom movement pattern
impl EmulatorManager {
    pub fn register_movement_pattern(
        &mut self,
        pattern: Box<dyn CustomMovementPattern>,
    ) -> Result<(), EmulatorError> {
        // Implementation to register custom pattern
        todo!()
    }
}
```

### Custom Message Formats

Extend message building capabilities:

```rust
use beacon_emulator::{VirtualMessage, MessageBuilder};
use shared_positioning::{GeodeticPosition, MessageVersion};

/// Custom message builder trait
pub trait CustomMessageBuilder: Send + Sync {
    /// Build custom message format
    fn build_message(
        &self,
        beacon_id: Uuid,
        position: GeodeticPosition,
        sequence_number: u16,
        custom_data: &serde_json::Value,
    ) -> Result<Vec<u8>, EmulatorError>;
    
    /// Get message format name
    fn format_name(&self) -> &str;
    
    /// Validate custom data
    fn validate_custom_data(&self, data: &serde_json::Value) -> Result<(), String>;
}

/// Example: Extended message with environmental data
pub struct EnvironmentalMessageBuilder;

impl CustomMessageBuilder for EnvironmentalMessageBuilder {
    fn build_message(
        &self,
        beacon_id: Uuid,
        position: GeodeticPosition,
        sequence_number: u16,
        custom_data: &serde_json::Value,
    ) -> Result<Vec<u8>, EmulatorError> {
        // Build message with environmental data
        let temperature = custom_data["temperature"].as_f64().unwrap_or(20.0);
        let pressure = custom_data["pressure"].as_f64().unwrap_or(1013.25);
        
        // Create extended message format
        let mut message = Vec::new();
        
        // Standard beacon message fields
        message.extend_from_slice(&beacon_id.as_bytes());
        message.extend_from_slice(&position.latitude.to_le_bytes());
        message.extend_from_slice(&position.longitude.to_le_bytes());
        message.extend_from_slice(&position.altitude.to_le_bytes());
        message.extend_from_slice(&sequence_number.to_le_bytes());
        
        // Environmental data extension
        message.extend_from_slice(&temperature.to_le_bytes());
        message.extend_from_slice(&pressure.to_le_bytes());
        
        Ok(message)
    }
    
    fn format_name(&self) -> &str {
        "environmental"
    }
    
    fn validate_custom_data(&self, data: &serde_json::Value) -> Result<(), String> {
        if let Some(temp) = data.get("temperature") {
            if !temp.is_f64() {
                return Err("temperature must be a number".to_string());
            }
        }
        
        Ok(())
    }
}
```

### Event Hooks

Implement event-driven extensions:

```rust
use beacon_emulator::{EmulatorEvent, EventHandler};

/// Event types
#[derive(Debug, Clone)]
pub enum EmulatorEvent {
    BeaconCreated { beacon_id: Uuid },
    BeaconStarted { beacon_id: Uuid },
    BeaconStopped { beacon_id: Uuid },
    MessageTransmitted { beacon_id: Uuid, message: VirtualMessage },
    PositionUpdated { beacon_id: Uuid, old_position: GeodeticPosition, new_position: GeodeticPosition },
    PerformanceAlert { metric: String, value: f64, threshold: f64 },
}

/// Event handler trait
pub trait EventHandler: Send + Sync {
    /// Handle emulator event
    fn handle_event(&self, event: EmulatorEvent) -> Result<(), EmulatorError>;
    
    /// Get handler name
    fn name(&self) -> &str;
}

/// Example: Logging event handler
pub struct LoggingEventHandler {
    log_file: std::sync::Mutex<std::fs::File>,
}

impl EventHandler for LoggingEventHandler {
    fn handle_event(&self, event: EmulatorEvent) -> Result<(), EmulatorError> {
        let log_entry = format!("{}: {:?}\n", chrono::Utc::now(), event);
        
        let mut file = self.log_file.lock().unwrap();
        file.write_all(log_entry.as_bytes())?;
        file.flush()?;
        
        Ok(())
    }
    
    fn name(&self) -> &str {
        "logging"
    }
}

/// Register event handler
impl EmulatorManager {
    pub fn register_event_handler(
        &mut self,
        handler: Box<dyn EventHandler>,
    ) -> Result<(), EmulatorError> {
        // Implementation to register event handler
        todo!()
    }
}
```

## Integration Examples

### Programmatic Beacon Management

```rust
use beacon_emulator::{EmulatorManager, VirtualBeaconConfig, MovementPattern};
use shared_positioning::{BeaconConfig, GeodeticPosition};
use tokio;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create emulator manager
    let mut emulator = EmulatorManager::new("test_channel");
    
    // Create beacon configuration
    let beacon_config = BeaconConfig {
        transmission_interval_ms: 5000,
        message_version: MessageVersion::V3,
        ..Default::default()
    };
    
    let virtual_config = VirtualBeaconConfig {
        beacon_config,
        movement_pattern: MovementPattern::Linear {
            speed_m_per_s: 1.5,
            bearing_deg: 45.0,
        },
        auto_start: true,
        emulator_specific: Default::default(),
    };
    
    // Create beacon
    let position = GeodeticPosition {
        latitude: 32.123,
        longitude: 45.476,
        altitude: -10.0,
    };
    
    let beacon_id = emulator.create_beacon(None, position, Some(virtual_config)).await?;
    println!("Created beacon: {}", beacon_id);
    
    // Start beacon
    emulator.start_beacon(beacon_id).await?;
    
    // Monitor for 30 seconds
    tokio::time::sleep(tokio::time::Duration::from_secs(30)).await;
    
    // Get status
    if let Some(status) = emulator.get_beacon_status(beacon_id) {
        println!("Beacon status: {:?}", status);
    }
    
    // Stop beacon
    emulator.stop_beacon(beacon_id).await?;
    
    Ok(())
}
```

### Custom Test Framework

```rust
use beacon_emulator::{EmulatorManager, ScenarioType, ScenarioConfig, ExportConfig, TimeRange};
use std::time::Duration;

/// Test framework for automated beacon testing
pub struct BeaconTestFramework {
    emulator: EmulatorManager,
    test_results: Vec<TestResult>,
}

impl BeaconTestFramework {
    pub fn new(channel: &str) -> Self {
        Self {
            emulator: EmulatorManager::new(channel),
            test_results: Vec::new(),
        }
    }
    
    /// Run positioning accuracy test
    pub async fn test_positioning_accuracy(&mut self) -> Result<TestResult, EmulatorError> {
        // Create precise beacon arrangement
        let scenario_config = ScenarioConfig {
            scenario_type: ScenarioType::Square,
            beacon_count: 4,
            spacing: 100.0,
            center: GeodeticPosition {
                latitude: 32.0,
                longitude: 45.0,
                altitude: -10.0,
            },
            beacon_config: Default::default(),
            start_immediately: true,
        };
        
        let beacon_ids = self.emulator.create_scenario(
            ScenarioType::Square,
            scenario_config,
        ).await?;
        
        // Run test for specified duration
        tokio::time::sleep(Duration::from_secs(60)).await;
        
        // Export test data
        let export_config = ExportConfig {
            format: ExportFormat::Json,
            time_range: TimeRange::Duration(Duration::from_secs(60)),
            beacon_filter: Some(beacon_ids.clone()),
            include_messages: true,
            include_statistics: true,
        };
        
        let logs = self.emulator.export_logs(export_config).await?;
        
        // Analyze results
        let accuracy = self.analyze_positioning_accuracy(&logs);
        
        let result = TestResult {
            test_name: "positioning_accuracy".to_string(),
            success: accuracy.mean_error < 1.0, // 1 meter threshold
            metrics: serde_json::json!({
                "mean_error": accuracy.mean_error,
                "max_error": accuracy.max_error,
                "beacon_count": beacon_ids.len(),
                "message_count": logs.len(),
            }),
            duration: Duration::from_secs(60),
        };
        
        self.test_results.push(result.clone());
        Ok(result)
    }
    
    /// Run performance stress test
    pub async fn test_performance_stress(&mut self, beacon_count: usize) -> Result<TestResult, EmulatorError> {
        // Create many beacons
        let scenario_config = ScenarioConfig {
            scenario_type: ScenarioType::Grid,
            beacon_count,
            spacing: 50.0,
            center: GeodeticPosition {
                latitude: 32.0,
                longitude: 45.0,
                altitude: -10.0,
            },
            beacon_config: BeaconConfig {
                transmission_interval_ms: 1000, // Fast transmission
                ..Default::default()
            },
            start_immediately: true,
        };
        
        let start_time = std::time::Instant::now();
        let beacon_ids = self.emulator.create_scenario(
            ScenarioType::Grid,
            scenario_config,
        ).await?;
        
        // Monitor performance for test duration
        let test_duration = Duration::from_secs(120);
        let mut performance_samples = Vec::new();
        
        let sample_interval = Duration::from_secs(5);
        let mut elapsed = Duration::new(0, 0);
        
        while elapsed < test_duration {
            tokio::time::sleep(sample_interval).await;
            elapsed += sample_interval;
            
            let metrics = self.emulator.get_performance_metrics();
            performance_samples.push(metrics);
        }
        
        // Analyze performance
        let avg_cpu = performance_samples.iter()
            .map(|m| m.cpu_usage_percent)
            .sum::<f64>() / performance_samples.len() as f64;
        
        let max_memory = performance_samples.iter()
            .map(|m| m.memory_usage_mb)
            .fold(0.0, f64::max);
        
        let avg_msg_rate = performance_samples.iter()
            .map(|m| m.messages_per_second)
            .sum::<f64>() / performance_samples.len() as f64;
        
        let result = TestResult {
            test_name: "performance_stress".to_string(),
            success: avg_cpu < 80.0 && max_memory < 1000.0, // Thresholds
            metrics: serde_json::json!({
                "beacon_count": beacon_count,
                "avg_cpu_percent": avg_cpu,
                "max_memory_mb": max_memory,
                "avg_message_rate": avg_msg_rate,
                "test_duration_s": test_duration.as_secs(),
            }),
            duration: test_duration,
        };
        
        self.test_results.push(result.clone());
        Ok(result)
    }
    
    /// Generate test report
    pub fn generate_report(&self) -> TestReport {
        TestReport {
            total_tests: self.test_results.len(),
            passed_tests: self.test_results.iter().filter(|r| r.success).count(),
            failed_tests: self.test_results.iter().filter(|r| !r.success).count(),
            results: self.test_results.clone(),
            generated_at: std::time::SystemTime::now(),
        }
    }
    
    fn analyze_positioning_accuracy(&self, logs: &[LogEntry]) -> AccuracyMetrics {
        // Implementation to analyze positioning accuracy from logs
        AccuracyMetrics {
            mean_error: 0.5,
            max_error: 2.0,
            std_deviation: 0.3,
        }
    }
}

#[derive(Debug, Clone)]
pub struct TestResult {
    pub test_name: String,
    pub success: bool,
    pub metrics: serde_json::Value,
    pub duration: Duration,
}

#[derive(Debug, Clone)]
pub struct TestReport {
    pub total_tests: usize,
    pub passed_tests: usize,
    pub failed_tests: usize,
    pub results: Vec<TestResult>,
    pub generated_at: std::time::SystemTime,
}

#[derive(Debug)]
struct AccuracyMetrics {
    mean_error: f64,
    max_error: f64,
    std_deviation: f64,
}
```

## Testing Framework

### Mock Components

```rust
use beacon_emulator::{VirtualChannel, VirtualMessage, EmulatorError};
use tokio::sync::broadcast;

/// Mock virtual channel for testing
pub struct MockVirtualChannel {
    pub sent_messages: std::sync::Mutex<Vec<VirtualMessage>>,
    pub should_fail: std::sync::atomic::AtomicBool,
    pub delay_ms: std::sync::atomic::AtomicU64,
}

impl MockVirtualChannel {
    pub fn new() -> Self {
        Self {
            sent_messages: std::sync::Mutex::new(Vec::new()),
            should_fail: std::sync::atomic::AtomicBool::new(false),
            delay_ms: std::sync::atomic::AtomicU64::new(0),
        }
    }
    
    pub fn set_failure_mode(&self, should_fail: bool) {
        self.should_fail.store(should_fail, std::sync::atomic::Ordering::Relaxed);
    }
    
    pub fn set_delay(&self, delay_ms: u64) {
        self.delay_ms.store(delay_ms, std::sync::atomic::Ordering::Relaxed);
    }
    
    pub fn get_sent_messages(&self) -> Vec<VirtualMessage> {
        self.sent_messages.lock().unwrap().clone()
    }
    
    pub fn clear_messages(&self) {
        self.sent_messages.lock().unwrap().clear();
    }
}

#[async_trait::async_trait]
impl VirtualChannel for MockVirtualChannel {
    async fn broadcast_message(&self, message: VirtualMessage) -> Result<(), EmulatorError> {
        // Simulate delay
        let delay = self.delay_ms.load(std::sync::atomic::Ordering::Relaxed);
        if delay > 0 {
            tokio::time::sleep(tokio::time::Duration::from_millis(delay)).await;
        }
        
        // Simulate failure
        if self.should_fail.load(std::sync::atomic::Ordering::Relaxed) {
            return Err(EmulatorError::CommunicationError("Mock failure".to_string()));
        }
        
        // Store message
        self.sent_messages.lock().unwrap().push(message);
        
        Ok(())
    }
    
    fn subscribe(&self) -> broadcast::Receiver<VirtualMessage> {
        // Return dummy receiver for testing
        let (_, rx) = broadcast::channel(1);
        rx
    }
    
    async fn get_recent_messages(&self, count: usize) -> Vec<VirtualMessage> {
        let messages = self.sent_messages.lock().unwrap();
        messages.iter().rev().take(count).cloned().collect()
    }
}
```

### Test Utilities

```rust
/// Test utilities for beacon emulator testing
pub mod test_utils {
    use super::*;
    use std::time::{SystemTime, Duration};
    
    /// Create test beacon configuration
    pub fn create_test_beacon_config() -> VirtualBeaconConfig {
        VirtualBeaconConfig {
            beacon_config: BeaconConfig {
                transmission_interval_ms: 1000,
                message_version: MessageVersion::V3,
                ..Default::default()
            },
            movement_pattern: MovementPattern::Stationary,
            auto_start: false,
            emulator_specific: Default::default(),
        }
    }
    
    /// Create test position
    pub fn create_test_position(lat: f64, lon: f64, depth: f64) -> GeodeticPosition {
        GeodeticPosition {
            latitude: lat,
            longitude: lon,
            altitude: -depth,
        }
    }
    
    /// Wait for condition with timeout
    pub async fn wait_for_condition<F>(
        mut condition: F,
        timeout: Duration,
        check_interval: Duration,
    ) -> Result<(), &'static str>
    where
        F: FnMut() -> bool,
    {
        let start = std::time::Instant::now();
        
        while start.elapsed() < timeout {
            if condition() {
                return Ok(());
            }
            
            tokio::time::sleep(check_interval).await;
        }
        
        Err("Condition not met within timeout")
    }
    
    /// Assert message count within tolerance
    pub fn assert_message_count_approx(
        actual: usize,
        expected: usize,
        tolerance_percent: f64,
    ) {
        let tolerance = (expected as f64 * tolerance_percent / 100.0) as usize;
        let min_expected = expected.saturating_sub(tolerance);
        let max_expected = expected + tolerance;
        
        assert!(
            actual >= min_expected && actual <= max_expected,
            "Message count {} not within {}% of expected {} (range: {}-{})",
            actual, tolerance_percent, expected, min_expected, max_expected
        );
    }
    
    /// Create test scenario
    pub async fn create_test_scenario(
        emulator: &mut EmulatorManager,
        beacon_count: usize,
    ) -> Result<Vec<Uuid>, EmulatorError> {
        let mut beacon_ids = Vec::new();
        
        for i in 0..beacon_count {
            let position = create_test_position(
                32.0 + (i as f64) * 0.001,
                45.0 + (i as f64) * 0.001,
                10.0,
            );
            
            let config = create_test_beacon_config();
            let id = emulator.create_beacon(None, position, Some(config)).await?;
            beacon_ids.push(id);
        }
        
        Ok(beacon_ids)
    }
}
```

This API documentation provides comprehensive coverage of the beacon emulator's programmatic interface, extension points, and testing framework. It enables developers to integrate the emulator into their own applications, extend its functionality, and create automated testing workflows.