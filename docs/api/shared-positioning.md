# Shared Positioning Library API Reference

The `shared-positioning` library provides common functionality used by both beacon and receiver systems in the underwater positioning system. This document provides comprehensive API documentation for all public interfaces.

## Table of Contents

- [Message Parser](#message-parser)
- [Transceiver Interface](#transceiver-interface)
- [Coordinate System](#coordinate-system)
- [Error Handling](#error-handling)
- [GPS Manager](#gps-manager)
- [Power Manager](#power-manager)
- [Communication Manager](#communication-manager)
- [Transmission Manager](#transmission-manager)
- [Configuration Management](#configuration-management)
- [Environmental Monitoring](#environmental-monitoring)
- [Hardware Monitoring](#hardware-monitoring)
- [Reliability Monitoring](#reliability-monitoring)

## Message Parser

The message parser handles parsing and building of underwater positioning messages in multiple format versions.

### MessageParser

```rust
pub struct MessageParser {
    pub fn new() -> Self
    pub fn parse_message(&self, data: &[u8]) -> Result<AnchorMessage, MessageParseError>
    pub fn validate_message(&self, data: &[u8]) -> ValidationResult
    pub fn get_stats(&self) -> ValidationStats
}
```

**Usage Example:**
```rust
use shared_positioning::MessageParser;

let parser = MessageParser::new();
let message = parser.parse_message(&raw_data)?;
println!("Received position: {}, {}", message.position.latitude, message.position.longitude);
```

### MessageBuilder

```rust
pub struct MessageBuilder {
    pub fn new() -> Self
    pub fn build_v1_message(&self, beacon_id: Uuid, position: GeodeticPosition, 
                           signal_quality: u8, sequence: u16) -> Result<Vec<u8>, MessageParseError>
    pub fn build_v2_message(&self, beacon_id: Uuid, position: GeodeticPosition, 
                           signal_quality: u8, sequence: u16) -> Result<Vec<u8>, MessageParseError>
    pub fn build_v3_message(&self, beacon_id: Uuid, position: GeodeticPosition, 
                           signal_quality: u8, sequence: u16) -> Result<Vec<u8>, MessageParseError>
}
```

**Usage Example:**
```rust
use shared_positioning::{MessageBuilder, GeodeticPosition};
use uuid::Uuid;

let builder = MessageBuilder::new();
let position = GeodeticPosition {
    latitude: 40.7128,
    longitude: -74.0060,
    altitude: 0.0,
};
let message = builder.build_v3_message(
    Uuid::new_v4(),
    position,
    95, // signal quality
    1   // sequence number
)?;
```

### Message Types

#### AnchorMessage
```rust
pub struct AnchorMessage {
    pub anchor_id: u16,
    pub position: GeodeticPosition,
    pub timestamp: SystemTime,
    pub signal_quality: u8,
    pub sequence_number: u16,
    pub message_version: MessageVersion,
}
```

#### GeodeticPosition
```rust
pub struct GeodeticPosition {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
}
```

#### MessageVersion
```rust
pub enum MessageVersion {
    V1,
    V2,
    V3, // Supports UUID beacon IDs
}
```

## Transceiver Interface

The transceiver interface provides hardware abstraction for underwater communication devices.

### TransceiverInterface Trait

```rust
pub trait TransceiverInterface {
    fn initialize(&mut self) -> Result<(), CommError>;
    fn send_data(&mut self, data: &[u8]) -> Result<(), CommError>;
    fn receive_data(&mut self) -> Result<Option<Vec<u8>>, CommError>;
    fn transmit_message(&mut self, data: &[u8]) -> Result<(), CommError>;
    fn set_transmission_power(&mut self, power_level: u8) -> Result<(), CommError>;
    fn get_transmission_status(&self) -> TransmissionStatus;
    fn get_connection_stats(&self) -> ConnectionStats;
    fn configure(&mut self, config: TransceiverConfig) -> Result<(), CommError>;
    fn shutdown(&mut self) -> Result<(), CommError>;
}
```

**Usage Example:**
```rust
use shared_positioning::{TransceiverInterface, TransceiverConfig};

fn setup_transceiver<T: TransceiverInterface>(mut transceiver: T) -> Result<(), CommError> {
    transceiver.initialize()?;
    transceiver.set_transmission_power(80)?; // 80% power
    
    let message = b"Hello underwater world!";
    transceiver.transmit_message(message)?;
    
    Ok(())
}
```

### Mock Implementations

#### MockTransceiver
```rust
pub struct MockTransceiver {
    pub fn new() -> Self
    pub fn set_transmission_success_rate(&mut self, rate: f64)
    pub fn set_transmission_delay(&mut self, delay: Duration)
    pub fn get_transmitted_messages(&self) -> &[Vec<u8>]
}
```

**Usage Example:**
```rust
use shared_positioning::MockTransceiver;

let mut mock = MockTransceiver::new();
mock.set_transmission_success_rate(0.95); // 95% success rate
mock.set_transmission_delay(Duration::from_millis(100));
```

### Configuration Types

#### TransceiverConfig
```rust
pub struct TransceiverConfig {
    pub baud_rate: u32,
    pub power_mode: PowerMode,
    pub timeout_ms: u32,
    pub retry_attempts: u32,
}
```

#### PowerMode
```rust
pub enum PowerMode {
    Low,
    Medium,
    High,
    Adaptive,
}
```

## Coordinate System

The coordinate system module handles GPS coordinate transformations and validation.

### CoordinateSystemManager

```rust
pub struct CoordinateSystemManager {
    pub fn new() -> Self
    pub fn validate_position(&self, position: &GeodeticPosition) -> bool
    pub fn calculate_distance(&self, pos1: &GeodeticPosition, pos2: &GeodeticPosition) -> f64
    pub fn to_utm(&self, position: &GeodeticPosition) -> Result<UTMCoordinate, CoordinateError>
    pub fn from_utm(&self, utm: &UTMCoordinate) -> Result<GeodeticPosition, CoordinateError>
}
```

**Usage Example:**
```rust
use shared_positioning::{CoordinateSystemManager, GeodeticPosition};

let coord_manager = CoordinateSystemManager::new();
let pos1 = GeodeticPosition { latitude: 40.7128, longitude: -74.0060, altitude: 0.0 };
let pos2 = GeodeticPosition { latitude: 40.7589, longitude: -73.9851, altitude: 0.0 };

let distance = coord_manager.calculate_distance(&pos1, &pos2);
println!("Distance: {:.2} meters", distance);
```

### UTMCoordinate

```rust
pub struct UTMCoordinate {
    pub easting: f64,
    pub northing: f64,
    pub zone: u8,
    pub hemisphere: char, // 'N' or 'S'
}
```

## Error Handling

Comprehensive error handling system with recovery strategies and diagnostic capabilities.

### BeaconError

```rust
pub enum BeaconError {
    GpsError(GpsErrorType, BeaconErrorContext),
    PowerError(PowerErrorType, BeaconErrorContext),
    CommunicationError(CommunicationErrorType, BeaconErrorContext),
    TransmissionError(TransmissionErrorType, BeaconErrorContext),
    ConfigurationError(ConfigurationErrorType, BeaconErrorContext),
    SystemError(SystemErrorType, BeaconErrorContext),
    HardwareFault(HardwareComponent, HardwareFaultType, BeaconErrorContext),
}
```

### Error Context

```rust
pub struct BeaconErrorContext {
    pub timestamp: SystemTime,
    pub component: String,
    pub operation: String,
    pub system_state: BeaconSystemState,
    pub resource_usage: ResourceUsageSnapshot,
    pub environmental_conditions: Option<EnvironmentalMetrics>,
}
```

### Diagnostic System

```rust
pub struct DiagnosticSystemManager {
    pub fn new() -> Self
    pub fn generate_diagnostic_report(&self) -> DiagnosticReport
    pub fn get_system_health_metrics(&self) -> SystemHealthMetrics
    pub fn check_component_health(&self, component: HardwareComponent) -> ComponentStatus
    pub fn get_error_statistics(&self) -> ErrorStatistics
}
```

**Usage Example:**
```rust
use shared_positioning::{DiagnosticSystemManager, HardwareComponent};

let diagnostics = DiagnosticSystemManager::new();
let report = diagnostics.generate_diagnostic_report();
let gps_health = diagnostics.check_component_health(HardwareComponent::GpsReceiver);

println!("System Health Score: {}", report.overall_health_score);
println!("GPS Status: {:?}", gps_health);
```

## GPS Manager

GPS position acquisition and accuracy monitoring.

### GpsManager Trait

```rust
pub trait GpsManager {
    fn start_acquisition(&mut self) -> Result<(), GpsError>;
    fn stop_acquisition(&mut self) -> Result<(), GpsError>;
    fn get_current_position(&self) -> Option<GpsPosition>;
    fn get_position_accuracy(&self) -> Option<f32>;
    fn is_locked(&self) -> bool;
    fn get_satellite_count(&self) -> u8;
    fn get_status(&self) -> GpsStatus;
    fn configure(&mut self, config: GpsConfig) -> Result<(), GpsError>;
}
```

### GpsPosition

```rust
pub struct GpsPosition {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    pub timestamp: SystemTime,
    pub accuracy_m: f32,
    pub satellite_count: u8,
    pub hdop: f32, // Horizontal dilution of precision
    pub vdop: f32, // Vertical dilution of precision
}
```

### GpsConfig

```rust
pub struct GpsConfig {
    pub acquisition_timeout_s: u32,
    pub update_interval_s: u32,
    pub min_satellite_count: u8,
    pub accuracy_threshold_m: f32,
    pub cold_start_timeout_s: u32,
    pub power_save_enabled: bool,
}
```

**Usage Example:**
```rust
use shared_positioning::{BasicGpsManager, GpsConfig};

let config = GpsConfig {
    acquisition_timeout_s: 60,
    update_interval_s: 30,
    min_satellite_count: 4,
    accuracy_threshold_m: 5.0,
    cold_start_timeout_s: 300,
    power_save_enabled: true,
};

let mut gps = BasicGpsManager::new(config);
gps.start_acquisition()?;

if let Some(position) = gps.get_current_position() {
    println!("Current position: {:.6}, {:.6}", position.latitude, position.longitude);
}
```

## Power Manager

Battery monitoring and power optimization.

### PowerManager Trait

```rust
pub trait PowerManager {
    fn get_battery_status(&self) -> BatteryStatus;
    fn get_charging_status(&self) -> ChargingStatus;
    fn set_power_mode(&mut self, mode: PowerOperationMode) -> Result<(), PowerError>;
    fn estimate_remaining_time(&self) -> Duration;
    fn configure_power_thresholds(&mut self, config: PowerConfig) -> Result<(), PowerError>;
    fn get_power_stats(&self) -> PowerStats;
    fn enable_power_saving(&mut self, enabled: bool) -> Result<(), PowerError>;
}
```

### BatteryStatus

```rust
pub struct BatteryStatus {
    pub voltage_v: f32,
    pub current_ma: f32,
    pub capacity_percent: f32,
    pub temperature_c: f32,
    pub health: BatteryHealth,
    pub cycle_count: u32,
    pub time_to_empty: Option<Duration>,
    pub time_to_full: Option<Duration>,
}
```

### PowerConfig

```rust
pub struct PowerConfig {
    pub low_battery_threshold_percent: f32,
    pub critical_battery_threshold_percent: f32,
    pub emergency_battery_threshold_percent: f32,
    pub power_save_mode_threshold_percent: f32,
    pub charging_enabled: bool,
    pub solar_charging_enabled: bool,
    pub temperature_monitoring_enabled: bool,
    pub max_charging_current_ma: f32,
}
```

**Usage Example:**
```rust
use shared_positioning::{BasicPowerManager, PowerConfig, PowerOperationMode};

let config = PowerConfig {
    low_battery_threshold_percent: 20.0,
    critical_battery_threshold_percent: 10.0,
    emergency_battery_threshold_percent: 5.0,
    power_save_mode_threshold_percent: 30.0,
    charging_enabled: true,
    solar_charging_enabled: false,
    temperature_monitoring_enabled: true,
    max_charging_current_ma: 500.0,
};

let mut power_manager = BasicPowerManager::new(config);
let battery_status = power_manager.get_battery_status();

if battery_status.capacity_percent < 20.0 {
    power_manager.set_power_mode(PowerOperationMode::PowerSave)?;
}
```

## Communication Manager

Long-range communication for remote monitoring and configuration.

### CommunicationManager Trait

```rust
pub trait CommunicationManager {
    fn connect(&mut self) -> Result<(), CommError>;
    fn disconnect(&mut self) -> Result<(), CommError>;
    fn send_status_report(&mut self, report: StatusReport) -> Result<(), CommError>;
    fn check_for_updates(&mut self) -> Result<Option<ConfigUpdate>, CommError>;
    fn is_connected(&self) -> bool;
    fn get_signal_strength(&self) -> Option<u8>;
    fn get_connection_stats(&self) -> ConnectionStats;
    fn configure(&mut self, config: CommunicationConfig) -> Result<(), CommError>;
}
```

### StatusReport

```rust
pub struct StatusReport {
    pub beacon_id: Uuid,
    pub timestamp: SystemTime,
    pub position_history: Vec<GpsPosition>,
    pub battery_status: BatteryStatus,
    pub system_health: SystemHealth,
    pub transmission_stats: CommTransmissionStats,
    pub error_log: Vec<ErrorLogEntry>,
    pub uptime: Duration,
}
```

### CommunicationConfig

```rust
pub struct CommunicationConfig {
    pub connection_interval_hours: u32,
    pub retry_attempts: u32,
    pub retry_backoff_ms: u32,
    pub max_retry_interval_hours: u32,
    pub connection_timeout_s: u32,
    pub data_compression_enabled: bool,
    pub encryption_enabled: bool,
    pub max_report_size_bytes: usize,
}
```

**Usage Example:**
```rust
use shared_positioning::{BasicCommunicationManager, CommunicationConfig, StatusReport};

let config = CommunicationConfig {
    connection_interval_hours: 24,
    retry_attempts: 3,
    retry_backoff_ms: 5000,
    max_retry_interval_hours: 6,
    connection_timeout_s: 60,
    data_compression_enabled: true,
    encryption_enabled: true,
    max_report_size_bytes: 1024 * 1024, // 1MB
};

let mut comm_manager = BasicCommunicationManager::new(config);
comm_manager.connect()?;

let status_report = StatusReport {
    beacon_id: Uuid::new_v4(),
    timestamp: SystemTime::now(),
    // ... other fields
};

comm_manager.send_status_report(status_report)?;
```

## Transmission Manager

Coordinates underwater message broadcasting and scheduling.

### TransmissionManager

```rust
pub struct TransmissionManager {
    pub fn new(config: TransmissionConfig) -> Self
    pub fn schedule_transmission(&mut self, message: Vec<u8>, priority: TransmissionPriority) -> Result<(), TransmissionError>
    pub fn process_transmission_queue(&mut self) -> Result<(), TransmissionError>
    pub fn get_statistics(&self) -> TransmissionStatistics
    pub fn set_transmission_interval(&mut self, interval: Duration) -> Result<(), TransmissionError>
    pub fn configure(&mut self, config: TransmissionConfig) -> Result<(), TransmissionError>
}
```

### TransmissionConfig

```rust
pub struct TransmissionConfig {
    pub default_interval_ms: u32,
    pub max_queue_size: usize,
    pub retry_attempts: u32,
    pub power_adaptation_enabled: bool,
    pub environmental_adaptation_enabled: bool,
    pub message_compression_enabled: bool,
}
```

**Usage Example:**
```rust
use shared_positioning::{TransmissionManager, TransmissionConfig, TransmissionPriority};

let config = TransmissionConfig {
    default_interval_ms: 5000,
    max_queue_size: 100,
    retry_attempts: 3,
    power_adaptation_enabled: true,
    environmental_adaptation_enabled: true,
    message_compression_enabled: false,
};

let mut tx_manager = TransmissionManager::new(config);
let message = b"Position update".to_vec();
tx_manager.schedule_transmission(message, TransmissionPriority::Normal)?;
tx_manager.process_transmission_queue()?;
```

## Configuration Management

Comprehensive configuration management with validation and migration support.

### BeaconConfig

```rust
pub struct BeaconConfig {
    pub beacon_id: Uuid,
    pub transmission_config: BeaconTransmissionConfig,
    pub gps_config: BeaconGpsConfig,
    pub power_config: BeaconPowerConfig,
    pub communication_config: BeaconCommunicationConfig,
    pub emergency_config: EmergencyConfig,
    pub hardware_config: HardwareConfig,
}
```

### BeaconConfigManager

```rust
pub struct BeaconConfigManager {
    pub fn new() -> Self
    pub fn load_config(&self, path: &Path) -> Result<BeaconConfig, ConfigError>
    pub fn save_config(&self, config: &BeaconConfig, path: &Path) -> Result<(), ConfigError>
    pub fn validate_config(&self, config: &BeaconConfig) -> Result<(), ConfigError>
    pub fn create_backup(&self, config: &BeaconConfig) -> BeaconConfigBackup
    pub fn restore_from_backup(&self, backup: &BeaconConfigBackup) -> BeaconConfig
    pub fn migrate_config(&self, old_config: &BeaconConfig, target_version: u32) -> Result<BeaconConfig, ConfigError>
}
```

**Usage Example:**
```rust
use shared_positioning::{BeaconConfigManager, BeaconConfig};
use std::path::Path;

let config_manager = BeaconConfigManager::new();
let config = config_manager.load_config(Path::new("beacon.toml"))?;

// Validate configuration
config_manager.validate_config(&config)?;

// Create backup before modifications
let backup = config_manager.create_backup(&config);

// Save updated configuration
config_manager.save_config(&config, Path::new("beacon.toml"))?;
```

## Environmental Monitoring

Real-time environmental condition monitoring and adaptation.

### EnvironmentalMonitor

```rust
pub struct EnvironmentalMonitor {
    pub fn new(thresholds: EnvironmentalThresholds) -> Self
    pub fn update_conditions(&mut self, conditions: ExtendedEnvironmentalConditions) -> Result<Vec<AdaptationAction>, EnvironmentalError>
    pub fn get_current_conditions(&self) -> Option<ExtendedEnvironmentalConditions>
    pub fn get_statistics(&self) -> EnvironmentalStats
    pub fn configure_thresholds(&mut self, thresholds: EnvironmentalThresholds) -> Result<(), EnvironmentalError>
}
```

### ExtendedEnvironmentalConditions

```rust
pub struct ExtendedEnvironmentalConditions {
    pub temperature_c: f32,
    pub humidity_percent: f32,
    pub pressure_hpa: f32,
    pub wind_speed_ms: f32,
    pub wave_height_m: f32,
    pub visibility_m: f32,
    pub salinity_ppt: f32,
    pub current_speed_ms: f32,
    pub measurement_quality: MeasurementQuality,
    pub timestamp: SystemTime,
}
```

**Usage Example:**
```rust
use shared_positioning::{EnvironmentalMonitor, EnvironmentalThresholds, ExtendedEnvironmentalConditions};

let thresholds = EnvironmentalThresholds {
    max_temperature_c: 60.0,
    min_temperature_c: -20.0,
    max_wave_height_m: 5.0,
    max_wind_speed_ms: 20.0,
    // ... other thresholds
};

let mut env_monitor = EnvironmentalMonitor::new(thresholds);

let conditions = ExtendedEnvironmentalConditions {
    temperature_c: 25.0,
    humidity_percent: 65.0,
    pressure_hpa: 1013.25,
    // ... other conditions
};

let adaptations = env_monitor.update_conditions(conditions)?;
for action in adaptations {
    println!("Adaptation required: {:?}", action);
}
```

## Hardware Monitoring

Hardware fault detection and diagnostic capabilities.

### HardwareMonitor

```rust
pub struct HardwareMonitor {
    pub fn new(config: HardwareMonitorConfig) -> Self
    pub fn run_diagnostics(&mut self) -> Result<Vec<DiagnosticResult>, HardwareFaultError>
    pub fn check_component(&mut self, component: HardwareComponent) -> Result<ComponentHealth, HardwareFaultError>
    pub fn get_statistics(&self) -> HardwareMonitorStats
    pub fn configure(&mut self, config: HardwareMonitorConfig) -> Result<(), HardwareFaultError>
}
```

### DiagnosticResult

```rust
pub struct DiagnosticResult {
    pub component: HardwareComponent,
    pub health: ComponentHealth,
    pub test_results: Vec<String>,
    pub recommended_actions: Vec<RecommendedAction>,
    pub timestamp: SystemTime,
}
```

**Usage Example:**
```rust
use shared_positioning::{HardwareMonitor, HardwareMonitorConfig, HardwareComponent};

let config = HardwareMonitorConfig {
    diagnostic_interval_s: 300, // 5 minutes
    component_timeout_s: 30,
    enable_predictive_analysis: true,
    // ... other config
};

let mut hw_monitor = HardwareMonitor::new(config);
let diagnostics = hw_monitor.run_diagnostics()?;

for result in diagnostics {
    println!("Component: {:?}, Health: {:?}", result.component, result.health);
    for action in result.recommended_actions {
        println!("  Recommended: {:?}", action);
    }
}
```

## Reliability Monitoring

System reliability tracking and predictive maintenance.

### ReliabilityMonitor

```rust
pub struct ReliabilityMonitor {
    pub fn new(thresholds: ReliabilityThresholds) -> Self
    pub fn record_failure_event(&mut self, event: FailureEvent)
    pub fn get_reliability_metrics(&self) -> ReliabilityMetrics
    pub fn generate_report(&self) -> ReliabilityReport
    pub fn get_maintenance_recommendations(&self) -> Vec<ReliabilityMaintenanceRecommendation>
    pub fn configure_thresholds(&mut self, thresholds: ReliabilityThresholds)
}
```

### ReliabilityMetrics

```rust
pub struct ReliabilityMetrics {
    pub overall_reliability_score: f64,
    pub component_reliability: HashMap<HardwareComponent, ComponentReliability>,
    pub mean_time_between_failures: Duration,
    pub mean_time_to_repair: Duration,
    pub availability_percentage: f64,
    pub health_trend: HealthTrend,
}
```

**Usage Example:**
```rust
use shared_positioning::{ReliabilityMonitor, ReliabilityThresholds, FailureEvent, HardwareComponent};

let thresholds = ReliabilityThresholds {
    min_reliability_score: 0.95,
    max_failure_rate_per_hour: 0.001,
    min_availability_percentage: 99.0,
    // ... other thresholds
};

let mut reliability_monitor = ReliabilityMonitor::new(thresholds);

// Record a failure event
let failure = FailureEvent {
    component: HardwareComponent::GpsReceiver,
    timestamp: SystemTime::now(),
    severity: ErrorSeverity::Warning,
    description: "GPS signal temporarily lost".to_string(),
    recovery_time: Some(Duration::from_secs(30)),
};

reliability_monitor.record_failure_event(failure);

let metrics = reliability_monitor.get_reliability_metrics();
println!("System reliability: {:.2}%", metrics.overall_reliability_score * 100.0);

let recommendations = reliability_monitor.get_maintenance_recommendations();
for rec in recommendations {
    println!("Maintenance: {:?} - {}", rec.maintenance_type, rec.description);
}
```

## Error Types and Recovery

### Common Error Types

All modules use consistent error handling patterns:

```rust
// GPS errors
pub enum GpsErrorType {
    AcquisitionTimeout,
    SignalLost,
    AccuracyTooLow,
    HardwareFault,
    ConfigurationInvalid,
}

// Power errors
pub enum PowerErrorType {
    BatteryDepleted,
    ChargingFault,
    TemperatureExtreme,
    VoltageOutOfRange,
    CurrentOverload,
}

// Communication errors
pub enum CommunicationErrorType {
    ConnectionFailed,
    TransmissionFailed,
    AuthenticationFailed,
    TimeoutExpired,
    DataCorrupted,
}
```

### Recovery Strategies

```rust
pub enum RecoveryStrategy {
    Retry { max_attempts: u32, backoff_ms: u32 },
    Fallback { alternative_action: String },
    Graceful { shutdown_delay_s: u32 },
    Emergency { immediate_action: String },
    Ignore,
}
```

## Thread Safety and Async Support

Most components support both synchronous and asynchronous operation:

```rust
// Synchronous usage
let mut gps = BasicGpsManager::new(config);
gps.start_acquisition()?;

// Async usage (where supported)
let mut gps = AsyncGpsManager::new(config);
gps.start_acquisition().await?;
```

## Testing Support

All components provide mock implementations for testing:

```rust
use shared_positioning::{MockGpsManager, MockPowerManager, MockTransceiver};

// Create mock components for testing
let mut mock_gps = MockGpsManager::new();
mock_gps.set_simulated_position(position);

let mut mock_power = MockPowerManager::new();
mock_power.set_battery_level(75.0);

let mut mock_transceiver = MockTransceiver::new();
mock_transceiver.set_transmission_success_rate(0.95);
```

## Performance Considerations

### Memory Usage

- All components are designed for minimal memory footprint
- Use streaming algorithms where possible
- Avoid large buffer allocations
- Implement object pooling for frequently used types

### CPU Optimization

- Efficient algorithms for coordinate transformations
- Minimal computational overhead in message parsing
- Optimized power management calculations
- Hardware-specific optimizations for ESP01-class devices

### Power Optimization

- Aggressive power management in all components
- Radio duty cycling support
- CPU frequency scaling integration
- Peripheral power gating coordination

## Integration Examples

### Complete Beacon Setup

```rust
use shared_positioning::*;
use uuid::Uuid;

fn setup_beacon_system() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize GPS
    let gps_config = GpsConfig {
        acquisition_timeout_s: 60,
        update_interval_s: 30,
        min_satellite_count: 4,
        accuracy_threshold_m: 5.0,
        cold_start_timeout_s: 300,
        power_save_enabled: true,
    };
    let mut gps = BasicGpsManager::new(gps_config);
    
    // Initialize power management
    let power_config = PowerConfig {
        low_battery_threshold_percent: 20.0,
        critical_battery_threshold_percent: 10.0,
        emergency_battery_threshold_percent: 5.0,
        power_save_mode_threshold_percent: 30.0,
        charging_enabled: true,
        solar_charging_enabled: false,
        temperature_monitoring_enabled: true,
        max_charging_current_ma: 500.0,
    };
    let mut power_manager = BasicPowerManager::new(power_config);
    
    // Initialize communication
    let comm_config = CommunicationConfig {
        connection_interval_hours: 24,
        retry_attempts: 3,
        retry_backoff_ms: 5000,
        max_retry_interval_hours: 6,
        connection_timeout_s: 60,
        data_compression_enabled: true,
        encryption_enabled: true,
        max_report_size_bytes: 1024 * 1024,
    };
    let mut comm_manager = BasicCommunicationManager::new(comm_config);
    
    // Initialize message building
    let message_builder = MessageBuilder::new();
    
    // Start GPS acquisition
    gps.start_acquisition()?;
    
    // Main operation loop
    loop {
        // Check power status
        let battery_status = power_manager.get_battery_status();
        if battery_status.capacity_percent < 20.0 {
            power_manager.set_power_mode(PowerOperationMode::PowerSave)?;
        }
        
        // Get GPS position
        if let Some(position) = gps.get_current_position() {
            let geodetic_pos = GeodeticPosition {
                latitude: position.latitude,
                longitude: position.longitude,
                altitude: position.altitude,
            };
            
            // Build and transmit message
            let message = message_builder.build_v3_message(
                Uuid::new_v4(),
                geodetic_pos,
                95, // signal quality
                1   // sequence number
            )?;
            
            // Transmit message (would use actual transceiver)
            println!("Transmitting position: {:.6}, {:.6}", 
                position.latitude, position.longitude);
        }
        
        // Sleep until next transmission
        std::thread::sleep(std::time::Duration::from_secs(5));
    }
}
```

This comprehensive API documentation covers all major components of the shared positioning library, providing developers with the information needed to integrate and extend the system effectively.