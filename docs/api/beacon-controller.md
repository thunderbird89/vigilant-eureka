# Beacon Controller API Reference

The beacon controller provides the main orchestration and state management for underwater positioning beacons. This document covers the complete API for beacon control, configuration, and monitoring.

## Table of Contents

- [BeaconController](#beaconcontroller)
- [Configuration Management](#configuration-management)
- [Status and Monitoring](#status-and-monitoring)
- [Emergency Handling](#emergency-handling)
- [Lifecycle Management](#lifecycle-management)
- [Error Types](#error-types)
- [Usage Examples](#usage-examples)

## BeaconController

The main beacon controller coordinates all subsystems and manages the beacon lifecycle.

### Constructor and Basic Operations

```rust
pub struct BeaconController<G, P, C, T>
where
    G: GpsManager,
    P: PowerManager,
    C: CommunicationManager,
    T: TransceiverInterface,
{
    pub fn new(
        config: BeaconConfig,
        gps_manager: G,
        power_manager: P,
        communication_manager: C,
        transceiver: T,
    ) -> Result<Self, BeaconError>
    
    pub fn start(&mut self) -> Result<(), BeaconError>
    pub fn stop(&mut self) -> Result<(), BeaconError>
    pub fn get_status(&self) -> BeaconStatus
    pub fn update_configuration(&mut self, config: BeaconConfig) -> Result<(), BeaconError>
    pub fn handle_emergency(&mut self, emergency_type: EmergencyType) -> Result<(), BeaconError>
}
```

### Generic Type Parameters

The beacon controller is generic over its subsystem implementations:

- `G: GpsManager` - GPS position acquisition implementation
- `P: PowerManager` - Power monitoring and management implementation  
- `C: CommunicationManager` - Long-range communication implementation
- `T: TransceiverInterface` - Underwater transceiver implementation

**Usage Example:**
```rust
use beacon::{BeaconController, BeaconConfig};
use shared_positioning::{BasicGpsManager, BasicPowerManager, BasicCommunicationManager, SerialTransceiver};
use uuid::Uuid;

// Create subsystem implementations
let gps_manager = BasicGpsManager::new(gps_config);
let power_manager = BasicPowerManager::new(power_config);
let comm_manager = BasicCommunicationManager::new(comm_config);
let transceiver = SerialTransceiver::new("/dev/ttyUSB0")?;

// Create beacon configuration
let beacon_config = BeaconConfig {
    beacon_id: Uuid::new_v4(),
    transmission_config: transmission_config,
    gps_config: gps_config,
    power_config: power_config,
    communication_config: comm_config,
    emergency_config: emergency_config,
    hardware_config: hardware_config,
};

// Create and start beacon controller
let mut beacon = BeaconController::new(
    beacon_config,
    gps_manager,
    power_manager,
    comm_manager,
    transceiver,
)?;

beacon.start()?;
```

## Configuration Management

### BeaconConfig

Complete beacon configuration structure:

```rust
pub struct BeaconConfig {
    pub beacon_id: Uuid,
    pub transmission_config: TransmissionConfig,
    pub gps_config: GpsConfig,
    pub power_config: PowerConfig,
    pub communication_config: CommunicationConfig,
    pub emergency_config: EmergencyConfig,
    pub hardware_config: HardwareConfig,
}
```

### Configuration Updates

```rust
impl<G, P, C, T> BeaconController<G, P, C, T> {
    pub fn update_configuration(&mut self, config: BeaconConfig) -> Result<(), BeaconError>
    pub fn get_configuration(&self) -> &BeaconConfig
    pub fn validate_configuration(&self, config: &BeaconConfig) -> Result<(), ConfigError>
    pub fn reload_configuration(&mut self) -> Result<(), BeaconError>
}
```

**Usage Example:**
```rust
// Update transmission interval
let mut new_config = beacon.get_configuration().clone();
new_config.transmission_config.interval_ms = 10000; // 10 seconds

// Validate and apply configuration
beacon.validate_configuration(&new_config)?;
beacon.update_configuration(new_config)?;
```

### Configuration Validation

The beacon controller validates all configuration parameters:

```rust
pub struct ConfigValidator {
    pub fn validate_transmission_config(&self, config: &TransmissionConfig) -> Result<(), ConfigError>
    pub fn validate_gps_config(&self, config: &GpsConfig) -> Result<(), ConfigError>
    pub fn validate_power_config(&self, config: &PowerConfig) -> Result<(), ConfigError>
    pub fn validate_communication_config(&self, config: &CommunicationConfig) -> Result<(), ConfigError>
    pub fn validate_emergency_config(&self, config: &EmergencyConfig) -> Result<(), ConfigError>
    pub fn validate_hardware_config(&self, config: &HardwareConfig) -> Result<(), ConfigError>
}
```

## Status and Monitoring

### BeaconStatus

Comprehensive beacon status information:

```rust
pub struct BeaconStatus {
    pub beacon_id: Uuid,
    pub operational_state: OperationalState,
    pub gps_status: GpsStatus,
    pub battery_status: BatteryStatus,
    pub communication_status: CommunicationStatus,
    pub transmission_stats: TransmissionStats,
    pub uptime: Duration,
    pub last_error: Option<BeaconError>,
    pub current_position: Option<GpsPosition>,
    pub system_health: SystemHealth,
    pub environmental_conditions: Option<EnvironmentalConditions>,
}
```

### Operational States

```rust
pub enum OperationalState {
    Initializing,
    GpsAcquisition,
    Normal,
    PowerSave,
    Emergency,
    Shutdown,
    Error(String),
}
```

### Status Monitoring Methods

```rust
impl<G, P, C, T> BeaconController<G, P, C, T> {
    pub fn get_status(&self) -> BeaconStatus
    pub fn get_detailed_status(&self) -> DetailedBeaconStatus
    pub fn get_gps_status(&self) -> GpsStatus
    pub fn get_power_status(&self) -> BatteryStatus
    pub fn get_communication_status(&self) -> CommunicationStatus
    pub fn get_transmission_statistics(&self) -> TransmissionStats
    pub fn get_system_health(&self) -> SystemHealth
    pub fn get_error_history(&self) -> Vec<BeaconError>
}
```

**Usage Example:**
```rust
// Get basic status
let status = beacon.get_status();
println!("Beacon {} is in state: {:?}", status.beacon_id, status.operational_state);

// Check specific subsystems
let gps_status = beacon.get_gps_status();
if gps_status.is_locked {
    println!("GPS locked with {} satellites", gps_status.satellite_count);
}

let battery_status = beacon.get_power_status();
if battery_status.capacity_percent < 20.0 {
    println!("Low battery warning: {:.1}%", battery_status.capacity_percent);
}
```

### System Health Monitoring

```rust
pub struct SystemHealth {
    pub overall_health_score: f64, // 0.0 to 1.0
    pub component_health: HashMap<String, ComponentHealth>,
    pub active_alerts: Vec<HealthAlert>,
    pub performance_metrics: PerformanceMetrics,
    pub resource_usage: ResourceUsage,
}

pub struct ComponentHealth {
    pub status: ComponentStatus,
    pub health_score: f64,
    pub last_check: SystemTime,
    pub error_count: u32,
    pub uptime: Duration,
}

pub enum ComponentStatus {
    Healthy,
    Warning,
    Critical,
    Failed,
    Unknown,
}
```

## Emergency Handling

### Emergency Types

```rust
pub enum EmergencyType {
    BatteryDepleted,
    HardwareFault,
    CommunicationLost,
    GpsSignalLost,
    TemperatureExtreme,
    SystemOverload,
    ManualEmergency,
}
```

### Emergency Response

```rust
impl<G, P, C, T> BeaconController<G, P, C, T> {
    pub fn handle_emergency(&mut self, emergency_type: EmergencyType) -> Result<(), BeaconError>
    pub fn declare_emergency(&mut self, emergency_type: EmergencyType, message: String) -> Result<(), BeaconError>
    pub fn clear_emergency(&mut self) -> Result<(), BeaconError>
    pub fn get_emergency_status(&self) -> Option<EmergencyStatus>
    pub fn configure_emergency_response(&mut self, config: EmergencyConfig) -> Result<(), BeaconError>
}
```

### Emergency Configuration

```rust
pub struct EmergencyConfig {
    pub emergency_transmission_interval_ms: u32,
    pub emergency_power_threshold_percent: f32,
    pub emergency_gps_timeout_s: u32,
    pub emergency_communication_timeout_s: u32,
    pub auto_shutdown_enabled: bool,
    pub emergency_message_count: u32,
    pub shutdown_delay_s: u32,
    pub emergency_contacts: Vec<String>,
}
```

**Usage Example:**
```rust
// Handle battery depletion emergency
if battery_status.capacity_percent < 5.0 {
    beacon.handle_emergency(EmergencyType::BatteryDepleted)?;
}

// Manual emergency declaration
beacon.declare_emergency(
    EmergencyType::ManualEmergency,
    "Beacon requires immediate attention".to_string()
)?;

// Check emergency status
if let Some(emergency) = beacon.get_emergency_status() {
    println!("Emergency active: {:?}", emergency.emergency_type);
    println!("Duration: {:?}", emergency.duration);
}
```

## Lifecycle Management

### Startup Sequence

```rust
impl<G, P, C, T> BeaconController<G, P, C, T> {
    pub fn start(&mut self) -> Result<(), BeaconError>
    pub fn initialize_subsystems(&mut self) -> Result<(), BeaconError>
    pub fn perform_self_test(&mut self) -> Result<SelfTestResults, BeaconError>
    pub fn begin_operation(&mut self) -> Result<(), BeaconError>
}
```

### Shutdown Sequence

```rust
impl<G, P, C, T> BeaconController<G, P, C, T> {
    pub fn stop(&mut self) -> Result<(), BeaconError>
    pub fn graceful_shutdown(&mut self, delay: Duration) -> Result<(), BeaconError>
    pub fn emergency_shutdown(&mut self) -> Result<(), BeaconError>
    pub fn prepare_for_shutdown(&mut self) -> Result<(), BeaconError>
}
```

### State Transitions

```rust
impl<G, P, C, T> BeaconController<G, P, C, T> {
    pub fn transition_to_state(&mut self, target_state: OperationalState) -> Result<(), BeaconError>
    pub fn can_transition_to(&self, target_state: OperationalState) -> bool
    pub fn get_valid_transitions(&self) -> Vec<OperationalState>
}
```

**Usage Example:**
```rust
// Start beacon with full initialization
beacon.start()?;

// Perform self-test
let test_results = beacon.perform_self_test()?;
if !test_results.all_passed() {
    println!("Self-test failures: {:?}", test_results.failures);
}

// Graceful shutdown with 30-second delay
beacon.graceful_shutdown(Duration::from_secs(30))?;
```

### Self-Test Results

```rust
pub struct SelfTestResults {
    pub gps_test: TestResult,
    pub power_test: TestResult,
    pub communication_test: TestResult,
    pub transceiver_test: TestResult,
    pub memory_test: TestResult,
    pub configuration_test: TestResult,
    pub overall_result: TestResult,
}

pub enum TestResult {
    Passed,
    Failed(String),
    Warning(String),
    Skipped(String),
}

impl SelfTestResults {
    pub fn all_passed(&self) -> bool
    pub fn has_failures(&self) -> bool
    pub fn get_failures(&self) -> Vec<&TestResult>
    pub fn get_warnings(&self) -> Vec<&TestResult>
}
```

## Error Types

### BeaconError

Comprehensive error enumeration for beacon operations:

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
    pub recovery_attempted: bool,
    pub recovery_strategy: Option<RecoveryStrategy>,
}
```

### Error Recovery

```rust
impl<G, P, C, T> BeaconController<G, P, C, T> {
    pub fn recover_from_error(&mut self, error: &BeaconError) -> Result<(), BeaconError>
    pub fn get_recovery_strategy(&self, error: &BeaconError) -> RecoveryStrategy
    pub fn attempt_automatic_recovery(&mut self) -> Result<(), BeaconError>
    pub fn reset_error_state(&mut self) -> Result<(), BeaconError>
}
```

**Usage Example:**
```rust
// Handle errors with automatic recovery
match beacon.start() {
    Ok(()) => println!("Beacon started successfully"),
    Err(error) => {
        println!("Startup error: {}", error);
        
        // Attempt recovery
        if let Ok(()) = beacon.recover_from_error(&error) {
            println!("Recovery successful, retrying startup");
            beacon.start()?;
        } else {
            println!("Recovery failed, manual intervention required");
        }
    }
}
```

## Advanced Features

### Diagnostic Capabilities

```rust
impl<G, P, C, T> BeaconController<G, P, C, T> {
    pub fn run_diagnostics(&mut self) -> Result<DiagnosticReport, BeaconError>
    pub fn run_component_diagnostic(&mut self, component: HardwareComponent) -> Result<ComponentDiagnostic, BeaconError>
    pub fn get_performance_metrics(&self) -> PerformanceMetrics
    pub fn get_resource_usage(&self) -> ResourceUsage
}
```

### Performance Monitoring

```rust
pub struct PerformanceMetrics {
    pub cpu_usage_percent: f32,
    pub memory_usage_bytes: usize,
    pub memory_usage_percent: f32,
    pub transmission_success_rate: f64,
    pub gps_acquisition_time_ms: u32,
    pub average_power_consumption_mw: f32,
    pub uptime: Duration,
    pub error_rate_per_hour: f64,
}
```

### Environmental Adaptation

```rust
impl<G, P, C, T> BeaconController<G, P, C, T> {
    pub fn adapt_to_conditions(&mut self, conditions: EnvironmentalConditions) -> Result<Vec<AdaptationAction>, BeaconError>
    pub fn get_environmental_status(&self) -> Option<EnvironmentalStatus>
    pub fn configure_environmental_thresholds(&mut self, thresholds: EnvironmentalThresholds) -> Result<(), BeaconError>
}
```

## Usage Examples

### Complete Beacon Setup and Operation

```rust
use beacon::{BeaconController, BeaconConfig};
use shared_positioning::*;
use uuid::Uuid;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Create configuration
    let beacon_config = BeaconConfig {
        beacon_id: Uuid::new_v4(),
        transmission_config: TransmissionConfig {
            interval_ms: 5000,
            power_level: 80,
            message_version: MessageVersion::V3,
            retry_attempts: 3,
        },
        gps_config: GpsConfig {
            acquisition_timeout_s: 60,
            update_interval_s: 30,
            min_satellite_count: 4,
            accuracy_threshold_m: 5.0,
            cold_start_timeout_s: 300,
            power_save_enabled: true,
        },
        power_config: PowerConfig {
            low_battery_threshold_percent: 20.0,
            critical_battery_threshold_percent: 10.0,
            emergency_battery_threshold_percent: 5.0,
            power_save_mode_threshold_percent: 30.0,
            charging_enabled: true,
            solar_charging_enabled: false,
            temperature_monitoring_enabled: true,
            max_charging_current_ma: 500.0,
        },
        communication_config: CommunicationConfig {
            connection_interval_hours: 24,
            retry_attempts: 3,
            retry_backoff_ms: 5000,
            max_retry_interval_hours: 6,
            connection_timeout_s: 60,
            data_compression_enabled: true,
            encryption_enabled: true,
            max_report_size_bytes: 1024 * 1024,
        },
        emergency_config: EmergencyConfig {
            emergency_transmission_interval_ms: 1000,
            emergency_power_threshold_percent: 5.0,
            emergency_gps_timeout_s: 300,
            emergency_communication_timeout_s: 1800,
            auto_shutdown_enabled: true,
            emergency_message_count: 10,
            shutdown_delay_s: 30,
            emergency_contacts: vec!["emergency@example.com".to_string()],
        },
        hardware_config: HardwareConfig {
            gpio_pin_mapping: default_gpio_mapping(),
            spi_config: default_spi_config(),
            i2c_config: default_i2c_config(),
            uart_config: default_uart_config(),
            power_management_enabled: true,
            watchdog_enabled: true,
            watchdog_timeout_s: 60,
        },
    };
    
    // Create subsystem implementations
    let gps_manager = BasicGpsManager::new(beacon_config.gps_config.clone());
    let power_manager = BasicPowerManager::new(beacon_config.power_config.clone());
    let comm_manager = BasicCommunicationManager::new(beacon_config.communication_config.clone());
    let transceiver = SerialTransceiver::new("/dev/ttyUSB0")?;
    
    // Create beacon controller
    let mut beacon = BeaconController::new(
        beacon_config,
        gps_manager,
        power_manager,
        comm_manager,
        transceiver,
    )?;
    
    // Perform self-test
    let test_results = beacon.perform_self_test()?;
    if !test_results.all_passed() {
        println!("Self-test warnings/failures:");
        for failure in test_results.get_failures() {
            println!("  FAIL: {:?}", failure);
        }
        for warning in test_results.get_warnings() {
            println!("  WARN: {:?}", warning);
        }
    }
    
    // Start beacon operation
    beacon.start()?;
    println!("Beacon started successfully");
    
    // Main monitoring loop
    loop {
        let status = beacon.get_status();
        
        // Check operational state
        match status.operational_state {
            OperationalState::Normal => {
                // Normal operation - check for issues
                if let Some(position) = status.current_position {
                    println!("Position: {:.6}, {:.6} (±{:.1}m, {} sats)",
                        position.latitude, position.longitude, 
                        position.accuracy_m, position.satellite_count);
                }
                
                // Check battery level
                if status.battery_status.capacity_percent < 20.0 {
                    println!("Low battery: {:.1}%", status.battery_status.capacity_percent);
                }
            },
            OperationalState::PowerSave => {
                println!("Beacon in power save mode");
            },
            OperationalState::Emergency => {
                println!("EMERGENCY STATE ACTIVE");
                if let Some(emergency) = beacon.get_emergency_status() {
                    println!("Emergency type: {:?}", emergency.emergency_type);
                }
            },
            OperationalState::Error(ref error_msg) => {
                println!("Beacon error: {}", error_msg);
                
                // Attempt recovery
                if let Err(recovery_error) = beacon.attempt_automatic_recovery() {
                    println!("Recovery failed: {}", recovery_error);
                } else {
                    println!("Recovery successful");
                }
            },
            _ => {
                println!("Beacon state: {:?}", status.operational_state);
            }
        }
        
        // Check system health
        let health = beacon.get_system_health();
        if health.overall_health_score < 0.8 {
            println!("System health degraded: {:.1}%", health.overall_health_score * 100.0);
            for alert in &health.active_alerts {
                println!("  Alert: {} - {}", alert.severity, alert.message);
            }
        }
        
        // Sleep before next status check
        std::thread::sleep(Duration::from_secs(10));
    }
}
```

### Configuration Update Example

```rust
use beacon::{BeaconController, BeaconConfig};
use std::path::Path;

fn update_beacon_configuration(
    beacon: &mut BeaconController<impl GpsManager, impl PowerManager, impl CommunicationManager, impl TransceiverInterface>,
    config_path: &Path
) -> Result<(), Box<dyn std::error::Error>> {
    // Load new configuration from file
    let config_manager = ConfigManager::new(config_path.to_path_buf());
    let new_config = config_manager.load_config().await?;
    
    // Validate configuration
    beacon.validate_configuration(&new_config)?;
    
    // Create backup of current configuration
    let current_config = beacon.get_configuration().clone();
    let backup = config_manager.create_backup(&current_config);
    
    // Apply new configuration
    match beacon.update_configuration(new_config) {
        Ok(()) => {
            println!("Configuration updated successfully");
            Ok(())
        },
        Err(error) => {
            println!("Configuration update failed: {}", error);
            
            // Restore from backup
            let restored_config = config_manager.restore_from_backup(&backup);
            beacon.update_configuration(restored_config)?;
            println!("Configuration restored from backup");
            
            Err(error.into())
        }
    }
}
```

### Emergency Handling Example

```rust
use beacon::{BeaconController, EmergencyType};
use std::time::Duration;

fn monitor_beacon_health(
    beacon: &mut BeaconController<impl GpsManager, impl PowerManager, impl CommunicationManager, impl TransceiverInterface>
) -> Result<(), Box<dyn std::error::Error>> {
    loop {
        let status = beacon.get_status();
        
        // Check for emergency conditions
        
        // Battery emergency
        if status.battery_status.capacity_percent < 5.0 {
            beacon.handle_emergency(EmergencyType::BatteryDepleted)?;
        }
        
        // GPS signal loss
        if let Some(gps_status) = status.gps_status {
            if !gps_status.is_locked && gps_status.time_since_last_fix > Duration::from_secs(300) {
                beacon.handle_emergency(EmergencyType::GpsSignalLost)?;
            }
        }
        
        // Temperature extreme
        if status.battery_status.temperature_c > 60.0 || status.battery_status.temperature_c < -20.0 {
            beacon.handle_emergency(EmergencyType::TemperatureExtreme)?;
        }
        
        // Communication loss
        if let Some(comm_status) = status.communication_status {
            if comm_status.time_since_last_contact > Duration::from_hours(48) {
                beacon.handle_emergency(EmergencyType::CommunicationLost)?;
            }
        }
        
        // System health check
        let health = beacon.get_system_health();
        if health.overall_health_score < 0.5 {
            beacon.declare_emergency(
                EmergencyType::SystemOverload,
                format!("System health critically low: {:.1}%", health.overall_health_score * 100.0)
            )?;
        }
        
        std::thread::sleep(Duration::from_secs(30));
    }
}
```

This comprehensive API documentation provides developers with all the information needed to integrate, configure, and monitor beacon controllers effectively. The examples demonstrate practical usage patterns for common scenarios including startup, configuration management, status monitoring, and emergency handling.