// Beacon controller and main orchestration
// Manages beacon lifecycle, operational state, and coordinates all subsystems

use std::time::{Duration, SystemTime, Instant};
use std::sync::mpsc::{self, Receiver, Sender};
use std::thread;
use uuid::Uuid;
use serde::{Serialize, Deserialize};

use shared_positioning::{
    GpsManager, PowerManager, CommunicationManager, TransceiverInterface,
    MessageBuilder, GeodeticPosition, GpsPosition, GpsConfig, GpsError, GpsStatus,
    PowerConfig, PowerError, PowerOperationMode, BatteryStatus,
    CommunicationConfig, StatusReport, SystemHealth, CommTransmissionStats,
    ErrorLogEntry, CommErrorSeverity, CommError, TransmissionManager,
    TransmissionConfig, TransmissionError,
    TransmissionStatistics, TransmissionMessageVersion, TransmissionPriority,
    EnvironmentalConditions,
    // Enhanced error handling imports
    BeaconError, BeaconErrorContext, DiagnosticSystemManager, HealthAlert,
    GpsErrorType, PowerErrorType, CommunicationErrorType, TransmissionErrorType,
    ConfigurationErrorType, SystemErrorType, HardwareComponent, HardwareFaultType,
    BatteryStatusSnapshot, BeaconBatteryHealth, BeaconSystemState, ResourceUsageSnapshot,
    GpsStatusSnapshot, CommunicationStatusSnapshot, TransmissionStatusSnapshot,
    EnvironmentalMetrics, ConsoleLogHandler, StructuredFileLogHandler,
    ErrorSeverity, RecoveryStrategy, BeaconPowerMode, BeaconChargingStatus,
    // Environmental and reliability monitoring imports
    EnvironmentalMonitor, ExtendedEnvironmentalConditions, EnvironmentalThresholds,
    EnvironmentalError, AdaptationAction, EnvironmentalStats,
    HardwareMonitor, HardwareMonitorConfig, HardwareMonitorStats, DiagnosticResult,
    ComponentHealth, RecommendedAction, HardwareFaultError,
    ReliabilityMonitor, ReliabilityMetrics, ReliabilityThresholds, ReliabilityReport,
    FailureEvent, ReliabilityError, HealthTrend
};




/// Configuration-specific error types
#[derive(Debug, Clone)]
pub enum ConfigError {
    InvalidParameter(String),
    ValidationFailed(String),
    UpdateFailed(String),
}

impl std::fmt::Display for ConfigError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConfigError::InvalidParameter(msg) => write!(f, "Invalid parameter: {}", msg),
            ConfigError::ValidationFailed(msg) => write!(f, "Validation failed: {}", msg),
            ConfigError::UpdateFailed(msg) => write!(f, "Update failed: {}", msg),
        }
    }
}

impl std::error::Error for ConfigError {}

/// System-specific error types
#[derive(Debug, Clone)]
pub enum SystemError {
    InitializationFailed(String),
    ResourceExhausted(String),
    HardwareFault(String),
    ThreadPanic(String),
}

impl std::fmt::Display for SystemError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SystemError::InitializationFailed(msg) => write!(f, "Initialization failed: {}", msg),
            SystemError::ResourceExhausted(msg) => write!(f, "Resource exhausted: {}", msg),
            SystemError::HardwareFault(msg) => write!(f, "Hardware fault: {}", msg),
            SystemError::ThreadPanic(msg) => write!(f, "Thread panic: {}", msg),
        }
    }
}

impl std::error::Error for SystemError {}

/// Emergency types that can trigger emergency handling
#[derive(Debug, Clone)]
pub enum EmergencyType {
    BatteryDepleted,
    HardwareFault,
    CommunicationLost,
    GpsSignalLost,
    TemperatureExtreme,
    SystemOverload,
}

/// Operational states for the beacon
#[derive(Debug, Clone, PartialEq)]
pub enum OperationalState {
    Initializing,
    GpsAcquisition,
    Normal,
    PowerSave,
    Emergency,
    Shutdown,
    Error(String),
}

/// Beacon configuration parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BeaconConfig {
    pub beacon_id: Uuid,
    pub transmission_interval_ms: u32,
    pub message_version: MessageVersion,
    pub gps_config: GpsConfig,
    pub power_config: PowerConfig,
    pub communication_config: CommunicationConfig,
    pub emergency_config: EmergencyConfig,
}

/// Message version selection
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum MessageVersion {
    V1,
    V2,
    V3,
}

/// Emergency handling configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EmergencyConfig {
    pub emergency_transmission_interval_ms: u32,
    pub emergency_power_threshold_percent: f32,
    pub emergency_gps_timeout_s: u32,
    pub emergency_communication_timeout_s: u32,
    pub auto_shutdown_enabled: bool,
    pub emergency_message_count: u32,
    pub shutdown_delay_s: u32,
}

impl Default for EmergencyConfig {
    fn default() -> Self {
        Self {
            emergency_transmission_interval_ms: 5000, // 5 seconds
            emergency_power_threshold_percent: 5.0,
            emergency_gps_timeout_s: 300, // 5 minutes
            emergency_communication_timeout_s: 1800, // 30 minutes
            auto_shutdown_enabled: true,
            emergency_message_count: 10,
            shutdown_delay_s: 30, // 30 seconds
        }
    }
}

impl Default for BeaconConfig {
    fn default() -> Self {
        Self {
            beacon_id: Uuid::new_v4(),
            transmission_interval_ms: 5000, // 5 seconds
            message_version: MessageVersion::V3,
            gps_config: GpsConfig::default(),
            power_config: PowerConfig::default(),
            communication_config: CommunicationConfig::default(),
            emergency_config: EmergencyConfig::default(),
        }
    }
}

/// Communication status information
#[derive(Debug, Clone)]
pub struct CommunicationStatus {
    pub is_connected: bool,
    pub signal_strength: Option<u8>,
    pub last_successful_contact: Option<SystemTime>,
}

/// Beacon status information
#[derive(Debug, Clone)]
pub struct BeaconStatus {
    pub beacon_id: Uuid,
    pub operational_state: OperationalState,
    pub gps_status: GpsStatus,
    pub current_position: Option<GpsPosition>,
    pub battery_status: BatteryStatus,
    pub communication_status: CommunicationStatus,
    pub transmission_stats: CommTransmissionStats,
    pub uptime: Duration,
    pub last_error: Option<BeaconError>,
    pub system_health: SystemHealth,
}

/// Control messages for beacon operation
#[derive(Debug)]
enum ControlMessage {
    Start,
    Stop,
    Emergency(EmergencyType),
    UpdateConfig(BeaconConfig),
    GetStatus,
    Shutdown,
}

/// Main beacon controller that orchestrates all subsystems
pub struct BeaconController<G, P, C, T> 
where
    G: GpsManager + Send + 'static,
    P: PowerManager + Send + 'static,
    C: CommunicationManager + Send + 'static,
    T: TransceiverInterface + Send + 'static,
{
    config: BeaconConfig,
    operational_state: OperationalState,
    gps_manager: G,
    power_manager: P,
    communication_manager: C,
    transceiver: T,
    transmission_config: TransmissionConfig,
    transmission_statistics: TransmissionStatistics,
    start_time: SystemTime,
    last_transmission: Option<SystemTime>,
    last_gps_update: Option<SystemTime>,
    last_communication: Option<SystemTime>,
    transmission_sequence: u16,
    current_power_level: u8,
    environmental_conditions: EnvironmentalConditions,
    error_log: Vec<ErrorLogEntry>,
    control_sender: Option<Sender<ControlMessage>>,
    control_receiver: Option<Receiver<ControlMessage>>,
    running: bool,
    // Enhanced error handling and diagnostics
    diagnostic_system: DiagnosticSystemManager,
    last_diagnostic_report: Option<SystemTime>,
    // Environmental monitoring and adaptation
    environmental_monitor: EnvironmentalMonitor,
    last_environmental_update: Option<SystemTime>,
    // Hardware fault detection and recovery
    hardware_monitor: HardwareMonitor,
    last_hardware_diagnostic: Option<SystemTime>,
    // System reliability monitoring
    reliability_monitor: ReliabilityMonitor,
    last_reliability_report: Option<SystemTime>,
}

impl<G, P, C, T> BeaconController<G, P, C, T>
where
    G: GpsManager + Send + 'static,
    P: PowerManager + Send + 'static,
    C: CommunicationManager + Send + 'static,
    T: TransceiverInterface + Send + 'static,
{
    /// Create new beacon controller with configuration
    pub fn new(
        config: BeaconConfig,
        gps_manager: G,
        power_manager: P,
        communication_manager: C,
        transceiver: T,
    ) -> Result<Self, BeaconError> {
        // Validate configuration
        Self::validate_config(&config)?;
        
        let (sender, receiver) = mpsc::channel();
        
        // Initialize diagnostic system with logging handlers
        let mut diagnostic_system = DiagnosticSystemManager::new(100);
        
        // Add console logging handler
        let console_handler = ConsoleLogHandler::new(ErrorSeverity::Warning);
        diagnostic_system.add_output_handler(Box::new(console_handler));
        
        // Add structured file logging handler
        let file_handler = StructuredFileLogHandler::new(
            format!("beacon_{}_errors.log", config.beacon_id),
            ErrorSeverity::Info,
            50
        );
        diagnostic_system.add_output_handler(Box::new(file_handler));
        
        Ok(Self {
            config,
            operational_state: OperationalState::Initializing,
            gps_manager,
            power_manager,
            communication_manager,
            transceiver,
            transmission_config: TransmissionConfig::default(),
            transmission_statistics: TransmissionStatistics::default(),
            start_time: SystemTime::now(),
            last_transmission: None,
            last_gps_update: None,
            last_communication: None,
            transmission_sequence: 0,
            current_power_level: 128,
            environmental_conditions: EnvironmentalConditions::default(),
            error_log: Vec::new(),
            control_sender: Some(sender),
            control_receiver: Some(receiver),
            running: false,
            diagnostic_system,
            last_diagnostic_report: None,
            // Initialize environmental monitoring
            environmental_monitor: EnvironmentalMonitor::new(EnvironmentalThresholds::default()),
            last_environmental_update: None,
            // Initialize hardware monitoring
            hardware_monitor: HardwareMonitor::new(HardwareMonitorConfig::default()),
            last_hardware_diagnostic: None,
            // Initialize reliability monitoring
            reliability_monitor: ReliabilityMonitor::new(ReliabilityThresholds::default()),
            last_reliability_report: None,
        })
    }
    
    /// Validate beacon configuration
    fn validate_config(config: &BeaconConfig) -> Result<(), BeaconError> {
        if config.transmission_interval_ms < 1000 || config.transmission_interval_ms > 60000 {
            return Err(BeaconError::ConfigurationError {
                error_type: ConfigurationErrorType::ParameterOutOfRange,
                parameter_name: "transmission_interval_ms".to_string(),
                current_value: config.transmission_interval_ms.to_string(),
                expected_value: "1000-60000".to_string(),
            });
        }
        
        if config.emergency_config.emergency_transmission_interval_ms < 1000 {
            return Err(BeaconError::ConfigurationError {
                error_type: ConfigurationErrorType::ParameterOutOfRange,
                parameter_name: "emergency_transmission_interval_ms".to_string(),
                current_value: config.emergency_config.emergency_transmission_interval_ms.to_string(),
                expected_value: ">=1000".to_string(),
            });
        }
        
        if config.emergency_config.emergency_power_threshold_percent < 1.0 || 
           config.emergency_config.emergency_power_threshold_percent > 20.0 {
            return Err(BeaconError::ConfigurationError {
                error_type: ConfigurationErrorType::ParameterOutOfRange,
                parameter_name: "emergency_power_threshold_percent".to_string(),
                current_value: config.emergency_config.emergency_power_threshold_percent.to_string(),
                expected_value: "1.0-20.0".to_string(),
            });
        }
        
        Ok(())
    }
    
    /// Start beacon operation
    pub fn start(&mut self) -> Result<(), BeaconError> {
        if self.running {
            return Ok(());
        }
        
        self.log_info("Starting beacon controller");
        self.operational_state = OperationalState::Initializing;
        
        // Initialize GPS manager
        self.gps_manager.configure(self.config.gps_config.clone())?;
        self.gps_manager.start_acquisition()?;
        self.operational_state = OperationalState::GpsAcquisition;
        
        // Configure power manager
        self.power_manager.configure_power_thresholds(self.config.power_config.clone())?;
        
        // Configure communication manager
        self.communication_manager.configure(self.config.communication_config.clone())?;
        
        self.running = true;
        self.start_time = SystemTime::now();
        
        // Start main control loop
        self.run_control_loop()?;
        
        Ok(())
    }
    
    /// Stop beacon operation
    pub fn stop(&mut self) -> Result<(), BeaconError> {
        if !self.running {
            return Ok(());
        }
        
        self.log_info("Stopping beacon controller");
        self.running = false;
        
        // Stop GPS acquisition
        self.gps_manager.stop()?;
        
        // Disconnect communication
        let _ = self.communication_manager.disconnect();
        
        self.operational_state = OperationalState::Shutdown;
        
        Ok(())
    }
    
    /// Update beacon configuration
    pub fn update_configuration(&mut self, config: BeaconConfig) -> Result<(), BeaconError> {
        // Validate new configuration
        Self::validate_config(&config)?;
        
        self.log_info("Updating beacon configuration");
        
        // Update GPS configuration if changed
        if config.gps_config != self.config.gps_config {
            self.gps_manager.configure(config.gps_config.clone())?;
        }
        
        // Update power configuration if changed
        if config.power_config != self.config.power_config {
            self.power_manager.configure_power_thresholds(config.power_config.clone())?;
        }
        
        // Update communication configuration if changed
        if config.communication_config != self.config.communication_config {
            self.communication_manager.configure(config.communication_config.clone())?;
        }
        
        self.config = config;
        
        Ok(())
    }
    
    /// Get current beacon configuration
    pub fn get_config(&self) -> &BeaconConfig {
        &self.config
    }
    
    /// Get current beacon status
    pub fn get_status(&self) -> BeaconStatus {
        let battery_status = self.power_manager.get_battery_status()
            .unwrap_or_else(|_| BatteryStatus::new(0.0, 0.0, 0.0, 0.0));
        
        let uptime = SystemTime::now()
            .duration_since(self.start_time)
            .unwrap_or(Duration::from_secs(0));
        
        let current_position = self.gps_manager.get_current_position();
        
        let communication_status = CommunicationStatus {
            is_connected: self.communication_manager.is_connected(),
            signal_strength: self.communication_manager.get_signal_strength(),
            last_successful_contact: self.last_communication,
        };
        
        BeaconStatus {
            beacon_id: self.config.beacon_id,
            operational_state: self.operational_state.clone(),
            gps_status: self.gps_manager.get_status(),
            current_position,
            battery_status,
            communication_status,
            transmission_stats: self.get_transmission_stats(),
            uptime,
            last_error: None, // TODO: Track last error
            system_health: self.get_system_health(),
        }
    }
    
    /// Handle emergency situations
    pub fn handle_emergency(&mut self, emergency_type: EmergencyType) -> Result<(), BeaconError> {
        self.log_error(&format!("Emergency triggered: {:?}", emergency_type));
        
        match emergency_type {
            EmergencyType::BatteryDepleted => {
                self.power_manager.set_power_mode(PowerOperationMode::Emergency)?;
                self.operational_state = OperationalState::Emergency;
                
                // Send emergency messages
                self.send_emergency_messages()?;
                
                if self.config.emergency_config.auto_shutdown_enabled {
                    self.prepare_emergency_shutdown()?;
                }
            }
            EmergencyType::HardwareFault => {
                self.operational_state = OperationalState::Error("Hardware fault".to_string());
                self.send_emergency_messages()?;
            }
            EmergencyType::CommunicationLost => {
                // Continue autonomous operation
                self.log_warning("Communication lost - continuing autonomous operation");
            }
            EmergencyType::GpsSignalLost => {
                // Continue with last known position
                self.log_warning("GPS signal lost - using last known position");
            }
            EmergencyType::TemperatureExtreme => {
                self.power_manager.set_power_mode(PowerOperationMode::PowerSave)?;
                self.log_warning("Temperature extreme - entering power save mode");
            }
            EmergencyType::SystemOverload => {
                self.power_manager.set_power_mode(PowerOperationMode::PowerSave)?;
                self.operational_state = OperationalState::PowerSave;
            }
        }
        
        Ok(())
    }
    
    /// Main control loop that coordinates all subsystems
    fn run_control_loop(&mut self) -> Result<(), BeaconError> {
        let mut last_gps_update = Instant::now();
        let mut last_transmission = Instant::now();
        let mut last_power_check = Instant::now();
        let mut last_communication_check = Instant::now();
        let mut last_health_check = Instant::now();
        let mut last_diagnostic_report = Instant::now();
        let mut last_environmental_check = Instant::now();
        let mut last_hardware_diagnostic = Instant::now();
        let mut last_reliability_check = Instant::now();
        
        while self.running {
            let now = Instant::now();
            
            // Update GPS manager
            if now.duration_since(last_gps_update) >= Duration::from_secs(1) {
                if let Err(e) = self.update_gps() {
                    self.log_error(&format!("GPS update failed: {}", e));
                }
                last_gps_update = now;
            }
            
            // Check power status
            if now.duration_since(last_power_check) >= Duration::from_secs(5) {
                if let Err(e) = self.check_power_status() {
                    self.log_error(&format!("Power check failed: {}", e));
                }
                last_power_check = now;
            }
            
            // Handle transmissions
            let transmission_interval = if self.operational_state == OperationalState::Emergency {
                Duration::from_millis(self.config.emergency_config.emergency_transmission_interval_ms as u64)
            } else {
                Duration::from_millis(self.config.transmission_interval_ms as u64)
            };
            
            if now.duration_since(last_transmission) >= transmission_interval {
                if let Err(e) = self.handle_transmission() {
                    self.log_error(&format!("Transmission failed: {}", e));
                }
                last_transmission = now;
            }
            
            // Check communication (less frequently)
            if now.duration_since(last_communication_check) >= Duration::from_secs(60) {
                if let Err(e) = self.check_communication() {
                    self.log_warning(&format!("Communication check failed: {}", e));
                }
                last_communication_check = now;
            }
            
            // Check system health periodically
            if now.duration_since(last_health_check) >= Duration::from_secs(30) {
                self.check_system_health();
                last_health_check = now;
            }
            
            // Generate diagnostic report periodically
            if now.duration_since(last_diagnostic_report) >= Duration::from_secs(300) { // Every 5 minutes
                let report = self.generate_diagnostic_report();
                self.last_diagnostic_report = Some(SystemTime::now());
                
                // Log key metrics from the report
                self.log_info(&format!(
                    "Diagnostic report: Health score: {:.2}, Error rate: {:.1}/hr, Battery: {:.1}%",
                    report.system_health.overall_health_score,
                    report.error_statistics.error_rate_per_hour,
                    report.system_health.power_health.estimated_runtime_hours
                ));
                
                last_diagnostic_report = now;
            }

            // Environmental monitoring and adaptation
            if now.duration_since(last_environmental_check) >= Duration::from_secs(30) { // Every 30 seconds
                if let Err(e) = self.update_environmental_monitoring() {
                    self.log_warning(&format!("Environmental monitoring failed: {}", e));
                }
                last_environmental_check = now;
            }

            // Hardware diagnostics
            if now.duration_since(last_hardware_diagnostic) >= Duration::from_secs(120) { // Every 2 minutes
                if let Err(e) = self.run_hardware_diagnostics() {
                    self.log_error(&format!("Hardware diagnostics failed: {}", e));
                }
                last_hardware_diagnostic = now;
            }

            // Reliability monitoring
            if now.duration_since(last_reliability_check) >= Duration::from_secs(600) { // Every 10 minutes
                if let Err(e) = self.update_reliability_monitoring() {
                    self.log_warning(&format!("Reliability monitoring failed: {}", e));
                }
                last_reliability_check = now;
            }
            
            // Process control messages
            if let Some(receiver) = &self.control_receiver {
                if let Ok(message) = receiver.try_recv() {
                    self.handle_control_message(message)?;
                }
            }
            
            // Sleep briefly to prevent busy waiting
            thread::sleep(Duration::from_millis(100));
        }
        
        Ok(())
    }
    
    /// Update GPS status and handle state transitions
    fn update_gps(&mut self) -> Result<(), BeaconError> {
        if let Err(gps_error) = self.gps_manager.update() {
            let beacon_error = BeaconError::GpsError {
                error_type: GpsErrorType::HardwareFault,
                last_known_position: self.gps_manager.get_current_position()
                    .map(|pos| (pos.latitude, pos.longitude, pos.altitude, SystemTime::now())),
                satellite_count: self.gps_manager.get_satellite_count(),
                signal_strength: None,
            };
            
            if let Some(recovery_strategy) = self.log_beacon_error(beacon_error.clone()) {
                self.apply_recovery_strategy(recovery_strategy)?;
            }
            
            return Err(beacon_error);
        }
        
        let gps_status = self.gps_manager.get_status();
        
        match gps_status {
            GpsStatus::Locked => {
                if self.operational_state == OperationalState::GpsAcquisition {
                    self.operational_state = OperationalState::Normal;
                    self.log_info("GPS acquired - entering normal operation");
                }
                self.last_gps_update = Some(SystemTime::now());
            }
            GpsStatus::SignalLost => {
                if let Some(last_update) = self.last_gps_update {
                    let time_since_update = SystemTime::now()
                        .duration_since(last_update)
                        .unwrap_or(Duration::from_secs(0));
                    
                    if time_since_update > Duration::from_secs(self.config.emergency_config.emergency_gps_timeout_s as u64) {
                        let beacon_error = BeaconError::GpsError {
                            error_type: GpsErrorType::SignalLost,
                            last_known_position: self.gps_manager.get_current_position()
                                .map(|pos| (pos.latitude, pos.longitude, pos.altitude, last_update)),
                            satellite_count: self.gps_manager.get_satellite_count(),
                            signal_strength: None,
                        };
                        
                        if let Some(recovery_strategy) = self.log_beacon_error(beacon_error) {
                            self.apply_recovery_strategy(recovery_strategy)?;
                        }
                        
                        self.handle_emergency(EmergencyType::GpsSignalLost)?;
                    }
                }
            }
            GpsStatus::HardwareFault => {
                let beacon_error = BeaconError::HardwareError {
                    component: HardwareComponent::GpsReceiver,
                    fault_type: HardwareFaultType::ComponentFailure,
                    diagnostic_data: vec![],
                    recovery_possible: true,
                };
                
                if let Some(recovery_strategy) = self.log_beacon_error(beacon_error) {
                    self.apply_recovery_strategy(recovery_strategy)?;
                }
                
                self.handle_emergency(EmergencyType::HardwareFault)?;
            }
            _ => {}
        }
        
        Ok(())
    }
    
    /// Check power status and handle power-related emergencies
    fn check_power_status(&mut self) -> Result<(), BeaconError> {
        let battery_status = match self.power_manager.get_battery_status() {
            Ok(status) => status,
            Err(power_error) => {
                let beacon_error = BeaconError::PowerError {
                    error_type: PowerErrorType::ThresholdViolation {
                        threshold_name: "battery_status_read".to_string(),
                        value: 0.0,
                    },
                    battery_status: BatteryStatusSnapshot {
                        voltage_v: 0.0,
                        current_ma: 0.0,
                        capacity_percent: 0.0,
                        temperature_c: 0.0,
                        health: BeaconBatteryHealth::Unknown,
                        cycles: 0,
                    },
                    power_mode: BeaconPowerMode::Normal,
                    charging_status: BeaconChargingStatus::NotCharging,
                };
                
                if let Some(recovery_strategy) = self.log_beacon_error(beacon_error.clone()) {
                    self.apply_recovery_strategy(recovery_strategy)?;
                }
                
                return Err(beacon_error);
            }
        };
        
        // Check for power emergencies
        if battery_status.capacity_percent <= self.config.emergency_config.emergency_power_threshold_percent {
            let beacon_error = BeaconError::PowerError {
                error_type: PowerErrorType::BatteryDepleted,
                battery_status: BatteryStatusSnapshot {
                    voltage_v: battery_status.voltage_v,
                    current_ma: battery_status.current_ma,
                    capacity_percent: battery_status.capacity_percent,
                    temperature_c: battery_status.temperature_c,
                    health: BeaconBatteryHealth::Critical,
                    cycles: 0,
                },
                power_mode: BeaconPowerMode::Emergency,
                charging_status: BeaconChargingStatus::NotCharging,
            };
            
            if let Some(recovery_strategy) = self.log_beacon_error(beacon_error) {
                self.apply_recovery_strategy(recovery_strategy)?;
            }
            
            self.handle_emergency(EmergencyType::BatteryDepleted)?;
        } else if battery_status.capacity_percent <= self.config.power_config.power_save_mode_threshold_percent {
            if self.operational_state == OperationalState::Normal {
                self.operational_state = OperationalState::PowerSave;
                if let Err(e) = self.power_manager.set_power_mode(PowerOperationMode::PowerSave) {
                    let beacon_error = BeaconError::PowerError {
                        error_type: PowerErrorType::PowerModeTransitionFailed,
                        battery_status: BatteryStatusSnapshot {
                            voltage_v: battery_status.voltage_v,
                            current_ma: battery_status.current_ma,
                            capacity_percent: battery_status.capacity_percent,
                            temperature_c: battery_status.temperature_c,
                            health: BeaconBatteryHealth::Fair,
                            cycles: 0,
                        },
                        power_mode: BeaconPowerMode::PowerSave,
                        charging_status: BeaconChargingStatus::NotCharging,
                    };
                    
                    if let Some(recovery_strategy) = self.log_beacon_error(beacon_error) {
                        self.apply_recovery_strategy(recovery_strategy)?;
                    }
                } else {
                    self.log_info("Entering power save mode");
                }
            }
        }
        
        // Check temperature extremes
        if battery_status.temperature_c < self.config.power_config.temperature_min_c ||
           battery_status.temperature_c > self.config.power_config.temperature_max_c {
            let beacon_error = BeaconError::PowerError {
                error_type: PowerErrorType::TemperatureExtreme {
                    temperature_c: battery_status.temperature_c,
                },
                battery_status: BatteryStatusSnapshot {
                    voltage_v: battery_status.voltage_v,
                    current_ma: battery_status.current_ma,
                    capacity_percent: battery_status.capacity_percent,
                    temperature_c: battery_status.temperature_c,
                    health: BeaconBatteryHealth::Poor,
                    cycles: 0,
                },
                power_mode: BeaconPowerMode::Emergency,
                charging_status: BeaconChargingStatus::NotCharging,
            };
            
            if let Some(recovery_strategy) = self.log_beacon_error(beacon_error) {
                self.apply_recovery_strategy(recovery_strategy)?;
            }
            
            self.handle_emergency(EmergencyType::TemperatureExtreme)?;
        }
        
        // Check for power threshold violations
        if let Ok(violations) = self.power_manager.check_thresholds() {
            for violation in violations {
                self.log_warning(&format!("Power threshold violation: {}", violation));
                
                let beacon_error = BeaconError::PowerError {
                    error_type: PowerErrorType::ThresholdViolation {
                        threshold_name: format!("{:?}", violation),
                        value: battery_status.capacity_percent,
                    },
                    battery_status: BatteryStatusSnapshot {
                        voltage_v: battery_status.voltage_v,
                        current_ma: battery_status.current_ma,
                        capacity_percent: battery_status.capacity_percent,
                        temperature_c: battery_status.temperature_c,
                        health: BeaconBatteryHealth::Fair,
                        cycles: 0,
                    },
                    power_mode: BeaconPowerMode::Normal,
                    charging_status: BeaconChargingStatus::NotCharging,
                };
                
                if let Some(recovery_strategy) = self.log_beacon_error(beacon_error) {
                    let _ = self.apply_recovery_strategy(recovery_strategy);
                }
            }
        }
        
        Ok(())
    }
    
    /// Handle message transmission
    fn handle_transmission(&mut self) -> Result<(), BeaconError> {
        // Get current position
        let position = if let Some(gps_pos) = self.gps_manager.get_current_position() {
            GeodeticPosition {
                latitude: gps_pos.latitude,
                longitude: gps_pos.longitude,
                depth: gps_pos.altitude,
            }
        } else {
            // Use last known position or default
            self.log_warning("No GPS position available for transmission");
            return Err(BeaconError::TransmissionError {
                error_type: TransmissionErrorType::MessageBuildFailed,
                message_sequence: self.transmission_sequence,
                transmission_power: self.current_power_level,
                retry_count: 0,
            });
        };
        
        // Calculate signal quality based on GPS and power status
        let signal_quality = self.calculate_signal_quality();
        
        // Build message based on configured version using MessageBuilder
        let message_builder = MessageBuilder::new();
        let message_data = match self.config.message_version {
            MessageVersion::V1 => {
                let legacy_id = self.beacon_id_to_u16();
                message_builder.build_v1_message(legacy_id, position, signal_quality, self.transmission_sequence)
            }
            MessageVersion::V2 => {
                let legacy_id = self.beacon_id_to_u16();
                message_builder.build_v2_message(legacy_id, position, signal_quality, self.transmission_sequence)
            }
            MessageVersion::V3 => {
                message_builder.build_v3_message(self.config.beacon_id, position, signal_quality, self.transmission_sequence)
            }
        }.map_err(|_| BeaconError::TransmissionError {
            error_type: TransmissionErrorType::MessageBuildFailed,
            message_sequence: self.transmission_sequence,
            transmission_power: self.current_power_level,
            retry_count: 0,
        })?;
        
        // Transmit message
        self.transceiver.transmit_message(&message_data)
            .map_err(|_| BeaconError::TransmissionError {
                error_type: TransmissionErrorType::TransceiverFault,
                message_sequence: self.transmission_sequence,
                transmission_power: self.current_power_level,
                retry_count: 0,
            })?;
        
        // Update transmission tracking
        self.transmission_sequence = self.transmission_sequence.wrapping_add(1);
        self.last_transmission = Some(SystemTime::now());
        
        Ok(())
    }
    
    /// Check communication status and handle updates
    fn check_communication(&mut self) -> Result<(), BeaconError> {
        // Try to connect if not connected
        if !self.communication_manager.is_connected() {
            if let Err(e) = self.communication_manager.connect() {
                self.log_warning(&format!("Communication connection failed: {}", e));
                return Ok(()); // Don't treat as fatal error
            }
        }
        
        // Send status report
        let status_report = self.create_status_report();
        if let Err(e) = self.communication_manager.send_status_report(status_report) {
            self.log_warning(&format!("Status report failed: {}", e));
        } else {
            self.last_communication = Some(SystemTime::now());
        }
        
        // Check for configuration updates
        if let Ok(Some(update)) = self.communication_manager.check_for_updates() {
            self.log_info(&format!("Configuration update received: {}", update.description));
            // TODO: Process configuration update
        }
        
        Ok(())
    }
    
    /// Handle control messages
    fn handle_control_message(&mut self, message: ControlMessage) -> Result<(), BeaconError> {
        match message {
            ControlMessage::Start => self.start(),
            ControlMessage::Stop => self.stop(),
            ControlMessage::Emergency(emergency_type) => self.handle_emergency(emergency_type),
            ControlMessage::UpdateConfig(config) => self.update_configuration(config),
            ControlMessage::GetStatus => {
                // Status is handled synchronously via get_status()
                Ok(())
            }
            ControlMessage::Shutdown => {
                self.prepare_emergency_shutdown()
            }
        }
    }
    
    /// Send emergency messages before shutdown
    fn send_emergency_messages(&mut self) -> Result<(), BeaconError> {
        self.log_info("Sending emergency messages");
        
        for i in 0..self.config.emergency_config.emergency_message_count {
            if let Err(e) = self.handle_transmission() {
                self.log_error(&format!("Emergency transmission {} failed: {}", i + 1, e));
            }
            
            // Brief delay between emergency messages
            thread::sleep(Duration::from_millis(1000));
        }
        
        Ok(())
    }
    
    /// Prepare for emergency shutdown
    fn prepare_emergency_shutdown(&mut self) -> Result<(), BeaconError> {
        self.log_info("Preparing emergency shutdown");
        
        // Send final emergency messages
        self.send_emergency_messages()?;
        
        // Prepare power manager for shutdown
        self.power_manager.prepare_emergency_shutdown()?;
        
        // Stop all operations
        self.stop()?;
        
        Ok(())
    }
    
    /// Calculate signal quality based on system status
    fn calculate_signal_quality(&self) -> u8 {
        let mut quality = 255u8; // Start with maximum quality
        
        // Reduce quality based on GPS accuracy
        if let Some(accuracy) = self.gps_manager.get_position_accuracy() {
            if accuracy > 10.0 {
                quality = quality.saturating_sub(50);
            } else if accuracy > 5.0 {
                quality = quality.saturating_sub(25);
            }
        } else {
            quality = quality.saturating_sub(100); // No GPS
        }
        
        // Reduce quality based on battery level
        if let Ok(battery_status) = self.power_manager.get_battery_status() {
            if battery_status.capacity_percent < 20.0 {
                quality = quality.saturating_sub(50);
            } else if battery_status.capacity_percent < 50.0 {
                quality = quality.saturating_sub(25);
            }
        }
        
        // Reduce quality based on operational state
        match self.operational_state {
            OperationalState::Emergency => quality.saturating_sub(100),
            OperationalState::PowerSave => quality.saturating_sub(50),
            OperationalState::Error(_) => quality.saturating_sub(150),
            _ => quality,
        }
    }
    
    /// Convert UUID to u16 for legacy message formats
    fn beacon_id_to_u16(&self) -> u16 {
        let bytes = self.config.beacon_id.as_bytes();
        (bytes[0] as u16) << 8 | (bytes[1] as u16)
    }
    
    /// Create status report for communication
    fn create_status_report(&self) -> StatusReport {
        let battery_status = self.power_manager.get_battery_status()
            .unwrap_or_else(|_| BatteryStatus::new(0.0, 0.0, 0.0, 0.0));
        
        let uptime = SystemTime::now()
            .duration_since(self.start_time)
            .unwrap_or(Duration::from_secs(0));
        
        StatusReport {
            beacon_id: self.config.beacon_id,
            timestamp: SystemTime::now(),
            position_history: self.get_position_history(),
            battery_status,
            system_health: self.get_system_health(),
            transmission_stats: self.get_transmission_stats(),
            uptime,
            recent_errors: self.error_log.clone(),
        }
    }
    
    /// Get recent position history
    fn get_position_history(&self) -> Vec<GpsPosition> {
        // TODO: Implement position history tracking
        if let Some(current_pos) = self.gps_manager.get_current_position() {
            vec![current_pos]
        } else {
            vec![]
        }
    }
    
    /// Get system health metrics
    fn get_system_health(&self) -> SystemHealth {
        SystemHealth {
            cpu_usage_percent: 25.0, // TODO: Implement actual CPU monitoring
            memory_usage_percent: 60.0, // TODO: Implement actual memory monitoring
            temperature_c: 30.0, // TODO: Implement actual temperature monitoring
            gps_signal_quality: self.gps_manager.get_satellite_count() * 10,
            comm_signal_quality: self.communication_manager.get_signal_strength().unwrap_or(0),
            restart_count: 0, // TODO: Track restart count
            last_restart_reason: "Normal startup".to_string(),
        }
    }
    
    /// Get transmission statistics
    fn get_transmission_stats(&self) -> CommTransmissionStats {
        CommTransmissionStats {
            messages_sent: self.transmission_sequence as u64,
            transmission_failures: 0, // TODO: Track transmission failures
            last_transmission_time: self.last_transmission,
            average_transmission_interval_ms: self.config.transmission_interval_ms,
            signal_quality_history: vec![], // TODO: Track signal quality history
            power_level_history: vec![], // TODO: Track power level history
        }
    }
    
    /// Log beacon error with enhanced diagnostics
    fn log_beacon_error(&mut self, error: BeaconError) -> Option<RecoveryStrategy> {
        let context = self.create_beacon_error_context();
        self.diagnostic_system.log_beacon_error(error, context)
    }
    
    /// Create comprehensive error context
    fn create_beacon_error_context(&self) -> BeaconErrorContext {
        let battery_status = self.power_manager.get_battery_status()
            .unwrap_or_else(|_| BatteryStatus::new(0.0, 0.0, 0.0, 0.0));
        
        BeaconErrorContext {
            timestamp: SystemTime::now(),
            beacon_id: self.config.beacon_id,
            operational_state: format!("{:?}", self.operational_state),
            system_state: BeaconSystemState {
                operational_state: format!("{:?}", self.operational_state),
                uptime_ms: SystemTime::now()
                    .duration_since(self.start_time)
                    .unwrap_or(Duration::from_secs(0))
                    .as_millis() as u64,
                last_gps_fix: self.last_gps_update,
                last_transmission: self.last_transmission,
                last_communication: self.last_communication,
                active_threads: 1, // TODO: Track actual thread count
                error_count: self.error_log.len() as u32,
            },
            gps_status: GpsStatusSnapshot {
                is_locked: self.gps_manager.is_locked(),
                satellite_count: self.gps_manager.get_satellite_count(),
                accuracy_m: self.gps_manager.get_position_accuracy(),
                last_fix_time: self.last_gps_update,
                signal_strength: None, // TODO: Get actual signal strength
            },
            power_status: BatteryStatusSnapshot {
                voltage_v: battery_status.voltage_v,
                current_ma: battery_status.current_ma,
                capacity_percent: battery_status.capacity_percent,
                temperature_c: battery_status.temperature_c,
                health: BeaconBatteryHealth::Good, // TODO: Determine actual health
                cycles: 0, // TODO: Track battery cycles
            },
            communication_status: CommunicationStatusSnapshot {
                is_connected: self.communication_manager.is_connected(),
                signal_strength: self.communication_manager.get_signal_strength(),
                last_successful_connection: self.last_communication,
                connection_attempts: 0, // TODO: Track connection attempts
                data_sent_bytes: 0, // TODO: Track data transfer
                data_received_bytes: 0,
            },
            transmission_status: TransmissionStatusSnapshot {
                last_transmission: self.last_transmission,
                transmission_count: self.transmission_sequence as u64,
                failure_count: 0, // TODO: Track transmission failures
                current_power_level: self.current_power_level,
                message_sequence: self.transmission_sequence,
                average_interval_ms: self.config.transmission_interval_ms,
            },
            environmental_conditions: EnvironmentalMetrics {
                temperature_c: battery_status.temperature_c,
                humidity_percent: 60.0, // TODO: Get actual humidity
                pressure_hpa: 1013.25, // TODO: Get actual pressure
                signal_attenuation_db: 5.0, // TODO: Calculate signal attenuation
                noise_level_db: 30.0, // TODO: Measure noise level
                environmental_stress_score: 0.2, // TODO: Calculate stress score
            },
            resource_usage: ResourceUsageSnapshot {
                memory_usage_bytes: 50000, // TODO: Get actual memory usage
                memory_total_bytes: 80000,
                cpu_usage_percent: 25.0, // TODO: Get actual CPU usage
                flash_usage_bytes: 200000, // TODO: Get actual flash usage
                flash_total_bytes: 512000,
                active_connections: 1,
            },
            recent_events: vec![], // TODO: Track recent events
        }
    }
    
    /// Generate diagnostic report
    pub fn generate_diagnostic_report(&mut self) -> shared_positioning::DiagnosticReport {
        self.diagnostic_system.generate_diagnostic_report(Some(self.config.beacon_id))
    }
    
    /// Check system health and handle alerts
    fn check_system_health(&mut self) {
        let alerts = self.diagnostic_system.check_system_health();
        
        for alert in alerts {
            match alert.severity {
                ErrorSeverity::Critical | ErrorSeverity::Fatal => {
                    self.log_error(&format!("CRITICAL HEALTH ALERT: {} - {}", alert.component, alert.message));
                    // Consider emergency actions
                    if alert.component.contains("Error Rate") {
                        let _ = self.handle_emergency(EmergencyType::SystemOverload);
                    }
                }
                ErrorSeverity::Warning => {
                    self.log_warning(&format!("Health warning: {} - {}", alert.component, alert.message));
                }
                _ => {}
            }
        }
    }
    
    /// Apply recovery strategy based on error analysis
    fn apply_recovery_strategy(&mut self, strategy: RecoveryStrategy) -> Result<(), BeaconError> {
        match strategy {
            RecoveryStrategy::Retry { max_attempts, delay_ms, .. } => {
                self.log_info(&format!("Applying retry strategy: max {} attempts, delay {} ms", max_attempts, delay_ms));
                thread::sleep(Duration::from_millis(delay_ms));
                // The actual retry will happen in the calling code
            }
            RecoveryStrategy::Fallback { fallback_mode, expected_accuracy_degradation } => {
                self.log_warning(&format!("Applying fallback strategy: {} (accuracy degradation: {}x)", 
                    fallback_mode, expected_accuracy_degradation));
                // Implement fallback mode logic based on the mode
                if fallback_mode == "last_known_position" {
                    // Continue using last known GPS position
                    self.log_info("Using last known GPS position for transmissions");
                }
            }
            RecoveryStrategy::IgnoreWithWarning { warning_message, monitoring_required } => {
                self.log_warning(&format!("Ignoring error with warning: {}", warning_message));
                if monitoring_required {
                    self.log_info("Enhanced monitoring enabled for this error type");
                }
            }
            RecoveryStrategy::Reset { subsystem, preserve_configuration } => {
                self.log_info(&format!("Resetting subsystem: {} (preserve config: {})", subsystem, preserve_configuration));
                match subsystem.as_str() {
                    "gps_receiver" => {
                        // Reset GPS manager
                        if let Err(e) = self.gps_manager.stop() {
                            self.log_error(&format!("Failed to stop GPS manager: {}", e));
                        }
                        if let Err(e) = self.gps_manager.start_acquisition() {
                            self.log_error(&format!("Failed to restart GPS manager: {}", e));
                        }
                    }
                    "transceiver" => {
                        // Reset transceiver (implementation would depend on transceiver interface)
                        self.log_info("Transceiver reset requested");
                    }
                    _ => {
                        self.log_warning(&format!("Unknown subsystem for reset: {}", subsystem));
                    }
                }
            }
            RecoveryStrategy::Degrade { disabled_features, performance_impact } => {
                self.log_warning(&format!("Degrading system performance: disabling {:?} (impact: {})", 
                    disabled_features, performance_impact));
                
                for feature in disabled_features {
                    match feature.as_str() {
                        "communication" => {
                            let _ = self.communication_manager.disconnect();
                            self.log_info("Communication disabled for power conservation");
                        }
                        "high_power_transmission" => {
                            self.current_power_level = self.current_power_level.saturating_sub(50);
                            self.log_info("Transmission power reduced");
                        }
                        "high_power_modes" => {
                            if let Err(e) = self.power_manager.set_power_mode(PowerOperationMode::PowerSave) {
                                self.log_error(&format!("Failed to set power save mode: {}", e));
                            }
                        }
                        _ => {
                            self.log_info(&format!("Feature degradation not implemented: {}", feature));
                        }
                    }
                }
            }
            RecoveryStrategy::UserIntervention { required_action, urgency } => {
                self.log_error(&format!("USER INTERVENTION REQUIRED ({:?}): {}", urgency, required_action));
                // In a real system, this might send an alert to operators
            }
            RecoveryStrategy::Shutdown { reason, save_state } => {
                self.log_error(&format!("SHUTDOWN REQUIRED: {} (save state: {})", reason, save_state));
                if save_state {
                    // Save current state before shutdown
                    let _ = self.generate_diagnostic_report();
                }
                return self.prepare_emergency_shutdown();
            }
        }
        
        Ok(())
    }
    
    /// Log informational message
    fn log_info(&mut self, message: &str) {
        let entry = ErrorLogEntry {
            timestamp: SystemTime::now(),
            severity: CommErrorSeverity::Info,
            message: message.to_string(),
            context: format!("State: {:?}", self.operational_state),
            recovery_action: None,
        };
        self.error_log.push(entry);
        println!("[INFO] {}", message);
    }
    
    /// Log warning message
    fn log_warning(&mut self, message: &str) {
        let entry = ErrorLogEntry {
            timestamp: SystemTime::now(),
            severity: CommErrorSeverity::Warning,
            message: message.to_string(),
            context: format!("State: {:?}", self.operational_state),
            recovery_action: None,
        };
        self.error_log.push(entry);
        println!("[WARN] {}", message);
    }
    
    /// Log error message
    fn log_error(&mut self, message: &str) {
        let entry = ErrorLogEntry {
            timestamp: SystemTime::now(),
            severity: CommErrorSeverity::Error,
            message: message.to_string(),
            context: format!("State: {:?}", self.operational_state),
            recovery_action: None,
        };
        self.error_log.push(entry);
        println!("[ERROR] {}", message);
    }

    /// Update environmental monitoring and handle adaptation actions
    fn update_environmental_monitoring(&mut self) -> Result<(), BeaconError> {
        // Collect current environmental conditions
        let battery_status = self.power_manager.get_battery_status()
            .map_err(|e| BeaconError::PowerError {
                error_type: PowerErrorType::ThresholdViolation {
                    threshold_name: "battery_read".to_string(),
                    value: 0.0,
                },
                battery_status: BatteryStatusSnapshot {
                    voltage_v: 0.0,
                    current_ma: 0.0,
                    capacity_percent: 0.0,
                    temperature_c: 0.0,
                    health: BeaconBatteryHealth::Unknown,
                    cycles: 0,
                },
                power_mode: BeaconPowerMode::Normal,
                charging_status: BeaconChargingStatus::NotCharging,
            })?;

        // Create extended environmental conditions with current sensor readings
        let extended_conditions = ExtendedEnvironmentalConditions {
            base_conditions: self.environmental_conditions.clone(),
            air_temperature_c: None, // Would be read from temperature sensor
            humidity_percent: None,   // Would be read from humidity sensor
            atmospheric_pressure_hpa: None, // Would be read from pressure sensor
            wind_speed_ms: None,     // Would be read from wind sensor
            solar_irradiance_wm2: None, // Would be read from solar sensor
            internal_temperature_c: Some(35.0), // Simulated internal temperature
            cpu_temperature_c: Some(45.0),      // Simulated CPU temperature
            battery_temperature_c: Some(battery_status.temperature_c),
            enclosure_humidity_percent: None,
            vibration_level_g: None, // Would be read from accelerometer
            magnetic_field_strength_ut: None,
            timestamp: SystemTime::now(),
            measurement_quality: shared_positioning::MeasurementQuality {
                overall_quality: 0.9,
                sensor_health: std::collections::HashMap::new(),
                calibration_status: std::collections::HashMap::new(),
                last_calibration: std::collections::HashMap::new(),
            },
        };

        // Update environmental monitor and get adaptation actions
        match self.environmental_monitor.update_conditions(extended_conditions) {
            Ok(actions) => {
                self.last_environmental_update = Some(SystemTime::now());
                
                // Process adaptation actions
                for action in actions {
                    if let Err(e) = self.process_adaptation_action(action) {
                        self.log_warning(&format!("Failed to process adaptation action: {}", e));
                    }
                }
            }
            Err(e) => {
                self.log_error(&format!("Environmental monitoring update failed: {}", e));
                return Err(BeaconError::EnvironmentalError {
                    condition: shared_positioning::error_handling::EnvironmentalCondition::TemperatureExtreme { 
                        temperature_c: battery_status.temperature_c 
                    },
                    severity: ErrorSeverity::Warning,
                    measurement: battery_status.temperature_c as f64,
                    threshold: 50.0,
                    mitigation_applied: false,
                });
            }
        }

        Ok(())
    }

    /// Process environmental adaptation actions
    fn process_adaptation_action(&mut self, action: AdaptationAction) -> Result<(), BeaconError> {
        match action {
            AdaptationAction::ReduceTransmissionPower { from, to } => {
                self.log_info(&format!("Reducing transmission power from {} to {} due to environmental conditions", from, to));
                self.current_power_level = to;
                self.transceiver.set_transmission_power(to)
                    .map_err(|_e| BeaconError::TransmissionError {
                        error_type: TransmissionErrorType::TransceiverFault,
                        message_sequence: self.transmission_sequence,
                        transmission_power: to,
                        retry_count: 0,
                    })?;
            }
            AdaptationAction::IncreaseTransmissionPower { from, to } => {
                self.log_info(&format!("Increasing transmission power from {} to {} due to environmental conditions", from, to));
                self.current_power_level = to;
                self.transceiver.set_transmission_power(to)
                    .map_err(|_e| BeaconError::TransmissionError {
                        error_type: TransmissionErrorType::TransceiverFault,
                        message_sequence: self.transmission_sequence,
                        transmission_power: to,
                        retry_count: 0,
                    })?;
            }
            AdaptationAction::ReduceTransmissionFrequency { from_ms, to_ms } => {
                self.log_info(&format!("Reducing transmission frequency from {}ms to {}ms due to environmental conditions", from_ms, to_ms));
                // Update transmission interval in config (simplified)
                // In real implementation, would need to update the actual transmission scheduling
            }
            AdaptationAction::EnablePowerSaveMode => {
                self.log_info("Enabling power save mode due to environmental conditions");
                self.power_manager.set_power_mode(PowerOperationMode::PowerSave)
                    .map_err(|_e| BeaconError::PowerError {
                        error_type: PowerErrorType::PowerModeTransitionFailed,
                        battery_status: BatteryStatusSnapshot {
                            voltage_v: 0.0,
                            current_ma: 0.0,
                            capacity_percent: 0.0,
                            temperature_c: 0.0,
                            health: BeaconBatteryHealth::Unknown,
                            cycles: 0,
                        },
                        power_mode: BeaconPowerMode::PowerSave,
                        charging_status: BeaconChargingStatus::NotCharging,
                    })?;
                self.operational_state = OperationalState::PowerSave;
            }
            AdaptationAction::DisableNonEssentialSystems => {
                self.log_warning("Disabling non-essential systems due to extreme environmental conditions");
                // In real implementation, would disable non-critical subsystems
            }
            AdaptationAction::ActivateThermalManagement => {
                self.log_warning("Activating thermal management due to high temperature");
                // In real implementation, would activate cooling systems or reduce CPU frequency
            }
            AdaptationAction::RequestEmergencyShutdown { reason } => {
                self.log_error(&format!("Emergency shutdown requested: {}", reason));
                self.handle_emergency(EmergencyType::TemperatureExtreme)?;
            }
            AdaptationAction::AdjustSensorSampling { sensor, from_ms, to_ms } => {
                self.log_info(&format!("Adjusting {} sensor sampling from {}ms to {}ms", sensor, from_ms, to_ms));
                // In real implementation, would adjust sensor sampling rates
            }
        }

        Ok(())
    }

    /// Run hardware diagnostics and handle fault detection
    fn run_hardware_diagnostics(&mut self) -> Result<(), BeaconError> {
        if !self.hardware_monitor.is_monitoring_active() {
            self.hardware_monitor.start_monitoring();
        }

        // Run comprehensive diagnostics
        match self.hardware_monitor.run_diagnostics(
            &mut self.gps_manager,
            &mut self.power_manager,
            &mut self.communication_manager,
            &mut self.transceiver,
        ) {
            Ok(results) => {
                self.last_hardware_diagnostic = Some(SystemTime::now());
                
                // Process diagnostic results
                for result in results {
                    match result.health {
                        ComponentHealth::Faulty { ref fault_description } => {
                            self.log_error(&format!("Hardware fault detected in {:?}: {}", result.component, fault_description));
                            
                            // Record failure event for reliability monitoring
                            let failure_event = FailureEvent {
                                timestamp: SystemTime::now(),
                                component: format!("{:?}", result.component),
                                failure_type: "hardware_fault".to_string(),
                                severity: ErrorSeverity::Error,
                                duration: None,
                                recovery_successful: false,
                                recovery_time: None,
                                impact_description: fault_description.clone(),
                                root_cause: Some("hardware_diagnostic".to_string()),
                            };
                            self.reliability_monitor.record_failure(failure_event);
                        }
                        ComponentHealth::Failed => {
                            self.log_error(&format!("Hardware component failed: {:?}", result.component));
                            
                            // Record critical failure
                            let failure_event = FailureEvent {
                                timestamp: SystemTime::now(),
                                component: format!("{:?}", result.component),
                                failure_type: "component_failure".to_string(),
                                severity: ErrorSeverity::Critical,
                                duration: None,
                                recovery_successful: false,
                                recovery_time: None,
                                impact_description: "Component completely failed".to_string(),
                                root_cause: Some("hardware_diagnostic".to_string()),
                            };
                            self.reliability_monitor.record_failure(failure_event);
                        }
                        ComponentHealth::Degraded { performance_impact } => {
                            self.log_warning(&format!("Hardware component degraded: {:?} (impact: {:.1}%)", 
                                result.component, performance_impact * 100.0));
                            
                            // Update component performance in reliability monitor
                            self.reliability_monitor.update_component_performance(
                                format!("{:?}", result.component),
                                1.0 - performance_impact as f64
                            );
                        }
                        ComponentHealth::Healthy => {
                            // Update component performance as healthy
                            self.reliability_monitor.update_component_performance(
                                format!("{:?}", result.component),
                                1.0
                            );
                        }
                        ComponentHealth::Unknown => {
                            self.log_warning(&format!("Hardware component status unknown: {:?}", result.component));
                        }
                    }

                    // Process recommended actions
                    match result.recommended_action {
                        RecommendedAction::SoftReset => {
                            self.log_info(&format!("Soft reset recommended for {:?}", result.component));
                            // In real implementation, would perform soft reset
                        }
                        RecommendedAction::HardReset => {
                            self.log_warning(&format!("Hard reset recommended for {:?}", result.component));
                            // In real implementation, would perform hard reset
                        }
                        RecommendedAction::PowerCycle => {
                            self.log_warning(&format!("Power cycle recommended for {:?}", result.component));
                            // In real implementation, would power cycle component
                        }
                        RecommendedAction::Recalibrate => {
                            self.log_info(&format!("Recalibration recommended for {:?}", result.component));
                            // In real implementation, would recalibrate component
                        }
                        RecommendedAction::ReplaceComponent => {
                            self.log_error(&format!("Component replacement required for {:?}", result.component));
                            // In real implementation, would alert for maintenance
                        }
                        RecommendedAction::EmergencyShutdown => {
                            self.log_error(&format!("Emergency shutdown recommended due to {:?} failure", result.component));
                            self.handle_emergency(EmergencyType::HardwareFault)?;
                        }
                        _ => {}
                    }
                }
            }
            Err(e) => {
                self.log_error(&format!("Hardware diagnostics failed: {}", e));
                return Err(BeaconError::HardwareError {
                    component: HardwareComponent::Microcontroller,
                    fault_type: HardwareFaultType::ComponentFailure,
                    diagnostic_data: vec![],
                    recovery_possible: true,
                });
            }
        }

        Ok(())
    }

    /// Update reliability monitoring and generate reports
    fn update_reliability_monitoring(&mut self) -> Result<(), BeaconError> {
        if !self.reliability_monitor.is_monitoring_active() {
            self.reliability_monitor.start_monitoring();
        }

        // Calculate current reliability metrics
        match self.reliability_monitor.calculate_reliability_metrics() {
            Ok(metrics) => {
                self.log_info(&format!(
                    "Reliability metrics: Availability: {:.1}%, MTBF: {:.1}h, Failure rate: {:.3}/h",
                    metrics.overall_availability * 100.0,
                    metrics.mean_time_between_failures.as_secs_f64() / 3600.0,
                    metrics.failure_rate
                ));

                // Check for reliability threshold violations
                if metrics.overall_availability < 0.95 {
                    self.log_warning(&format!("Low system availability: {:.1}%", metrics.overall_availability * 100.0));
                }

                if metrics.failure_rate > 0.1 {
                    self.log_warning(&format!("High failure rate: {:.3} failures/hour", metrics.failure_rate));
                }

                // Generate full reliability report periodically (every hour)
                if let Some(last_report) = self.last_reliability_report {
                    if SystemTime::now().duration_since(last_report).unwrap_or(Duration::from_secs(0)) > Duration::from_secs(3600) {
                        self.generate_reliability_report()?;
                    }
                } else {
                    self.generate_reliability_report()?;
                }
            }
            Err(e) => {
                self.log_error(&format!("Reliability metrics calculation failed: {}", e));
                return Err(BeaconError::SystemError {
                    error_type: SystemErrorType::SystemOverload,
                    system_state: BeaconSystemState {
                        operational_state: format!("{:?}", self.operational_state),
                        uptime_ms: SystemTime::now().duration_since(self.start_time).unwrap_or(Duration::from_secs(0)).as_millis() as u64,
                        last_gps_fix: self.last_gps_update,
                        last_transmission: self.last_transmission,
                        last_communication: self.last_communication,
                        active_threads: 1,
                        error_count: 0,
                    },
                    resource_usage: ResourceUsageSnapshot {
                        memory_usage_bytes: 0,
                        memory_total_bytes: 0,
                        cpu_usage_percent: 0.0,
                        flash_usage_bytes: 0,
                        flash_total_bytes: 0,
                        active_connections: 0,
                    },
                });
            }
        }

        Ok(())
    }

    /// Generate comprehensive reliability report
    fn generate_reliability_report(&mut self) -> Result<(), BeaconError> {
        match self.reliability_monitor.generate_reliability_report() {
            Ok(report) => {
                self.last_reliability_report = Some(SystemTime::now());
                
                self.log_info(&format!("Reliability Report Generated: {}", report.report_id));
                self.log_info(&format!("Executive Summary: {}", report.executive_summary));
                
                // Log maintenance recommendations
                if !report.maintenance_recommendations.is_empty() {
                    self.log_info("Maintenance Recommendations:");
                    for recommendation in &report.maintenance_recommendations {
                        self.log_info(&format!("- {}: {} (Urgency: {:?})", 
                            recommendation.component, 
                            recommendation.description,
                            recommendation.urgency
                        ));
                    }
                }

                // Log threshold violations
                if !report.threshold_violations.is_empty() {
                    self.log_warning("Reliability Threshold Violations:");
                    for violation in &report.threshold_violations {
                        self.log_warning(&format!("- {}: {:.3} exceeds threshold {:.3}", 
                            violation.metric, 
                            violation.current_value,
                            violation.threshold_value
                        ));
                    }
                }

                // In real implementation, would save report to file or send to monitoring system
            }
            Err(e) => {
                self.log_error(&format!("Failed to generate reliability report: {}", e));
                return Err(BeaconError::SystemError {
                    error_type: SystemErrorType::SystemOverload,
                    system_state: BeaconSystemState {
                        operational_state: format!("{:?}", self.operational_state),
                        uptime_ms: SystemTime::now().duration_since(self.start_time).unwrap_or(Duration::from_secs(0)).as_millis() as u64,
                        last_gps_fix: self.last_gps_update,
                        last_transmission: self.last_transmission,
                        last_communication: self.last_communication,
                        active_threads: 1,
                        error_count: 0,
                    },
                    resource_usage: ResourceUsageSnapshot {
                        memory_usage_bytes: 0,
                        memory_total_bytes: 0,
                        cpu_usage_percent: 0.0,
                        flash_usage_bytes: 0,
                        flash_total_bytes: 0,
                        active_connections: 0,
                    },
                });
            }
        }

        Ok(())
    }

    /// Get environmental monitoring statistics
    pub fn get_environmental_stats(&self) -> &EnvironmentalStats {
        self.environmental_monitor.get_statistics()
    }

    /// Get hardware monitoring statistics
    pub fn get_hardware_stats(&self) -> &HardwareMonitorStats {
        self.hardware_monitor.get_statistics()
    }

    /// Get reliability monitoring statistics
    pub fn get_reliability_stats(&self) -> (usize, usize, usize) {
        self.reliability_monitor.get_statistics()
    }

    /// Check if thermal management is active
    pub fn is_thermal_management_active(&self) -> bool {
        self.environmental_monitor.is_thermal_management_active()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use shared_positioning::{MockGpsManager, MockPowerManager, MockCommunicationManager, MockTransceiver};
    
    fn create_test_config() -> BeaconConfig {
        BeaconConfig {
            beacon_id: Uuid::new_v4(),
            transmission_interval_ms: 5000,
            message_version: MessageVersion::V3,
            gps_config: GpsConfig::default(),
            power_config: PowerConfig::default(),
            communication_config: CommunicationConfig::default(),
            emergency_config: EmergencyConfig::default(),
        }
    }
    
    fn create_test_controller() -> BeaconController<MockGpsManager, MockPowerManager, MockCommunicationManager, MockTransceiver> {
        let config = create_test_config();
        let gps_manager = MockGpsManager::new(GpsConfig::default()).unwrap();
        let power_manager = MockPowerManager::new();
        let comm_manager = MockCommunicationManager::new();
        let transceiver = MockTransceiver::new(1);
        
        BeaconController::new(config, gps_manager, power_manager, comm_manager, transceiver).unwrap()
    }
    
    #[test]
    fn test_beacon_controller_creation() {
        let controller = create_test_controller();
        assert_eq!(controller.operational_state, OperationalState::Initializing);
        assert!(!controller.running);
    }
    
    #[test]
    fn test_config_validation() {
        let mut config = create_test_config();
        
        // Valid configuration
        assert!(BeaconController::<MockGpsManager, MockPowerManager, MockCommunicationManager, MockTransceiver>::validate_config(&config).is_ok());
        
        // Invalid transmission interval
        config.transmission_interval_ms = 500; // Too short
        assert!(BeaconController::<MockGpsManager, MockPowerManager, MockCommunicationManager, MockTransceiver>::validate_config(&config).is_err());
        
        config.transmission_interval_ms = 70000; // Too long
        assert!(BeaconController::<MockGpsManager, MockPowerManager, MockCommunicationManager, MockTransceiver>::validate_config(&config).is_err());
        
        // Invalid emergency power threshold
        config = create_test_config();
        config.emergency_config.emergency_power_threshold_percent = 0.5; // Too low
        assert!(BeaconController::<MockGpsManager, MockPowerManager, MockCommunicationManager, MockTransceiver>::validate_config(&config).is_err());
        
        config.emergency_config.emergency_power_threshold_percent = 25.0; // Too high
        assert!(BeaconController::<MockGpsManager, MockPowerManager, MockCommunicationManager, MockTransceiver>::validate_config(&config).is_err());
    }
    
    #[test]
    fn test_beacon_status() {
        let controller = create_test_controller();
        let status = controller.get_status();
        
        assert_eq!(status.beacon_id, controller.config.beacon_id);
        assert_eq!(status.operational_state, OperationalState::Initializing);
    }
    
    #[test]
    fn test_signal_quality_calculation() {
        let controller = create_test_controller();
        let quality = controller.calculate_signal_quality();
        
        // Should be reduced due to no GPS and initializing state
        assert!(quality < 255);
    }
    
    #[test]
    fn test_beacon_id_to_u16_conversion() {
        let controller = create_test_controller();
        let u16_id = controller.beacon_id_to_u16();
        
        // Should be derived from first two bytes of UUID
        let uuid_bytes = controller.config.beacon_id.as_bytes();
        let expected = (uuid_bytes[0] as u16) << 8 | (uuid_bytes[1] as u16);
        assert_eq!(u16_id, expected);
    }
    
    #[test]
    fn test_emergency_config_default() {
        let config = EmergencyConfig::default();
        assert_eq!(config.emergency_transmission_interval_ms, 30000);
        assert_eq!(config.emergency_power_threshold_percent, 5.0);
        assert!(config.auto_shutdown_enabled);
        assert_eq!(config.emergency_message_count, 10);
    }
    
    #[test]
    fn test_operational_state_transitions() {
        let mut controller = create_test_controller();
        
        // Initial state
        assert_eq!(controller.operational_state, OperationalState::Initializing);
        
        // Test emergency handling
        assert!(controller.handle_emergency(EmergencyType::BatteryDepleted).is_ok());
        assert_eq!(controller.operational_state, OperationalState::Emergency);
    }
}
