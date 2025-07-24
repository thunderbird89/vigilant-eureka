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
    ErrorLogEntry, CommErrorSeverity, CommError
};

/// Beacon-specific error types
#[derive(Debug, Clone)]
pub enum BeaconError {
    GpsError(GpsError),
    PowerError(PowerError),
    CommunicationError(CommError),
    TransmissionError(TransmissionError),
    ConfigurationError(ConfigError),
    SystemError(SystemError),
}

impl std::fmt::Display for BeaconError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BeaconError::GpsError(e) => write!(f, "GPS error: {}", e),
            BeaconError::PowerError(e) => write!(f, "Power error: {}", e),
            BeaconError::CommunicationError(e) => write!(f, "Communication error: {}", e),
            BeaconError::TransmissionError(e) => write!(f, "Transmission error: {}", e),
            BeaconError::ConfigurationError(e) => write!(f, "Configuration error: {}", e),
            BeaconError::SystemError(e) => write!(f, "System error: {}", e),
        }
    }
}

impl std::error::Error for BeaconError {}

impl From<GpsError> for BeaconError {
    fn from(error: GpsError) -> Self {
        BeaconError::GpsError(error)
    }
}

impl From<PowerError> for BeaconError {
    fn from(error: PowerError) -> Self {
        BeaconError::PowerError(error)
    }
}

impl From<CommError> for BeaconError {
    fn from(error: CommError) -> Self {
        BeaconError::CommunicationError(error)
    }
}

/// Transmission-specific error types
#[derive(Debug, Clone)]
pub enum TransmissionError {
    TransceiverFault,
    MessageBuildFailed,
    PowerInsufficient,
    SchedulingConflict,
    HardwareTimeout,
}

impl std::fmt::Display for TransmissionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TransmissionError::TransceiverFault => write!(f, "Transceiver fault"),
            TransmissionError::MessageBuildFailed => write!(f, "Message build failed"),
            TransmissionError::PowerInsufficient => write!(f, "Insufficient power"),
            TransmissionError::SchedulingConflict => write!(f, "Scheduling conflict"),
            TransmissionError::HardwareTimeout => write!(f, "Hardware timeout"),
        }
    }
}

impl std::error::Error for TransmissionError {}

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
}

impl Default for EmergencyConfig {
    fn default() -> Self {
        Self {
            emergency_transmission_interval_ms: 30000, // 30 seconds
            emergency_power_threshold_percent: 5.0,
            emergency_gps_timeout_s: 300, // 5 minutes
            emergency_communication_timeout_s: 1800, // 30 minutes
            auto_shutdown_enabled: true,
            emergency_message_count: 10,
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

/// Beacon status information
#[derive(Debug, Clone)]
pub struct BeaconStatus {
    pub beacon_id: Uuid,
    pub operational_state: OperationalState,
    pub gps_status: GpsStatus,
    pub battery_status: BatteryStatus,
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
    message_builder: MessageBuilder,
    start_time: SystemTime,
    last_transmission: Option<SystemTime>,
    last_gps_update: Option<SystemTime>,
    last_communication: Option<SystemTime>,
    transmission_sequence: u16,
    error_log: Vec<ErrorLogEntry>,
    control_sender: Option<Sender<ControlMessage>>,
    control_receiver: Option<Receiver<ControlMessage>>,
    running: bool,
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
        
        Ok(Self {
            config,
            operational_state: OperationalState::Initializing,
            gps_manager,
            power_manager,
            communication_manager,
            transceiver,
            message_builder: MessageBuilder::new(),
            start_time: SystemTime::now(),
            last_transmission: None,
            last_gps_update: None,
            last_communication: None,
            transmission_sequence: 0,
            error_log: Vec::new(),
            control_sender: Some(sender),
            control_receiver: Some(receiver),
            running: false,
        })
    }
    
    /// Validate beacon configuration
    fn validate_config(config: &BeaconConfig) -> Result<(), BeaconError> {
        if config.transmission_interval_ms < 1000 || config.transmission_interval_ms > 60000 {
            return Err(BeaconError::ConfigurationError(ConfigError::InvalidParameter(
                "Transmission interval must be between 1-60 seconds".to_string()
            )));
        }
        
        if config.emergency_config.emergency_transmission_interval_ms < 1000 {
            return Err(BeaconError::ConfigurationError(ConfigError::InvalidParameter(
                "Emergency transmission interval must be at least 1 second".to_string()
            )));
        }
        
        if config.emergency_config.emergency_power_threshold_percent < 1.0 || 
           config.emergency_config.emergency_power_threshold_percent > 20.0 {
            return Err(BeaconError::ConfigurationError(ConfigError::InvalidParameter(
                "Emergency power threshold must be between 1-20%".to_string()
            )));
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
        
        BeaconStatus {
            beacon_id: self.config.beacon_id,
            operational_state: self.operational_state.clone(),
            gps_status: self.gps_manager.get_status(),
            battery_status,
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
        self.gps_manager.update()?;
        
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
                        self.handle_emergency(EmergencyType::GpsSignalLost)?;
                    }
                }
            }
            GpsStatus::HardwareFault => {
                self.handle_emergency(EmergencyType::HardwareFault)?;
            }
            _ => {}
        }
        
        Ok(())
    }
    
    /// Check power status and handle power-related emergencies
    fn check_power_status(&mut self) -> Result<(), BeaconError> {
        let battery_status = self.power_manager.get_battery_status()?;
        
        // Check for power emergencies
        if battery_status.capacity_percent <= self.config.emergency_config.emergency_power_threshold_percent {
            self.handle_emergency(EmergencyType::BatteryDepleted)?;
        } else if battery_status.capacity_percent <= self.config.power_config.power_save_mode_threshold_percent {
            if self.operational_state == OperationalState::Normal {
                self.operational_state = OperationalState::PowerSave;
                self.power_manager.set_power_mode(PowerOperationMode::PowerSave)?;
                self.log_info("Entering power save mode");
            }
        }
        
        // Check temperature extremes
        if battery_status.temperature_c < self.config.power_config.temperature_min_c ||
           battery_status.temperature_c > self.config.power_config.temperature_max_c {
            self.handle_emergency(EmergencyType::TemperatureExtreme)?;
        }
        
        // Check for power threshold violations
        if let Ok(violations) = self.power_manager.check_thresholds() {
            for violation in violations {
                self.log_warning(&format!("Power threshold violation: {}", violation));
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
            return Err(BeaconError::TransmissionError(TransmissionError::MessageBuildFailed));
        };
        
        // Calculate signal quality based on GPS and power status
        let signal_quality = self.calculate_signal_quality();
        
        // Build message based on configured version
        let message_data = match self.config.message_version {
            MessageVersion::V1 => {
                let legacy_id = self.beacon_id_to_u16();
                self.message_builder.build_v1_message(legacy_id, position, signal_quality, self.transmission_sequence)
            }
            MessageVersion::V2 => {
                let legacy_id = self.beacon_id_to_u16();
                self.message_builder.build_v2_message(legacy_id, position, signal_quality, self.transmission_sequence)
            }
            MessageVersion::V3 => {
                self.message_builder.build_v3_message(self.config.beacon_id, position, signal_quality, self.transmission_sequence)
            }
        }.map_err(|_| BeaconError::TransmissionError(TransmissionError::MessageBuildFailed))?;
        
        // Transmit message
        self.transceiver.transmit_message(&message_data)
            .map_err(|_| BeaconError::TransmissionError(TransmissionError::TransceiverFault))?;
        
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