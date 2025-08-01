// Hardware abstraction layer for transceiver communication
// Supports UART/Serial and I2C interfaces for JANUS transceivers

use std::collections::VecDeque;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use crate::message_parser::RawMessage;

/// Communication errors for transceiver interfaces
#[derive(Debug, Clone, PartialEq)]
pub enum CommError {
    /// Connection failed or lost
    ConnectionFailed(String),
    /// Timeout waiting for data
    Timeout { timeout_ms: u64 },
    /// Invalid data received
    InvalidData { details: String },
    /// Hardware error
    HardwareError { error_code: u16, details: String },
    /// Buffer overflow
    BufferOverflow { buffer_size: usize },
    /// Configuration error
    ConfigurationError(String),
    /// Checksum or integrity error
    IntegrityError { details: String },
    /// Transceiver not responding
    NoResponse { attempts: u32 },
    /// Unsupported operation
    UnsupportedOperation { operation: String },
    /// Authentication failed
    AuthenticationFailed,
    /// Not connected to remote service
    NotConnected,
    /// Retry limit exceeded
    RetryLimitExceeded,
    /// Serialization/deserialization error
    SerializationError(String),
}

impl std::fmt::Display for CommError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CommError::ConnectionFailed(details) => {
                write!(f, "Connection failed: {}", details)
            }
            CommError::Timeout { timeout_ms } => {
                write!(f, "Timeout after {} ms", timeout_ms)
            }
            CommError::InvalidData { details } => {
                write!(f, "Invalid data: {}", details)
            }
            CommError::HardwareError { error_code, details } => {
                write!(f, "Hardware error 0x{:04X}: {}", error_code, details)
            }
            CommError::BufferOverflow { buffer_size } => {
                write!(f, "Buffer overflow (size: {})", buffer_size)
            }
            CommError::ConfigurationError(details) => {
                write!(f, "Configuration error: {}", details)
            }
            CommError::IntegrityError { details } => {
                write!(f, "Data integrity error: {}", details)
            }
            CommError::NoResponse { attempts } => {
                write!(f, "No response after {} attempts", attempts)
            }
            CommError::UnsupportedOperation { operation } => {
                write!(f, "Unsupported operation: {}", operation)
            }
            CommError::AuthenticationFailed => {
                write!(f, "Authentication failed")
            }
            CommError::NotConnected => {
                write!(f, "Not connected to remote service")
            }
            CommError::RetryLimitExceeded => {
                write!(f, "Retry limit exceeded")
            }
            CommError::SerializationError(details) => {
                write!(f, "Serialization error: {}", details)
            }
        }
    }
}

impl std::error::Error for CommError {}

/// Transceiver status information
#[derive(Debug, Clone, PartialEq)]
pub struct TransceiverStatus {
    pub is_connected: bool,
    pub signal_strength: Option<u8>,
    pub battery_level: Option<u8>,
    pub temperature: Option<i16>, // Celsius * 100
    pub error_count: u32,
    pub last_message_time: Option<Instant>,
    pub firmware_version: Option<String>,
    pub hardware_id: Option<u16>,
}

impl Default for TransceiverStatus {
    fn default() -> Self {
        Self {
            is_connected: false,
            signal_strength: None,
            battery_level: None,
            temperature: None,
            error_count: 0,
            last_message_time: None,
            firmware_version: None,
            hardware_id: None,
        }
    }
}

/// Transceiver configuration parameters
#[derive(Debug, Clone)]
pub struct TransceiverConfig {
    pub baud_rate: u32,
    pub timeout_ms: u64,
    pub buffer_size: usize,
    pub retry_attempts: u32,
    pub enable_flow_control: bool,
    pub enable_error_correction: bool,
    pub power_mode: PowerMode,
    pub frequency_channel: Option<u8>,
}

impl Default for TransceiverConfig {
    fn default() -> Self {
        Self {
            baud_rate: 9600,
            timeout_ms: 1000,
            buffer_size: 1024,
            retry_attempts: 3,
            enable_flow_control: false,
            enable_error_correction: true,
            power_mode: PowerMode::Normal,
            frequency_channel: None,
        }
    }
}

/// Power management modes for battery optimization
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PowerMode {
    /// Full power operation
    Normal,
    /// Reduced power with longer response times
    PowerSave,
    /// Minimal power, periodic wake-up
    Sleep,
    /// Emergency low power mode
    Emergency,
}

/// Transmission status for beacon operations
#[derive(Debug, Clone, PartialEq)]
pub struct TransmissionStatus {
    pub is_transmitting: bool,
    pub last_transmission_time: Option<Instant>,
    pub transmission_count: u64,
    pub transmission_failures: u32,
    pub current_power_level: u8,
    pub transmission_stats: TransmissionStats,
}

/// Detailed transmission statistics for beacon operations
#[derive(Debug, Clone, PartialEq)]
pub struct TransmissionStats {
    pub total_bytes_transmitted: u64,
    pub average_transmission_time_ms: f64,
    pub successful_transmissions: u64,
    pub failed_transmissions: u64,
    pub power_level_history: Vec<u8>,
    pub transmission_intervals_ms: Vec<u64>,
    pub last_error: Option<CommError>,
    pub error_count_by_type: std::collections::HashMap<String, u32>,
}

impl Default for TransmissionStatus {
    fn default() -> Self {
        Self {
            is_transmitting: false,
            last_transmission_time: None,
            transmission_count: 0,
            transmission_failures: 0,
            current_power_level: 128, // Mid-range power
            transmission_stats: TransmissionStats::default(),
        }
    }
}

impl Default for TransmissionStats {
    fn default() -> Self {
        Self {
            total_bytes_transmitted: 0,
            average_transmission_time_ms: 0.0,
            successful_transmissions: 0,
            failed_transmissions: 0,
            power_level_history: Vec::new(),
            transmission_intervals_ms: Vec::new(),
            last_error: None,
            error_count_by_type: std::collections::HashMap::new(),
        }
    }
}

impl TransmissionStats {
    /// Record a successful transmission
    pub fn record_successful_transmission(&mut self, bytes: usize, duration_ms: u64, power_level: u8) {
        self.successful_transmissions += 1;
        self.total_bytes_transmitted += bytes as u64;
        self.power_level_history.push(power_level);
        
        // Update average transmission time
        let total_time = self.average_transmission_time_ms * (self.successful_transmissions - 1) as f64;
        self.average_transmission_time_ms = (total_time + duration_ms as f64) / self.successful_transmissions as f64;
        
        // Keep history limited to prevent memory growth
        if self.power_level_history.len() > 100 {
            self.power_level_history.remove(0);
        }
    }
    
    /// Record a failed transmission
    pub fn record_failed_transmission(&mut self, error: CommError) {
        self.failed_transmissions += 1;
        self.last_error = Some(error.clone());
        
        // Count errors by type
        let error_type = match error {
            CommError::ConnectionFailed(_) => "ConnectionFailed",
            CommError::Timeout { .. } => "Timeout",
            CommError::HardwareError { .. } => "HardwareError",
            CommError::BufferOverflow { .. } => "BufferOverflow",
            CommError::IntegrityError { .. } => "IntegrityError",
            CommError::ConfigurationError(_) => "ConfigurationError",
            CommError::AuthenticationFailed => "AuthenticationFailed",
            CommError::NotConnected => "NotConnected",
            CommError::RetryLimitExceeded => "RetryLimitExceeded",
            CommError::SerializationError(_) => "SerializationError",
            _ => "Other",
        };
        
        *self.error_count_by_type.entry(error_type.to_string()).or_insert(0) += 1;
    }
    
    /// Record transmission interval for adaptive timing
    pub fn record_transmission_interval(&mut self, interval_ms: u64) {
        self.transmission_intervals_ms.push(interval_ms);
        
        // Keep history limited
        if self.transmission_intervals_ms.len() > 50 {
            self.transmission_intervals_ms.remove(0);
        }
    }
    
    /// Get transmission success rate
    pub fn get_success_rate(&self) -> f64 {
        let total = self.successful_transmissions + self.failed_transmissions;
        if total == 0 {
            0.0
        } else {
            self.successful_transmissions as f64 / total as f64
        }
    }
    
    /// Get average transmission interval
    pub fn get_average_interval_ms(&self) -> Option<f64> {
        if self.transmission_intervals_ms.is_empty() {
            None
        } else {
            let sum: u64 = self.transmission_intervals_ms.iter().sum();
            Some(sum as f64 / self.transmission_intervals_ms.len() as f64)
        }
    }
}

/// Environmental conditions for adaptive transmission
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct EnvironmentalConditions {
    pub water_temperature_c: Option<f32>,
    pub salinity_ppt: Option<f32>,
    pub depth_m: Option<f32>,
    pub current_speed_ms: Option<f32>,
    pub noise_level_db: Option<f32>,
    pub signal_attenuation_factor: Option<f32>,
}

impl Default for EnvironmentalConditions {
    fn default() -> Self {
        Self {
            water_temperature_c: None,
            salinity_ppt: Some(35.0), // Typical seawater salinity
            depth_m: None,
            current_speed_ms: None,
            noise_level_db: None,
            signal_attenuation_factor: None,
        }
    }
}

/// Hardware abstraction trait for transceiver communication
pub trait TransceiverInterface {
    /// Read a raw message from the transceiver
    /// Returns Ok(Some(message)) if data available, Ok(None) if no data, Err on error
    fn read_message(&mut self) -> Result<Option<RawMessage>, CommError>;
    
    /// Transmit a message (for beacon functionality)
    fn transmit_message(&mut self, data: &[u8]) -> Result<(), CommError>;
    
    /// Transmit a message with timing measurement for statistics
    fn transmit_message_timed(&mut self, data: &[u8]) -> Result<Duration, CommError> {
        let start = Instant::now();
        self.transmit_message(data)?;
        Ok(start.elapsed())
    }
    
    /// Set transmission power level (0-255, for beacon functionality)
    fn set_transmission_power(&mut self, power_level: u8) -> Result<(), CommError>;
    
    /// Adapt transmission power based on environmental conditions (requirement 6.1)
    fn adapt_transmission_power(&mut self, conditions: &EnvironmentalConditions) -> Result<u8, CommError> {
        let mut power_level = 128u8; // Start with mid-range power
        
        // Adjust for depth - deeper water requires more power
        if let Some(depth) = conditions.depth_m {
            power_level = (power_level as f32 * (1.0 + depth / 100.0).min(2.0)) as u8;
        }
        
        // Adjust for temperature - colder water has better sound transmission
        if let Some(temp) = conditions.water_temperature_c {
            let temp_factor = if temp < 10.0 { 0.9 } else if temp > 25.0 { 1.1 } else { 1.0 };
            power_level = (power_level as f32 * temp_factor) as u8;
        }
        
        // Adjust for noise level
        if let Some(noise) = conditions.noise_level_db {
            let noise_factor = if noise > 80.0 { 1.3 } else if noise < 40.0 { 0.8 } else { 1.0 };
            power_level = (power_level as f32 * noise_factor) as u8;
        }
        
        // Apply signal attenuation factor if available
        if let Some(attenuation) = conditions.signal_attenuation_factor {
            power_level = (power_level as f32 * (1.0 + attenuation)).min(255.0) as u8;
        }
        
        // Ensure power level is within valid range
        power_level = power_level.clamp(10, 255);
        
        self.set_transmission_power(power_level)?;
        Ok(power_level)
    }
    
    /// Get transmission status (for beacon functionality)
    fn get_transmission_status(&self) -> TransmissionStatus;
    
    /// Get current transceiver status
    fn get_status(&self) -> TransceiverStatus;
    
    /// Configure transceiver parameters
    fn configure(&mut self, config: TransceiverConfig) -> Result<(), CommError>;
    
    /// Send command to transceiver (for configuration/control)
    fn send_command(&mut self, command: &[u8]) -> Result<Vec<u8>, CommError>;
    
    /// Check if transceiver is connected and responsive
    fn is_connected(&self) -> bool;
    
    /// Reset transceiver to default state
    fn reset(&mut self) -> Result<(), CommError>;
    
    /// Get transceiver identification
    fn get_id(&self) -> u8;
    
    /// Flush input/output buffers
    fn flush_buffers(&mut self) -> Result<(), CommError>;
    
    /// Set power mode for battery optimization
    fn set_power_mode(&mut self, mode: PowerMode) -> Result<(), CommError>;
}

/// Connection statistics for diagnostics
#[derive(Debug, Clone, Default)]
pub struct ConnectionStats {
    pub messages_received: u64,
    pub messages_transmitted: u64,
    pub messages_failed: u64,
    pub bytes_received: u64,
    pub bytes_transmitted: u64,
    pub connection_errors: u32,
    pub timeout_errors: u32,
    pub integrity_errors: u32,
    pub last_error: Option<CommError>,
    pub uptime: Duration,
    pub connection_start: Option<Instant>,
}

impl ConnectionStats {
    pub fn new() -> Self {
        Self {
            connection_start: Some(Instant::now()),
            ..Default::default()
        }
    }
    
    pub fn record_message_received(&mut self, bytes: usize) {
        self.messages_received += 1;
        self.bytes_received += bytes as u64;
    }
    
    pub fn record_message_transmitted(&mut self, bytes: usize) {
        self.messages_transmitted += 1;
        self.bytes_transmitted += bytes as u64;
    }
    
    pub fn record_error(&mut self, error: CommError) {
        self.messages_failed += 1;
        match error {
            CommError::ConnectionFailed { .. } => self.connection_errors += 1,
            CommError::Timeout { .. } => self.timeout_errors += 1,
            CommError::IntegrityError { .. } => self.integrity_errors += 1,
            _ => {}
        }
        self.last_error = Some(error);
    }
    
    pub fn get_success_rate(&self) -> f64 {
        let total = self.messages_received + self.messages_transmitted + self.messages_failed;
        if total == 0 {
            0.0
        } else {
            (self.messages_received + self.messages_transmitted) as f64 / total as f64
        }
    }
    
    pub fn update_uptime(&mut self) {
        if let Some(start) = self.connection_start {
            self.uptime = start.elapsed();
        }
    }
}

/// Base transceiver implementation with common functionality
pub struct BaseTransceiver {
    pub id: u8,
    pub config: TransceiverConfig,
    pub status: TransceiverStatus,
    pub transmission_status: TransmissionStatus,
    pub stats: ConnectionStats,
    pub message_buffer: VecDeque<RawMessage>,
}

impl BaseTransceiver {
    pub fn new(id: u8) -> Self {
        Self {
            id,
            config: TransceiverConfig::default(),
            status: TransceiverStatus::default(),
            transmission_status: TransmissionStatus::default(),
            stats: ConnectionStats::new(),
            message_buffer: VecDeque::new(),
        }
    }
    
    /// Common message validation logic
    pub fn validate_message_data(&self, data: &[u8]) -> Result<(), CommError> {
        if data.is_empty() {
            return Err(CommError::InvalidData {
                details: "Empty message data".to_string(),
            });
        }
        
        if data.len() > self.config.buffer_size {
            return Err(CommError::BufferOverflow {
                buffer_size: self.config.buffer_size,
            });
        }
        
        // Basic integrity check - ensure message has minimum required fields
        if data.len() < 4 {
            return Err(CommError::InvalidData {
                details: "Message too short for valid format".to_string(),
            });
        }
        
        Ok(())
    }
    
    /// Create raw message from validated data
    pub fn create_raw_message(&mut self, data: Vec<u8>, signal_strength: Option<u8>) -> RawMessage {
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;
            
        self.stats.record_message_received(data.len());
        self.status.last_message_time = Some(Instant::now());
        
        RawMessage {
            data,
            timestamp_received: timestamp,
            transceiver_id: self.id,
            signal_strength,
        }
    }
    
    /// Record successful transmission with detailed statistics
    pub fn record_transmission(&mut self, data_len: usize) {
        self.stats.record_message_transmitted(data_len);
        self.transmission_status.transmission_count += 1;
        self.transmission_status.last_transmission_time = Some(Instant::now());
    }
    
    /// Record successful transmission with timing and power level
    pub fn record_transmission_with_stats(&mut self, data_len: usize, duration: Duration, power_level: u8) {
        self.record_transmission(data_len);
        self.transmission_status.transmission_stats.record_successful_transmission(
            data_len, 
            duration.as_millis() as u64, 
            power_level
        );
    }
    
    /// Record transmission failure with error details
    pub fn record_transmission_failure(&mut self) {
        self.transmission_status.transmission_failures += 1;
    }
    
    /// Record transmission failure with error details
    pub fn record_transmission_failure_with_error(&mut self, error: CommError) {
        self.record_transmission_failure();
        self.transmission_status.transmission_stats.record_failed_transmission(error);
    }
    
    /// Update connection status
    pub fn update_status(&mut self, connected: bool, signal_strength: Option<u8>) {
        self.status.is_connected = connected;
        self.status.signal_strength = signal_strength;
        self.stats.update_uptime();
    }
}

/// Mock transceiver implementation for testing and development
pub struct MockTransceiver {
    base: BaseTransceiver,
    simulated_messages: VecDeque<Vec<u8>>,
    simulate_errors: bool,
    error_probability: f64,
    response_delay: Duration,
    is_initialized: bool,
}

impl MockTransceiver {
    pub fn new(id: u8) -> Self {
        Self {
            base: BaseTransceiver::new(id),
            simulated_messages: VecDeque::new(),
            simulate_errors: false,
            error_probability: 0.0,
            response_delay: Duration::from_millis(10),
            is_initialized: false,
        }
    }
    
    /// Add simulated message data for testing
    pub fn add_test_message(&mut self, data: Vec<u8>) {
        self.simulated_messages.push_back(data);
    }
    
    /// Enable error simulation for testing error handling
    pub fn enable_error_simulation(&mut self, probability: f64) {
        self.simulate_errors = true;
        self.error_probability = probability.clamp(0.0, 1.0);
    }
    
    /// Set simulated response delay
    pub fn set_response_delay(&mut self, delay: Duration) {
        self.response_delay = delay;
    }
    
    /// Create test anchor message data
    pub fn create_test_anchor_message(anchor_id: u16, lat: f64, lon: f64, depth: f64) -> Vec<u8> {
        let mut data = Vec::new();
        
        // Version 1 message format
        data.push(1u8); // version
        data.extend_from_slice(&anchor_id.to_le_bytes()); // anchor_id
        
        // Current timestamp
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;
        data.extend_from_slice(&timestamp.to_le_bytes()); // timestamp
        
        data.extend_from_slice(&lat.to_le_bytes()); // latitude
        data.extend_from_slice(&lon.to_le_bytes()); // longitude
        data.extend_from_slice(&(depth as f32).to_le_bytes()); // depth
        data.push(200u8); // signal_quality
        data.extend_from_slice(&1u16.to_le_bytes()); // sequence
        
        // Calculate checksum (simple CRC16)
        let mut crc: u16 = 0xFFFF;
        for &byte in &data {
            crc ^= byte as u16;
            for _ in 0..8 {
                if crc & 1 != 0 {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        data.extend_from_slice(&crc.to_le_bytes()); // checksum
        
        data
    }
    
    /// Set whether transmission should fail for testing
    pub fn set_should_fail_transmission(&mut self, should_fail: bool) {
        if should_fail {
            self.enable_error_simulation(1.0); // 100% failure rate
        } else {
            self.simulate_errors = false;
            self.error_probability = 0.0;
        }
    }
    
    /// Simulate random error for testing
    fn should_simulate_error(&self) -> bool {
        if !self.simulate_errors {
            return false;
        }
        
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};
        
        let mut hasher = DefaultHasher::new();
        SystemTime::now().hash(&mut hasher);
        let hash = hasher.finish();
        
        (hash as f64 / u64::MAX as f64) < self.error_probability
    }
}

impl TransceiverInterface for MockTransceiver {
    fn read_message(&mut self) -> Result<Option<RawMessage>, CommError> {
        if !self.is_initialized {
            return Err(CommError::ConnectionFailed("Transceiver not initialized".to_string()));
        }
        
        // Simulate response delay
        std::thread::sleep(self.response_delay);
        
        // Simulate random errors
        if self.should_simulate_error() {
            let error = CommError::Timeout { timeout_ms: 1000 };
            self.base.stats.record_error(error.clone());
            return Err(error);
        }
        
        // Return simulated message if available
        if let Some(data) = self.simulated_messages.pop_front() {
            match self.base.validate_message_data(&data) {
                Ok(()) => {
                    let signal_strength = Some(180 + (data.len() % 50) as u8); // Simulate varying signal
                    let raw_message = self.base.create_raw_message(data, signal_strength);
                    self.base.update_status(true, signal_strength);
                    Ok(Some(raw_message))
                }
                Err(e) => {
                    self.base.stats.record_error(e.clone());
                    Err(e)
                }
            }
        } else {
            // No data available
            Ok(None)
        }
    }
    
    fn transmit_message(&mut self, data: &[u8]) -> Result<(), CommError> {
        if !self.is_initialized {
            return Err(CommError::ConnectionFailed("Transceiver not initialized".to_string()));
        }
        
        // Validate message data
        self.base.validate_message_data(data)?;
        
        let start_time = Instant::now();
        
        // Simulate transmission delay
        std::thread::sleep(Duration::from_millis(20));
        
        let transmission_duration = start_time.elapsed();
        
        // Simulate transmission errors
        if self.should_simulate_error() {
            let error = CommError::HardwareError {
                error_code: 0x1001,
                details: "Simulated transmission failure".to_string(),
            };
            self.base.record_transmission_failure_with_error(error.clone());
            return Err(error);
        }
        
        // Record successful transmission with detailed statistics
        let power_level = self.base.transmission_status.current_power_level;
        self.base.record_transmission_with_stats(data.len(), transmission_duration, power_level);
        
        Ok(())
    }
    
    fn set_transmission_power(&mut self, power_level: u8) -> Result<(), CommError> {
        if !self.is_initialized {
            return Err(CommError::ConnectionFailed("Transceiver not initialized".to_string()));
        }
        
        self.base.transmission_status.current_power_level = power_level;
        Ok(())
    }
    
    fn get_transmission_status(&self) -> TransmissionStatus {
        self.base.transmission_status.clone()
    }
    
    fn get_status(&self) -> TransceiverStatus {
        let mut status = self.base.status.clone();
        if self.is_initialized {
            status.firmware_version = Some("MockTransceiver v1.0".to_string());
            status.hardware_id = Some(0x1234);
            status.battery_level = Some(85); // Simulate 85% battery
            status.temperature = Some(2350); // 23.5°C
        }
        status
    }
    
    fn configure(&mut self, config: TransceiverConfig) -> Result<(), CommError> {
        // Validate configuration parameters
        if config.baud_rate == 0 {
            return Err(CommError::ConfigurationError("Baud rate cannot be zero".to_string()));
        }
        
        if config.buffer_size < 64 {
            return Err(CommError::ConfigurationError("Buffer size too small (minimum 64 bytes)".to_string()));
        }
        
        self.base.config = config;
        self.is_initialized = true;
        self.base.update_status(true, Some(200));
        
        Ok(())
    }
    
    fn send_command(&mut self, command: &[u8]) -> Result<Vec<u8>, CommError> {
        if !self.is_initialized {
            return Err(CommError::ConnectionFailed("Transceiver not initialized".to_string()));
        }
        
        // Simulate command processing delay
        std::thread::sleep(Duration::from_millis(50));
        
        // Mock command responses
        match command.get(0) {
            Some(0x01) => Ok(b"OK\r\n".to_vec()), // Generic OK response
            Some(0x02) => Ok(b"STATUS:READY\r\n".to_vec()), // Status query
            Some(0x03) => Ok(b"VERSION:1.0\r\n".to_vec()), // Version query
            Some(0x04) => Ok(b"SIGNAL:200\r\n".to_vec()), // Signal strength
            _ => Err(CommError::UnsupportedOperation {
                operation: format!("Command 0x{:02X}", command.get(0).unwrap_or(&0)),
            }),
        }
    }
    
    fn is_connected(&self) -> bool {
        self.is_initialized && self.base.status.is_connected
    }
    
    fn reset(&mut self) -> Result<(), CommError> {
        self.is_initialized = false;
        self.base.status = TransceiverStatus::default();
        self.base.transmission_status = TransmissionStatus::default();
        self.base.message_buffer.clear();
        self.simulated_messages.clear();
        
        // Simulate reset delay
        std::thread::sleep(Duration::from_millis(100));
        
        self.is_initialized = true;
        self.base.update_status(true, Some(200));
        
        Ok(())
    }
    
    fn get_id(&self) -> u8 {
        self.base.id
    }
    
    fn flush_buffers(&mut self) -> Result<(), CommError> {
        self.base.message_buffer.clear();
        self.simulated_messages.clear();
        Ok(())
    }
    
    fn set_power_mode(&mut self, mode: PowerMode) -> Result<(), CommError> {
        self.base.config.power_mode = mode;
        
        // Simulate power mode effects
        match mode {
            PowerMode::Normal => {
                self.response_delay = Duration::from_millis(10);
            }
            PowerMode::PowerSave => {
                self.response_delay = Duration::from_millis(50);
            }
            PowerMode::Sleep => {
                self.response_delay = Duration::from_millis(200);
            }
            PowerMode::Emergency => {
                self.response_delay = Duration::from_millis(500);
            }
        }
        
        Ok(())
    }
}

/// Serial/UART transceiver implementation for JANUS transceivers
pub struct SerialTransceiver {
    base: BaseTransceiver,
    port_name: String,
    is_open: bool,
    read_buffer: Vec<u8>,
    write_buffer: Vec<u8>,
}

impl SerialTransceiver {
    pub fn new(id: u8, port_name: String) -> Self {
        Self {
            base: BaseTransceiver::new(id),
            port_name,
            is_open: false,
            read_buffer: Vec::new(),
            write_buffer: Vec::new(),
        }
    }
    
    /// Open serial port connection
    pub fn open(&mut self) -> Result<(), CommError> {
        if self.is_open {
            return Ok(());
        }
        
        // In a real implementation, this would open the actual serial port
        // For now, simulate the connection
        self.is_open = true;
        self.base.update_status(true, Some(180));
        
        Ok(())
    }
    
    /// Close serial port connection
    pub fn close(&mut self) -> Result<(), CommError> {
        if !self.is_open {
            return Ok(());
        }
        
        self.is_open = false;
        self.base.update_status(false, None);
        self.read_buffer.clear();
        self.write_buffer.clear();
        
        Ok(())
    }
}

impl TransceiverInterface for SerialTransceiver {
    fn read_message(&mut self) -> Result<Option<RawMessage>, CommError> {
        if !self.is_open {
            return Err(CommError::ConnectionFailed("Serial port not open".to_string()));
        }
        
        // In a real implementation, this would read from actual serial port
        // For simulation, return None (no data available)
        Ok(None)
    }
    
    fn transmit_message(&mut self, data: &[u8]) -> Result<(), CommError> {
        if !self.is_open {
            return Err(CommError::ConnectionFailed("Serial port not open".to_string()));
        }
        
        // Validate message data
        self.base.validate_message_data(data)?;
        
        let start_time = Instant::now();
        
        // In a real implementation, this would write to actual serial port
        // Simulate transmission time
        std::thread::sleep(Duration::from_millis(10));
        
        let transmission_duration = start_time.elapsed();
        
        // Record successful transmission with detailed statistics
        let power_level = self.base.transmission_status.current_power_level;
        self.base.record_transmission_with_stats(data.len(), transmission_duration, power_level);
        
        Ok(())
    }
    
    fn set_transmission_power(&mut self, power_level: u8) -> Result<(), CommError> {
        if !self.is_open {
            return Err(CommError::ConnectionFailed("Serial port not open".to_string()));
        }
        
        self.base.transmission_status.current_power_level = power_level;
        Ok(())
    }
    
    fn get_transmission_status(&self) -> TransmissionStatus {
        self.base.transmission_status.clone()
    }
    
    fn get_status(&self) -> TransceiverStatus {
        let mut status = self.base.status.clone();
        if self.is_open {
            status.firmware_version = Some("JANUS Serial v2.1".to_string());
            status.hardware_id = Some(0x5678);
        }
        status
    }
    
    fn configure(&mut self, config: TransceiverConfig) -> Result<(), CommError> {
        // Validate serial-specific parameters
        if config.baud_rate < 1200 || config.baud_rate > 115200 {
            return Err(CommError::ConfigurationError("Baud rate must be between 1200 and 115200".to_string()));
        }
        
        self.base.config = config;
        
        // Apply configuration to serial port
        if self.is_open {
            // In real implementation, reconfigure the serial port here
            self.base.update_status(true, Some(180));
        }
        
        Ok(())
    }
    
    fn send_command(&mut self, command: &[u8]) -> Result<Vec<u8>, CommError> {
        if !self.is_open {
            return Err(CommError::ConnectionFailed("Serial port not open".to_string()));
        }
        
        // In a real implementation, this would send command to serial port
        // For simulation, return mock responses
        match command.get(0) {
            Some(0x01) => Ok(b"OK\r\n".to_vec()),
            Some(0x02) => Ok(b"STATUS:READY\r\n".to_vec()),
            _ => Err(CommError::UnsupportedOperation {
                operation: format!("Serial command 0x{:02X}", command.get(0).unwrap_or(&0)),
            }),
        }
    }
    
    fn is_connected(&self) -> bool {
        self.is_open && self.base.status.is_connected
    }
    
    fn reset(&mut self) -> Result<(), CommError> {
        self.close()?;
        std::thread::sleep(Duration::from_millis(100));
        self.open()
    }
    
    fn get_id(&self) -> u8 {
        self.base.id
    }
    
    fn flush_buffers(&mut self) -> Result<(), CommError> {
        self.base.message_buffer.clear();
        self.read_buffer.clear();
        self.write_buffer.clear();
        Ok(())
    }
    
    fn set_power_mode(&mut self, mode: PowerMode) -> Result<(), CommError> {
        self.base.config.power_mode = mode;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_transmission_status_default() {
        let status = TransmissionStatus::default();
        assert!(!status.is_transmitting);
        assert_eq!(status.transmission_count, 0);
        assert_eq!(status.transmission_failures, 0);
        assert_eq!(status.current_power_level, 128);
        assert_eq!(status.transmission_stats.successful_transmissions, 0);
    }

    #[test]
    fn test_transmission_stats_record_success() {
        let mut stats = TransmissionStats::default();
        
        stats.record_successful_transmission(100, 50, 200);
        
        assert_eq!(stats.successful_transmissions, 1);
        assert_eq!(stats.total_bytes_transmitted, 100);
        assert_eq!(stats.average_transmission_time_ms, 50.0);
        assert_eq!(stats.power_level_history.len(), 1);
        assert_eq!(stats.power_level_history[0], 200);
    }

    #[test]
    fn test_transmission_stats_record_failure() {
        let mut stats = TransmissionStats::default();
        let error = CommError::Timeout { timeout_ms: 1000 };
        
        stats.record_failed_transmission(error.clone());
        
        assert_eq!(stats.failed_transmissions, 1);
        assert_eq!(stats.last_error, Some(error));
        assert_eq!(stats.error_count_by_type.get("Timeout"), Some(&1));
    }

    #[test]
    fn test_transmission_stats_success_rate() {
        let mut stats = TransmissionStats::default();
        
        // Record some successes and failures
        stats.record_successful_transmission(100, 50, 200);
        stats.record_successful_transmission(100, 50, 200);
        stats.record_failed_transmission(CommError::Timeout { timeout_ms: 1000 });
        
        let success_rate = stats.get_success_rate();
        assert!((success_rate - 0.6667).abs() < 0.001); // 2/3 ≈ 0.6667
    }

    #[test]
    fn test_environmental_conditions_default() {
        let conditions = EnvironmentalConditions::default();
        assert_eq!(conditions.salinity_ppt, Some(35.0));
        assert_eq!(conditions.water_temperature_c, None);
    }

    #[test]
    fn test_mock_transceiver_transmission() {
        let mut transceiver = MockTransceiver::new(1);
        let config = TransceiverConfig::default();
        transceiver.configure(config).unwrap();
        
        let test_data = b"test message";
        let result = transceiver.transmit_message(test_data);
        
        assert!(result.is_ok());
        
        let status = transceiver.get_transmission_status();
        assert_eq!(status.transmission_count, 1);
        assert_eq!(status.transmission_stats.successful_transmissions, 1);
        assert_eq!(status.transmission_stats.total_bytes_transmitted, test_data.len() as u64);
    }

    #[test]
    fn test_mock_transceiver_transmission_with_errors() {
        let mut transceiver = MockTransceiver::new(1);
        let config = TransceiverConfig::default();
        transceiver.configure(config).unwrap();
        transceiver.enable_error_simulation(1.0); // 100% error rate
        
        let test_data = b"test message";
        let result = transceiver.transmit_message(test_data);
        
        assert!(result.is_err());
        
        let status = transceiver.get_transmission_status();
        assert_eq!(status.transmission_failures, 1);
        assert_eq!(status.transmission_stats.failed_transmissions, 1);
        assert!(status.transmission_stats.last_error.is_some());
    }

    #[test]
    fn test_transmission_power_control() {
        let mut transceiver = MockTransceiver::new(1);
        let config = TransceiverConfig::default();
        transceiver.configure(config).unwrap();
        
        let result = transceiver.set_transmission_power(200);
        assert!(result.is_ok());
        
        let status = transceiver.get_transmission_status();
        assert_eq!(status.current_power_level, 200);
    }

    #[test]
    fn test_adaptive_transmission_power_depth() {
        let mut transceiver = MockTransceiver::new(1);
        let config = TransceiverConfig::default();
        transceiver.configure(config).unwrap();
        
        let mut conditions = EnvironmentalConditions::default();
        conditions.depth_m = Some(50.0); // 50 meter depth
        
        let power_level = transceiver.adapt_transmission_power(&conditions).unwrap();
        
        // Power should be increased for deeper water
        assert!(power_level > 128); // Should be higher than default mid-range
        
        let status = transceiver.get_transmission_status();
        assert_eq!(status.current_power_level, power_level);
    }

    #[test]
    fn test_adaptive_transmission_power_temperature() {
        let mut transceiver = MockTransceiver::new(1);
        let config = TransceiverConfig::default();
        transceiver.configure(config).unwrap();
        
        // Test cold water (better transmission)
        let mut conditions = EnvironmentalConditions::default();
        conditions.water_temperature_c = Some(5.0);
        
        let cold_power = transceiver.adapt_transmission_power(&conditions).unwrap();
        
        // Test warm water (worse transmission)
        conditions.water_temperature_c = Some(30.0);
        let warm_power = transceiver.adapt_transmission_power(&conditions).unwrap();
        
        // Warm water should require more power than cold water
        assert!(warm_power > cold_power);
    }

    #[test]
    fn test_adaptive_transmission_power_noise() {
        let mut transceiver = MockTransceiver::new(1);
        let config = TransceiverConfig::default();
        transceiver.configure(config).unwrap();
        
        // Test high noise environment
        let mut conditions = EnvironmentalConditions::default();
        conditions.noise_level_db = Some(90.0);
        
        let high_noise_power = transceiver.adapt_transmission_power(&conditions).unwrap();
        
        // Test low noise environment
        conditions.noise_level_db = Some(30.0);
        let low_noise_power = transceiver.adapt_transmission_power(&conditions).unwrap();
        
        // High noise should require more power than low noise
        assert!(high_noise_power > low_noise_power);
    }

    #[test]
    fn test_adaptive_transmission_power_bounds() {
        let mut transceiver = MockTransceiver::new(1);
        let config = TransceiverConfig::default();
        transceiver.configure(config).unwrap();
        
        // Test extreme conditions that would push power beyond limits
        let mut conditions = EnvironmentalConditions::default();
        conditions.depth_m = Some(1000.0); // Very deep
        conditions.noise_level_db = Some(120.0); // Very noisy
        conditions.signal_attenuation_factor = Some(2.0); // High attenuation
        
        let power_level = transceiver.adapt_transmission_power(&conditions).unwrap();
        
        // Power should be clamped to maximum (255)
        assert_eq!(power_level, 255);
    }

    #[test]
    fn test_transmission_timed() {
        let mut transceiver = MockTransceiver::new(1);
        let config = TransceiverConfig::default();
        transceiver.configure(config).unwrap();
        
        let test_data = b"test message";
        let duration = transceiver.transmit_message_timed(test_data).unwrap();
        
        // Should have some measurable duration
        assert!(duration.as_millis() > 0);
        
        let status = transceiver.get_transmission_status();
        assert_eq!(status.transmission_count, 1);
        assert!(status.transmission_stats.average_transmission_time_ms > 0.0);
    }

    #[test]
    fn test_transmission_interval_tracking() {
        let mut stats = TransmissionStats::default();
        
        stats.record_transmission_interval(1000);
        stats.record_transmission_interval(1500);
        stats.record_transmission_interval(800);
        
        let avg_interval = stats.get_average_interval_ms().unwrap();
        assert!((avg_interval - 1100.0).abs() < 0.1); // (1000+1500+800)/3 = 1100
    }

    #[test]
    fn test_serial_transceiver_transmission() {
        let mut transceiver = SerialTransceiver::new(2, "/dev/ttyUSB0".to_string());
        transceiver.open().unwrap();
        
        let test_data = b"test message";
        let result = transceiver.transmit_message(test_data);
        
        assert!(result.is_ok());
        
        let status = transceiver.get_transmission_status();
        assert_eq!(status.transmission_count, 1);
        assert_eq!(status.transmission_stats.successful_transmissions, 1);
    }

    #[test]
    fn test_power_level_history_limit() {
        let mut stats = TransmissionStats::default();
        
        // Add more than 100 power level entries
        for i in 0..150 {
            stats.record_successful_transmission(100, 50, (i % 256) as u8);
        }
        
        // History should be limited to 100 entries
        assert_eq!(stats.power_level_history.len(), 100);
        assert_eq!(stats.successful_transmissions, 150);
    }

    #[test]
    fn test_transmission_interval_history_limit() {
        let mut stats = TransmissionStats::default();
        
        // Add more than 50 interval entries
        for i in 0..80 {
            stats.record_transmission_interval(1000 + i);
        }
        
        // History should be limited to 50 entries
        assert_eq!(stats.transmission_intervals_ms.len(), 50);
    }

    #[test]
    fn test_error_count_by_type() {
        let mut stats = TransmissionStats::default();
        
        // Record different types of errors
        stats.record_failed_transmission(CommError::Timeout { timeout_ms: 1000 });
        stats.record_failed_transmission(CommError::Timeout { timeout_ms: 2000 });
        stats.record_failed_transmission(CommError::ConnectionFailed("test".to_string()));
        stats.record_failed_transmission(CommError::HardwareError { error_code: 1, details: "test".to_string() });
        
        assert_eq!(stats.error_count_by_type.get("Timeout"), Some(&2));
        assert_eq!(stats.error_count_by_type.get("ConnectionFailed"), Some(&1));
        assert_eq!(stats.error_count_by_type.get("HardwareError"), Some(&1));
    }
}