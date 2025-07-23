// Hardware abstraction layer for transceiver communication
// Supports UART/Serial and I2C interfaces for JANUS transceivers

use std::collections::VecDeque;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};
use crate::message_parser::RawMessage;

/// Communication errors for transceiver interfaces
#[derive(Debug, Clone, PartialEq)]
pub enum CommError {
    /// Connection failed or lost
    ConnectionFailed { details: String },
    /// Timeout waiting for data
    Timeout { timeout_ms: u64 },
    /// Invalid data received
    InvalidData { details: String },
    /// Hardware error
    HardwareError { error_code: u16, details: String },
    /// Buffer overflow
    BufferOverflow { buffer_size: usize },
    /// Configuration error
    ConfigurationError { parameter: String, details: String },
    /// Checksum or integrity error
    IntegrityError { details: String },
    /// Transceiver not responding
    NoResponse { attempts: u32 },
    /// Unsupported operation
    UnsupportedOperation { operation: String },
}

impl std::fmt::Display for CommError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            CommError::ConnectionFailed { details } => {
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
            CommError::ConfigurationError { parameter, details } => {
                write!(f, "Configuration error for {}: {}", parameter, details)
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
}

impl Default for TransmissionStatus {
    fn default() -> Self {
        Self {
            is_transmitting: false,
            last_transmission_time: None,
            transmission_count: 0,
            transmission_failures: 0,
            current_power_level: 128, // Mid-range power
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
    
    /// Set transmission power level (0-255, for beacon functionality)
    fn set_transmission_power(&mut self, power_level: u8) -> Result<(), CommError>;
    
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
    
    /// Record successful transmission
    pub fn record_transmission(&mut self, data_len: usize) {
        self.stats.record_message_transmitted(data_len);
        self.transmission_status.transmission_count += 1;
        self.transmission_status.last_transmission_time = Some(Instant::now());
    }
    
    /// Record transmission failure
    pub fn record_transmission_failure(&mut self) {
        self.transmission_status.transmission_failures += 1;
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
            return Err(CommError::ConnectionFailed {
                details: "Transceiver not initialized".to_string(),
            });
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
            return Err(CommError::ConnectionFailed {
                details: "Transceiver not initialized".to_string(),
            });
        }
        
        // Validate message data
        self.base.validate_message_data(data)?;
        
        // Simulate transmission delay
        std::thread::sleep(Duration::from_millis(20));
        
        // Simulate transmission errors
        if self.should_simulate_error() {
            self.base.record_transmission_failure();
            return Err(CommError::HardwareError {
                error_code: 0x1001,
                details: "Simulated transmission failure".to_string(),
            });
        }
        
        // Record successful transmission
        self.base.record_transmission(data.len());
        
        Ok(())
    }
    
    fn set_transmission_power(&mut self, power_level: u8) -> Result<(), CommError> {
        if !self.is_initialized {
            return Err(CommError::ConnectionFailed {
                details: "Transceiver not initialized".to_string(),
            });
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
            return Err(CommError::ConfigurationError {
                parameter: "baud_rate".to_string(),
                details: "Baud rate cannot be zero".to_string(),
            });
        }
        
        if config.buffer_size < 64 {
            return Err(CommError::ConfigurationError {
                parameter: "buffer_size".to_string(),
                details: "Buffer size too small (minimum 64 bytes)".to_string(),
            });
        }
        
        self.base.config = config;
        self.is_initialized = true;
        self.base.update_status(true, Some(200));
        
        Ok(())
    }
    
    fn send_command(&mut self, command: &[u8]) -> Result<Vec<u8>, CommError> {
        if !self.is_initialized {
            return Err(CommError::ConnectionFailed {
                details: "Transceiver not initialized".to_string(),
            });
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
            return Err(CommError::ConnectionFailed {
                details: "Serial port not open".to_string(),
            });
        }
        
        // In a real implementation, this would read from actual serial port
        // For simulation, return None (no data available)
        Ok(None)
    }
    
    fn transmit_message(&mut self, data: &[u8]) -> Result<(), CommError> {
        if !self.is_open {
            return Err(CommError::ConnectionFailed {
                details: "Serial port not open".to_string(),
            });
        }
        
        // Validate message data
        self.base.validate_message_data(data)?;
        
        // In a real implementation, this would write to actual serial port
        // For simulation, just record the transmission
        self.base.record_transmission(data.len());
        
        Ok(())
    }
    
    fn set_transmission_power(&mut self, power_level: u8) -> Result<(), CommError> {
        if !self.is_open {
            return Err(CommError::ConnectionFailed {
                details: "Serial port not open".to_string(),
            });
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
            return Err(CommError::ConfigurationError {
                parameter: "baud_rate".to_string(),
                details: "Baud rate must be between 1200 and 115200".to_string(),
            });
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
            return Err(CommError::ConnectionFailed {
                details: "Serial port not open".to_string(),
            });
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