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

/// Hardware abstraction trait for transceiver communication
pub trait TransceiverInterface {
    /// Read a raw message from the transceiver
    /// Returns Ok(Some(message)) if data available, Ok(None) if no data, Err on error
    fn read_message(&mut self) -> Result<Option<RawMessage>, CommError>;
    
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
    pub messages_failed: u64,
    pub bytes_received: u64,
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
    
    pub fn record_message(&mut self, bytes: usize) {
        self.messages_received += 1;
        self.bytes_received += bytes as u64;
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
        let total = self.messages_received + self.messages_failed;
        if total == 0 {
            0.0
        } else {
            self.messages_received as f64 / total as f64
        }
    }
    
    pub fn update_uptime(&mut self) {
        if let Some(start) = self.connection_start {
            self.uptime = start.elapsed();
        }
    }
}

/// Error recovery strategies
#[derive(Debug, Clone)]
pub enum RecoveryStrategy {
    /// Retry the operation
    Retry { max_attempts: u32, delay_ms: u64 },
    /// Reset the connection
    Reset,
    /// Switch to backup transceiver
    Failover { backup_id: u8 },
    /// Ignore and continue
    Ignore,
    /// Shutdown and report critical error
    Shutdown,
}

/// Error recovery manager
pub struct ErrorRecoveryManager {
    strategies: std::collections::HashMap<String, RecoveryStrategy>,
    recovery_attempts: std::collections::HashMap<String, u32>,
    last_recovery: std::collections::HashMap<String, Instant>,
}

impl ErrorRecoveryManager {
    pub fn new() -> Self {
        let mut strategies = std::collections::HashMap::new();
        
        // Default recovery strategies
        strategies.insert("Timeout".to_string(), RecoveryStrategy::Retry { max_attempts: 3, delay_ms: 100 });
        strategies.insert("ConnectionFailed".to_string(), RecoveryStrategy::Reset);
        strategies.insert("HardwareError".to_string(), RecoveryStrategy::Reset);
        strategies.insert("IntegrityError".to_string(), RecoveryStrategy::Retry { max_attempts: 2, delay_ms: 50 });
        strategies.insert("BufferOverflow".to_string(), RecoveryStrategy::Reset);
        
        Self {
            strategies,
            recovery_attempts: std::collections::HashMap::new(),
            last_recovery: std::collections::HashMap::new(),
        }
    }
    
    pub fn handle_error(&mut self, error: &CommError) -> Option<RecoveryStrategy> {
        let error_type = match error {
            CommError::Timeout { .. } => "Timeout",
            CommError::ConnectionFailed { .. } => "ConnectionFailed",
            CommError::HardwareError { .. } => "HardwareError",
            CommError::IntegrityError { .. } => "IntegrityError",
            CommError::BufferOverflow { .. } => "BufferOverflow",
            _ => "Unknown",
        };
        
        // Check if we should apply recovery strategy
        let now = Instant::now();
        if let Some(last_time) = self.last_recovery.get(error_type) {
            if now.duration_since(*last_time) < Duration::from_millis(1000) {
                // Too soon since last recovery attempt
                return None;
            }
        }
        
        let attempts = self.recovery_attempts.entry(error_type.to_string()).or_insert(0);
        
        if let Some(strategy) = self.strategies.get(error_type) {
            match strategy {
                RecoveryStrategy::Retry { max_attempts, .. } => {
                    if *attempts < *max_attempts {
                        *attempts += 1;
                        self.last_recovery.insert(error_type.to_string(), now);
                        return Some(strategy.clone());
                    }
                }
                _ => {
                    *attempts += 1;
                    self.last_recovery.insert(error_type.to_string(), now);
                    return Some(strategy.clone());
                }
            }
        }
        
        None
    }
    
    pub fn reset_attempts(&mut self, error_type: &str) {
        self.recovery_attempts.remove(error_type);
    }
}

impl Default for ErrorRecoveryManager {
    fn default() -> Self {
        Self::new()
    }
}

/// Base transceiver implementation with common functionality
pub struct BaseTransceiver {
    pub id: u8,
    pub config: TransceiverConfig,
    pub status: TransceiverStatus,
    pub stats: ConnectionStats,
    pub recovery_manager: ErrorRecoveryManager,
    pub message_buffer: VecDeque<RawMessage>,
}

impl BaseTransceiver {
    pub fn new(id: u8) -> Self {
        Self {
            id,
            config: TransceiverConfig::default(),
            status: TransceiverStatus::default(),
            stats: ConnectionStats::new(),
            recovery_manager: ErrorRecoveryManager::new(),
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
            
        self.stats.record_message(data.len());
        self.status.last_message_time = Some(Instant::now());
        
        RawMessage {
            data,
            timestamp_received: timestamp,
            transceiver_id: self.id,
            signal_strength,
        }
    }
    
    /// Handle communication error with recovery
    pub fn handle_error(&mut self, error: CommError) -> Result<(), CommError> {
        self.stats.record_error(error.clone());
        
        if let Some(strategy) = self.recovery_manager.handle_error(&error) {
            match strategy {
                RecoveryStrategy::Retry { delay_ms, .. } => {
                    std::thread::sleep(Duration::from_millis(delay_ms));
                    // Caller should retry the operation
                    Ok(())
                }
                RecoveryStrategy::Reset => {
                    // Reset will be handled by specific implementation
                    Err(error)
                }
                RecoveryStrategy::Ignore => {
                    Ok(())
                }
                _ => Err(error),
            }
        } else {
            Err(error)
        }
    }
    
    /// Update connection status
    pub fn update_status(&mut self, connected: bool, signal_strength: Option<u8>) {
        self.status.is_connected = connected;
        self.status.signal_strength = signal_strength;
        self.stats.update_uptime();
        
        if connected {
            self.recovery_manager.reset_attempts("ConnectionFailed");
        }
    }
    
    /// Generate diagnostic report
    pub fn generate_diagnostic_report(&self) -> String {
        let mut report = String::new();
        report.push_str(&format!("=== TRANSCEIVER {} DIAGNOSTIC REPORT ===\n\n", self.id));
        
        // Status
        report.push_str("STATUS:\n");
        report.push_str(&format!("  Connected: {}\n", self.status.is_connected));
        if let Some(signal) = self.status.signal_strength {
            report.push_str(&format!("  Signal strength: {}\n", signal));
        }
        if let Some(battery) = self.status.battery_level {
            report.push_str(&format!("  Battery level: {}%\n", battery));
        }
        if let Some(temp) = self.status.temperature {
            report.push_str(&format!("  Temperature: {:.1}°C\n", temp as f32 / 100.0));
        }
        report.push_str(&format!("  Error count: {}\n", self.status.error_count));
        
        // Statistics
        report.push_str("\nSTATISTICS:\n");
        report.push_str(&format!("  Messages received: {}\n", self.stats.messages_received));
        report.push_str(&format!("  Messages failed: {}\n", self.stats.messages_failed));
        report.push_str(&format!("  Bytes received: {}\n", self.stats.bytes_received));
        report.push_str(&format!("  Success rate: {:.2}%\n", self.stats.get_success_rate() * 100.0));
        report.push_str(&format!("  Connection errors: {}\n", self.stats.connection_errors));
        report.push_str(&format!("  Timeout errors: {}\n", self.stats.timeout_errors));
        report.push_str(&format!("  Integrity errors: {}\n", self.stats.integrity_errors));
        report.push_str(&format!("  Uptime: {:.1}s\n", self.stats.uptime.as_secs_f64()));
        
        // Configuration
        report.push_str("\nCONFIGURATION:\n");
        report.push_str(&format!("  Baud rate: {}\n", self.config.baud_rate));
        report.push_str(&format!("  Timeout: {} ms\n", self.config.timeout_ms));
        report.push_str(&format!("  Buffer size: {} bytes\n", self.config.buffer_size));
        report.push_str(&format!("  Retry attempts: {}\n", self.config.retry_attempts));
        report.push_str(&format!("  Flow control: {}\n", self.config.enable_flow_control));
        report.push_str(&format!("  Error correction: {}\n", self.config.enable_error_correction));
        report.push_str(&format!("  Power mode: {:?}\n", self.config.power_mode));
        
        if let Some(error) = &self.stats.last_error {
            report.push_str(&format!("\nLAST ERROR: {}\n", error));
        }
        
        report
    }
}
///
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
    
    /// Get diagnostic report from base transceiver
    pub fn get_diagnostic_report(&self) -> String {
        self.base.generate_diagnostic_report()
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
            return self.base.handle_error(error).map(|_| None);
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
                    self.base.handle_error(e).map(|_| None)
                }
            }
        } else {
            // No data available
            Ok(None)
        }
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
    
    /// Read data from serial port with timeout
    fn read_serial_data(&mut self, _timeout: Duration) -> Result<Vec<u8>, CommError> {
        if !self.is_open {
            return Err(CommError::ConnectionFailed {
                details: "Serial port not open".to_string(),
            });
        }
        
        // Simulate serial read operation
        // In real implementation, this would read from actual serial port
        std::thread::sleep(Duration::from_millis(10));
        
        // For simulation, return empty data most of the time
        Ok(Vec::new())
    }
    
    /// Write data to serial port
    fn write_serial_data(&mut self, data: &[u8]) -> Result<usize, CommError> {
        if !self.is_open {
            return Err(CommError::ConnectionFailed {
                details: "Serial port not open".to_string(),
            });
        }
        
        // Simulate serial write operation
        std::thread::sleep(Duration::from_millis(5));
        
        Ok(data.len())
    }
    
    /// Parse message from buffer
    fn parse_message_from_buffer(&mut self) -> Option<Vec<u8>> {
        // Look for complete message in buffer
        // JANUS messages typically have a specific frame format
        if self.read_buffer.len() < 4 {
            return None;
        }
        
        // Simple frame detection: look for version byte and reasonable length
        if let Some(version) = self.read_buffer.get(0) {
            let expected_length = match version {
                1 => 36, // V1 message length
                2 => 23, // V2 message length
                _ => return None,
            };
            
            if self.read_buffer.len() >= expected_length {
                let message_data = self.read_buffer.drain(0..expected_length).collect();
                return Some(message_data);
            }
        }
        
        None
    }
}

impl TransceiverInterface for SerialTransceiver {
    fn read_message(&mut self) -> Result<Option<RawMessage>, CommError> {
        if !self.is_open {
            return Err(CommError::ConnectionFailed {
                details: "Serial port not open".to_string(),
            });
        }
        
        // Read new data from serial port
        let timeout = Duration::from_millis(self.base.config.timeout_ms);
        match self.read_serial_data(timeout) {
            Ok(data) => {
                if !data.is_empty() {
                    self.read_buffer.extend_from_slice(&data);
                }
            }
            Err(CommError::Timeout { .. }) => {
                // Timeout is normal when no data available
            }
            Err(e) => return self.base.handle_error(e).map(|_| None),
        }
        
        // Try to parse complete message from buffer
        if let Some(message_data) = self.parse_message_from_buffer() {
            match self.base.validate_message_data(&message_data) {
                Ok(()) => {
                    let signal_strength = Some(150 + (message_data.len() % 100) as u8);
                    let raw_message = self.base.create_raw_message(message_data, signal_strength);
                    self.base.update_status(true, signal_strength);
                    Ok(Some(raw_message))
                }
                Err(e) => self.base.handle_error(e).map(|_| None),
            }
        } else {
            Ok(None)
        }
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
        
        // Send command via serial port
        self.write_serial_data(command)?;
        
        // Wait for response
        let timeout = Duration::from_millis(self.base.config.timeout_ms);
        let start_time = Instant::now();
        
        while start_time.elapsed() < timeout {
            if let Ok(data) = self.read_serial_data(Duration::from_millis(10)) {
                if !data.is_empty() {
                    return Ok(data);
                }
            }
            std::thread::sleep(Duration::from_millis(10));
        }
        
        Err(CommError::Timeout {
            timeout_ms: self.base.config.timeout_ms,
        })
    }
    
    fn is_connected(&self) -> bool {
        self.is_open && self.base.status.is_connected
    }
    
    fn reset(&mut self) -> Result<(), CommError> {
        if self.is_open {
            self.close()?;
            std::thread::sleep(Duration::from_millis(100));
            self.open()?;
        }
        
        self.base.status.error_count = 0;
        self.read_buffer.clear();
        self.write_buffer.clear();
        
        Ok(())
    }
    
    fn get_id(&self) -> u8 {
        self.base.id
    }
    
    fn flush_buffers(&mut self) -> Result<(), CommError> {
        self.read_buffer.clear();
        self.write_buffer.clear();
        self.base.message_buffer.clear();
        Ok(())
    }
    
    fn set_power_mode(&mut self, mode: PowerMode) -> Result<(), CommError> {
        self.base.config.power_mode = mode;
        
        // Send power mode command to transceiver
        let power_cmd = match mode {
            PowerMode::Normal => b"\x10\x01",
            PowerMode::PowerSave => b"\x10\x02",
            PowerMode::Sleep => b"\x10\x03",
            PowerMode::Emergency => b"\x10\x04",
        };
        
        self.send_command(power_cmd)?;
        Ok(())
    }
}

/// I2C transceiver implementation for alternative interface
pub struct I2CTransceiver {
    base: BaseTransceiver,
    device_address: u8,
    bus_number: u8,
    is_initialized: bool,
    register_cache: std::collections::HashMap<u8, u8>,
}

impl I2CTransceiver {
    pub fn new(id: u8, device_address: u8, bus_number: u8) -> Self {
        Self {
            base: BaseTransceiver::new(id),
            device_address,
            bus_number,
            is_initialized: false,
            register_cache: std::collections::HashMap::new(),
        }
    }
    
    /// Initialize I2C connection
    pub fn initialize(&mut self) -> Result<(), CommError> {
        if self.is_initialized {
            return Ok(());
        }
        
        // In real implementation, this would initialize I2C bus
        // For simulation, just mark as initialized
        self.is_initialized = true;
        self.base.update_status(true, Some(190));
        
        Ok(())
    }
    
    /// Read from I2C register
    fn read_register(&mut self, register: u8) -> Result<u8, CommError> {
        if !self.is_initialized {
            return Err(CommError::ConnectionFailed {
                details: "I2C not initialized".to_string(),
            });
        }
        
        // Simulate I2C read operation
        std::thread::sleep(Duration::from_millis(1));
        
        // Return cached value or simulate data
        Ok(self.register_cache.get(&register).copied().unwrap_or(0))
    }
    
    /// Write to I2C register
    fn write_register(&mut self, register: u8, value: u8) -> Result<(), CommError> {
        if !self.is_initialized {
            return Err(CommError::ConnectionFailed {
                details: "I2C not initialized".to_string(),
            });
        }
        
        // Simulate I2C write operation
        std::thread::sleep(Duration::from_millis(1));
        
        self.register_cache.insert(register, value);
        Ok(())
    }
    
    /// Read message data from I2C registers
    fn read_message_data(&mut self) -> Result<Option<Vec<u8>>, CommError> {
        // Check if message is available (register 0x00 = status)
        let status = self.read_register(0x00)?;
        if status & 0x01 == 0 {
            return Ok(None); // No message available
        }
        
        // Read message length (register 0x01)
        let length = self.read_register(0x01)?;
        if length == 0 || length > 64 {
            return Err(CommError::InvalidData {
                details: format!("Invalid message length: {}", length),
            });
        }
        
        // Read message data from registers 0x10-0x4F
        let mut data = Vec::new();
        for i in 0..length {
            let byte = self.read_register(0x10 + i)?;
            data.push(byte);
        }
        
        // Clear message available flag
        self.write_register(0x00, status & !0x01)?;
        
        Ok(Some(data))
    }
}

impl TransceiverInterface for I2CTransceiver {
    fn read_message(&mut self) -> Result<Option<RawMessage>, CommError> {
        if !self.is_initialized {
            return Err(CommError::ConnectionFailed {
                details: "I2C transceiver not initialized".to_string(),
            });
        }
        
        match self.read_message_data() {
            Ok(Some(data)) => {
                match self.base.validate_message_data(&data) {
                    Ok(()) => {
                        let signal_strength = Some(170 + (data.len() % 80) as u8);
                        let raw_message = self.base.create_raw_message(data, signal_strength);
                        self.base.update_status(true, signal_strength);
                        Ok(Some(raw_message))
                    }
                    Err(e) => self.base.handle_error(e).map(|_| None),
                }
            }
            Ok(None) => Ok(None),
            Err(e) => self.base.handle_error(e).map(|_| None),
        }
    }
    
    fn get_status(&self) -> TransceiverStatus {
        let mut status = self.base.status.clone();
        if self.is_initialized {
            status.firmware_version = Some("JANUS I2C v1.5".to_string());
            status.hardware_id = Some(0x9ABC);
        }
        status
    }
    
    fn configure(&mut self, config: TransceiverConfig) -> Result<(), CommError> {
        // I2C doesn't use baud rate, but validate other parameters
        if config.buffer_size < 32 {
            return Err(CommError::ConfigurationError {
                parameter: "buffer_size".to_string(),
                details: "I2C buffer size must be at least 32 bytes".to_string(),
            });
        }
        
        self.base.config = config;
        
        if self.is_initialized {
            // Configure I2C transceiver registers
            self.write_register(0x02, (self.base.config.timeout_ms / 10) as u8)?; // Timeout config
            self.write_register(0x03, if self.base.config.enable_error_correction { 1 } else { 0 })?;
        }
        
        Ok(())
    }
    
    fn send_command(&mut self, command: &[u8]) -> Result<Vec<u8>, CommError> {
        if !self.is_initialized {
            return Err(CommError::ConnectionFailed {
                details: "I2C transceiver not initialized".to_string(),
            });
        }
        
        if command.is_empty() {
            return Err(CommError::InvalidData {
                details: "Empty command".to_string(),
            });
        }
        
        // Write command to command register (0x05)
        self.write_register(0x05, command[0])?;
        
        // Write command parameters to data registers (0x06-0x09)
        for (i, &byte) in command.iter().skip(1).take(4).enumerate() {
            self.write_register(0x06 + i as u8, byte)?;
        }
        
        // Trigger command execution (register 0x04)
        self.write_register(0x04, 0x01)?;
        
        // Wait for command completion
        let start_time = Instant::now();
        let timeout = Duration::from_millis(self.base.config.timeout_ms);
        
        while start_time.elapsed() < timeout {
            let status = self.read_register(0x04)?;
            if status & 0x01 == 0 {
                // Command completed, read response
                let response_length = self.read_register(0x0A)?;
                let mut response = Vec::new();
                
                for i in 0..response_length.min(8) {
                    let byte = self.read_register(0x0B + i)?;
                    response.push(byte);
                }
                
                return Ok(response);
            }
            
            std::thread::sleep(Duration::from_millis(10));
        }
        
        Err(CommError::Timeout {
            timeout_ms: self.base.config.timeout_ms,
        })
    }
    
    fn is_connected(&self) -> bool {
        self.is_initialized && self.base.status.is_connected
    }
    
    fn reset(&mut self) -> Result<(), CommError> {
        if self.is_initialized {
            // Send reset command to register 0xFF
            self.write_register(0xFF, 0xAA)?;
            std::thread::sleep(Duration::from_millis(100));
        }
        
        self.register_cache.clear();
        self.base.status.error_count = 0;
        self.base.update_status(true, Some(190));
        
        Ok(())
    }
    
    fn get_id(&self) -> u8 {
        self.base.id
    }
    
    fn flush_buffers(&mut self) -> Result<(), CommError> {
        if self.is_initialized {
            // Clear message buffer register
            self.write_register(0x00, 0x00)?;
        }
        
        self.base.message_buffer.clear();
        Ok(())
    }
    
    fn set_power_mode(&mut self, mode: PowerMode) -> Result<(), CommError> {
        self.base.config.power_mode = mode;
        
        if self.is_initialized {
            let power_value = match mode {
                PowerMode::Normal => 0x00,
                PowerMode::PowerSave => 0x01,
                PowerMode::Sleep => 0x02,
                PowerMode::Emergency => 0x03,
            };
            
            self.write_register(0x0F, power_value)?;
        }
        
        Ok(())
    }
}