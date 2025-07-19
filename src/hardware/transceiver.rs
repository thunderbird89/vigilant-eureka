//! Transceiver interface trait and configuration

use crate::hardware::{RawMessage, CommError, CommResult};
use serde::{Deserialize, Serialize};

/// Hardware abstraction trait for JANUS transceivers
pub trait TransceiverInterface {
    /// Read a message from the transceiver
    /// Returns Ok(Some(message)) if a message is available
    /// Returns Ok(None) if no message is available (non-blocking)
    /// Returns Err(error) if communication fails
    fn read_message(&mut self) -> CommResult<Option<RawMessage>>;
    
    /// Send a message to the transceiver (for configuration/control)
    fn send_message(&mut self, data: &[u8]) -> CommResult<()>;
    
    /// Get current transceiver status
    fn get_status(&self) -> TransceiverStatus;
    
    /// Configure the transceiver
    fn configure(&mut self, config: &TransceiverConfig) -> CommResult<()>;
    
    /// Reset the transceiver connection
    fn reset(&mut self) -> CommResult<()>;
    
    /// Check if the transceiver is connected and responsive
    fn is_connected(&self) -> bool;
    
    /// Get the transceiver ID for identification
    fn get_id(&self) -> u8;
    
    /// Flush any pending data
    fn flush(&mut self) -> CommResult<()>;
}

/// Transceiver status information
#[derive(Debug, Clone, PartialEq)]
pub struct TransceiverStatus {
    pub id: u8,
    pub connected: bool,
    pub signal_strength: Option<u8>,
    pub last_message_time: Option<u64>,
    pub error_count: u32,
    pub messages_received: u32,
    pub firmware_version: Option<String>,
}

impl TransceiverStatus {
    pub fn new(id: u8) -> Self {
        Self {
            id,
            connected: false,
            signal_strength: None,
            last_message_time: None,
            error_count: 0,
            messages_received: 0,
            firmware_version: None,
        }
    }
    
    pub fn is_healthy(&self) -> bool {
        self.connected && self.error_count < 10
    }
}

/// Transceiver configuration parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransceiverConfig {
    /// Unique transceiver identifier
    pub id: u8,
    /// Communication interface type
    pub interface: InterfaceType,
    /// Baud rate for serial communication
    pub baud_rate: Option<u32>,
    /// I2C address for I2C communication
    pub i2c_address: Option<u8>,
    /// Timeout for read operations (milliseconds)
    pub read_timeout_ms: u32,
    /// Timeout for write operations (milliseconds)
    pub write_timeout_ms: u32,
    /// Maximum message size in bytes
    pub max_message_size: usize,
    /// Enable message checksums
    pub enable_checksum: bool,
    /// Retry count for failed operations
    pub retry_count: u8,
}

impl Default for TransceiverConfig {
    fn default() -> Self {
        Self {
            id: 0,
            interface: InterfaceType::Serial,
            baud_rate: Some(115200),
            i2c_address: None,
            read_timeout_ms: 1000,
            write_timeout_ms: 500,
            max_message_size: 256,
            enable_checksum: true,
            retry_count: 3,
        }
    }
}

/// Communication interface types
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum InterfaceType {
    /// Serial/UART communication
    Serial,
    /// I2C communication
    I2C,
    /// Mock interface for testing
    Mock,
}

impl TransceiverConfig {
    pub fn serial(id: u8, baud_rate: u32) -> Self {
        Self {
            id,
            interface: InterfaceType::Serial,
            baud_rate: Some(baud_rate),
            i2c_address: None,
            ..Default::default()
        }
    }
    
    pub fn i2c(id: u8, address: u8) -> Self {
        Self {
            id,
            interface: InterfaceType::I2C,
            baud_rate: None,
            i2c_address: Some(address),
            ..Default::default()
        }
    }
    
    pub fn mock(id: u8) -> Self {
        Self {
            id,
            interface: InterfaceType::Mock,
            baud_rate: None,
            i2c_address: None,
            read_timeout_ms: 100,
            write_timeout_ms: 100,
            ..Default::default()
        }
    }
    
    pub fn validate(&self) -> CommResult<()> {
        match self.interface {
            InterfaceType::Serial => {
                if self.baud_rate.is_none() {
                    return Err(CommError::ConfigurationError {
                        parameter: "baud_rate".to_string(),
                        value: "None".to_string(),
                    });
                }
            }
            InterfaceType::I2C => {
                if self.i2c_address.is_none() {
                    return Err(CommError::ConfigurationError {
                        parameter: "i2c_address".to_string(),
                        value: "None".to_string(),
                    });
                }
            }
            InterfaceType::Mock => {
                // Mock interface doesn't require additional validation
            }
        }
        
        if self.max_message_size == 0 || self.max_message_size > 4096 {
            return Err(CommError::ConfigurationError {
                parameter: "max_message_size".to_string(),
                value: self.max_message_size.to_string(),
            });
        }
        
        Ok(())
    }
}