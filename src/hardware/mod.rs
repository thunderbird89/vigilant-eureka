//! Hardware abstraction layer for transceiver communication
//! 
//! This module provides hardware abstraction for JANUS transceivers,
//! supporting multiple communication interfaces and error recovery.

pub mod transceiver;
pub mod serial;
pub mod i2c;
pub mod mock;
pub mod error;

pub use transceiver::{TransceiverInterface, TransceiverStatus, TransceiverConfig};
pub use serial::SerialTransceiver;
pub use i2c::I2CTransceiver;
pub use mock::MockTransceiver;
pub use error::{CommError, CommResult};

/// Raw message received from transceiver hardware
#[derive(Debug, Clone)]
pub struct RawMessage {
    pub data: Vec<u8>,
    pub timestamp_ms: u64,
    pub signal_strength: Option<u8>,
    pub checksum: Option<u16>,
}

impl RawMessage {
    pub fn new(data: Vec<u8>) -> Self {
        Self {
            data,
            timestamp_ms: 0, // Will be set by transceiver implementation
            signal_strength: None,
            checksum: None,
        }
    }
    
    pub fn with_timestamp(mut self, timestamp_ms: u64) -> Self {
        self.timestamp_ms = timestamp_ms;
        self
    }
    
    pub fn with_signal_strength(mut self, strength: u8) -> Self {
        self.signal_strength = Some(strength);
        self
    }
    
    pub fn with_checksum(mut self, checksum: u16) -> Self {
        self.checksum = Some(checksum);
        self
    }
}