//! Communication error types and handling

use std::fmt;

/// Communication error types for transceiver interfaces
#[derive(Debug, Clone, PartialEq)]
pub enum CommError {
    /// Connection to transceiver failed or lost
    ConnectionLost { transceiver_id: u8 },
    /// Timeout waiting for response
    Timeout { timeout_ms: u32 },
    /// Invalid or corrupted message received
    InvalidMessage { details: String },
    /// Checksum validation failed
    ChecksumError { expected: u16, received: u16 },
    /// Hardware-specific error
    HardwareError { code: u32, description: String },
    /// Configuration error
    ConfigurationError { parameter: String, value: String },
    /// Buffer overflow or underflow
    BufferError { operation: String },
    /// Protocol version mismatch
    ProtocolError { expected: u8, received: u8 },
}

impl fmt::Display for CommError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            CommError::ConnectionLost { transceiver_id } => {
                write!(f, "Connection lost to transceiver {}", transceiver_id)
            }
            CommError::Timeout { timeout_ms } => {
                write!(f, "Communication timeout after {}ms", timeout_ms)
            }
            CommError::InvalidMessage { details } => {
                write!(f, "Invalid message: {}", details)
            }
            CommError::ChecksumError { expected, received } => {
                write!(f, "Checksum error: expected 0x{:04X}, received 0x{:04X}", expected, received)
            }
            CommError::HardwareError { code, description } => {
                write!(f, "Hardware error {}: {}", code, description)
            }
            CommError::ConfigurationError { parameter, value } => {
                write!(f, "Configuration error: invalid {} = {}", parameter, value)
            }
            CommError::BufferError { operation } => {
                write!(f, "Buffer error during {}", operation)
            }
            CommError::ProtocolError { expected, received } => {
                write!(f, "Protocol version mismatch: expected {}, received {}", expected, received)
            }
        }
    }
}

impl std::error::Error for CommError {}

/// Result type for communication operations
pub type CommResult<T> = Result<T, CommError>;

/// Error recovery strategy for communication failures
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RecoveryStrategy {
    /// Retry the operation immediately
    Retry,
    /// Wait and then retry
    RetryWithDelay { delay_ms: u32 },
    /// Reset the connection and retry
    ResetAndRetry,
    /// Skip this operation and continue
    Skip,
    /// Fail permanently
    Fail,
}

impl CommError {
    /// Get the recommended recovery strategy for this error
    pub fn recovery_strategy(&self) -> RecoveryStrategy {
        match self {
            CommError::ConnectionLost { .. } => RecoveryStrategy::ResetAndRetry,
            CommError::Timeout { .. } => RecoveryStrategy::RetryWithDelay { delay_ms: 100 },
            CommError::InvalidMessage { .. } => RecoveryStrategy::Skip,
            CommError::ChecksumError { .. } => RecoveryStrategy::Retry,
            CommError::HardwareError { .. } => RecoveryStrategy::ResetAndRetry,
            CommError::ConfigurationError { .. } => RecoveryStrategy::Fail,
            CommError::BufferError { .. } => RecoveryStrategy::RetryWithDelay { delay_ms: 50 },
            CommError::ProtocolError { .. } => RecoveryStrategy::Fail,
        }
    }
    
    /// Check if this error is recoverable
    pub fn is_recoverable(&self) -> bool {
        !matches!(self.recovery_strategy(), RecoveryStrategy::Fail)
    }
}