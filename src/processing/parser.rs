use serde::{Deserialize, Serialize};
use std::fmt;

/// Represents different message format versions supported by the system
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MessageVersion {
    V1 = 1,
    V2 = 2,
}

/// Geodetic position representation for anchor locations
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GeodeticPosition {
    pub latitude: f64,
    pub longitude: f64,
    pub depth: f64,
}

/// Structured anchor message after parsing from raw transceiver data
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct AnchorMessage {
    pub anchor_id: u16,
    pub timestamp_ms: u64,
    pub position: GeodeticPosition,
    pub signal_quality: u8,
    pub message_sequence: u16,
    pub version: MessageVersion,
}

/// Compact anchor message for memory-constrained embedded systems
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct CompactAnchorMessage {
    pub anchor_id: u16,
    pub timestamp_ms: u32,      // Relative timestamp
    pub lat_micro_deg: i32,     // Latitude * 1e6
    pub lon_micro_deg: i32,     // Longitude * 1e6
    pub depth_mm: u16,          // Depth in millimeters
    pub quality: u8,
    pub sequence: u16,
    pub version: u8,
}

/// Raw message data from transceiver before parsing
#[derive(Debug, Clone)]
pub struct RawMessage {
    pub data: Vec<u8>,
    pub timestamp_received: u64,
    pub transceiver_id: u8,
}

/// Errors that can occur during message parsing
#[derive(Debug, Clone, PartialEq)]
pub enum ParseError {
    InvalidFormat { details: String },
    UnsupportedVersion { version: u8 },
    InvalidChecksum { expected: u16, actual: u16 },
    InsufficientData { required: usize, available: usize },
    InvalidTimestamp { timestamp: u64 },
    InvalidPosition { field: String, value: f64 },
    InvalidAnchorId { id: u16 },
    CorruptedData { details: String },
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ParseError::InvalidFormat { details } => write!(f, "Invalid message format: {}", details),
            ParseError::UnsupportedVersion { version } => write!(f, "Unsupported message version: {}", version),
            ParseError::InvalidChecksum { expected, actual } => write!(f, "Checksum mismatch: expected {}, got {}", expected, actual),
            ParseError::InsufficientData { required, available } => write!(f, "Insufficient data: need {} bytes, got {}", required, available),
            ParseError::InvalidTimestamp { timestamp } => write!(f, "Invalid timestamp: {}", timestamp),
            ParseError::InvalidPosition { field, value } => write!(f, "Invalid position {}: {}", field, value),
            ParseError::InvalidAnchorId { id } => write!(f, "Invalid anchor ID: {}", id),
            ParseError::CorruptedData { details } => write!(f, "Corrupted data: {}", details),
        }
    }
}

impl std::error::Error for ParseError {}

/// Message parser for converting raw transceiver data into structured anchor messages
pub struct MessageParser {
    supported_versions: Vec<MessageVersion>,
    strict_validation: bool,
}

impl MessageParser {
    /// Create a new message parser with default settings
    pub fn new() -> Self {
        Self {
            supported_versions: vec![MessageVersion::V1, MessageVersion::V2],
            strict_validation: true,
        }
    }

    /// Create a parser with specific version support
    pub fn with_versions(versions: Vec<MessageVersion>) -> Self {
        Self {
            supported_versions: versions,
            strict_validation: true,
        }
    }

    /// Enable or disable strict validation
    pub fn set_strict_validation(&mut self, strict: bool) {
        self.strict_validation = strict;
    }

    /// Parse a raw message into a structured AnchorMessage
    pub fn parse_message(&self, raw: &RawMessage) -> Result<AnchorMessage, ParseError> {
        if raw.data.len() < 4 {
            return Err(ParseError::InsufficientData {
                required: 4,
                available: raw.data.len(),
            });
        }

        // Extract version from first byte
        let version_byte = raw.data[0];
        let version = match version_byte {
            1 => MessageVersion::V1,
            2 => MessageVersion::V2,
            _ => return Err(ParseError::UnsupportedVersion { version: version_byte }),
        };

        if !self.supported_versions.contains(&version) {
            return Err(ParseError::UnsupportedVersion { version: version_byte });
        }

        match version {
            MessageVersion::V1 => self.parse_v1_message(raw),
            MessageVersion::V2 => self.parse_v2_message(raw),
        }
    }

    /// Parse version 1 message format
    fn parse_v1_message(&self, raw: &RawMessage) -> Result<AnchorMessage, ParseError> {
        // V1 format: [version(1)] [anchor_id(2)] [timestamp(8)] [lat(8)] [lon(8)] [depth(8)] [quality(1)] [seq(2)] [checksum(2)]
        const V1_MESSAGE_SIZE: usize = 40;
        
        if raw.data.len() < V1_MESSAGE_SIZE {
            return Err(ParseError::InsufficientData {
                required: V1_MESSAGE_SIZE,
                available: raw.data.len(),
            });
        }

        let mut offset = 1; // Skip version byte

        // Parse anchor ID
        let anchor_id = u16::from_le_bytes([raw.data[offset], raw.data[offset + 1]]);
        offset += 2;

        // Parse timestamp
        let timestamp_ms = u64::from_le_bytes([
            raw.data[offset], raw.data[offset + 1], raw.data[offset + 2], raw.data[offset + 3],
            raw.data[offset + 4], raw.data[offset + 5], raw.data[offset + 6], raw.data[offset + 7],
        ]);
        offset += 8;

        // Parse position
        let latitude = f64::from_le_bytes([
            raw.data[offset], raw.data[offset + 1], raw.data[offset + 2], raw.data[offset + 3],
            raw.data[offset + 4], raw.data[offset + 5], raw.data[offset + 6], raw.data[offset + 7],
        ]);
        offset += 8;

        let longitude = f64::from_le_bytes([
            raw.data[offset], raw.data[offset + 1], raw.data[offset + 2], raw.data[offset + 3],
            raw.data[offset + 4], raw.data[offset + 5], raw.data[offset + 6], raw.data[offset + 7],
        ]);
        offset += 8;

        let depth = f64::from_le_bytes([
            raw.data[offset], raw.data[offset + 1], raw.data[offset + 2], raw.data[offset + 3],
            raw.data[offset + 4], raw.data[offset + 5], raw.data[offset + 6], raw.data[offset + 7],
        ]);
        offset += 8;

        // Parse quality and sequence
        let signal_quality = raw.data[offset];
        offset += 1;

        let message_sequence = u16::from_le_bytes([raw.data[offset], raw.data[offset + 1]]);
        offset += 2;

        // Parse and verify checksum
        let expected_checksum = u16::from_le_bytes([raw.data[offset], raw.data[offset + 1]]);
        let calculated_checksum = self.calculate_checksum(&raw.data[0..offset]);

        if self.strict_validation && expected_checksum != calculated_checksum {
            return Err(ParseError::InvalidChecksum {
                expected: expected_checksum,
                actual: calculated_checksum,
            });
        }

        // Validate parsed data
        self.validate_anchor_message_data(anchor_id, timestamp_ms, latitude, longitude, depth)?;

        Ok(AnchorMessage {
            anchor_id,
            timestamp_ms,
            position: GeodeticPosition {
                latitude,
                longitude,
                depth,
            },
            signal_quality,
            message_sequence,
            version: MessageVersion::V1,
        })
    }

    /// Parse version 2 message format (more compact)
    fn parse_v2_message(&self, raw: &RawMessage) -> Result<AnchorMessage, ParseError> {
        // V2 format: [version(1)] [anchor_id(2)] [timestamp_rel(4)] [lat_micro(4)] [lon_micro(4)] [depth_mm(2)] [quality(1)] [seq(2)] [checksum(2)]
        const V2_MESSAGE_SIZE: usize = 22;
        
        if raw.data.len() < V2_MESSAGE_SIZE {
            return Err(ParseError::InsufficientData {
                required: V2_MESSAGE_SIZE,
                available: raw.data.len(),
            });
        }

        let mut offset = 1; // Skip version byte

        // Parse anchor ID
        let anchor_id = u16::from_le_bytes([raw.data[offset], raw.data[offset + 1]]);
        offset += 2;

        // Parse relative timestamp (convert to absolute)
        let timestamp_rel = u32::from_le_bytes([
            raw.data[offset], raw.data[offset + 1], raw.data[offset + 2], raw.data[offset + 3],
        ]);
        let timestamp_ms = raw.timestamp_received - (timestamp_rel as u64);
        offset += 4;

        // Parse position (micro-degrees and millimeters)
        let lat_micro = i32::from_le_bytes([
            raw.data[offset], raw.data[offset + 1], raw.data[offset + 2], raw.data[offset + 3],
        ]);
        let latitude = lat_micro as f64 / 1_000_000.0;
        offset += 4;

        let lon_micro = i32::from_le_bytes([
            raw.data[offset], raw.data[offset + 1], raw.data[offset + 2], raw.data[offset + 3],
        ]);
        let longitude = lon_micro as f64 / 1_000_000.0;
        offset += 4;

        let depth_mm = u16::from_le_bytes([raw.data[offset], raw.data[offset + 1]]);
        let depth = depth_mm as f64 / 1000.0;
        offset += 2;

        // Parse quality and sequence
        let signal_quality = raw.data[offset];
        offset += 1;

        let message_sequence = u16::from_le_bytes([raw.data[offset], raw.data[offset + 1]]);
        offset += 2;

        // Parse and verify checksum
        let expected_checksum = u16::from_le_bytes([raw.data[offset], raw.data[offset + 1]]);
        let calculated_checksum = self.calculate_checksum(&raw.data[0..offset]);

        if self.strict_validation && expected_checksum != calculated_checksum {
            return Err(ParseError::InvalidChecksum {
                expected: expected_checksum,
                actual: calculated_checksum,
            });
        }

        // Validate parsed data
        self.validate_anchor_message_data(anchor_id, timestamp_ms, latitude, longitude, depth)?;

        Ok(AnchorMessage {
            anchor_id,
            timestamp_ms,
            position: GeodeticPosition {
                latitude,
                longitude,
                depth,
            },
            signal_quality,
            message_sequence,
            version: MessageVersion::V2,
        })
    }

    /// Calculate CRC16 checksum for message integrity
    pub fn calculate_checksum(&self, data: &[u8]) -> u16 {
        let mut crc: u16 = 0xFFFF;
        
        for &byte in data {
            crc ^= byte as u16;
            for _ in 0..8 {
                if crc & 1 != 0 {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        
        crc
    }

    /// Validate anchor message data for consistency and reasonable values
    fn validate_anchor_message_data(
        &self,
        anchor_id: u16,
        timestamp_ms: u64,
        latitude: f64,
        longitude: f64,
        depth: f64,
    ) -> Result<(), ParseError> {
        // Validate anchor ID
        if anchor_id == 0 {
            return Err(ParseError::InvalidAnchorId { id: anchor_id });
        }

        // Validate timestamp (should be reasonable - not too far in past or future)
        let current_time = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis() as u64;
        
        if self.strict_validation {
            // Allow 1 hour in past or future
            let time_tolerance = 3600 * 1000; // 1 hour in milliseconds
            if timestamp_ms < current_time.saturating_sub(time_tolerance) || 
               timestamp_ms > current_time + time_tolerance {
                return Err(ParseError::InvalidTimestamp { timestamp: timestamp_ms });
            }
        }

        // Validate latitude (-90 to 90 degrees)
        if latitude < -90.0 || latitude > 90.0 {
            return Err(ParseError::InvalidPosition {
                field: "latitude".to_string(),
                value: latitude,
            });
        }

        // Validate longitude (-180 to 180 degrees)
        if longitude < -180.0 || longitude > 180.0 {
            return Err(ParseError::InvalidPosition {
                field: "longitude".to_string(),
                value: longitude,
            });
        }

        // Validate depth (should be positive for underwater applications)
        if depth < 0.0 || depth > 11000.0 { // Mariana Trench is ~11km deep
            return Err(ParseError::InvalidPosition {
                field: "depth".to_string(),
                value: depth,
            });
        }

        Ok(())
    }

    /// Convert AnchorMessage to compact format for embedded systems
    pub fn to_compact_format(&self, message: &AnchorMessage) -> CompactAnchorMessage {
        CompactAnchorMessage {
            anchor_id: message.anchor_id,
            timestamp_ms: (message.timestamp_ms & 0xFFFFFFFF) as u32, // Truncate to 32-bit
            lat_micro_deg: (message.position.latitude * 1_000_000.0) as i32,
            lon_micro_deg: (message.position.longitude * 1_000_000.0) as i32,
            depth_mm: (message.position.depth * 1000.0).min(65535.0) as u16,
            quality: message.signal_quality,
            sequence: message.message_sequence,
            version: message.version as u8,
        }
    }

    /// Convert compact format back to full AnchorMessage
    pub fn from_compact_format(&self, compact: &CompactAnchorMessage, base_timestamp: u64) -> AnchorMessage {
        AnchorMessage {
            anchor_id: compact.anchor_id,
            timestamp_ms: base_timestamp + compact.timestamp_ms as u64,
            position: GeodeticPosition {
                latitude: compact.lat_micro_deg as f64 / 1_000_000.0,
                longitude: compact.lon_micro_deg as f64 / 1_000_000.0,
                depth: compact.depth_mm as f64 / 1000.0,
            },
            signal_quality: compact.quality,
            message_sequence: compact.sequence,
            version: match compact.version {
                1 => MessageVersion::V1,
                2 => MessageVersion::V2,
                _ => MessageVersion::V1, // Default fallback
            },
        }
    }
}

impl Default for MessageParser {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_v1_message() {
        let mut parser = MessageParser::new();
        parser.set_strict_validation(false); // Disable strict validation for tests
        
        // Create a valid V1 message
        let mut data = Vec::new();
        data.push(1u8); // Version
        data.extend_from_slice(&123u16.to_le_bytes()); // Anchor ID
        data.extend_from_slice(&1234567890123u64.to_le_bytes()); // Timestamp
        data.extend_from_slice(&32.123456f64.to_le_bytes()); // Latitude
        data.extend_from_slice(&(-117.654321f64).to_le_bytes()); // Longitude
        data.extend_from_slice(&10.5f64.to_le_bytes()); // Depth
        data.push(85u8); // Quality
        data.extend_from_slice(&456u16.to_le_bytes()); // Sequence
        
        // Calculate and append checksum
        let checksum = parser.calculate_checksum(&data);
        data.extend_from_slice(&checksum.to_le_bytes());
        
        let raw = RawMessage {
            data,
            timestamp_received: 1234567890123,
            transceiver_id: 1,
        };
        
        let result = parser.parse_message(&raw);
        assert!(result.is_ok());
        
        let message = result.unwrap();
        assert_eq!(message.anchor_id, 123);
        assert_eq!(message.timestamp_ms, 1234567890123);
        assert_eq!(message.position.latitude, 32.123456);
        assert_eq!(message.position.longitude, -117.654321);
        assert_eq!(message.position.depth, 10.5);
        assert_eq!(message.signal_quality, 85);
        assert_eq!(message.message_sequence, 456);
        assert_eq!(message.version, MessageVersion::V1);
    }

    #[test]
    fn test_parse_v2_message() {
        let mut parser = MessageParser::new();
        parser.set_strict_validation(false); // Disable strict validation for tests
        
        // Create a valid V2 message
        let mut data = Vec::new();
        data.push(2u8); // Version
        data.extend_from_slice(&123u16.to_le_bytes()); // Anchor ID
        data.extend_from_slice(&1000u32.to_le_bytes()); // Relative timestamp
        data.extend_from_slice(&32123456i32.to_le_bytes()); // Latitude micro-degrees
        data.extend_from_slice(&(-117654321i32).to_le_bytes()); // Longitude micro-degrees
        data.extend_from_slice(&10500u16.to_le_bytes()); // Depth mm
        data.push(85u8); // Quality
        data.extend_from_slice(&456u16.to_le_bytes()); // Sequence
        
        // Calculate and append checksum
        let checksum = parser.calculate_checksum(&data);
        data.extend_from_slice(&checksum.to_le_bytes());
        
        let raw = RawMessage {
            data,
            timestamp_received: 1234567890123,
            transceiver_id: 1,
        };
        
        let result = parser.parse_message(&raw);
        assert!(result.is_ok());
        
        let message = result.unwrap();
        assert_eq!(message.anchor_id, 123);
        assert_eq!(message.timestamp_ms, 1234567890123 - 1000);
        assert!((message.position.latitude - 32.123456).abs() < 0.000001);
        assert!((message.position.longitude - (-117.654321)).abs() < 0.000001);
        assert_eq!(message.position.depth, 10.5);
        assert_eq!(message.signal_quality, 85);
        assert_eq!(message.message_sequence, 456);
        assert_eq!(message.version, MessageVersion::V2);
    }

    #[test]
    fn test_invalid_checksum() {
        let mut parser = MessageParser::new();
        parser.set_strict_validation(true); // Keep strict validation for checksum test
        
        // Create a message with invalid checksum
        let mut data = Vec::new();
        data.push(1u8); // Version
        data.extend_from_slice(&123u16.to_le_bytes()); // Anchor ID
        data.extend_from_slice(&1234567890123u64.to_le_bytes()); // Timestamp
        data.extend_from_slice(&32.123456f64.to_le_bytes()); // Latitude
        data.extend_from_slice(&(-117.654321f64).to_le_bytes()); // Longitude
        data.extend_from_slice(&10.5f64.to_le_bytes()); // Depth
        data.push(85u8); // Quality
        data.extend_from_slice(&456u16.to_le_bytes()); // Sequence
        data.extend_from_slice(&0xDEADu16.to_le_bytes()); // Invalid checksum
        
        let raw = RawMessage {
            data,
            timestamp_received: 1234567890123,
            transceiver_id: 1,
        };
        
        let result = parser.parse_message(&raw);
        assert!(result.is_err());
        
        if let Err(ParseError::InvalidChecksum { expected, actual }) = result {
            assert_eq!(expected, 0xDEAD);
            assert_ne!(actual, 0xDEAD);
        } else {
            panic!("Expected InvalidChecksum error");
        }
    }

    #[test]
    fn test_compact_format_conversion() {
        let parser = MessageParser::new();
        
        let original = AnchorMessage {
            anchor_id: 123,
            timestamp_ms: 1234567890123,
            position: GeodeticPosition {
                latitude: 32.123456,
                longitude: -117.654321,
                depth: 10.5,
            },
            signal_quality: 85,
            message_sequence: 456,
            version: MessageVersion::V2,
        };
        
        let compact = parser.to_compact_format(&original);
        let restored = parser.from_compact_format(&compact, 1234567890000);
        
        assert_eq!(restored.anchor_id, original.anchor_id);
        assert!((restored.position.latitude - original.position.latitude).abs() < 0.000001);
        assert!((restored.position.longitude - original.position.longitude).abs() < 0.000001);
        assert!((restored.position.depth - original.position.depth).abs() < 0.001);
        assert_eq!(restored.signal_quality, original.signal_quality);
        assert_eq!(restored.message_sequence, original.message_sequence);
    }
}