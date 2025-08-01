// Message parsing and validation system - no external serialization needed for core functionality
use std::collections::HashMap;
use std::time::{SystemTime, UNIX_EPOCH};
use uuid::Uuid;

/// Raw message data from transceiver interface
#[derive(Debug, Clone)]
pub struct RawMessage {
    pub data: Vec<u8>,
    pub timestamp_received: u64,
    pub transceiver_id: u8,
    pub signal_strength: Option<u8>,
}

/// Parsed anchor message with validation
#[derive(Debug, Clone, PartialEq)]
pub struct AnchorMessage {
    pub anchor_id: u16,
    pub timestamp_ms: u64,
    pub position: GeodeticPosition,
    pub signal_quality: u8,
    pub message_sequence: u16,
    pub message_version: u8,
    pub checksum: u16,
}

/// Geodetic position with validation
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize, Default)]
pub struct GeodeticPosition {
    pub latitude: f64,
    pub longitude: f64,
    pub depth: f64,
}

/// Message format versions supported
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MessageVersion {
    V1 = 1,
    V2 = 2,
    V3 = 3, // New version with UUID support for beacons
}

/// Message parsing errors with detailed context
#[derive(Debug, Clone, PartialEq)]
pub enum MessageParseError {
    InvalidLength { expected: usize, actual: usize },
    InvalidVersion { version: u8 },
    ChecksumMismatch { expected: u16, calculated: u16 },
    InvalidAnchorId { id: u16 },
    InvalidTimestamp { timestamp: u64 },
    InvalidPosition { field: String, value: f64 },
    InvalidSignalQuality { quality: u8 },
    CorruptedData { details: String },
    UnsupportedFormat { format_id: u8 },
}

impl std::fmt::Display for MessageParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MessageParseError::InvalidLength { expected, actual } => {
                write!(f, "Invalid message length: expected {}, got {}", expected, actual)
            }
            MessageParseError::InvalidVersion { version } => {
                write!(f, "Unsupported message version: {}", version)
            }
            MessageParseError::ChecksumMismatch { expected, calculated } => {
                write!(f, "Checksum mismatch: expected 0x{:04X}, calculated 0x{:04X}", expected, calculated)
            }
            MessageParseError::InvalidAnchorId { id } => {
                write!(f, "Invalid anchor ID: {}", id)
            }
            MessageParseError::InvalidTimestamp { timestamp } => {
                write!(f, "Invalid timestamp: {}", timestamp)
            }
            MessageParseError::InvalidPosition { field, value } => {
                write!(f, "Invalid position {}: {}", field, value)
            }
            MessageParseError::InvalidSignalQuality { quality } => {
                write!(f, "Invalid signal quality: {}", quality)
            }
            MessageParseError::CorruptedData { details } => {
                write!(f, "Corrupted message data: {}", details)
            }
            MessageParseError::UnsupportedFormat { format_id } => {
                write!(f, "Unsupported message format: {}", format_id)
            }
        }
    }
}

impl std::error::Error for MessageParseError {}

/// Position validation bounds
#[derive(Debug, Clone)]
pub struct PositionBounds {
    pub min_latitude: f64,
    pub max_latitude: f64,
    pub min_longitude: f64,
    pub max_longitude: f64,
    pub max_depth: f64,
}

impl Default for PositionBounds {
    fn default() -> Self {
        Self {
            min_latitude: -90.0,
            max_latitude: 90.0,
            min_longitude: -180.0,
            max_longitude: 180.0,
            max_depth: 11000.0, // Mariana Trench depth
        }
    }
}

/// Message parser statistics for diagnostics
#[derive(Debug, Clone, Default)]
pub struct MessageParserStats {
    pub total_messages_processed: u64,
    pub successful_parses: u64,
    pub parse_errors: u64,
    pub checksum_failures: u64,
    pub validation_failures: u64,
    pub version_distribution: HashMap<u8, u64>,
    pub error_distribution: HashMap<String, u64>,
}

/// Message parser with support for multiple formats and versions
pub struct MessageParser {
    /// Supported message versions
    supported_versions: Vec<MessageVersion>,
    /// Maximum allowed timestamp age (milliseconds)
    max_timestamp_age_ms: u64,
    /// Valid anchor ID ranges
    valid_anchor_id_ranges: Vec<(u16, u16)>,
    /// Position validation bounds
    position_bounds: PositionBounds,
    /// Message statistics for diagnostics
    stats: MessageParserStats,
}

impl MessageParser {
    /// Create a new message parser with default configuration
    pub fn new() -> Self {
        Self {
            supported_versions: vec![MessageVersion::V1, MessageVersion::V2, MessageVersion::V3],
            max_timestamp_age_ms: 60000, // 60 seconds
            valid_anchor_id_ranges: vec![(1, 65535)], // All valid u16 except 0
            position_bounds: PositionBounds::default(),
            stats: MessageParserStats::default(),
        }
    }

    /// Create parser with custom configuration
    pub fn with_config(
        supported_versions: Vec<MessageVersion>,
        max_timestamp_age_ms: u64,
        valid_anchor_id_ranges: Vec<(u16, u16)>,
        position_bounds: PositionBounds,
    ) -> Self {
        Self {
            supported_versions,
            max_timestamp_age_ms,
            valid_anchor_id_ranges,
            position_bounds,
            stats: MessageParserStats::default(),
        }
    }

    /// Parse raw message data into structured AnchorMessage
    pub fn parse_message(&mut self, raw_message: &RawMessage) -> Result<AnchorMessage, MessageParseError> {
        self.stats.total_messages_processed += 1;

        // Basic length validation
        if raw_message.data.len() < 4 {
            self.record_error("InvalidLength");
            return Err(MessageParseError::InvalidLength {
                expected: 4,
                actual: raw_message.data.len(),
            });
        }

        // Extract version from first byte
        let version = raw_message.data[0];
        self.stats.version_distribution
            .entry(version)
            .and_modify(|e| *e += 1)
            .or_insert(1);

        // Validate version
        let message_version = match version {
            1 => MessageVersion::V1,
            2 => MessageVersion::V2,
            3 => MessageVersion::V3,
            _ => {
                self.record_error("InvalidVersion");
                return Err(MessageParseError::InvalidVersion { version });
            }
        };

        if !self.supported_versions.contains(&message_version) {
            self.record_error("UnsupportedVersion");
            return Err(MessageParseError::InvalidVersion { version });
        }

        // Parse based on version
        let anchor_message = match message_version {
            MessageVersion::V1 => self.parse_v1_message(&raw_message.data)?,
            MessageVersion::V2 => self.parse_v2_message(&raw_message.data)?,
            MessageVersion::V3 => self.parse_v3_message(&raw_message.data)?,
        };

        // Validate parsed message
        self.validate_message(&anchor_message)?;

        self.stats.successful_parses += 1;
        Ok(anchor_message)
    }

    /// Parse Version 1 message format
    /// Format: [version:1][anchor_id:2][timestamp:8][lat:8][lon:8][depth:4][quality:1][seq:2][checksum:2]
    fn parse_v1_message(&mut self, data: &[u8]) -> Result<AnchorMessage, MessageParseError> {
        const V1_MESSAGE_LENGTH: usize = 36;
        
        if data.len() != V1_MESSAGE_LENGTH {
            self.record_error("InvalidLength");
            return Err(MessageParseError::InvalidLength {
                expected: V1_MESSAGE_LENGTH,
                actual: data.len(),
            });
        }

        let mut offset = 1; // Skip version byte

        // Parse anchor ID (2 bytes, little-endian)
        let anchor_id = u16::from_le_bytes([data[offset], data[offset + 1]]);
        offset += 2;

        // Parse timestamp (8 bytes, little-endian)
        let timestamp_ms = u64::from_le_bytes([
            data[offset], data[offset + 1], data[offset + 2], data[offset + 3],
            data[offset + 4], data[offset + 5], data[offset + 6], data[offset + 7],
        ]);
        offset += 8;

        // Parse latitude (8 bytes, little-endian, f64)
        let latitude = f64::from_le_bytes([
            data[offset], data[offset + 1], data[offset + 2], data[offset + 3],
            data[offset + 4], data[offset + 5], data[offset + 6], data[offset + 7],
        ]);
        offset += 8;

        // Parse longitude (8 bytes, little-endian, f64)
        let longitude = f64::from_le_bytes([
            data[offset], data[offset + 1], data[offset + 2], data[offset + 3],
            data[offset + 4], data[offset + 5], data[offset + 6], data[offset + 7],
        ]);
        offset += 8;

        // Parse depth (4 bytes, little-endian, f32 -> f64)
        let depth = f32::from_le_bytes([
            data[offset], data[offset + 1], data[offset + 2], data[offset + 3],
        ]) as f64;
        offset += 4;

        // Parse signal quality (1 byte)
        let signal_quality = data[offset];
        offset += 1;

        // Parse message sequence (2 bytes, little-endian)
        let message_sequence = u16::from_le_bytes([data[offset], data[offset + 1]]);
        offset += 2;

        // Parse checksum (2 bytes, little-endian)
        let checksum = u16::from_le_bytes([data[offset], data[offset + 1]]);

        // Verify checksum (exclude checksum bytes from calculation)
        let calculated_checksum = self.calculate_checksum(&data[..data.len() - 2]);
        if checksum != calculated_checksum {
            self.stats.checksum_failures += 1;
            self.record_error("ChecksumMismatch");
            return Err(MessageParseError::ChecksumMismatch {
                expected: checksum,
                calculated: calculated_checksum,
            });
        }

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
            message_version: 1,
            checksum,
        })
    }

    /// Parse Version 2 message format (more compact)
    /// Format: [version:1][anchor_id:2][timestamp:4][lat:4][lon:4][depth:2][quality:1][seq:2][flags:1][checksum:2]
    fn parse_v2_message(&mut self, data: &[u8]) -> Result<AnchorMessage, MessageParseError> {
        const V2_MESSAGE_LENGTH: usize = 23;
        
        if data.len() != V2_MESSAGE_LENGTH {
            self.record_error("InvalidLength");
            return Err(MessageParseError::InvalidLength {
                expected: V2_MESSAGE_LENGTH,
                actual: data.len(),
            });
        }

        let mut offset = 1; // Skip version byte

        // Parse anchor ID (2 bytes, little-endian)
        let anchor_id = u16::from_le_bytes([data[offset], data[offset + 1]]);
        offset += 2;

        // Parse relative timestamp (4 bytes, little-endian) - milliseconds since epoch start
        let relative_timestamp = u32::from_le_bytes([
            data[offset], data[offset + 1], data[offset + 2], data[offset + 3],
        ]);
        // Convert to absolute timestamp (base epoch of 2024-01-01)
        let base_epoch = 1704067200000u64; // 2024-01-01
        let timestamp_ms = base_epoch + (relative_timestamp as u64 * 1000); // Convert seconds to milliseconds
        offset += 4;

        // Parse latitude (4 bytes, little-endian, scaled by 1e6)
        let lat_scaled = i32::from_le_bytes([
            data[offset], data[offset + 1], data[offset + 2], data[offset + 3],
        ]);
        let latitude = lat_scaled as f64 / 1_000_000.0;
        offset += 4;

        // Parse longitude (4 bytes, little-endian, scaled by 1e6)
        let lon_scaled = i32::from_le_bytes([
            data[offset], data[offset + 1], data[offset + 2], data[offset + 3],
        ]);
        let longitude = lon_scaled as f64 / 1_000_000.0;
        offset += 4;

        // Parse depth (2 bytes, little-endian, in millimeters)
        let depth_mm = u16::from_le_bytes([data[offset], data[offset + 1]]);
        let depth = depth_mm as f64 / 1000.0;
        offset += 2;

        // Parse signal quality (1 byte)
        let signal_quality = data[offset];
        offset += 1;

        // Parse message sequence (2 bytes, little-endian)
        let message_sequence = u16::from_le_bytes([data[offset], data[offset + 1]]);
        offset += 2;

        // Parse flags (1 byte) - reserved for future use
        let _flags = data[offset];
        offset += 1;

        // Parse checksum (2 bytes, little-endian)
        let checksum = u16::from_le_bytes([data[offset], data[offset + 1]]);

        // Verify checksum
        let calculated_checksum = self.calculate_checksum(&data[..data.len() - 2]);
        if checksum != calculated_checksum {
            self.stats.checksum_failures += 1;
            self.record_error("ChecksumMismatch");
            return Err(MessageParseError::ChecksumMismatch {
                expected: checksum,
                calculated: calculated_checksum,
            });
        }

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
            message_version: 2,
            checksum,
        })
    }

    /// Parse Version 3 message format (with UUID support for beacons)
    /// Format: [version:1][beacon_uuid:16][timestamp:8][lat:8][lon:8][depth:4][quality:1][seq:2][checksum:2]
    fn parse_v3_message(&mut self, data: &[u8]) -> Result<AnchorMessage, MessageParseError> {
        const V3_MESSAGE_LENGTH: usize = 50;
        
        if data.len() != V3_MESSAGE_LENGTH {
            self.record_error("InvalidLength");
            return Err(MessageParseError::InvalidLength {
                expected: V3_MESSAGE_LENGTH,
                actual: data.len(),
            });
        }

        let mut offset = 1; // Skip version byte

        // Parse beacon UUID (16 bytes)
        let mut uuid_bytes = [0u8; 16];
        uuid_bytes.copy_from_slice(&data[offset..offset + 16]);
        let _beacon_uuid = Uuid::from_bytes(uuid_bytes);
        offset += 16;

        // For compatibility, use a hash of the UUID as anchor_id
        let anchor_id = (uuid_bytes[0] as u16) << 8 | (uuid_bytes[1] as u16);

        // Parse timestamp (8 bytes, little-endian)
        let timestamp_ms = u64::from_le_bytes([
            data[offset], data[offset + 1], data[offset + 2], data[offset + 3],
            data[offset + 4], data[offset + 5], data[offset + 6], data[offset + 7],
        ]);
        offset += 8;

        // Parse latitude (8 bytes, little-endian, f64)
        let latitude = f64::from_le_bytes([
            data[offset], data[offset + 1], data[offset + 2], data[offset + 3],
            data[offset + 4], data[offset + 5], data[offset + 6], data[offset + 7],
        ]);
        offset += 8;

        // Parse longitude (8 bytes, little-endian, f64)
        let longitude = f64::from_le_bytes([
            data[offset], data[offset + 1], data[offset + 2], data[offset + 3],
            data[offset + 4], data[offset + 5], data[offset + 6], data[offset + 7],
        ]);
        offset += 8;

        // Parse depth (4 bytes, little-endian, f32 -> f64)
        let depth = f32::from_le_bytes([
            data[offset], data[offset + 1], data[offset + 2], data[offset + 3],
        ]) as f64;
        offset += 4;

        // Parse signal quality (1 byte)
        let signal_quality = data[offset];
        offset += 1;

        // Parse message sequence (2 bytes, little-endian)
        let message_sequence = u16::from_le_bytes([data[offset], data[offset + 1]]);
        offset += 2;

        // Parse checksum (2 bytes, little-endian)
        let checksum = u16::from_le_bytes([data[offset], data[offset + 1]]);

        // Verify checksum (exclude checksum bytes from calculation)
        let calculated_checksum = self.calculate_checksum(&data[..data.len() - 2]);
        if checksum != calculated_checksum {
            self.stats.checksum_failures += 1;
            self.record_error("ChecksumMismatch");
            return Err(MessageParseError::ChecksumMismatch {
                expected: checksum,
                calculated: calculated_checksum,
            });
        }

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
            message_version: 3,
            checksum,
        })
    }

    /// Validate parsed anchor message
    fn validate_message(&mut self, message: &AnchorMessage) -> Result<(), MessageParseError> {
        // Validate anchor ID
        if !self.is_valid_anchor_id(message.anchor_id) {
            self.stats.validation_failures += 1;
            self.record_error("InvalidAnchorId");
            return Err(MessageParseError::InvalidAnchorId {
                id: message.anchor_id,
            });
        }

        // Validate timestamp (not too old, not in future)
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;
        
        if message.timestamp_ms > current_time + 1000 {
            // Allow 1 second in future for clock skew
            self.stats.validation_failures += 1;
            self.record_error("InvalidTimestamp");
            return Err(MessageParseError::InvalidTimestamp {
                timestamp: message.timestamp_ms,
            });
        }

        if current_time.saturating_sub(message.timestamp_ms) > self.max_timestamp_age_ms {
            self.stats.validation_failures += 1;
            self.record_error("InvalidTimestamp");
            return Err(MessageParseError::InvalidTimestamp {
                timestamp: message.timestamp_ms,
            });
        }

        // Validate position
        self.validate_position(&message.position)?;

        // Validate signal quality (0-255 range is inherently valid for u8)
        // But we can check for reasonable values
        if message.signal_quality == 0 {
            // Zero signal quality might indicate a problem
            eprintln!("WARNING: Zero signal quality for anchor {}", message.anchor_id);
        }

        Ok(())
    }

    /// Validate geodetic position
    fn validate_position(&mut self, position: &GeodeticPosition) -> Result<(), MessageParseError> {
        // Validate latitude
        if position.latitude < self.position_bounds.min_latitude 
            || position.latitude > self.position_bounds.max_latitude 
            || !position.latitude.is_finite() {
            self.stats.validation_failures += 1;
            self.record_error("InvalidPosition");
            return Err(MessageParseError::InvalidPosition {
                field: "latitude".to_string(),
                value: position.latitude,
            });
        }

        // Validate longitude
        if position.longitude < self.position_bounds.min_longitude 
            || position.longitude > self.position_bounds.max_longitude 
            || !position.longitude.is_finite() {
            self.stats.validation_failures += 1;
            self.record_error("InvalidPosition");
            return Err(MessageParseError::InvalidPosition {
                field: "longitude".to_string(),
                value: position.longitude,
            });
        }

        // Validate depth (must be non-negative and reasonable)
        if position.depth < 0.0 
            || position.depth > self.position_bounds.max_depth 
            || !position.depth.is_finite() {
            self.stats.validation_failures += 1;
            self.record_error("InvalidPosition");
            return Err(MessageParseError::InvalidPosition {
                field: "depth".to_string(),
                value: position.depth,
            });
        }

        Ok(())
    }

    /// Check if anchor ID is in valid ranges
    fn is_valid_anchor_id(&self, anchor_id: u16) -> bool {
        if anchor_id == 0 {
            return false; // Reserve 0 as invalid
        }

        self.valid_anchor_id_ranges
            .iter()
            .any(|(min, max)| anchor_id >= *min && anchor_id <= *max)
    }

    /// Calculate CRC16 checksum for message integrity
    pub fn calculate_checksum(&self, data: &[u8]) -> u16 {
        let mut crc: u16 = 0xFFFF;
        
        for &byte in data {
            crc ^= byte as u16;
            for _ in 0..8 {
                if crc & 1 != 0 {
                    crc = (crc >> 1) ^ 0xA001; // CRC16-ANSI polynomial
                } else {
                    crc >>= 1;
                }
            }
        }
        
        crc
    }

    /// Record error for statistics
    fn record_error(&mut self, error_type: &str) {
        self.stats.parse_errors += 1;
        self.stats.error_distribution
            .entry(error_type.to_string())
            .and_modify(|e| *e += 1)
            .or_insert(1);
    }

    /// Get parser statistics
    pub fn get_stats(&self) -> &MessageParserStats {
        &self.stats
    }

    /// Reset parser statistics
    pub fn reset_stats(&mut self) {
        self.stats = MessageParserStats::default();
    }
}

impl Default for MessageParser {
    fn default() -> Self {
        Self::new()
    }
}

/// Message builder for beacon transmission capabilities
pub struct MessageBuilder {
    checksum_calculator: MessageParser,
}

impl MessageBuilder {
    pub fn new() -> Self {
        Self {
            checksum_calculator: MessageParser::new(),
        }
    }

    /// Calculate checksum for message data
    pub fn calculate_checksum(&self, data: &[u8]) -> u16 {
        // Simple CRC16-like checksum
        let mut checksum: u16 = 0xFFFF;
        for byte in data {
            checksum ^= *byte as u16;
            for _ in 0..8 {
                if checksum & 0x0001 != 0 {
                    checksum = (checksum >> 1) ^ 0xA001;
                } else {
                    checksum >>= 1;
                }
            }
        }
        checksum
    }

    /// Build V1 message format for transmission
    pub fn build_v1_message(
        &self,
        beacon_id: u16,
        position: GeodeticPosition,
        signal_quality: u8,
        sequence: u16,
    ) -> Result<Vec<u8>, MessageParseError> {
        let mut data = Vec::new();
        
        // Version
        data.push(1u8);
        
        // Beacon ID (2 bytes, little-endian)
        data.extend_from_slice(&beacon_id.to_le_bytes());
        
        // Timestamp (8 bytes, little-endian)
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;
        data.extend_from_slice(&timestamp.to_le_bytes());
        
        // Position (latitude: 8 bytes, longitude: 8 bytes, depth: 4 bytes)
        data.extend_from_slice(&position.latitude.to_le_bytes());
        data.extend_from_slice(&position.longitude.to_le_bytes());
        data.extend_from_slice(&(position.depth as f32).to_le_bytes());
        
        // Signal quality (1 byte)
        data.push(signal_quality);
        
        // Message sequence (2 bytes, little-endian)
        data.extend_from_slice(&sequence.to_le_bytes());
        
        // Calculate and append checksum
        let checksum = self.checksum_calculator.calculate_checksum(&data);
        data.extend_from_slice(&checksum.to_le_bytes());
        
        Ok(data)
    }

    /// Build V2 message format for transmission (more compact)
    pub fn build_v2_message(
        &self,
        beacon_id: u16,
        position: GeodeticPosition,
        signal_quality: u8,
        sequence: u16,
    ) -> Result<Vec<u8>, MessageParseError> {
        let mut data = Vec::new();
        
        // Version
        data.push(2u8);
        
        // Beacon ID (2 bytes, little-endian)
        data.extend_from_slice(&beacon_id.to_le_bytes());
        
        // Relative timestamp (4 bytes, little-endian)
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_secs() as u32;
        let base_epoch = 1704067200u32; // 2024-01-01 in seconds
        let relative_timestamp = timestamp.saturating_sub(base_epoch);
        data.extend_from_slice(&relative_timestamp.to_le_bytes());
        
        // Position (scaled for compactness)
        let lat_scaled = (position.latitude * 1_000_000.0) as i32;
        let lon_scaled = (position.longitude * 1_000_000.0) as i32;
        let depth_mm = (position.depth * 1000.0) as u16;
        
        data.extend_from_slice(&lat_scaled.to_le_bytes());
        data.extend_from_slice(&lon_scaled.to_le_bytes());
        data.extend_from_slice(&depth_mm.to_le_bytes());
        
        // Signal quality (1 byte)
        data.push(signal_quality);
        
        // Message sequence (2 bytes, little-endian)
        data.extend_from_slice(&sequence.to_le_bytes());
        
        // Flags (1 byte, reserved)
        data.push(0u8);
        
        // Calculate and append checksum
        let checksum = self.checksum_calculator.calculate_checksum(&data);
        data.extend_from_slice(&checksum.to_le_bytes());
        
        Ok(data)
    }

    /// Build V3 message format with UUID support for beacons
    pub fn build_v3_message(
        &self,
        beacon_uuid: Uuid,
        position: GeodeticPosition,
        signal_quality: u8,
        sequence: u16,
    ) -> Result<Vec<u8>, MessageParseError> {
        let mut data = Vec::new();
        
        // Version
        data.push(3u8);
        
        // Beacon UUID (16 bytes)
        data.extend_from_slice(beacon_uuid.as_bytes());
        
        // Timestamp (8 bytes, little-endian)
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;
        data.extend_from_slice(&timestamp.to_le_bytes());
        
        // Position (latitude: 8 bytes, longitude: 8 bytes, depth: 4 bytes)
        data.extend_from_slice(&position.latitude.to_le_bytes());
        data.extend_from_slice(&position.longitude.to_le_bytes());
        data.extend_from_slice(&(position.depth as f32).to_le_bytes());
        
        // Signal quality (1 byte)
        data.push(signal_quality);
        
        // Message sequence (2 bytes, little-endian)
        data.extend_from_slice(&sequence.to_le_bytes());
        
        // Calculate and append checksum
        let checksum = self.checksum_calculator.calculate_checksum(&data);
        data.extend_from_slice(&checksum.to_le_bytes());
        
        Ok(data)
    }

    /// Build V1 message format with UUID (converts UUID to u16 for compatibility)
    pub fn build_v1_message_with_uuid(
        &self,
        beacon_uuid: Uuid,
        position: GeodeticPosition,
        signal_quality: u8,
        sequence: u16,
    ) -> Result<Vec<u8>, MessageParseError> {
        // Convert UUID to u16 for V1 compatibility (use first 2 bytes)
        let beacon_id = (beacon_uuid.as_bytes()[0] as u16) << 8 | (beacon_uuid.as_bytes()[1] as u16);
        self.build_v1_message(beacon_id, position, signal_quality, sequence)
    }

    /// Build V2 message format with UUID (converts UUID to u16 for compatibility)
    pub fn build_v2_message_with_uuid(
        &self,
        beacon_uuid: Uuid,
        position: GeodeticPosition,
        signal_quality: u8,
        sequence: u16,
    ) -> Result<Vec<u8>, MessageParseError> {
        // Convert UUID to u16 for V2 compatibility (use first 2 bytes)
        let beacon_id = (beacon_uuid.as_bytes()[0] as u16) << 8 | (beacon_uuid.as_bytes()[1] as u16);
        self.build_v2_message(beacon_id, position, signal_quality, sequence)
    }

    /// Validate message data before transmission
    pub fn validate_message_data(&self, data: &[u8]) -> Result<(), MessageParseError> {
        if data.is_empty() {
            return Err(MessageParseError::CorruptedData {
                details: "Empty message data".to_string(),
            });
        }

        // Check minimum length based on version
        let version = data[0];
        let expected_length = match version {
            1 => 36,
            2 => 23,
            3 => 50,
            _ => return Err(MessageParseError::InvalidVersion { version }),
        };

        if data.len() != expected_length {
            return Err(MessageParseError::InvalidLength {
                expected: expected_length,
                actual: data.len(),
            });
        }

        // Verify checksum
        let checksum_offset = data.len() - 2;
        let stored_checksum = u16::from_le_bytes([data[checksum_offset], data[checksum_offset + 1]]);
        let calculated_checksum = self.checksum_calculator.calculate_checksum(&data[..checksum_offset]);

        if stored_checksum != calculated_checksum {
            return Err(MessageParseError::ChecksumMismatch {
                expected: stored_checksum,
                calculated: calculated_checksum,
            });
        }

        Ok(())
    }
}

impl Default for MessageBuilder {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_message_builder_v1() {
        let builder = MessageBuilder::new();
        let position = GeodeticPosition {
            latitude: 32.123456,
            longitude: -117.654321,
            depth: 10.5,
        };

        let message_data = builder.build_v1_message(123, position, 200, 456).unwrap();
        
        // Validate the built message
        assert!(builder.validate_message_data(&message_data).is_ok());
        
        // Parse it back to verify correctness
        let mut parser = MessageParser::new();
        let raw_message = RawMessage {
            data: message_data,
            timestamp_received: SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_millis() as u64,
            transceiver_id: 1,
            signal_strength: Some(200),
        };
        
        let parsed = parser.parse_message(&raw_message).unwrap();
        assert_eq!(parsed.anchor_id, 123);
        assert_eq!(parsed.signal_quality, 200);
        assert_eq!(parsed.message_sequence, 456);
        assert_eq!(parsed.message_version, 1);
    }

    #[test]
    fn test_message_builder_v2() {
        let builder = MessageBuilder::new();
        let position = GeodeticPosition {
            latitude: 32.123456,
            longitude: -117.654321,
            depth: 10.5,
        };

        let message_data = builder.build_v2_message(123, position, 200, 456).unwrap();
        
        // Validate the built message
        assert!(builder.validate_message_data(&message_data).is_ok());
        
        // Parse it back to verify correctness
        let mut parser = MessageParser::new();
        let raw_message = RawMessage {
            data: message_data,
            timestamp_received: SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_millis() as u64,
            transceiver_id: 1,
            signal_strength: Some(200),
        };
        
        let parsed = parser.parse_message(&raw_message).unwrap();
        assert_eq!(parsed.anchor_id, 123);
        assert_eq!(parsed.signal_quality, 200);
        assert_eq!(parsed.message_sequence, 456);
        assert_eq!(parsed.message_version, 2);
        
        // Check position accuracy (V2 has reduced precision)
        assert!((parsed.position.latitude - position.latitude).abs() < 0.000001);
        assert!((parsed.position.longitude - position.longitude).abs() < 0.000001);
        assert!((parsed.position.depth - position.depth).abs() < 0.001);
    }

    #[test]
    fn test_message_builder_v3_with_uuid() {
        let builder = MessageBuilder::new();
        let beacon_uuid = Uuid::new_v4();
        let position = GeodeticPosition {
            latitude: 32.123456,
            longitude: -117.654321,
            depth: 10.5,
        };

        let message_data = builder.build_v3_message(beacon_uuid, position, 200, 456).unwrap();
        
        // Validate the built message
        assert!(builder.validate_message_data(&message_data).is_ok());
        
        // Parse it back to verify correctness
        let mut parser = MessageParser::new();
        let raw_message = RawMessage {
            data: message_data,
            timestamp_received: SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_millis() as u64,
            transceiver_id: 1,
            signal_strength: Some(200),
        };
        
        let parsed = parser.parse_message(&raw_message).unwrap();
        assert_eq!(parsed.signal_quality, 200);
        assert_eq!(parsed.message_sequence, 456);
        assert_eq!(parsed.message_version, 3);
        
        // Check position accuracy (V3 has full precision)
        assert!((parsed.position.latitude - position.latitude).abs() < 1e-10);
        assert!((parsed.position.longitude - position.longitude).abs() < 1e-10);
        assert!((parsed.position.depth - position.depth).abs() < 1e-6);
    }

    #[test]
    fn test_message_validation() {
        let builder = MessageBuilder::new();
        let position = GeodeticPosition {
            latitude: 32.123456,
            longitude: -117.654321,
            depth: 10.5,
        };

        // Test valid message
        let valid_message = builder.build_v1_message(123, position, 200, 456).unwrap();
        assert!(builder.validate_message_data(&valid_message).is_ok());

        // Test invalid checksum
        let mut invalid_checksum = valid_message.clone();
        let len = invalid_checksum.len();
        invalid_checksum[len - 1] ^= 0xFF; // Corrupt checksum
        assert!(builder.validate_message_data(&invalid_checksum).is_err());

        // Test invalid length
        let mut invalid_length = valid_message.clone();
        invalid_length.pop(); // Remove last byte
        assert!(builder.validate_message_data(&invalid_length).is_err());

        // Test empty message
        assert!(builder.validate_message_data(&[]).is_err());
    }

    #[test]
    fn test_checksum_calculation() {
        let builder = MessageBuilder::new();
        
        // Test known data
        let test_data = b"Hello, World!";
        let checksum1 = builder.checksum_calculator.calculate_checksum(test_data);
        let checksum2 = builder.checksum_calculator.calculate_checksum(test_data);
        
        // Checksum should be consistent
        assert_eq!(checksum1, checksum2);
        
        // Different data should produce different checksums
        let different_data = b"Hello, World?";
        let checksum3 = builder.checksum_calculator.calculate_checksum(different_data);
        assert_ne!(checksum1, checksum3);
    }

    #[test]
    fn test_message_builder_edge_cases() {
        let builder = MessageBuilder::new();
        
        // Test extreme position values
        let extreme_position = GeodeticPosition {
            latitude: 89.999999,
            longitude: 179.999999,
            depth: 10000.0,
        };
        
        // Should handle extreme but valid positions
        assert!(builder.build_v1_message(1, extreme_position, 255, 65535).is_ok());
        assert!(builder.build_v2_message(1, extreme_position, 255, 65535).is_ok());
        assert!(builder.build_v3_message(Uuid::new_v4(), extreme_position, 255, 65535).is_ok());
        
        // Test minimum values
        let min_position = GeodeticPosition {
            latitude: -89.999999,
            longitude: -179.999999,
            depth: 0.0,
        };
        
        assert!(builder.build_v1_message(1, min_position, 0, 0).is_ok());
        assert!(builder.build_v2_message(1, min_position, 0, 0).is_ok());
        assert!(builder.build_v3_message(Uuid::new_v4(), min_position, 0, 0).is_ok());
    }

    #[test]
    fn test_message_format_compatibility() {
        let builder = MessageBuilder::new();
        let mut parser = MessageParser::new();
        
        let position = GeodeticPosition {
            latitude: 32.123456,
            longitude: -117.654321,
            depth: 10.5,
        };
        
        // Test that all message formats can be parsed correctly
        let v1_data = builder.build_v1_message(123, position, 200, 456).unwrap();
        let v2_data = builder.build_v2_message(123, position, 200, 456).unwrap();
        let v3_data = builder.build_v3_message(Uuid::new_v4(), position, 200, 456).unwrap();
        
        let timestamp = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_millis() as u64;
        
        let v1_raw = RawMessage {
            data: v1_data,
            timestamp_received: timestamp,
            transceiver_id: 1,
            signal_strength: Some(200),
        };
        
        let v2_raw = RawMessage {
            data: v2_data,
            timestamp_received: timestamp,
            transceiver_id: 1,
            signal_strength: Some(200),
        };
        
        let v3_raw = RawMessage {
            data: v3_data,
            timestamp_received: timestamp,
            transceiver_id: 1,
            signal_strength: Some(200),
        };
        
        // All should parse successfully
        assert!(parser.parse_message(&v1_raw).is_ok());
        assert!(parser.parse_message(&v2_raw).is_ok());
        assert!(parser.parse_message(&v3_raw).is_ok());
    }

    #[test]
    fn test_uuid_beacon_id_handling() {
        let builder = MessageBuilder::new();
        let mut parser = MessageParser::new();
        
        let beacon_uuid = Uuid::parse_str("550e8400-e29b-41d4-a716-446655440000").unwrap();
        let position = GeodeticPosition {
            latitude: 32.123456,
            longitude: -117.654321,
            depth: 10.5,
        };
        
        let message_data = builder.build_v3_message(beacon_uuid, position, 200, 456).unwrap();
        
        let raw_message = RawMessage {
            data: message_data,
            timestamp_received: SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_millis() as u64,
            transceiver_id: 1,
            signal_strength: Some(200),
        };
        
        let parsed = parser.parse_message(&raw_message).unwrap();
        
        // The anchor_id should be derived from the UUID (first 2 bytes)
        let expected_anchor_id = (beacon_uuid.as_bytes()[0] as u16) << 8 | (beacon_uuid.as_bytes()[1] as u16);
        assert_eq!(parsed.anchor_id, expected_anchor_id);
    }

    #[test]
    fn test_message_sequence_rollover() {
        let builder = MessageBuilder::new();
        let position = GeodeticPosition {
            latitude: 32.123456,
            longitude: -117.654321,
            depth: 10.5,
        };
        
        // Test sequence number at maximum value
        let max_seq_message = builder.build_v1_message(123, position, 200, u16::MAX).unwrap();
        assert!(builder.validate_message_data(&max_seq_message).is_ok());
        
        // Test sequence number rollover (should work fine)
        let rollover_message = builder.build_v1_message(123, position, 200, 0).unwrap();
        assert!(builder.validate_message_data(&rollover_message).is_ok());
    }

    #[test]
    fn test_message_builder_comprehensive_validation() {
        let builder = MessageBuilder::new();
        let mut parser = MessageParser::new();
        
        let position = GeodeticPosition {
            latitude: 32.123456,
            longitude: -117.654321,
            depth: 10.5,
        };
        
        // Test all message formats with comprehensive validation
        let formats = vec![
            ("V1", builder.build_v1_message(123, position, 200, 456).unwrap()),
            ("V2", builder.build_v2_message(123, position, 200, 456).unwrap()),
            ("V3", builder.build_v3_message(Uuid::new_v4(), position, 200, 456).unwrap()),
        ];
        
        for (format_name, message_data) in formats {
            // Validate message structure
            assert!(builder.validate_message_data(&message_data).is_ok(), 
                    "Failed to validate {} message", format_name);
            
            // Parse and verify round-trip consistency
            let raw_message = RawMessage {
                data: message_data.clone(),
                timestamp_received: SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_millis() as u64,
                transceiver_id: 1,
                signal_strength: Some(200),
            };
            
            let parsed = parser.parse_message(&raw_message).unwrap();
            assert_eq!(parsed.signal_quality, 200, "{} signal quality mismatch", format_name);
            assert_eq!(parsed.message_sequence, 456, "{} sequence mismatch", format_name);
            
            // Verify position accuracy based on format
            match format_name {
                "V1" | "V3" => {
                    // Full precision formats
                    assert!((parsed.position.latitude - position.latitude).abs() < 1e-10, 
                            "{} latitude precision loss", format_name);
                    assert!((parsed.position.longitude - position.longitude).abs() < 1e-10, 
                            "{} longitude precision loss", format_name);
                    assert!((parsed.position.depth - position.depth).abs() < 1e-6, 
                            "{} depth precision loss", format_name);
                }
                "V2" => {
                    // Reduced precision format
                    assert!((parsed.position.latitude - position.latitude).abs() < 0.000001, 
                            "{} latitude precision acceptable", format_name);
                    assert!((parsed.position.longitude - position.longitude).abs() < 0.000001, 
                            "{} longitude precision acceptable", format_name);
                    assert!((parsed.position.depth - position.depth).abs() < 0.001, 
                            "{} depth precision acceptable", format_name);
                }
                _ => panic!("Unknown format: {}", format_name),
            }
        }
    }

    #[test]
    fn test_message_builder_error_conditions() {
        let builder = MessageBuilder::new();
        
        // Test validation with corrupted data
        let mut corrupted_data = vec![1u8; 36]; // V1 message length
        corrupted_data[0] = 1; // Valid version
        // Leave rest as invalid data
        
        // Should fail validation due to invalid checksum
        assert!(builder.validate_message_data(&corrupted_data).is_err());
        
        // Test with invalid version
        let mut invalid_version = vec![0u8; 36];
        invalid_version[0] = 99; // Invalid version
        assert!(builder.validate_message_data(&invalid_version).is_err());
        
        // Test with wrong length for version
        let wrong_length = vec![1u8; 20]; // Too short for V1
        assert!(builder.validate_message_data(&wrong_length).is_err());
    }

    #[test]
    fn test_uuid_to_anchor_id_conversion() {
        let builder = MessageBuilder::new();
        let position = GeodeticPosition {
            latitude: 32.123456,
            longitude: -117.654321,
            depth: 10.5,
        };
        
        // Test specific UUID to ensure consistent conversion
        let test_uuid = Uuid::parse_str("12345678-1234-5678-9abc-123456789abc").unwrap();
        
        // Build V1 and V2 messages with UUID
        let v1_message = builder.build_v1_message_with_uuid(test_uuid, position, 200, 456).unwrap();
        let v2_message = builder.build_v2_message_with_uuid(test_uuid, position, 200, 456).unwrap();
        
        // Parse both messages
        let mut parser = MessageParser::new();
        let timestamp = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_millis() as u64;
        
        let v1_raw = RawMessage {
            data: v1_message,
            timestamp_received: timestamp,
            transceiver_id: 1,
            signal_strength: Some(200),
        };
        
        let v2_raw = RawMessage {
            data: v2_message,
            timestamp_received: timestamp,
            transceiver_id: 1,
            signal_strength: Some(200),
        };
        
        let v1_parsed = parser.parse_message(&v1_raw).unwrap();
        let v2_parsed = parser.parse_message(&v2_raw).unwrap();
        
        // Both should have the same anchor_id derived from UUID
        assert_eq!(v1_parsed.anchor_id, v2_parsed.anchor_id);
        
        // Verify the conversion matches expected value
        let expected_anchor_id = (test_uuid.as_bytes()[0] as u16) << 8 | (test_uuid.as_bytes()[1] as u16);
        assert_eq!(v1_parsed.anchor_id, expected_anchor_id);
    }

    #[test]
    fn test_message_builder_performance_characteristics() {
        let builder = MessageBuilder::new();
        let position = GeodeticPosition {
            latitude: 32.123456,
            longitude: -117.654321,
            depth: 10.5,
        };
        
        // Test message size characteristics
        let v1_message = builder.build_v1_message(123, position, 200, 456).unwrap();
        let v2_message = builder.build_v2_message(123, position, 200, 456).unwrap();
        let v3_message = builder.build_v3_message(Uuid::new_v4(), position, 200, 456).unwrap();
        
        // Verify expected message sizes
        assert_eq!(v1_message.len(), 36, "V1 message should be 36 bytes");
        assert_eq!(v2_message.len(), 23, "V2 message should be 23 bytes (more compact)");
        assert_eq!(v3_message.len(), 50, "V3 message should be 50 bytes (with UUID)");
        
        // V2 should be the most compact for bandwidth efficiency
        assert!(v2_message.len() < v1_message.len(), "V2 should be more compact than V1");
        assert!(v2_message.len() < v3_message.len(), "V2 should be more compact than V3");
    }
}

/// Data validator for comprehensive message validation
pub struct MessageValidator {
    /// Timestamp precision requirements (milliseconds)
    required_timestamp_precision_ms: u64,
    /// Position precision requirements (meters)
    required_position_precision_m: f64,
    /// Signal quality thresholds
    min_signal_quality: u8,
    /// Sequence number tracking for duplicate detection
    sequence_tracking: HashMap<u16, u16>, // anchor_id -> last_sequence
    /// Validation statistics
    stats: ValidationStats,
}

/// Validation statistics
#[derive(Debug, Clone, Default)]
pub struct ValidationStats {
    pub total_validations: u64,
    pub passed_validations: u64,
    pub failed_validations: u64,
    pub duplicate_messages: u64,
    pub out_of_order_messages: u64,
    pub precision_violations: u64,
}

/// Validation result with detailed information
#[derive(Debug, Clone)]
pub struct ValidationResult {
    pub is_valid: bool,
    pub warnings: Vec<String>,
    pub errors: Vec<String>,
    pub quality_score: f32, // 0.0 to 1.0
}

impl MessageValidator {
    /// Create new message validator
    pub fn new() -> Self {
        Self {
            required_timestamp_precision_ms: 1, // 1ms precision
            required_position_precision_m: 0.01, // 1cm precision
            min_signal_quality: 50, // Minimum acceptable signal quality
            sequence_tracking: HashMap::new(),
            stats: ValidationStats::default(),
        }
    }

    /// Validate anchor message with comprehensive checks
    pub fn validate_anchor_message(&mut self, message: &AnchorMessage) -> ValidationResult {
        self.stats.total_validations += 1;
        
        let mut result = ValidationResult {
            is_valid: true,
            warnings: Vec::new(),
            errors: Vec::new(),
            quality_score: 1.0,
        };

        // Check timestamp precision
        if message.timestamp_ms % self.required_timestamp_precision_ms != 0 {
            result.warnings.push(format!(
                "Timestamp precision lower than required {}ms",
                self.required_timestamp_precision_ms
            ));
            result.quality_score *= 0.95;
            self.stats.precision_violations += 1;
        }

        // Check signal quality
        if message.signal_quality < self.min_signal_quality {
            result.warnings.push(format!(
                "Signal quality {} below minimum threshold {}",
                message.signal_quality, self.min_signal_quality
            ));
            result.quality_score *= 0.9;
        }

        // Check for duplicate/out-of-order messages
        if let Some(&last_sequence) = self.sequence_tracking.get(&message.anchor_id) {
            if message.message_sequence == last_sequence {
                result.errors.push("Duplicate message sequence number".to_string());
                result.is_valid = false;
                self.stats.duplicate_messages += 1;
            } else if message.message_sequence < last_sequence {
                result.warnings.push("Out-of-order message sequence".to_string());
                result.quality_score *= 0.8;
                self.stats.out_of_order_messages += 1;
            }
        }

        // Update sequence tracking
        self.sequence_tracking.insert(message.anchor_id, message.message_sequence);

        // Update statistics
        if result.is_valid {
            self.stats.passed_validations += 1;
        } else {
            self.stats.failed_validations += 1;
        }

        result
    }

    /// Get validation statistics
    pub fn get_stats(&self) -> &ValidationStats {
        &self.stats
    }

    /// Reset validation statistics
    pub fn reset_stats(&mut self) {
        self.stats = ValidationStats::default();
        self.sequence_tracking.clear();
    }
}

impl Default for MessageValidator {
    fn default() -> Self {
        Self::new()
    }
}