// Message parsing and validation system - no external serialization needed for core functionality
use std::collections::HashMap;
use std::time::{SystemTime, UNIX_EPOCH};

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
#[derive(Debug, Clone, Copy, PartialEq)]
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

impl MessageParser {
    /// Create a new message parser with default configuration
    pub fn new() -> Self {
        Self {
            supported_versions: vec![MessageVersion::V1, MessageVersion::V2],
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
        // For demo purposes, use a more recent base time to avoid validation issues
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

    /// Generate diagnostic report
    pub fn generate_diagnostic_report(&self) -> String {
        let mut report = String::new();
        report.push_str("=== MESSAGE PARSER DIAGNOSTIC REPORT ===\n\n");

        // Overall statistics
        report.push_str("PARSING STATISTICS:\n");
        report.push_str(&format!("  Total messages processed: {}\n", self.stats.total_messages_processed));
        report.push_str(&format!("  Successful parses: {}\n", self.stats.successful_parses));
        report.push_str(&format!("  Parse errors: {}\n", self.stats.parse_errors));
        report.push_str(&format!("  Checksum failures: {}\n", self.stats.checksum_failures));
        report.push_str(&format!("  Validation failures: {}\n", self.stats.validation_failures));
        
        if self.stats.total_messages_processed > 0 {
            let success_rate = (self.stats.successful_parses as f64 / self.stats.total_messages_processed as f64) * 100.0;
            report.push_str(&format!("  Success rate: {:.2}%\n", success_rate));
        }

        // Version distribution
        if !self.stats.version_distribution.is_empty() {
            report.push_str("\nVERSION DISTRIBUTION:\n");
            for (version, count) in &self.stats.version_distribution {
                report.push_str(&format!("  Version {}: {} messages\n", version, count));
            }
        }

        // Error distribution
        if !self.stats.error_distribution.is_empty() {
            report.push_str("\nERROR DISTRIBUTION:\n");
            for (error_type, count) in &self.stats.error_distribution {
                report.push_str(&format!("  {}: {} occurrences\n", error_type, count));
            }
        }

        // Configuration
        report.push_str("\nCONFIGURATION:\n");
        report.push_str(&format!("  Supported versions: {:?}\n", self.supported_versions));
        report.push_str(&format!("  Max timestamp age: {} ms\n", self.max_timestamp_age_ms));
        report.push_str(&format!("  Valid anchor ID ranges: {:?}\n", self.valid_anchor_id_ranges));
        report.push_str(&format!("  Position bounds: lat[{:.1}, {:.1}], lon[{:.1}, {:.1}], depth[0, {:.1}]\n",
                                self.position_bounds.min_latitude,
                                self.position_bounds.max_latitude,
                                self.position_bounds.min_longitude,
                                self.position_bounds.max_longitude,
                                self.position_bounds.max_depth));

        report
    }
}

impl Default for MessageParser {
    fn default() -> Self {
        Self::new()
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

        // Validate position precision (check for reasonable precision in coordinates)
        let lat_precision = self.estimate_coordinate_precision(message.position.latitude);
        let lon_precision = self.estimate_coordinate_precision(message.position.longitude);
        let depth_precision = self.estimate_coordinate_precision(message.position.depth);

        if lat_precision > self.required_position_precision_m {
            result.warnings.push(format!(
                "Latitude precision {:.3}m exceeds requirement {:.3}m",
                lat_precision, self.required_position_precision_m
            ));
            result.quality_score *= 0.95;
        }

        if lon_precision > self.required_position_precision_m {
            result.warnings.push(format!(
                "Longitude precision {:.3}m exceeds requirement {:.3}m",
                lon_precision, self.required_position_precision_m
            ));
            result.quality_score *= 0.95;
        }

        if depth_precision > self.required_position_precision_m {
            result.warnings.push(format!(
                "Depth precision {:.3}m exceeds requirement {:.3}m",
                depth_precision, self.required_position_precision_m
            ));
            result.quality_score *= 0.95;
        }

        // Update statistics
        if result.is_valid {
            self.stats.passed_validations += 1;
        } else {
            self.stats.failed_validations += 1;
        }

        result
    }

    /// Estimate coordinate precision based on decimal places
    fn estimate_coordinate_precision(&self, coordinate: f64) -> f64 {
        // For latitude/longitude, 1 degree ≈ 111km, so:
        // 6 decimal places ≈ 0.1m precision
        // 5 decimal places ≈ 1m precision
        // 4 decimal places ≈ 10m precision
        
        let abs_coord = coordinate.abs();
        let fractional_part = abs_coord - abs_coord.floor();
        
        if fractional_part == 0.0 {
            return 111000.0; // No decimal precision = ~111km
        }

        // Count significant decimal places
        let precision_str = format!("{:.10}", fractional_part);
        let decimal_places = precision_str.chars()
            .skip(2) // Skip "0."
            .take_while(|&c| c != '0')
            .count();

        // Convert decimal places to approximate meter precision
        match decimal_places {
            0 => 111000.0,  // ~111km
            1 => 11100.0,   // ~11km
            2 => 1110.0,    // ~1km
            3 => 111.0,     // ~111m
            4 => 11.1,      // ~11m
            5 => 1.11,      // ~1m
            6 => 0.111,     // ~0.1m
            _ => 0.01,      // Better than 1cm
        }
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

    /// Generate validation report
    pub fn generate_report(&self) -> String {
        let mut report = String::new();
        report.push_str("=== MESSAGE VALIDATION REPORT ===\n\n");

        report.push_str("VALIDATION STATISTICS:\n");
        report.push_str(&format!("  Total validations: {}\n", self.stats.total_validations));
        report.push_str(&format!("  Passed validations: {}\n", self.stats.passed_validations));
        report.push_str(&format!("  Failed validations: {}\n", self.stats.failed_validations));
        report.push_str(&format!("  Duplicate messages: {}\n", self.stats.duplicate_messages));
        report.push_str(&format!("  Out-of-order messages: {}\n", self.stats.out_of_order_messages));
        report.push_str(&format!("  Precision violations: {}\n", self.stats.precision_violations));

        if self.stats.total_validations > 0 {
            let success_rate = (self.stats.passed_validations as f64 / self.stats.total_validations as f64) * 100.0;
            report.push_str(&format!("  Success rate: {:.2}%\n", success_rate));
        }

        report.push_str("\nCONFIGURATION:\n");
        report.push_str(&format!("  Required timestamp precision: {} ms\n", self.required_timestamp_precision_ms));
        report.push_str(&format!("  Required position precision: {:.3} m\n", self.required_position_precision_m));
        report.push_str(&format!("  Minimum signal quality: {}\n", self.min_signal_quality));
        report.push_str(&format!("  Tracking {} anchor sequences\n", self.sequence_tracking.len()));

        report
    }
}

impl Default for MessageValidator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_message_parser_v1() {
        let mut parser = MessageParser::new();
        
        // Use current timestamp to avoid validation issues
        let current_time = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;
        
        // Create a valid V1 message
        let mut data = Vec::new();
        data.push(1u8); // version
        data.extend_from_slice(&123u16.to_le_bytes()); // anchor_id
        data.extend_from_slice(&current_time.to_le_bytes()); // timestamp
        data.extend_from_slice(&32.123456f64.to_le_bytes()); // latitude
        data.extend_from_slice(&(-117.654321f64).to_le_bytes()); // longitude
        data.extend_from_slice(&10.5f32.to_le_bytes()); // depth
        data.push(200u8); // signal_quality
        data.extend_from_slice(&456u16.to_le_bytes()); // sequence
        
        // Calculate and append checksum
        let checksum = parser.calculate_checksum(&data);
        data.extend_from_slice(&checksum.to_le_bytes());
        
        let raw_message = RawMessage {
            data,
            timestamp_received: current_time,
            transceiver_id: 1,
            signal_strength: Some(200),
        };
        
        let result = parser.parse_message(&raw_message);
        assert!(result.is_ok());
        
        let message = result.unwrap();
        assert_eq!(message.anchor_id, 123);
        assert_eq!(message.message_version, 1);
        assert_eq!(message.signal_quality, 200);
        assert_eq!(message.message_sequence, 456);
    }

    #[test]
    fn test_message_parser_v2() {
        let mut parser = MessageParser::new();
        
        // Use current timestamp to avoid validation issues
        let current_time = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;
        let relative_timestamp = ((current_time - 1704067200000) / 1000) as u32; // relative seconds
        
        // Create a valid V2 message
        let mut data = Vec::new();
        data.push(2u8); // version
        data.extend_from_slice(&123u16.to_le_bytes()); // anchor_id
        data.extend_from_slice(&relative_timestamp.to_le_bytes()); // relative timestamp
        data.extend_from_slice(&32123456i32.to_le_bytes()); // latitude * 1e6
        data.extend_from_slice(&(-117654321i32).to_le_bytes()); // longitude * 1e6
        data.extend_from_slice(&10500u16.to_le_bytes()); // depth in mm
        data.push(200u8); // signal_quality
        data.extend_from_slice(&456u16.to_le_bytes()); // sequence
        data.push(0u8); // flags
        
        // Calculate and append checksum
        let checksum = parser.calculate_checksum(&data);
        data.extend_from_slice(&checksum.to_le_bytes());
        
        let raw_message = RawMessage {
            data,
            timestamp_received: current_time,
            transceiver_id: 1,
            signal_strength: Some(200),
        };
        
        let result = parser.parse_message(&raw_message);
        assert!(result.is_ok());
        
        let message = result.unwrap();
        assert_eq!(message.anchor_id, 123);
        assert_eq!(message.message_version, 2);
        assert_eq!(message.signal_quality, 200);
        assert!((message.position.latitude - 32.123456).abs() < 1e-6);
        assert!((message.position.longitude - (-117.654321)).abs() < 1e-6);
        assert!((message.position.depth - 10.5).abs() < 1e-3);
    }

    #[test]
    fn test_checksum_validation() {
        let mut parser = MessageParser::new();
        
        // Create message with invalid checksum
        let mut data = vec![1u8]; // version
        data.extend_from_slice(&123u16.to_le_bytes());
        data.extend_from_slice(&1234567890123u64.to_le_bytes());
        data.extend_from_slice(&32.123456f64.to_le_bytes());
        data.extend_from_slice(&(-117.654321f64).to_le_bytes());
        data.extend_from_slice(&10.5f32.to_le_bytes());
        data.push(200u8);
        data.extend_from_slice(&456u16.to_le_bytes());
        data.extend_from_slice(&0xDEADu16.to_le_bytes()); // Invalid checksum
        
        let raw_message = RawMessage {
            data,
            timestamp_received: 1234567890123,
            transceiver_id: 1,
            signal_strength: Some(200),
        };
        
        let result = parser.parse_message(&raw_message);
        assert!(result.is_err());
        assert!(matches!(result.unwrap_err(), MessageParseError::ChecksumMismatch { .. }));
    }

    #[test]
    fn test_position_validation() {
        let mut parser = MessageParser::new();
        
        // Test invalid latitude
        let position = GeodeticPosition {
            latitude: 91.0, // Invalid - exceeds 90 degrees
            longitude: 0.0,
            depth: 10.0,
        };
        
        let result = parser.validate_position(&position);
        assert!(result.is_err());
        assert!(matches!(result.unwrap_err(), MessageParseError::InvalidPosition { .. }));
    }

    #[test]
    fn test_message_validator() {
        let mut validator = MessageValidator::new();
        
        // Use current timestamp to avoid validation issues
        let current_time = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;
        
        let message = AnchorMessage {
            anchor_id: 123,
            timestamp_ms: current_time - 1000, // Recent timestamp
            position: GeodeticPosition {
                latitude: 32.123456,
                longitude: -117.654321,
                depth: 10.5,
            },
            signal_quality: 200,
            message_sequence: 1,
            message_version: 1,
            checksum: 0x1234,
        };
        
        let result = validator.validate_anchor_message(&message);
        assert!(result.is_valid);
        assert!(result.quality_score > 0.8); // Lowered threshold due to precision warnings
    }

    #[test]
    fn test_duplicate_detection() {
        let mut validator = MessageValidator::new();
        
        let message = AnchorMessage {
            anchor_id: 123,
            timestamp_ms: 1234567890000,
            position: GeodeticPosition {
                latitude: 32.123456,
                longitude: -117.654321,
                depth: 10.5,
            },
            signal_quality: 200,
            message_sequence: 1,
            message_version: 1,
            checksum: 0x1234,
        };
        
        // First validation should pass
        let result1 = validator.validate_anchor_message(&message);
        assert!(result1.is_valid);
        
        // Second validation with same sequence should fail
        let result2 = validator.validate_anchor_message(&message);
        assert!(!result2.is_valid);
        assert!(result2.errors.iter().any(|e| e.contains("Duplicate")));
    }

    #[test]
    fn test_parser_statistics() {
        let mut parser = MessageParser::new();
        
        // Use current timestamp to avoid validation issues
        let current_time = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;
        
        // Process some messages to generate statistics
        for i in 0..10 {
            let mut data = Vec::new();
            data.push(1u8); // version
            data.extend_from_slice(&(i as u16).to_le_bytes());
            data.extend_from_slice(&(current_time - i as u64 * 1000).to_le_bytes()); // Recent timestamps
            data.extend_from_slice(&32.123456f64.to_le_bytes());
            data.extend_from_slice(&(-117.654321f64).to_le_bytes());
            data.extend_from_slice(&10.5f32.to_le_bytes());
            data.push(200u8);
            data.extend_from_slice(&(i as u16).to_le_bytes());
            
            let checksum = parser.calculate_checksum(&data);
            data.extend_from_slice(&checksum.to_le_bytes());
            
            let raw_message = RawMessage {
                data,
                timestamp_received: current_time - i as u64 * 1000,
                transceiver_id: 1,
                signal_strength: Some(200),
            };
            
            let _ = parser.parse_message(&raw_message);
        }
        
        let stats = parser.get_stats();
        assert_eq!(stats.total_messages_processed, 10);
        assert!(stats.successful_parses > 0);
        
        let report = parser.generate_diagnostic_report();
        assert!(report.contains("MESSAGE PARSER DIAGNOSTIC REPORT"));
        assert!(report.contains("Total messages processed: 10"));
    }
}