use crate::processing::parser::{AnchorMessage, GeodeticPosition};
use std::collections::HashMap;
use std::fmt;
use serde::{Serialize, Deserialize};

/// Configuration for data validation parameters
#[derive(Debug, Clone)]
pub struct ValidationConfig {
    /// Maximum age of anchor messages in milliseconds
    pub max_message_age_ms: u64,
    /// Maximum allowed time drift between anchors in milliseconds
    pub max_time_drift_ms: u64,
    /// Minimum signal quality threshold (0-255)
    pub min_signal_quality: u8,
    /// Maximum distance between consecutive position reports from same anchor (meters)
    pub max_position_jump_m: f64,
    /// Maximum allowed depth for underwater operations (meters)
    pub max_depth_m: f64,
    /// Minimum number of anchors required for positioning
    pub min_anchor_count: usize,
    /// Enable strict geometric validation
    pub strict_geometry_validation: bool,
}

impl Default for ValidationConfig {
    fn default() -> Self {
        Self {
            max_message_age_ms: 30000,      // 30 seconds
            max_time_drift_ms: 1000,       // 1 second
            min_signal_quality: 50,        // Minimum acceptable quality
            max_position_jump_m: 100.0,    // 100 meters max jump
            max_depth_m: 6000.0,           // 6km max depth
            min_anchor_count: 3,           // Minimum for 2D positioning
            strict_geometry_validation: true,
        }
    }
}

/// Validation errors that can occur during data validation
#[derive(Debug, Clone, PartialEq)]
pub enum ValidationError {
    StaleMessage { anchor_id: u16, age_ms: u64 },
    TimeDriftExceeded { anchor_id: u16, drift_ms: u64 },
    PoorSignalQuality { anchor_id: u16, quality: u8 },
    PositionJumpDetected { anchor_id: u16, distance_m: f64 },
    InvalidDepth { anchor_id: u16, depth: f64 },
    InsufficientAnchors { available: usize, required: usize },
    DuplicateAnchor { anchor_id: u16 },
    GeometryTooNarrow { condition_number: f64 },
    AnchorTooClose { anchor1: u16, anchor2: u16, distance_m: f64 },
    TimestampInconsistency { anchor_id: u16, details: String },
    SequenceNumberGap { anchor_id: u16, expected: u16, received: u16 },
}

impl fmt::Display for ValidationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ValidationError::StaleMessage { anchor_id, age_ms } => {
                write!(f, "Stale message from anchor {}: {} ms old", anchor_id, age_ms)
            }
            ValidationError::TimeDriftExceeded { anchor_id, drift_ms } => {
                write!(f, "Time drift exceeded for anchor {}: {} ms", anchor_id, drift_ms)
            }
            ValidationError::PoorSignalQuality { anchor_id, quality } => {
                write!(f, "Poor signal quality from anchor {}: {}", anchor_id, quality)
            }
            ValidationError::PositionJumpDetected { anchor_id, distance_m } => {
                write!(f, "Position jump detected for anchor {}: {:.2} m", anchor_id, distance_m)
            }
            ValidationError::InvalidDepth { anchor_id, depth } => {
                write!(f, "Invalid depth for anchor {}: {:.2} m", anchor_id, depth)
            }
            ValidationError::InsufficientAnchors { available, required } => {
                write!(f, "Insufficient anchors: {} available, {} required", available, required)
            }
            ValidationError::DuplicateAnchor { anchor_id } => {
                write!(f, "Duplicate anchor ID: {}", anchor_id)
            }
            ValidationError::GeometryTooNarrow { condition_number } => {
                write!(f, "Anchor geometry too narrow: condition number {:.2}", condition_number)
            }
            ValidationError::AnchorTooClose { anchor1, anchor2, distance_m } => {
                write!(f, "Anchors {} and {} too close: {:.2} m", anchor1, anchor2, distance_m)
            }
            ValidationError::TimestampInconsistency { anchor_id, details } => {
                write!(f, "Timestamp inconsistency for anchor {}: {}", anchor_id, details)
            }
            ValidationError::SequenceNumberGap { anchor_id, expected, received } => {
                write!(f, "Sequence gap for anchor {}: expected {}, got {}", anchor_id, expected, received)
            }
        }
    }
}

impl std::error::Error for ValidationError {}

/// Validation result containing validated messages and any warnings
#[derive(Debug, Clone)]
pub struct ValidationResult {
    pub valid_messages: Vec<AnchorMessage>,
    pub warnings: Vec<ValidationError>,
    pub rejected_messages: Vec<(AnchorMessage, ValidationError)>,
    pub geometry_quality: GeometryQuality,
}

/// Assessment of anchor geometry quality for positioning
#[derive(Debug, Clone, PartialEq, PartialOrd, Serialize, Deserialize)]
pub enum GeometryQuality {
    Excellent,      // Well-distributed anchors, low GDOP
    Good,          // Adequate distribution
    Acceptable,    // Marginal but usable
    Poor,          // Poor distribution, high uncertainty
    Degenerate,    // Nearly singular geometry
}

/// Historical data for an anchor to track consistency
#[derive(Debug, Clone)]
struct AnchorHistory {
    last_position: GeodeticPosition,
    last_timestamp: u64,
    last_sequence: u16,
    message_count: u32,
    quality_history: Vec<u8>,
}

/// Data validator for anchor messages and positioning data
pub struct DataValidator {
    config: ValidationConfig,
    anchor_history: HashMap<u16, AnchorHistory>,
    reference_time: Option<u64>,
}

impl DataValidator {
    /// Create a new data validator with default configuration
    pub fn new() -> Self {
        Self {
            config: ValidationConfig::default(),
            anchor_history: HashMap::new(),
            reference_time: None,
        }
    }

    /// Create a validator with custom configuration
    pub fn with_config(config: ValidationConfig) -> Self {
        Self {
            config,
            anchor_history: HashMap::new(),
            reference_time: None,
        }
    }

    /// Update validation configuration
    pub fn update_config(&mut self, config: ValidationConfig) {
        self.config = config;
    }

    /// Validate a batch of anchor messages
    pub fn validate_messages(&mut self, messages: Vec<AnchorMessage>) -> ValidationResult {
        let mut valid_messages = Vec::new();
        let mut warnings = Vec::new();
        let mut rejected_messages = Vec::new();
        let current_time = self.get_current_time();

        // Set reference time if not set
        if self.reference_time.is_none() && !messages.is_empty() {
            self.reference_time = Some(messages[0].timestamp_ms);
        }

        // Check for duplicate anchor IDs in this batch
        let mut seen_anchors = std::collections::HashSet::new();
        for message in &messages {
            if !seen_anchors.insert(message.anchor_id) {
                warnings.push(ValidationError::DuplicateAnchor {
                    anchor_id: message.anchor_id,
                });
            }
        }

        // Validate each message individually
        for message in messages {
            match self.validate_single_message(&message, current_time) {
                Ok(()) => {
                    self.update_anchor_history(&message);
                    valid_messages.push(message);
                }
                Err(error) => {
                    rejected_messages.push((message, error));
                }
            }
        }

        // Check minimum anchor count
        if valid_messages.len() < self.config.min_anchor_count {
            warnings.push(ValidationError::InsufficientAnchors {
                available: valid_messages.len(),
                required: self.config.min_anchor_count,
            });
        }

        // Assess geometry quality
        let geometry_quality = if valid_messages.len() >= 3 {
            self.assess_geometry_quality(&valid_messages)
        } else {
            GeometryQuality::Degenerate
        };

        ValidationResult {
            valid_messages,
            warnings,
            rejected_messages,
            geometry_quality,
        }
    }

    /// Validate a single anchor message
    fn validate_single_message(&self, message: &AnchorMessage, current_time: u64) -> Result<(), ValidationError> {
        // Check message age
        let message_age = current_time.saturating_sub(message.timestamp_ms);
        if message_age > self.config.max_message_age_ms {
            return Err(ValidationError::StaleMessage {
                anchor_id: message.anchor_id,
                age_ms: message_age,
            });
        }

        // Check signal quality
        if message.signal_quality < self.config.min_signal_quality {
            return Err(ValidationError::PoorSignalQuality {
                anchor_id: message.anchor_id,
                quality: message.signal_quality,
            });
        }

        // Check depth validity
        if message.position.depth < 0.0 || message.position.depth > self.config.max_depth_m {
            return Err(ValidationError::InvalidDepth {
                anchor_id: message.anchor_id,
                depth: message.position.depth,
            });
        }

        // Check against historical data if available
        if let Some(history) = self.anchor_history.get(&message.anchor_id) {
            self.validate_against_history(message, history)?;
        }

        // Check time drift against reference
        if let Some(ref_time) = self.reference_time {
            let time_diff = (message.timestamp_ms as i64 - ref_time as i64).abs() as u64;
            if time_diff > self.config.max_time_drift_ms {
                return Err(ValidationError::TimeDriftExceeded {
                    anchor_id: message.anchor_id,
                    drift_ms: time_diff,
                });
            }
        }

        Ok(())
    }

    /// Validate message against historical data for consistency
    fn validate_against_history(&self, message: &AnchorMessage, history: &AnchorHistory) -> Result<(), ValidationError> {
        // Check for position jumps
        let distance = self.calculate_distance(&message.position, &history.last_position);
        if distance > self.config.max_position_jump_m {
            return Err(ValidationError::PositionJumpDetected {
                anchor_id: message.anchor_id,
                distance_m: distance,
            });
        }

        // Check timestamp consistency (should be increasing)
        if message.timestamp_ms <= history.last_timestamp {
            return Err(ValidationError::TimestampInconsistency {
                anchor_id: message.anchor_id,
                details: format!("Timestamp {} <= previous {}", message.timestamp_ms, history.last_timestamp),
            });
        }

        // Check sequence number consistency
        let expected_sequence = history.last_sequence.wrapping_add(1);
        if message.message_sequence != expected_sequence && message.message_sequence != 0 {
            // Allow sequence reset to 0, but warn about gaps
            if message.message_sequence > expected_sequence {
                return Err(ValidationError::SequenceNumberGap {
                    anchor_id: message.anchor_id,
                    expected: expected_sequence,
                    received: message.message_sequence,
                });
            }
        }

        Ok(())
    }

    /// Update historical data for an anchor
    fn update_anchor_history(&mut self, message: &AnchorMessage) {
        let history = self.anchor_history.entry(message.anchor_id).or_insert(AnchorHistory {
            last_position: message.position.clone(),
            last_timestamp: message.timestamp_ms,
            last_sequence: message.message_sequence,
            message_count: 0,
            quality_history: Vec::new(),
        });

        history.last_position = message.position.clone();
        history.last_timestamp = message.timestamp_ms;
        history.last_sequence = message.message_sequence;
        history.message_count += 1;
        
        // Keep last 10 quality measurements
        history.quality_history.push(message.signal_quality);
        if history.quality_history.len() > 10 {
            history.quality_history.remove(0);
        }
    }

    /// Assess the quality of anchor geometry for positioning
    fn assess_geometry_quality(&self, messages: &[AnchorMessage]) -> GeometryQuality {
        if messages.len() < 3 {
            return GeometryQuality::Degenerate;
        }

        // Check for minimum distances between anchors
        let min_distance = 10.0; // 10 meters minimum
        for i in 0..messages.len() {
            for j in (i + 1)..messages.len() {
                let distance = self.calculate_distance(&messages[i].position, &messages[j].position);
                if distance < min_distance {
                    return GeometryQuality::Poor;
                }
            }
        }

        // For 3D positioning, check if anchors are coplanar
        if messages.len() >= 4 && self.config.strict_geometry_validation {
            let volume = self.calculate_tetrahedron_volume(messages);
            if volume < 1000.0 { // Less than 1000 cubic meters
                return GeometryQuality::Poor;
            }
        }

        // Calculate approximate condition number for geometry assessment
        let condition_number = self.estimate_condition_number(messages);
        
        match condition_number {
            x if x < 5.0 => GeometryQuality::Excellent,
            x if x < 15.0 => GeometryQuality::Good,
            x if x < 50.0 => GeometryQuality::Acceptable,
            x if x < 1000.0 => GeometryQuality::Poor,
            _ => GeometryQuality::Degenerate,
        }
    }

    /// Calculate distance between two geodetic positions (simplified)
    fn calculate_distance(&self, pos1: &GeodeticPosition, pos2: &GeodeticPosition) -> f64 {
        // Simplified distance calculation using Euclidean approximation
        // For more accuracy, use proper geodetic distance calculation
        let lat_diff = (pos1.latitude - pos2.latitude) * 111320.0; // ~111.32 km per degree
        let lon_diff = (pos1.longitude - pos2.longitude) * 111320.0 * pos1.latitude.to_radians().cos();
        let depth_diff = pos1.depth - pos2.depth;
        
        (lat_diff * lat_diff + lon_diff * lon_diff + depth_diff * depth_diff).sqrt()
    }

    /// Calculate volume of tetrahedron formed by first 4 anchors
    fn calculate_tetrahedron_volume(&self, messages: &[AnchorMessage]) -> f64 {
        if messages.len() < 4 {
            return 0.0;
        }

        // Convert to local coordinates for volume calculation
        let ref_pos = &messages[0].position;
        let mut points = Vec::new();
        
        for message in messages.iter().take(4) {
            let lat_m = (message.position.latitude - ref_pos.latitude) * 111320.0;
            let lon_m = (message.position.longitude - ref_pos.longitude) * 111320.0 * ref_pos.latitude.to_radians().cos();
            let depth_m = message.position.depth - ref_pos.depth;
            points.push([lat_m, lon_m, depth_m]);
        }

        // Calculate volume using scalar triple product
        let a = [points[1][0] - points[0][0], points[1][1] - points[0][1], points[1][2] - points[0][2]];
        let b = [points[2][0] - points[0][0], points[2][1] - points[0][1], points[2][2] - points[0][2]];
        let c = [points[3][0] - points[0][0], points[3][1] - points[0][1], points[3][2] - points[0][2]];

        // Cross product b × c
        let cross = [
            b[1] * c[2] - b[2] * c[1],
            b[2] * c[0] - b[0] * c[2],
            b[0] * c[1] - b[1] * c[0],
        ];

        // Dot product a · (b × c)
        let scalar_triple = a[0] * cross[0] + a[1] * cross[1] + a[2] * cross[2];
        
        (scalar_triple.abs() / 6.0)
    }

    /// Estimate condition number for geometry assessment
    fn estimate_condition_number(&self, messages: &[AnchorMessage]) -> f64 {
        // Simplified condition number estimation based on anchor spread
        if messages.len() < 3 {
            return f64::INFINITY;
        }

        let mut min_distance = f64::INFINITY;
        let mut max_distance = 0.0f64;

        for i in 0..messages.len() {
            for j in (i + 1)..messages.len() {
                let distance = self.calculate_distance(&messages[i].position, &messages[j].position);
                min_distance = min_distance.min(distance);
                max_distance = max_distance.max(distance);
            }
        }

        if min_distance > 0.0 {
            max_distance / min_distance
        } else {
            f64::INFINITY
        }
    }

    /// Get current system time in milliseconds
    fn get_current_time(&self) -> u64 {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis() as u64
    }

    /// Clear historical data for all anchors
    pub fn clear_history(&mut self) {
        self.anchor_history.clear();
        self.reference_time = None;
    }

    /// Get statistics for a specific anchor
    pub fn get_anchor_statistics(&self, anchor_id: u16) -> Option<AnchorStatistics> {
        self.anchor_history.get(&anchor_id).map(|history| {
            let avg_quality = if history.quality_history.is_empty() {
                0.0
            } else {
                history.quality_history.iter().map(|&q| q as f64).sum::<f64>() / history.quality_history.len() as f64
            };

            AnchorStatistics {
                anchor_id,
                message_count: history.message_count,
                last_seen: history.last_timestamp,
                average_quality: avg_quality,
                last_position: history.last_position.clone(),
            }
        })
    }
}

/// Statistics for an individual anchor
#[derive(Debug, Clone)]
pub struct AnchorStatistics {
    pub anchor_id: u16,
    pub message_count: u32,
    pub last_seen: u64,
    pub average_quality: f64,
    pub last_position: GeodeticPosition,
}

impl Default for DataValidator {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::message_parser::MessageVersion;

    fn create_test_message(anchor_id: u16, timestamp: u64, lat: f64, lon: f64, depth: f64, quality: u8) -> AnchorMessage {
        AnchorMessage {
            anchor_id,
            timestamp_ms: timestamp,
            position: GeodeticPosition {
                latitude: lat,
                longitude: lon,
                depth,
            },
            signal_quality: quality,
            message_sequence: 1,
            version: MessageVersion::V1,
        }
    }

    fn get_current_time() -> u64 {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis() as u64
    }

    #[test]
    fn test_valid_messages() {
        let mut validator = DataValidator::new();
        let current_time = get_current_time();
        
        let messages = vec![
            create_test_message(1, current_time - 1000, 32.0, -117.0, 10.0, 100),
            create_test_message(2, current_time - 999, 32.001, -117.001, 10.0, 95),
            create_test_message(3, current_time - 998, 32.002, -117.002, 10.0, 90),
        ];

        let result = validator.validate_messages(messages);
        assert_eq!(result.valid_messages.len(), 3);
        assert!(result.warnings.is_empty());
        assert!(result.rejected_messages.is_empty());
    }

    #[test]
    fn test_stale_message_rejection() {
        let mut validator = DataValidator::with_config(ValidationConfig {
            max_message_age_ms: 1000,
            ..Default::default()
        });

        let current_time = validator.get_current_time();
        let old_timestamp = current_time - 2000; // 2 seconds old

        let messages = vec![
            create_test_message(1, old_timestamp, 32.0, -117.0, 10.0, 100),
        ];

        let result = validator.validate_messages(messages);
        assert_eq!(result.valid_messages.len(), 0);
        assert_eq!(result.rejected_messages.len(), 1);
        
        if let ValidationError::StaleMessage { anchor_id, age_ms } = &result.rejected_messages[0].1 {
            assert_eq!(*anchor_id, 1);
            assert!(*age_ms >= 2000);
        } else {
            panic!("Expected StaleMessage error");
        }
    }

    #[test]
    fn test_poor_signal_quality_rejection() {
        let mut validator = DataValidator::with_config(ValidationConfig {
            min_signal_quality: 80,
            ..Default::default()
        });
        let current_time = get_current_time();

        let messages = vec![
            create_test_message(1, current_time - 1000, 32.0, -117.0, 10.0, 50), // Below threshold
        ];

        let result = validator.validate_messages(messages);
        assert_eq!(result.valid_messages.len(), 0);
        assert_eq!(result.rejected_messages.len(), 1);
        
        if let ValidationError::PoorSignalQuality { anchor_id, quality } = &result.rejected_messages[0].1 {
            assert_eq!(*anchor_id, 1);
            assert_eq!(*quality, 50);
        } else {
            panic!("Expected PoorSignalQuality error");
        }
    }

    #[test]
    fn test_position_jump_detection() {
        let mut validator = DataValidator::with_config(ValidationConfig {
            max_position_jump_m: 50.0,
            ..Default::default()
        });
        let current_time = get_current_time();

        // First message to establish history
        let messages1 = vec![
            create_test_message(1, current_time - 2000, 32.0, -117.0, 10.0, 100),
        ];
        validator.validate_messages(messages1);

        // Second message with large position jump
        let messages2 = vec![
            create_test_message(1, current_time - 1000, 32.1, -117.1, 10.0, 100), // ~15km jump
        ];

        let result = validator.validate_messages(messages2);
        assert_eq!(result.valid_messages.len(), 0);
        assert_eq!(result.rejected_messages.len(), 1);
        
        if let ValidationError::PositionJumpDetected { anchor_id, distance_m } = &result.rejected_messages[0].1 {
            assert_eq!(*anchor_id, 1);
            assert!(*distance_m > 50.0);
        } else {
            panic!("Expected PositionJumpDetected error");
        }
    }

    #[test]
    fn test_geometry_quality_assessment() {
        let mut validator = DataValidator::new();
        let current_time = get_current_time();
        
        // Good geometry - well distributed anchors
        let good_messages = vec![
            create_test_message(1, current_time - 1000, 32.0, -117.0, 0.0, 100),
            create_test_message(2, current_time - 999, 32.01, -117.0, 0.0, 100),
            create_test_message(3, current_time - 998, 32.005, -117.01, 0.0, 100),
            create_test_message(4, current_time - 997, 32.005, -117.005, 10.0, 100),
        ];

        let result = validator.validate_messages(good_messages);
        assert!(matches!(result.geometry_quality, GeometryQuality::Good | GeometryQuality::Excellent));

        // Poor geometry - anchors too close (within 10m minimum distance)
        validator.clear_history();
        let poor_messages = vec![
            create_test_message(1, current_time - 1000, 32.0, -117.0, 0.0, 100),
            create_test_message(2, current_time - 999, 32.00001, -117.00001, 0.0, 100), // ~1m apart
            create_test_message(3, current_time - 998, 32.00002, -117.00002, 0.0, 100), // ~2m apart
        ];

        let result = validator.validate_messages(poor_messages);
        assert_eq!(result.geometry_quality, GeometryQuality::Poor);
    }
}