//! Common API types and data structures

use crate::core::{Position, Anchor};
use crate::hardware::{TransceiverConfig, CommError};
use serde::{Deserialize, Serialize};

/// Result type for API operations
pub type ApiResult<T> = Result<T, ApiError>;

/// API error types
#[derive(Debug, Clone, PartialEq)]
pub enum ApiError {
    /// Insufficient anchors for positioning
    InsufficientAnchors { available: u8, required: u8 },
    /// Poor anchor geometry
    DegenerateGeometry { condition_number: f32 },
    /// Stale anchor data
    StaleData { oldest_anchor_age_ms: u32 },
    /// Computation failure
    ComputationFailure { details: String },
    /// Hardware communication error
    HardwareError { error: CommError },
    /// Invalid configuration
    ConfigurationError { parameter: String, value: String },
    /// System not initialized
    NotInitialized,
    /// Operation timeout
    Timeout { timeout_ms: u32 },
    /// Invalid request parameters
    InvalidRequest { reason: String },
}

impl From<CommError> for ApiError {
    fn from(error: CommError) -> Self {
        ApiError::HardwareError { error }
    }
}

/// Position request parameters
#[derive(Debug, Clone)]
pub struct PositionRequest {
    /// Maximum time to wait for position (milliseconds)
    pub timeout_ms: u32,
    /// Minimum number of anchors required
    pub min_anchors: u8,
    /// Maximum anchor age (milliseconds)
    pub max_anchor_age_ms: u32,
    /// Required accuracy threshold (meters)
    pub accuracy_threshold_m: f32,
    /// Output format preference
    pub output_format: OutputFormat,
}

impl Default for PositionRequest {
    fn default() -> Self {
        Self {
            timeout_ms: 1000,
            min_anchors: 3,
            max_anchor_age_ms: 5000,
            accuracy_threshold_m: 10.0,
            output_format: OutputFormat::Geodetic,
        }
    }
}

/// Position response with accuracy and quality information
#[derive(Debug, Clone, PartialEq)]
pub struct PositionResponse {
    /// Calculated position
    pub position: Position,
    /// Local position (East-North-Down) if available
    pub local_position: Option<LocalPosition>,
    /// Estimated accuracy (meters)
    pub accuracy_estimate: f32,
    /// Position timestamp (milliseconds since epoch)
    pub timestamp_ms: u64,
    /// Number of anchors used
    pub anchor_count: u8,
    /// Geometry quality indicator
    pub geometry_quality: GeometryQuality,
    /// Computation time (microseconds)
    pub computation_time_us: u32,
    /// Sequence number for tracking
    pub sequence_number: u32,
}

/// Local position in East-North-Down coordinates
#[derive(Debug, Clone, PartialEq)]
pub struct LocalPosition {
    /// East coordinate (meters)
    pub east_m: f32,
    /// North coordinate (meters)
    pub north_m: f32,
    /// Down coordinate (meters, positive downward)
    pub down_m: f32,
}

/// Geometry quality assessment
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum GeometryQuality {
    /// Excellent geometry (DOP < 2.0)
    Excellent,
    /// Good geometry (DOP < 5.0)
    Good,
    /// Acceptable geometry (DOP < 10.0)
    Acceptable,
    /// Poor geometry (DOP >= 10.0)
    Poor,
    /// Degenerate geometry (nearly singular)
    Degenerate,
}

impl GeometryQuality {
    pub fn from_dop(dop: f32) -> Self {
        if dop < 2.0 {
            GeometryQuality::Excellent
        } else if dop < 5.0 {
            GeometryQuality::Good
        } else if dop < 10.0 {
            GeometryQuality::Acceptable
        } else if dop.is_finite() {
            GeometryQuality::Poor
        } else {
            GeometryQuality::Degenerate
        }
    }
    
    pub fn is_acceptable(&self) -> bool {
        matches!(self, GeometryQuality::Excellent | GeometryQuality::Good | GeometryQuality::Acceptable)
    }
}

/// Output format options
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum OutputFormat {
    /// Geodetic coordinates (lat/lon/depth)
    Geodetic,
    /// Local tangent plane (East-North-Down)
    Local,
    /// Both geodetic and local
    Both,
    /// Compact binary format
    Compact,
}

/// System state information
#[derive(Debug, Clone)]
pub struct SystemState {
    /// System initialization status
    pub initialized: bool,
    /// Number of active transceivers
    pub active_transceivers: u8,
    /// Last successful position timestamp
    pub last_position_time: Option<u64>,
    /// Total positions calculated
    pub positions_calculated: u32,
    /// Total errors encountered
    pub error_count: u32,
    /// Average computation time (microseconds)
    pub avg_computation_time_us: u32,
    /// Memory usage (bytes)
    pub memory_usage_bytes: u32,
    /// System uptime (milliseconds)
    pub uptime_ms: u64,
}

impl Default for SystemState {
    fn default() -> Self {
        Self {
            initialized: false,
            active_transceivers: 0,
            last_position_time: None,
            positions_calculated: 0,
            error_count: 0,
            avg_computation_time_us: 0,
            memory_usage_bytes: 0,
            uptime_ms: 0,
        }
    }
}

/// API configuration parameters
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ApiConfig {
    /// Default timeout for position requests (milliseconds)
    pub default_timeout_ms: u32,
    /// Default minimum anchors required
    pub default_min_anchors: u8,
    /// Default maximum anchor age (milliseconds)
    pub default_max_anchor_age_ms: u32,
    /// Default accuracy threshold (meters)
    pub default_accuracy_threshold_m: f32,
    /// Enable automatic error recovery
    pub enable_auto_recovery: bool,
    /// Maximum queue size for non-blocking operations
    pub max_queue_size: usize,
    /// Enable performance monitoring
    pub enable_monitoring: bool,
    /// Log level for debugging
    pub log_level: LogLevel,
}

impl Default for ApiConfig {
    fn default() -> Self {
        Self {
            default_timeout_ms: 1000,
            default_min_anchors: 3,
            default_max_anchor_age_ms: 5000,
            default_accuracy_threshold_m: 10.0,
            enable_auto_recovery: true,
            max_queue_size: 16,
            enable_monitoring: true,
            log_level: LogLevel::Info,
        }
    }
}

/// Logging levels
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum LogLevel {
    /// No logging
    None,
    /// Error messages only
    Error,
    /// Error and warning messages
    Warn,
    /// Error, warning, and info messages
    Info,
    /// All messages including debug
    Debug,
}

/// Compact binary position format for embedded systems
#[repr(C, packed)]
#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub struct CompactPosition {
    /// Latitude in micro-degrees (lat * 1e6)
    pub lat_micro_deg: i32,
    /// Longitude in micro-degrees (lon * 1e6)
    pub lon_micro_deg: i32,
    /// Depth in millimeters
    pub depth_mm: u16,
    /// Accuracy estimate in centimeters
    pub accuracy_cm: u16,
    /// Timestamp (seconds since epoch)
    pub timestamp_sec: u32,
    /// Anchor count and quality flags
    pub flags: u8,
}

impl CompactPosition {
    /// Create compact position from standard position
    pub fn from_position(pos: &PositionResponse) -> Self {
        let flags = (pos.anchor_count & 0x0F) | ((pos.geometry_quality as u8) << 4);
        
        Self {
            lat_micro_deg: (pos.position.lat * 1e6) as i32,
            lon_micro_deg: (pos.position.lon * 1e6) as i32,
            depth_mm: (pos.position.depth * 1000.0) as u16,
            accuracy_cm: (pos.accuracy_estimate * 100.0) as u16,
            timestamp_sec: (pos.timestamp_ms / 1000) as u32,
            flags,
        }
    }
    
    /// Convert to standard position
    pub fn to_position(&self) -> Position {
        Position {
            lat: self.lat_micro_deg as f64 / 1e6,
            lon: self.lon_micro_deg as f64 / 1e6,
            depth: self.depth_mm as f64 / 1000.0,
        }
    }
    
    /// Get anchor count from flags
    pub fn anchor_count(&self) -> u8 {
        self.flags & 0x0F
    }
    
    /// Get geometry quality from flags
    pub fn geometry_quality(&self) -> GeometryQuality {
        match (self.flags >> 4) & 0x0F {
            0 => GeometryQuality::Excellent,
            1 => GeometryQuality::Good,
            2 => GeometryQuality::Acceptable,
            3 => GeometryQuality::Poor,
            _ => GeometryQuality::Degenerate,
        }
    }
}