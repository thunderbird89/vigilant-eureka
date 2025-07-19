//! Underwater Positioning System
//! 
//! A high-precision underwater positioning system using acoustic trilateration
//! with advanced algorithms for sub-meter accuracy.

pub mod core;
pub mod algorithms;
pub mod processing;
pub mod validation;
pub mod utils;
pub mod hardware;
pub mod api;

// Re-export commonly used types
pub use core::{Position, Anchor, SPEED_OF_SOUND_WATER};
pub use algorithms::trilateration::AdvancedTrilateration;
pub use algorithms::embedded_trilateration::{EmbeddedTrilateration, FixedVector3, FixedMatrix3x3};
pub use algorithms::embedded_coordinates::{EmbeddedCoordinateManager, CoordinateSystem, ReferencePoint, FixedCoordinate, CoordinateValidator};
pub use processing::kalman::PositionKalmanFilter;
pub use validation::accuracy::AccuracyValidator;
pub use hardware::{TransceiverInterface, TransceiverStatus, TransceiverConfig, RawMessage, CommError, CommResult};
pub use api::{
    BlockingPositioningApi, NonBlockingPositioningApi, CallbackPositioningApi,
    ApiResult, ApiError, PositionRequest, PositionResponse, SystemState, ApiConfig,
    OutputFormat, PositionFormatter, FormattedPosition, TextFormatter, JsonFormatter, CsvFormatter
};
pub use api::types::{GeometryQuality, LocalPosition, CompactPosition};