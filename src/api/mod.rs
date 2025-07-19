//! Embedded-optimized API and interfaces
//! 
//! This module provides microcontroller-friendly APIs for the underwater positioning system.
//! It includes blocking, non-blocking, callback-based, and C-compatible interfaces.

pub mod blocking;
pub mod nonblocking;
pub mod callback;
pub mod c_api;
pub mod types;
pub mod formatting;

// Re-export commonly used API types
pub use types::{
    ApiResult, ApiError, PositionRequest, PositionResponse, 
    SystemState, ApiConfig, OutputFormat
};
pub use blocking::BlockingPositioningApi;
pub use nonblocking::NonBlockingPositioningApi;
pub use callback::{CallbackPositioningApi, PositionCallback, EventCallback};
pub use formatting::{
    PositionFormatter, FormattedPosition, TextFormatter, JsonFormatter, CsvFormatter,
    PositionData, QualityIndicators, TimingInfo, DiagnosticInfo
};