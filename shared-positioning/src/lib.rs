// Shared positioning library for underwater positioning system
// Contains common components used by both beacon and receiver systems

pub mod message_parser;
pub mod transceiver_interface;
pub mod coordinate_system;
pub mod error_handling;
pub mod config;
pub mod gps_manager;
pub mod power_manager;
pub mod communication_manager;

// Re-export commonly used types for convenience
pub use message_parser::{
    MessageParser, MessageBuilder, AnchorMessage, GeodeticPosition, 
    MessageParseError, MessageVersion, RawMessage, MessageValidator,
    ValidationResult, ValidationStats
};
pub use transceiver_interface::{
    TransceiverInterface, CommError, TransceiverStatus, TransceiverConfig, 
    PowerMode, MockTransceiver, SerialTransceiver, TransmissionStatus,
    TransmissionStats, EnvironmentalConditions, ConnectionStats, BaseTransceiver
};
pub use coordinate_system::{
    CoordinateSystemManager, MultiCoordinateSystemManager, CoordinateSystemType,
    UTMCoordinate, Position
};
pub use error_handling::{
    PositioningError, ErrorSeverity, ErrorContext, ErrorRecoveryManager,
    ErrorLogger, RecoveryStrategy, ConsoleLogHandler, GeometryIssue, 
    CriticalFailureType, TimingIssue
};
pub use config::{
    SystemConfig, AnchorConfig, PositioningConfig, RuntimeParameterManager
};
pub use gps_manager::{
    GpsManager, GpsPosition, GpsConfig, GpsError, GpsStatus, 
    BasicGpsManager, MockGpsManager
};
pub use power_manager::{
    PowerManager, PowerError, BatteryStatus, BatteryHealth, ChargingStatus,
    PowerOperationMode, PowerConfig, PowerStats, BasicPowerManager, MockPowerManager
};
pub use communication_manager::{
    CommunicationManager, CommunicationConfig, StatusReport, ConfigUpdate,
    SystemHealth, ErrorLogEntry, BasicCommunicationManager, MockCommunicationManager
};