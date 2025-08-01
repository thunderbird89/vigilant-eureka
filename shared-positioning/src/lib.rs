// Shared positioning library for underwater positioning system
// Contains common components used by both beacon and receiver systems

pub mod message_parser;
pub mod transceiver_interface;
pub mod coordinate_system;
pub mod error_handling;
pub mod config;
pub mod beacon_config;
pub mod gps_manager;
pub mod power_manager;
pub mod communication_manager;
pub mod transmission_manager;
pub mod environmental_monitor;
pub mod hardware_monitor;
pub mod reliability_monitor;
pub mod virtual_communication;
pub mod watchdog_timer;
pub mod config_backup;
pub mod emergency_beacon;

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
    CriticalFailureType, TimingIssue, ConfigError, Result,
    // Beacon-specific error handling exports
    BeaconError, BeaconErrorContext, BeaconErrorLogEntry, BeaconLogOutputHandler,
    DiagnosticSystemManager, DiagnosticReport, SystemHealthMetrics, ErrorStatistics,
    PerformanceMetrics, EnvironmentalMetrics, HardwareStatusReport, RecoveryEffectivenessReport,
    HealthAlert, StructuredFileLogHandler, ErrorTrendAnalyzer,
    // Error type enums
    GpsErrorType, PowerErrorType, CommunicationErrorType, TransmissionErrorType,
    ConfigurationErrorType, SystemErrorType, HardwareComponent, HardwareFaultType,
    // Status snapshots
    BatteryStatusSnapshot, BatteryHealth as BeaconBatteryHealth, BeaconSystemState, ResourceUsageSnapshot,
    GpsStatusSnapshot, CommunicationStatusSnapshot, TransmissionStatusSnapshot,
    // Power and charging enums
    PowerMode as BeaconPowerMode, ChargingStatus as BeaconChargingStatus,
    // Health and diagnostic types
    MemoryHealthMetrics, PowerHealthMetrics, CommunicationHealthMetrics, GpsHealthMetrics,
    ComponentStatus, HardwareFaultRecord, MaintenanceRecommendation, MaintenanceType,
    MaintenanceUrgency, FailedRecoveryRecord, PowerConsumptionTrend, ThermalStatus,
    SignalQualityTrend, SatelliteVisibility, PositionStability, ErrorTrendAnalysis,
    TrendDirection, SeasonalPattern, HealthThresholds
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
    SystemHealth, ErrorLogEntry, TransmissionStats as CommTransmissionStats, 
    ErrorSeverity as CommErrorSeverity, BasicCommunicationManager, MockCommunicationManager
};
pub use transmission_manager::{
    TransmissionManager, TransmissionConfig, TransmissionError, TransmissionStatistics,
    MessageVersion as TransmissionMessageVersion, TransmissionPriority, ScheduledTransmission
};
pub use beacon_config::{
    BeaconConfig, BeaconConfigManager, BeaconConfigBackup, ConfigMigrator, ConfigTemplates,
    TransmissionConfig as BeaconTransmissionConfig, GpsConfig as BeaconGpsConfig,
    PowerConfig as BeaconPowerConfig, CommunicationConfig as BeaconCommunicationConfig,
    EmergencyConfig, HardwareConfig, MessageVersion as BeaconMessageVersion,
    ConfigExportFormat, ConfigValidator
};
pub use environmental_monitor::{
    EnvironmentalMonitor, ExtendedEnvironmentalConditions, EnvironmentalThresholds,
    EnvironmentalError, AdaptationAction, EnvironmentalStats, MeasurementQuality,
    CalibrationStatus, ExtremeSeverity, SeaState, EnvironmentalDataPoint
};
pub use hardware_monitor::{
    HardwareMonitor, HardwareMonitorConfig, HardwareMonitorStats, DiagnosticResult,
    ComponentHealth, RecommendedAction, RecoveryStrategy as HardwareRecoveryStrategy, HardwareFaultError
};
pub use reliability_monitor::{
    ReliabilityMonitor, ReliabilityMetrics, ComponentReliability, ReliabilityThresholds,
    ReliabilityReport, FailureEvent, MaintenanceRecommendation as ReliabilityMaintenanceRecommendation,
    ReliabilityError, HealthTrend, ThresholdViolation as ReliabilityThresholdViolation
};
pub use virtual_communication::{
    VirtualMessage, IpcMessage
};
pub use watchdog_timer::{
    WatchdogTimer, WatchdogConfig, WatchdogEvent, WatchdogStats, ComponentWatchdogHealth, WatchdogError
};
pub use config_backup::{
    ConfigBackupManager, ConfigBackupError, BackupMetadata, BackupType, ConfigBackup,
    ValidationResults, BackupManagerConfig, BackupStatistics
};
pub use emergency_beacon::{
    EmergencyBeaconSystem, EmergencyBeaconError, EmergencyType, EmergencySeverity,
    EmergencyBeaconId, EmergencyContact, BeaconCapabilities, BeaconCertification,
    DistressSignal, EmergencyPriority, AssistanceType, EmergencyBeaconConfig,
    EmergencyTransmissionStats
};