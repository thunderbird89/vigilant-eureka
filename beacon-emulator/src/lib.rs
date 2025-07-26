//! Beacon Emulator Library
//! 
//! This library provides virtual beacon emulation capabilities for testing
//! the underwater positioning system without requiring physical hardware.

pub mod cli;
pub mod error;
pub mod emulator;
pub mod virtual_beacon;
pub mod virtual_channel;
pub mod movement;
pub mod scenario;
pub mod export;
pub mod daemon_protocol;
pub mod daemon_server;
pub mod ipc_server;
pub mod monitor;

#[cfg(test)]
pub mod test_data_models;

// Re-export main types for convenience
pub use error::EmulatorError;
pub use emulator::{EmulatorManager, EmulatorManagerStats};
pub use virtual_beacon::{VirtualBeacon, VirtualBeaconStatus, VirtualBeaconStats};
pub use virtual_channel::{VirtualCommunicationSpace, VirtualChannel, VirtualChannelStats};
pub use shared_positioning::VirtualMessage;
pub use movement::{MovementPattern, MovementCoordinateTransformer, MovementPatternValidator};
pub use scenario::ScenarioType;
pub use export::ExportFormat;
pub use ipc_server::IpcServer;
pub use monitor::{BeaconMonitor, MonitorConfig, BeaconHealthIndicator, HealthStatus};

// Integration tests
#[cfg(test)]
mod scenario_integration_test;