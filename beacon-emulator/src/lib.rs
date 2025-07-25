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

// Re-export main types for convenience
pub use error::EmulatorError;
pub use emulator::EmulatorManager;
pub use virtual_beacon::{VirtualBeacon, VirtualBeaconStatus, VirtualBeaconStats};
pub use virtual_channel::{VirtualCommunicationSpace, VirtualChannel, VirtualMessage};
pub use movement::MovementPattern;
pub use scenario::ScenarioType;
pub use export::ExportFormat;