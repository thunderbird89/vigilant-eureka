use thiserror::Error;
use uuid::Uuid;

/// Comprehensive error types for the beacon emulator
#[derive(Error, Debug)]
pub enum EmulatorError {
    #[error("Beacon {0} already exists")]
    BeaconExists(Uuid),
    
    #[error("Beacon {0} not found")]
    BeaconNotFound(Uuid),
    
    #[error("Message build error: {0}")]
    MessageBuildError(String),
    
    #[error("Channel error: {0}")]
    ChannelError(String),
    
    #[error("Configuration error: {0}")]
    ConfigError(String),
    
    #[error("IO error: {0}")]
    IoError(#[from] std::io::Error),
    
    #[error("Serialization error: {0}")]
    SerializationError(#[from] serde_json::Error),
    
    #[error("TOML parsing error: {0}")]
    TomlError(#[from] toml::de::Error),
    
    #[error("Invalid scenario parameters: {0}")]
    InvalidScenario(String),
    
    #[error("Movement pattern error: {0}")]
    MovementError(String),
    
    #[error("Task join error: {0}")]
    TaskJoinError(#[from] tokio::task::JoinError),
    
    #[error("UUID parsing error: {0}")]
    UuidError(#[from] uuid::Error),
    
    #[error("Time error: {0}")]
    TimeError(String),
    
    #[error("Export error: {0}")]
    ExportError(String),
}