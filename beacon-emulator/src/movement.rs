use clap::ValueEnum;
use serde::{Serialize, Deserialize};
use std::str::FromStr;
use crate::EmulatorError;

/// Movement patterns for virtual beacons
#[derive(Debug, Clone, Serialize, Deserialize, ValueEnum)]
#[serde(rename_all = "lowercase")]
pub enum MovementPattern {
    /// Beacon remains stationary
    Stationary,
    /// Linear movement with specified speed and bearing
    Linear,
    /// Circular movement around initial position
    Circular,
    /// Random walk movement
    Random,
}

impl Default for MovementPattern {
    fn default() -> Self {
        Self::Stationary
    }
}

impl FromStr for MovementPattern {
    type Err = EmulatorError;
    
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "stationary" => Ok(Self::Stationary),
            "linear" => Ok(Self::Linear),
            "circular" => Ok(Self::Circular),
            "random" => Ok(Self::Random),
            _ => Err(EmulatorError::MovementError(
                format!("Unknown movement pattern: {}", s)
            )),
        }
    }
}

impl std::fmt::Display for MovementPattern {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Stationary => write!(f, "stationary"),
            Self::Linear => write!(f, "linear"),
            Self::Circular => write!(f, "circular"),
            Self::Random => write!(f, "random"),
        }
    }
}