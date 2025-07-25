use clap::ValueEnum;
use serde::{Serialize, Deserialize};
use std::str::FromStr;
use crate::EmulatorError;

/// Predefined test scenario types for beacon arrangements
#[derive(Debug, Clone, Serialize, Deserialize, ValueEnum)]
#[serde(rename_all = "lowercase")]
pub enum ScenarioType {
    /// Triangular arrangement of beacons
    Triangle,
    /// Square arrangement of beacons
    Square,
    /// Linear arrangement of beacons
    Line,
    /// Grid arrangement of beacons
    Grid,
}

impl FromStr for ScenarioType {
    type Err = EmulatorError;
    
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "triangle" => Ok(Self::Triangle),
            "square" => Ok(Self::Square),
            "line" => Ok(Self::Line),
            "grid" => Ok(Self::Grid),
            _ => Err(EmulatorError::InvalidScenario(
                format!("Unknown scenario type: {}", s)
            )),
        }
    }
}

impl std::fmt::Display for ScenarioType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Triangle => write!(f, "triangle"),
            Self::Square => write!(f, "square"),
            Self::Line => write!(f, "line"),
            Self::Grid => write!(f, "grid"),
        }
    }
}