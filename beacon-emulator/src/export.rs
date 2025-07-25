use clap::ValueEnum;
use serde::{Serialize, Deserialize};
use std::str::FromStr;
use crate::EmulatorError;

/// Export formats for beacon activity logs
#[derive(Debug, Clone, Serialize, Deserialize, ValueEnum)]
#[serde(rename_all = "lowercase")]
pub enum ExportFormat {
    /// JSON format
    Json,
    /// CSV format
    Csv,
}

impl Default for ExportFormat {
    fn default() -> Self {
        Self::Json
    }
}

impl FromStr for ExportFormat {
    type Err = EmulatorError;
    
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "json" => Ok(Self::Json),
            "csv" => Ok(Self::Csv),
            _ => Err(EmulatorError::ExportError(
                format!("Unknown export format: {}", s)
            )),
        }
    }
}

impl std::fmt::Display for ExportFormat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Json => write!(f, "json"),
            Self::Csv => write!(f, "csv"),
        }
    }
}