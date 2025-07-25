use clap::{Parser, Subcommand, ValueEnum};
use uuid::Uuid;
use std::path::PathBuf;
use crate::{MovementPattern, ScenarioType, ExportFormat};

#[derive(Parser)]
#[command(name = "beacon-emulator")]
#[command(about = "Virtual beacon emulator for underwater positioning system testing")]
#[command(version = "0.1.0")]
pub struct Cli {
    /// Virtual communication channel (shared with receivers)
    #[arg(short, long, default_value = "default")]
    pub channel: String,
    
    /// Log level (trace, debug, info, warn, error)
    #[arg(short, long, default_value = "info")]
    pub log_level: String,
    
    #[command(subcommand)]
    pub command: EmulatorCommand,
}

#[derive(Subcommand)]
pub enum EmulatorCommand {
    /// Create a new virtual beacon
    Create {
        /// Beacon ID (UUID or auto-generate)
        #[arg(short, long)]
        id: Option<Uuid>,
        
        /// Latitude in degrees
        #[arg(long)]
        lat: f64,
        
        /// Longitude in degrees  
        #[arg(long)]
        lon: f64,
        
        /// Depth in meters (positive down)
        #[arg(long, default_value = "0.0")]
        depth: f64,
        
        /// Configuration file to initialize beacon
        #[arg(short, long)]
        config: Option<PathBuf>,
        
        /// Transmission interval in milliseconds
        #[arg(long, default_value = "5000")]
        interval: u32,
        
        /// Message version (V1, V2, V3)
        #[arg(long, default_value = "v3")]
        version: MessageVersion,
        
        /// Movement pattern
        #[arg(long, default_value = "stationary")]
        movement: MovementPattern,
    },
    
    /// List active virtual beacons
    List {
        /// Show detailed information
        #[arg(short, long)]
        detailed: bool,
    },
    
    /// Stop a virtual beacon
    Stop {
        /// Beacon ID to stop
        id: Uuid,
    },
    
    /// Stop all virtual beacons
    StopAll,
    
    /// Update beacon configuration
    Update {
        /// Beacon ID to update
        id: Uuid,
        
        /// New position (lat,lon,depth)
        #[arg(long)]
        position: Option<String>,
        
        /// New transmission interval
        #[arg(long)]
        interval: Option<u32>,
        
        /// Movement pattern
        #[arg(long)]
        movement: Option<MovementPattern>,
    },
    
    /// Create predefined test scenario
    Scenario {
        /// Scenario type
        scenario_type: ScenarioType,
        
        /// Number of beacons
        #[arg(short, long, default_value = "4")]
        count: usize,
        
        /// Spacing between beacons in meters
        #[arg(short, long, default_value = "100.0")]
        spacing: f64,
        
        /// Center position (lat,lon,depth)
        #[arg(long, default_value = "32.123,45.476,10.0")]
        center: String,
    },
    
    /// Monitor virtual beacon activity
    Monitor {
        /// Specific beacon ID to monitor
        #[arg(short, long)]
        beacon: Option<Uuid>,
        
        /// Update interval in seconds
        #[arg(long, default_value = "1")]
        interval: u64,
    },
    
    /// Export beacon activity logs
    Export {
        /// Output file path
        #[arg(short, long)]
        output: PathBuf,
        
        /// Export format
        #[arg(short, long, default_value = "json")]
        format: ExportFormat,
        
        /// Time range in seconds (from now)
        #[arg(long, default_value = "3600")]
        duration: u64,
    },
}

#[derive(Clone, Debug, ValueEnum)]
pub enum MessageVersion {
    V1,
    V2,
    V3,
}

impl Default for MessageVersion {
    fn default() -> Self {
        Self::V3
    }
}