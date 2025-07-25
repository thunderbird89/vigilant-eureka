use clap::{Parser, Subcommand, ValueEnum};
use uuid::Uuid;
use std::path::PathBuf;
use std::str::FromStr;
use shared_positioning::GeodeticPosition;
use crate::{ScenarioType, ExportFormat, MovementPattern};

#[derive(Parser)]
#[command(name = "beacon-emulator")]
#[command(about = "Virtual beacon emulator for underwater positioning system testing")]
#[command(version = "0.1.0")]
#[command(long_about = r#"
Virtual beacon emulator for underwater positioning system testing.

This tool creates and manages virtual beacons that transmit positioning messages
using the same protocols as real beacons. Virtual beacons can be configured with
different positions, movement patterns, and transmission parameters to test
receiver systems and positioning algorithms.

EXAMPLES:
    # Create a stationary beacon at specific coordinates
    beacon-emulator create --lat 32.123 --lon 45.476 --depth 10.0

    # Create a beacon with linear movement
    beacon-emulator create --lat 32.0 --lon 45.0 --movement linear:1.5:45

    # Create a triangular test scenario with 3 beacons
    beacon-emulator scenario triangle --count 3 --spacing 100

    # List all active beacons with detailed information
    beacon-emulator list --detailed

    # Monitor beacon activity in real-time
    beacon-emulator monitor --interval 2

    # Export logs to CSV format
    beacon-emulator export --output logs.csv --format csv --duration 1800

For more information about movement patterns and configuration options,
use the --help flag with specific commands.
"#)]
pub struct Cli {
    /// Virtual communication channel (shared with receivers)
    #[arg(short, long, default_value = "default", help = "Communication channel name")]
    pub channel: String,
    
    /// Log level (trace, debug, info, warn, error)
    #[arg(short, long, default_value = "info", help = "Logging verbosity level")]
    pub log_level: String,
    
    #[command(subcommand)]
    pub command: EmulatorCommand,
}

#[derive(Subcommand)]
pub enum EmulatorCommand {
    /// Create a new virtual beacon
    #[command(long_about = r#"
Create a new virtual beacon with specified position and configuration.

The beacon will be created but not started automatically. Use the returned
beacon ID with other commands to manage the beacon.

MOVEMENT PATTERNS:
    stationary                    - Beacon remains at fixed position
    linear:speed:bearing         - Linear movement (speed in m/s, bearing 0-360°)
    circular:radius:period       - Circular movement (radius in m, period in s)
    random:max_speed            - Random walk (max speed in m/s)

EXAMPLES:
    # Create stationary beacon
    beacon-emulator create --lat 32.123 --lon 45.476 --depth 10.0

    # Create beacon with custom ID and config file
    beacon-emulator create --id 550e8400-e29b-41d4-a716-446655440000 \
                          --lat 32.0 --lon 45.0 --config beacon.toml

    # Create beacon with linear movement (1.5 m/s northeast)
    beacon-emulator create --lat 32.0 --lon 45.0 --movement linear:1.5:45

    # Create beacon with circular movement (100m radius, 60s period)
    beacon-emulator create --lat 32.0 --lon 45.0 --movement circular:100:60
"#)]
    Create {
        /// Beacon ID (UUID or auto-generate if not specified)
        #[arg(short, long, help = "Specific UUID for the beacon (auto-generated if not provided)")]
        id: Option<Uuid>,
        
        /// Latitude in degrees (-90 to 90)
        #[arg(long, help = "Latitude coordinate in decimal degrees", value_parser = validate_latitude)]
        lat: f64,
        
        /// Longitude in degrees (-180 to 180)
        #[arg(long, help = "Longitude coordinate in decimal degrees", value_parser = validate_longitude)]
        lon: f64,
        
        /// Depth in meters (positive down, 0 to 11000)
        #[arg(long, default_value = "0.0", help = "Depth below surface in meters", value_parser = validate_depth)]
        depth: f64,
        
        /// Configuration file to initialize beacon
        #[arg(short, long, help = "TOML configuration file path")]
        config: Option<PathBuf>,
        
        /// Transmission interval in milliseconds (100 to 300000)
        #[arg(long, default_value = "5000", help = "Message transmission interval", value_parser = validate_interval)]
        interval: u32,
        
        /// Message version (V1, V2, V3)
        #[arg(long, default_value = "v3", help = "Message protocol version")]
        version: MessageVersion,
        
        /// Movement pattern specification
        #[arg(long, default_value = "stationary", help = "Movement pattern (see examples above)", value_parser = validate_movement_pattern)]
        movement: MovementPattern,
        
        /// Start the beacon immediately after creation
        #[arg(long, help = "Start beacon transmission immediately")]
        start: bool,
    },
    
    /// List active virtual beacons
    #[command(long_about = r#"
List all virtual beacons with their current status and configuration.

EXAMPLES:
    # List all beacons with basic information
    beacon-emulator list

    # List all beacons with detailed status
    beacon-emulator list --detailed

    # List only running beacons
    beacon-emulator list --running-only
"#)]
    List {
        /// Show detailed information including statistics and configuration
        #[arg(short, long, help = "Show detailed beacon information")]
        detailed: bool,
        
        /// Show only running beacons
        #[arg(short, long, help = "Show only currently running beacons")]
        running_only: bool,
        
        /// Output format for the list
        #[arg(long, default_value = "table", help = "Output format")]
        format: ListFormat,
    },
    
    /// Stop a virtual beacon
    #[command(long_about = r#"
Stop a specific virtual beacon by its UUID.

The beacon will stop transmitting but remain in the registry.
Use 'remove' command to completely remove a stopped beacon.

EXAMPLES:
    # Stop a specific beacon
    beacon-emulator stop 550e8400-e29b-41d4-a716-446655440000
"#)]
    Stop {
        /// Beacon ID to stop
        #[arg(help = "UUID of the beacon to stop")]
        id: Uuid,
        
        /// Remove the beacon after stopping
        #[arg(long, help = "Remove beacon from registry after stopping")]
        remove: bool,
    },
    
    /// Stop all virtual beacons
    #[command(long_about = r#"
Stop all currently running virtual beacons.

This command will attempt to gracefully stop all beacons.
Failed stops will be reported but won't prevent other beacons from stopping.

EXAMPLES:
    # Stop all beacons
    beacon-emulator stop-all

    # Stop and remove all beacons
    beacon-emulator stop-all --remove
"#)]
    StopAll {
        /// Remove all beacons after stopping
        #[arg(long, help = "Remove all beacons from registry after stopping")]
        remove: bool,
    },
    
    /// Update beacon configuration
    #[command(long_about = r#"
Update configuration of an existing virtual beacon.

The beacon can be updated while running. Position and movement pattern
changes take effect immediately. Interval changes require a restart.

EXAMPLES:
    # Update beacon position
    beacon-emulator update 550e8400-e29b-41d4-a716-446655440000 \
                          --position 32.456,45.789,15.0

    # Update movement pattern to circular
    beacon-emulator update 550e8400-e29b-41d4-a716-446655440000 \
                          --movement circular:50:30

    # Update transmission interval
    beacon-emulator update 550e8400-e29b-41d4-a716-446655440000 \
                          --interval 3000
"#)]
    Update {
        /// Beacon ID to update
        #[arg(help = "UUID of the beacon to update")]
        id: Uuid,
        
        /// New position (lat,lon,depth)
        #[arg(long, help = "New position as 'lat,lon,depth'", value_parser = validate_position_string)]
        position: Option<GeodeticPosition>,
        
        /// New transmission interval in milliseconds
        #[arg(long, help = "New transmission interval in milliseconds", value_parser = validate_interval)]
        interval: Option<u32>,
        
        /// New movement pattern
        #[arg(long, help = "New movement pattern", value_parser = validate_movement_pattern)]
        movement: Option<MovementPattern>,
        
        /// Restart beacon if running (required for interval changes)
        #[arg(long, help = "Restart beacon to apply changes")]
        restart: bool,
    },
    
    /// Start a stopped beacon
    #[command(long_about = r#"
Start a previously created but stopped virtual beacon.

EXAMPLES:
    # Start a specific beacon
    beacon-emulator start 550e8400-e29b-41d4-a716-446655440000

    # Start all stopped beacons
    beacon-emulator start --all
"#)]
    Start {
        /// Beacon ID to start (required unless --all is used)
        #[arg(help = "UUID of the beacon to start", required_unless_present = "all")]
        id: Option<Uuid>,
        
        /// Start all stopped beacons
        #[arg(long, help = "Start all stopped beacons", conflicts_with = "id")]
        all: bool,
    },
    
    /// Remove a stopped beacon
    #[command(long_about = r#"
Remove a stopped virtual beacon from the registry.

The beacon must be stopped before it can be removed.

EXAMPLES:
    # Remove a specific stopped beacon
    beacon-emulator remove 550e8400-e29b-41d4-a716-446655440000
"#)]
    Remove {
        /// Beacon ID to remove
        #[arg(help = "UUID of the beacon to remove")]
        id: Uuid,
        
        /// Force removal even if beacon is running
        #[arg(long, help = "Force removal (stops beacon first if running)")]
        force: bool,
    },
    
    /// Create predefined test scenario
    #[command(long_about = r#"
Create multiple beacons arranged in predefined geometric patterns.

SCENARIO TYPES:
    triangle    - 3 beacons in triangular arrangement
    square      - 4 beacons in square arrangement  
    line        - N beacons in linear arrangement
    grid        - N beacons in rectangular grid

EXAMPLES:
    # Create triangular scenario with 100m spacing
    beacon-emulator scenario triangle --spacing 100

    # Create line of 5 beacons with custom center
    beacon-emulator scenario line --count 5 --spacing 50 \
                                 --center 32.123,45.476,10.0

    # Create 3x3 grid and start all beacons
    beacon-emulator scenario grid --count 9 --spacing 75 --start-all
"#)]
    Scenario {
        /// Scenario type (triangle, square, line, grid)
        #[arg(help = "Type of beacon arrangement")]
        scenario_type: ScenarioType,
        
        /// Number of beacons to create
        #[arg(short, long, default_value = "4", help = "Number of beacons in scenario", value_parser = validate_beacon_count)]
        count: usize,
        
        /// Spacing between beacons in meters
        #[arg(short, long, default_value = "100.0", help = "Distance between adjacent beacons", value_parser = validate_spacing)]
        spacing: f64,
        
        /// Center position (lat,lon,depth)
        #[arg(long, default_value = "32.123,45.476,10.0", help = "Center point of arrangement", value_parser = validate_position_string)]
        center: GeodeticPosition,
        
        /// Start all beacons after creation
        #[arg(long, help = "Start all created beacons immediately")]
        start_all: bool,
        
        /// Base transmission interval for all beacons
        #[arg(long, default_value = "5000", help = "Transmission interval for all beacons", value_parser = validate_interval)]
        interval: u32,
    },
    
    /// Monitor virtual beacon activity
    #[command(long_about = r#"
Monitor virtual beacon activity in real-time.

Displays beacon status, transmission statistics, and position updates.
Press Ctrl+C to exit monitoring mode.

EXAMPLES:
    # Monitor all beacons with 1-second updates
    beacon-emulator monitor

    # Monitor specific beacon with 5-second updates
    beacon-emulator monitor --beacon 550e8400-e29b-41d4-a716-446655440000 \
                           --interval 5

    # Monitor with compact display
    beacon-emulator monitor --compact
"#)]
    Monitor {
        /// Specific beacon ID to monitor (monitor all if not specified)
        #[arg(short, long, help = "Monitor only this specific beacon")]
        beacon: Option<Uuid>,
        
        /// Update interval in seconds
        #[arg(short, long, default_value = "1", help = "Display update interval in seconds", value_parser = validate_monitor_interval)]
        interval: u64,
        
        /// Use compact display format
        #[arg(short, long, help = "Use compact display format")]
        compact: bool,
        
        /// Show message content in monitoring
        #[arg(long, help = "Display transmitted message content")]
        show_messages: bool,
    },
    
    /// Export beacon activity logs
    #[command(long_about = r#"
Export beacon activity logs and message history to files.

Supports JSON and CSV formats with configurable time ranges and filtering.

EXAMPLES:
    # Export last hour of activity to JSON
    beacon-emulator export --output activity.json --duration 3600

    # Export specific beacon logs to CSV
    beacon-emulator export --output beacon.csv --format csv \
                          --beacon 550e8400-e29b-41d4-a716-446655440000

    # Export all activity since startup
    beacon-emulator export --output full.json --all-time
"#)]
    Export {
        /// Output file path
        #[arg(short, long, help = "Output file path")]
        output: PathBuf,
        
        /// Export format (json, csv)
        #[arg(short, long, default_value = "json", help = "Export file format")]
        format: ExportFormat,
        
        /// Time range in seconds from now (ignored if --all-time is used)
        #[arg(long, default_value = "3600", help = "Time range in seconds", value_parser = validate_duration)]
        duration: u64,
        
        /// Export all available data
        #[arg(long, help = "Export all available data", conflicts_with = "duration")]
        all_time: bool,
        
        /// Export data for specific beacon only
        #[arg(long, help = "Export data for specific beacon only")]
        beacon: Option<Uuid>,
        
        /// Include message content in export
        #[arg(long, help = "Include full message content")]
        include_messages: bool,
    },
    
    /// Show emulator status and statistics
    #[command(long_about = r#"
Display overall emulator status and statistics.

Shows information about active beacons, channels, and system resources.

EXAMPLES:
    # Show basic status
    beacon-emulator status

    # Show detailed statistics
    beacon-emulator status --detailed
"#)]
    Status {
        /// Show detailed statistics
        #[arg(short, long, help = "Show detailed statistics")]
        detailed: bool,
    },
}

#[derive(Clone, Debug, ValueEnum)]
pub enum MessageVersion {
    #[value(name = "v1")]
    V1,
    #[value(name = "v2")]
    V2,
    #[value(name = "v3")]
    V3,
}

impl Default for MessageVersion {
    fn default() -> Self {
        Self::V3
    }
}

#[derive(Clone, Debug, ValueEnum)]
pub enum ListFormat {
    Table,
    Json,
    Csv,
}

impl Default for ListFormat {
    fn default() -> Self {
        Self::Table
    }
}

// Validation functions for CLI arguments
fn validate_latitude(s: &str) -> Result<f64, String> {
    let lat: f64 = s.parse().map_err(|_| "Invalid latitude format")?;
    if lat < -90.0 || lat > 90.0 {
        return Err("Latitude must be between -90 and 90 degrees".to_string());
    }
    Ok(lat)
}

fn validate_longitude(s: &str) -> Result<f64, String> {
    let lon: f64 = s.parse().map_err(|_| "Invalid longitude format")?;
    if lon < -180.0 || lon > 180.0 {
        return Err("Longitude must be between -180 and 180 degrees".to_string());
    }
    Ok(lon)
}

fn validate_depth(s: &str) -> Result<f64, String> {
    let depth: f64 = s.parse().map_err(|_| "Invalid depth format")?;
    if depth < 0.0 || depth > 11000.0 {
        return Err("Depth must be between 0 and 11000 meters".to_string());
    }
    Ok(depth)
}

fn validate_interval(s: &str) -> Result<u32, String> {
    let interval: u32 = s.parse().map_err(|_| "Invalid interval format")?;
    if interval < 100 || interval > 300_000 {
        return Err("Interval must be between 100 and 300000 milliseconds".to_string());
    }
    Ok(interval)
}

fn validate_movement_pattern(s: &str) -> Result<MovementPattern, String> {
    use crate::EmulatorError;
    MovementPattern::from_str(s).map_err(|e: EmulatorError| e.to_string())
}

fn validate_position_string(s: &str) -> Result<GeodeticPosition, String> {
    let parts: Vec<&str> = s.split(',').collect();
    if parts.len() != 3 {
        return Err("Position must be in format 'lat,lon,depth'".to_string());
    }
    
    let lat = validate_latitude(parts[0])?;
    let lon = validate_longitude(parts[1])?;
    let depth = validate_depth(parts[2])?;
    
    Ok(GeodeticPosition {
        latitude: lat,
        longitude: lon,
        depth,
    })
}

fn validate_beacon_count(s: &str) -> Result<usize, String> {
    let count: usize = s.parse().map_err(|_| "Invalid beacon count format")?;
    if count == 0 || count > 100 {
        return Err("Beacon count must be between 1 and 100".to_string());
    }
    Ok(count)
}

fn validate_spacing(s: &str) -> Result<f64, String> {
    let spacing: f64 = s.parse().map_err(|_| "Invalid spacing format")?;
    if spacing <= 0.0 || spacing > 10000.0 {
        return Err("Spacing must be between 0 and 10000 meters".to_string());
    }
    Ok(spacing)
}

fn validate_monitor_interval(s: &str) -> Result<u64, String> {
    let interval: u64 = s.parse().map_err(|_| "Invalid monitor interval format")?;
    if interval == 0 || interval > 3600 {
        return Err("Monitor interval must be between 1 and 3600 seconds".to_string());
    }
    Ok(interval)
}

fn validate_duration(s: &str) -> Result<u64, String> {
    let duration: u64 = s.parse().map_err(|_| "Invalid duration format")?;
    if duration == 0 || duration > 86400 {
        return Err("Duration must be between 1 and 86400 seconds (24 hours)".to_string());
    }
    Ok(duration)
}