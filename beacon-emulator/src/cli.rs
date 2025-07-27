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

    # Use a custom state file location
    beacon-emulator --state-file /tmp/my_beacons.json list

    # Create a triangular test scenario with 3 beacons
    beacon-emulator scenario triangle --count 3 --spacing 100

    # List all beacons (shows intended state)
    beacon-emulator list --detailed

    # Run daemon mode to actually transmit beacon signals
    beacon-emulator daemon --auto-start

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
    
    /// Path to the persistent state file
    #[arg(long, help = "Custom path for the persistent state file (default: data/emulator_state.json)")]
    pub state_file: Option<PathBuf>,
    
    /// IPC server port for virtual receiver connections
    #[arg(long, default_value = "8765", help = "Port for IPC server (for virtual receiver connections)")]
    pub ipc_port: u16,
    
    /// Run in quiet mode (minimal output, suitable for automation)
    #[arg(short, long, help = "Suppress non-essential output for automation")]
    pub quiet: bool,
    
    /// Non-interactive mode (skip all prompts, use defaults)
    #[arg(long, help = "Run in non-interactive mode for automation")]
    pub non_interactive: bool,
    
    /// Exit with specific codes for automation (0=success, 1=error, 2=warning)
    #[arg(long, help = "Use specific exit codes for automation")]
    pub automation_mode: bool,
    
    #[command(subcommand)]
    pub command: EmulatorCommand,
}

#[derive(Clone, Subcommand)]
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
    
    /// Clear all beacon state and start fresh
    #[command(long_about = r#"
Clear all beacon state and remove the persistent state file.

This will stop and remove all beacons and delete the state file,
effectively resetting the emulator to a clean state.

EXAMPLES:
    # Clear all state
    beacon-emulator clear

    # Clear state with confirmation
    beacon-emulator clear --confirm
"#)]
    Clear {
        /// Skip confirmation prompt
        #[arg(short, long, help = "Skip confirmation prompt")]
        confirm: bool,
    },
    
    /// Run emulator in daemon mode to keep beacons running
    #[command(long_about = r#"
Run the emulator in daemon mode to keep beacons actively transmitting.

In daemon mode, the emulator process stays running and maintains active
beacon transmission tasks. This is required for beacons to actually
transmit messages. Without daemon mode, beacons are only "intended" to
run but don't actually transmit.

EXAMPLES:
    # Run daemon mode with default settings
    beacon-emulator daemon

    # Run daemon with custom update interval
    beacon-emulator daemon --status-interval 10

    # Run daemon and start all intended-running beacons
    beacon-emulator daemon --auto-start
"#)]
    Daemon {
        /// Status update interval in seconds
        #[arg(long, default_value = "30", help = "Status update interval in seconds")]
        status_interval: u64,
        
        /// Automatically start all beacons marked as intended-running
        #[arg(long, help = "Auto-start all beacons marked as intended-running")]
        auto_start: bool,
        
        /// Run in background (suppress status output)
        #[arg(long, help = "Run in background mode with minimal output")]
        background: bool,
    },
    
    /// Generate a configuration template file
    #[command(long_about = r#"
Generate a configuration template file with default values.

This creates a template configuration file that can be customized for
specific beacon requirements. The template includes all available
configuration options with sensible defaults for emulator use.

EXAMPLES:
    # Generate TOML template
    beacon-emulator generate-template --output template.toml

    # Generate JSON template
    beacon-emulator generate-template --output template.json --format json

    # Generate emulator-specific template
    beacon-emulator generate-template --output emulator.json \
                                     --emulator --lat 32.0 --lon 45.0
"#)]
    GenerateTemplate {
        /// Output file path
        #[arg(short, long, help = "Output template file path")]
        output: PathBuf,
        
        /// Template format
        #[arg(short, long, default_value = "toml", help = "Template file format")]
        format: ConfigFormat,
        
        /// Generate emulator-specific template
        #[arg(long, help = "Generate emulator-specific configuration template")]
        emulator: bool,
        
        /// Latitude for emulator template (required if --emulator is used)
        #[arg(long, help = "Latitude for emulator template", required_if_eq("emulator", "true"), value_parser = validate_latitude)]
        lat: Option<f64>,
        
        /// Longitude for emulator template (required if --emulator is used)
        #[arg(long, help = "Longitude for emulator template", required_if_eq("emulator", "true"), value_parser = validate_longitude)]
        lon: Option<f64>,
        
        /// Depth for emulator template
        #[arg(long, default_value = "0.0", help = "Depth for emulator template", value_parser = validate_depth)]
        depth: f64,
    },
    
    /// Validate a configuration file
    #[command(long_about = r#"
Validate a beacon configuration file for emulator compatibility.

This command checks that a configuration file is valid and suitable
for use with the beacon emulator. It validates both the file format
and the configuration values against emulator requirements.

EXAMPLES:
    # Validate a TOML configuration
    beacon-emulator validate-config --config beacon.toml

    # Validate with detailed output
    beacon-emulator validate-config --config beacon.toml --verbose

    # Validate emulator-specific configuration
    beacon-emulator validate-config --config emulator-beacon.json --emulator
"#)]
    ValidateConfig {
        /// Configuration file to validate
        #[arg(short, long, help = "Configuration file path")]
        config: PathBuf,
        
        /// Validate as emulator-specific configuration
        #[arg(long, help = "Validate as emulator-specific configuration")]
        emulator: bool,
        
        /// Show detailed validation output
        #[arg(short, long, help = "Show detailed validation information")]
        verbose: bool,
    },
    
    /// Convert configuration between formats
    #[command(long_about = r#"
Convert a configuration file between different formats (TOML, JSON, YAML).

This command reads a configuration file in one format and writes it
in another format, preserving all configuration values.

EXAMPLES:
    # Convert TOML to JSON
    beacon-emulator convert-config --input beacon.toml --output beacon.json

    # Convert JSON to YAML
    beacon-emulator convert-config --input beacon.json --output beacon.yaml

    # Convert with validation
    beacon-emulator convert-config --input beacon.toml --output beacon.json --validate
"#)]
    ConvertConfig {
        /// Input configuration file
        #[arg(short, long, help = "Input configuration file path")]
        input: PathBuf,
        
        /// Output configuration file
        #[arg(short, long, help = "Output configuration file path")]
        output: PathBuf,
        
        /// Validate configuration during conversion
        #[arg(long, help = "Validate configuration during conversion")]
        validate: bool,
    },
    
    /// Execute batch operations from a configuration file
    #[command(long_about = r#"
Execute multiple beacon operations from a batch configuration file.

This command allows you to define multiple beacon creation, scenario setup,
and management operations in a single configuration file and execute them
all at once. This is particularly useful for automated testing and CI/CD
pipelines.

BATCH FILE FORMAT (JSON):
{
  "operations": [
    {
      "type": "create_beacon",
      "id": "550e8400-e29b-41d4-a716-446655440000",
      "position": [32.123, 45.476, 10.0],
      "interval": 5000,
      "movement": "stationary",
      "start": true
    },
    {
      "type": "create_scenario",
      "scenario_type": "triangle",
      "count": 3,
      "spacing": 100.0,
      "center": [32.0, 45.0, 15.0],
      "start_all": true
    },
    {
      "type": "wait",
      "duration": 10
    },
    {
      "type": "export_logs",
      "output": "test_results.json",
      "format": "json",
      "duration": 60
    }
  ]
}

EXAMPLES:
    # Execute batch operations
    beacon-emulator batch --file operations.json

    # Execute with dry-run to validate
    beacon-emulator batch --file operations.json --dry-run

    # Execute with custom timeout
    beacon-emulator batch --file operations.json --timeout 300
"#)]
    Batch {
        /// Batch operations file (JSON format)
        #[arg(short, long, help = "Batch operations configuration file")]
        file: PathBuf,
        
        /// Perform dry-run without executing operations
        #[arg(long, help = "Validate batch file without executing operations")]
        dry_run: bool,
        
        /// Timeout for batch execution in seconds
        #[arg(long, default_value = "600", help = "Maximum execution time in seconds")]
        timeout: u64,
        
        /// Continue on errors instead of stopping
        #[arg(long, help = "Continue executing operations even if some fail")]
        continue_on_error: bool,
        
        /// Output detailed execution log
        #[arg(long, help = "Output detailed execution log")]
        verbose: bool,
    },
    
    /// Reset emulator to clean state for automated testing
    #[command(long_about = r#"
Reset the emulator to a clean state for automated testing.

This command provides a comprehensive reset that stops all beacons,
clears all state, and optionally resets configuration files. It's
designed for use in automated testing environments where you need
a clean slate between test runs.

EXAMPLES:
    # Basic reset (stops all beacons, clears state)
    beacon-emulator reset

    # Reset with confirmation skip (for automation)
    beacon-emulator reset --force

    # Reset and remove all config files
    beacon-emulator reset --force --clean-configs

    # Reset with custom state file
    beacon-emulator reset --state-file /tmp/test_state.json --force
"#)]
    Reset {
        /// Skip confirmation prompts (for automation)
        #[arg(long, help = "Skip all confirmation prompts")]
        force: bool,
        
        /// Also remove generated configuration files
        #[arg(long, help = "Remove generated configuration files")]
        clean_configs: bool,
        
        /// Reset specific state file
        #[arg(long, help = "Reset specific state file instead of default")]
        state_file: Option<PathBuf>,
    },
    
    /// Generate test scenario configuration files
    #[command(long_about = r#"
Generate configuration files for common test scenarios.

This command creates pre-configured scenario files that can be used
with the batch command for automated testing. It includes templates
for common testing patterns like performance tests, accuracy validation,
and stress testing.

SCENARIO TEMPLATES:
    performance     - High-throughput beacon setup for performance testing
    accuracy        - Precise beacon arrangements for accuracy validation  
    stress          - Large number of beacons for stress testing
    integration     - Multi-scenario setup for integration testing
    custom          - Interactive custom scenario builder

EXAMPLES:
    # Generate performance test scenario
    beacon-emulator generate-scenario --template performance --output perf_test.json

    # Generate custom scenario with specific parameters
    beacon-emulator generate-scenario --template custom --output custom.json \
                                     --beacon-count 10 --area-size 500

    # Generate all standard scenarios
    beacon-emulator generate-scenario --template all --output-dir scenarios/
"#)]
    GenerateScenario {
        /// Scenario template type
        #[arg(short, long, help = "Scenario template to generate")]
        template: ScenarioTemplate,
        
        /// Output file path (or directory for 'all' template)
        #[arg(short, long, help = "Output file or directory path")]
        output: PathBuf,
        
        /// Number of beacons for custom scenarios
        #[arg(long, help = "Number of beacons (for custom template)")]
        beacon_count: Option<usize>,
        
        /// Area size in meters for beacon distribution
        #[arg(long, help = "Area size in meters (for custom template)")]
        area_size: Option<f64>,
        
        /// Test duration in seconds
        #[arg(long, help = "Test duration in seconds")]
        duration: Option<u64>,
        
        /// Include receiver configuration
        #[arg(long, help = "Include virtual receiver configuration")]
        include_receiver: bool,
    },
    
    /// Show performance metrics and system resource usage
    #[command(long_about = r#"
Display current performance metrics including message transmission rates,
memory usage, CPU utilization, and rate limiting statistics.

EXAMPLES:
    # Show current performance metrics
    beacon-emulator performance

    # Show detailed performance breakdown
    beacon-emulator performance --detailed

    # Export performance metrics to JSON
    beacon-emulator performance --export metrics.json
"#)]
    Performance {
        /// Show detailed performance breakdown
        #[arg(short, long, help = "Show detailed performance information")]
        detailed: bool,
        
        /// Export metrics to file
        #[arg(long, help = "Export performance metrics to JSON file")]
        export: Option<PathBuf>,
        
        /// Reset performance counters
        #[arg(long, help = "Reset all performance counters")]
        reset: bool,
    },
    
    /// Configure performance optimization settings
    #[command(long_about = r#"
Configure performance optimization settings including rate limiting,
collision avoidance, and memory management.

EXAMPLES:
    # Set global rate limit to 500 messages/second
    beacon-emulator optimize --global-rate-limit 500

    # Set per-beacon rate limit to 5 messages/second
    beacon-emulator optimize --per-beacon-rate-limit 5

    # Enable collision avoidance with 50ms window
    beacon-emulator optimize --collision-avoidance --collision-window 50

    # Show current optimization settings
    beacon-emulator optimize --show-config
"#)]
    Optimize {
        /// Set global message rate limit (messages per second)
        #[arg(long, help = "Global rate limit in messages per second")]
        global_rate_limit: Option<f64>,
        
        /// Set per-beacon message rate limit (messages per second)
        #[arg(long, help = "Per-beacon rate limit in messages per second")]
        per_beacon_rate_limit: Option<f64>,
        
        /// Enable or disable collision avoidance
        #[arg(long, help = "Enable collision avoidance")]
        collision_avoidance: Option<bool>,
        
        /// Set collision avoidance window (milliseconds)
        #[arg(long, help = "Collision avoidance window in milliseconds")]
        collision_window: Option<u64>,
        
        /// Enable or disable automatic optimization
        #[arg(long, help = "Enable automatic performance optimization")]
        auto_optimization: Option<bool>,
        
        /// Show current optimization configuration
        #[arg(long, help = "Show current optimization settings")]
        show_config: bool,
        
        /// Get performance recommendations
        #[arg(long, help = "Get performance optimization recommendations")]
        recommendations: bool,
    },
    
    /// Memory management and cleanup operations
    #[command(long_about = r#"
Perform memory management operations including cleanup of old messages,
memory usage analysis, and garbage collection.

EXAMPLES:
    # Show memory usage breakdown
    beacon-emulator memory --breakdown

    # Clean up messages older than 1 hour
    beacon-emulator memory --cleanup 3600

    # Force garbage collection
    beacon-emulator memory --gc

    # Set memory warning threshold to 50MB
    beacon-emulator memory --warning-threshold 52428800
"#)]
    Memory {
        /// Show memory usage breakdown
        #[arg(short, long, help = "Show detailed memory usage breakdown")]
        breakdown: bool,
        
        /// Clean up messages older than specified seconds
        #[arg(long, help = "Clean up messages older than N seconds")]
        cleanup: Option<u64>,
        
        /// Force garbage collection
        #[arg(long, help = "Force garbage collection and memory cleanup")]
        gc: bool,
        
        /// Set memory warning threshold (bytes)
        #[arg(long, help = "Set memory warning threshold in bytes")]
        warning_threshold: Option<u64>,
        
        /// Show memory statistics over time
        #[arg(long, help = "Show memory usage statistics")]
        stats: bool,
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

#[derive(Clone, Debug, ValueEnum)]
pub enum ConfigFormat {
    #[value(name = "toml")]
    Toml,
    #[value(name = "json")]
    Json,
    #[value(name = "yaml")]
    Yaml,
}

impl Default for ConfigFormat {
    fn default() -> Self {
        Self::Toml
    }
}

#[derive(Clone, Debug, ValueEnum)]
pub enum ScenarioTemplate {
    #[value(name = "performance")]
    Performance,
    #[value(name = "accuracy")]
    Accuracy,
    #[value(name = "stress")]
    Stress,
    #[value(name = "integration")]
    Integration,
    #[value(name = "custom")]
    Custom,
    #[value(name = "all")]
    All,
}

impl From<ConfigFormat> for crate::config::ConfigFormat {
    fn from(format: ConfigFormat) -> Self {
        match format {
            ConfigFormat::Toml => crate::config::ConfigFormat::Toml,
            ConfigFormat::Json => crate::config::ConfigFormat::Json,
            ConfigFormat::Yaml => crate::config::ConfigFormat::Yaml,
        }
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

