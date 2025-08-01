# Beacon Emulator CLI Reference

The beacon emulator provides a comprehensive command-line interface for creating and managing virtual beacons for testing the underwater positioning system.

## Table of Contents

- [Installation](#installation)
- [Basic Usage](#basic-usage)
- [Command Reference](#command-reference)
- [Configuration](#configuration)
- [Movement Patterns](#movement-patterns)
- [Test Scenarios](#test-scenarios)
- [Monitoring and Logging](#monitoring-and-logging)
- [Automation](#automation)
- [Examples](#examples)

## Installation

The beacon emulator is part of the underwater positioning system workspace. Build it using:

```bash
cargo build -p beacon-emulator
```

Or run directly with:

```bash
cargo run -p beacon-emulator -- [COMMAND] [OPTIONS]
```

## Basic Usage

### Global Options

All commands support these global options:

- `--channel <name>`: Virtual communication channel name (default: "default")
- `--log-level <level>`: Logging verbosity (trace, debug, info, warn, error)
- `--state-file <path>`: Custom path for persistent state file
- `--ipc-port <port>`: Port for IPC server (default: 8765)
- `--quiet`: Suppress non-essential output for automation
- `--non-interactive`: Skip all prompts, use defaults
- `--automation-mode`: Use specific exit codes for automation

### Quick Start

```bash
# Create a virtual beacon
beacon-emulator create --lat 32.123 --lon 45.476 --depth 10.0

# List active beacons
beacon-emulator list

# Start daemon mode to enable transmission
beacon-emulator daemon --auto-start

# Monitor beacon activity
beacon-emulator monitor
```

## Command Reference

### create

Create a new virtual beacon with specified position and configuration.

```bash
beacon-emulator create [OPTIONS] --lat <LAT> --lon <LON>
```

**Options:**
- `--id <UUID>`: Specific UUID for the beacon (auto-generated if not provided)
- `--lat <degrees>`: Latitude coordinate in decimal degrees (-90 to 90)
- `--lon <degrees>`: Longitude coordinate in decimal degrees (-180 to 180)
- `--depth <meters>`: Depth below surface in meters (default: 0.0, range: 0-11000)
- `--config <path>`: TOML configuration file path
- `--interval <ms>`: Message transmission interval (default: 5000, range: 100-300000)
- `--version <version>`: Message protocol version (V1, V2, V3, default: V3)
- `--movement <pattern>`: Movement pattern (default: stationary)
- `--start`: Start beacon transmission immediately

**Examples:**
```bash
# Create stationary beacon
beacon-emulator create --lat 32.123 --lon 45.476 --depth 10.0

# Create beacon with custom ID and config file
beacon-emulator create --id 550e8400-e29b-41d4-a716-446655440000 \
                      --lat 32.0 --lon 45.0 --config beacon.toml

# Create beacon with linear movement (1.5 m/s northeast)
beacon-emulator create --lat 32.0 --lon 45.0 --movement linear:1.5:45

# Create and start beacon immediately
beacon-emulator create --lat 32.0 --lon 45.0 --start
```

### list

List all virtual beacons with their current status and configuration.

```bash
beacon-emulator list [OPTIONS]
```

**Options:**
- `--detailed`: Show detailed beacon information including statistics
- `--running-only`: Show only currently running beacons
- `--format <format>`: Output format (table, json, csv)

**Examples:**
```bash
# List all beacons with basic information
beacon-emulator list

# List with detailed status
beacon-emulator list --detailed

# List only running beacons in JSON format
beacon-emulator list --running-only --format json
```

### start

Start a previously created but stopped virtual beacon.

```bash
beacon-emulator start [BEACON_ID] [OPTIONS]
```

**Options:**
- `--all`: Start all stopped beacons

**Examples:**
```bash
# Start specific beacon
beacon-emulator start 550e8400-e29b-41d4-a716-446655440000

# Start all stopped beacons
beacon-emulator start --all
```

### stop

Stop a specific virtual beacon by its UUID.

```bash
beacon-emulator stop <BEACON_ID> [OPTIONS]
```

**Options:**
- `--remove`: Remove beacon from registry after stopping

**Examples:**
```bash
# Stop specific beacon
beacon-emulator stop 550e8400-e29b-41d4-a716-446655440000

# Stop and remove beacon
beacon-emulator stop 550e8400-e29b-41d4-a716-446655440000 --remove
```

### stop-all

Stop all currently running virtual beacons.

```bash
beacon-emulator stop-all [OPTIONS]
```

**Options:**
- `--remove`: Remove all beacons from registry after stopping

**Examples:**
```bash
# Stop all beacons
beacon-emulator stop-all

# Stop and remove all beacons
beacon-emulator stop-all --remove
```

### update

Update configuration of an existing virtual beacon.

```bash
beacon-emulator update <BEACON_ID> [OPTIONS]
```

**Options:**
- `--position <lat,lon,depth>`: New position coordinates
- `--interval <ms>`: New transmission interval in milliseconds
- `--movement <pattern>`: New movement pattern
- `--restart`: Restart beacon to apply changes

**Examples:**
```bash
# Update beacon position
beacon-emulator update 550e8400-e29b-41d4-a716-446655440000 \
                      --position 32.456,45.789,15.0

# Update movement pattern to circular
beacon-emulator update 550e8400-e29b-41d4-a716-446655440000 \
                      --movement circular:50:30

# Update transmission interval (requires restart)
beacon-emulator update 550e8400-e29b-41d4-a716-446655440000 \
                      --interval 3000 --restart
```

### remove

Remove a stopped virtual beacon from the registry.

```bash
beacon-emulator remove <BEACON_ID> [OPTIONS]
```

**Options:**
- `--force`: Force removal even if beacon is running

**Examples:**
```bash
# Remove stopped beacon
beacon-emulator remove 550e8400-e29b-41d4-a716-446655440000

# Force remove running beacon
beacon-emulator remove 550e8400-e29b-41d4-a716-446655440000 --force
```

### scenario

Create multiple beacons arranged in predefined geometric patterns.

```bash
beacon-emulator scenario <SCENARIO_TYPE> [OPTIONS]
```

**Scenario Types:**
- `triangle`: 3 beacons in triangular arrangement
- `square`: 4 beacons in square arrangement
- `line`: N beacons in linear arrangement
- `grid`: N beacons in rectangular grid

**Options:**
- `--count <n>`: Number of beacons in scenario (default: 4)
- `--spacing <meters>`: Distance between adjacent beacons (default: 100.0)
- `--center <lat,lon,depth>`: Center point of arrangement (default: 32.123,45.476,10.0)
- `--start-all`: Start all created beacons immediately
- `--interval <ms>`: Transmission interval for all beacons (default: 5000)

**Examples:**
```bash
# Create triangular scenario with 100m spacing
beacon-emulator scenario triangle --spacing 100

# Create line of 5 beacons with custom center
beacon-emulator scenario line --count 5 --spacing 50 \
                             --center 32.123,45.476,10.0

# Create 3x3 grid and start all beacons
beacon-emulator scenario grid --count 9 --spacing 75 --start-all
```

### monitor

Monitor virtual beacon activity in real-time.

```bash
beacon-emulator monitor [OPTIONS]
```

**Options:**
- `--beacon <UUID>`: Monitor only specific beacon
- `--interval <seconds>`: Display update interval (default: 1)
- `--compact`: Use compact display format
- `--show-messages`: Display transmitted message content

**Examples:**
```bash
# Monitor all beacons with 1-second updates
beacon-emulator monitor

# Monitor specific beacon with 5-second updates
beacon-emulator monitor --beacon 550e8400-e29b-41d4-a716-446655440000 \
                       --interval 5

# Monitor with compact display and message content
beacon-emulator monitor --compact --show-messages
```

### export

Export beacon activity logs and message history to files.

```bash
beacon-emulator export --output <PATH> [OPTIONS]
```

**Options:**
- `--format <format>`: Export file format (json, csv, default: json)
- `--duration <seconds>`: Time range in seconds from now (default: 3600)
- `--all-time`: Export all available data
- `--beacon <UUID>`: Export data for specific beacon only
- `--include-messages`: Include full message content

**Examples:**
```bash
# Export last hour of activity to JSON
beacon-emulator export --output activity.json --duration 3600

# Export specific beacon logs to CSV
beacon-emulator export --output beacon.csv --format csv \
                      --beacon 550e8400-e29b-41d4-a716-446655440000

# Export all activity with message content
beacon-emulator export --output full.json --all-time --include-messages
```

### daemon

Run the emulator in daemon mode to keep beacons actively transmitting.

```bash
beacon-emulator daemon [OPTIONS]
```

**Options:**
- `--status-interval <seconds>`: Status update interval (default: 30)
- `--auto-start`: Automatically start all beacons marked as intended-running
- `--background`: Run in background mode with minimal output

**Examples:**
```bash
# Run daemon mode with default settings
beacon-emulator daemon

# Run daemon with auto-start and custom interval
beacon-emulator daemon --auto-start --status-interval 10

# Run in background mode
beacon-emulator daemon --background
```

### status

Display overall emulator status and statistics.

```bash
beacon-emulator status [OPTIONS]
```

**Options:**
- `--detailed`: Show detailed statistics

**Examples:**
```bash
# Show basic status
beacon-emulator status

# Show detailed statistics
beacon-emulator status --detailed
```

### clear

Clear all beacon state and remove the persistent state file.

```bash
beacon-emulator clear [OPTIONS]
```

**Options:**
- `--confirm`: Skip confirmation prompt

**Examples:**
```bash
# Clear all state with confirmation
beacon-emulator clear

# Clear state without confirmation
beacon-emulator clear --confirm
```

### batch

Execute multiple beacon operations from a batch configuration file.

```bash
beacon-emulator batch --file <PATH> [OPTIONS]
```

**Options:**
- `--dry-run`: Validate batch file without executing operations
- `--timeout <seconds>`: Maximum execution time (default: 600)
- `--continue-on-error`: Continue executing operations even if some fail
- `--verbose`: Output detailed execution log

**Examples:**
```bash
# Execute batch operations
beacon-emulator batch --file operations.json

# Validate batch file without execution
beacon-emulator batch --file operations.json --dry-run

# Execute with error tolerance and verbose output
beacon-emulator batch --file operations.json --continue-on-error --verbose
```

### generate-template

Generate a configuration template file with default values.

```bash
beacon-emulator generate-template --output <PATH> [OPTIONS]
```

**Options:**
- `--format <format>`: Template file format (toml, json, yaml, default: toml)
- `--emulator`: Generate emulator-specific configuration template
- `--lat <degrees>`: Latitude for emulator template (required with --emulator)
- `--lon <degrees>`: Longitude for emulator template (required with --emulator)
- `--depth <meters>`: Depth for emulator template (default: 0.0)

**Examples:**
```bash
# Generate TOML template
beacon-emulator generate-template --output template.toml

# Generate emulator-specific JSON template
beacon-emulator generate-template --output emulator.json --format json \
                                 --emulator --lat 32.0 --lon 45.0
```

### validate-config

Validate a beacon configuration file for emulator compatibility.

```bash
beacon-emulator validate-config --config <PATH> [OPTIONS]
```

**Options:**
- `--emulator`: Validate as emulator-specific configuration
- `--verbose`: Show detailed validation information

**Examples:**
```bash
# Validate TOML configuration
beacon-emulator validate-config --config beacon.toml

# Validate with detailed output
beacon-emulator validate-config --config beacon.toml --verbose
```

### performance

Display current performance metrics and system resource usage.

```bash
beacon-emulator performance [OPTIONS]
```

**Options:**
- `--detailed`: Show detailed performance breakdown
- `--export <path>`: Export performance metrics to JSON file
- `--reset`: Reset all performance counters

**Examples:**
```bash
# Show current performance metrics
beacon-emulator performance

# Export detailed metrics to file
beacon-emulator performance --detailed --export metrics.json
```

## Configuration

### Configuration Files

Beacon configurations can be loaded from TOML, JSON, or YAML files:

```toml
# beacon.toml
[beacon]
id = "550e8400-e29b-41d4-a716-446655440000"
transmission_interval_ms = 5000
message_version = "V3"

[position]
latitude = 32.123456
longitude = 45.476789
depth = 10.0

[emulator]
movement_pattern = "stationary"
auto_start = true
```

### State Persistence

The emulator maintains persistent state in `data/emulator_state.json` by default. This includes:
- Beacon configurations and status
- Runtime statistics
- Channel information
- Performance metrics

## Movement Patterns

### Stationary
Beacon remains at fixed position.
```bash
--movement stationary
```

### Linear Movement
Beacon moves in straight line at constant speed.
```bash
--movement linear:speed:bearing
# speed: meters per second
# bearing: degrees (0-360, 0=north, 90=east)

# Example: 1.5 m/s northeast
--movement linear:1.5:45
```

### Circular Movement
Beacon moves in circular pattern around initial position.
```bash
--movement circular:radius:period
# radius: meters from center
# period: seconds for complete circle

# Example: 100m radius, 60 second period
--movement circular:100:60
```

### Random Walk
Beacon moves randomly with maximum speed constraint.
```bash
--movement random:max_speed
# max_speed: maximum speed in meters per second

# Example: random walk up to 0.5 m/s
--movement random:0.5
```

## Test Scenarios

### Triangle Scenario
Creates 3 beacons in equilateral triangle formation:
```bash
beacon-emulator scenario triangle --spacing 100 --center 32.0,45.0,10.0
```

### Square Scenario
Creates 4 beacons in square formation:
```bash
beacon-emulator scenario square --spacing 100 --center 32.0,45.0,10.0
```

### Line Scenario
Creates N beacons in straight line:
```bash
beacon-emulator scenario line --count 5 --spacing 50 --center 32.0,45.0,10.0
```

### Grid Scenario
Creates N beacons in rectangular grid:
```bash
beacon-emulator scenario grid --count 9 --spacing 75 --center 32.0,45.0,10.0
```

## Monitoring and Logging

### Real-time Monitoring
The monitor command provides real-time status updates:
```bash
beacon-emulator monitor --interval 2 --show-messages
```

Display includes:
- Beacon ID and status
- Current position
- Message transmission rate
- Signal quality
- Movement pattern
- Runtime statistics

### Log Export
Export activity logs for analysis:
```bash
# JSON format with all data
beacon-emulator export --output full_log.json --all-time --include-messages

# CSV format for specific time range
beacon-emulator export --output recent.csv --format csv --duration 1800
```

### Performance Metrics
Monitor system performance:
```bash
beacon-emulator performance --detailed
```

Metrics include:
- Message transmission rates
- Memory usage
- CPU utilization
- Channel statistics
- Error rates

## Automation

### Batch Operations
Define complex operations in JSON configuration:

```json
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
```

Execute with:
```bash
beacon-emulator batch --file operations.json
```

### CI/CD Integration
Use automation-friendly options:
```bash
# Non-interactive mode with specific exit codes
beacon-emulator --automation-mode --non-interactive create \
               --lat 32.0 --lon 45.0 --start

# Quiet mode for minimal output
beacon-emulator --quiet list --format json

# Reset for clean test environment
beacon-emulator reset --force
```

### Exit Codes
When `--automation-mode` is enabled:
- 0: Success
- 1: Error
- 2: Warning

## Examples

### Basic Testing Workflow
```bash
# 1. Create test scenario
beacon-emulator scenario triangle --spacing 100 --start-all

# 2. Start daemon mode
beacon-emulator daemon --background &

# 3. Monitor for 30 seconds
timeout 30 beacon-emulator monitor --compact

# 4. Export results
beacon-emulator export --output test_results.json --duration 60

# 5. Clean up
beacon-emulator stop-all --remove
```

### Performance Testing
```bash
# Create many beacons for stress testing
beacon-emulator scenario grid --count 25 --spacing 50 --start-all

# Monitor performance
beacon-emulator performance --detailed

# Run daemon with performance monitoring
beacon-emulator daemon --status-interval 5
```

### Integration Testing
```bash
# Start emulator on specific channel
beacon-emulator --channel integration_test create \
               --lat 32.0 --lon 45.0 --start

# Start virtual receiver on same channel
cd receiver
cargo run -- virtual --channel integration_test

# Monitor both systems
beacon-emulator --channel integration_test monitor
```

### Automated Test Suite
```bash
#!/bin/bash
# test_suite.sh

# Reset environment
beacon-emulator reset --force

# Run test scenarios
beacon-emulator batch --file accuracy_test.json
beacon-emulator batch --file performance_test.json
beacon-emulator batch --file stress_test.json

# Generate reports
beacon-emulator export --output test_report.json --all-time
beacon-emulator performance --export performance_report.json

# Cleanup
beacon-emulator clear --confirm
```

This CLI reference provides comprehensive documentation for all beacon emulator commands and features. For additional help with specific commands, use:

```bash
beacon-emulator <command> --help
```