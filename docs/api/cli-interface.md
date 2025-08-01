# CLI Interface Documentation

The beacon system provides a comprehensive command-line interface for configuration, deployment, monitoring, and management operations. This document covers all CLI commands and their usage.

## Table of Contents

- [Installation and Setup](#installation-and-setup)
- [Basic Commands](#basic-commands)
- [Configuration Commands](#configuration-commands)
- [Status and Monitoring](#status-and-monitoring)
- [Deployment Commands](#deployment-commands)
- [Fleet Management](#fleet-management)
- [Diagnostic Commands](#diagnostic-commands)
- [Utility Commands](#utility-commands)
- [Configuration File Format](#configuration-file-format)
- [Examples and Use Cases](#examples-and-use-cases)

## Installation and Setup

### Building the CLI

```bash
# Build the beacon CLI tool
cargo build --release --bin beacon

# Install to system path (optional)
cargo install --path beacon
```

### Basic Usage

```bash
beacon [OPTIONS] <COMMAND>
```

### Global Options

```
-c, --config <CONFIG>    Configuration file path [default: beacon.toml]
-v, --verbose           Enable verbose output
-q, --quiet             Suppress non-error output
-h, --help              Print help information
-V, --version           Print version information
```

## Basic Commands

### Generate Configuration

Create a default configuration file:

```bash
beacon generate-config [OPTIONS]
```

**Options:**
- `-o, --output <FILE>` - Output file path [default: beacon-default.toml]
- `--template <TEMPLATE>` - Use configuration template (basic, advanced, marine, test)
- `--beacon-id <UUID>` - Set specific beacon ID

**Examples:**
```bash
# Generate default configuration
beacon generate-config

# Generate with specific output file
beacon generate-config --output my-beacon.toml

# Generate using marine template
beacon generate-config --template marine --output marine-beacon.toml

# Generate with specific beacon ID
beacon generate-config --beacon-id 12345678-1234-5678-9abc-123456789abc
```

### Validate Configuration

Validate a configuration file:

```bash
beacon validate-config [OPTIONS]
```

**Options:**
- `-c, --config <FILE>` - Configuration file to validate [default: beacon.toml]
- `--strict` - Enable strict validation mode
- `--check-hardware` - Validate hardware-specific settings

**Examples:**
```bash
# Validate default configuration
beacon validate-config

# Validate specific configuration file
beacon validate-config --config deployment/beacon-001.toml

# Strict validation with hardware checks
beacon validate-config --strict --check-hardware
```

## Configuration Commands

### Update Configuration

Update configuration parameters:

```bash
beacon config update [OPTIONS] <KEY> <VALUE>
```

**Examples:**
```bash
# Update transmission interval
beacon config update transmission.interval_ms 10000

# Update GPS accuracy threshold
beacon config update gps.accuracy_threshold_m 3.0

# Update power save threshold
beacon config update power.power_save_mode_threshold_percent 25.0
```

### Show Configuration

Display current configuration:

```bash
beacon config show [OPTIONS]
```

**Options:**
- `--section <SECTION>` - Show specific section (transmission, gps, power, communication, emergency, hardware)
- `--format <FORMAT>` - Output format (toml, json, yaml) [default: toml]

**Examples:**
```bash
# Show full configuration
beacon config show

# Show only GPS configuration
beacon config show --section gps

# Show configuration in JSON format
beacon config show --format json
```

### Reset Configuration

Reset configuration to defaults:

```bash
beacon config reset [OPTIONS]
```

**Options:**
- `--section <SECTION>` - Reset specific section only
- `--backup` - Create backup before reset

**Examples:**
```bash
# Reset entire configuration with backup
beacon config reset --backup

# Reset only power configuration
beacon config reset --section power
```

## Status and Monitoring

### Show Status

Display beacon status information:

```bash
beacon status [OPTIONS]
```

**Options:**
- `-d, --detailed` - Show detailed status information
- `--gps` - Show GPS status only
- `--power` - Show power status only
- `--communication` - Show communication status only
- `--transmission` - Show transmission statistics only
- `--refresh <SECONDS>` - Auto-refresh interval
- `--format <FORMAT>` - Output format (text, json, yaml) [default: text]

**Examples:**
```bash
# Show basic status
beacon status

# Show detailed status
beacon status --detailed

# Show GPS status only
beacon status --gps

# Auto-refresh every 5 seconds
beacon status --refresh 5

# Show status in JSON format
beacon status --format json
```

### Monitor Beacon

Continuous monitoring with real-time updates:

```bash
beacon monitor [OPTIONS]
```

**Options:**
- `-i, --interval <SECONDS>` - Update interval [default: 10]
- `--alerts-only` - Show only alerts and warnings
- `--log-file <FILE>` - Log monitoring data to file
- `--threshold <THRESHOLD>` - Health threshold for alerts [default: 0.8]

**Examples:**
```bash
# Monitor with default 10-second interval
beacon monitor

# Monitor with 30-second interval
beacon monitor --interval 30

# Monitor and log to file
beacon monitor --log-file beacon-monitor.log

# Show only alerts
beacon monitor --alerts-only
```

## Deployment Commands

### Generate Deployment Configurations

Generate multiple beacon configurations for deployment:

```bash
beacon deploy generate-configs [OPTIONS]
```

**Options:**
- `--count <COUNT>` - Number of beacon configurations to generate
- `--output <DIR>` - Output directory [default: deployment/]
- `--site-config <FILE>` - Site configuration file
- `--template <FILE>` - Configuration template file
- `--prefix <PREFIX>` - Beacon ID prefix
- `--sequential-ids` - Use sequential beacon IDs

**Examples:**
```bash
# Generate 5 beacon configurations
beacon deploy generate-configs --count 5

# Generate with site-specific settings
beacon deploy generate-configs --count 10 --site-config site-001.json

# Generate with custom template
beacon deploy generate-configs --count 3 --template marine-template.json
```

### Validate Deployment

Validate a deployment configuration:

```bash
beacon deploy validate-deployment [OPTIONS]
```

**Options:**
- `--deployment <DIR>` - Deployment directory [default: deployment/]
- `--test-duration <SECONDS>` - Test duration [default: 60]
- `--parallel` - Run validation tests in parallel
- `--report <FILE>` - Generate validation report

**Examples:**
```bash
# Validate deployment with default settings
beacon deploy validate-deployment

# Validate with extended test duration
beacon deploy validate-deployment --test-duration 300

# Generate validation report
beacon deploy validate-deployment --report validation-report.json
```

### Test Deployment

Test deployment configurations with mock hardware:

```bash
beacon deploy test-deployment [OPTIONS]
```

**Options:**
- `--deployment <DIR>` - Deployment directory
- `--duration <SECONDS>` - Test duration [default: 300]
- `--mock-gps` - Use mock GPS data
- `--mock-power` - Use mock power system
- `--mock-communication` - Use mock communication
- `--scenario <SCENARIO>` - Test scenario (normal, low-power, gps-loss, communication-loss)

**Examples:**
```bash
# Test deployment for 5 minutes
beacon deploy test-deployment --duration 300

# Test with GPS signal loss scenario
beacon deploy test-deployment --scenario gps-loss

# Test with all mock systems
beacon deploy test-deployment --mock-gps --mock-power --mock-communication
```

## Fleet Management

### Create Fleet

Create a new beacon fleet:

```bash
beacon fleet create [OPTIONS] <FLEET_ID>
```

**Options:**
- `--name <NAME>` - Fleet display name
- `--description <DESC>` - Fleet description
- `--data-dir <DIR>` - Fleet data directory [default: fleet_data/]

**Examples:**
```bash
# Create basic fleet
beacon fleet create north_atlantic_fleet --name "North Atlantic Deployment"

# Create fleet with description
beacon fleet create test_fleet --name "Test Fleet" --description "Testing deployment"
```

### Add Site to Fleet

Add a deployment site to a fleet:

```bash
beacon fleet add-site [OPTIONS] <FLEET_ID>
```

**Options:**
- `--site-config <FILE>` - Site configuration file
- `--data-dir <DIR>` - Fleet data directory [default: fleet_data/]

**Examples:**
```bash
# Add site to fleet
beacon fleet add-site north_atlantic_fleet --site-config site-001.json
```

### Add Beacons to Fleet

Add beacon deployment to a fleet:

```bash
beacon fleet add-beacons [OPTIONS] <FLEET_ID>
```

**Options:**
- `--deployment <DIR>` - Deployment directory
- `--manifest <FILE>` - Deployment manifest file
- `--data-dir <DIR>` - Fleet data directory [default: fleet_data/]

**Examples:**
```bash
# Add deployment to fleet
beacon fleet add-beacons north_atlantic_fleet --deployment deployment/

# Add using manifest file
beacon fleet add-beacons north_atlantic_fleet --manifest deployment-manifest.json
```

### Fleet Status

Show fleet status and summary:

```bash
beacon fleet status [OPTIONS] <FLEET_ID>
```

**Options:**
- `--detailed` - Show detailed beacon information
- `--data-dir <DIR>` - Fleet data directory [default: fleet_data/]
- `--format <FORMAT>` - Output format (text, json, yaml) [default: text]

**Examples:**
```bash
# Show fleet summary
beacon fleet status north_atlantic_fleet

# Show detailed fleet status
beacon fleet status north_atlantic_fleet --detailed

# Show status in JSON format
beacon fleet status north_atlantic_fleet --format json
```

### Monitor Fleet

Monitor fleet health and status:

```bash
beacon fleet monitor [OPTIONS] <FLEET_ID>
```

**Options:**
- `--interval <SECONDS>` - Update interval [default: 300]
- `--alerts-only` - Show only alerts and warnings
- `--data-dir <DIR>` - Fleet data directory [default: fleet_data/]
- `--log-file <FILE>` - Log monitoring data to file

**Examples:**
```bash
# Monitor fleet with 5-minute intervals
beacon fleet monitor north_atlantic_fleet

# Monitor with custom interval
beacon fleet monitor north_atlantic_fleet --interval 60

# Monitor and log to file
beacon fleet monitor north_atlantic_fleet --log-file fleet-monitor.log
```

### Generate Fleet Report

Generate comprehensive fleet report:

```bash
beacon fleet report [OPTIONS] <FLEET_ID>
```

**Options:**
- `--output <FILE>` - Output report file
- `--format <FORMAT>` - Report format (json, yaml, html, pdf) [default: json]
- `--data-dir <DIR>` - Fleet data directory [default: fleet_data/]
- `--include-history` - Include historical data
- `--time-range <RANGE>` - Time range for report (1h, 24h, 7d, 30d)

**Examples:**
```bash
# Generate basic fleet report
beacon fleet report north_atlantic_fleet --output fleet-report.json

# Generate HTML report with history
beacon fleet report north_atlantic_fleet --format html --include-history --output report.html

# Generate report for last 7 days
beacon fleet report north_atlantic_fleet --time-range 7d --output weekly-report.json
```

## Diagnostic Commands

### Run Diagnostics

Run comprehensive system diagnostics:

```bash
beacon diagnostic [OPTIONS]
```

**Options:**
- `--all` - Run all diagnostic tests
- `--gps` - Test GPS functionality
- `--power` - Test power system
- `--communication` - Test communication system
- `--transceiver` - Test transceiver
- `--memory` - Test memory usage
- `--hardware` - Test hardware components
- `--output <FILE>` - Save diagnostic report to file
- `--format <FORMAT>` - Report format (text, json, yaml) [default: text]

**Examples:**
```bash
# Run all diagnostics
beacon diagnostic --all

# Test specific components
beacon diagnostic --gps --power

# Generate diagnostic report
beacon diagnostic --all --output diagnostic-report.json --format json
```

### System Health Check

Check overall system health:

```bash
beacon health [OPTIONS]
```

**Options:**
- `--threshold <THRESHOLD>` - Health threshold [default: 0.8]
- `--detailed` - Show detailed health metrics
- `--history` - Show health history
- `--format <FORMAT>` - Output format (text, json, yaml) [default: text]

**Examples:**
```bash
# Basic health check
beacon health

# Detailed health check with history
beacon health --detailed --history

# Health check with custom threshold
beacon health --threshold 0.9
```

### Performance Analysis

Analyze system performance:

```bash
beacon performance [OPTIONS]
```

**Options:**
- `--duration <SECONDS>` - Analysis duration [default: 300]
- `--metrics <METRICS>` - Specific metrics (cpu, memory, power, transmission)
- `--output <FILE>` - Save performance data to file
- `--real-time` - Show real-time performance data

**Examples:**
```bash
# Basic performance analysis
beacon performance

# Analyze specific metrics for 10 minutes
beacon performance --duration 600 --metrics cpu,memory,power

# Real-time performance monitoring
beacon performance --real-time
```

## Utility Commands

### Backup Configuration

Create configuration backup:

```bash
beacon backup [OPTIONS]
```

**Options:**
- `--output <FILE>` - Backup file path
- `--include-logs` - Include log files in backup
- `--compress` - Compress backup file

**Examples:**
```bash
# Create basic backup
beacon backup --output beacon-backup.tar.gz

# Create compressed backup with logs
beacon backup --output full-backup.tar.gz --include-logs --compress
```

### Restore Configuration

Restore configuration from backup:

```bash
beacon restore [OPTIONS] <BACKUP_FILE>
```

**Options:**
- `--verify` - Verify backup before restore
- `--force` - Force restore without confirmation

**Examples:**
```bash
# Restore from backup with verification
beacon restore beacon-backup.tar.gz --verify

# Force restore without confirmation
beacon restore beacon-backup.tar.gz --force
```

### Export Configuration

Export configuration in different formats:

```bash
beacon export [OPTIONS]
```

**Options:**
- `--format <FORMAT>` - Export format (toml, json, yaml, env)
- `--output <FILE>` - Output file path
- `--section <SECTION>` - Export specific section only

**Examples:**
```bash
# Export to JSON
beacon export --format json --output beacon-config.json

# Export GPS configuration to YAML
beacon export --format yaml --section gps --output gps-config.yaml

# Export as environment variables
beacon export --format env --output beacon.env
```

### Import Configuration

Import configuration from different formats:

```bash
beacon import [OPTIONS] <INPUT_FILE>
```

**Options:**
- `--format <FORMAT>` - Input format (toml, json, yaml, env)
- `--merge` - Merge with existing configuration
- `--validate` - Validate after import

**Examples:**
```bash
# Import JSON configuration
beacon import --format json beacon-config.json

# Merge YAML configuration
beacon import --format yaml --merge additional-config.yaml --validate
```

## Configuration File Format

### Basic Configuration Structure

```toml
# Beacon Configuration File
beacon_id = "12345678-1234-5678-9abc-123456789abc"

[transmission]
interval_ms = 5000
power_level = 80
message_version = "V3"
retry_attempts = 3

[gps]
acquisition_timeout_s = 60
update_interval_s = 30
min_satellite_count = 4
accuracy_threshold_m = 5.0
cold_start_timeout_s = 300
power_save_enabled = true

[power]
low_battery_threshold_percent = 20.0
critical_battery_threshold_percent = 10.0
emergency_battery_threshold_percent = 5.0
power_save_mode_threshold_percent = 30.0
charging_enabled = true
solar_charging_enabled = false
temperature_monitoring_enabled = true
max_charging_current_ma = 500.0

[communication]
connection_interval_hours = 24
retry_attempts = 3
retry_backoff_ms = 5000
max_retry_interval_hours = 6
connection_timeout_s = 60
data_compression_enabled = true
encryption_enabled = true
max_report_size_bytes = 1048576

[emergency]
emergency_transmission_interval_ms = 1000
emergency_power_threshold_percent = 5.0
emergency_gps_timeout_s = 300
emergency_communication_timeout_s = 1800
auto_shutdown_enabled = true
emergency_message_count = 10
shutdown_delay_s = 30
emergency_contacts = ["emergency@example.com"]

[hardware]
power_management_enabled = true
watchdog_enabled = true
watchdog_timeout_s = 60

[hardware.gpio_pin_mapping]
gps_enable = 2
power_monitor = 4
status_led = 16
emergency_button = 0

[hardware.spi_config]
frequency_hz = 1000000
mode = 0

[hardware.i2c_config]
frequency_hz = 100000
address = 0x48

[hardware.uart_config]
baud_rate = 9600
data_bits = 8
stop_bits = 1
parity = "None"
```

### Site Configuration Format

```json
{
  "site_id": "site_001",
  "name": "North Atlantic Site",
  "location": {
    "latitude": 40.7128,
    "longitude": -74.0060,
    "altitude": 0.0
  },
  "deployment_type": "Surface",
  "environmental_conditions": {
    "temperature_range_c": [5.0, 25.0],
    "wave_height_m": 2.0,
    "current_speed_ms": 0.8,
    "salinity_ppt": 35.0,
    "depth_m": 0.0
  },
  "expected_beacons": 10,
  "deployment_date": "2024-01-15T10:00:00Z",
  "maintenance_schedule": {
    "interval_days": 30,
    "next_maintenance": "2024-02-15T10:00:00Z"
  }
}
```

## Examples and Use Cases

### Quick Start Example

```bash
# 1. Generate default configuration
beacon generate-config --output my-beacon.toml

# 2. Validate configuration
beacon validate-config --config my-beacon.toml

# 3. Check beacon status
beacon status --config my-beacon.toml

# 4. Run diagnostics
beacon diagnostic --all --config my-beacon.toml

# 5. Monitor beacon
beacon monitor --config my-beacon.toml --interval 30
```

### Deployment Workflow Example

```bash
# 1. Create deployment configurations
beacon deploy generate-configs --count 5 --site-config site.json --output deployment/

# 2. Validate deployment
beacon deploy validate-deployment --deployment deployment/ --test-duration 300

# 3. Create fleet
beacon fleet create my_deployment --name "My Deployment"

# 4. Add deployment to fleet
beacon fleet add-beacons my_deployment --deployment deployment/

# 5. Monitor fleet
beacon fleet monitor my_deployment --interval 300
```

### Troubleshooting Example

```bash
# 1. Check system health
beacon health --detailed

# 2. Run comprehensive diagnostics
beacon diagnostic --all --output diagnostic-report.json

# 3. Analyze performance
beacon performance --duration 600 --metrics cpu,memory,power

# 4. Check configuration
beacon validate-config --strict --check-hardware

# 5. Monitor for issues
beacon monitor --alerts-only --log-file issues.log
```

### Configuration Management Example

```bash
# 1. Backup current configuration
beacon backup --output backup-$(date +%Y%m%d).tar.gz --include-logs

# 2. Update configuration
beacon config update transmission.interval_ms 10000
beacon config update power.low_battery_threshold_percent 15.0

# 3. Validate changes
beacon validate-config --strict

# 4. Export configuration for documentation
beacon export --format json --output current-config.json

# 5. Test configuration
beacon deploy test-deployment --duration 300
```

### Fleet Management Example

```bash
# 1. Create fleet with multiple sites
beacon fleet create ocean_deployment --name "Ocean Deployment"
beacon fleet add-site ocean_deployment --site-config site-north.json
beacon fleet add-site ocean_deployment --site-config site-south.json

# 2. Deploy beacons to sites
beacon fleet add-beacons ocean_deployment --deployment north-deployment/
beacon fleet add-beacons ocean_deployment --deployment south-deployment/

# 3. Monitor fleet health
beacon fleet monitor ocean_deployment --interval 300 --log-file fleet.log

# 4. Generate regular reports
beacon fleet report ocean_deployment --format html --include-history --output weekly-report.html

# 5. Check fleet status
beacon fleet status ocean_deployment --detailed
```

This comprehensive CLI documentation provides users with all the information needed to effectively use the beacon system's command-line interface for configuration, deployment, monitoring, and management tasks.