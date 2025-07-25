# CLI Tools Manual

This manual provides comprehensive documentation for all command-line tools included with the underwater positioning beacon system. Each tool is designed for specific aspects of beacon management, deployment, and monitoring.

## Table of Contents

- [Overview](#overview)
- [Installation and Setup](#installation-and-setup)
- [Main Beacon CLI](#main-beacon-cli)
- [Deployment Tools](#deployment-tools)
- [Fleet Management Tools](#fleet-management-tools)
- [Diagnostic Tools](#diagnostic-tools)
- [Configuration Tools](#configuration-tools)
- [Monitoring Tools](#monitoring-tools)
- [Utility Tools](#utility-tools)
- [Scripting and Automation](#scripting-and-automation)
- [Integration Examples](#integration-examples)

## Overview

The beacon system includes several CLI tools for different aspects of system management:

| Tool | Purpose | Primary Users |
|------|---------|---------------|
| `beacon` | Main beacon control and configuration | Operators, Developers |
| `beacon-deploy` | Deployment and fleet management | Deployment Engineers |
| `beacon-monitor` | System monitoring and alerting | Operations Teams |
| `beacon-diagnostic` | System diagnostics and troubleshooting | Support Engineers |
| `beacon-config` | Configuration management | System Administrators |
| `beacon-test` | Testing and validation | QA Engineers, Developers |

## Installation and Setup

### Building the Tools

```bash
# Build all CLI tools
cargo build --release --bins

# Install to system path (optional)
cargo install --path . --bins

# Or install specific tools
cargo install --path beacon --bin beacon
cargo install --path deployment --bin beacon-deploy
```

### Environment Setup

```bash
# Set default configuration directory
export BEACON_CONFIG_DIR="$HOME/.beacon"

# Set default data directory
export BEACON_DATA_DIR="$HOME/.beacon/data"

# Set log level
export BEACON_LOG_LEVEL="info"

# Add to shell profile for persistence
echo 'export BEACON_CONFIG_DIR="$HOME/.beacon"' >> ~/.bashrc
```

### Shell Completion

```bash
# Generate bash completion
beacon completion bash > ~/.beacon-completion.bash
echo 'source ~/.beacon-completion.bash' >> ~/.bashrc

# Generate zsh completion
beacon completion zsh > ~/.beacon-completion.zsh
echo 'source ~/.beacon-completion.zsh' >> ~/.zshrc

# Generate fish completion
beacon completion fish > ~/.config/fish/completions/beacon.fish
```

## Main Beacon CLI

The primary `beacon` command provides core functionality for beacon management.

### Basic Usage

```bash
beacon [GLOBAL_OPTIONS] <COMMAND> [COMMAND_OPTIONS]
```

### Global Options

```
-c, --config <FILE>     Configuration file path [default: beacon.toml]
-v, --verbose           Enable verbose output
-q, --quiet             Suppress non-error output
--log-level <LEVEL>     Set log level (error, warn, info, debug, trace)
--log-file <FILE>       Write logs to file
--no-color              Disable colored output
-h, --help              Print help information
-V, --version           Print version information
```

### Configuration Commands

#### Generate Configuration

```bash
beacon generate-config [OPTIONS]

Options:
  -o, --output <FILE>         Output file path [default: beacon-default.toml]
  -t, --template <TEMPLATE>   Configuration template (basic, marine, test, low-power)
  --beacon-id <UUID>          Set specific beacon ID
  --site-config <FILE>        Site configuration file
  --hardware <TYPE>           Hardware type (esp01, esp32, generic)
```

**Examples:**
```bash
# Generate basic configuration
beacon generate-config

# Generate marine deployment configuration
beacon generate-config --template marine --output marine-beacon.toml

# Generate configuration for specific hardware
beacon generate-config --hardware esp01 --output esp01-beacon.toml

# Generate with site-specific settings
beacon generate-config --site-config site.json --output site-beacon.toml
```

#### Validate Configuration

```bash
beacon validate-config [OPTIONS]

Options:
  -c, --config <FILE>     Configuration file to validate
  --strict                Enable strict validation mode
  --check-hardware        Validate hardware-specific settings
  --check-compatibility   Check receiver compatibility
  --output <FORMAT>       Output format (text, json, yaml)
```

**Examples:**
```bash
# Basic validation
beacon validate-config

# Strict validation with hardware checks
beacon validate-config --strict --check-hardware

# Validate and output results as JSON
beacon validate-config --output json > validation-results.json
```

#### Configuration Management

```bash
# Show current configuration
beacon config show [--section <SECTION>] [--format <FORMAT>]

# Update configuration parameter
beacon config update <KEY> <VALUE>

# Reset configuration section
beacon config reset [--section <SECTION>] [--backup]

# Import configuration
beacon config import <FILE> [--format <FORMAT>] [--merge]

# Export configuration
beacon config export [--format <FORMAT>] [--output <FILE>]
```

**Examples:**
```bash
# Show GPS configuration
beacon config show --section gps

# Update transmission interval
beacon config update transmission.interval_ms 10000

# Reset power configuration with backup
beacon config reset --section power --backup

# Export configuration as JSON
beacon config export --format json --output config.json
```

### Status and Monitoring Commands

#### Status Check

```bash
beacon status [OPTIONS]

Options:
  -d, --detailed          Show detailed status information
  --gps                   Show GPS status only
  --power                 Show power status only
  --communication         Show communication status only
  --transmission          Show transmission statistics only
  --system                Show system health information
  --format <FORMAT>       Output format (text, json, yaml, table)
  --refresh <SECONDS>     Auto-refresh interval
```

**Examples:**
```bash
# Basic status
beacon status

# Detailed status with auto-refresh
beacon status --detailed --refresh 30

# GPS status in JSON format
beacon status --gps --format json

# System health monitoring
beacon status --system --refresh 10
```

#### Real-time Monitoring

```bash
beacon monitor [OPTIONS]

Options:
  -i, --interval <SECONDS>    Update interval [default: 10]
  --alerts-only               Show only alerts and warnings
  --threshold <THRESHOLD>     Health threshold for alerts [default: 0.8]
  --log-file <FILE>           Log monitoring data to file
  --email-alerts <EMAIL>      Send email alerts
  --webhook <URL>             Send webhook notifications
  --dashboard                 Launch web dashboard
```

**Examples:**
```bash
# Basic monitoring
beacon monitor

# Monitor with alerts only
beacon monitor --alerts-only --threshold 0.9

# Monitor with logging and email alerts
beacon monitor --log-file monitor.log --email-alerts admin@example.com

# Launch web dashboard
beacon monitor --dashboard --port 8080
```

### Diagnostic Commands

#### System Diagnostics

```bash
beacon diagnostic [OPTIONS]

Options:
  --all                   Run all diagnostic tests
  --gps                   Test GPS functionality
  --power                 Test power system
  --communication         Test communication system
  --transceiver           Test transceiver
  --memory                Test memory usage
  --hardware              Test hardware components
  --performance           Run performance tests
  --output <FILE>         Save diagnostic report to file
  --format <FORMAT>       Report format (text, json, yaml, html)
```

**Examples:**
```bash
# Run all diagnostics
beacon diagnostic --all

# Test specific components
beacon diagnostic --gps --power --communication

# Generate HTML diagnostic report
beacon diagnostic --all --format html --output diagnostic-report.html
```

#### Performance Analysis

```bash
beacon performance [OPTIONS]

Options:
  --duration <SECONDS>        Analysis duration [default: 300]
  --metrics <METRICS>         Specific metrics (cpu, memory, power, transmission, gps)
  --real-time                 Show real-time performance data
  --baseline                  Establish performance baseline
  --compare <FILE>            Compare with baseline file
  --output <FILE>             Save performance data to file
```

**Examples:**
```bash
# Basic performance analysis
beacon performance --duration 600

# Real-time performance monitoring
beacon performance --real-time --metrics cpu,memory,power

# Establish baseline
beacon performance --baseline --duration 1800 --output baseline.json

# Compare with baseline
beacon performance --compare baseline.json --duration 600
```

#### Health Check

```bash
beacon health [OPTIONS]

Options:
  --threshold <THRESHOLD>     Health threshold [default: 0.8]
  --detailed                  Show detailed health metrics
  --history                   Show health history
  --continuous                Continuous health monitoring
  --alert-email <EMAIL>       Email for health alerts
  --alert-webhook <URL>       Webhook for health alerts
```

**Examples:**
```bash
# Basic health check
beacon health

# Detailed health with history
beacon health --detailed --history

# Continuous monitoring with alerts
beacon health --continuous --alert-email admin@example.com
```

### Control Commands

#### Beacon Control

```bash
# Start beacon operation
beacon start [--config <FILE>] [--daemon] [--pid-file <FILE>]

# Stop beacon operation
beacon stop [--graceful] [--timeout <SECONDS>]

# Restart beacon
beacon restart [--graceful] [--config <FILE>]

# Emergency shutdown
beacon emergency-shutdown [--reason <REASON>]
```

**Examples:**
```bash
# Start beacon as daemon
beacon start --daemon --pid-file /var/run/beacon.pid

# Graceful stop with 30-second timeout
beacon stop --graceful --timeout 30

# Emergency shutdown with reason
beacon emergency-shutdown --reason "Battery critically low"
```

#### Configuration Updates

```bash
# Reload configuration
beacon reload-config [--validate] [--backup]

# Update configuration remotely
beacon remote-update --beacon-id <UUID> --config <FILE>

# Batch configuration update
beacon batch-update --beacons <FILE> --config <FILE>
```

## Deployment Tools

The `beacon-deploy` tool handles deployment-specific operations.

### Deployment Configuration Generation

```bash
beacon-deploy generate-configs [OPTIONS]

Options:
  --count <COUNT>             Number of beacon configurations
  --output <DIR>              Output directory [default: deployment/]
  --site-config <FILE>        Site configuration file
  --template <FILE>           Configuration template file
  --prefix <PREFIX>           Beacon ID prefix
  --sequential-ids            Use sequential beacon IDs
  --spacing <METERS>          Minimum beacon spacing
  --coverage-area <FILE>      Coverage area definition
```

**Examples:**
```bash
# Generate 10 beacon configurations
beacon-deploy generate-configs --count 10

# Generate with site-specific settings
beacon-deploy generate-configs --count 5 --site-config marine-site.json

# Generate with specific spacing
beacon-deploy generate-configs --count 8 --spacing 1000 --coverage-area area.geojson
```

### Deployment Validation

```bash
beacon-deploy validate-deployment [OPTIONS]

Options:
  --deployment <DIR>          Deployment directory
  --test-duration <SECONDS>   Test duration [default: 300]
  --parallel                  Run validation tests in parallel
  --report <FILE>             Generate validation report
  --strict                    Enable strict validation
  --hardware-test             Include hardware testing
```

**Examples:**
```bash
# Validate deployment
beacon-deploy validate-deployment --deployment my-deployment/

# Extended validation with hardware testing
beacon-deploy validate-deployment --test-duration 1800 --hardware-test

# Generate validation report
beacon-deploy validate-deployment --report validation-report.html --format html
```

### Deployment Testing

```bash
beacon-deploy test-deployment [OPTIONS]

Options:
  --deployment <DIR>          Deployment directory
  --duration <SECONDS>        Test duration [default: 600]
  --scenario <SCENARIO>       Test scenario (normal, stress, failure, environmental)
  --mock-hardware             Use mock hardware interfaces
  --coverage-test             Test coverage area
  --interference-test         Test interference patterns
```

**Examples:**
```bash
# Test deployment with normal scenario
beacon-deploy test-deployment --duration 1800

# Stress test with mock hardware
beacon-deploy test-deployment --scenario stress --mock-hardware

# Test coverage area
beacon-deploy test-deployment --coverage-test --duration 3600
```

## Fleet Management Tools

Fleet management commands are integrated into the main beacon CLI.

### Fleet Creation and Management

```bash
# Create new fleet
beacon fleet create <FLEET_ID> [OPTIONS]

Options:
  --name <NAME>               Fleet display name
  --description <DESC>        Fleet description
  --data-dir <DIR>            Fleet data directory
  --template <TEMPLATE>       Fleet template
```

**Examples:**
```bash
# Create basic fleet
beacon fleet create ocean_deployment --name "Ocean Deployment"

# Create fleet with template
beacon fleet create test_fleet --template testing --data-dir test-data/
```

### Site Management

```bash
# Add site to fleet
beacon fleet add-site <FLEET_ID> [OPTIONS]

Options:
  --site-config <FILE>        Site configuration file
  --name <NAME>               Site name
  --location <LAT,LON>        Site coordinates
  --depth <METERS>            Deployment depth
```

**Examples:**
```bash
# Add site with configuration file
beacon fleet add-site ocean_deployment --site-config north-site.json

# Add site with coordinates
beacon fleet add-site ocean_deployment --name "North Site" --location 40.7128,-74.0060
```

### Beacon Management

```bash
# Add beacons to fleet
beacon fleet add-beacons <FLEET_ID> [OPTIONS]

Options:
  --deployment <DIR>          Deployment directory
  --manifest <FILE>           Deployment manifest file
  --site <SITE_ID>            Target site ID
  --auto-assign               Auto-assign beacons to sites
```

**Examples:**
```bash
# Add deployment to fleet
beacon fleet add-beacons ocean_deployment --deployment north-deployment/

# Add with site assignment
beacon fleet add-beacons ocean_deployment --deployment south-deployment/ --site south_site
```

### Fleet Monitoring

```bash
# Show fleet status
beacon fleet status <FLEET_ID> [OPTIONS]

Options:
  --detailed                  Show detailed beacon information
  --site <SITE_ID>            Show specific site only
  --health-only               Show only health information
  --format <FORMAT>           Output format
```

**Examples:**
```bash
# Show fleet summary
beacon fleet status ocean_deployment

# Show detailed status for specific site
beacon fleet status ocean_deployment --site north_site --detailed
```

```bash
# Monitor fleet health
beacon fleet monitor <FLEET_ID> [OPTIONS]

Options:
  --interval <SECONDS>        Update interval [default: 300]
  --alerts-only               Show only alerts and warnings
  --site <SITE_ID>            Monitor specific site only
  --threshold <THRESHOLD>     Health threshold for alerts
  --log-file <FILE>           Log monitoring data to file
```

**Examples:**
```bash
# Monitor entire fleet
beacon fleet monitor ocean_deployment --interval 600

# Monitor with alerts and logging
beacon fleet monitor ocean_deployment --alerts-only --log-file fleet.log
```

### Fleet Reporting

```bash
# Generate fleet report
beacon fleet report <FLEET_ID> [OPTIONS]

Options:
  --output <FILE>             Output report file
  --format <FORMAT>           Report format (json, yaml, html, pdf)
  --include-history           Include historical data
  --time-range <RANGE>        Time range (1h, 24h, 7d, 30d)
  --template <TEMPLATE>       Report template
```

**Examples:**
```bash
# Generate basic report
beacon fleet report ocean_deployment --output fleet-report.json

# Generate HTML report with history
beacon fleet report ocean_deployment --format html --include-history --output report.html

# Generate weekly report
beacon fleet report ocean_deployment --time-range 7d --format pdf --output weekly.pdf
```

## Diagnostic Tools

### System Diagnostics

```bash
beacon-diagnostic system [OPTIONS]

Options:
  --component <COMPONENT>     Test specific component
  --full                      Run comprehensive diagnostics
  --hardware                  Include hardware diagnostics
  --performance               Include performance analysis
  --output <FILE>             Save diagnostic report
```

### Network Diagnostics

```bash
beacon-diagnostic network [OPTIONS]

Options:
  --connectivity              Test network connectivity
  --bandwidth                 Test bandwidth capabilities
  --latency                   Measure network latency
  --interference              Scan for interference
  --range-test                Test transmission range
```

### Hardware Diagnostics

```bash
beacon-diagnostic hardware [OPTIONS]

Options:
  --gpio                      Test GPIO functionality
  --i2c                       Test I2C bus
  --spi                       Test SPI bus
  --memory                    Test memory systems
  --power                     Test power systems
  --sensors                   Test sensor functionality
```

## Configuration Tools

### Configuration Validation

```bash
beacon-config validate [OPTIONS] <CONFIG_FILE>

Options:
  --schema <FILE>             Validation schema file
  --strict                    Enable strict validation
  --hardware <TYPE>           Hardware type for validation
  --output <FORMAT>           Output format for results
```

### Configuration Migration

```bash
beacon-config migrate [OPTIONS] <INPUT_FILE>

Options:
  --from-version <VERSION>    Source configuration version
  --to-version <VERSION>      Target configuration version
  --output <FILE>             Output file for migrated configuration
  --backup                    Create backup of original
```

### Configuration Templates

```bash
beacon-config template [OPTIONS]

Options:
  --list                      List available templates
  --show <TEMPLATE>           Show template contents
  --create <NAME>             Create new template
  --edit <TEMPLATE>           Edit existing template
```

## Monitoring Tools

### Real-time Monitoring

```bash
beacon-monitor realtime [OPTIONS]

Options:
  --metrics <METRICS>         Metrics to monitor
  --interval <SECONDS>        Update interval
  --dashboard                 Launch web dashboard
  --alerts                    Enable alerting
  --export <FORMAT>           Export format for data
```

### Historical Analysis

```bash
beacon-monitor analyze [OPTIONS]

Options:
  --time-range <RANGE>        Analysis time range
  --metrics <METRICS>         Metrics to analyze
  --baseline <FILE>           Baseline for comparison
  --trends                    Show trend analysis
  --anomalies                 Detect anomalies
```

### Alerting

```bash
beacon-monitor alerts [OPTIONS]

Options:
  --configure                 Configure alert rules
  --test                      Test alert configuration
  --history                   Show alert history
  --acknowledge <ID>          Acknowledge alert
```

## Utility Tools

### Backup and Restore

```bash
# Create backup
beacon backup [OPTIONS]

Options:
  --output <FILE>             Backup file path
  --include-logs              Include log files
  --include-data              Include data files
  --compress                  Compress backup
  --encrypt                   Encrypt backup
```

```bash
# Restore from backup
beacon restore [OPTIONS] <BACKUP_FILE>

Options:
  --verify                    Verify backup before restore
  --selective                 Selective restore
  --force                     Force restore without confirmation
```

### Data Export/Import

```bash
# Export data
beacon export [OPTIONS]

Options:
  --type <TYPE>               Data type (config, logs, metrics, status)
  --format <FORMAT>           Export format
  --time-range <RANGE>        Time range for data
  --output <FILE>             Output file
```

```bash
# Import data
beacon import [OPTIONS] <INPUT_FILE>

Options:
  --type <TYPE>               Data type
  --format <FORMAT>           Input format
  --merge                     Merge with existing data
  --validate                  Validate before import
```

### Testing Tools

```bash
# Test beacon functionality
beacon test [OPTIONS]

Options:
  --component <COMPONENT>     Test specific component
  --duration <SECONDS>        Test duration
  --scenario <SCENARIO>       Test scenario
  --mock                      Use mock interfaces
  --stress                    Stress testing
```

## Scripting and Automation

### Batch Operations

```bash
# Batch configuration update
beacon batch-config-update [OPTIONS]

Options:
  --beacons <FILE>            List of beacon IDs
  --config <FILE>             Configuration to apply
  --parallel                  Parallel execution
  --dry-run                   Show what would be done
```

### Scheduled Operations

```bash
# Schedule health checks
beacon schedule health-check [OPTIONS]

Options:
  --interval <INTERVAL>       Check interval (1h, 6h, 24h)
  --fleet <FLEET_ID>          Target fleet
  --alert-threshold <VALUE>   Alert threshold
  --action <ACTION>           Action on failure
```

### Automation Scripts

```bash
# Generate automation scripts
beacon generate-scripts [OPTIONS]

Options:
  --type <TYPE>               Script type (monitoring, maintenance, deployment)
  --platform <PLATFORM>      Target platform (bash, powershell, python)
  --output <DIR>              Output directory
```

## Integration Examples

### CI/CD Integration

```bash
# Validate configuration in CI
beacon validate-config --strict --output json > validation.json
if [ $? -ne 0 ]; then
    echo "Configuration validation failed"
    exit 1
fi

# Deploy in CD pipeline
beacon-deploy validate-deployment --deployment staging/
beacon-deploy test-deployment --duration 300 --scenario normal
beacon fleet add-beacons production --deployment staging/
```

### Monitoring Integration

```bash
# Export metrics for external monitoring
beacon export --type metrics --format prometheus --output metrics.txt

# Send status to monitoring system
beacon status --format json | curl -X POST -H "Content-Type: application/json" \
    -d @- http://monitoring.example.com/api/beacon-status
```

### Alerting Integration

```bash
# Configure webhook alerts
beacon monitor --webhook http://alerts.example.com/webhook \
    --threshold 0.8 --alerts-only

# Email alert configuration
beacon config update alerts.email_enabled true
beacon config update alerts.email_recipients "admin@example.com,ops@example.com"
beacon config update alerts.email_threshold 0.7
```

### Log Management Integration

```bash
# Export logs to external system
beacon export --type logs --format json --time-range 24h | \
    curl -X POST -H "Content-Type: application/json" \
        -d @- http://logs.example.com/api/ingest

# Configure log forwarding
beacon config update logging.forward_enabled true
beacon config update logging.forward_endpoint "http://logs.example.com/api/logs"
beacon config update logging.forward_format "json"
```

### Custom Tool Development

```bash
# Use beacon CLI in custom scripts
#!/bin/bash

# Custom monitoring script
while true; do
    STATUS=$(beacon status --format json)
    HEALTH=$(echo $STATUS | jq '.system_health.overall_health_score')
    
    if (( $(echo "$HEALTH < 0.8" | bc -l) )); then
        echo "Health degraded: $HEALTH"
        beacon diagnostic --all --output "diagnostic-$(date +%s).json"
    fi
    
    sleep 300
done
```

### API Integration

```bash
# REST API wrapper using CLI
function beacon_api() {
    local endpoint=$1
    local method=${2:-GET}
    local data=$3
    
    case $endpoint in
        "status")
            beacon status --format json
            ;;
        "health")
            beacon health --format json
            ;;
        "config")
            beacon config show --format json
            ;;
        *)
            echo "Unknown endpoint: $endpoint"
            return 1
            ;;
    esac
}

# Usage
beacon_api status | jq '.battery_status.capacity_percent'
```

This comprehensive CLI tools manual provides detailed information for effectively using all command-line tools in the beacon system. The tools are designed to work together and can be combined for complex operational workflows.