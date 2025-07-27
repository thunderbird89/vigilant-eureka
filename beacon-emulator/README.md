# Beacon Emulator

A comprehensive CLI-based virtual beacon emulator for testing the underwater positioning system without requiring physical hardware.

## Overview

The beacon emulator creates and manages virtual beacons that transmit positioning messages using the same formats and protocols as real beacons. This enables comprehensive testing of receiver systems, positioning algorithms, and system integration in controlled environments.

## Features

- **Virtual Beacon Management**: Create, configure, and manage multiple virtual beacons with full lifecycle control
- **Movement Patterns**: Support for stationary, linear, circular, and random movement patterns with real-time position updates
- **Protocol Compatibility**: Uses the same message formats (V1, V2, V3) as real beacons with identical timing characteristics
- **Virtual Communication Space**: Shared communication medium enabling virtual receivers to connect and receive beacon signals
- **Test Scenarios**: Predefined beacon arrangements (triangle, square, line, grid) for systematic testing
- **Real-time Monitoring**: Live status monitoring with transmission statistics and position tracking
- **Comprehensive Logging**: Activity logging with export capabilities in JSON and CSV formats
- **Configuration Support**: Load beacon configurations from existing TOML/JSON config files
- **Performance Optimization**: Rate limiting, collision avoidance, and resource management for high beacon counts
- **Automation Support**: Batch operations, scripting support, and CI/CD integration
- **Daemon Mode**: Background operation for continuous beacon transmission

## Installation

The beacon emulator is part of the underwater positioning system workspace. Build it using:

```bash
cargo build -p beacon-emulator
```

Or run directly:

```bash
cargo run -p beacon-emulator -- [COMMAND] [OPTIONS]
```

## Quick Start

```bash
# 1. Create a virtual beacon
beacon-emulator create --lat 32.123 --lon 45.476 --depth 10.0

# 2. Start daemon mode for transmission
beacon-emulator daemon --auto-start &

# 3. Monitor beacon activity
beacon-emulator monitor

# 4. Connect virtual receiver (in another terminal)
cd receiver && cargo run -- virtual --channel default

# 5. Export test data
beacon-emulator export --output test_results.json --duration 300

# 6. Clean up
beacon-emulator stop-all --remove
```

## Documentation

### User Documentation
- **[CLI Reference](../docs/manuals/beacon-emulator-cli.md)**: Complete command-line interface documentation
- **[Tutorials](../docs/tutorials/beacon-emulator-tutorials.md)**: Step-by-step guides for common testing scenarios
- **[Troubleshooting Guide](../docs/troubleshooting/beacon-emulator-troubleshooting.md)**: Common issues and solutions

### Developer Documentation
- **[API Documentation](../docs/api/beacon-emulator-api.md)**: Programmatic API for extensions and integration
- **[Virtual Receiver Integration](../docs/virtual-receiver-integration.md)**: Connecting virtual receivers to emulator

### Key Concepts

**Virtual Beacons**: Software implementations of physical beacons that transmit positioning messages with configurable positions, movement patterns, and transmission parameters.

**Virtual Communication Space**: A shared medium where virtual beacons broadcast messages and virtual receivers can subscribe to receive them, enabling end-to-end testing without physical hardware.

**Daemon Mode**: Background operation mode where the emulator actively manages beacon transmission tasks. Beacons are only "intended" to run until daemon mode is started.

**Movement Patterns**: Configurable beacon movement including stationary, linear motion, circular patterns, and random walk for testing dynamic positioning scenarios.

## Usage Examples

### Basic Beacon Management

```bash
# Create stationary beacon
beacon-emulator create --lat 32.123 --lon 45.476 --depth 10.0

# Create moving beacon
beacon-emulator create --lat 32.0 --lon 45.0 --movement linear:1.5:45

# List all beacons
beacon-emulator list --detailed

# Start specific beacon
beacon-emulator start <beacon-id>

# Update beacon position
beacon-emulator update <beacon-id> --position 32.456,45.789,15.0
```

### Test Scenarios

```bash
# Create triangular test arrangement
beacon-emulator scenario triangle --spacing 100 --start-all

# Create grid for stress testing
beacon-emulator scenario grid --count 25 --spacing 50

# Create line formation
beacon-emulator scenario line --count 5 --spacing 75
```

### Monitoring and Analysis

```bash
# Real-time monitoring
beacon-emulator monitor --show-messages --interval 2

# Performance metrics
beacon-emulator performance --detailed

# Export activity logs
beacon-emulator export --output results.json --include-messages --duration 1800
```

### Automation

```bash
# Batch operations
beacon-emulator batch --file test_scenario.json

# CI/CD integration
beacon-emulator --automation-mode reset --force
beacon-emulator --automation-mode scenario triangle --start-all
beacon-emulator --automation-mode daemon --background &
```

## Project Structure

```
beacon-emulator/
├── src/
│   ├── main.rs                    # CLI entry point
│   ├── lib.rs                     # Library exports
│   ├── cli.rs                     # Command-line interface
│   ├── error.rs                   # Error types
│   ├── emulator.rs                # Main emulator manager
│   ├── virtual_beacon.rs          # Virtual beacon implementation
│   ├── virtual_channel.rs         # Communication space
│   ├── movement.rs                # Movement patterns
│   ├── scenario.rs                # Test scenarios
│   ├── scenario_templates.rs      # Predefined scenario templates
│   ├── monitor.rs                 # Real-time monitoring
│   ├── export.rs                  # Log export functionality
│   ├── batch.rs                   # Batch operations
│   ├── performance.rs             # Performance monitoring
│   ├── config.rs                  # Configuration management
│   ├── logging.rs                 # Logging infrastructure
│   ├── ipc_server.rs              # IPC server for virtual receivers
│   └── daemon_server.rs           # Daemon mode implementation
├── tests/
│   ├── integration_tests.rs       # Integration test suite
│   ├── unit_tests.rs              # Unit tests
│   ├── performance_tests.rs       # Performance benchmarks
│   ├── cli_workflow_tests.rs      # CLI workflow tests
│   └── mock_framework.rs          # Testing utilities
├── data/
│   └── emulator_state.json        # Persistent state file
├── Cargo.toml                     # Dependencies and configuration
└── README.md                      # This file
```

## Key Dependencies

- **clap**: Advanced command-line argument parsing with subcommands
- **tokio**: Async runtime for concurrent beacon execution and real-time operations
- **uuid**: Beacon ID generation and management
- **serde**: Configuration serialization and data export
- **shared-positioning**: Core positioning system library for message formats
- **chrono**: Timestamp handling and time-based operations
- **tracing**: Structured logging and diagnostics

## Development Status

The beacon emulator is fully implemented and tested:

- ✅ **Core Infrastructure**: Project structure, dependencies, and error handling
- ✅ **Virtual Beacon System**: Complete virtual beacon implementation with lifecycle management
- ✅ **Communication Space**: Virtual channels with message broadcasting and IPC server
- ✅ **CLI Interface**: Comprehensive command-line interface with all planned features
- ✅ **Movement Patterns**: All movement types (stationary, linear, circular, random)
- ✅ **Test Scenarios**: Predefined geometric arrangements and scenario templates
- ✅ **Monitoring & Logging**: Real-time monitoring and comprehensive logging system
- ✅ **Performance Features**: Rate limiting, collision avoidance, and resource optimization
- ✅ **Automation Support**: Batch operations, scripting, and CI/CD integration
- ✅ **Virtual Receiver Integration**: IPC server and virtual transceiver support
- ✅ **Testing Framework**: Comprehensive test suite with unit, integration, and performance tests
- ✅ **Documentation**: Complete user and developer documentation

## Integration

The beacon emulator integrates seamlessly with:

- **shared-positioning**: Core library for message formats and beacon configuration
- **Virtual receivers**: IPC-based connection for end-to-end testing
- **CI/CD pipelines**: Automation-friendly commands and exit codes
- **Testing frameworks**: Mock components and test utilities
- **Monitoring systems**: Performance metrics and structured logging

## Performance

The emulator is optimized for:
- **High beacon counts**: Supports 50+ concurrent virtual beacons
- **Message throughput**: Efficient message broadcasting and channel management
- **Resource usage**: Memory management and CPU optimization
- **Scalability**: Rate limiting and collision avoidance for dense deployments

## Contributing

When contributing to the beacon emulator:

1. Run the full test suite: `cargo test -p beacon-emulator`
2. Test CLI workflows: `cargo test -p beacon-emulator cli_workflow_tests`
3. Validate performance: `cargo test -p beacon-emulator performance_tests`
4. Update documentation for new features
5. Ensure automation compatibility with `--automation-mode` flag

## License

Part of the underwater positioning system project.