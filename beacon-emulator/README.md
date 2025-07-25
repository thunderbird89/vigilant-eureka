# Beacon Emulator

A CLI-based virtual beacon emulator for testing the underwater positioning system without requiring physical hardware.

## Overview

The beacon emulator creates and manages virtual beacons that transmit positioning messages using the same formats and protocols as real beacons. This enables comprehensive testing of receiver systems, positioning algorithms, and system integration.

## Features

- **Virtual Beacon Management**: Create, configure, and manage multiple virtual beacons
- **Movement Patterns**: Support for stationary, linear, circular, and random movement patterns
- **Protocol Compatibility**: Uses the same message formats (V1, V2, V3) as real beacons
- **Virtual Communication Space**: Shared communication medium for virtual receivers
- **Test Scenarios**: Predefined beacon arrangements (triangle, square, line, grid)
- **Monitoring & Logging**: Real-time status monitoring and activity logging
- **Configuration Support**: Load beacon configurations from existing config files

## Installation

The beacon emulator is part of the underwater positioning system workspace. Build it using:

```bash
cargo build -p beacon-emulator
```

## Usage

### Basic Commands

```bash
# Show help
beacon-emulator --help

# Create a virtual beacon
beacon-emulator create --lat 32.123 --lon 45.476 --depth 10.0

# List active beacons
beacon-emulator list

# Stop a specific beacon
beacon-emulator stop <beacon-id>

# Stop all beacons
beacon-emulator stop-all
```

### Advanced Usage

```bash
# Create beacon with custom configuration
beacon-emulator create --lat 32.123 --lon 45.476 --config beacon.toml

# Create beacon with movement pattern
beacon-emulator create --lat 32.123 --lon 45.476 --movement linear

# Create test scenario
beacon-emulator scenario triangle --count 3 --spacing 100.0

# Monitor beacon activity
beacon-emulator monitor

# Export activity logs
beacon-emulator export --output logs.json --format json
```

## Project Structure

```
beacon-emulator/
├── src/
│   ├── main.rs              # CLI entry point
│   ├── lib.rs               # Library exports
│   ├── cli.rs               # Command-line interface
│   ├── error.rs             # Error types
│   ├── emulator.rs          # Main emulator manager
│   ├── virtual_beacon.rs    # Virtual beacon implementation
│   ├── virtual_channel.rs   # Communication space
│   ├── movement.rs          # Movement patterns
│   ├── scenario.rs          # Test scenarios
│   └── export.rs            # Log export functionality
├── Cargo.toml               # Dependencies and configuration
└── README.md                # This file
```

## Dependencies

- **clap**: Command-line argument parsing
- **tokio**: Async runtime for concurrent beacon execution
- **uuid**: Beacon ID generation and handling
- **serde**: Configuration serialization
- **shared-positioning**: Core positioning system library

## Development Status

This is the initial project structure setup. Core functionality will be implemented in subsequent development phases:

1. ✅ Project structure and dependencies
2. 🔄 Core data models and error types
3. 🔄 Virtual communication space
4. 🔄 Virtual beacon implementation
5. 🔄 CLI command implementation
6. 🔄 Movement patterns and scenarios
7. 🔄 Monitoring and logging
8. 🔄 Testing and documentation

## Integration

The beacon emulator integrates with:
- **shared-positioning**: Core library for message formats and beacon configuration
- **Virtual receivers**: Can connect to the same virtual communication channels
- **CI/CD pipelines**: Supports automated testing workflows