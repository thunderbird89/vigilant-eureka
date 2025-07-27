# Beacon Emulator Configuration Structure

This document describes the configuration structure for the beacon emulator system. The emulator supports both standard beacon configurations and extended emulator-specific configurations for simulation scenarios.

## Configuration Types

The beacon emulator supports two types of configurations:

1. **Standard Beacon Configuration**: Compatible with physical beacon configs (TOML/JSON/YAML)
2. **Extended Emulator Configuration**: Enhanced with emulator-specific features (JSON/TOML only)

## Standard Beacon Configuration

The emulator can load standard beacon configuration files (see [Beacon Configuration](beacon-configuration.md)) with emulator-optimized defaults:

### Emulator-Optimized Defaults

```toml
[transmission]
interval_ms = 5000                    # 5 second interval for testing
power_level = 255                     # Full power for consistent testing
adaptive_power = false                # Disabled for predictable behavior

[gps]
accuracy_threshold_m = 5.0            # More lenient for virtual GPS
enable_dgps = false                   # Not relevant for simulation

[power]
charging_enabled = false              # Not relevant for virtual beacons
solar_charging_enabled = false        # Not relevant for virtual beacons
power_save_transmission_multiplier = 1.0  # Disabled for testing

[communication]
connection_interval_hours = 24        # Infrequent for testing
compression_enabled = false           # Disabled for simplicity

[emergency]
emergency_mode_enabled = false        # Disabled for testing unless needed
auto_recovery_enabled = false         # Manual control preferred
```

## Extended Emulator Configuration

Enhanced configuration format with emulator-specific features:

### Root Structure

```json
{
  "beacon_config": { /* Standard beacon configuration */ },
  "initial_position": { /* Starting position */ },
  "movement_pattern": "stationary",
  "auto_start": false,
  "name": "Test Beacon 1",
  "tags": ["test", "scenario-1"],
  "metadata": { /* Configuration metadata */ }
}
```

### Beacon Configuration Section

Contains the standard beacon configuration (see [Beacon Configuration](beacon-configuration.md)):

```json
{
  "beacon_config": {
    "beacon_id": "a3ee99ac-b65c-4689-8e42-a0a043e29c5e",
    "transmission": {
      "interval_ms": 5000,
      "message_version": "V3",
      "power_level": 255,
      "max_retries": 3,
      "retry_delay_ms": 1000,
      "adaptive_power": false,
      "sequence_rollover": 65535
    },
    "gps": {
      "acquisition_timeout_s": 60,
      "update_interval_s": 2,
      "min_satellite_count": 4,
      "accuracy_threshold_m": 3.0,
      "cold_start_timeout_s": 120,
      "enable_dgps": false,
      "max_fix_age_s": 30
    }
    /* ... other standard beacon config sections ... */
  }
}
```

### Initial Position

Defines the starting position for the virtual beacon:

```json
{
  "initial_position": {
    "latitude": 32.1225500845841,      # Decimal degrees (WGS84)
    "longitude": 45.47691858833027,    # Decimal degrees (WGS84)
    "depth": 10.0                      # Meters (positive downward)
  }
}
```

**Validation Rules:**
- `latitude`: -90.0 to 90.0 degrees
- `longitude`: -180.0 to 180.0 degrees
- `depth`: 0.0 to 11000.0 meters

### Movement Patterns

Defines how the virtual beacon moves during simulation:

```json
{
  "movement_pattern": "stationary"     # Movement type
}
```

**Available Movement Patterns:**
- `"stationary"`: Beacon remains at initial position
- `"linear"`: Linear movement between waypoints
- `"circular"`: Circular movement pattern
- `"random_walk"`: Random movement within bounds
- `"waypoint"`: Follow predefined waypoints
- `"drift"`: Simulate ocean current drift

### Emulator-Specific Settings

```json
{
  "auto_start": false,                 # Start beacon automatically when loaded
  "name": "Test Beacon 1",             # Human-readable name
  "tags": ["test", "scenario-1"]       # Organization tags
}
```

### Extended Metadata

Enhanced metadata for emulator configurations:

```json
{
  "metadata": {
    "schema_version": "1.0.0",         # Configuration schema version
    "created_at": 1753474380,          # Creation timestamp (Unix)
    "modified_at": 1753474380,         # Last modification timestamp
    "description": "Test beacon for scenario validation",
    "author": "Test Engineer",         # Configuration author
    "checksum": "",                    # Configuration integrity checksum
    "migration_history": []            # Migration history array
  }
}
```

## Emulator State Configuration

The emulator maintains state in JSON format:

### Emulator State Structure

```json
{
  "beacons": [
    {
      "id": "a3ee99ac-b65c-4689-8e42-a0a043e29c5e",
      "position": {
        "latitude": 32.1225500845841,
        "longitude": 45.47691858833027,
        "depth": 10.0
      },
      "config": { /* Full beacon configuration */ },
      "movement_pattern": "stationary",
      "intended_running": true,
      "config_file_path": null,
      "config_checksum": null,
      "config_updated_at": 1753599515
    }
  ],
  "current_channel": "default"
}
```

### Beacon State Fields

- `id`: Unique beacon identifier (UUID)
- `position`: Current beacon position
- `config`: Complete beacon configuration
- `movement_pattern`: Current movement pattern
- `intended_running`: Whether beacon should be running
- `config_file_path`: Path to source configuration file (if any)
- `config_checksum`: Configuration integrity checksum
- `config_updated_at`: Last configuration update timestamp

## Configuration Management

### Loading Configurations

The emulator supports multiple loading methods:

```bash
# Load standard beacon configuration
beacon-emulator --config beacon.toml

# Load extended emulator configuration
beacon-emulator --emulator-config emulator-beacon.json

# Load from state file
beacon-emulator --state-file emulator_state.json
```

### Configuration Validation

The emulator performs enhanced validation:

#### Standard Validation
- All standard beacon configuration rules
- Emulator-specific range adjustments
- Hardware compatibility checks (disabled for virtual hardware)

#### Emulator-Specific Validation
- Position coordinate validation
- Movement pattern validation
- Metadata completeness checks
- Configuration integrity verification

### Validation Rules

```rust
// Emulator-specific validation rules
ValidationRules {
    min_transmission_interval_ms: 100,      // Faster than physical beacons
    max_transmission_interval_ms: 300_000,  // 5 minutes maximum
    min_power_level: 1,
    max_power_level: 255,
    max_retry_attempts: 10,
    min_gps_accuracy_threshold_m: 0.1,      // More precise for testing
    max_gps_accuracy_threshold_m: 100.0,
    min_battery_threshold_percent: 1.0,
    max_battery_threshold_percent: 100.0,
}
```

## Configuration Templates

### Basic Test Beacon

```json
{
  "beacon_config": {
    "beacon_id": "test-beacon-001",
    "transmission": {
      "interval_ms": 5000,
      "message_version": "V3",
      "power_level": 255,
      "max_retries": 3,
      "retry_delay_ms": 1000,
      "adaptive_power": false
    },
    "gps": {
      "acquisition_timeout_s": 30,
      "update_interval_s": 5,
      "min_satellite_count": 3,
      "accuracy_threshold_m": 5.0
    },
    "power": {
      "low_battery_threshold_percent": 20.0,
      "critical_battery_threshold_percent": 10.0,
      "emergency_battery_threshold_percent": 5.0,
      "charging_enabled": false
    }
  },
  "initial_position": {
    "latitude": 32.123,
    "longitude": 45.476,
    "depth": 10.0
  },
  "movement_pattern": "stationary",
  "auto_start": false,
  "name": "Basic Test Beacon",
  "tags": ["test", "basic"],
  "metadata": {
    "schema_version": "1.0.0",
    "description": "Basic test beacon configuration",
    "author": "Test System"
  }
}
```

### Performance Test Beacon

```json
{
  "beacon_config": {
    "beacon_id": "perf-beacon-001",
    "transmission": {
      "interval_ms": 1000,               # High frequency for performance testing
      "message_version": "V3",
      "power_level": 255,
      "max_retries": 1,                  # Minimal retries for speed
      "adaptive_power": false
    },
    "gps": {
      "acquisition_timeout_s": 10,       # Fast GPS acquisition
      "update_interval_s": 1,            # High update rate
      "accuracy_threshold_m": 10.0       # Relaxed accuracy for speed
    }
  },
  "initial_position": {
    "latitude": 32.123,
    "longitude": 45.476,
    "depth": 5.0
  },
  "movement_pattern": "linear",
  "auto_start": true,
  "name": "Performance Test Beacon",
  "tags": ["performance", "high-frequency"]
}
```

## Configuration Migration

The emulator supports configuration migration between versions:

### Migration Process

1. **Version Detection**: Automatically detect configuration schema version
2. **Migration Chain**: Apply migrations in sequence if needed
3. **Validation**: Validate configuration after migration
4. **History Tracking**: Record migration history in metadata

### Migration Example

```json
{
  "metadata": {
    "schema_version": "1.0.0",
    "migration_history": [
      {
        "from_version": "0.9.0",
        "to_version": "1.0.0",
        "timestamp": 1753474380,
        "description": "Migrated to new transmission configuration format"
      }
    ]
  }
}
```

## Best Practices

1. **Use Extended Format**: Prefer extended emulator configuration for complex scenarios
2. **Validate Positions**: Ensure initial positions are realistic for your test environment
3. **Choose Appropriate Intervals**: Use faster intervals for testing, realistic ones for validation
4. **Disable Irrelevant Features**: Turn off power management and charging for virtual beacons
5. **Use Descriptive Names**: Name beacons and use tags for organization
6. **Version Control**: Track configuration changes in version control
7. **Test Configurations**: Validate configurations before running long scenarios
8. **Document Scenarios**: Use metadata description field to document test scenarios

## CLI Configuration Commands

```bash
# Generate default emulator configuration
beacon-emulator generate-config --type emulator --output test-beacon.json

# Validate configuration
beacon-emulator validate-config --config test-beacon.json

# Convert between formats
beacon-emulator convert-config --input beacon.toml --output beacon.json

# Create configuration from template
beacon-emulator create-config --template performance --output perf-beacon.json
```