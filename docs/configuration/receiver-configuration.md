# Receiver Configuration Structure

This document describes the configuration structure for the receiver system. The receiver uses both system-wide positioning configurations and embedded-optimized anchor configurations for real-time underwater positioning.

## Configuration Types

The receiver system uses two main configuration types:

1. **System Configuration**: Global positioning system parameters
2. **Anchor Configuration**: Individual anchor definitions and settings

## System Configuration

Controls global positioning system behavior and performance parameters.

### Basic System Configuration

```rust
pub struct EmbeddedSystemConfig {
    pub sound_speed_mm_per_ms: u16,     // Sound speed in mm/ms (space-optimized)
    pub max_anchor_age_ms: u32,         // Maximum age of anchor data
    pub min_anchors: u8,                // Minimum anchors required
    pub position_timeout_ms: u32,       // Position calculation timeout
    pub accuracy_threshold_mm: u16,     // Accuracy threshold in millimeters
}
```

### Default System Configuration

```rust
EmbeddedSystemConfig {
    sound_speed_mm_per_ms: 1500,        // 1500 m/s = 1.5 mm/ms
    max_anchor_age_ms: 5000,            // 5 seconds maximum age
    min_anchors: 3,                     // Minimum 3 anchors for positioning
    position_timeout_ms: 200,           // 200ms calculation timeout
    accuracy_threshold_mm: 2000,        // 2 meters accuracy threshold
}
```

### System Configuration Parameters

#### Sound Speed Configuration
- **Parameter**: `sound_speed_mm_per_ms`
- **Type**: `u16` (millimeters per millisecond)
- **Range**: 1400-1600 (equivalent to 1400-1600 m/s)
- **Default**: 1500 (1500 m/s)
- **Description**: Speed of sound in water, varies with temperature, salinity, and pressure

#### Anchor Age Limits
- **Parameter**: `max_anchor_age_ms`
- **Type**: `u32` (milliseconds)
- **Range**: 100-60000 (0.1 seconds to 1 minute)
- **Default**: 5000 (5 seconds)
- **Description**: Maximum age of anchor data before considered stale

#### Minimum Anchors
- **Parameter**: `min_anchors`
- **Type**: `u8`
- **Range**: 3-10
- **Default**: 3
- **Description**: Minimum number of anchors required for position calculation

#### Position Timeout
- **Parameter**: `position_timeout_ms`
- **Type**: `u32` (milliseconds)
- **Range**: 10-10000 (10ms to 10 seconds)
- **Default**: 200 (200ms)
- **Description**: Maximum time allowed for position calculation

#### Accuracy Threshold
- **Parameter**: `accuracy_threshold_mm`
- **Type**: `u16` (millimeters)
- **Range**: 100-1000000 (0.1m to 1000m)
- **Default**: 2000 (2 meters)
- **Description**: Required positioning accuracy threshold

## Anchor Configuration

Defines individual anchor positions and operational parameters.

### Embedded Anchor Configuration

```rust
pub struct EmbeddedAnchorConfig {
    pub id: u16,                        // Unique anchor identifier
    pub position: CompactAnchorMessage, // Anchor position data
    pub max_range_mm: u32,              // Maximum range in millimeters
    pub enabled: bool,                  // Whether anchor is enabled
}
```

### Anchor Configuration Parameters

#### Anchor Identification
- **Parameter**: `id`
- **Type**: `u16`
- **Range**: 1-65535
- **Description**: Unique identifier for the anchor

#### Position Data
- **Parameter**: `position`
- **Type**: `CompactAnchorMessage`
- **Fields**:
  - `latitude`: f64 (decimal degrees, WGS84)
  - `longitude`: f64 (decimal degrees, WGS84)
  - `depth`: f64 (meters, positive downward)
- **Validation**:
  - Latitude: -90.0 to 90.0 degrees
  - Longitude: -180.0 to 180.0 degrees
  - Depth: 0.0 to 11000.0 meters

#### Maximum Range
- **Parameter**: `max_range_mm`
- **Type**: `u32` (millimeters)
- **Range**: 1000-100000000 (1m to 100km)
- **Default**: 5000000 (5000m)
- **Description**: Maximum operational range for this anchor

#### Enable/Disable
- **Parameter**: `enabled`
- **Type**: `bool`
- **Default**: `true`
- **Description**: Whether this anchor is active for positioning

## Configuration Presets

### Standard Configuration

For typical underwater positioning applications:

```rust
// System configuration
EmbeddedSystemConfig {
    sound_speed_mm_per_ms: 1500,        // Standard seawater
    max_anchor_age_ms: 5000,            // 5 second freshness
    min_anchors: 3,                     // Basic triangulation
    position_timeout_ms: 200,           // Real-time response
    accuracy_threshold_mm: 2000,        // 2 meter accuracy
}

// Example anchor configuration
EmbeddedAnchorConfig {
    id: 1,
    position: CompactAnchorMessage::new(1, 0, 32.123, 45.476, 10.0, 255),
    max_range_mm: 5000000,              // 5km range
    enabled: true,
}
```

### High-Precision Configuration

For applications requiring high accuracy:

```rust
// System configuration
EmbeddedSystemConfig {
    sound_speed_mm_per_ms: 1500,        // Calibrated for conditions
    max_anchor_age_ms: 10000,           // Allow older data for precision
    min_anchors: 4,                     // 4+ anchors for 3D positioning
    position_timeout_ms: 500,           // Allow more calculation time
    accuracy_threshold_mm: 500,         // 0.5 meter accuracy
}

// Anchor configuration with extended range
EmbeddedAnchorConfig {
    id: 1,
    position: CompactAnchorMessage::new(1, 0, 32.123, 45.476, 10.0, 255),
    max_range_mm: 20000000,             // 20km range
    enabled: true,
}
```

### Embedded/Real-Time Configuration

For resource-constrained embedded systems:

```rust
// System configuration
EmbeddedSystemConfig {
    sound_speed_mm_per_ms: 1500,        // Fixed value
    max_anchor_age_ms: 3000,            // Shorter for real-time
    min_anchors: 3,                     // Minimum required
    position_timeout_ms: 100,           // Fast response required
    accuracy_threshold_mm: 5000,        // More lenient for speed
}

// Anchor configuration with limited range
EmbeddedAnchorConfig {
    id: 1,
    position: CompactAnchorMessage::new(1, 0, 32.123, 45.476, 10.0, 255),
    max_range_mm: 5000000,              // 5km range
    enabled: true,
}
```

## Legacy JSON Configuration Support

The receiver maintains compatibility with JSON anchor configuration files:

### JSON Anchor Format

```json
{
  "anchors": [
    {
      "id": "001",
      "timestamp": 1753474380000,
      "position": {
        "lat": 32.123,
        "long": 45.476,
        "depth": 10.0
      }
    },
    {
      "id": "002", 
      "timestamp": 1753474380000,
      "position": {
        "lat": 32.124,
        "long": 45.477,
        "depth": 12.0
      }
    }
  ]
}
```

### JSON Field Descriptions

- `id`: String identifier for the anchor
- `timestamp`: Unix timestamp in milliseconds
- `position.lat`: Latitude in decimal degrees
- `position.long`: Longitude in decimal degrees (note: "long" not "lon")
- `position.depth`: Depth in meters (positive downward)

## Configuration Management

### Runtime Configuration

The receiver supports runtime configuration updates through the `RuntimeParameterManager`:

```rust
let mut param_manager = RuntimeParameterManager::new(config);

// Create backup before changes
param_manager.create_backup();

// Update sound speed
param_manager.update_sound_speed(1520.0)?;

// Rollback if needed
param_manager.rollback()?;
```

### Configuration Validation

All configurations are validated with specific rules:

```rust
// System configuration validation
if sound_speed_m_per_s < 1400.0 || sound_speed_m_per_s > 1600.0 {
    return Err("Sound speed outside valid range (1400-1600 m/s)");
}

// Anchor configuration validation
if position.latitude < -90.0 || position.latitude > 90.0 {
    return Err("Invalid latitude");
}
if position.depth < 0.0 || position.depth > 11000.0 {
    return Err("Invalid depth");
}
```

### Memory Optimization

The receiver uses memory-optimized structures for embedded systems:

- **Fixed-size arrays**: Replace dynamic vectors with fixed arrays
- **Compact data types**: Use `u16` for millimeter measurements instead of `f32`
- **Bit packing**: Efficient storage of boolean flags and small integers
- **Stack allocation**: Prefer stack over heap allocation where possible

## Configuration Examples

### Basic 3-Anchor Setup

```rust
// System configuration
let system_config = EmbeddedSystemConfig::new();

// Anchor configurations
let anchors = [
    Some(EmbeddedAnchorConfig::new(1, 32.123, 45.476, 10.0, 5000.0)),
    Some(EmbeddedAnchorConfig::new(2, 32.124, 45.477, 12.0, 5000.0)),
    Some(EmbeddedAnchorConfig::new(3, 32.125, 45.478, 8.0, 5000.0)),
    None, None, None, None, None  // Unused slots
];
```

### High-Density Anchor Network

```rust
// System configuration for dense network
let system_config = EmbeddedSystemConfig {
    sound_speed_mm_per_ms: 1500,
    max_anchor_age_ms: 3000,        // Shorter age for dense network
    min_anchors: 4,                 // Require 4+ anchors
    position_timeout_ms: 300,       // More time for complex calculations
    accuracy_threshold_mm: 1000,    // 1 meter accuracy
};

// Multiple anchors with overlapping coverage
let anchors = [
    Some(EmbeddedAnchorConfig::new(1, 32.120, 45.470, 10.0, 3000.0)),
    Some(EmbeddedAnchorConfig::new(2, 32.125, 45.475, 12.0, 3000.0)),
    Some(EmbeddedAnchorConfig::new(3, 32.130, 45.480, 8.0, 3000.0)),
    Some(EmbeddedAnchorConfig::new(4, 32.122, 45.482, 15.0, 3000.0)),
    Some(EmbeddedAnchorConfig::new(5, 32.128, 45.472, 9.0, 3000.0)),
    None, None, None  // Unused slots
];
```

## Performance Considerations

### Memory Usage

- **System Config**: ~16 bytes
- **Anchor Config**: ~32 bytes per anchor
- **Total for 8 anchors**: ~272 bytes
- **JSON parsing overhead**: Temporary, released after parsing

### Processing Time

- **Configuration validation**: <1ms
- **Anchor distance calculation**: <0.1ms per anchor
- **Position timeout**: Configurable (default 200ms)

### Real-Time Constraints

- **Maximum anchor age**: Keep under 5 seconds for real-time applications
- **Position timeout**: Balance accuracy vs. responsiveness
- **Minimum anchors**: Use 3 for speed, 4+ for accuracy

## Best Practices

1. **Calibrate Sound Speed**: Measure actual sound speed for your water conditions
2. **Position Anchors Strategically**: Ensure good geometric dilution of precision (GDOP)
3. **Set Realistic Timeouts**: Balance responsiveness with accuracy requirements
4. **Monitor Anchor Age**: Ensure anchor data remains fresh
5. **Use Appropriate Accuracy**: Don't over-constrain accuracy requirements
6. **Test Configurations**: Validate in actual deployment conditions
7. **Plan for Failures**: Configure redundant anchors for critical applications
8. **Optimize for Hardware**: Adjust parameters for your specific embedded platform

## Troubleshooting

### Common Configuration Issues

1. **No Position Fix**: Check minimum anchor count and timeout settings
2. **Poor Accuracy**: Verify sound speed calibration and anchor positions
3. **Timeout Errors**: Increase position timeout or reduce accuracy requirements
4. **Stale Data**: Reduce maximum anchor age or improve data freshness
5. **Memory Issues**: Reduce anchor count or optimize data structures

### Diagnostic Commands

```bash
# Validate current configuration
receiver validate-config

# Test anchor connectivity
receiver test-anchors

# Monitor positioning performance
receiver monitor --duration 60

# Check system resources
receiver system-info
```