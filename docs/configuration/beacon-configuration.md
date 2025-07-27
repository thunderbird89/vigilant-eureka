# Beacon Configuration Structure

This document describes the configuration structure for physical beacon devices. Beacon configurations are stored in TOML, JSON, or YAML format and define all operational parameters for a beacon device.

## File Format Support

Beacon configurations support multiple file formats:
- **TOML** (default): `.toml` extension
- **JSON**: `.json` extension  
- **YAML**: `.yaml` or `.yml` extension

## Configuration Structure

### Root Level

```toml
beacon_id = "6ca184e2-8169-40d8-91f2-bd0ae91fb495"  # UUID v4 identifier
```

### Transmission Configuration

Controls how the beacon transmits position data:

```toml
[transmission]
interval_ms = 3000                    # Transmission interval (1000-60000ms)
message_version = "V2"                # Protocol version: "V1", "V2", "V3"
power_level = 200                     # Transmission power (1-255)
max_retries = 3                       # Maximum retry attempts (0-10)
retry_delay_ms = 1000                 # Delay between retries (100-10000ms)
adaptive_power = false                # Enable adaptive power control
sequence_rollover = 65535             # Sequence number rollover point
```

**Validation Rules:**
- `interval_ms`: 1000-60000 (1 second to 1 minute)
- `power_level`: 1-255 (higher = more power)
- `max_retries`: 0-10 attempts
- `retry_delay_ms`: 100-10000ms

### GPS Configuration

Controls GPS acquisition and positioning:

```toml
[gps]
acquisition_timeout_s = 60            # GPS acquisition timeout (10-300s)
update_interval_s = 2                 # GPS update frequency (1-60s)
min_satellite_count = 4               # Minimum satellites required (3-12)
accuracy_threshold_m = 3.0            # Required accuracy in meters (1.0-100.0)
cold_start_timeout_s = 120            # Cold start timeout (30-600s)
enable_dgps = false                   # Enable differential GPS
max_fix_age_s = 30                    # Maximum age of GPS fix (5-300s)
```

**Validation Rules:**
- `acquisition_timeout_s`: 10-300 seconds
- `update_interval_s`: 1-60 seconds
- `min_satellite_count`: 3-12 satellites
- `accuracy_threshold_m`: 1.0-100.0 meters

### Power Management Configuration

Controls battery and power management:

```toml
[power]
low_battery_threshold_percent = 20.0          # Low battery warning (5.0-50.0%)
critical_battery_threshold_percent = 10.0     # Critical battery level (1.0-25.0%)
emergency_battery_threshold_percent = 5.0     # Emergency shutdown level (1.0-15.0%)
power_save_threshold_percent = 25.0           # Enter power save mode (10.0-60.0%)
charging_enabled = true                       # Enable charging system
solar_charging_enabled = false                # Enable solar charging
monitoring_interval_s = 30                    # Battery monitoring interval (10-300s)

[power.temperature_limits]
min_operating_c = -10.0               # Minimum operating temperature
max_operating_c = 50.0                # Maximum operating temperature
warning_threshold_c = 55.0            # Temperature warning threshold
emergency_threshold_c = 65.0          # Emergency shutdown temperature

[power.power_modes]
normal_transmission_multiplier = 1.0          # Normal mode transmission rate
power_save_transmission_multiplier = 2.0      # Power save mode rate multiplier
emergency_transmission_multiplier = 0.5       # Emergency mode rate multiplier

[power.power_modes.cpu_scaling]
normal_frequency_percent = 100        # CPU frequency in normal mode
power_save_frequency_percent = 60     # CPU frequency in power save mode
emergency_frequency_percent = 80      # CPU frequency in emergency mode
```

**Validation Rules:**
- Battery thresholds must be in descending order: low > critical > emergency
- All percentages: 1.0-100.0%
- Temperature limits: -40.0 to 85.0°C

### Communication Configuration

Controls external communication and data reporting:

```toml
[communication]
connection_interval_hours = 6         # Connection attempt interval (1-168h)
max_retry_attempts = 5                # Maximum connection retries (1-10)
initial_retry_backoff_ms = 2000       # Initial retry delay (1000-30000ms)
max_retry_interval_hours = 4          # Maximum retry interval (1-24h)
connection_timeout_s = 120            # Connection timeout (10-300s)
compression_enabled = true            # Enable data compression

[communication.status_report]
include_position_history = true       # Include position history in reports
max_position_history = 100            # Maximum position records (10-1000)
include_battery_details = true        # Include detailed battery info
include_transmission_stats = true     # Include transmission statistics
include_system_health = false         # Include system health metrics

[communication.endpoints]
primary_url = "https://api.example.com/beacon"    # Primary API endpoint
api_key = "your-api-key"              # API authentication key
device_token = "your-device-token"    # Device authentication token
```

### Emergency Configuration

Controls emergency mode behavior:

```toml
[emergency]
emergency_mode_enabled = true         # Enable emergency mode
emergency_interval_ms = 1000          # Emergency transmission interval (500-10000ms)
emergency_power_boost_percent = 150   # Power boost in emergency (100-200%)
max_emergency_duration_minutes = 30   # Maximum emergency duration (5-120min)
auto_recovery_enabled = false         # Enable automatic recovery

[emergency.shutdown_conditions]
battery_shutdown_enabled = true       # Enable battery-based shutdown
temperature_shutdown_enabled = true   # Enable temperature-based shutdown
hardware_fault_shutdown_enabled = true # Enable hardware fault shutdown
shutdown_grace_period_s = 60          # Shutdown grace period (5-300s)
```

### Hardware Configuration

Controls hardware-specific settings:

```toml
[hardware.gpio_pins]
status_led_pin = 2                    # Status LED GPIO pin
emergency_button_pin = null           # Emergency button pin (optional)
power_control_pin = null              # Power control pin (optional)
external_reset_pin = null             # External reset pin (optional)

[hardware.spi_config]
clock_frequency_hz = 1000000          # SPI clock frequency
mode = 0                              # SPI mode (0-3)
msb_first = true                      # MSB first transmission

[hardware.i2c_config]
clock_frequency_hz = 100000           # I2C clock frequency
timeout_ms = 1000                     # I2C operation timeout

[hardware.memory_config]
max_heap_usage_percent = 70           # Maximum heap usage (50-90%)
stack_size_bytes = 4096               # Stack size in bytes
memory_monitoring_enabled = false     # Enable memory monitoring
memory_check_interval_s = 60          # Memory check interval

[hardware.watchdog_config]
enabled = false                       # Enable hardware watchdog
timeout_s = 5                         # Watchdog timeout (1-30s)
auto_reset_enabled = true             # Enable automatic reset
```

### Metadata

Configuration metadata for tracking and validation:

```toml
[metadata]
schema_version = "1.0.0"              # Configuration schema version
created_at = 1753468665               # Creation timestamp (Unix)
modified_at = 1753468665              # Last modification timestamp
description = "Default beacon configuration"  # Human-readable description
author = "System"                     # Configuration author
checksum = "9a8c09679ab4b503"         # Configuration integrity checksum
migration_history = []                # Migration history array
```

## Configuration Management

### Loading Configuration

The beacon system automatically detects file format based on extension:

```bash
# Load TOML configuration (default)
beacon --config beacon.toml

# Load JSON configuration
beacon --config beacon.json

# Load YAML configuration
beacon --config beacon.yaml
```

### Validation

All configurations are validated on load with specific rules:

- **Range validation**: All numeric values must be within specified ranges
- **Dependency validation**: Related settings must be consistent
- **Hardware validation**: Hardware-specific settings must be valid for the target device

### Configuration Generation

Generate a default configuration file:

```bash
beacon generate-config --format toml --output beacon.toml
beacon generate-config --format json --output beacon.json
beacon generate-config --format yaml --output beacon.yaml
```

## Example Complete Configuration

```toml
beacon_id = "6ca184e2-8169-40d8-91f2-bd0ae91fb495"

[transmission]
interval_ms = 3000
message_version = "V2"
power_level = 200
max_retries = 3
retry_delay_ms = 1000
adaptive_power = false
sequence_rollover = 65535

[gps]
acquisition_timeout_s = 60
update_interval_s = 2
min_satellite_count = 4
accuracy_threshold_m = 3.0
cold_start_timeout_s = 120
enable_dgps = false
max_fix_age_s = 30

[power]
low_battery_threshold_percent = 20.0
critical_battery_threshold_percent = 10.0
emergency_battery_threshold_percent = 5.0
power_save_threshold_percent = 25.0
charging_enabled = true
solar_charging_enabled = false
monitoring_interval_s = 30

[power.temperature_limits]
min_operating_c = -10.0
max_operating_c = 50.0
warning_threshold_c = 55.0
emergency_threshold_c = 65.0

[power.power_modes]
normal_transmission_multiplier = 1.0
power_save_transmission_multiplier = 2.0
emergency_transmission_multiplier = 0.5

[power.power_modes.cpu_scaling]
normal_frequency_percent = 100
power_save_frequency_percent = 60
emergency_frequency_percent = 80

[communication]
connection_interval_hours = 6
max_retry_attempts = 5
initial_retry_backoff_ms = 2000
max_retry_interval_hours = 4
connection_timeout_s = 120
compression_enabled = true

[communication.status_report]
include_position_history = true
max_position_history = 100
include_battery_details = true
include_transmission_stats = true
include_system_health = false

[communication.endpoints]
primary_url = "https://api.example.com/beacon"
api_key = "your-api-key"
device_token = "your-device-token"

[emergency]
emergency_mode_enabled = true
emergency_interval_ms = 1000
emergency_power_boost_percent = 150
max_emergency_duration_minutes = 30
auto_recovery_enabled = false

[emergency.shutdown_conditions]
battery_shutdown_enabled = true
temperature_shutdown_enabled = true
hardware_fault_shutdown_enabled = true
shutdown_grace_period_s = 60

[hardware.gpio_pins]
status_led_pin = 2

[hardware.spi_config]
clock_frequency_hz = 1000000
mode = 0
msb_first = true

[hardware.i2c_config]
clock_frequency_hz = 100000
timeout_ms = 1000

[hardware.memory_config]
max_heap_usage_percent = 70
stack_size_bytes = 4096
memory_monitoring_enabled = false
memory_check_interval_s = 60

[hardware.watchdog_config]
enabled = false
timeout_s = 5
auto_reset_enabled = true

[metadata]
schema_version = "1.0.0"
created_at = 1753468665
modified_at = 1753468665
description = "Default beacon configuration"
author = "System"
checksum = "9a8c09679ab4b503"
migration_history = []
```

## Best Practices

1. **Always validate** configurations before deployment
2. **Use appropriate intervals** based on your use case and power requirements
3. **Set realistic GPS parameters** for your operating environment
4. **Configure power management** according to your power source
5. **Test emergency modes** in controlled environments
6. **Keep backups** of working configurations
7. **Document changes** in the description field
8. **Use version control** for configuration files