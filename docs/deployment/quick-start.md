# Quick Start Guide

This guide will help you get your first underwater positioning beacon deployed and operational quickly. Follow these steps to go from zero to a working beacon system.

## Prerequisites

### Hardware Requirements

- ESP01-class microcomputer or compatible device
- GPS receiver module (UART interface)
- Underwater acoustic transceiver
- Battery system with monitoring capability
- Optional: Long-range communication module (cellular/satellite)

### Software Requirements

- Rust toolchain (1.70 or later)
- Git for source code management
- Serial terminal program (optional, for debugging)

### System Requirements

- Linux, macOS, or Windows development environment
- USB-to-serial adapter for device programming
- Minimum 4GB RAM for compilation
- 1GB free disk space

## Step 1: Get the Source Code

Clone the repository and build the beacon system:

```bash
# Clone the repository
git clone https://github.com/your-org/underwater-positioning-system.git
cd underwater-positioning-system

# Build the beacon system
cargo build --release

# Build the CLI tools
cargo build --release --bin beacon
```

## Step 2: Generate Initial Configuration

Create a default configuration file for your beacon:

```bash
# Generate default configuration
./target/release/beacon generate-config --output my-beacon.toml

# Or use a template for marine deployment
./target/release/beacon generate-config --template marine --output my-beacon.toml
```

This creates a configuration file with sensible defaults. The generated file will look like this:

```toml
# Beacon Configuration
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

# ... additional configuration sections
```

## Step 3: Customize Configuration

Edit the configuration file to match your deployment requirements:

### Basic Customization

```bash
# Set a unique beacon ID (or let the system generate one)
./target/release/beacon config update beacon_id "$(uuidgen)"

# Adjust transmission interval (in milliseconds)
./target/release/beacon config update transmission.interval_ms 10000

# Set GPS accuracy requirements
./target/release/beacon config update gps.accuracy_threshold_m 3.0

# Configure power thresholds for your battery system
./target/release/beacon config update power.low_battery_threshold_percent 25.0
```

### Hardware-Specific Settings

Edit the `[hardware]` section in your configuration file:

```toml
[hardware]
power_management_enabled = true
watchdog_enabled = true
watchdog_timeout_s = 60

[hardware.gpio_pin_mapping]
gps_enable = 2        # GPIO pin for GPS enable
power_monitor = 4     # GPIO pin for power monitoring
status_led = 16       # GPIO pin for status LED
emergency_button = 0  # GPIO pin for emergency button

[hardware.uart_config]
baud_rate = 9600     # UART baud rate for GPS
data_bits = 8
stop_bits = 1
parity = "None"
```

## Step 4: Validate Configuration

Ensure your configuration is valid:

```bash
# Basic validation
./target/release/beacon validate-config --config my-beacon.toml

# Strict validation with hardware checks
./target/release/beacon validate-config --config my-beacon.toml --strict --check-hardware
```

If validation passes, you'll see:
```
✓ Configuration validation passed
✓ All parameters within valid ranges
✓ Hardware configuration compatible with ESP01
✓ Power management settings optimized
```

## Step 5: Test with Mock Hardware

Before deploying to real hardware, test with mock components:

```bash
# Run system test with mock hardware
./target/release/beacon deploy test-deployment \
  --duration 300 \
  --mock-gps \
  --mock-power \
  --mock-communication \
  --scenario normal
```

This will simulate beacon operation for 5 minutes and show:
```
=== DEPLOYMENT TEST RESULTS ===
Test Duration: 300 seconds
Scenario: Normal Operation

✓ GPS Acquisition: Successful (simulated)
✓ Power Management: Battery at 85%, charging enabled
✓ Message Transmission: 60 messages sent, 100% success rate
✓ Communication: Status reports sent successfully
✓ System Health: 98% overall health score

Test completed successfully - ready for hardware deployment
```

## Step 6: Deploy to Hardware

### Flash the Firmware

```bash
# Build firmware for ESP01 target
cargo build --release --target xtensa-esp32-espidf

# Flash to device (adjust port as needed)
espflash flash --port /dev/ttyUSB0 target/xtensa-esp32-espidf/release/beacon
```

### Upload Configuration

```bash
# Upload configuration to device
./target/release/beacon config upload --config my-beacon.toml --port /dev/ttyUSB0
```

## Step 7: Initial System Check

Once deployed to hardware, perform initial checks:

```bash
# Check beacon status
./target/release/beacon status --config my-beacon.toml --detailed

# Run hardware diagnostics
./target/release/beacon diagnostic --all --config my-beacon.toml
```

Expected output:
```
=== BEACON STATUS ===
Beacon ID: 12345678-1234-5678-9abc-123456789abc
Operational State: Normal
Uptime: 0:02:15

--- GPS Status ---
GPS Status: Locked
Position: 40.712800, -74.006000 (±2.1m)
Satellites: 8
Last Update: 2024-01-15T10:30:00Z

--- Power Status ---
Battery Level: 87.5%
Voltage: 3.85V
Current: 125mA
Temperature: 23.2°C
Charging: Enabled

--- Transmission Status ---
Messages Sent: 12
Success Rate: 100.0%
Last Transmission: 2024-01-15T10:29:55Z
Signal Quality: 95%
```

## Step 8: Monitor Operation

Start monitoring your beacon:

```bash
# Real-time monitoring with 30-second updates
./target/release/beacon monitor --config my-beacon.toml --interval 30

# Monitor with logging
./target/release/beacon monitor --config my-beacon.toml --log-file beacon-monitor.log
```

## Step 9: Verify Receiver Integration

Test that your beacon is compatible with receiver systems:

```bash
# Test message format compatibility
./target/release/beacon test-receiver-integration --config my-beacon.toml --duration 60

# Check message parsing
cd receiver
cargo run -- --config test-config.json --test-mode --duration 60
```

## Common Issues and Solutions

### GPS Not Acquiring Lock

**Problem:** GPS status shows "Acquiring" for extended periods.

**Solutions:**
1. Check GPS antenna connection
2. Ensure clear sky view
3. Increase acquisition timeout:
   ```bash
   ./target/release/beacon config update gps.acquisition_timeout_s 120
   ```
4. Enable cold start timeout:
   ```bash
   ./target/release/beacon config update gps.cold_start_timeout_s 600
   ```

### Power Consumption Too High

**Problem:** Battery drains faster than expected.

**Solutions:**
1. Enable power save mode:
   ```bash
   ./target/release/beacon config update power.power_save_mode_threshold_percent 40.0
   ```
2. Reduce transmission frequency:
   ```bash
   ./target/release/beacon config update transmission.interval_ms 15000
   ```
3. Lower transmission power:
   ```bash
   ./target/release/beacon config update transmission.power_level 60
   ```

### Communication Failures

**Problem:** Long-range communication not working.

**Solutions:**
1. Check communication module connection
2. Verify network coverage
3. Increase retry attempts:
   ```bash
   ./target/release/beacon config update communication.retry_attempts 5
   ```
4. Enable data compression:
   ```bash
   ./target/release/beacon config update communication.data_compression_enabled true
   ```

### Memory Issues

**Problem:** System running out of memory on ESP01.

**Solutions:**
1. Disable unnecessary features in configuration
2. Reduce log buffer sizes
3. Enable memory optimization:
   ```toml
   [hardware]
   memory_optimization_enabled = true
   log_buffer_size = 512
   message_queue_size = 10
   ```

## Next Steps

### For Single Beacon Deployment

1. **Monitor Performance**: Set up continuous monitoring to track beacon health
2. **Optimize Settings**: Fine-tune configuration based on actual performance
3. **Plan Maintenance**: Schedule regular maintenance based on battery life and environmental conditions

### For Multi-Beacon Deployment

1. **Fleet Management**: Set up fleet management for multiple beacons
2. **Site Planning**: Plan beacon placement for optimal coverage
3. **Coordination**: Configure beacons to avoid interference

### Advanced Features

1. **Environmental Adaptation**: Enable automatic adaptation to environmental conditions
2. **Predictive Maintenance**: Set up predictive maintenance alerts
3. **Remote Management**: Configure remote configuration updates

## Configuration Templates

### Marine Environment Template

For harsh marine environments with high reliability requirements:

```bash
./target/release/beacon generate-config --template marine --output marine-beacon.toml
```

Key features:
- Extended GPS acquisition timeouts
- Aggressive power management
- Enhanced error recovery
- Environmental adaptation enabled

### Test Environment Template

For development and testing:

```bash
./target/release/beacon generate-config --template test --output test-beacon.toml
```

Key features:
- Shorter timeouts for faster testing
- Verbose logging enabled
- Mock hardware support
- Relaxed validation rules

### Low Power Template

For extended battery operation:

```bash
./target/release/beacon generate-config --template low-power --output low-power-beacon.toml
```

Key features:
- Extended transmission intervals
- Aggressive power saving
- Reduced GPS update frequency
- Optimized for battery life

## Verification Checklist

Before considering your beacon deployment complete, verify:

- [ ] Configuration validates without errors
- [ ] GPS acquires lock within expected time
- [ ] Battery monitoring reports accurate levels
- [ ] Messages transmit successfully
- [ ] Receiver systems can parse messages
- [ ] Power consumption within expected range
- [ ] System health score above 90%
- [ ] Emergency procedures tested
- [ ] Monitoring system operational
- [ ] Documentation updated with deployment details

## Getting Help

If you encounter issues:

1. **Check Logs**: Review system logs for error messages
2. **Run Diagnostics**: Use the diagnostic tools to identify problems
3. **Consult Documentation**: Review the troubleshooting guide
4. **Community Support**: Check the project's issue tracker
5. **Professional Support**: Contact support for critical deployments

## Summary

You now have a working underwater positioning beacon! The system is:

- ✅ Configured for your deployment environment
- ✅ Validated and tested
- ✅ Deployed to hardware
- ✅ Monitored and operational
- ✅ Compatible with receiver systems

Your beacon is now transmitting position messages that can be used by underwater receivers for positioning calculations. Monitor the system regularly and adjust configuration as needed based on operational experience.

For more advanced deployment scenarios, fleet management, and optimization techniques, continue to the [Deployment Guide](deployment-guide.md) and [Performance Tuning](../hardware/performance-tuning.md) documentation.