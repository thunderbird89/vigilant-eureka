# Beacon Emulator Troubleshooting Guide

This guide helps diagnose and resolve common issues with the beacon emulator.

## Table of Contents

- [Quick Diagnostics](#quick-diagnostics)
- [Common Issues](#common-issues)
- [Error Messages](#error-messages)
- [Performance Issues](#performance-issues)
- [Integration Problems](#integration-problems)
- [Debugging Tools](#debugging-tools)
- [FAQ](#faq)

## Quick Diagnostics

### Health Check Commands

Run these commands to quickly assess system health:

```bash
# Check overall status
beacon-emulator status --detailed

# List all beacons and their states
beacon-emulator list --detailed

# Check performance metrics
beacon-emulator performance --detailed

# Validate configuration files
beacon-emulator validate-config --config your_config.toml --verbose
```

### Log Analysis

Enable debug logging for detailed troubleshooting:

```bash
# Run with debug logging
beacon-emulator --log-level debug [command]

# Or set environment variable
RUST_LOG=debug beacon-emulator [command]

# For trace-level logging (very verbose)
RUST_LOG=trace beacon-emulator [command]
```

## Common Issues

### 1. Beacons Not Transmitting

**Symptoms:**
- Beacons show as "Running" but no messages in monitor
- Virtual receivers not receiving signals
- Export shows no message data

**Diagnosis:**
```bash
# Check if daemon is running
ps aux | grep "beacon-emulator daemon"

# Check beacon status
beacon-emulator list --detailed

# Monitor for transmission activity
beacon-emulator monitor --show-messages
```

**Solutions:**

**A. Daemon Not Running**
```bash
# Start daemon mode
beacon-emulator daemon --auto-start

# Or start in background
beacon-emulator daemon --auto-start --background &
```

**B. Beacons Not Started**
```bash
# Start all beacons
beacon-emulator start --all

# Or start specific beacon
beacon-emulator start <beacon-id>
```

**C. Configuration Issues**
```bash
# Check for configuration errors
beacon-emulator validate-config --config beacon.toml --verbose

# Reset to clean state
beacon-emulator reset --force
```

### 2. High CPU Usage

**Symptoms:**
- System becomes slow when running many beacons
- High CPU usage in system monitor
- Delayed responses from CLI commands

**Diagnosis:**
```bash
# Check performance metrics
beacon-emulator performance --detailed

# Monitor system resources
top -p $(pgrep beacon-emulator)

# Check beacon count and intervals
beacon-emulator list --detailed
```

**Solutions:**

**A. Reduce Message Rates**
```bash
# Increase transmission intervals
for beacon_id in $(beacon-emulator list --format json | jq -r '.beacons[].id'); do
    beacon-emulator update $beacon_id --interval 10000 --restart
done
```

**B. Enable Rate Limiting**
```bash
# Set global rate limit
beacon-emulator optimize --global-rate-limit 100

# Set per-beacon rate limit
beacon-emulator optimize --per-beacon-rate-limit 2
```

**C. Reduce Beacon Count**
```bash
# Stop some beacons
beacon-emulator stop-all
# Then start only essential ones
beacon-emulator start <essential-beacon-id>
```

### 3. Memory Issues

**Symptoms:**
- Increasing memory usage over time
- Out of memory errors
- System becomes unresponsive

**Diagnosis:**
```bash
# Check memory usage
beacon-emulator performance --detailed | grep -i memory

# Monitor process memory
ps -o pid,vsz,rss,comm -p $(pgrep beacon-emulator)
```

**Solutions:**

**A. Clear Message History**
```bash
# Export current data first
beacon-emulator export --output backup.json --all-time

# Reset to clear accumulated data
beacon-emulator reset --force
```

**B. Reduce Log Retention**
```bash
# Export and clear logs more frequently
beacon-emulator export --output logs_$(date +%Y%m%d_%H%M%S).json --duration 3600
```

**C. Restart Daemon Periodically**
```bash
# Stop daemon
pkill -f "beacon-emulator daemon"

# Restart with auto-start
beacon-emulator daemon --auto-start --background &
```

### 4. State File Corruption

**Symptoms:**
- Beacons disappear after restart
- Error loading state file
- Inconsistent beacon status

**Diagnosis:**
```bash
# Check state file
cat data/emulator_state.json | jq .

# Validate JSON format
jq empty data/emulator_state.json
```

**Solutions:**

**A. Backup and Reset**
```bash
# Backup current state
cp data/emulator_state.json data/emulator_state.json.backup

# Reset to clean state
beacon-emulator reset --force
```

**B. Manual State Repair**
```bash
# If JSON is corrupted, restore from backup
cp data/emulator_state.json.backup data/emulator_state.json

# Or start fresh
rm data/emulator_state.json
beacon-emulator status  # Will create new state file
```

### 5. Virtual Receiver Connection Issues

**Symptoms:**
- Virtual receiver can't connect to emulator
- No messages received by virtual receiver
- Connection timeout errors

**Diagnosis:**
```bash
# Check if IPC server is running
netstat -ln | grep 8765

# Check channel names match
beacon-emulator --channel test_channel list
# Should match receiver: cargo run -- virtual --channel test_channel

# Check for firewall issues
telnet localhost 8765
```

**Solutions:**

**A. Channel Name Mismatch**
```bash
# Ensure same channel name
beacon-emulator --channel my_channel daemon --auto-start &
cd receiver && cargo run -- virtual --channel my_channel
```

**B. Port Conflicts**
```bash
# Use different port
beacon-emulator --ipc-port 8766 daemon --auto-start &
# Update receiver configuration accordingly
```

**C. Restart IPC Server**
```bash
# Stop all beacon-emulator processes
pkill -f beacon-emulator

# Restart daemon
beacon-emulator daemon --auto-start &
```

## Error Messages

### "Beacon not found"

**Error:** `Error: Beacon 550e8400-e29b-41d4-a716-446655440000 not found`

**Causes:**
- Beacon ID doesn't exist
- Typo in beacon ID
- Beacon was removed

**Solutions:**
```bash
# List all beacons to find correct ID
beacon-emulator list

# Use tab completion if available
beacon-emulator stop <TAB>

# Check if beacon was removed
beacon-emulator list --format json | jq '.beacons[] | select(.id | contains("550e8400"))'
```

### "Invalid position coordinates"

**Error:** `Error: Invalid latitude: must be between -90 and 90 degrees`

**Causes:**
- Coordinates outside valid ranges
- Wrong coordinate format
- Decimal separator issues

**Solutions:**
```bash
# Check coordinate ranges
# Latitude: -90 to 90
# Longitude: -180 to 180
# Depth: 0 to 11000

# Use correct format
beacon-emulator create --lat 32.123 --lon 45.476 --depth 10.0

# Not: --lat 32,123 or --lat 32°7'23"
```

### "Configuration file not found"

**Error:** `Error: Configuration file 'beacon.toml' not found`

**Causes:**
- File doesn't exist
- Wrong file path
- Permission issues

**Solutions:**
```bash
# Check file exists
ls -la beacon.toml

# Use absolute path
beacon-emulator create --config /full/path/to/beacon.toml --lat 32.0 --lon 45.0

# Generate template if needed
beacon-emulator generate-template --output beacon.toml
```

### "Port already in use"

**Error:** `Error: Address already in use (os error 48)`

**Causes:**
- Another beacon-emulator instance running
- Port conflict with other service
- Previous instance didn't shut down cleanly

**Solutions:**
```bash
# Find process using port
lsof -i :8765

# Kill existing beacon-emulator processes
pkill -f beacon-emulator

# Use different port
beacon-emulator --ipc-port 8766 daemon
```

### "Invalid movement pattern"

**Error:** `Error: Invalid movement pattern format`

**Causes:**
- Wrong movement pattern syntax
- Missing parameters
- Invalid parameter values

**Solutions:**
```bash
# Correct formats:
beacon-emulator create --movement stationary
beacon-emulator create --movement linear:1.5:45
beacon-emulator create --movement circular:100:60
beacon-emulator create --movement random:0.5

# Not: --movement "linear 1.5 45" or --movement linear:1.5
```

## Performance Issues

### Slow Response Times

**Symptoms:**
- CLI commands take long to respond
- Monitor updates are delayed
- Export operations are slow

**Diagnosis:**
```bash
# Check system load
beacon-emulator performance --detailed

# Monitor command execution time
time beacon-emulator list

# Check beacon count
beacon-emulator list | wc -l
```

**Solutions:**

**A. Reduce Beacon Count**
```bash
# Stop non-essential beacons
beacon-emulator stop <beacon-id>

# Or use scenarios with fewer beacons
beacon-emulator scenario triangle  # 3 beacons instead of grid
```

**B. Optimize Intervals**
```bash
# Increase transmission intervals
beacon-emulator update <beacon-id> --interval 10000 --restart
```

**C. Use Compact Monitoring**
```bash
# Use compact display
beacon-emulator monitor --compact --interval 5
```

### High Message Loss

**Symptoms:**
- Gaps in exported message data
- Inconsistent transmission statistics
- Virtual receivers missing messages

**Diagnosis:**
```bash
# Check transmission statistics
beacon-emulator list --detailed | grep -i "messages\|errors"

# Monitor for errors
beacon-emulator --log-level debug monitor
```

**Solutions:**

**A. Enable Collision Avoidance**
```bash
beacon-emulator optimize --collision-avoidance true --collision-window 100
```

**B. Reduce Message Rates**
```bash
# Increase intervals to reduce collisions
for beacon_id in $(beacon-emulator list --format json | jq -r '.beacons[].id'); do
    beacon-emulator update $beacon_id --interval 5000 --restart
done
```

**C. Check System Resources**
```bash
# Ensure adequate system resources
beacon-emulator performance --detailed
```

## Integration Problems

### Virtual Receiver Not Receiving Messages

**Problem:** Virtual receiver connects but receives no beacon messages.

**Diagnosis:**
```bash
# Check beacon transmission
beacon-emulator monitor --show-messages

# Verify channel names match
beacon-emulator --channel test_channel list
# vs
cd receiver && cargo run -- virtual --channel test_channel

# Check IPC server status
netstat -ln | grep 8765
```

**Solutions:**

**A. Verify Channel Names**
```bash
# Use same channel for both
CHANNEL="integration_test"
beacon-emulator --channel $CHANNEL daemon --auto-start &
cd receiver && cargo run -- virtual --channel $CHANNEL
```

**B. Check Beacon Status**
```bash
# Ensure beacons are actually running
beacon-emulator list --running-only

# Start beacons if needed
beacon-emulator start --all
```

**C. Restart Both Systems**
```bash
# Stop everything
pkill -f beacon-emulator
pkill -f receiver

# Restart in order
beacon-emulator daemon --auto-start &
sleep 2
cd receiver && cargo run -- virtual
```

### Message Format Incompatibility

**Problem:** Virtual receiver receives messages but can't parse them.

**Diagnosis:**
```bash
# Check message versions
beacon-emulator list --detailed | grep -i version

# Monitor raw message content
beacon-emulator monitor --show-messages

# Check receiver logs for parsing errors
```

**Solutions:**

**A. Ensure Compatible Message Versions**
```bash
# Use V3 messages (most compatible)
beacon-emulator update <beacon-id> --version V3 --restart
```

**B. Validate Message Format**
```bash
# Export messages and check format
beacon-emulator export --output test.json --include-messages --duration 60
jq '.messages[0]' test.json
```

## Debugging Tools

### Enable Detailed Logging

```bash
# Debug level logging
RUST_LOG=debug beacon-emulator [command]

# Trace level (very verbose)
RUST_LOG=trace beacon-emulator [command]

# Module-specific logging
RUST_LOG=beacon_emulator::virtual_beacon=debug beacon-emulator [command]
```

### State Inspection

```bash
# View current state
cat data/emulator_state.json | jq .

# Check specific beacon state
cat data/emulator_state.json | jq '.beacons["<beacon-id>"]'

# Monitor state changes
watch -n 1 'beacon-emulator status'
```

### Network Debugging

```bash
# Check IPC server
netstat -ln | grep 8765
telnet localhost 8765

# Monitor network traffic (if using network IPC)
tcpdump -i lo port 8765

# Check process connections
lsof -p $(pgrep beacon-emulator)
```

### Performance Profiling

```bash
# Monitor resource usage
top -p $(pgrep beacon-emulator)

# Memory usage over time
while true; do
    ps -o pid,vsz,rss,comm -p $(pgrep beacon-emulator)
    sleep 5
done

# CPU profiling (if built with profiling support)
perf record -g beacon-emulator daemon
perf report
```

## FAQ

### Q: Why aren't my beacons transmitting messages?

**A:** Beacons need daemon mode to actually transmit. The `create` and `start` commands only set up the intended state. Run `beacon-emulator daemon --auto-start` to enable actual transmission.

### Q: How many beacons can I run simultaneously?

**A:** The system is designed to handle 50+ beacons, but actual limits depend on:
- System resources (CPU, memory)
- Transmission intervals
- Message processing overhead

Start with 10-20 beacons and monitor performance.

### Q: Can I run multiple emulator instances?

**A:** Yes, use different channels and ports:
```bash
# Instance 1
beacon-emulator --channel test1 --ipc-port 8765 daemon &

# Instance 2  
beacon-emulator --channel test2 --ipc-port 8766 daemon &
```

### Q: How do I backup my beacon configurations?

**A:** Export current state and configurations:
```bash
# Export beacon data
beacon-emulator export --output backup.json --all-time

# Backup state file
cp data/emulator_state.json data/emulator_state.backup

# Generate config templates for recreation
beacon-emulator generate-template --output template.toml
```

### Q: Why is my virtual receiver not getting messages?

**A:** Check these common issues:
1. Channel names must match exactly
2. Daemon mode must be running
3. Beacons must be started (not just created)
4. IPC port must be available (default 8765)

### Q: How do I simulate beacon failures?

**A:** Use these techniques:
```bash
# Stop individual beacons
beacon-emulator stop <beacon-id>

# Simulate intermittent failures
beacon-emulator stop <beacon-id>
sleep 30
beacon-emulator start <beacon-id>

# Simulate position drift
beacon-emulator update <beacon-id> --movement random:0.1
```

### Q: Can I use real beacon configuration files?

**A:** Yes, but validate them first:
```bash
# Validate existing config
beacon-emulator validate-config --config real_beacon.toml --verbose

# Use with emulator
beacon-emulator create --config real_beacon.toml --lat 32.0 --lon 45.0
```

### Q: How do I test positioning accuracy?

**A:** Use precise beacon arrangements:
```bash
# Create exact square formation
beacon-emulator scenario square --spacing 100 --center 32.0,45.0,10.0

# Use fast transmission for more data points
beacon-emulator update <beacon-id> --interval 1000 --restart

# Export for analysis
beacon-emulator export --output accuracy_test.json --include-messages
```

### Q: What's the difference between stopped and removed beacons?

**A:** 
- **Stopped**: Beacon configuration preserved, can be restarted
- **Removed**: Beacon completely deleted from registry

```bash
# Stop (preserves configuration)
beacon-emulator stop <beacon-id>
beacon-emulator start <beacon-id>  # Can restart

# Remove (deletes completely)
beacon-emulator remove <beacon-id>
beacon-emulator start <beacon-id>  # Error: beacon not found
```

### Q: How do I automate testing workflows?

**A:** Use batch operations and scripting:
```bash
# Create batch configuration
beacon-emulator batch --file test_scenario.json

# Use automation-friendly options
beacon-emulator --automation-mode --non-interactive create --lat 32.0 --lon 45.0

# Script with error handling
beacon-emulator reset --force || exit 1
beacon-emulator scenario triangle --start-all || exit 1
```

This troubleshooting guide covers the most common issues and provides systematic approaches to diagnosis and resolution. For additional help, enable debug logging and examine the detailed error messages.