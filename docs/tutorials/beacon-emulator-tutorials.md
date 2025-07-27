# Beacon Emulator Tutorials

This document provides step-by-step tutorials for common testing scenarios and workflows using the beacon emulator.

## Table of Contents

- [Getting Started](#getting-started)
- [Basic Testing Workflow](#basic-testing-workflow)
- [Positioning Accuracy Testing](#positioning-accuracy-testing)
- [Movement Pattern Testing](#movement-pattern-testing)
- [Performance and Stress Testing](#performance-and-stress-testing)
- [Integration Testing with Virtual Receivers](#integration-testing-with-virtual-receivers)
- [Automated Testing Workflows](#automated-testing-workflows)
- [Advanced Scenarios](#advanced-scenarios)

## Getting Started

### Prerequisites

1. Build the beacon emulator:
   ```bash
   cargo build -p beacon-emulator
   ```

2. Verify installation:
   ```bash
   beacon-emulator --help
   ```

3. Check current status:
   ```bash
   beacon-emulator status
   ```

### Your First Virtual Beacon

Let's create and run your first virtual beacon:

1. **Create a beacon:**
   ```bash
   beacon-emulator create --lat 32.123 --lon 45.476 --depth 10.0
   ```
   
   This creates a stationary beacon at the specified coordinates. Note the beacon ID returned.

2. **List beacons:**
   ```bash
   beacon-emulator list --detailed
   ```
   
   You'll see your beacon listed with status "Stopped" (created but not transmitting).

3. **Start the beacon:**
   ```bash
   beacon-emulator start <beacon-id>
   ```
   
   Or start all beacons:
   ```bash
   beacon-emulator start --all
   ```

4. **Run daemon mode to enable transmission:**
   ```bash
   beacon-emulator daemon --auto-start
   ```
   
   Keep this running in a separate terminal. The beacon will now actively transmit messages.

5. **Monitor beacon activity:**
   ```bash
   # In another terminal
   beacon-emulator monitor
   ```
   
   You'll see real-time transmission statistics and beacon status.

6. **Stop and clean up:**
   ```bash
   beacon-emulator stop-all --remove
   ```

## Basic Testing Workflow

This tutorial demonstrates a complete testing workflow from setup to cleanup.

### Step 1: Environment Setup

1. **Reset to clean state:**
   ```bash
   beacon-emulator reset --force
   ```

2. **Create a test scenario:**
   ```bash
   beacon-emulator scenario triangle --spacing 100 --center 32.0,45.0,10.0
   ```
   
   This creates 3 beacons in a triangular arrangement with 100m spacing.

3. **Verify beacon creation:**
   ```bash
   beacon-emulator list --detailed
   ```

### Step 2: Start Testing

1. **Start all beacons:**
   ```bash
   beacon-emulator start --all
   ```

2. **Run daemon mode:**
   ```bash
   beacon-emulator daemon --auto-start --status-interval 10 &
   ```
   
   The `&` runs it in background. Use `--status-interval 10` for status updates every 10 seconds.

3. **Monitor activity:**
   ```bash
   beacon-emulator monitor --interval 2 --show-messages
   ```
   
   Press Ctrl+C to exit monitoring.

### Step 3: Data Collection

1. **Let beacons run for test duration:**
   ```bash
   sleep 60  # Run for 1 minute
   ```

2. **Export test data:**
   ```bash
   beacon-emulator export --output test_results.json --duration 120 --include-messages
   ```

3. **Check performance metrics:**
   ```bash
   beacon-emulator performance --detailed --export performance.json
   ```

### Step 4: Cleanup

1. **Stop daemon:**
   ```bash
   # If running in background, find and kill the process
   pkill -f "beacon-emulator daemon"
   ```

2. **Stop all beacons:**
   ```bash
   beacon-emulator stop-all --remove
   ```

3. **Verify cleanup:**
   ```bash
   beacon-emulator status
   ```

## Positioning Accuracy Testing

This tutorial tests positioning accuracy using precisely positioned beacons.

### Scenario Setup

We'll create a square formation with known positions to test trilateration accuracy.

1. **Create precision test beacons:**
   ```bash
   # Create 4 beacons in exact square formation
   beacon-emulator create --id 00000000-0000-0000-0000-000000000001 \
                         --lat 32.000000 --lon 45.000000 --depth 10.0 \
                         --interval 1000
   
   beacon-emulator create --id 00000000-0000-0000-0000-000000000002 \
                         --lat 32.000900 --lon 45.000000 --depth 10.0 \
                         --interval 1000
   
   beacon-emulator create --id 00000000-0000-0000-0000-000000000003 \
                         --lat 32.000900 --lon 45.001200 --depth 10.0 \
                         --interval 1000
   
   beacon-emulator create --id 00000000-0000-0000-0000-000000000004 \
                         --lat 32.000000 --lon 45.001200 --depth 10.0 \
                         --interval 1000
   ```
   
   These coordinates form a 100m x 100m square (approximately).

2. **Start beacons and daemon:**
   ```bash
   beacon-emulator start --all
   beacon-emulator daemon --auto-start &
   ```

### Testing with Virtual Receiver

1. **Start virtual receiver:**
   ```bash
   cd receiver
   cargo run -- virtual --channel default --update-interval 1
   ```
   
   The receiver will calculate positions using the 4 beacon signals.

2. **Monitor both systems:**
   ```bash
   # Terminal 1: Monitor beacons
   beacon-emulator monitor --compact
   
   # Terminal 2: Monitor receiver (check receiver output)
   ```

3. **Collect accuracy data:**
   ```bash
   # Let system run for 5 minutes
   sleep 300
   
   # Export beacon data
   beacon-emulator export --output accuracy_test.json --duration 300
   ```

### Analysis

The exported data contains:
- Exact beacon positions (ground truth)
- Transmission timestamps
- Message sequence numbers
- Signal quality indicators

Compare receiver-calculated positions with known beacon positions to evaluate accuracy.

## Movement Pattern Testing

This tutorial demonstrates testing with moving beacons to validate dynamic positioning.

### Linear Movement Test

1. **Create beacon with linear movement:**
   ```bash
   beacon-emulator create --lat 32.0 --lon 45.0 --depth 10.0 \
                         --movement linear:1.0:90 \
                         --interval 2000
   ```
   
   This creates a beacon moving east at 1 m/s.

2. **Create stationary reference beacons:**
   ```bash
   beacon-emulator scenario triangle --count 3 --spacing 200 \
                                    --center 32.0,45.0,10.0
   ```

3. **Start all beacons:**
   ```bash
   beacon-emulator start --all
   beacon-emulator daemon --auto-start &
   ```

4. **Monitor movement:**
   ```bash
   beacon-emulator monitor --show-messages --interval 1
   ```
   
   You should see the moving beacon's position changing over time.

### Circular Movement Test

1. **Update beacon to circular movement:**
   ```bash
   # Get the moving beacon ID from list command
   BEACON_ID=$(beacon-emulator list --format json | jq -r '.beacons[0].id')
   
   beacon-emulator update $BEACON_ID --movement circular:50:60 --restart
   ```
   
   This changes to circular movement with 50m radius and 60-second period.

2. **Monitor circular pattern:**
   ```bash
   beacon-emulator monitor --beacon $BEACON_ID --interval 2
   ```
   
   Watch the position coordinates trace a circular pattern.

### Random Movement Test

1. **Switch to random movement:**
   ```bash
   beacon-emulator update $BEACON_ID --movement random:0.5 --restart
   ```
   
   Random walk with maximum 0.5 m/s speed.

2. **Collect movement data:**
   ```bash
   # Run for 2 minutes
   sleep 120
   
   beacon-emulator export --output movement_test.json --duration 150 \
                         --beacon $BEACON_ID --include-messages
   ```

## Performance and Stress Testing

This tutorial tests system performance under high beacon counts and message rates.

### High Beacon Count Test

1. **Create many beacons:**
   ```bash
   # Create 5x5 grid (25 beacons)
   beacon-emulator scenario grid --count 25 --spacing 50 \
                                --center 32.0,45.0,10.0 \
                                --interval 3000
   ```

2. **Monitor resource usage before starting:**
   ```bash
   beacon-emulator performance --detailed
   ```

3. **Start all beacons:**
   ```bash
   beacon-emulator start --all
   beacon-emulator daemon --auto-start --status-interval 5 &
   ```

4. **Monitor performance during operation:**
   ```bash
   # Monitor system performance
   beacon-emulator performance --detailed
   
   # Monitor beacon activity
   beacon-emulator monitor --compact --interval 5
   ```

5. **Stress test with high message rates:**
   ```bash
   # Update all beacons to faster transmission (1 second intervals)
   for beacon_id in $(beacon-emulator list --format json | jq -r '.beacons[].id'); do
       beacon-emulator update $beacon_id --interval 1000 --restart
   done
   ```

### Message Throughput Test

1. **Create high-throughput scenario:**
   ```bash
   beacon-emulator reset --force
   
   # Create 10 beacons with very fast transmission
   for i in {1..10}; do
       lat=$(echo "32.0 + $i * 0.001" | bc)
       beacon-emulator create --lat $lat --lon 45.0 --depth 10.0 \
                             --interval 500  # 500ms = 2 messages/second
   done
   ```

2. **Start and monitor throughput:**
   ```bash
   beacon-emulator start --all
   beacon-emulator daemon --auto-start &
   
   # Monitor message rates
   beacon-emulator performance --detailed
   ```

3. **Measure sustained performance:**
   ```bash
   # Run for 5 minutes and collect metrics
   sleep 300
   beacon-emulator performance --export stress_test_metrics.json
   beacon-emulator export --output stress_test_data.json --duration 300
   ```

## Integration Testing with Virtual Receivers

This tutorial demonstrates end-to-end testing with virtual receivers.

### Single Receiver Test

1. **Set up test environment:**
   ```bash
   beacon-emulator reset --force
   
   # Create optimal beacon arrangement for positioning
   beacon-emulator scenario square --spacing 150 --center 32.0,45.0,10.0 \
                                  --interval 2000 --start-all
   
   beacon-emulator daemon --auto-start &
   ```

2. **Start virtual receiver:**
   ```bash
   cd receiver
   cargo run -- virtual --channel default --id 1 --update-interval 2 &
   ```

3. **Monitor both systems:**
   ```bash
   # Terminal 1: Monitor beacons
   beacon-emulator monitor --compact
   
   # Terminal 2: Check receiver logs for positioning results
   tail -f receiver_output.log  # If logging to file
   ```

### Multiple Receiver Test

1. **Start multiple virtual receivers:**
   ```bash
   # Start 3 receivers on same channel
   cd receiver
   cargo run -- virtual --channel default --id 1 &
   cargo run -- virtual --channel default --id 2 &
   cargo run -- virtual --channel default --id 3 &
   ```

2. **Monitor system load:**
   ```bash
   beacon-emulator performance --detailed
   ```

### Channel Isolation Test

1. **Create beacons on different channels:**
   ```bash
   # Channel A beacons
   beacon-emulator --channel channel_a scenario triangle --spacing 100 \
                                                         --center 32.0,45.0,10.0 \
                                                         --start-all
   
   # Channel B beacons  
   beacon-emulator --channel channel_b scenario triangle --spacing 100 \
                                                         --center 32.1,45.1,10.0 \
                                                         --start-all
   ```

2. **Start daemons for both channels:**
   ```bash
   beacon-emulator --channel channel_a daemon --auto-start &
   beacon-emulator --channel channel_b daemon --auto-start &
   ```

3. **Start receivers on different channels:**
   ```bash
   cd receiver
   cargo run -- virtual --channel channel_a --id 1 &
   cargo run -- virtual --channel channel_b --id 2 &
   ```

4. **Verify isolation:**
   ```bash
   # Monitor each channel separately
   beacon-emulator --channel channel_a monitor --compact &
   beacon-emulator --channel channel_b monitor --compact &
   ```

## Automated Testing Workflows

This tutorial shows how to create automated test suites.

### Batch Operation Example

1. **Create batch configuration file:**
   ```json
   {
     "operations": [
       {
         "type": "reset",
         "force": true
       },
       {
         "type": "create_scenario",
         "scenario_type": "triangle",
         "count": 3,
         "spacing": 100.0,
         "center": [32.0, 45.0, 10.0],
         "start_all": true
       },
       {
         "type": "start_daemon",
         "auto_start": true,
         "background": true
       },
       {
         "type": "wait",
         "duration": 30
       },
       {
         "type": "export_logs",
         "output": "automated_test_results.json",
         "format": "json",
         "duration": 60,
         "include_messages": true
       },
       {
         "type": "performance_report",
         "output": "performance_metrics.json"
       },
       {
         "type": "cleanup",
         "stop_all": true,
         "remove": true
       }
     ]
   }
   ```

2. **Execute batch operations:**
   ```bash
   beacon-emulator batch --file automated_test.json --verbose
   ```

### CI/CD Integration Script

Create a test script for continuous integration:

```bash
#!/bin/bash
# ci_test_suite.sh

set -e  # Exit on any error

echo "Starting beacon emulator CI test suite..."

# Configuration
TEST_DURATION=60
RESULTS_DIR="test_results_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RESULTS_DIR"

# Test 1: Basic functionality
echo "Test 1: Basic functionality test"
beacon-emulator --automation-mode reset --force
beacon-emulator --automation-mode scenario triangle --spacing 100 --start-all
beacon-emulator --automation-mode daemon --auto-start --background &
DAEMON_PID=$!

sleep $TEST_DURATION

beacon-emulator --automation-mode export --output "$RESULTS_DIR/basic_test.json" \
                                        --duration $TEST_DURATION
kill $DAEMON_PID
beacon-emulator --automation-mode stop-all --remove

# Test 2: Performance test
echo "Test 2: Performance test"
beacon-emulator --automation-mode scenario grid --count 16 --spacing 50 --start-all
beacon-emulator --automation-mode daemon --auto-start --background &
DAEMON_PID=$!

sleep $TEST_DURATION

beacon-emulator --automation-mode performance --export "$RESULTS_DIR/performance.json"
beacon-emulator --automation-mode export --output "$RESULTS_DIR/performance_test.json" \
                                        --duration $TEST_DURATION
kill $DAEMON_PID
beacon-emulator --automation-mode stop-all --remove

# Test 3: Movement patterns
echo "Test 3: Movement pattern test"
beacon-emulator --automation-mode create --lat 32.0 --lon 45.0 \
                                        --movement linear:1.0:90 --start
beacon-emulator --automation-mode scenario triangle --spacing 200 \
                                                   --center 32.0,45.0,10.0 \
                                                   --start-all
beacon-emulator --automation-mode daemon --auto-start --background &
DAEMON_PID=$!

sleep $TEST_DURATION

beacon-emulator --automation-mode export --output "$RESULTS_DIR/movement_test.json" \
                                        --duration $TEST_DURATION --include-messages
kill $DAEMON_PID
beacon-emulator --automation-mode stop-all --remove

# Generate test report
echo "Generating test report..."
cat > "$RESULTS_DIR/test_report.md" << EOF
# Beacon Emulator Test Report

Generated: $(date)

## Test Results

### Basic Functionality Test
- Duration: ${TEST_DURATION}s
- Results: $RESULTS_DIR/basic_test.json

### Performance Test  
- Duration: ${TEST_DURATION}s
- Beacon count: 16
- Results: $RESULTS_DIR/performance_test.json
- Metrics: $RESULTS_DIR/performance.json

### Movement Pattern Test
- Duration: ${TEST_DURATION}s
- Pattern: Linear movement + stationary references
- Results: $RESULTS_DIR/movement_test.json

## Summary

All tests completed successfully.
EOF

echo "Test suite completed. Results in: $RESULTS_DIR"
```

Make it executable and run:
```bash
chmod +x ci_test_suite.sh
./ci_test_suite.sh
```

## Advanced Scenarios

### Custom Movement Patterns

Create beacons with complex movement combinations:

```bash
# Create a convoy of moving beacons
beacon-emulator create --lat 32.0 --lon 45.0 --movement linear:2.0:45 --start
beacon-emulator create --lat 32.001 --lon 45.001 --movement linear:2.0:45 --start
beacon-emulator create --lat 32.002 --lon 45.002 --movement linear:2.0:45 --start

# Create orbiting beacons around a center
beacon-emulator create --lat 32.0 --lon 45.0 --movement circular:100:120 --start
beacon-emulator create --lat 32.0 --lon 45.0 --movement circular:150:180 --start
```

### Environmental Simulation

Simulate challenging conditions:

```bash
# Create beacons with different transmission intervals (simulating interference)
beacon-emulator create --lat 32.0 --lon 45.0 --interval 1000 --start
beacon-emulator create --lat 32.001 --lon 45.001 --interval 1500 --start
beacon-emulator create --lat 32.002 --lon 45.002 --interval 2000 --start

# Add random movement to simulate current effects
beacon-emulator create --lat 32.003 --lon 45.003 --movement random:0.3 --start
```

### Large-Scale Deployment Simulation

Test with realistic deployment scenarios:

```bash
# Create a large underwater beacon network
beacon-emulator scenario grid --count 49 --spacing 200 \
                             --center 32.0,45.0,15.0 \
                             --interval 5000

# Add some mobile beacons (AUVs)
for i in {1..5}; do
    lat=$(echo "32.0 + $i * 0.01" | bc)
    beacon-emulator create --lat $lat --lon 45.0 --depth 20.0 \
                          --movement random:1.5 --interval 3000 --start
done

# Start the network
beacon-emulator start --all
beacon-emulator daemon --auto-start
```

This completes the comprehensive tutorial documentation. Each tutorial provides practical, step-by-step instructions for different testing scenarios, from basic usage to advanced automation workflows.