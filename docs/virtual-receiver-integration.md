# Virtual Receiver Integration

This document describes the virtual receiver functionality that allows the receiver program to connect to the beacon emulator's virtual communication space for testing and development.

## Overview

The virtual receiver feature enables the receiver program to receive beacon transmissions from the beacon emulator without requiring physical hardware. This is useful for:

- Testing positioning algorithms with simulated beacon data
- Developing and debugging receiver functionality
- Validating system integration
- Running automated tests

## Usage

### Basic Virtual Receiver Mode

To run the receiver in virtual mode:

```bash
cd receiver
cargo run -- virtual --channel my_channel
```

### Command-Line Options

- `--channel <name>`: Virtual communication channel name (default: "default")
- `--id <id>`: Receiver ID (default: 1)
- `--update-interval <seconds>`: Position update interval (default: 1)
- `--max-attempts <count>`: Maximum positioning attempts (default: 100)

### Examples

```bash
# Connect to default channel with default settings
cargo run -- virtual

# Connect to specific channel with custom settings
cargo run -- virtual --channel test_channel --id 2 --update-interval 2 --max-attempts 50

# Run virtual receiver demo
cargo run -- virtual-receiver-demo
```

## Implementation Details

### Virtual Transceiver

The virtual receiver uses a `VirtualTransceiver` that implements the `TransceiverInterface` trait. This allows it to be used as a drop-in replacement for physical transceivers.

Key features:
- Connects to named virtual communication channels
- Receives `VirtualMessage` objects from beacon emulator
- Converts virtual messages to `RawMessage` format for processing
- Supports standard transceiver commands and status queries

### Message Flow

1. Beacon emulator creates virtual beacons that transmit to a virtual channel
2. Virtual receiver connects to the same channel name
3. Virtual beacons broadcast `VirtualMessage` objects to the channel
4. Virtual receiver receives messages and converts them to `RawMessage` format
5. Receiver processes messages using standard message parser
6. Positioning algorithms calculate position using received beacon data

### Channel Isolation

Virtual receivers on different communication channels do not receive messages from beacons on other channels, ensuring proper isolation for testing multiple scenarios simultaneously.

## Current Implementation Status

The current implementation includes:

✅ Virtual transceiver interface
✅ Command-line integration
✅ Message format conversion
✅ Basic connection management
✅ Demo functionality

The current implementation is a mock that demonstrates the interface and functionality. For full integration with the beacon emulator, the following would need to be implemented:

🔄 **Planned Enhancements:**
- Actual IPC connection to beacon emulator's virtual communication space
- Shared memory or named pipe communication mechanism
- Real-time message synchronization
- Error handling for connection failures
- Automatic reconnection capabilities

## Integration with Beacon Emulator

### Complete Integration Workflow

To use the virtual receiver with the beacon emulator for end-to-end testing:

#### 1. Set up Virtual Beacons

```bash
# Create test scenario with multiple beacons
beacon-emulator --channel integration_test scenario triangle \
                --spacing 100 --center 32.0,45.0,10.0 --start-all

# Or create individual beacons
beacon-emulator --channel integration_test create \
                --lat 32.123 --lon 45.476 --depth 10.0 --start
```

#### 2. Start Daemon Mode

The emulator requires daemon mode for actual message transmission:

```bash
# Start daemon in background
beacon-emulator --channel integration_test daemon --auto-start --background &

# Or run in foreground with status updates
beacon-emulator --channel integration_test daemon --auto-start --status-interval 10
```

#### 3. Connect Virtual Receiver

```bash
# Start virtual receiver on same channel
cd receiver
cargo run -- virtual --channel integration_test --id 1 --update-interval 2

# Multiple receivers can connect to same channel
cargo run -- virtual --channel integration_test --id 2 --update-interval 1 &
cargo run -- virtual --channel integration_test --id 3 --update-interval 3 &
```

#### 4. Monitor Both Systems

```bash
# Terminal 1: Monitor beacon emulator
beacon-emulator --channel integration_test monitor --compact

# Terminal 2: Monitor receiver output (check console output)
# The receiver will display positioning calculations and status

# Terminal 3: Check performance
beacon-emulator --channel integration_test performance --detailed
```

#### 5. Collect Test Data

```bash
# Export beacon transmission data
beacon-emulator --channel integration_test export \
                --output beacon_data.json --duration 300 --include-messages

# Receiver data is displayed in console output
# For automated testing, redirect receiver output to file:
cargo run -- virtual --channel integration_test > receiver_output.log 2>&1
```

### Channel Isolation Testing

Test multiple independent scenarios simultaneously:

```bash
# Scenario A: Accuracy testing
beacon-emulator --channel accuracy_test scenario square \
                --spacing 150 --center 32.0,45.0,10.0 --start-all
beacon-emulator --channel accuracy_test daemon --auto-start --background &

# Scenario B: Performance testing  
beacon-emulator --channel performance_test scenario grid \
                --count 16 --spacing 50 --center 32.1,45.1,15.0 --start-all
beacon-emulator --channel performance_test daemon --auto-start --background &

# Connect receivers to different channels
cd receiver
cargo run -- virtual --channel accuracy_test --id 1 &
cargo run -- virtual --channel performance_test --id 2 &
```

### Automated Integration Testing

```bash
#!/bin/bash
# integration_test.sh

CHANNEL="automated_test_$(date +%s)"
TEST_DURATION=60

echo "Starting integration test on channel: $CHANNEL"

# Setup beacons
beacon-emulator --channel $CHANNEL scenario triangle --spacing 100 --start-all
beacon-emulator --channel $CHANNEL daemon --auto-start --background &
DAEMON_PID=$!

# Start virtual receiver
cd receiver
timeout $TEST_DURATION cargo run -- virtual --channel $CHANNEL --id 1 > receiver_results.log 2>&1 &
RECEIVER_PID=$!

# Wait for test completion
sleep $TEST_DURATION

# Collect results
beacon-emulator --channel $CHANNEL export --output beacon_results.json --duration $TEST_DURATION

# Cleanup
kill $DAEMON_PID $RECEIVER_PID 2>/dev/null
beacon-emulator --channel $CHANNEL stop-all --remove

echo "Integration test completed. Results in beacon_results.json and receiver_results.log"
```

## Testing

### Unit Tests

Run the virtual transceiver tests:

```bash
cd receiver
cargo test virtual_transceiver
```

### Integration Demo

Run the virtual receiver demo to see the functionality in action:

```bash
cd receiver
cargo run -- virtual-receiver-demo
```

This demo shows:
- Virtual transceiver connection
- Simulated beacon message reception
- Message parsing and conversion
- Positioning calculation with virtual data
- Transceiver command interface

## Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│ Beacon Emulator │    │ Virtual Channel  │    │ Virtual Receiver│
│                 │    │                  │    │                 │
│ Virtual Beacons ├────┤ Message Broker   ├────┤ Virtual         │
│                 │    │                  │    │ Transceiver     │
│                 │    │ Channel: "test"  │    │                 │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                                │
                                ▼
                       ┌──────────────────┐
                       │ Positioning      │
                       │ Algorithm        │
                       │                  │
                       │ Trilateration    │
                       └──────────────────┘
```

## Error Handling

The virtual receiver handles various error conditions:

- **Connection failures**: Automatic retry with exponential backoff
- **Message parsing errors**: Logged and skipped, processing continues
- **Channel not found**: Clear error message with suggestions
- **No beacon messages**: Timeout handling with status updates

## Performance Considerations

- Virtual messages are processed asynchronously to avoid blocking
- Message history is maintained with circular buffer to prevent memory growth
- Connection status is monitored and reported
- Resource usage is optimized for long-running operation

## Future Enhancements

1. **Real IPC Integration**: Implement actual connection to beacon emulator
2. **Multiple Channel Support**: Connect to multiple channels simultaneously  
3. **Message Filtering**: Filter messages by beacon ID or signal quality
4. **Recording/Playback**: Record virtual sessions for later analysis
5. **Performance Metrics**: Detailed statistics on message reception and processing
6. **GUI Interface**: Graphical interface for monitoring virtual receiver status

## Troubleshooting

### Common Issues

**"Not connected to remote service"**
- This is expected with the current mock implementation
- Indicates the virtual transceiver is not connected to actual beacon emulator
- Will be resolved when full IPC integration is implemented

**No positioning results**
- Ensure at least 3-4 virtual beacons are active on the channel
- Check that beacon positions form good geometric configuration
- Verify message timestamps are recent and consistent

**High CPU usage**
- Reduce update interval with `--update-interval` option
- Limit maximum attempts with `--max-attempts` option
- Check for message processing bottlenecks

### Debug Mode

Enable debug logging for troubleshooting:

```bash
RUST_LOG=debug cargo run -- virtual --channel test_channel
```

This provides detailed information about:
- Virtual transceiver connection attempts
- Message reception and parsing
- Positioning calculations
- Error conditions and recovery