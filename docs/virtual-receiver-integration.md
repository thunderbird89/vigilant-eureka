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

To use the virtual receiver with the beacon emulator:

1. Start the beacon emulator with virtual beacons on a specific channel:
   ```bash
   cd beacon-emulator
   cargo run -- create --channel test_channel --lat 32.123 --lon 45.476 --depth 0
   ```

2. Start the virtual receiver on the same channel:
   ```bash
   cd receiver
   cargo run -- virtual --channel test_channel
   ```

3. The receiver will connect to the virtual channel and begin receiving beacon transmissions

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