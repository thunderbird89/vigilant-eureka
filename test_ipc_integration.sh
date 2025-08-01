#!/bin/bash

# Test script for IPC integration between beacon-emulator and receiver

echo "Testing IPC integration between beacon-emulator and receiver..."

# Start beacon emulator in background with IPC server
echo "Starting beacon emulator with IPC server..."
cd beacon-emulator
cargo run -- create --id test-beacon-1 --lat 35.0 --lon 25.0 --depth 10.0 --channel test-channel &
EMULATOR_PID=$!

# Wait for emulator to start
sleep 3

# Start virtual receiver to connect to the emulator
echo "Starting virtual receiver..."
cd ../receiver
timeout 10s cargo run -- virtual --channel test-channel --receiver-id 1 --update-interval 1000 --max-attempts 10 &
RECEIVER_PID=$!

# Wait for test to run
sleep 8

# Clean up
echo "Cleaning up processes..."
kill $EMULATOR_PID 2>/dev/null
kill $RECEIVER_PID 2>/dev/null

echo "Test completed"