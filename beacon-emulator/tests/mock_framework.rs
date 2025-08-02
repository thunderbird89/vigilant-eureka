//! Mock testing framework for isolated component testing
//! 
//! This module provides mock implementations and testing utilities for
//! isolating components during testing.

use beacon_emulator::*;
use shared_positioning::{BeaconConfig, GeodeticPosition, VirtualMessage};
use uuid::Uuid;
use tokio::sync::Mutex;
use std::sync::Arc;
use std::collections::HashMap;
use std::time::SystemTime;

/// Mock virtual channel for testing without real communication
pub struct MockVirtualChannel {
    pub name: String,
    pub sent_messages: Arc<Mutex<Vec<VirtualMessage>>>,
    pub should_fail: bool,
    pub fail_after_count: Option<usize>,
    pub broadcast_delay_ms: Option<u64>,
}

impl MockVirtualChannel {
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            sent_messages: Arc::new(Mutex::new(Vec::new())),
            should_fail: false,
            fail_after_count: None,
            broadcast_delay_ms: None,
        }
    }
    
    pub fn with_failure(mut self, should_fail: bool) -> Self {
        self.should_fail = should_fail;
        self
    }
    
    pub fn with_failure_after(mut self, count: usize) -> Self {
        self.fail_after_count = Some(count);
        self
    }
    
    pub fn with_broadcast_delay(mut self, delay_ms: u64) -> Self {
        self.broadcast_delay_ms = Some(delay_ms);
        self
    }
    
    pub async fn broadcast_message(&self, message: VirtualMessage) -> Result<(), EmulatorError> {
        // Simulate broadcast delay if configured
        if let Some(delay) = self.broadcast_delay_ms {
            tokio::time::sleep(tokio::time::Duration::from_millis(delay)).await;
        }
        
        let mut messages = self.sent_messages.lock().await;
        
        // Check failure conditions
        if self.should_fail {
            return Err(EmulatorError::ChannelError("Mock failure".to_string()));
        }
        
        if let Some(fail_count) = self.fail_after_count {
            if messages.len() >= fail_count {
                return Err(EmulatorError::ChannelError(
                    format!("Mock failure after {} messages", fail_count)
                ));
            }
        }
        
        messages.push(message);
        Ok(())
    }
    
    pub fn subscribe(&self) -> MockReceiver {
        MockReceiver::new(self.sent_messages.clone())
    }
    
    pub async fn get_message_count(&self) -> usize {
        let messages = self.sent_messages.lock().await;
        messages.len()
    }
    
    pub async fn get_messages(&self) -> Vec<VirtualMessage> {
        let messages = self.sent_messages.lock().await;
        messages.clone()
    }
    
    pub async fn clear_messages(&self) {
        let mut messages = self.sent_messages.lock().await;
        messages.clear();
    }
    
    pub fn name(&self) -> &str {
        &self.name
    }
}

/// Mock receiver for testing message reception
pub struct MockReceiver {
    messages: Arc<Mutex<Vec<VirtualMessage>>>,
    read_index: usize,
}

impl MockReceiver {
    fn new(messages: Arc<Mutex<Vec<VirtualMessage>>>) -> Self {
        Self {
            messages,
            read_index: 0,
        }
    }
    
    pub async fn recv(&mut self) -> Result<VirtualMessage, EmulatorError> {
        let messages = self.messages.lock().await;
        
        if self.read_index < messages.len() {
            let message = messages[self.read_index].clone();
            self.read_index += 1;
            Ok(message)
        } else {
            Err(EmulatorError::ChannelError("No more messages".to_string()))
        }
    }
    
    pub async fn try_recv(&mut self) -> Option<VirtualMessage> {
        self.recv().await.ok()
    }
}

/// Mock communication space for testing
pub struct MockCommunicationSpace {
    channels: HashMap<String, MockVirtualChannel>,
    global_failure: bool,
}

impl MockCommunicationSpace {
    pub fn new() -> Self {
        Self {
            channels: HashMap::new(),
            global_failure: false,
        }
    }
    
    pub fn with_global_failure(mut self, should_fail: bool) -> Self {
        self.global_failure = should_fail;
        self
    }
    
    pub fn get_or_create_channel(&mut self, name: &str) -> MockVirtualChannel {
        if self.global_failure {
            return MockVirtualChannel::new(name).with_failure(true);
        }
        
        self.channels.entry(name.to_string())
            .or_insert_with(|| MockVirtualChannel::new(name))
            .clone()
    }
    
    pub fn add_channel(&mut self, name: &str, channel: MockVirtualChannel) {
        self.channels.insert(name.to_string(), channel);
    }
    
    pub fn get_channel(&self, name: &str) -> Option<&MockVirtualChannel> {
        self.channels.get(name)
    }
    
    pub fn list_channels(&self) -> Vec<String> {
        self.channels.keys().cloned().collect()
    }
}

impl Clone for MockVirtualChannel {
    fn clone(&self) -> Self {
        Self {
            name: self.name.clone(),
            sent_messages: self.sent_messages.clone(),
            should_fail: self.should_fail,
            fail_after_count: self.fail_after_count,
            broadcast_delay_ms: self.broadcast_delay_ms,
        }
    }
}

/// Mock beacon configuration for testing
pub struct MockBeaconConfig;

impl MockBeaconConfig {
    pub fn create_test_config() -> BeaconConfig {
        BeaconConfig::new(Uuid::new_v4())
    }
    
    pub fn create_fast_config() -> BeaconConfig {
        let mut config = BeaconConfig::new(Uuid::new_v4());
        config.transmission.interval_ms = 100; // Fast transmission for testing
        config
    }
    
    pub fn create_slow_config() -> BeaconConfig {
        let mut config = BeaconConfig::new(Uuid::new_v4());
        config.transmission.interval_ms = 10000; // Slow transmission for testing
        config
    }
    
    pub fn create_config_with_interval(interval_ms: u32) -> BeaconConfig {
        let mut config = BeaconConfig::new(Uuid::new_v4());
        config.transmission.interval_ms = interval_ms;
        config
    }
}

/// Mock position generator for testing
pub struct MockPositionGenerator;

impl MockPositionGenerator {
    pub fn create_test_position() -> GeodeticPosition {
        GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        }
    }
    
    pub fn create_positions_grid(rows: usize, cols: usize, spacing: f64) -> Vec<GeodeticPosition> {
        let mut positions = Vec::new();
        let base_lat = 32.0;
        let base_lon = 45.0;
        let base_depth = 10.0;
        
        for row in 0..rows {
            for col in 0..cols {
                positions.push(GeodeticPosition {
                    latitude: base_lat + (row as f64) * spacing / 111_132.0,
                    longitude: base_lon + (col as f64) * spacing / 111_320.0,
                    depth: base_depth,
                });
            }
        }
        
        positions
    }
    
    pub fn create_positions_line(count: usize, spacing: f64) -> Vec<GeodeticPosition> {
        let mut positions = Vec::new();
        let base_lat = 32.0;
        let base_lon = 45.0;
        let base_depth = 10.0;
        
        for i in 0..count {
            positions.push(GeodeticPosition {
                latitude: base_lat + (i as f64) * spacing / 111_132.0,
                longitude: base_lon,
                depth: base_depth,
            });
        }
        
        positions
    }
    
    pub fn create_invalid_position() -> GeodeticPosition {
        GeodeticPosition {
            latitude: 95.0, // Invalid: > 90
            longitude: 45.476,
            depth: 10.0,
        }
    }
}

/// Mock message builder for testing
pub struct MockMessageBuilder {
    pub should_fail: bool,
    pub message_size: usize,
    pub sequence_counter: Arc<Mutex<u16>>,
}

impl MockMessageBuilder {
    pub fn new() -> Self {
        Self {
            should_fail: false,
            message_size: 32, // Default message size
            sequence_counter: Arc::new(Mutex::new(0)),
        }
    }
    
    pub fn with_failure(mut self, should_fail: bool) -> Self {
        self.should_fail = should_fail;
        self
    }
    
    pub fn with_message_size(mut self, size: usize) -> Self {
        self.message_size = size;
        self
    }
    
    pub async fn build_message(
        &self,
        beacon_id: Uuid,
        position: GeodeticPosition,
        signal_quality: u8,
        sequence_number: u16,
    ) -> Result<Vec<u8>, EmulatorError> {
        if self.should_fail {
            return Err(EmulatorError::MessageBuildError("Mock failure".to_string()));
        }
        
        // Create mock message data
        let mut message = Vec::with_capacity(self.message_size);
        
        // Add beacon ID bytes
        message.extend_from_slice(&beacon_id.as_bytes()[0..8]);
        
        // Add position data (simplified)
        message.extend_from_slice(&(position.latitude as f32).to_le_bytes());
        message.extend_from_slice(&(position.longitude as f32).to_le_bytes());
        message.extend_from_slice(&(position.depth as f32).to_le_bytes());
        
        // Add signal quality and sequence number
        message.push(signal_quality);
        message.extend_from_slice(&sequence_number.to_le_bytes());
        
        // Pad to desired size
        while message.len() < self.message_size {
            message.push(0);
        }
        
        // Update sequence counter
        let mut counter = self.sequence_counter.lock().await;
        *counter = counter.wrapping_add(1);
        
        Ok(message)
    }
    
    pub async fn get_sequence_count(&self) -> u16 {
        let counter = self.sequence_counter.lock().await;
        *counter
    }
    
    pub async fn reset_sequence(&self) {
        let mut counter = self.sequence_counter.lock().await;
        *counter = 0;
    }
}

/// Test harness for isolated component testing
pub struct ComponentTestHarness {
    pub mock_channel: MockVirtualChannel,
    pub mock_config: BeaconConfig,
    pub mock_position: GeodeticPosition,
    pub mock_message_builder: MockMessageBuilder,
}

impl ComponentTestHarness {
    pub fn new() -> Self {
        Self {
            mock_channel: MockVirtualChannel::new("test_channel"),
            mock_config: MockBeaconConfig::create_test_config(),
            mock_position: MockPositionGenerator::create_test_position(),
            mock_message_builder: MockMessageBuilder::new(),
        }
    }
    
    pub fn with_failing_channel(mut self) -> Self {
        self.mock_channel = self.mock_channel.with_failure(true);
        self
    }
    
    pub fn with_delayed_channel(mut self, delay_ms: u64) -> Self {
        self.mock_channel = self.mock_channel.with_broadcast_delay(delay_ms);
        self
    }
    
    pub fn with_fast_config(mut self) -> Self {
        self.mock_config = MockBeaconConfig::create_fast_config();
        self
    }
    
    pub fn with_failing_message_builder(mut self) -> Self {
        self.mock_message_builder = self.mock_message_builder.with_failure(true);
        self
    }
    
    // Note: Cannot create VirtualBeacon with MockVirtualChannel due to type mismatch
    // This would require implementing a trait or using dependency injection
    // For now, we'll test the mock components separately
    
    pub async fn verify_no_messages_sent(&self) {
        assert_eq!(self.mock_channel.get_message_count().await, 0);
    }
    
    pub async fn verify_messages_sent(&self, expected_count: usize) {
        assert_eq!(self.mock_channel.get_message_count().await, expected_count);
    }
    
    pub async fn get_sent_messages(&self) -> Vec<VirtualMessage> {
        self.mock_channel.get_messages().await
    }
}

/// Utility functions for mock testing
pub mod mock_utils {
    use super::*;
    
    pub fn create_mock_virtual_message(beacon_id: Uuid, timestamp: SystemTime) -> VirtualMessage {
        VirtualMessage {
            beacon_id,
            timestamp,
            position: MockPositionGenerator::create_test_position(),
            message_data: vec![0x01, 0x02, 0x03, 0x04],
            signal_quality: 255,
        }
    }
    
    pub async fn wait_for_messages(
        receiver: &mut MockReceiver,
        expected_count: usize,
        timeout_ms: u64,
    ) -> Vec<VirtualMessage> {
        let mut messages = Vec::new();
        let start_time = std::time::Instant::now();
        let timeout = std::time::Duration::from_millis(timeout_ms);
        
        while messages.len() < expected_count && start_time.elapsed() < timeout {
            if let Some(message) = receiver.try_recv().await {
                messages.push(message);
            } else {
                tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
            }
        }
        
        messages
    }
    
    pub fn assert_message_valid(message: &VirtualMessage, expected_beacon_id: Uuid) {
        assert_eq!(message.beacon_id, expected_beacon_id);
        assert!(!message.message_data.is_empty());
        assert!(message.signal_quality > 0);
        assert!(message.timestamp <= SystemTime::now());
    }
    
    pub fn assert_position_approximately_equal(pos1: GeodeticPosition, pos2: GeodeticPosition, tolerance: f64) {
        assert!((pos1.latitude - pos2.latitude).abs() < tolerance, 
                "Latitude difference too large: {} vs {}", pos1.latitude, pos2.latitude);
        assert!((pos1.longitude - pos2.longitude).abs() < tolerance,
                "Longitude difference too large: {} vs {}", pos1.longitude, pos2.longitude);
        assert!((pos1.depth - pos2.depth).abs() < tolerance,
                "Depth difference too large: {} vs {}", pos1.depth, pos2.depth);
    }
}

#[cfg(test)]
mod mock_framework_tests {
    use super::*;
    use tokio::time::Duration;
    
    #[tokio::test]
    async fn test_mock_virtual_channel() {
        let channel = MockVirtualChannel::new("test");
        let beacon_id = Uuid::new_v4();
        let message = mock_utils::create_mock_virtual_message(beacon_id, SystemTime::now());
        
        // Test successful message broadcast
        channel.broadcast_message(message.clone()).await.unwrap();
        assert_eq!(channel.get_message_count().await, 1);
        
        let messages = channel.get_messages().await;
        assert_eq!(messages.len(), 1);
        assert_eq!(messages[0].beacon_id, beacon_id);
    }
    
    #[tokio::test]
    async fn test_mock_channel_failure() {
        let channel = MockVirtualChannel::new("test").with_failure(true);
        let beacon_id = Uuid::new_v4();
        let message = mock_utils::create_mock_virtual_message(beacon_id, SystemTime::now());
        
        // Should fail to broadcast
        let result = channel.broadcast_message(message).await;
        assert!(result.is_err());
        assert_eq!(channel.get_message_count().await, 0);
    }
    
    #[tokio::test]
    async fn test_mock_channel_failure_after_count() {
        let channel = MockVirtualChannel::new("test").with_failure_after(2);
        let beacon_id = Uuid::new_v4();
        
        // First two messages should succeed
        for _ in 0..2 {
            let message = mock_utils::create_mock_virtual_message(beacon_id, SystemTime::now());
            channel.broadcast_message(message).await.unwrap();
        }
        
        assert_eq!(channel.get_message_count().await, 2);
        
        // Third message should fail
        let message = mock_utils::create_mock_virtual_message(beacon_id, SystemTime::now());
        let result = channel.broadcast_message(message).await;
        assert!(result.is_err());
        assert_eq!(channel.get_message_count().await, 2);
    }
    
    #[tokio::test]
    async fn test_mock_channel_delay() {
        let channel = MockVirtualChannel::new("test").with_broadcast_delay(100);
        let beacon_id = Uuid::new_v4();
        let message = mock_utils::create_mock_virtual_message(beacon_id, SystemTime::now());
        
        let start_time = std::time::Instant::now();
        channel.broadcast_message(message).await.unwrap();
        let elapsed = start_time.elapsed();
        
        // Should have taken at least 100ms due to delay
        assert!(elapsed >= Duration::from_millis(90)); // Allow some tolerance
        assert_eq!(channel.get_message_count().await, 1);
    }
    
    #[tokio::test]
    async fn test_mock_receiver() {
        let channel = MockVirtualChannel::new("test");
        let mut receiver = channel.subscribe();
        let beacon_id = Uuid::new_v4();
        
        // Send messages
        for i in 0..3 {
            let mut message = mock_utils::create_mock_virtual_message(beacon_id, SystemTime::now());
            message.message_data = vec![i as u8];
            channel.broadcast_message(message).await.unwrap();
        }
        
        // Receive messages
        let mut received_messages = Vec::new();
        for _ in 0..3 {
            let message = receiver.recv().await.unwrap();
            received_messages.push(message);
        }
        
        assert_eq!(received_messages.len(), 3);
        for (i, message) in received_messages.iter().enumerate() {
            assert_eq!(message.beacon_id, beacon_id);
            assert_eq!(message.message_data[0], i as u8);
        }
        
        // No more messages should be available
        let result = receiver.recv().await;
        assert!(result.is_err());
    }
    
    #[tokio::test]
    async fn test_component_test_harness() {
        let harness = ComponentTestHarness::new();
        
        // Test harness creation
        assert_eq!(harness.mock_position.latitude, 32.123);
        
        // Verify no messages sent initially
        harness.verify_no_messages_sent().await;
    }
    
    #[tokio::test]
    async fn test_component_test_harness_with_failures() {
        let harness = ComponentTestHarness::new().with_failing_channel();
        
        // Test harness with failing channel
        assert!(harness.mock_channel.should_fail);
        
        // Verify no messages sent initially
        harness.verify_no_messages_sent().await;
    }
    
    #[tokio::test]
    async fn test_mock_position_generator() {
        let position = MockPositionGenerator::create_test_position();
        assert_eq!(position.latitude, 32.123);
        assert_eq!(position.longitude, 45.476);
        assert_eq!(position.depth, 10.0);
        
        let grid_positions = MockPositionGenerator::create_positions_grid(2, 2, 100.0);
        assert_eq!(grid_positions.len(), 4);
        
        let line_positions = MockPositionGenerator::create_positions_line(3, 50.0);
        assert_eq!(line_positions.len(), 3);
        
        // All line positions should have same longitude
        for pos in &line_positions {
            assert_eq!(pos.longitude, 45.0);
        }
    }
    
    #[tokio::test]
    async fn test_mock_message_builder() {
        let builder = MockMessageBuilder::new();
        let beacon_id = Uuid::new_v4();
        let position = MockPositionGenerator::create_test_position();
        
        let message = builder.build_message(beacon_id, position, 255, 1).await.unwrap();
        assert!(!message.is_empty());
        assert_eq!(message.len(), 32); // Default size
        
        // Test with custom size
        let builder = MockMessageBuilder::new().with_message_size(64);
        let message = builder.build_message(beacon_id, position, 255, 1).await.unwrap();
        assert_eq!(message.len(), 64);
        
        // Test failure
        let builder = MockMessageBuilder::new().with_failure(true);
        let result = builder.build_message(beacon_id, position, 255, 1).await;
        assert!(result.is_err());
    }
    
    #[tokio::test]
    async fn test_mock_utils() {
        let beacon_id = Uuid::new_v4();
        let message = mock_utils::create_mock_virtual_message(beacon_id, SystemTime::now());
        
        mock_utils::assert_message_valid(&message, beacon_id);
        
        let pos1 = GeodeticPosition { latitude: 32.0, longitude: 45.0, depth: 10.0 };
        let pos2 = GeodeticPosition { latitude: 32.001, longitude: 45.001, depth: 10.1 };
        
        mock_utils::assert_position_approximately_equal(pos1, pos2, 0.15);
    }
}