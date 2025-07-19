//! Mock transceiver implementation for testing and development

use crate::hardware::{
    TransceiverInterface, TransceiverStatus, TransceiverConfig, 
    RawMessage, CommError, CommResult
};
use std::collections::VecDeque;
use std::time::{SystemTime, UNIX_EPOCH};

/// Mock transceiver for testing and development
pub struct MockTransceiver {
    id: u8,
    status: TransceiverStatus,
    config: TransceiverConfig,
    message_queue: VecDeque<RawMessage>,
    sent_messages: Vec<Vec<u8>>,
    simulate_errors: bool,
    error_probability: f32,
    connected: bool,
}

impl MockTransceiver {
    /// Create a new mock transceiver
    pub fn new(id: u8) -> Self {
        let mut status = TransceiverStatus::new(id);
        status.connected = true;
        status.firmware_version = Some("MockTransceiver v1.0".to_string());
        
        Self {
            id,
            status,
            config: TransceiverConfig::mock(id),
            message_queue: VecDeque::new(),
            sent_messages: Vec::new(),
            simulate_errors: false,
            error_probability: 0.0,
            connected: true,
        }
    }
    
    /// Add a message to the mock transceiver's queue
    pub fn add_message(&mut self, data: Vec<u8>) {
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;
            
        let message = RawMessage::new(data)
            .with_timestamp(timestamp)
            .with_signal_strength(75); // Mock signal strength
            
        self.message_queue.push_back(message);
    }
    
    /// Add a JANUS anchor message to the queue
    pub fn add_anchor_message(&mut self, anchor_id: u16, lat: f64, lon: f64, depth: f64) {
        // Simulate a simple JANUS message format
        let mut data = Vec::new();
        data.extend_from_slice(&anchor_id.to_le_bytes());
        data.extend_from_slice(&lat.to_le_bytes());
        data.extend_from_slice(&lon.to_le_bytes());
        data.extend_from_slice(&depth.to_le_bytes());
        
        self.add_message(data);
    }
    
    /// Enable error simulation with given probability (0.0 to 1.0)
    pub fn simulate_errors(&mut self, enable: bool, probability: f32) {
        self.simulate_errors = enable;
        self.error_probability = probability.clamp(0.0, 1.0);
    }
    
    /// Simulate connection loss
    pub fn disconnect(&mut self) {
        self.connected = false;
        self.status.connected = false;
    }
    
    /// Restore connection
    pub fn reconnect(&mut self) {
        self.connected = true;
        self.status.connected = true;
    }
    
    /// Get all messages that were sent to this transceiver
    pub fn get_sent_messages(&self) -> &[Vec<u8>] {
        &self.sent_messages
    }
    
    /// Clear the sent messages history
    pub fn clear_sent_messages(&mut self) {
        self.sent_messages.clear();
    }
    
    /// Get the number of queued messages
    pub fn queued_message_count(&self) -> usize {
        self.message_queue.len()
    }
    
    /// Check if we should simulate an error
    fn should_simulate_error(&self) -> bool {
        if !self.simulate_errors {
            return false;
        }
        
        use rand::Rng;
        let mut rng = rand::thread_rng();
        rng.gen::<f32>() < self.error_probability
    }
}

impl TransceiverInterface for MockTransceiver {
    fn read_message(&mut self) -> CommResult<Option<RawMessage>> {
        if !self.connected {
            return Err(CommError::ConnectionLost { 
                transceiver_id: self.id 
            });
        }
        
        if self.should_simulate_error() {
            self.status.error_count += 1;
            return Err(CommError::Timeout { 
                timeout_ms: self.config.read_timeout_ms 
            });
        }
        
        if let Some(message) = self.message_queue.pop_front() {
            self.status.messages_received += 1;
            self.status.last_message_time = Some(message.timestamp_ms);
            Ok(Some(message))
        } else {
            Ok(None)
        }
    }
    
    fn send_message(&mut self, data: &[u8]) -> CommResult<()> {
        if !self.connected {
            return Err(CommError::ConnectionLost { 
                transceiver_id: self.id 
            });
        }
        
        if self.should_simulate_error() {
            self.status.error_count += 1;
            return Err(CommError::HardwareError {
                code: 1001,
                description: "Simulated send failure".to_string(),
            });
        }
        
        if data.len() > self.config.max_message_size {
            return Err(CommError::BufferError {
                operation: "send".to_string(),
            });
        }
        
        self.sent_messages.push(data.to_vec());
        Ok(())
    }
    
    fn get_status(&self) -> TransceiverStatus {
        self.status.clone()
    }
    
    fn configure(&mut self, config: &TransceiverConfig) -> CommResult<()> {
        config.validate()?;
        
        if config.id != self.id {
            return Err(CommError::ConfigurationError {
                parameter: "id".to_string(),
                value: format!("expected {}, got {}", self.id, config.id),
            });
        }
        
        self.config = config.clone();
        Ok(())
    }
    
    fn reset(&mut self) -> CommResult<()> {
        if self.should_simulate_error() {
            return Err(CommError::HardwareError {
                code: 2001,
                description: "Reset failed".to_string(),
            });
        }
        
        self.message_queue.clear();
        self.sent_messages.clear();
        self.status.error_count = 0;
        self.status.messages_received = 0;
        self.status.last_message_time = None;
        
        // Simulate reset delay
        std::thread::sleep(std::time::Duration::from_millis(10));
        
        Ok(())
    }
    
    fn is_connected(&self) -> bool {
        self.connected
    }
    
    fn get_id(&self) -> u8 {
        self.id
    }
    
    fn flush(&mut self) -> CommResult<()> {
        if !self.connected {
            return Err(CommError::ConnectionLost { 
                transceiver_id: self.id 
            });
        }
        
        // Mock flush operation - clear any pending data
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_mock_transceiver_creation() {
        let transceiver = MockTransceiver::new(1);
        assert_eq!(transceiver.get_id(), 1);
        assert!(transceiver.is_connected());
        assert_eq!(transceiver.queued_message_count(), 0);
    }
    
    #[test]
    fn test_message_queue() {
        let mut transceiver = MockTransceiver::new(1);
        
        // Add a message
        transceiver.add_message(vec![1, 2, 3, 4]);
        assert_eq!(transceiver.queued_message_count(), 1);
        
        // Read the message
        let result = transceiver.read_message().unwrap();
        assert!(result.is_some());
        let message = result.unwrap();
        assert_eq!(message.data, vec![1, 2, 3, 4]);
        assert_eq!(transceiver.queued_message_count(), 0);
        
        // No more messages
        let result = transceiver.read_message().unwrap();
        assert!(result.is_none());
    }
    
    #[test]
    fn test_anchor_message() {
        let mut transceiver = MockTransceiver::new(1);
        transceiver.add_anchor_message(123, 45.0, -122.0, 10.5);
        
        let result = transceiver.read_message().unwrap();
        assert!(result.is_some());
        let message = result.unwrap();
        assert_eq!(message.data.len(), 2 + 8 + 8 + 8); // u16 + 3 * f64
    }
    
    #[test]
    fn test_connection_simulation() {
        let mut transceiver = MockTransceiver::new(1);
        
        // Disconnect
        transceiver.disconnect();
        assert!(!transceiver.is_connected());
        
        let result = transceiver.read_message();
        assert!(matches!(result, Err(CommError::ConnectionLost { .. })));
        
        // Reconnect
        transceiver.reconnect();
        assert!(transceiver.is_connected());
        
        let result = transceiver.read_message().unwrap();
        assert!(result.is_none()); // No messages, but no error
    }
    
    #[test]
    fn test_error_simulation() {
        let mut transceiver = MockTransceiver::new(1);
        transceiver.simulate_errors(true, 1.0); // 100% error rate
        
        let result = transceiver.read_message();
        assert!(result.is_err());
        assert!(transceiver.get_status().error_count > 0);
    }
}