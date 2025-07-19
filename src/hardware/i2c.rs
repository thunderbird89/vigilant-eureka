//! I2C communication module for JANUS transceivers

use crate::hardware::{
    TransceiverInterface, TransceiverStatus, TransceiverConfig, 
    RawMessage, CommError, CommResult
};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

/// I2C transceiver implementation
/// 
/// Note: This is a simplified implementation that would need to be adapted
/// for actual I2C communication using a crate like `linux-embedded-hal` or `rppal`
pub struct I2CTransceiver {
    id: u8,
    status: TransceiverStatus,
    config: TransceiverConfig,
    i2c_address: u8,
    last_read_time: Instant,
    message_buffer: Vec<u8>,
    register_cache: [u8; 256],
}

impl I2CTransceiver {
    /// Create a new I2C transceiver
    pub fn new(id: u8, config: TransceiverConfig) -> CommResult<Self> {
        config.validate()?;
        
        let i2c_address = config.i2c_address.ok_or_else(|| {
            CommError::ConfigurationError {
                parameter: "i2c_address".to_string(),
                value: "None".to_string(),
            }
        })?;
        
        let mut status = TransceiverStatus::new(id);
        status.connected = false; // Will be set when connection is established
        
        Ok(Self {
            id,
            status,
            config,
            i2c_address,
            last_read_time: Instant::now(),
            message_buffer: Vec::new(),
            register_cache: [0; 256],
        })
    }
    
    /// Attempt to establish I2C connection
    pub fn connect(&mut self) -> CommResult<()> {
        // In a real implementation, this would initialize the I2C bus
        // and verify the device is present at the specified address
        
        // Simulate device detection
        if self.i2c_address > 0x7F {
            return Err(CommError::ConfigurationError {
                parameter: "i2c_address".to_string(),
                value: format!("0x{:02X} (must be <= 0x7F)", self.i2c_address),
            });
        }
        
        // Simulate connection delay
        std::thread::sleep(Duration::from_millis(50));
        
        self.status.connected = true;
        self.status.firmware_version = Some("JANUS I2C v1.5".to_string());
        
        // Initialize device registers (simulation)
        self.write_register(0x00, 0x01)?; // Enable device
        self.write_register(0x01, 0x80)?; // Set default configuration
        
        Ok(())
    }
    
    /// Disconnect from I2C device
    pub fn disconnect(&mut self) {
        self.status.connected = false;
        self.message_buffer.clear();
        self.register_cache = [0; 256];
    }
    
    /// Write to an I2C register
    fn write_register(&mut self, register: u8, value: u8) -> CommResult<()> {
        if !self.status.connected {
            return Err(CommError::ConnectionLost { 
                transceiver_id: self.id 
            });
        }
        
        // In a real implementation, this would write to the I2C device
        // For simulation, we'll update our register cache
        self.register_cache[register as usize] = value;
        
        // Simulate I2C transaction time
        std::thread::sleep(Duration::from_micros(100));
        
        Ok(())
    }
    
    /// Read from an I2C register
    fn read_register(&mut self, register: u8) -> CommResult<u8> {
        if !self.status.connected {
            return Err(CommError::ConnectionLost { 
                transceiver_id: self.id 
            });
        }
        
        // Simulate I2C transaction time
        std::thread::sleep(Duration::from_micros(100));
        
        // In a real implementation, this would read from the I2C device
        // For simulation, we'll return from our register cache
        Ok(self.register_cache[register as usize])
    }
    
    /// Read multiple bytes from I2C device
    fn read_bytes(&mut self, register: u8, count: usize) -> CommResult<Vec<u8>> {
        if !self.status.connected {
            return Err(CommError::ConnectionLost { 
                transceiver_id: self.id 
            });
        }
        
        if count > 64 {
            return Err(CommError::BufferError {
                operation: "read_bytes".to_string(),
            });
        }
        
        let mut data = Vec::with_capacity(count);
        for i in 0..count {
            let reg_addr = register.wrapping_add(i as u8);
            data.push(self.register_cache[reg_addr as usize]);
        }
        
        // Simulate I2C transaction time
        std::thread::sleep(Duration::from_micros(50 * count as u64));
        
        Ok(data)
    }
    
    /// Write multiple bytes to I2C device
    fn write_bytes(&mut self, register: u8, data: &[u8]) -> CommResult<()> {
        if !self.status.connected {
            return Err(CommError::ConnectionLost { 
                transceiver_id: self.id 
            });
        }
        
        if data.len() > 64 {
            return Err(CommError::BufferError {
                operation: "write_bytes".to_string(),
            });
        }
        
        for (i, &byte) in data.iter().enumerate() {
            let reg_addr = register.wrapping_add(i as u8);
            self.register_cache[reg_addr as usize] = byte;
        }
        
        // Simulate I2C transaction time
        std::thread::sleep(Duration::from_micros(50 * data.len() as u64));
        
        Ok(())
    }
    
    /// Check for available messages in device buffer
    fn check_message_available(&mut self) -> CommResult<bool> {
        // Check status register for message availability
        let status = self.read_register(0x02)?;
        Ok((status & 0x01) != 0) // Bit 0 indicates message available
    }
    
    /// Read message from device buffer
    fn read_device_message(&mut self) -> CommResult<Option<RawMessage>> {
        if !self.check_message_available()? {
            return Ok(None);
        }
        
        // Read message length from register
        let length = self.read_register(0x03)?;
        if length == 0 || length as usize > self.config.max_message_size {
            return Err(CommError::InvalidMessage {
                details: format!("Invalid message length: {}", length),
            });
        }
        
        // Read message data from buffer registers (starting at 0x10)
        let data = self.read_bytes(0x10, length as usize)?;
        
        // Read checksum if enabled
        let checksum = if self.config.enable_checksum {
            Some(self.read_register(0x10 + length)? as u16)
        } else {
            None
        };
        
        // Verify checksum if present
        if let Some(expected_checksum) = checksum {
            let calculated_checksum = data.iter().fold(0u8, |acc, &b| acc.wrapping_add(b)) as u16;
            if calculated_checksum != expected_checksum {
                self.status.error_count += 1;
                return Err(CommError::ChecksumError {
                    expected: calculated_checksum,
                    received: expected_checksum,
                });
            }
        }
        
        // Clear message available flag
        self.write_register(0x02, 0x00)?;
        
        let timestamp = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;
        
        let mut message = RawMessage::new(data).with_timestamp(timestamp);
        
        if let Some(cs) = checksum {
            message = message.with_checksum(cs);
        }
        
        // Read signal strength if available
        let signal_strength = self.read_register(0x04)?;
        if signal_strength > 0 {
            message = message.with_signal_strength(signal_strength);
        }
        
        self.status.messages_received += 1;
        self.status.last_message_time = Some(timestamp);
        self.status.signal_strength = Some(signal_strength);
        
        Ok(Some(message))
    }
    
    /// Simulate periodic message arrival for testing
    fn simulate_message_arrival(&mut self) {
        if self.last_read_time.elapsed() > Duration::from_secs(2) {
            // Simulate a test message arrival
            let test_data = vec![0x42, 0x43, 0x44, 0x45]; // Mock anchor data
            let checksum = test_data.iter().fold(0u8, |acc, &b| acc.wrapping_add(b));
            
            // Write message to device registers
            self.register_cache[0x02] = 0x01; // Set message available flag
            self.register_cache[0x03] = test_data.len() as u8; // Message length
            self.register_cache[0x04] = 85; // Signal strength
            
            // Write message data
            for (i, &byte) in test_data.iter().enumerate() {
                self.register_cache[0x10 + i] = byte;
            }
            
            // Write checksum
            if self.config.enable_checksum {
                self.register_cache[0x10 + test_data.len()] = checksum;
            }
            
            self.last_read_time = Instant::now();
        }
    }
}

impl TransceiverInterface for I2CTransceiver {
    fn read_message(&mut self) -> CommResult<Option<RawMessage>> {
        if !self.status.connected {
            self.connect()?;
        }
        
        // Simulate message arrival for testing
        self.simulate_message_arrival();
        
        self.read_device_message()
    }
    
    fn send_message(&mut self, data: &[u8]) -> CommResult<()> {
        if !self.status.connected {
            return Err(CommError::ConnectionLost { 
                transceiver_id: self.id 
            });
        }
        
        if data.len() > self.config.max_message_size {
            return Err(CommError::BufferError {
                operation: "send".to_string(),
            });
        }
        
        // Write message length
        self.write_register(0x05, data.len() as u8)?;
        
        // Write message data to output buffer (starting at 0x20)
        self.write_bytes(0x20, data)?;
        
        // Calculate and write checksum if enabled
        if self.config.enable_checksum {
            let checksum = data.iter().fold(0u8, |acc, &b| acc.wrapping_add(b));
            self.write_register(0x20 + data.len() as u8, checksum)?;
        }
        
        // Trigger transmission
        self.write_register(0x06, 0x01)?;
        
        // Wait for transmission to complete (simulate)
        std::thread::sleep(Duration::from_millis(data.len() as u64 * 2));
        
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
        
        let new_address = config.i2c_address.ok_or_else(|| {
            CommError::ConfigurationError {
                parameter: "i2c_address".to_string(),
                value: "None".to_string(),
            }
        })?;
        
        // Disconnect if address changed
        let address_changed = new_address != self.i2c_address;
        let was_connected = self.status.connected;
        
        if address_changed && was_connected {
            self.disconnect();
        }
        
        self.config = config.clone();
        self.i2c_address = new_address;
        
        // Reconnect if we were connected before
        if was_connected {
            self.connect()?;
        }
        
        Ok(())
    }
    
    fn reset(&mut self) -> CommResult<()> {
        self.disconnect();
        self.message_buffer.clear();
        self.status.error_count = 0;
        self.status.messages_received = 0;
        self.status.last_message_time = None;
        
        // Reconnect
        self.connect()?;
        
        // Send reset command to device
        self.write_register(0x07, 0xFF)?;
        std::thread::sleep(Duration::from_millis(100)); // Wait for reset
        
        Ok(())
    }
    
    fn is_connected(&self) -> bool {
        self.status.connected
    }
    
    fn get_id(&self) -> u8 {
        self.id
    }
    
    fn flush(&mut self) -> CommResult<()> {
        if !self.status.connected {
            return Err(CommError::ConnectionLost { 
                transceiver_id: self.id 
            });
        }
        
        // Clear input buffer
        self.write_register(0x08, 0x01)?;
        
        // Clear output buffer
        self.write_register(0x09, 0x01)?;
        
        self.message_buffer.clear();
        
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_i2c_transceiver_creation() {
        let config = TransceiverConfig::i2c(1, 0x42);
        let transceiver = I2CTransceiver::new(1, config);
        assert!(transceiver.is_ok());
        
        let transceiver = transceiver.unwrap();
        assert_eq!(transceiver.get_id(), 1);
        assert_eq!(transceiver.i2c_address, 0x42);
        assert!(!transceiver.is_connected()); // Not connected initially
    }
    
    #[test]
    fn test_invalid_i2c_address() {
        let config = TransceiverConfig::i2c(1, 0xFF); // Invalid address
        let mut transceiver = I2CTransceiver::new(1, config).unwrap();
        
        let result = transceiver.connect();
        assert!(result.is_err());
        assert!(matches!(result, Err(CommError::ConfigurationError { .. })));
    }
    
    #[test]
    fn test_register_operations() {
        let config = TransceiverConfig::i2c(1, 0x42);
        let mut transceiver = I2CTransceiver::new(1, config).unwrap();
        transceiver.connect().unwrap();
        
        // Test register write/read
        transceiver.write_register(0x10, 0xAB).unwrap();
        let value = transceiver.read_register(0x10).unwrap();
        assert_eq!(value, 0xAB);
        
        // Test multi-byte operations
        let test_data = vec![1, 2, 3, 4, 5];
        transceiver.write_bytes(0x20, &test_data).unwrap();
        let read_data = transceiver.read_bytes(0x20, test_data.len()).unwrap();
        assert_eq!(read_data, test_data);
    }
    
    #[test]
    fn test_missing_i2c_address_config() {
        use crate::hardware::transceiver::InterfaceType;
        
        let mut config = TransceiverConfig::default();
        config.interface = InterfaceType::I2C;
        config.i2c_address = None; // Missing required address
        
        let result = I2CTransceiver::new(1, config);
        assert!(result.is_err());
        assert!(matches!(result, Err(CommError::ConfigurationError { .. })));
    }
}