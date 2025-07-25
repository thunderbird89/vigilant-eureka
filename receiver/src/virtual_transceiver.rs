use shared_positioning::{
    TransceiverInterface, CommError, TransceiverStatus, TransceiverConfig, 
    PowerMode, TransmissionStatus, RawMessage, BaseTransceiver, GeodeticPosition
};
use tokio::sync::broadcast;
use std::time::{SystemTime, UNIX_EPOCH, Duration, Instant};
use uuid::Uuid;
use serde::{Serialize, Deserialize, Serializer, Deserializer};

/// Virtual transceiver that connects to beacon emulator's virtual communication space
pub struct VirtualTransceiver {
    base: BaseTransceiver,
    channel_name: String,
    receiver: Option<broadcast::Receiver<VirtualMessage>>,
    is_connected: bool,
}

/// Message format used in virtual communication space (copied from beacon-emulator)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VirtualMessage {
    pub beacon_id: Uuid,
    #[serde(
        serialize_with = "serialize_system_time",
        deserialize_with = "deserialize_system_time"
    )]
    pub timestamp: SystemTime,
    pub position: GeodeticPosition,
    pub message_data: Vec<u8>,
    pub signal_quality: u8,
}

// Custom serialization for SystemTime
fn serialize_system_time<S>(
    time: &SystemTime,
    serializer: S,
) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    let duration = time.duration_since(UNIX_EPOCH)
        .map_err(serde::ser::Error::custom)?;
    serializer.serialize_u64(duration.as_secs())
}

fn deserialize_system_time<'de, D>(
    deserializer: D,
) -> Result<SystemTime, D::Error>
where
    D: Deserializer<'de>,
{
    let secs: u64 = u64::deserialize(deserializer)?;
    Ok(UNIX_EPOCH + Duration::from_secs(secs))
}

impl VirtualTransceiver {
    pub fn new(id: u8, channel_name: String) -> Self {
        Self {
            base: BaseTransceiver::new(id),
            channel_name,
            receiver: None,
            is_connected: false,
        }
    }
    
    /// Connect to the virtual communication channel
    pub async fn connect(&mut self) -> Result<(), CommError> {
        if self.is_connected {
            return Ok(());
        }
        
        // For now, we'll use a simple file-based communication mechanism
        // This can be extended later to use proper IPC mechanisms like named pipes,
        // shared memory, or network sockets
        
        // Try to connect to the virtual channel by checking for a channel file
        let channel_file = format!("/tmp/beacon_emulator_channel_{}", self.channel_name);
        
        // For this implementation, we'll create a mock receiver that can be extended
        // to read from the actual virtual channel
        let (sender, receiver) = broadcast::channel(1000);
        self.receiver = Some(receiver);
        self.is_connected = true;
        
        self.base.update_status(true, Some(200));
        
        println!("Virtual transceiver {} connected to channel '{}' (mock mode)", self.base.id, self.channel_name);
        println!("Note: This is a mock implementation. In a full implementation, this would");
        println!("      connect to the beacon emulator's virtual communication space.");
        
        Ok(())
    }
    
    /// Disconnect from the virtual communication channel
    pub fn disconnect(&mut self) {
        self.receiver = None;
        self.is_connected = false;
        self.base.update_status(false, None);
        
        println!("Virtual transceiver {} disconnected from channel '{}'", self.base.id, self.channel_name);
    }
    
    /// Get the channel name this transceiver is connected to
    pub fn channel_name(&self) -> &str {
        &self.channel_name
    }
    
    /// Convert virtual message to raw message format expected by receiver
    fn convert_virtual_to_raw(&mut self, virtual_msg: VirtualMessage) -> RawMessage {
        let timestamp_ms = virtual_msg.timestamp
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis() as u64;
        
        self.base.stats.record_message_received(virtual_msg.message_data.len());
        self.base.status.last_message_time = Some(Instant::now());
        
        RawMessage {
            data: virtual_msg.message_data,
            timestamp_received: timestamp_ms,
            transceiver_id: self.base.id,
            signal_strength: Some(virtual_msg.signal_quality),
        }
    }
}

impl TransceiverInterface for VirtualTransceiver {
    fn read_message(&mut self) -> Result<Option<RawMessage>, CommError> {
        if !self.is_connected {
            return Err(CommError::NotConnected);
        }
        
        if let Some(receiver) = &mut self.receiver {
            // Try to receive a message without blocking
            match receiver.try_recv() {
                Ok(virtual_msg) => {
                    let raw_msg = self.convert_virtual_to_raw(virtual_msg);
                    Ok(Some(raw_msg))
                }
                Err(broadcast::error::TryRecvError::Empty) => {
                    // No messages available
                    Ok(None)
                }
                Err(broadcast::error::TryRecvError::Lagged(skipped)) => {
                    // Receiver lagged behind, some messages were dropped
                    eprintln!("Warning: Virtual transceiver {} lagged, {} messages skipped", 
                             self.base.id, skipped);
                    Ok(None)
                }
                Err(broadcast::error::TryRecvError::Closed) => {
                    // Channel closed
                    self.disconnect();
                    Err(CommError::ConnectionFailed("Virtual channel closed".to_string()))
                }
            }
        } else {
            Err(CommError::NotConnected)
        }
    }
    
    fn transmit_message(&mut self, _data: &[u8]) -> Result<(), CommError> {
        // Virtual receivers are receive-only for now
        // In the future, this could be extended to support bidirectional communication
        Err(CommError::UnsupportedOperation {
            operation: "Virtual receiver transmission not supported".to_string(),
        })
    }
    
    fn set_transmission_power(&mut self, _power_level: u8) -> Result<(), CommError> {
        // Not applicable for receive-only virtual transceiver
        Err(CommError::UnsupportedOperation {
            operation: "Power control not supported for virtual receiver".to_string(),
        })
    }
    
    fn get_transmission_status(&self) -> TransmissionStatus {
        // Virtual receivers don't transmit
        TransmissionStatus::default()
    }
    
    fn get_status(&self) -> TransceiverStatus {
        let mut status = self.base.status.clone();
        if self.is_connected {
            status.firmware_version = Some("VirtualTransceiver v1.0".to_string());
            status.hardware_id = Some(0x5649); // "VI" in hex
            status.battery_level = Some(100); // Virtual transceivers don't use battery
            status.temperature = Some(2000); // 20.0°C
        }
        status
    }
    
    fn configure(&mut self, config: TransceiverConfig) -> Result<(), CommError> {
        // Validate configuration parameters
        if config.buffer_size < 64 {
            return Err(CommError::ConfigurationError(
                "Buffer size too small (minimum 64 bytes)".to_string()
            ));
        }
        
        self.base.config = config;
        Ok(())
    }
    
    fn send_command(&mut self, command: &[u8]) -> Result<Vec<u8>, CommError> {
        if !self.is_connected {
            return Err(CommError::NotConnected);
        }
        
        // Mock command responses for virtual transceiver
        match command.get(0) {
            Some(0x01) => Ok(b"VIRTUAL_OK\r\n".to_vec()),
            Some(0x02) => Ok(format!("VIRTUAL_STATUS:CONNECTED:{}\r\n", self.channel_name).into_bytes()),
            Some(0x03) => Ok(b"VIRTUAL_VERSION:1.0\r\n".to_vec()),
            Some(0x04) => Ok(b"VIRTUAL_SIGNAL:200\r\n".to_vec()),
            Some(0x05) => Ok(format!("VIRTUAL_CHANNEL:{}\r\n", self.channel_name).into_bytes()),
            _ => Err(CommError::UnsupportedOperation {
                operation: format!("Virtual command 0x{:02X}", command.get(0).unwrap_or(&0)),
            }),
        }
    }
    
    fn is_connected(&self) -> bool {
        self.is_connected
    }
    
    fn reset(&mut self) -> Result<(), CommError> {
        self.disconnect();
        
        // Simulate reset delay
        std::thread::sleep(Duration::from_millis(100));
        
        // Reconnect after reset
        tokio::task::block_in_place(|| {
            tokio::runtime::Handle::current().block_on(async {
                self.connect().await
            })
        })
    }
    
    fn get_id(&self) -> u8 {
        self.base.id
    }
    
    fn flush_buffers(&mut self) -> Result<(), CommError> {
        if let Some(receiver) = &mut self.receiver {
            // Drain all pending messages
            while let Ok(_) = receiver.try_recv() {
                // Discard messages
            }
        }
        Ok(())
    }
    
    fn set_power_mode(&mut self, mode: PowerMode) -> Result<(), CommError> {
        self.base.config.power_mode = mode;
        
        // Virtual transceivers don't have real power management,
        // but we can simulate different response characteristics
        match mode {
            PowerMode::Normal => {
                // Normal operation
            }
            PowerMode::PowerSave => {
                // Could simulate reduced polling frequency
            }
            PowerMode::Sleep => {
                // Could simulate periodic wake-up
            }
            PowerMode::Emergency => {
                // Could simulate minimal operation
            }
        }
        
        Ok(())
    }
}

/// Factory for creating virtual transceivers
pub struct VirtualTransceiverFactory;

impl VirtualTransceiverFactory {
    /// Create a new virtual transceiver for the specified channel
    pub fn create_transceiver(id: u8, channel_name: String) -> VirtualTransceiver {
        VirtualTransceiver::new(id, channel_name)
    }
    
    /// Create and connect a virtual transceiver
    pub async fn create_and_connect(id: u8, channel_name: String) -> Result<VirtualTransceiver, CommError> {
        let mut transceiver = Self::create_transceiver(id, channel_name);
        transceiver.connect().await?;
        Ok(transceiver)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio::time::{sleep, Duration as TokioDuration};

    #[tokio::test]
    async fn test_virtual_transceiver_creation() {
        let transceiver = VirtualTransceiver::new(1, "test_channel".to_string());
        assert_eq!(transceiver.channel_name(), "test_channel");
        assert_eq!(transceiver.get_id(), 1);
        assert!(!transceiver.is_connected());
    }

    #[tokio::test]
    async fn test_virtual_transceiver_connection() {
        let mut transceiver = VirtualTransceiver::new(1, "test_channel".to_string());
        
        // Initially not connected
        assert!(!transceiver.is_connected());
        
        // Connect
        transceiver.connect().await.unwrap();
        assert!(transceiver.is_connected());
        
        // Disconnect
        transceiver.disconnect();
        assert!(!transceiver.is_connected());
    }

    #[tokio::test]
    async fn test_virtual_transceiver_configuration() {
        let mut transceiver = VirtualTransceiver::new(1, "test_channel".to_string());
        
        let config = TransceiverConfig {
            buffer_size: 1024,
            timeout_ms: 5000,
            ..Default::default()
        };
        
        transceiver.configure(config.clone()).unwrap();
        assert_eq!(transceiver.base.config.buffer_size, 1024);
        assert_eq!(transceiver.base.config.timeout_ms, 5000);
    }

    #[tokio::test]
    async fn test_virtual_transceiver_commands() {
        let mut transceiver = VirtualTransceiver::new(1, "test_channel".to_string());
        transceiver.connect().await.unwrap();
        
        // Test status command
        let response = transceiver.send_command(&[0x02]).unwrap();
        let response_str = String::from_utf8(response).unwrap();
        assert!(response_str.contains("VIRTUAL_STATUS:CONNECTED:test_channel"));
        
        // Test version command
        let response = transceiver.send_command(&[0x03]).unwrap();
        let response_str = String::from_utf8(response).unwrap();
        assert!(response_str.contains("VIRTUAL_VERSION:1.0"));
        
        // Test channel command
        let response = transceiver.send_command(&[0x05]).unwrap();
        let response_str = String::from_utf8(response).unwrap();
        assert!(response_str.contains("VIRTUAL_CHANNEL:test_channel"));
    }

    #[tokio::test]
    async fn test_virtual_transceiver_unsupported_operations() {
        let mut transceiver = VirtualTransceiver::new(1, "test_channel".to_string());
        transceiver.connect().await.unwrap();
        
        // Test unsupported transmission
        let result = transceiver.transmit_message(b"test");
        assert!(matches!(result, Err(CommError::UnsupportedOperation { .. })));
        
        // Test unsupported power control
        let result = transceiver.set_transmission_power(128);
        assert!(matches!(result, Err(CommError::UnsupportedOperation { .. })));
    }

    #[tokio::test]
    async fn test_virtual_transceiver_factory() {
        let transceiver = VirtualTransceiverFactory::create_transceiver(2, "factory_test".to_string());
        assert_eq!(transceiver.get_id(), 2);
        assert_eq!(transceiver.channel_name(), "factory_test");
        
        let mut connected_transceiver = VirtualTransceiverFactory::create_and_connect(3, "connected_test".to_string()).await.unwrap();
        assert!(connected_transceiver.is_connected());
        assert_eq!(connected_transceiver.get_id(), 3);
    }

    #[tokio::test]
    async fn test_virtual_transceiver_status() {
        let mut transceiver = VirtualTransceiver::new(1, "test_channel".to_string());
        
        // Status when disconnected
        let status = transceiver.get_status();
        assert!(!status.is_connected);
        assert!(status.firmware_version.is_none());
        
        // Status when connected
        transceiver.connect().await.unwrap();
        let status = transceiver.get_status();
        assert!(status.is_connected);
        assert_eq!(status.firmware_version, Some("VirtualTransceiver v1.0".to_string()));
        assert_eq!(status.hardware_id, Some(0x5649)); // "VI" in hex
        assert_eq!(status.battery_level, Some(100));
    }

    #[tokio::test]
    async fn test_virtual_transceiver_power_modes() {
        let mut transceiver = VirtualTransceiver::new(1, "test_channel".to_string());
        transceiver.connect().await.unwrap();
        
        // Test different power modes
        transceiver.set_power_mode(PowerMode::Normal).unwrap();
        assert_eq!(transceiver.base.config.power_mode, PowerMode::Normal);
        
        transceiver.set_power_mode(PowerMode::PowerSave).unwrap();
        assert_eq!(transceiver.base.config.power_mode, PowerMode::PowerSave);
        
        transceiver.set_power_mode(PowerMode::Sleep).unwrap();
        assert_eq!(transceiver.base.config.power_mode, PowerMode::Sleep);
        
        transceiver.set_power_mode(PowerMode::Emergency).unwrap();
        assert_eq!(transceiver.base.config.power_mode, PowerMode::Emergency);
    }

    #[tokio::test]
    async fn test_virtual_transceiver_reset() {
        let mut transceiver = VirtualTransceiver::new(1, "test_channel".to_string());
        transceiver.connect().await.unwrap();
        assert!(transceiver.is_connected());
        
        // Reset should disconnect and reconnect
        transceiver.reset().unwrap();
        assert!(transceiver.is_connected());
    }

    #[tokio::test]
    async fn test_virtual_transceiver_buffer_flush() {
        let mut transceiver = VirtualTransceiver::new(1, "test_channel".to_string());
        transceiver.connect().await.unwrap();
        
        // Flush should succeed
        transceiver.flush_buffers().unwrap();
    }
}