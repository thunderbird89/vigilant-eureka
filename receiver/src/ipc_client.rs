use tokio::net::{TcpStream, tcp::{OwnedReadHalf, OwnedWriteHalf}};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::sync::mpsc;
use tokio::time::{timeout, Duration};
use std::net::SocketAddr;
use std::sync::Arc;
use tokio::sync::Mutex;
use uuid::Uuid;
use shared_positioning::{GeodeticPosition, VirtualMessage, IpcMessage};

/// Default port for connecting to the IPC server
pub const DEFAULT_IPC_PORT: u16 = 8765;

// Using shared IPC message types from shared-positioning crate

/// IPC client for connecting to beacon emulator
pub struct IpcClient {
    server_addr: SocketAddr,
    write_stream: Option<Arc<Mutex<OwnedWriteHalf>>>,
    message_rx: Option<mpsc::UnboundedReceiver<VirtualMessage>>,
    _message_task: Option<tokio::task::JoinHandle<()>>,
}

impl IpcClient {
    pub fn new(host: Option<&str>, port: Option<u16>) -> Self {
        let host = host.unwrap_or("127.0.0.1");
        let port = port.unwrap_or(DEFAULT_IPC_PORT);
        let server_addr = format!("{}:{}", host, port).parse().unwrap();
        
        Self {
            server_addr,
            write_stream: None,
            message_rx: None,
            _message_task: None,
        }
    }
    
    /// Connect to the IPC server
    pub async fn connect(&mut self) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        println!("Connecting to IPC server at {}", self.server_addr);
        
        // Connect with timeout
        let stream = timeout(Duration::from_secs(5), TcpStream::connect(self.server_addr))
            .await
            .map_err(|_| "Connection timeout")?
            .map_err(|e| format!("Connection failed: {}", e))?;
        
        println!("Connected to IPC server");
        
        // Split the stream for reading and writing
        let (mut read_stream, write_stream) = stream.into_split();
        let write_stream = Arc::new(Mutex::new(write_stream));
        
        // Set up message receiving
        let (message_tx, message_rx) = mpsc::unbounded_channel();
        self.message_rx = Some(message_rx);
        
        // Spawn task to handle incoming messages
        let message_task = tokio::spawn(async move {
            let mut buffer = vec![0u8; 8192];
            
            loop {
                match read_stream.read(&mut buffer).await {
                    Ok(0) => {
                        println!("IPC server disconnected");
                        break;
                    }
                    Ok(n) => {
                        println!("🔗 IPC: Received {} bytes from server", n);
                        match bincode::deserialize::<IpcMessage>(&buffer[..n]) {
                            Ok(IpcMessage::VirtualMessage(virtual_msg)) => {
                                println!("🔗 IPC: Decoded VirtualMessage from beacon {}", virtual_msg.beacon_id);
                                if let Err(_) = message_tx.send(virtual_msg) {
                                    // Receiver dropped, exit
                                    println!("🔗 IPC: Message receiver dropped, stopping message handler");
                                    break;
                                }
                            }
                            Ok(IpcMessage::Error { message }) => {
                                eprintln!("🔗 IPC server error: {}", message);
                            }
                            Ok(msg) => {
                                println!("🔗 IPC: Received message: {:?}", msg);
                            }
                            Err(e) => {
                                eprintln!("🔗 IPC: Failed to deserialize message: {}", e);
                                eprintln!("🔗 IPC: Raw data: {:02X?}", &buffer[..std::cmp::min(32, n)]);
                            }
                        }
                    }
                    Err(e) => {
                        eprintln!("Error reading from IPC server: {}", e);
                        break;
                    }
                }
            }
        });
        
        self._message_task = Some(message_task);
        self.write_stream = Some(write_stream);
        
        Ok(())
    }
    
    /// Subscribe to a virtual channel
    pub async fn subscribe(&mut self, channel_name: &str, receiver_id: u8) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        if let Some(write_stream) = &self.write_stream {
            let message = IpcMessage::Subscribe {
                channel_name: channel_name.to_string(),
                receiver_id,
            };
            
            let data = bincode::serialize(&message)?;
            let mut stream = write_stream.lock().await;
            stream.write_all(&data).await?;
            
            println!("Subscribed to channel '{}' with receiver ID {}", channel_name, receiver_id);
            Ok(())
        } else {
            Err("Not connected to IPC server".into())
        }
    }
    
    /// Unsubscribe from a virtual channel
    pub async fn unsubscribe(&mut self, channel_name: &str, receiver_id: u8) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        if let Some(write_stream) = &self.write_stream {
            let message = IpcMessage::Unsubscribe {
                channel_name: channel_name.to_string(),
                receiver_id,
            };
            
            let data = bincode::serialize(&message)?;
            let mut stream = write_stream.lock().await;
            stream.write_all(&data).await?;
            
            println!("Unsubscribed from channel '{}'", channel_name);
            Ok(())
        } else {
            Err("Not connected to IPC server".into())
        }
    }
    
    /// List available channels
    pub async fn list_channels(&mut self) -> Result<Vec<String>, Box<dyn std::error::Error + Send + Sync>> {
        if let Some(write_stream) = &self.write_stream {
            let message = IpcMessage::ListChannels;
            let data = bincode::serialize(&message)?;
            let mut stream = write_stream.lock().await;
            stream.write_all(&data).await?;
            
            // For now, return empty list - full implementation would wait for response
            Ok(Vec::new())
        } else {
            Err("Not connected to IPC server".into())
        }
    }
    
    /// Send heartbeat to keep connection alive
    pub async fn heartbeat(&mut self) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
        if let Some(write_stream) = &self.write_stream {
            let message = IpcMessage::Heartbeat;
            let data = bincode::serialize(&message)?;
            let mut stream = write_stream.lock().await;
            stream.write_all(&data).await?;
            Ok(())
        } else {
            Err("Not connected to IPC server".into())
        }
    }
    
    /// Try to receive a virtual message (non-blocking)
    pub fn try_recv_message(&mut self) -> Option<VirtualMessage> {
        if let Some(rx) = &mut self.message_rx {
            rx.try_recv().ok()
        } else {
            None
        }
    }
    
    /// Receive a virtual message (blocking)
    pub async fn recv_message(&mut self) -> Option<VirtualMessage> {
        if let Some(rx) = &mut self.message_rx {
            rx.recv().await
        } else {
            None
        }
    }
    
    /// Check if connected to server
    pub fn is_connected(&self) -> bool {
        self.write_stream.is_some()
    }
    
    /// Disconnect from server
    pub async fn disconnect(&mut self) {
        if let Some(write_stream) = self.write_stream.take() {
            // Try to shutdown the write stream gracefully
            if let Ok(mut stream) = write_stream.try_lock() {
                let _ = stream.shutdown().await;
            }
        }
        
        if let Some(task) = self._message_task.take() {
            task.abort();
        }
        
        self.message_rx = None;
        println!("Disconnected from IPC server");
    }
    
    /// Get server address
    pub fn server_addr(&self) -> SocketAddr {
        self.server_addr
    }
}

impl Drop for IpcClient {
    fn drop(&mut self) {
        if let Some(task) = self._message_task.take() {
            task.abort();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio::time::sleep;

    #[tokio::test]
    async fn test_ipc_client_creation() {
        let client = IpcClient::new(Some("127.0.0.1"), Some(8768));
        assert_eq!(client.server_addr().port(), 8768);
        assert!(!client.is_connected());
    }

    #[tokio::test]
    async fn test_message_serialization() {
        let message = IpcMessage::Subscribe {
            channel_name: "test_channel".to_string(),
            receiver_id: 1,
        };
        
        let serialized = bincode::serialize(&message).unwrap();
        let deserialized: IpcMessage = bincode::deserialize(&serialized).unwrap();
        
        match deserialized {
            IpcMessage::Subscribe { channel_name, receiver_id } => {
                assert_eq!(channel_name, "test_channel");
                assert_eq!(receiver_id, 1);
            }
            _ => panic!("Wrong message type"),
        }
    }

    #[tokio::test]
    async fn test_virtual_message_serialization() {
        let virtual_msg = VirtualMessage {
            beacon_id: Uuid::new_v4(),
            timestamp: std::time::SystemTime::now(),
            position: GeodeticPosition {
                latitude: 32.123,
                longitude: 45.476,
                depth: 10.0,
            },
            message_data: vec![0x01, 0x02, 0x03],
            signal_quality: 255,
        };
        
        let ipc_msg = IpcMessage::VirtualMessage(virtual_msg.clone());
        let serialized = bincode::serialize(&ipc_msg).unwrap();
        let deserialized: IpcMessage = bincode::deserialize(&serialized).unwrap();
        
        match deserialized {
            IpcMessage::VirtualMessage(msg) => {
                assert_eq!(msg.beacon_id, virtual_msg.beacon_id);
                assert_eq!(msg.message_data, virtual_msg.message_data);
                assert_eq!(msg.signal_quality, virtual_msg.signal_quality);
            }
            _ => panic!("Wrong message type"),
        }
    }

    #[tokio::test]
    async fn test_client_connection_failure() {
        let mut client = IpcClient::new(Some("127.0.0.1"), Some(9999)); // Non-existent server
        let result = client.connect().await;
        assert!(result.is_err());
    }
}