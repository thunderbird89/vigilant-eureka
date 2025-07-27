use tokio::net::{TcpListener, TcpStream};
use tokio::sync::{broadcast, RwLock};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use std::collections::HashMap;
use std::sync::Arc;
use std::net::SocketAddr;
use uuid::Uuid;
use crate::virtual_channel::VirtualCommunicationSpace;
use crate::EmulatorError;
use shared_positioning::{VirtualMessage, IpcMessage};

/// Default port for the IPC server
pub const DEFAULT_IPC_PORT: u16 = 8765;

// Using shared IPC message types from shared-positioning crate

/// Client connection information
#[derive(Debug, Clone)]
struct ClientInfo {
    id: u8,
    address: SocketAddr,
    subscribed_channels: Vec<String>,
}

/// Cross-platform IPC server for virtual communication
pub struct IpcServer {
    port: u16,
    communication_space: Arc<RwLock<VirtualCommunicationSpace>>,
    clients: Arc<RwLock<HashMap<SocketAddr, ClientInfo>>>,
    shutdown_tx: Option<broadcast::Sender<()>>,
}

impl IpcServer {
    pub fn new(port: Option<u16>) -> Self {
        Self {
            port: port.unwrap_or(DEFAULT_IPC_PORT),
            communication_space: Arc::new(RwLock::new(VirtualCommunicationSpace::new())),
            clients: Arc::new(RwLock::new(HashMap::new())),
            shutdown_tx: None,
        }
    }
    
    /// Create a new IPC server with a shared communication space
    pub fn new_with_shared_communication_space(
        port: Option<u16>, 
        communication_space: Arc<RwLock<VirtualCommunicationSpace>>
    ) -> Self {
        Self {
            port: port.unwrap_or(DEFAULT_IPC_PORT),
            communication_space,
            clients: Arc::new(RwLock::new(HashMap::new())),
            shutdown_tx: None,
        }
    }
    
    /// Start the IPC server
    pub async fn start(&mut self) -> Result<(), EmulatorError> {
        let addr = format!("127.0.0.1:{}", self.port);
        let listener = TcpListener::bind(&addr).await
            .map_err(|e| EmulatorError::ConfigError(format!("Failed to bind to {}: {}", addr, e)))?;
        
        println!("IPC server listening on {}", addr);
        
        let (shutdown_tx, _) = broadcast::channel(1);
        self.shutdown_tx = Some(shutdown_tx.clone());
        
        let communication_space = self.communication_space.clone();
        let clients = self.clients.clone();
        
        tokio::spawn(async move {
            let mut shutdown_rx = shutdown_tx.subscribe();
            
            loop {
                tokio::select! {
                    result = listener.accept() => {
                        match result {
                            Ok((stream, addr)) => {
                                println!("New client connected: {}", addr);
                                
                                let communication_space = communication_space.clone();
                                let clients = clients.clone();
                                let shutdown_rx = shutdown_tx.subscribe();
                                
                                tokio::spawn(async move {
                                    if let Err(e) = Self::handle_client(stream, addr, communication_space, clients, shutdown_rx).await {
                                        eprintln!("Client {} error: {}", addr, e);
                                    }
                                });
                            }
                            Err(e) => {
                                eprintln!("Failed to accept connection: {}", e);
                            }
                        }
                    }
                    _ = shutdown_rx.recv() => {
                        println!("IPC server shutting down");
                        break;
                    }
                }
            }
        });
        
        Ok(())
    }
    
    /// Handle individual client connection
    async fn handle_client(
        mut stream: TcpStream,
        addr: SocketAddr,
        communication_space: Arc<RwLock<VirtualCommunicationSpace>>,
        clients: Arc<RwLock<HashMap<SocketAddr, ClientInfo>>>,
        mut shutdown_rx: broadcast::Receiver<()>,
    ) -> Result<(), EmulatorError> {
        let mut buffer = vec![0u8; 8192];
        let mut subscriptions: HashMap<String, broadcast::Receiver<VirtualMessage>> = HashMap::new();
        
        loop {
            tokio::select! {
                // Handle incoming messages from client
                result = stream.read(&mut buffer) => {
                    match result {
                        Ok(0) => {
                            // Client disconnected
                            println!("Client {} disconnected", addr);
                            break;
                        }
                        Ok(n) => {
                            // Process message
                            match bincode::deserialize::<IpcMessage>(&buffer[..n]) {
                                Ok(message) => {
                                    if let Err(e) = Self::process_client_message(
                                        &mut stream,
                                        addr,
                                        message,
                                        &communication_space,
                                        &clients,
                                        &mut subscriptions,
                                    ).await {
                                        eprintln!("Error processing message from {}: {}", addr, e);
                                    }
                                }
                                Err(e) => {
                                    eprintln!("Failed to deserialize message from {}: {}", addr, e);
                                    let error_msg = IpcMessage::Error {
                                        message: format!("Invalid message format: {}", e),
                                    };
                                    if let Ok(data) = bincode::serialize(&error_msg) {
                                        let _ = stream.write_all(&data).await;
                                    }
                                }
                            }
                        }
                        Err(e) => {
                            eprintln!("Error reading from client {}: {}", addr, e);
                            break;
                        }
                    }
                }
                
                // Handle virtual messages from subscribed channels
                message = Self::receive_from_subscriptions(&mut subscriptions) => {
                    if let Some(virtual_msg) = message {
                        let ipc_msg = IpcMessage::VirtualMessage(virtual_msg);
                        if let Ok(data) = bincode::serialize(&ipc_msg) {
                            if let Err(e) = stream.write_all(&data).await {
                                eprintln!("Error sending message to client {}: {}", addr, e);
                                break;
                            }
                        }
                    }
                }
                
                // Handle shutdown signal
                _ = shutdown_rx.recv() => {
                    println!("Client handler for {} shutting down", addr);
                    break;
                }
            }
        }
        
        // Clean up client info
        {
            let mut clients_guard = clients.write().await;
            clients_guard.remove(&addr);
        }
        
        Ok(())
    }
    
    /// Process a message from a client
    async fn process_client_message(
        stream: &mut TcpStream,
        addr: SocketAddr,
        message: IpcMessage,
        communication_space: &Arc<RwLock<VirtualCommunicationSpace>>,
        clients: &Arc<RwLock<HashMap<SocketAddr, ClientInfo>>>,
        subscriptions: &mut HashMap<String, broadcast::Receiver<VirtualMessage>>,
    ) -> Result<(), EmulatorError> {
        match message {
            IpcMessage::Subscribe { channel_name, receiver_id } => {
                println!("Client {} subscribing to channel '{}' with ID {}", addr, channel_name, receiver_id);
                
                // Get or create the channel
                let receiver = {
                    let mut space = communication_space.write().await;
                    let channel = space.get_or_create_channel(&channel_name);
                    channel.subscribe()
                };
                
                // Store subscription
                subscriptions.insert(channel_name.clone(), receiver);
                
                // Update client info
                {
                    let mut clients_guard = clients.write().await;
                    let client_info = clients_guard.entry(addr).or_insert_with(|| ClientInfo {
                        id: receiver_id,
                        address: addr,
                        subscribed_channels: Vec::new(),
                    });
                    client_info.subscribed_channels.push(channel_name);
                }
                
                // Send acknowledgment
                let ack = IpcMessage::Ack;
                let data = bincode::serialize(&ack)
                    .map_err(|e| EmulatorError::ConfigError(format!("Serialization error: {}", e)))?;
                stream.write_all(&data).await
                    .map_err(|e| EmulatorError::IoError(e))?;
            }
            
            IpcMessage::Unsubscribe { channel_name, receiver_id: _ } => {
                println!("Client {} unsubscribing from channel '{}'", addr, channel_name);
                
                // Remove subscription
                subscriptions.remove(&channel_name);
                
                // Update client info
                {
                    let mut clients_guard = clients.write().await;
                    if let Some(client_info) = clients_guard.get_mut(&addr) {
                        client_info.subscribed_channels.retain(|ch| ch != &channel_name);
                    }
                }
                
                // Send acknowledgment
                let ack = IpcMessage::Ack;
                let data = bincode::serialize(&ack)
                    .map_err(|e| EmulatorError::ConfigError(format!("Serialization error: {}", e)))?;
                stream.write_all(&data).await
                    .map_err(|e| EmulatorError::IoError(e))?;
            }
            
            IpcMessage::ListChannels => {
                let channels = {
                    let space = communication_space.read().await;
                    space.list_channels()
                };
                
                let response = IpcMessage::ChannelList { channels };
                let data = bincode::serialize(&response)
                    .map_err(|e| EmulatorError::ConfigError(format!("Serialization error: {}", e)))?;
                stream.write_all(&data).await
                    .map_err(|e| EmulatorError::IoError(e))?;
            }
            
            IpcMessage::Heartbeat => {
                // Respond to heartbeat
                let ack = IpcMessage::Ack;
                let data = bincode::serialize(&ack)
                    .map_err(|e| EmulatorError::ConfigError(format!("Serialization error: {}", e)))?;
                stream.write_all(&data).await
                    .map_err(|e| EmulatorError::IoError(e))?;
            }
            
            _ => {
                // Unexpected message from client
                let error_msg = IpcMessage::Error {
                    message: "Unexpected message type from client".to_string(),
                };
                let data = bincode::serialize(&error_msg)
                    .map_err(|e| EmulatorError::ConfigError(format!("Serialization error: {}", e)))?;
                stream.write_all(&data).await
                    .map_err(|e| EmulatorError::IoError(e))?;
            }
        }
        
        Ok(())
    }
    
    /// Receive messages from any subscribed channel
    async fn receive_from_subscriptions(
        subscriptions: &mut HashMap<String, broadcast::Receiver<VirtualMessage>>,
    ) -> Option<VirtualMessage> {
        if subscriptions.is_empty() {
            // No subscriptions, wait indefinitely
            tokio::time::sleep(tokio::time::Duration::from_secs(3600)).await;
            return None;
        }
        
        // Create a vector of channel names for reference
        let channel_names: Vec<String> = subscriptions.keys().cloned().collect();
        
        // Use tokio::select! to wait for messages from any channel
        // We'll handle up to 10 channels (can be extended if needed)
        match channel_names.len() {
            1 => {
                let channel_name = &channel_names[0];
                if let Some(receiver) = subscriptions.get_mut(channel_name) {
                    match receiver.recv().await {
                        Ok(message) => {
                            println!("🔗 IPC: Received message from channel '{}' for beacon {} (forwarding to clients)", 
                                     channel_name, message.beacon_id);
                            Some(message)
                        }
                        Err(broadcast::error::RecvError::Lagged(skipped)) => {
                            eprintln!("Warning: Receiver lagged on channel '{}', {} messages skipped", 
                                      channel_name, skipped);
                            None
                        }
                        Err(broadcast::error::RecvError::Closed) => {
                            eprintln!("Channel '{}' closed", channel_name);
                            subscriptions.remove(channel_name);
                            None
                        }
                    }
                } else {
                    None
                }
            }
            _ => {
                // For multiple channels, just try the first one for now
                // This is a simplified implementation - in a full implementation,
                // we'd use a proper select mechanism
                let channel_name = &channel_names[0];
                if let Some(receiver) = subscriptions.get_mut(channel_name) {
                    match receiver.try_recv() {
                        Ok(message) => {
                            println!("🔗 IPC: Received message from channel '{}' for beacon {}", 
                                     channel_name, message.beacon_id);
                            Some(message)
                        }
                        Err(broadcast::error::TryRecvError::Empty) => {
                            // No message available, wait a bit
                            tokio::time::sleep(tokio::time::Duration::from_millis(10)).await;
                            None
                        }
                        Err(broadcast::error::TryRecvError::Lagged(skipped)) => {
                            eprintln!("Warning: Receiver lagged on channel '{}', {} messages skipped", 
                                      channel_name, skipped);
                            None
                        }
                        Err(broadcast::error::TryRecvError::Closed) => {
                            eprintln!("Channel '{}' closed", channel_name);
                            subscriptions.remove(channel_name);
                            None
                        }
                    }
                } else {
                    None
                }
            }
        }
    }
    
    /// Get the communication space for broadcasting messages
    pub fn get_communication_space(&self) -> Arc<RwLock<VirtualCommunicationSpace>> {
        self.communication_space.clone()
    }
    
    /// Get connected clients information
    pub async fn get_clients(&self) -> Vec<ClientInfo> {
        let clients_guard = self.clients.read().await;
        clients_guard.values().cloned().collect()
    }
    
    /// Shutdown the server
    pub async fn shutdown(&self) -> Result<(), EmulatorError> {
        if let Some(shutdown_tx) = &self.shutdown_tx {
            let _ = shutdown_tx.send(());
        }
        Ok(())
    }
    
    /// Get server port
    pub fn port(&self) -> u16 {
        self.port
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tokio::time::{sleep, Duration};
    use shared_positioning::GeodeticPosition;

    #[tokio::test]
    async fn test_ipc_server_creation() {
        let server = IpcServer::new(Some(8766));
        assert_eq!(server.port(), 8766);
    }

    #[tokio::test]
    async fn test_ipc_server_start() {
        let mut server = IpcServer::new(Some(8767));
        
        // Start server
        server.start().await.unwrap();
        
        // Give it a moment to start
        sleep(Duration::from_millis(100)).await;
        
        // Try to connect
        let result = TcpStream::connect("127.0.0.1:8767").await;
        assert!(result.is_ok());
        
        // Shutdown
        server.shutdown().await.unwrap();
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
}