use tokio::sync::{broadcast, Mutex};
use std::collections::HashMap;
use std::sync::Arc;
use uuid::Uuid;
use serde::{Serialize, Deserialize};
use shared_positioning::config::GeodeticPosition;
use crate::EmulatorError;

/// Virtual communication space that manages multiple channels
pub struct VirtualCommunicationSpace {
    channels: HashMap<String, VirtualChannel>,
}

impl VirtualCommunicationSpace {
    pub fn new() -> Self {
        Self {
            channels: HashMap::new(),
        }
    }
    
    pub fn get_or_create_channel(&mut self, name: &str) -> VirtualChannel {
        self.channels.entry(name.to_string())
            .or_insert_with(|| VirtualChannel::new(name))
            .clone()
    }
    
    pub fn list_channels(&self) -> Vec<String> {
        self.channels.keys().cloned().collect()
    }
}

/// Virtual channel for message broadcasting
#[derive(Clone)]
pub struct VirtualChannel {
    name: String,
    sender: broadcast::Sender<VirtualMessage>,
    message_log: Arc<Mutex<Vec<VirtualMessage>>>,
}

impl VirtualChannel {
    pub fn new(name: &str) -> Self {
        let (sender, _) = broadcast::channel(1000); // Buffer up to 1000 messages
        
        Self {
            name: name.to_string(),
            sender,
            message_log: Arc::new(Mutex::new(Vec::new())),
        }
    }
    
    pub async fn broadcast_message(&self, message: VirtualMessage) -> Result<(), EmulatorError> {
        // Log message
        {
            let mut log = self.message_log.lock().await;
            log.push(message.clone());
            
            // Keep only recent messages (last 10000)
            if log.len() > 10000 {
                log.drain(0..1000);
            }
        }
        
        // Broadcast to subscribers
        self.sender.send(message)
            .map_err(|_| EmulatorError::ChannelError("No receivers".to_string()))?;
        
        Ok(())
    }
    
    pub fn subscribe(&self) -> broadcast::Receiver<VirtualMessage> {
        self.sender.subscribe()
    }
    
    pub async fn get_recent_messages(&self, count: usize) -> Vec<VirtualMessage> {
        let log = self.message_log.lock().await;
        log.iter().rev().take(count).cloned().collect()
    }
    
    pub async fn get_messages_since(&self, since: std::time::SystemTime) -> Vec<VirtualMessage> {
        let log = self.message_log.lock().await;
        log.iter()
            .filter(|msg| msg.timestamp >= since)
            .cloned()
            .collect()
    }
    
    pub fn name(&self) -> &str {
        &self.name
    }
}

/// Message transmitted in virtual communication space
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VirtualMessage {
    pub beacon_id: Uuid,
    pub timestamp: std::time::SystemTime,
    pub position: GeodeticPosition,
    pub message_data: Vec<u8>,
    pub signal_quality: u8,
}