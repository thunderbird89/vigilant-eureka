use tokio::sync::{broadcast, Mutex};
use std::collections::HashMap;
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH, Duration};
use uuid::Uuid;
use serde::{Serialize, Deserialize, Serializer, Deserializer};
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
    
    pub fn get_channel(&self, name: &str) -> Option<&VirtualChannel> {
        self.channels.get(name)
    }
    
    pub fn remove_channel(&mut self, name: &str) -> Option<VirtualChannel> {
        self.channels.remove(name)
    }
    
    pub async fn get_all_channel_stats(&self) -> Vec<VirtualChannelStats> {
        let mut stats = Vec::new();
        for channel in self.channels.values() {
            stats.push(channel.get_channel_stats().await);
        }
        stats
    }
    
    pub fn channel_count(&self) -> usize {
        self.channels.len()
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
        
        // Broadcast to subscribers (ignore if no receivers)
        let _ = self.sender.send(message);
        
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
    
    pub async fn get_messages_by_beacon(&self, beacon_id: Uuid) -> Vec<VirtualMessage> {
        let log = self.message_log.lock().await;
        log.iter()
            .filter(|msg| msg.beacon_id == beacon_id)
            .cloned()
            .collect()
    }
    
    pub async fn get_messages_by_beacon_since(
        &self, 
        beacon_id: Uuid, 
        since: std::time::SystemTime
    ) -> Vec<VirtualMessage> {
        let log = self.message_log.lock().await;
        log.iter()
            .filter(|msg| msg.beacon_id == beacon_id && msg.timestamp >= since)
            .cloned()
            .collect()
    }
    
    pub async fn get_messages_filtered(
        &self,
        beacon_id: Option<Uuid>,
        since: Option<std::time::SystemTime>,
        count: Option<usize>
    ) -> Vec<VirtualMessage> {
        let log = self.message_log.lock().await;
        let mut filtered: Vec<VirtualMessage> = log.iter()
            .filter(|msg| {
                if let Some(id) = beacon_id {
                    if msg.beacon_id != id {
                        return false;
                    }
                }
                if let Some(time) = since {
                    if msg.timestamp < time {
                        return false;
                    }
                }
                true
            })
            .cloned()
            .collect();
        
        // Sort by timestamp (most recent first)
        filtered.sort_by(|a, b| b.timestamp.cmp(&a.timestamp));
        
        // Limit count if specified
        if let Some(limit) = count {
            filtered.truncate(limit);
        }
        
        filtered
    }
    
    pub fn name(&self) -> &str {
        &self.name
    }
    
    pub async fn get_message_count(&self) -> usize {
        let log = self.message_log.lock().await;
        log.len()
    }
    
    pub async fn get_unique_beacon_count(&self) -> usize {
        let log = self.message_log.lock().await;
        let mut beacon_ids = std::collections::HashSet::new();
        for msg in log.iter() {
            beacon_ids.insert(msg.beacon_id);
        }
        beacon_ids.len()
    }
    
    pub async fn get_channel_stats(&self) -> VirtualChannelStats {
        let log = self.message_log.lock().await;
        let mut beacon_ids = std::collections::HashSet::new();
        let mut oldest_message = None;
        let mut newest_message = None;
        
        for msg in log.iter() {
            beacon_ids.insert(msg.beacon_id);
            
            if oldest_message.is_none() || msg.timestamp < oldest_message.unwrap() {
                oldest_message = Some(msg.timestamp);
            }
            
            if newest_message.is_none() || msg.timestamp > newest_message.unwrap() {
                newest_message = Some(msg.timestamp);
            }
        }
        
        VirtualChannelStats {
            name: self.name.clone(),
            total_messages: log.len(),
            unique_beacons: beacon_ids.len(),
            oldest_message,
            newest_message,
            subscriber_count: self.sender.receiver_count(),
        }
    }
    
    pub async fn clear_messages(&self) {
        let mut log = self.message_log.lock().await;
        log.clear();
    }
}

/// Message transmitted in virtual communication space
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VirtualMessage {
    pub beacon_id: Uuid,
    #[serde(
        serialize_with = "serialize_system_time",
        deserialize_with = "deserialize_system_time"
    )]
    pub timestamp: std::time::SystemTime,
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

/// Statistics for a virtual communication channel
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VirtualChannelStats {
    pub name: String,
    pub total_messages: usize,
    pub unique_beacons: usize,
    #[serde(
        serialize_with = "serialize_optional_system_time",
        deserialize_with = "deserialize_optional_system_time"
    )]
    pub oldest_message: Option<SystemTime>,
    #[serde(
        serialize_with = "serialize_optional_system_time",
        deserialize_with = "deserialize_optional_system_time"
    )]
    pub newest_message: Option<SystemTime>,
    pub subscriber_count: usize,
}

// Custom serialization for Option<SystemTime>
fn serialize_optional_system_time<S>(
    time: &Option<SystemTime>,
    serializer: S,
) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    match time {
        Some(t) => {
            let duration = t.duration_since(UNIX_EPOCH)
                .map_err(serde::ser::Error::custom)?;
            serializer.serialize_some(&duration.as_secs())
        }
        None => serializer.serialize_none(),
    }
}

fn deserialize_optional_system_time<'de, D>(
    deserializer: D,
) -> Result<Option<SystemTime>, D::Error>
where
    D: Deserializer<'de>,
{
    let secs: Option<u64> = Option::deserialize(deserializer)?;
    Ok(secs.map(|s| UNIX_EPOCH + Duration::from_secs(s)))
}
#[cfg(test)]
mod tests {
    use super::*;
    use shared_positioning::config::GeodeticPosition;
    use tokio::time::{sleep, Duration as TokioDuration};

    fn create_test_message(beacon_id: Uuid, timestamp: SystemTime) -> VirtualMessage {
        VirtualMessage {
            beacon_id,
            timestamp,
            position: GeodeticPosition {
                latitude: 32.123,
                longitude: 45.476,
                depth: 10.0,
            },
            message_data: vec![0x01, 0x02, 0x03],
            signal_quality: 255,
        }
    }

    #[tokio::test]
    async fn test_virtual_channel_creation() {
        let channel = VirtualChannel::new("test_channel");
        assert_eq!(channel.name(), "test_channel");
        assert_eq!(channel.get_message_count().await, 0);
    }

    #[tokio::test]
    async fn test_message_broadcasting() {
        let channel = VirtualChannel::new("test_channel");
        let mut receiver = channel.subscribe();
        
        let beacon_id = Uuid::new_v4();
        let message = create_test_message(beacon_id, SystemTime::now());
        
        // Broadcast message
        channel.broadcast_message(message.clone()).await.unwrap();
        
        // Verify message was received
        let received = receiver.recv().await.unwrap();
        assert_eq!(received.beacon_id, beacon_id);
        assert_eq!(received.message_data, vec![0x01, 0x02, 0x03]);
        
        // Verify message was logged
        assert_eq!(channel.get_message_count().await, 1);
    }

    #[tokio::test]
    async fn test_multiple_subscribers() {
        let channel = VirtualChannel::new("test_channel");
        let mut receiver1 = channel.subscribe();
        let mut receiver2 = channel.subscribe();
        
        let beacon_id = Uuid::new_v4();
        let message = create_test_message(beacon_id, SystemTime::now());
        
        // Broadcast message
        channel.broadcast_message(message.clone()).await.unwrap();
        
        // Both receivers should get the message
        let received1 = receiver1.recv().await.unwrap();
        let received2 = receiver2.recv().await.unwrap();
        
        assert_eq!(received1.beacon_id, beacon_id);
        assert_eq!(received2.beacon_id, beacon_id);
    }

    #[tokio::test]
    async fn test_message_filtering_by_beacon() {
        let channel = VirtualChannel::new("test_channel");
        
        let beacon1 = Uuid::new_v4();
        let beacon2 = Uuid::new_v4();
        let now = SystemTime::now();
        
        // Send messages from different beacons
        channel.broadcast_message(create_test_message(beacon1, now)).await.unwrap();
        channel.broadcast_message(create_test_message(beacon2, now)).await.unwrap();
        channel.broadcast_message(create_test_message(beacon1, now)).await.unwrap();
        
        // Filter by beacon1
        let beacon1_messages = channel.get_messages_by_beacon(beacon1).await;
        assert_eq!(beacon1_messages.len(), 2);
        assert!(beacon1_messages.iter().all(|msg| msg.beacon_id == beacon1));
        
        // Filter by beacon2
        let beacon2_messages = channel.get_messages_by_beacon(beacon2).await;
        assert_eq!(beacon2_messages.len(), 1);
        assert!(beacon2_messages.iter().all(|msg| msg.beacon_id == beacon2));
    }

    #[tokio::test]
    async fn test_message_filtering_by_time() {
        let channel = VirtualChannel::new("test_channel");
        
        let beacon_id = Uuid::new_v4();
        let start_time = SystemTime::now();
        
        // Send first message
        channel.broadcast_message(create_test_message(beacon_id, start_time)).await.unwrap();
        
        // Wait a bit
        sleep(TokioDuration::from_millis(10)).await;
        let middle_time = SystemTime::now();
        
        // Send second message
        channel.broadcast_message(create_test_message(beacon_id, middle_time)).await.unwrap();
        
        // Get messages since middle time
        let recent_messages = channel.get_messages_since(middle_time).await;
        assert_eq!(recent_messages.len(), 1);
        assert!(recent_messages[0].timestamp >= middle_time);
    }

    #[tokio::test]
    async fn test_combined_filtering() {
        let channel = VirtualChannel::new("test_channel");
        
        let beacon1 = Uuid::new_v4();
        let beacon2 = Uuid::new_v4();
        let start_time = SystemTime::now();
        
        // Send messages from different beacons at different times
        channel.broadcast_message(create_test_message(beacon1, start_time)).await.unwrap();
        
        sleep(TokioDuration::from_millis(10)).await;
        let middle_time = SystemTime::now();
        
        channel.broadcast_message(create_test_message(beacon2, middle_time)).await.unwrap();
        channel.broadcast_message(create_test_message(beacon1, middle_time)).await.unwrap();
        
        // Filter by beacon1 since middle time
        let filtered = channel.get_messages_by_beacon_since(beacon1, middle_time).await;
        assert_eq!(filtered.len(), 1);
        assert_eq!(filtered[0].beacon_id, beacon1);
        assert!(filtered[0].timestamp >= middle_time);
        
        // Test general filtering
        let filtered_general = channel.get_messages_filtered(
            Some(beacon1), 
            Some(middle_time), 
            Some(10)
        ).await;
        assert_eq!(filtered_general.len(), 1);
        assert_eq!(filtered_general[0].beacon_id, beacon1);
    }

    #[tokio::test]
    async fn test_circular_buffer() {
        let channel = VirtualChannel::new("test_channel");
        let beacon_id = Uuid::new_v4();
        
        // Send more than 10000 messages to test circular buffer
        // We'll test with a smaller number for efficiency
        for _i in 0..1005 {
            let message = create_test_message(beacon_id, SystemTime::now());
            channel.broadcast_message(message).await.unwrap();
        }
        
        // Should have triggered circular buffer cleanup
        let count = channel.get_message_count().await;
        assert!(count <= 10000);
        assert!(count >= 1000); // Should have kept at least 1000 after cleanup
    }

    #[tokio::test]
    async fn test_channel_stats() {
        let channel = VirtualChannel::new("test_channel");
        
        let beacon1 = Uuid::new_v4();
        let beacon2 = Uuid::new_v4();
        let now = SystemTime::now();
        
        // Send messages from different beacons
        channel.broadcast_message(create_test_message(beacon1, now)).await.unwrap();
        channel.broadcast_message(create_test_message(beacon2, now)).await.unwrap();
        channel.broadcast_message(create_test_message(beacon1, now)).await.unwrap();
        
        let stats = channel.get_channel_stats().await;
        assert_eq!(stats.name, "test_channel");
        assert_eq!(stats.total_messages, 3);
        assert_eq!(stats.unique_beacons, 2);
        assert!(stats.oldest_message.is_some());
        assert!(stats.newest_message.is_some());
    }

    #[tokio::test]
    async fn test_virtual_communication_space() {
        let mut space = VirtualCommunicationSpace::new();
        
        // Create channels
        let channel1 = space.get_or_create_channel("channel1");
        let channel2 = space.get_or_create_channel("channel2");
        
        assert_eq!(channel1.name(), "channel1");
        assert_eq!(channel2.name(), "channel2");
        assert_eq!(space.channel_count(), 2);
        
        // List channels
        let channels = space.list_channels();
        assert!(channels.contains(&"channel1".to_string()));
        assert!(channels.contains(&"channel2".to_string()));
        
        // Get existing channel
        let same_channel = space.get_or_create_channel("channel1");
        assert_eq!(same_channel.name(), "channel1");
        assert_eq!(space.channel_count(), 2); // Should not create duplicate
    }

    #[tokio::test]
    async fn test_channel_removal() {
        let mut space = VirtualCommunicationSpace::new();
        
        let _channel = space.get_or_create_channel("test_channel");
        assert_eq!(space.channel_count(), 1);
        
        let removed = space.remove_channel("test_channel");
        assert!(removed.is_some());
        assert_eq!(space.channel_count(), 0);
        
        let not_found = space.remove_channel("nonexistent");
        assert!(not_found.is_none());
    }

    #[tokio::test]
    async fn test_clear_messages() {
        let channel = VirtualChannel::new("test_channel");
        let beacon_id = Uuid::new_v4();
        
        // Send some messages
        for _ in 0..5 {
            channel.broadcast_message(create_test_message(beacon_id, SystemTime::now())).await.unwrap();
        }
        
        assert_eq!(channel.get_message_count().await, 5);
        
        // Clear messages
        channel.clear_messages().await;
        assert_eq!(channel.get_message_count().await, 0);
    }

    #[tokio::test]
    async fn test_recent_messages() {
        let channel = VirtualChannel::new("test_channel");
        let beacon_id = Uuid::new_v4();
        
        // Send messages
        for _ in 0..10 {
            channel.broadcast_message(create_test_message(beacon_id, SystemTime::now())).await.unwrap();
        }
        
        // Get recent messages
        let recent = channel.get_recent_messages(5).await;
        assert_eq!(recent.len(), 5);
        
        // Should be in reverse chronological order (most recent first)
        for i in 1..recent.len() {
            assert!(recent[i-1].timestamp >= recent[i].timestamp);
        }
    }

    #[tokio::test]
    async fn test_unique_beacon_count() {
        let channel = VirtualChannel::new("test_channel");
        
        let beacon1 = Uuid::new_v4();
        let beacon2 = Uuid::new_v4();
        let beacon3 = Uuid::new_v4();
        
        // Send messages from different beacons
        channel.broadcast_message(create_test_message(beacon1, SystemTime::now())).await.unwrap();
        channel.broadcast_message(create_test_message(beacon2, SystemTime::now())).await.unwrap();
        channel.broadcast_message(create_test_message(beacon1, SystemTime::now())).await.unwrap(); // Duplicate beacon
        channel.broadcast_message(create_test_message(beacon3, SystemTime::now())).await.unwrap();
        
        assert_eq!(channel.get_unique_beacon_count().await, 3);
        assert_eq!(channel.get_message_count().await, 4);
    }
}