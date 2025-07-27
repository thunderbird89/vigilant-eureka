use std::collections::VecDeque;
use std::sync::Arc;
use std::time::{SystemTime, UNIX_EPOCH};
use tokio::sync::RwLock;
use serde::{Serialize, Deserialize};
use uuid::Uuid;
use shared_positioning::{GeodeticPosition, VirtualMessage};
use crate::EmulatorError;

/// Maximum number of log entries to keep in memory
const MAX_LOG_ENTRIES: usize = 100_000;

/// Cleanup threshold - when to trigger log rotation
const CLEANUP_THRESHOLD: usize = 120_000;

/// Number of entries to keep after cleanup
const CLEANUP_KEEP_COUNT: usize = 80_000;

/// Comprehensive log entry for beacon activities
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BeaconLogEntry {
    /// Unique log entry ID
    pub id: u64,
    /// Timestamp when the event occurred
    pub timestamp: SystemTime,
    /// Beacon ID that generated this log entry
    pub beacon_id: Uuid,
    /// Type of log entry
    pub entry_type: LogEntryType,
    /// Beacon position at the time of the event
    pub position: GeodeticPosition,
    /// Optional message data if this is a transmission log
    pub message_data: Option<Vec<u8>>,
    /// Signal quality at the time of the event
    pub signal_quality: u8,
    /// Sequence number for transmission events
    pub sequence_number: Option<u16>,
    /// Additional metadata
    pub metadata: LogMetadata,
}

/// Types of log entries
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum LogEntryType {
    /// Beacon was created
    BeaconCreated,
    /// Beacon was started
    BeaconStarted,
    /// Beacon was stopped
    BeaconStopped,
    /// Beacon was removed
    BeaconRemoved,
    /// Message was transmitted
    MessageTransmitted,
    /// Transmission failed
    TransmissionFailed { error: String },
    /// Position was updated
    PositionUpdated { old_position: GeodeticPosition },
    /// Configuration was updated
    ConfigurationUpdated,
    /// Movement pattern was changed
    MovementPatternChanged,
    /// Beacon health check
    HealthCheck { status: String },
}

/// Additional metadata for log entries
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LogMetadata {
    /// Channel name where the event occurred
    pub channel: String,
    /// Message format version (for transmission events)
    pub message_version: Option<String>,
    /// Transmission interval at the time of the event
    pub transmission_interval_ms: Option<u32>,
    /// Movement pattern at the time of the event
    pub movement_pattern: Option<String>,
    /// Additional custom fields
    pub custom_fields: std::collections::HashMap<String, String>,
}

/// Filtering criteria for log retrieval
#[derive(Debug, Clone)]
pub struct LogFilter {
    /// Filter by beacon ID
    pub beacon_id: Option<Uuid>,
    /// Filter by entry type
    pub entry_type: Option<LogEntryType>,
    /// Filter by time range (start time)
    pub start_time: Option<SystemTime>,
    /// Filter by time range (end time)
    pub end_time: Option<SystemTime>,
    /// Filter by channel
    pub channel: Option<String>,
    /// Maximum number of entries to return
    pub limit: Option<usize>,
}

impl Default for LogFilter {
    fn default() -> Self {
        Self {
            beacon_id: None,
            entry_type: None,
            start_time: None,
            end_time: None,
            channel: None,
            limit: None,
        }
    }
}

/// Statistics about the logging system
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoggingStats {
    /// Total number of log entries
    pub total_entries: usize,
    /// Number of entries by type
    pub entries_by_type: std::collections::HashMap<String, usize>,
    /// Number of entries by beacon
    pub entries_by_beacon: std::collections::HashMap<Uuid, usize>,
    /// Oldest log entry timestamp
    pub oldest_entry: Option<SystemTime>,
    /// Newest log entry timestamp
    pub newest_entry: Option<SystemTime>,
    /// Memory usage estimate in bytes
    pub estimated_memory_usage: usize,
    /// Number of cleanup operations performed
    pub cleanup_count: u64,
}

/// Comprehensive logging system for beacon activities
pub struct BeaconLogger {
    /// Log entries stored in memory with circular buffer behavior
    entries: Arc<RwLock<VecDeque<BeaconLogEntry>>>,
    /// Next log entry ID
    next_id: Arc<RwLock<u64>>,
    /// Statistics
    stats: Arc<RwLock<LoggingStats>>,
}

impl BeaconLogger {
    /// Create a new beacon logger
    pub fn new() -> Self {
        Self {
            entries: Arc::new(RwLock::new(VecDeque::with_capacity(MAX_LOG_ENTRIES))),
            next_id: Arc::new(RwLock::new(1)),
            stats: Arc::new(RwLock::new(LoggingStats {
                total_entries: 0,
                entries_by_type: std::collections::HashMap::new(),
                entries_by_beacon: std::collections::HashMap::new(),
                oldest_entry: None,
                newest_entry: None,
                estimated_memory_usage: 0,
                cleanup_count: 0,
            })),
        }
    }

    /// Log a beacon activity event
    pub async fn log_event(
        &self,
        beacon_id: Uuid,
        entry_type: LogEntryType,
        position: GeodeticPosition,
        signal_quality: u8,
        metadata: LogMetadata,
    ) -> Result<u64, EmulatorError> {
        let timestamp = SystemTime::now();
        let id = {
            let mut next_id = self.next_id.write().await;
            let current_id = *next_id;
            *next_id += 1;
            current_id
        };

        let entry = BeaconLogEntry {
            id,
            timestamp,
            beacon_id,
            entry_type: entry_type.clone(),
            position,
            message_data: None,
            signal_quality,
            sequence_number: None,
            metadata,
        };

        self.add_entry(entry).await?;
        Ok(id)
    }

    /// Log a message transmission event
    pub async fn log_message_transmission(
        &self,
        beacon_id: Uuid,
        position: GeodeticPosition,
        message: &VirtualMessage,
        metadata: LogMetadata,
    ) -> Result<u64, EmulatorError> {
        let timestamp = SystemTime::now();
        let id = {
            let mut next_id = self.next_id.write().await;
            let current_id = *next_id;
            *next_id += 1;
            current_id
        };

        let entry = BeaconLogEntry {
            id,
            timestamp,
            beacon_id,
            entry_type: LogEntryType::MessageTransmitted,
            position,
            message_data: Some(message.message_data.clone()),
            signal_quality: message.signal_quality,
            sequence_number: None, // Could be extracted from message if needed
            metadata,
        };

        self.add_entry(entry).await?;
        Ok(id)
    }

    /// Log a transmission failure
    pub async fn log_transmission_failure(
        &self,
        beacon_id: Uuid,
        position: GeodeticPosition,
        error: String,
        metadata: LogMetadata,
    ) -> Result<u64, EmulatorError> {
        let timestamp = SystemTime::now();
        let id = {
            let mut next_id = self.next_id.write().await;
            let current_id = *next_id;
            *next_id += 1;
            current_id
        };

        let entry = BeaconLogEntry {
            id,
            timestamp,
            beacon_id,
            entry_type: LogEntryType::TransmissionFailed { error },
            position,
            message_data: None,
            signal_quality: 0,
            sequence_number: None,
            metadata,
        };

        self.add_entry(entry).await?;
        Ok(id)
    }

    /// Add a log entry to the system
    async fn add_entry(&self, entry: BeaconLogEntry) -> Result<(), EmulatorError> {
        let mut entries = self.entries.write().await;
        let mut stats = self.stats.write().await;

        // Check if we need to perform cleanup
        if entries.len() >= CLEANUP_THRESHOLD {
            self.perform_cleanup(&mut entries, &mut stats).await;
        }

        // Add the new entry
        entries.push_back(entry.clone());

        // Update statistics
        stats.total_entries += 1;
        
        let type_key = format!("{:?}", entry.entry_type);
        *stats.entries_by_type.entry(type_key).or_insert(0) += 1;
        *stats.entries_by_beacon.entry(entry.beacon_id).or_insert(0) += 1;

        if stats.oldest_entry.is_none() {
            stats.oldest_entry = Some(entry.timestamp);
        }
        stats.newest_entry = Some(entry.timestamp);

        // Estimate memory usage (rough calculation)
        stats.estimated_memory_usage = entries.len() * std::mem::size_of::<BeaconLogEntry>();

        Ok(())
    }

    /// Perform log rotation and cleanup
    async fn perform_cleanup(
        &self,
        entries: &mut VecDeque<BeaconLogEntry>,
        stats: &mut LoggingStats,
    ) {
        let remove_count = entries.len().saturating_sub(CLEANUP_KEEP_COUNT);
        
        for _ in 0..remove_count {
            if let Some(removed_entry) = entries.pop_front() {
                // Update statistics
                let type_key = format!("{:?}", removed_entry.entry_type);
                if let Some(count) = stats.entries_by_type.get_mut(&type_key) {
                    *count = count.saturating_sub(1);
                    if *count == 0 {
                        stats.entries_by_type.remove(&type_key);
                    }
                }

                if let Some(count) = stats.entries_by_beacon.get_mut(&removed_entry.beacon_id) {
                    *count = count.saturating_sub(1);
                    if *count == 0 {
                        stats.entries_by_beacon.remove(&removed_entry.beacon_id);
                    }
                }
            }
        }

        // Update oldest entry timestamp
        if let Some(oldest) = entries.front() {
            stats.oldest_entry = Some(oldest.timestamp);
        }

        stats.cleanup_count += 1;
        stats.estimated_memory_usage = entries.len() * std::mem::size_of::<BeaconLogEntry>();

        tracing::info!(
            "Log cleanup performed: removed {} entries, {} entries remaining",
            remove_count,
            entries.len()
        );
    }

    /// Retrieve log entries with filtering
    pub async fn get_entries(&self, filter: LogFilter) -> Vec<BeaconLogEntry> {
        let entries = self.entries.read().await;
        let mut filtered_entries: Vec<BeaconLogEntry> = entries
            .iter()
            .filter(|entry| {
                // Filter by beacon ID
                if let Some(beacon_id) = filter.beacon_id {
                    if entry.beacon_id != beacon_id {
                        return false;
                    }
                }

                // Filter by entry type
                if let Some(ref entry_type) = filter.entry_type {
                    if std::mem::discriminant(&entry.entry_type) != std::mem::discriminant(entry_type) {
                        return false;
                    }
                }

                // Filter by start time
                if let Some(start_time) = filter.start_time {
                    if entry.timestamp < start_time {
                        return false;
                    }
                }

                // Filter by end time
                if let Some(end_time) = filter.end_time {
                    if entry.timestamp > end_time {
                        return false;
                    }
                }

                // Filter by channel
                if let Some(ref channel) = filter.channel {
                    if entry.metadata.channel != *channel {
                        return false;
                    }
                }

                true
            })
            .cloned()
            .collect();

        // Sort by timestamp (newest first)
        filtered_entries.sort_by(|a, b| b.timestamp.cmp(&a.timestamp));

        // Apply limit
        if let Some(limit) = filter.limit {
            filtered_entries.truncate(limit);
        }

        filtered_entries
    }

    /// Get entries for a specific time range
    pub async fn get_entries_since(&self, since: SystemTime) -> Vec<BeaconLogEntry> {
        let filter = LogFilter {
            start_time: Some(since),
            ..Default::default()
        };
        self.get_entries(filter).await
    }

    /// Get entries for a specific beacon
    pub async fn get_beacon_entries(&self, beacon_id: Uuid) -> Vec<BeaconLogEntry> {
        let filter = LogFilter {
            beacon_id: Some(beacon_id),
            ..Default::default()
        };
        self.get_entries(filter).await
    }

    /// Get recent entries (limited count)
    pub async fn get_recent_entries(&self, count: usize) -> Vec<BeaconLogEntry> {
        let filter = LogFilter {
            limit: Some(count),
            ..Default::default()
        };
        self.get_entries(filter).await
    }

    /// Get logging statistics
    pub async fn get_stats(&self) -> LoggingStats {
        self.stats.read().await.clone()
    }

    /// Clear all log entries
    pub async fn clear_logs(&self) {
        let mut entries = self.entries.write().await;
        let mut stats = self.stats.write().await;

        entries.clear();
        *stats = LoggingStats {
            total_entries: 0,
            entries_by_type: std::collections::HashMap::new(),
            entries_by_beacon: std::collections::HashMap::new(),
            oldest_entry: None,
            newest_entry: None,
            estimated_memory_usage: 0,
            cleanup_count: stats.cleanup_count, // Preserve cleanup count
        };

        tracing::info!("All log entries cleared");
    }

    /// Get total number of log entries
    pub async fn get_entry_count(&self) -> usize {
        self.entries.read().await.len()
    }
}

impl Default for BeaconLogger {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    fn create_test_metadata(channel: &str) -> LogMetadata {
        LogMetadata {
            channel: channel.to_string(),
            message_version: Some("V3".to_string()),
            transmission_interval_ms: Some(5000),
            movement_pattern: Some("Stationary".to_string()),
            custom_fields: std::collections::HashMap::new(),
        }
    }

    fn create_test_position() -> GeodeticPosition {
        GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        }
    }

    #[tokio::test]
    async fn test_logger_creation() {
        let logger = BeaconLogger::new();
        assert_eq!(logger.get_entry_count().await, 0);
        
        let stats = logger.get_stats().await;
        assert_eq!(stats.total_entries, 0);
        assert!(stats.oldest_entry.is_none());
        assert!(stats.newest_entry.is_none());
    }

    #[tokio::test]
    async fn test_log_event() {
        let logger = BeaconLogger::new();
        let beacon_id = Uuid::new_v4();
        let position = create_test_position();
        let metadata = create_test_metadata("test_channel");

        let log_id = logger.log_event(
            beacon_id,
            LogEntryType::BeaconCreated,
            position,
            255,
            metadata,
        ).await.unwrap();

        assert_eq!(log_id, 1);
        assert_eq!(logger.get_entry_count().await, 1);

        let entries = logger.get_recent_entries(10).await;
        assert_eq!(entries.len(), 1);
        assert_eq!(entries[0].beacon_id, beacon_id);
        assert!(matches!(entries[0].entry_type, LogEntryType::BeaconCreated));
    }

    #[tokio::test]
    async fn test_log_message_transmission() {
        let logger = BeaconLogger::new();
        let beacon_id = Uuid::new_v4();
        let position = create_test_position();
        let metadata = create_test_metadata("test_channel");

        let message = VirtualMessage {
            beacon_id,
            timestamp: SystemTime::now(),
            position,
            message_data: vec![0x01, 0x02, 0x03],
            signal_quality: 200,
        };

        let log_id = logger.log_message_transmission(
            beacon_id,
            position,
            &message,
            metadata,
        ).await.unwrap();

        assert_eq!(log_id, 1);
        
        let entries = logger.get_recent_entries(10).await;
        assert_eq!(entries.len(), 1);
        assert!(matches!(entries[0].entry_type, LogEntryType::MessageTransmitted));
        assert_eq!(entries[0].message_data, Some(vec![0x01, 0x02, 0x03]));
        assert_eq!(entries[0].signal_quality, 200);
    }

    #[tokio::test]
    async fn test_filtering_by_beacon() {
        let logger = BeaconLogger::new();
        let beacon1 = Uuid::new_v4();
        let beacon2 = Uuid::new_v4();
        let position = create_test_position();
        let metadata = create_test_metadata("test_channel");

        // Log events for different beacons
        logger.log_event(beacon1, LogEntryType::BeaconCreated, position, 255, metadata.clone()).await.unwrap();
        logger.log_event(beacon2, LogEntryType::BeaconCreated, position, 255, metadata.clone()).await.unwrap();
        logger.log_event(beacon1, LogEntryType::BeaconStarted, position, 255, metadata.clone()).await.unwrap();

        // Filter by beacon1
        let beacon1_entries = logger.get_beacon_entries(beacon1).await;
        assert_eq!(beacon1_entries.len(), 2);
        assert!(beacon1_entries.iter().all(|e| e.beacon_id == beacon1));

        // Filter by beacon2
        let beacon2_entries = logger.get_beacon_entries(beacon2).await;
        assert_eq!(beacon2_entries.len(), 1);
        assert!(beacon2_entries.iter().all(|e| e.beacon_id == beacon2));
    }

    #[tokio::test]
    async fn test_filtering_by_time() {
        let logger = BeaconLogger::new();
        let beacon_id = Uuid::new_v4();
        let position = create_test_position();
        let metadata = create_test_metadata("test_channel");

        // Log first event
        logger.log_event(beacon_id, LogEntryType::BeaconCreated, position, 255, metadata.clone()).await.unwrap();

        // Wait a bit
        tokio::time::sleep(Duration::from_millis(10)).await;
        let middle_time = SystemTime::now();

        // Log second event
        logger.log_event(beacon_id, LogEntryType::BeaconStarted, position, 255, metadata.clone()).await.unwrap();

        // Get entries since middle time
        let recent_entries = logger.get_entries_since(middle_time).await;
        assert_eq!(recent_entries.len(), 1);
        assert!(matches!(recent_entries[0].entry_type, LogEntryType::BeaconStarted));
    }

    #[tokio::test]
    async fn test_statistics() {
        let logger = BeaconLogger::new();
        let beacon1 = Uuid::new_v4();
        let beacon2 = Uuid::new_v4();
        let position = create_test_position();
        let metadata = create_test_metadata("test_channel");

        // Log various events
        logger.log_event(beacon1, LogEntryType::BeaconCreated, position, 255, metadata.clone()).await.unwrap();
        logger.log_event(beacon1, LogEntryType::BeaconStarted, position, 255, metadata.clone()).await.unwrap();
        logger.log_event(beacon2, LogEntryType::BeaconCreated, position, 255, metadata.clone()).await.unwrap();

        let stats = logger.get_stats().await;
        assert_eq!(stats.total_entries, 3);
        assert_eq!(stats.entries_by_beacon.len(), 2);
        assert_eq!(stats.entries_by_beacon[&beacon1], 2);
        assert_eq!(stats.entries_by_beacon[&beacon2], 1);
        assert!(stats.oldest_entry.is_some());
        assert!(stats.newest_entry.is_some());
    }

    #[tokio::test]
    async fn test_clear_logs() {
        let logger = BeaconLogger::new();
        let beacon_id = Uuid::new_v4();
        let position = create_test_position();
        let metadata = create_test_metadata("test_channel");

        // Add some entries
        logger.log_event(beacon_id, LogEntryType::BeaconCreated, position, 255, metadata.clone()).await.unwrap();
        logger.log_event(beacon_id, LogEntryType::BeaconStarted, position, 255, metadata.clone()).await.unwrap();

        assert_eq!(logger.get_entry_count().await, 2);

        // Clear logs
        logger.clear_logs().await;

        assert_eq!(logger.get_entry_count().await, 0);
        let stats = logger.get_stats().await;
        assert_eq!(stats.total_entries, 0);
        assert!(stats.oldest_entry.is_none());
        assert!(stats.newest_entry.is_none());
    }

    #[tokio::test]
    async fn test_complex_filtering() {
        let logger = BeaconLogger::new();
        let beacon1 = Uuid::new_v4();
        let beacon2 = Uuid::new_v4();
        let position = create_test_position();
        let metadata1 = create_test_metadata("channel1");
        let metadata2 = create_test_metadata("channel2");

        // Log events on different channels
        logger.log_event(beacon1, LogEntryType::BeaconCreated, position, 255, metadata1.clone()).await.unwrap();
        logger.log_event(beacon2, LogEntryType::BeaconCreated, position, 255, metadata2.clone()).await.unwrap();
        logger.log_event(beacon1, LogEntryType::BeaconStarted, position, 255, metadata1.clone()).await.unwrap();

        // Filter by beacon and channel
        let filter = LogFilter {
            beacon_id: Some(beacon1),
            channel: Some("channel1".to_string()),
            ..Default::default()
        };

        let filtered_entries = logger.get_entries(filter).await;
        assert_eq!(filtered_entries.len(), 2);
        assert!(filtered_entries.iter().all(|e| e.beacon_id == beacon1));
        assert!(filtered_entries.iter().all(|e| e.metadata.channel == "channel1"));
    }
}