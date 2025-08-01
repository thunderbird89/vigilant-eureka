use clap::ValueEnum;
use serde::{Serialize, Deserialize};
use std::str::FromStr;
use std::path::Path;
use std::time::{SystemTime, UNIX_EPOCH};
use uuid::Uuid;
use crate::{EmulatorError, BeaconLogEntry, LogFilter, BeaconLogger, VirtualBeaconStatus};

/// Export formats for beacon activity logs
#[derive(Debug, Clone, Serialize, Deserialize, ValueEnum)]
#[serde(rename_all = "lowercase")]
pub enum ExportFormat {
    /// JSON format
    Json,
    /// CSV format
    Csv,
}

impl Default for ExportFormat {
    fn default() -> Self {
        Self::Json
    }
}

impl FromStr for ExportFormat {
    type Err = EmulatorError;
    
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "json" => Ok(Self::Json),
            "csv" => Ok(Self::Csv),
            _ => Err(EmulatorError::ExportError(
                format!("Unknown export format: {}", s)
            )),
        }
    }
}

impl std::fmt::Display for ExportFormat {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Json => write!(f, "json"),
            Self::Csv => write!(f, "csv"),
        }
    }
}

/// Export data structure for JSON format
#[derive(Debug, Serialize, Deserialize)]
pub struct ExportData {
    /// Export metadata
    pub metadata: ExportMetadata,
    /// Beacon status information
    pub beacon_status: Vec<VirtualBeaconStatus>,
    /// Log entries
    pub log_entries: Vec<BeaconLogEntry>,
}

/// Metadata about the export
#[derive(Debug, Serialize, Deserialize)]
pub struct ExportMetadata {
    /// Export timestamp
    pub export_timestamp: SystemTime,
    /// Export format
    pub format: String,
    /// Time range start
    pub time_range_start: Option<SystemTime>,
    /// Time range end
    pub time_range_end: Option<SystemTime>,
    /// Filtered beacon ID
    pub beacon_filter: Option<Uuid>,
    /// Channel filter
    pub channel_filter: Option<String>,
    /// Total entries exported
    pub total_entries: usize,
    /// Export version
    pub export_version: String,
}

/// CSV record for log entries
#[derive(Debug, Serialize)]
struct LogEntryCsvRecord {
    id: u64,
    timestamp: String,
    timestamp_unix: u64,
    beacon_id: String,
    entry_type: String,
    latitude: f64,
    longitude: f64,
    depth: f64,
    signal_quality: u8,
    sequence_number: Option<u16>,
    channel: String,
    message_version: Option<String>,
    transmission_interval_ms: Option<u32>,
    movement_pattern: Option<String>,
    message_data_hex: Option<String>,
    message_data_length: Option<usize>,
}

/// CSV record for beacon status
#[derive(Debug, Serialize)]
struct BeaconStatusCsvRecord {
    beacon_id: String,
    is_running: bool,
    latitude: f64,
    longitude: f64,
    depth: f64,
    movement_pattern: String,
    messages_sent: u64,
    transmission_interval_ms: u32,
    uptime_seconds: u64,
    last_transmission: Option<String>,
    transmission_failures: u32,
}

/// Export beacon logs and status to a file
pub async fn export_beacon_data(
    logger: &BeaconLogger,
    beacon_status: &[VirtualBeaconStatus],
    output_path: &Path,
    format: ExportFormat,
    filter: LogFilter,
    include_messages: bool,
) -> Result<usize, EmulatorError> {
    // Get filtered log entries
    let mut log_entries = logger.get_entries(filter.clone()).await;
    
    // Filter out message data if not requested
    if !include_messages {
        for entry in &mut log_entries {
            entry.message_data = None;
        }
    }

    let export_count = log_entries.len();

    match format {
        ExportFormat::Json => {
            export_json(logger, beacon_status, &log_entries, output_path, filter).await?;
        }
        ExportFormat::Csv => {
            export_csv(beacon_status, &log_entries, output_path, include_messages).await?;
        }
    }

    Ok(export_count)
}

/// Export data in JSON format
async fn export_json(
    _logger: &BeaconLogger,
    beacon_status: &[VirtualBeaconStatus],
    log_entries: &[BeaconLogEntry],
    output_path: &Path,
    filter: LogFilter,
) -> Result<(), EmulatorError> {
    let export_data = ExportData {
        metadata: ExportMetadata {
            export_timestamp: SystemTime::now(),
            format: "json".to_string(),
            time_range_start: filter.start_time,
            time_range_end: filter.end_time,
            beacon_filter: filter.beacon_id,
            channel_filter: filter.channel,
            total_entries: log_entries.len(),
            export_version: "1.0".to_string(),
        },
        beacon_status: beacon_status.to_vec(),
        log_entries: log_entries.to_vec(),
    };

    let json = serde_json::to_string_pretty(&export_data)?;
    tokio::fs::write(output_path, json).await?;

    Ok(())
}

/// Export data in CSV format
async fn export_csv(
    beacon_status: &[VirtualBeaconStatus],
    log_entries: &[BeaconLogEntry],
    output_path: &Path,
    include_messages: bool,
) -> Result<(), EmulatorError> {
    let mut csv_content = String::new();

    // Write beacon status section
    csv_content.push_str("# Beacon Status\n");
    csv_content.push_str("beacon_id,is_running,latitude,longitude,depth,movement_pattern,messages_sent,transmission_interval_ms,uptime_seconds,last_transmission,transmission_failures\n");

    for status in beacon_status {
        let record = BeaconStatusCsvRecord {
            beacon_id: status.id.to_string(),
            is_running: status.is_running,
            latitude: status.position.latitude,
            longitude: status.position.longitude,
            depth: status.position.depth,
            movement_pattern: format!("{:?}", status.movement_pattern),
            messages_sent: status.stats.messages_sent,
            transmission_interval_ms: status.config.transmission.interval_ms,
            uptime_seconds: status.stats.uptime.as_secs(),
            last_transmission: status.stats.last_transmission.map(|t| {
                format_timestamp(t)
            }),
            transmission_failures: status.stats.transmission_failures,
        };

        let mut writer = csv::Writer::from_writer(Vec::new());
        writer.serialize(&record).map_err(|e| {
            EmulatorError::ExportError(format!("CSV serialization error: {}", e))
        })?;
        
        let data = String::from_utf8(writer.into_inner().map_err(|e| {
            EmulatorError::ExportError(format!("CSV writer error: {}", e))
        })?).map_err(|e| {
            EmulatorError::ExportError(format!("UTF-8 conversion error: {}", e))
        })?;
        
        csv_content.push_str(&data);
    }

    // Write log entries section
    csv_content.push_str("\n# Log Entries\n");
    
    if include_messages {
        csv_content.push_str("id,timestamp,timestamp_unix,beacon_id,entry_type,latitude,longitude,depth,signal_quality,sequence_number,channel,message_version,transmission_interval_ms,movement_pattern,message_data_hex,message_data_length\n");
    } else {
        csv_content.push_str("id,timestamp,timestamp_unix,beacon_id,entry_type,latitude,longitude,depth,signal_quality,sequence_number,channel,message_version,transmission_interval_ms,movement_pattern\n");
    }

    for entry in log_entries {
        let record = LogEntryCsvRecord {
            id: entry.id,
            timestamp: format_timestamp(entry.timestamp),
            timestamp_unix: entry.timestamp.duration_since(UNIX_EPOCH)
                .unwrap_or_default().as_secs(),
            beacon_id: entry.beacon_id.to_string(),
            entry_type: format!("{:?}", entry.entry_type),
            latitude: entry.position.latitude,
            longitude: entry.position.longitude,
            depth: entry.position.depth,
            signal_quality: entry.signal_quality,
            sequence_number: entry.sequence_number,
            channel: entry.metadata.channel.clone(),
            message_version: entry.metadata.message_version.clone(),
            transmission_interval_ms: entry.metadata.transmission_interval_ms,
            movement_pattern: entry.metadata.movement_pattern.clone(),
            message_data_hex: if include_messages {
                entry.message_data.as_ref().map(|data| hex::encode(data))
            } else {
                None
            },
            message_data_length: entry.message_data.as_ref().map(|data| data.len()),
        };

        let mut writer = csv::Writer::from_writer(Vec::new());
        writer.serialize(&record).map_err(|e| {
            EmulatorError::ExportError(format!("CSV serialization error: {}", e))
        })?;
        
        let data = String::from_utf8(writer.into_inner().map_err(|e| {
            EmulatorError::ExportError(format!("CSV writer error: {}", e))
        })?).map_err(|e| {
            EmulatorError::ExportError(format!("UTF-8 conversion error: {}", e))
        })?;
        
        csv_content.push_str(&data);
    }

    tokio::fs::write(output_path, csv_content).await?;

    Ok(())
}

/// Format timestamp for human-readable display
fn format_timestamp(timestamp: SystemTime) -> String {
    match timestamp.duration_since(UNIX_EPOCH) {
        Ok(duration) => {
            let secs = duration.as_secs();
            let dt = chrono::DateTime::from_timestamp(secs as i64, 0)
                .unwrap_or_else(|| chrono::DateTime::from_timestamp(0, 0).unwrap());
            dt.format("%Y-%m-%d %H:%M:%S UTC").to_string()
        }
        Err(_) => "Invalid timestamp".to_string(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{BeaconLogger, LogEntryType, LogMetadata};
    use shared_positioning::GeodeticPosition;
    use std::collections::HashMap;
    use tempfile::NamedTempFile;

    fn create_test_metadata() -> LogMetadata {
        LogMetadata {
            channel: "test_channel".to_string(),
            message_version: Some("V3".to_string()),
            transmission_interval_ms: Some(5000),
            movement_pattern: Some("Stationary".to_string()),
            custom_fields: HashMap::new(),
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
    async fn test_json_export() {
        let logger = BeaconLogger::new();
        let beacon_id = Uuid::new_v4();
        let position = create_test_position();
        let metadata = create_test_metadata();

        // Add some log entries
        logger.log_event(beacon_id, LogEntryType::BeaconCreated, position, 255, metadata.clone()).await.unwrap();
        logger.log_event(beacon_id, LogEntryType::BeaconStarted, position, 255, metadata.clone()).await.unwrap();

        // Create temporary file
        let temp_file = NamedTempFile::new().unwrap();
        let output_path = temp_file.path();

        // Export to JSON
        let filter = LogFilter::default();
        let beacon_status = vec![];
        
        let count = export_beacon_data(
            &logger,
            &beacon_status,
            output_path,
            ExportFormat::Json,
            filter,
            true,
        ).await.unwrap();

        assert_eq!(count, 2);

        // Verify file was created and contains valid JSON
        let content = tokio::fs::read_to_string(output_path).await.unwrap();
        let export_data: ExportData = serde_json::from_str(&content).unwrap();
        
        assert_eq!(export_data.log_entries.len(), 2);
        assert_eq!(export_data.metadata.total_entries, 2);
        assert_eq!(export_data.metadata.format, "json");
    }

    #[tokio::test]
    async fn test_csv_export() {
        let logger = BeaconLogger::new();
        let beacon_id = Uuid::new_v4();
        let position = create_test_position();
        let metadata = create_test_metadata();

        // Add some log entries
        logger.log_event(beacon_id, LogEntryType::BeaconCreated, position, 255, metadata.clone()).await.unwrap();

        // Create temporary file
        let temp_file = NamedTempFile::new().unwrap();
        let output_path = temp_file.path();

        // Export to CSV
        let filter = LogFilter::default();
        let beacon_status = vec![];
        
        let count = export_beacon_data(
            &logger,
            &beacon_status,
            output_path,
            ExportFormat::Csv,
            filter,
            false,
        ).await.unwrap();

        assert_eq!(count, 1);

        // Verify file was created and contains CSV data
        let content = tokio::fs::read_to_string(output_path).await.unwrap();
        assert!(content.contains("# Beacon Status"));
        assert!(content.contains("# Log Entries"));
        assert!(content.contains("beacon_id,is_running"));
        assert!(content.contains(&beacon_id.to_string()));
    }

    #[tokio::test]
    async fn test_export_with_filtering() {
        let logger = BeaconLogger::new();
        let beacon1 = Uuid::new_v4();
        let beacon2 = Uuid::new_v4();
        let position = create_test_position();
        let metadata = create_test_metadata();

        // Add log entries for different beacons
        logger.log_event(beacon1, LogEntryType::BeaconCreated, position, 255, metadata.clone()).await.unwrap();
        logger.log_event(beacon2, LogEntryType::BeaconCreated, position, 255, metadata.clone()).await.unwrap();
        logger.log_event(beacon1, LogEntryType::BeaconStarted, position, 255, metadata.clone()).await.unwrap();

        // Create temporary file
        let temp_file = NamedTempFile::new().unwrap();
        let output_path = temp_file.path();

        // Export with beacon filter
        let filter = LogFilter {
            beacon_id: Some(beacon1),
            ..Default::default()
        };
        let beacon_status = vec![];
        
        let count = export_beacon_data(
            &logger,
            &beacon_status,
            output_path,
            ExportFormat::Json,
            filter,
            true,
        ).await.unwrap();

        assert_eq!(count, 2); // Only beacon1 entries

        // Verify filtered content
        let content = tokio::fs::read_to_string(output_path).await.unwrap();
        let export_data: ExportData = serde_json::from_str(&content).unwrap();
        
        assert_eq!(export_data.log_entries.len(), 2);
        assert!(export_data.log_entries.iter().all(|e| e.beacon_id == beacon1));
    }

    #[tokio::test]
    async fn test_export_without_messages() {
        let logger = BeaconLogger::new();
        let beacon_id = Uuid::new_v4();
        let position = create_test_position();
        let metadata = create_test_metadata();

        // Add a message transmission entry
        let message = shared_positioning::VirtualMessage {
            beacon_id,
            timestamp: SystemTime::now(),
            position,
            message_data: vec![0x01, 0x02, 0x03, 0x04],
            signal_quality: 200,
        };

        logger.log_message_transmission(beacon_id, position, &message, metadata).await.unwrap();

        // Create temporary file
        let temp_file = NamedTempFile::new().unwrap();
        let output_path = temp_file.path();

        // Export without message content
        let filter = LogFilter::default();
        let beacon_status = vec![];
        
        export_beacon_data(
            &logger,
            &beacon_status,
            output_path,
            ExportFormat::Json,
            filter,
            false, // Don't include messages
        ).await.unwrap();

        // Verify message data was filtered out
        let content = tokio::fs::read_to_string(output_path).await.unwrap();
        let export_data: ExportData = serde_json::from_str(&content).unwrap();
        
        assert_eq!(export_data.log_entries.len(), 1);
        assert!(export_data.log_entries[0].message_data.is_none());
    }
}