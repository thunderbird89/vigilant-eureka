use serde::{Serialize, Deserialize, Serializer, Deserializer};
use std::time::{SystemTime, UNIX_EPOCH, Duration};
use uuid::Uuid;
use crate::GeodeticPosition;

/// Message transmitted in virtual communication space
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

/// IPC protocol messages for virtual communication
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum IpcMessage {
    /// Subscribe to a virtual channel
    Subscribe { channel_name: String, receiver_id: u8 },
    /// Unsubscribe from a virtual channel
    Unsubscribe { channel_name: String, receiver_id: u8 },
    /// Virtual beacon message broadcast
    VirtualMessage(VirtualMessage),
    /// Acknowledgment response
    Ack,
    /// Error response
    Error { message: String },
    /// Heartbeat/keepalive
    Heartbeat,
    /// List available channels
    ListChannels,
    /// Channel list response
    ChannelList { channels: Vec<String> },
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