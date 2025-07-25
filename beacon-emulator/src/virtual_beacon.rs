use uuid::Uuid;
use serde::{Serialize, Deserialize, Serializer, Deserializer};
use std::time::{SystemTime, UNIX_EPOCH, Duration};
use shared_positioning::{
    BeaconConfig,
    config::GeodeticPosition,
};
use crate::{
    EmulatorError, 
    VirtualChannel, 
    MovementPattern
};

/// Virtual beacon that emulates real beacon behavior
pub struct VirtualBeacon {
    id: Uuid,
    config: BeaconConfig,
    position: GeodeticPosition,
    movement_pattern: MovementPattern,
    sequence_number: u16,
    is_running: bool,
    virtual_channel: VirtualChannel,
    stats: VirtualBeaconStats,
}

impl VirtualBeacon {
    pub fn new(
        id: Uuid,
        config: BeaconConfig,
        initial_position: GeodeticPosition,
        virtual_channel: VirtualChannel,
    ) -> Result<Self, EmulatorError> {
        Ok(Self {
            id,
            config,
            position: initial_position,
            movement_pattern: MovementPattern::Stationary,
            sequence_number: 0,
            is_running: false,
            virtual_channel,
            stats: VirtualBeaconStats::new(),
        })
    }
    
    pub fn id(&self) -> Uuid {
        self.id
    }
    
    pub fn position(&self) -> GeodeticPosition {
        self.position
    }
    
    pub fn is_running(&self) -> bool {
        self.is_running
    }
    
    pub fn get_status(&self) -> VirtualBeaconStatus {
        VirtualBeaconStatus {
            id: self.id,
            position: self.position,
            is_running: self.is_running,
            movement_pattern: self.movement_pattern.clone(),
            stats: self.stats.clone(),
            config: self.config.clone(),
        }
    }
    
    pub fn stop(&mut self) {
        self.is_running = false;
    }
    
    pub fn update_position(&mut self, position: GeodeticPosition) {
        self.position = position;
    }
    
    pub fn set_movement_pattern(&mut self, pattern: MovementPattern) {
        self.movement_pattern = pattern;
    }
}

/// Status information for a virtual beacon
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VirtualBeaconStatus {
    pub id: Uuid,
    pub position: GeodeticPosition,
    pub is_running: bool,
    pub movement_pattern: MovementPattern,
    pub stats: VirtualBeaconStats,
    pub config: BeaconConfig,
}

/// Statistics for virtual beacon operation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VirtualBeaconStats {
    pub messages_sent: u64,
    #[serde(
        serialize_with = "serialize_optional_system_time",
        deserialize_with = "deserialize_optional_system_time"
    )]
    pub last_transmission: Option<SystemTime>,
    pub uptime: Duration,
    pub transmission_failures: u32,
}

impl VirtualBeaconStats {
    pub fn new() -> Self {
        Self {
            messages_sent: 0,
            last_transmission: None,
            uptime: Duration::new(0, 0),
            transmission_failures: 0,
        }
    }
}

// Custom serialization for SystemTime
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
    let opt_secs: Option<u64> = Option::deserialize(deserializer)?;
    match opt_secs {
        Some(secs) => {
            let time = UNIX_EPOCH + Duration::from_secs(secs);
            Ok(Some(time))
        }
        None => Ok(None),
    }
}