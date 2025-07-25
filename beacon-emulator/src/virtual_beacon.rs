use uuid::Uuid;
use serde::{Serialize, Deserialize};
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
    pub last_transmission: Option<std::time::SystemTime>,
    pub uptime: std::time::Duration,
    pub transmission_failures: u32,
}

impl VirtualBeaconStats {
    pub fn new() -> Self {
        Self {
            messages_sent: 0,
            last_transmission: None,
            uptime: std::time::Duration::new(0, 0),
            transmission_failures: 0,
        }
    }
}