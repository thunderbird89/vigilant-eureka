use std::collections::HashMap;
use uuid::Uuid;
use tokio::task::JoinHandle;
use shared_positioning::{
    BeaconConfig,
    config::GeodeticPosition,
};
use crate::{
    EmulatorError,
    VirtualBeacon,
    VirtualBeaconStatus,
    VirtualCommunicationSpace,
    ScenarioType,
    ExportFormat,
};

/// Main emulator manager that coordinates virtual beacons
pub struct EmulatorManager {
    virtual_beacons: HashMap<Uuid, VirtualBeacon>,
    beacon_tasks: HashMap<Uuid, JoinHandle<()>>,
    communication_space: VirtualCommunicationSpace,
    current_channel: String,
}

impl EmulatorManager {
    pub fn new(channel_name: &str) -> Self {
        let communication_space = VirtualCommunicationSpace::new();
        let current_channel = channel_name.to_string();
        
        Self {
            virtual_beacons: HashMap::new(),
            beacon_tasks: HashMap::new(),
            communication_space,
            current_channel,
        }
    }
    
    pub async fn create_beacon(
        &mut self,
        id: Option<Uuid>,
        position: GeodeticPosition,
        config: Option<BeaconConfig>,
    ) -> Result<Uuid, EmulatorError> {
        let beacon_id = id.unwrap_or_else(Uuid::new_v4);
        
        if self.virtual_beacons.contains_key(&beacon_id) {
            return Err(EmulatorError::BeaconExists(beacon_id));
        }
        
        let beacon_config = config.unwrap_or_else(|| {
            // Create default config for emulator
            BeaconConfig::new(beacon_id)
        });
        
        let virtual_channel = self.communication_space.get_or_create_channel(&self.current_channel);
        
        let virtual_beacon = VirtualBeacon::new(
            beacon_id,
            beacon_config,
            position,
            virtual_channel,
        )?;
        
        self.virtual_beacons.insert(beacon_id, virtual_beacon);
        
        Ok(beacon_id)
    }
    
    pub async fn stop_beacon(&mut self, id: Uuid) -> Result<(), EmulatorError> {
        if let Some(mut beacon) = self.virtual_beacons.remove(&id) {
            beacon.stop();
            
            if let Some(task) = self.beacon_tasks.remove(&id) {
                task.abort();
            }
            
            Ok(())
        } else {
            Err(EmulatorError::BeaconNotFound(id))
        }
    }
    
    pub async fn stop_all_beacons(&mut self) -> Result<(), EmulatorError> {
        let beacon_ids: Vec<Uuid> = self.virtual_beacons.keys().cloned().collect();
        
        for id in beacon_ids {
            self.stop_beacon(id).await?;
        }
        
        Ok(())
    }
    
    pub fn list_beacons(&self) -> Vec<VirtualBeaconStatus> {
        self.virtual_beacons.values()
            .map(|beacon| beacon.get_status())
            .collect()
    }
    
    pub async fn update_beacon_position(
        &mut self,
        id: Uuid,
        position: GeodeticPosition,
    ) -> Result<(), EmulatorError> {
        if let Some(beacon) = self.virtual_beacons.get_mut(&id) {
            beacon.update_position(position);
            Ok(())
        } else {
            Err(EmulatorError::BeaconNotFound(id))
        }
    }
    
    pub async fn create_scenario(
        &mut self,
        _scenario_type: ScenarioType,
        _count: usize,
        _spacing: f64,
        _center: GeodeticPosition,
    ) -> Result<Vec<Uuid>, EmulatorError> {
        // TODO: Implement scenario generation
        // This will be implemented in a later task
        Err(EmulatorError::InvalidScenario("Scenario generation not yet implemented".to_string()))
    }
    
    pub async fn export_logs(
        &self,
        _output_path: &std::path::Path,
        _format: ExportFormat,
        _duration_s: u64,
    ) -> Result<(), EmulatorError> {
        // TODO: Implement log export
        // This will be implemented in a later task
        Err(EmulatorError::ExportError("Log export not yet implemented".to_string()))
    }
}