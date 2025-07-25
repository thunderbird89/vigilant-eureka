use crate::{
    EmulatorManager, EmulatorError,
    daemon_protocol::{DaemonCommand, DaemonResponse, get_socket_path}
};
use tokio::net::{UnixListener, UnixStream};
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use std::sync::Arc;
use tokio::sync::Mutex;
use tracing::{info, error, debug, warn};

pub struct DaemonServer {
    emulator: Arc<Mutex<EmulatorManager>>,
    listener: UnixListener,
}

impl DaemonServer {
    pub async fn new(emulator: EmulatorManager) -> Result<Self, Box<dyn std::error::Error>> {
        let socket_path = get_socket_path();
        
        // Remove existing socket file if it exists
        if socket_path.exists() {
            std::fs::remove_file(&socket_path)?;
        }
        
        let listener = UnixListener::bind(&socket_path)?;
        info!("Daemon listening on socket: {}", socket_path.display());
        
        Ok(Self {
            emulator: Arc::new(Mutex::new(emulator)),
            listener,
        })
    }
    
    pub async fn run(&self) -> Result<(), Box<dyn std::error::Error>> {
        info!("Daemon server started, accepting connections...");
        
        loop {
            match self.listener.accept().await {
                Ok((stream, _)) => {
                    let emulator = Arc::clone(&self.emulator);
                    tokio::spawn(async move {
                        if let Err(e) = Self::handle_client(stream, emulator).await {
                            error!("Error handling client: {}", e);
                        }
                    });
                }
                Err(e) => {
                    error!("Error accepting connection: {}", e);
                }
            }
        }
    }
    
    async fn handle_client(
        mut stream: UnixStream,
        emulator: Arc<Mutex<EmulatorManager>>,
    ) -> Result<(), Box<dyn std::error::Error>> {
        // Read command length
        let mut length_bytes = [0u8; 4];
        stream.read_exact(&mut length_bytes).await?;
        let command_length = u32::from_le_bytes(length_bytes) as usize;
        
        // Read command
        let mut command_bytes = vec![0u8; command_length];
        stream.read_exact(&mut command_bytes).await?;
        
        let command_json = String::from_utf8(command_bytes)?;
        let command: DaemonCommand = serde_json::from_str(&command_json)?;
        
        debug!("Received command: {:?}", command);
        
        // Process command
        let response = Self::process_command(command, emulator).await;
        
        // Send response
        let response_json = serde_json::to_string(&response)?;
        let response_bytes = response_json.as_bytes();
        let length = response_bytes.len() as u32;
        
        // Send length prefix
        stream.write_all(&length.to_le_bytes()).await?;
        // Send response
        stream.write_all(response_bytes).await?;
        stream.flush().await?;
        
        Ok(())
    }
    
    async fn process_command(
        command: DaemonCommand,
        emulator: Arc<Mutex<EmulatorManager>>,
    ) -> DaemonResponse {
        let mut emulator = emulator.lock().await;
        
        match command {
            DaemonCommand::CreateBeacon { id, position, config_path, interval, movement, start } => {
                match Self::handle_create_beacon(&mut emulator, id, position, config_path, interval, movement, start).await {
                    Ok(beacon_id) => DaemonResponse::BeaconCreated { id: beacon_id },
                    Err(e) => DaemonResponse::Error { message: e.to_string() },
                }
            }
            
            DaemonCommand::StartBeacon { id } => {
                match emulator.start_beacon(id).await {
                    Ok(()) => DaemonResponse::Success { 
                        message: Some(format!("Started beacon {}", id)) 
                    },
                    Err(e) => DaemonResponse::Error { message: e.to_string() },
                }
            }
            
            DaemonCommand::StartAllBeacons => {
                match emulator.start_all_beacons().await {
                    Ok(ids) => DaemonResponse::BeaconsStarted { ids },
                    Err(e) => DaemonResponse::Error { message: e.to_string() },
                }
            }
            
            DaemonCommand::StopBeacon { id, remove } => {
                match emulator.stop_beacon(id).await {
                    Ok(()) => {
                        if remove {
                            match emulator.remove_beacon(id).await {
                                Ok(()) => DaemonResponse::Success { 
                                    message: Some(format!("Stopped and removed beacon {}", id)) 
                                },
                                Err(e) => DaemonResponse::Error { message: e.to_string() },
                            }
                        } else {
                            DaemonResponse::Success { 
                                message: Some(format!("Stopped beacon {}", id)) 
                            }
                        }
                    }
                    Err(e) => DaemonResponse::Error { message: e.to_string() },
                }
            }
            
            DaemonCommand::StopAllBeacons { remove } => {
                match emulator.stop_all_beacons().await {
                    Ok(ids) => {
                        if remove {
                            let mut removed_count = 0;
                            for beacon_id in &ids {
                                if emulator.remove_beacon(*beacon_id).await.is_ok() {
                                    removed_count += 1;
                                }
                            }
                            DaemonResponse::Success { 
                                message: Some(format!("Stopped {} beacon(s) and removed {} beacon(s)", ids.len(), removed_count)) 
                            }
                        } else {
                            DaemonResponse::BeaconsStopped { ids }
                        }
                    }
                    Err(e) => DaemonResponse::Error { message: e.to_string() },
                }
            }
            
            DaemonCommand::RemoveBeacon { id, force } => {
                if force {
                    let status = emulator.get_beacon_status(id);
                    if let Ok(status) = status {
                        if status.is_running {
                            if let Err(e) = emulator.stop_beacon(id).await {
                                return DaemonResponse::Error { message: e.to_string() };
                            }
                        }
                    }
                }
                
                match emulator.remove_beacon(id).await {
                    Ok(()) => DaemonResponse::Success { 
                        message: Some(format!("Removed beacon {}", id)) 
                    },
                    Err(e) => DaemonResponse::Error { message: e.to_string() },
                }
            }
            
            DaemonCommand::UpdateBeacon { id, position, interval, movement, restart } => {
                match Self::handle_update_beacon(&mut emulator, id, position, interval, movement, restart).await {
                    Ok(message) => DaemonResponse::Success { message: Some(message) },
                    Err(e) => DaemonResponse::Error { message: e.to_string() },
                }
            }
            
            DaemonCommand::ListBeacons { detailed: _, running_only } => {
                let mut beacons = emulator.list_beacons();
                if running_only {
                    beacons.retain(|beacon| beacon.is_running);
                }
                DaemonResponse::BeaconList { beacons }
            }
            
            DaemonCommand::GetStatus { detailed: _ } => {
                let stats = emulator.get_manager_stats();
                DaemonResponse::Status { stats }
            }
            
            DaemonCommand::CreateScenario { scenario_type, count, spacing, center, start_all, interval: _ } => {
                match emulator.create_scenario(scenario_type.clone(), count, spacing, center).await {
                    Ok(beacon_ids) => {
                        if start_all {
                            let mut started_count = 0;
                            for beacon_id in &beacon_ids {
                                if emulator.start_beacon(*beacon_id).await.is_ok() {
                                    started_count += 1;
                                }
                            }
                            DaemonResponse::Success { 
                                message: Some(format!("Created {} scenario with {} beacons, started {}", 
                                    scenario_type, beacon_ids.len(), started_count)) 
                            }
                        } else {
                            DaemonResponse::Success { 
                                message: Some(format!("Created {} scenario with {} beacons", 
                                    scenario_type, beacon_ids.len())) 
                            }
                        }
                    }
                    Err(e) => DaemonResponse::Error { message: e.to_string() },
                }
            }
            
            DaemonCommand::ExportLogs { output, format, duration, all_time, beacon, include_messages } => {
                match Self::handle_export_logs(&emulator, output, format, duration, all_time, beacon, include_messages).await {
                    Ok(message) => DaemonResponse::Success { message: Some(message) },
                    Err(e) => DaemonResponse::Error { message: e.to_string() },
                }
            }
            
            DaemonCommand::ClearState { confirm: _ } => {
                match emulator.clear_state().await {
                    Ok(()) => DaemonResponse::Success { 
                        message: Some("Cleared all beacon state".to_string()) 
                    },
                    Err(e) => DaemonResponse::Error { message: e.to_string() },
                }
            }
            
            DaemonCommand::Shutdown => {
                info!("Received shutdown command");
                DaemonResponse::Success { 
                    message: Some("Daemon shutting down".to_string()) 
                }
            }
            
            DaemonCommand::Ping => DaemonResponse::Pong,
        }
    }
    
    async fn handle_create_beacon(
        emulator: &mut EmulatorManager,
        id: Option<uuid::Uuid>,
        position: shared_positioning::GeodeticPosition,
        config_path: Option<std::path::PathBuf>,
        interval: u32,
        movement: crate::MovementPattern,
        start: bool,
    ) -> Result<uuid::Uuid, EmulatorError> {
        use crate::movement::MovementPatternValidator;
        use shared_positioning::BeaconConfig;
        
        // Validate movement pattern
        MovementPatternValidator::validate_pattern(&movement)?;
        
        // Validate position
        MovementPatternValidator::validate_position(position)?;
        
        // Load configuration if provided
        let config = if let Some(config_path) = config_path {
            if !config_path.exists() {
                return Err(EmulatorError::IoError(
                    std::io::Error::new(std::io::ErrorKind::NotFound, "Configuration file not found")
                ));
            }
            Some(emulator.load_beacon_config(&config_path).await?)
        } else {
            // Create default config with custom interval
            let beacon_id = id.unwrap_or_else(uuid::Uuid::new_v4);
            let mut config = BeaconConfig::new(beacon_id);
            config.transmission.interval_ms = interval;
            Some(config)
        };
        
        // Create beacon
        let beacon_id = emulator.create_beacon(id, position, config).await?;
        
        // Set movement pattern
        emulator.update_beacon_movement_pattern(beacon_id, movement).await?;
        
        // Start beacon if requested
        if start {
            emulator.start_beacon(beacon_id).await?;
        }
        
        Ok(beacon_id)
    }
    
    async fn handle_update_beacon(
        emulator: &mut EmulatorManager,
        id: uuid::Uuid,
        position: Option<shared_positioning::GeodeticPosition>,
        interval: Option<u32>,
        movement: Option<crate::MovementPattern>,
        restart: bool,
    ) -> Result<String, EmulatorError> {
        use crate::movement::MovementPatternValidator;
        
        // Check if beacon exists
        let beacon_status = emulator.get_beacon_status(id)?;
        let was_running = beacon_status.is_running;
        
        let mut updates = Vec::new();
        
        // Update position if provided
        if let Some(new_position) = position {
            MovementPatternValidator::validate_position(new_position)?;
            emulator.update_beacon_position(id, new_position).await?;
            updates.push(format!("position to ({:.6}, {:.6}, {:.1}m)", 
                new_position.latitude, new_position.longitude, new_position.depth));
        }
        
        // Update movement pattern if provided
        if let Some(new_movement) = movement {
            MovementPatternValidator::validate_pattern(&new_movement)?;
            emulator.update_beacon_movement_pattern(id, new_movement.clone()).await?;
            updates.push(format!("movement pattern to: {}", new_movement));
        }
        
        // Update interval if provided (requires restart)
        if let Some(new_interval) = interval {
            if new_interval < 100 || new_interval > 300_000 {
                return Err(EmulatorError::ConfigError(
                    "Interval must be between 100 and 300000 milliseconds".to_string()
                ));
            }
            
            // Get current config and update interval
            let mut config = beacon_status.config;
            config.transmission.interval_ms = new_interval;
            emulator.update_beacon_config(id, config).await?;
            
            updates.push(format!("transmission interval to {}ms", new_interval));
            
            if was_running && !restart {
                updates.push("Note: Beacon must be restarted for interval changes to take effect".to_string());
            }
        }
        
        // Restart beacon if requested and it was running
        if restart && was_running {
            emulator.stop_beacon(id).await?;
            tokio::time::sleep(tokio::time::Duration::from_millis(100)).await; // Brief pause
            emulator.start_beacon(id).await?;
            updates.push("restarted beacon".to_string());
        }
        
        Ok(format!("Updated beacon {}: {}", id, updates.join(", ")))
    }
    
    async fn handle_export_logs(
        emulator: &EmulatorManager,
        output: std::path::PathBuf,
        format: crate::ExportFormat,
        duration: u64,
        all_time: bool,
        beacon_id: Option<uuid::Uuid>,
        _include_messages: bool,
    ) -> Result<String, EmulatorError> {
        let _actual_duration = if all_time { u64::MAX } else { duration };
        
        // For now, just export beacon status since log export is not yet implemented
        let beacons = if let Some(id) = beacon_id {
            vec![emulator.get_beacon_status(id)?]
        } else {
            emulator.list_beacons()
        };
        
        match format {
            crate::ExportFormat::Json => {
                let json = serde_json::to_string_pretty(&beacons)
                    .map_err(|e| EmulatorError::SerializationError(e))?;
                tokio::fs::write(&output, json).await?;
            }
            crate::ExportFormat::Csv => {
                let mut csv_content = String::new();
                csv_content.push_str("id,status,latitude,longitude,depth,movement,messages_sent,interval_ms\n");
                
                for beacon in &beacons {
                    csv_content.push_str(&format!(
                        "{},{},{},{},{},{},{},{}\n",
                        beacon.id,
                        if beacon.is_running { "running" } else { "stopped" },
                        beacon.position.latitude,
                        beacon.position.longitude,
                        beacon.position.depth,
                        beacon.movement_pattern,
                        beacon.stats.messages_sent,
                        beacon.config.transmission.interval_ms
                    ));
                }
                
                tokio::fs::write(&output, csv_content).await?;
            }
        }
        
        let message = format!("Exported {} beacon(s) to {} (format: {})", 
            beacons.len(), output.display(), format);
        
        if !all_time {
            warn!("Full log export with message history not yet implemented. Currently exporting beacon status only.");
        }
        
        Ok(message)
    }
}

impl Drop for DaemonServer {
    fn drop(&mut self) {
        // Clean up socket file
        let socket_path = get_socket_path();
        if socket_path.exists() {
            let _ = std::fs::remove_file(&socket_path);
        }
    }
}