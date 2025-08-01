use serde::{Deserialize, Serialize};
use std::path::PathBuf;
use uuid::Uuid;
use shared_positioning::GeodeticPosition;
use crate::{MovementPattern, VirtualBeaconStatus, EmulatorManagerStats, ScenarioType, ExportFormat};

/// Commands that can be sent to the daemon
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum DaemonCommand {
    /// Create a new beacon
    CreateBeacon {
        id: Option<Uuid>,
        position: GeodeticPosition,
        config_path: Option<PathBuf>,
        interval: u32,
        movement: MovementPattern,
        start: bool,
    },
    
    /// Start a beacon
    StartBeacon { id: Uuid },
    
    /// Start all beacons
    StartAllBeacons,
    
    /// Stop a beacon
    StopBeacon { id: Uuid, remove: bool },
    
    /// Stop all beacons
    StopAllBeacons { remove: bool },
    
    /// Remove a beacon
    RemoveBeacon { id: Uuid, force: bool },
    
    /// Update beacon configuration
    UpdateBeacon {
        id: Uuid,
        position: Option<GeodeticPosition>,
        interval: Option<u32>,
        movement: Option<MovementPattern>,
        restart: bool,
    },
    
    /// List all beacons
    ListBeacons { detailed: bool, running_only: bool },
    
    /// Get emulator status
    GetStatus { detailed: bool },
    
    /// Create a scenario
    CreateScenario {
        scenario_type: ScenarioType,
        count: usize,
        spacing: f64,
        center: GeodeticPosition,
        start_all: bool,
        interval: u32,
    },
    
    /// Export logs
    ExportLogs {
        output: PathBuf,
        format: ExportFormat,
        duration: u64,
        all_time: bool,
        beacon: Option<Uuid>,
        include_messages: bool,
    },
    
    /// Clear all state
    ClearState { confirm: bool },
    
    /// Shutdown daemon
    Shutdown,
    
    /// Ping daemon (health check)
    Ping,
}

/// Responses from the daemon
#[derive(Serialize, Deserialize, Debug, Clone)]
pub enum DaemonResponse {
    /// Success with optional message
    Success { message: Option<String> },
    
    /// Success with beacon ID
    BeaconCreated { id: Uuid },
    
    /// Success with multiple beacon IDs
    BeaconsStarted { ids: Vec<Uuid> },
    
    /// Success with multiple beacon IDs
    BeaconsStopped { ids: Vec<Uuid> },
    
    /// Beacon list response
    BeaconList { beacons: Vec<VirtualBeaconStatus> },
    
    /// Status response
    Status { stats: EmulatorManagerStats },
    
    /// Error response
    Error { message: String },
    
    /// Pong response
    Pong,
}

/// Get the default socket path for daemon communication
pub fn get_socket_path() -> PathBuf {
    std::env::temp_dir().join("beacon-emulator.sock")
}

/// Check if daemon is running by trying to connect to socket
pub async fn is_daemon_running() -> bool {
    let socket_path = get_socket_path();
    if !socket_path.exists() {
        return false;
    }
    
    match tokio::net::UnixStream::connect(&socket_path).await {
        Ok(_) => true,
        Err(_) => {
            // Clean up stale socket file
            let _ = std::fs::remove_file(&socket_path);
            false
        }
    }
}

/// Send a command to the daemon and get response
pub async fn send_daemon_command(command: DaemonCommand) -> Result<DaemonResponse, Box<dyn std::error::Error>> {
    let socket_path = get_socket_path();
    
    if !is_daemon_running().await {
        return Err("Daemon is not running. Start it with 'beacon-emulator daemon'".into());
    }
    
    let mut stream = tokio::net::UnixStream::connect(&socket_path).await?;
    
    // Send command
    let command_json = serde_json::to_string(&command)?;
    let command_bytes = command_json.as_bytes();
    let length = command_bytes.len() as u32;
    
    use tokio::io::{AsyncWriteExt, AsyncReadExt};
    
    // Send length prefix
    stream.write_all(&length.to_le_bytes()).await?;
    // Send command
    stream.write_all(command_bytes).await?;
    stream.flush().await?;
    
    // Read response length
    let mut length_bytes = [0u8; 4];
    stream.read_exact(&mut length_bytes).await?;
    let response_length = u32::from_le_bytes(length_bytes) as usize;
    
    // Read response
    let mut response_bytes = vec![0u8; response_length];
    stream.read_exact(&mut response_bytes).await?;
    
    let response_json = String::from_utf8(response_bytes)?;
    let response: DaemonResponse = serde_json::from_str(&response_json)?;
    
    Ok(response)
}