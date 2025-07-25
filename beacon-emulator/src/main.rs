use beacon_emulator::{
    cli::{Cli, EmulatorCommand, ListFormat},
    EmulatorManager, EmulatorError, MovementPattern, MovementPatternValidator
};
use clap::Parser;
use shared_positioning::{GeodeticPosition, BeaconConfig};
use std::io::{self, Write};
use std::time::{Duration, SystemTime};
use tokio::time::sleep;
use tracing::{info, error, warn, debug};
use tracing_subscriber::{EnvFilter, fmt};
use uuid::Uuid;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Parse CLI arguments first to get log level
    let cli = Cli::parse();
    
    // Initialize logging with specified level
    let log_level = match cli.log_level.to_lowercase().as_str() {
        "trace" => "trace",
        "debug" => "debug", 
        "info" => "info",
        "warn" => "warn",
        "error" => "error",
        _ => {
            eprintln!("Invalid log level '{}', using 'info'", cli.log_level);
            "info"
        }
    };
    
    let filter = EnvFilter::try_from_default_env()
        .unwrap_or_else(|_| EnvFilter::new(log_level));
    
    fmt()
        .with_env_filter(filter)
        .with_target(false)
        .init();

    info!("Starting beacon emulator on channel: {}", cli.channel);
    
    // Create emulator manager
    let mut emulator = EmulatorManager::new(&cli.channel);
    
    // Execute command with proper error handling
    match execute_command(&mut emulator, cli.command).await {
        Ok(()) => {
            debug!("Command completed successfully");
        }
        Err(e) => {
            error!("Command failed: {}", e);
            
            // Provide helpful error messages for common issues
            match &e {
                EmulatorError::BeaconNotFound(id) => {
                    eprintln!("Beacon {} not found. Use 'list' command to see available beacons.", id);
                }
                EmulatorError::BeaconExists(id) => {
                    eprintln!("Beacon {} already exists. Use a different ID or update the existing beacon.", id);
                }
                EmulatorError::ConfigError(msg) => {
                    eprintln!("Configuration error: {}", msg);
                    eprintln!("Check your configuration file format and parameter values.");
                }
                EmulatorError::MovementError(msg) => {
                    eprintln!("Movement pattern error: {}", msg);
                    eprintln!("Valid patterns: stationary, linear:speed:bearing, circular:radius:period, random:max_speed");
                }
                EmulatorError::InvalidScenario(msg) => {
                    eprintln!("Scenario error: {}", msg);
                    eprintln!("Valid scenarios: triangle, square, line, grid");
                }
                _ => {
                    eprintln!("Error: {}", e);
                }
            }
            
            std::process::exit(1);
        }
    }
    
    // Graceful shutdown
    debug!("Shutting down emulator");
    if let Err(e) = emulator.shutdown().await {
        warn!("Error during shutdown: {}", e);
    }
    
    Ok(())
}

async fn execute_command(
    emulator: &mut EmulatorManager,
    command: EmulatorCommand,
) -> Result<(), EmulatorError> {
    match command {
        EmulatorCommand::Create {
            id,
            lat,
            lon,
            depth,
            config,
            interval,
            version: _version, // TODO: Use version when implementing message building
            movement,
            start,
        } => {
            execute_create_command(emulator, id, lat, lon, depth, config, interval, movement, start).await
        }
        
        EmulatorCommand::List { detailed, running_only, format } => {
            execute_list_command(emulator, detailed, running_only, format).await
        }
        
        EmulatorCommand::Stop { id, remove } => {
            execute_stop_command(emulator, id, remove).await
        }
        
        EmulatorCommand::StopAll { remove } => {
            execute_stop_all_command(emulator, remove).await
        }
        
        EmulatorCommand::Update { id, position, interval, movement, restart } => {
            execute_update_command(emulator, id, position, interval, movement, restart).await
        }
        
        EmulatorCommand::Start { id, all } => {
            execute_start_command(emulator, id, all).await
        }
        
        EmulatorCommand::Remove { id, force } => {
            execute_remove_command(emulator, id, force).await
        }
        
        EmulatorCommand::Scenario { scenario_type, count, spacing, center, start_all, interval } => {
            execute_scenario_command(emulator, scenario_type, count, spacing, center, start_all, interval).await
        }
        
        EmulatorCommand::Monitor { beacon, interval, compact, show_messages } => {
            execute_monitor_command(emulator, beacon, interval, compact, show_messages).await
        }
        
        EmulatorCommand::Export { output, format, duration, all_time, beacon, include_messages } => {
            execute_export_command(emulator, output, format, duration, all_time, beacon, include_messages).await
        }
        
        EmulatorCommand::Status { detailed } => {
            execute_status_command(emulator, detailed).await
        }
    }
}

async fn execute_create_command(
    emulator: &mut EmulatorManager,
    id: Option<Uuid>,
    lat: f64,
    lon: f64,
    depth: f64,
    config_path: Option<std::path::PathBuf>,
    interval: u32,
    movement: MovementPattern,
    start: bool,
) -> Result<(), EmulatorError> {
    // Validate movement pattern
    MovementPatternValidator::validate_pattern(&movement)?;
    
    // Create position
    let position = GeodeticPosition {
        latitude: lat,
        longitude: lon,
        depth,
    };
    
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
        let beacon_id = id.unwrap_or_else(Uuid::new_v4);
        let mut config = BeaconConfig::new(beacon_id);
        config.transmission.interval_ms = interval;
        Some(config)
    };
    
    // Create beacon
    let beacon_id = emulator.create_beacon(id, position, config).await?;
    
    // Set movement pattern
    emulator.update_beacon_movement_pattern(beacon_id, movement.clone()).await?;
    
    println!("Created beacon {} at position ({:.6}, {:.6}, {:.1}m)", 
             beacon_id, lat, lon, depth);
    println!("Movement pattern: {}", movement);
    
    // Start beacon if requested
    if start {
        emulator.start_beacon(beacon_id).await?;
        println!("Started beacon {}", beacon_id);
    } else {
        println!("Beacon created but not started. Use 'start {}' to begin transmission.", beacon_id);
    }
    
    Ok(())
}

async fn execute_list_command(
    emulator: &EmulatorManager,
    detailed: bool,
    running_only: bool,
    format: ListFormat,
) -> Result<(), EmulatorError> {
    let mut beacons = emulator.list_beacons();
    
    if running_only {
        beacons.retain(|beacon| beacon.is_running);
    }
    
    if beacons.is_empty() {
        if running_only {
            println!("No running beacons found.");
        } else {
            println!("No beacons found.");
        }
        return Ok(());
    }
    
    match format {
        ListFormat::Table => {
            print_beacons_table(&beacons, detailed);
        }
        ListFormat::Json => {
            let json = serde_json::to_string_pretty(&beacons)?;
            println!("{}", json);
        }
        ListFormat::Csv => {
            print_beacons_csv(&beacons, detailed)?;
        }
    }
    
    Ok(())
}

fn print_beacons_table(beacons: &[beacon_emulator::VirtualBeaconStatus], detailed: bool) {
    if detailed {
        for beacon in beacons {
            println!("Beacon ID: {}", beacon.id);
            println!("  Status: {}", if beacon.is_running { "Running" } else { "Stopped" });
            println!("  Position: {:.6}, {:.6}, {:.1}m", 
                     beacon.position.latitude, beacon.position.longitude, beacon.position.depth);
            println!("  Movement: {}", beacon.movement_pattern);
            println!("  Messages sent: {}", beacon.stats.messages_sent);
            println!("  Interval: {}ms", beacon.config.transmission.interval_ms);
            if let Some(last_tx) = beacon.stats.last_transmission {
                if let Ok(elapsed) = last_tx.elapsed() {
                    println!("  Last transmission: {:.1}s ago", elapsed.as_secs_f64());
                }
            }
            println!();
        }
    } else {
        println!("{:<36} {:<8} {:<20} {:<30} {:<10}", 
                 "ID", "Status", "Position", "Movement", "Messages");
        println!("{}", "-".repeat(110));
        
        for beacon in beacons {
            let status = if beacon.is_running { "Running" } else { "Stopped" };
            let position = format!("{:.3},{:.3},{:.0}m", 
                                 beacon.position.latitude, beacon.position.longitude, beacon.position.depth);
            let movement = format!("{}", beacon.movement_pattern);
            let movement_truncated = if movement.len() > 28 {
                format!("{}...", &movement[..25])
            } else {
                movement
            };
            
            println!("{:<36} {:<8} {:<20} {:<30} {:<10}", 
                     beacon.id, status, position, movement_truncated, beacon.stats.messages_sent);
        }
    }
}

fn print_beacons_csv(beacons: &[beacon_emulator::VirtualBeaconStatus], detailed: bool) -> Result<(), EmulatorError> {
    if detailed {
        println!("id,status,latitude,longitude,depth,movement,messages_sent,interval_ms,last_transmission");
        for beacon in beacons {
            let last_tx = beacon.stats.last_transmission
                .and_then(|t| t.duration_since(SystemTime::UNIX_EPOCH).ok())
                .map(|d| d.as_secs().to_string())
                .unwrap_or_else(|| "".to_string());
            
            println!("{},{},{},{},{},{},{},{},{}",
                     beacon.id,
                     if beacon.is_running { "running" } else { "stopped" },
                     beacon.position.latitude,
                     beacon.position.longitude,
                     beacon.position.depth,
                     beacon.movement_pattern,
                     beacon.stats.messages_sent,
                     beacon.config.transmission.interval_ms,
                     last_tx);
        }
    } else {
        println!("id,status,latitude,longitude,depth,messages_sent");
        for beacon in beacons {
            println!("{},{},{},{},{},{}",
                     beacon.id,
                     if beacon.is_running { "running" } else { "stopped" },
                     beacon.position.latitude,
                     beacon.position.longitude,
                     beacon.position.depth,
                     beacon.stats.messages_sent);
        }
    }
    Ok(())
}

async fn execute_stop_command(
    emulator: &mut EmulatorManager,
    id: Uuid,
    remove: bool,
) -> Result<(), EmulatorError> {
    emulator.stop_beacon(id).await?;
    println!("Stopped beacon {}", id);
    
    if remove {
        emulator.remove_beacon(id).await?;
        println!("Removed beacon {}", id);
    }
    
    Ok(())
}

async fn execute_stop_all_command(
    emulator: &mut EmulatorManager,
    remove: bool,
) -> Result<(), EmulatorError> {
    let stopped_beacons = emulator.stop_all_beacons().await?;
    
    if stopped_beacons.is_empty() {
        println!("No running beacons to stop.");
    } else {
        println!("Stopped {} beacon(s): {:?}", stopped_beacons.len(), stopped_beacons);
    }
    
    if remove {
        let mut removed_count = 0;
        for beacon_id in stopped_beacons {
            if emulator.remove_beacon(beacon_id).await.is_ok() {
                removed_count += 1;
            }
        }
        println!("Removed {} beacon(s)", removed_count);
    }
    
    Ok(())
}

async fn execute_update_command(
    emulator: &mut EmulatorManager,
    id: Uuid,
    position: Option<GeodeticPosition>,
    interval: Option<u32>,
    movement: Option<MovementPattern>,
    restart: bool,
) -> Result<(), EmulatorError> {
    // Check if beacon exists
    let beacon_status = emulator.get_beacon_status(id)?;
    let was_running = beacon_status.is_running;
    
    // Update position if provided
    if let Some(new_position) = position {
        MovementPatternValidator::validate_position(new_position)?;
        emulator.update_beacon_position(id, new_position).await?;
        println!("Updated position for beacon {} to ({:.6}, {:.6}, {:.1}m)", 
                 id, new_position.latitude, new_position.longitude, new_position.depth);
    }
    
    // Update movement pattern if provided
    if let Some(new_movement) = movement {
        MovementPatternValidator::validate_pattern(&new_movement)?;
        emulator.update_beacon_movement_pattern(id, new_movement.clone()).await?;
        println!("Updated movement pattern for beacon {} to: {}", id, new_movement);
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
        
        println!("Updated transmission interval for beacon {} to {}ms", id, new_interval);
        
        if was_running && !restart {
            println!("Note: Beacon must be restarted for interval changes to take effect. Use --restart flag.");
        }
    }
    
    // Restart beacon if requested and it was running
    if restart && was_running {
        emulator.stop_beacon(id).await?;
        sleep(Duration::from_millis(100)).await; // Brief pause
        emulator.start_beacon(id).await?;
        println!("Restarted beacon {}", id);
    }
    
    Ok(())
}

async fn execute_start_command(
    emulator: &mut EmulatorManager,
    id: Option<Uuid>,
    all: bool,
) -> Result<(), EmulatorError> {
    if all {
        let started_beacons = emulator.start_all_beacons().await?;
        if started_beacons.is_empty() {
            println!("No stopped beacons to start.");
        } else {
            println!("Started {} beacon(s): {:?}", started_beacons.len(), started_beacons);
        }
    } else if let Some(beacon_id) = id {
        emulator.start_beacon(beacon_id).await?;
        println!("Started beacon {}", beacon_id);
    } else {
        return Err(EmulatorError::ConfigError(
            "Either specify a beacon ID or use --all flag".to_string()
        ));
    }
    
    Ok(())
}

async fn execute_remove_command(
    emulator: &mut EmulatorManager,
    id: Uuid,
    force: bool,
) -> Result<(), EmulatorError> {
    if force {
        // Stop beacon first if running
        let status = emulator.get_beacon_status(id)?;
        if status.is_running {
            emulator.stop_beacon(id).await?;
            println!("Stopped beacon {} before removal", id);
        }
    }
    
    emulator.remove_beacon(id).await?;
    println!("Removed beacon {}", id);
    
    Ok(())
}

async fn execute_scenario_command(
    emulator: &mut EmulatorManager,
    scenario_type: beacon_emulator::ScenarioType,
    count: usize,
    spacing: f64,
    center: GeodeticPosition,
    start_all: bool,
    _interval: u32,
) -> Result<(), EmulatorError> {
    // Validate parameters
    MovementPatternValidator::validate_position(center)?;
    
    if spacing <= 0.0 || spacing > 10000.0 {
        return Err(EmulatorError::InvalidScenario(
            "Spacing must be between 0 and 10000 meters".to_string()
        ));
    }
    
    // Create scenario
    let beacon_ids = emulator.create_scenario(scenario_type.clone(), count, spacing, center).await?;
    
    println!("Created {} scenario with {} beacons:", scenario_type, beacon_ids.len());
    for (i, beacon_id) in beacon_ids.iter().enumerate() {
        println!("  Beacon {}: {}", i + 1, beacon_id);
    }
    
    // Start all beacons if requested
    if start_all {
        let mut started_count = 0;
        for beacon_id in &beacon_ids {
            if emulator.start_beacon(*beacon_id).await.is_ok() {
                started_count += 1;
            }
        }
        println!("Started {} out of {} beacons", started_count, beacon_ids.len());
    } else {
        println!("Beacons created but not started. Use 'start --all' to begin transmission.");
    }
    
    Ok(())
}

async fn execute_monitor_command(
    emulator: &EmulatorManager,
    beacon_id: Option<Uuid>,
    interval: u64,
    compact: bool,
    show_messages: bool,
) -> Result<(), EmulatorError> {
    println!("Monitoring beacon activity (Press Ctrl+C to exit)");
    println!("Update interval: {}s", interval);
    
    if let Some(id) = beacon_id {
        println!("Monitoring beacon: {}", id);
    } else {
        println!("Monitoring all beacons");
    }
    
    println!("{}", "-".repeat(80));
    
    // Set up Ctrl+C handler
    let (tx, mut rx) = tokio::sync::mpsc::channel(1);
    tokio::spawn(async move {
        tokio::signal::ctrl_c().await.expect("Failed to listen for Ctrl+C");
        let _ = tx.send(()).await;
    });
    
    let mut interval_timer = tokio::time::interval(Duration::from_secs(interval));
    
    loop {
        tokio::select! {
            _ = interval_timer.tick() => {
                // Update display
                print!("\x1B[2J\x1B[1;1H"); // Clear screen and move cursor to top
                
                let beacons = if let Some(id) = beacon_id {
                    match emulator.get_beacon_status(id) {
                        Ok(status) => vec![status],
                        Err(_) => {
                            println!("Beacon {} not found", id);
                            break;
                        }
                    }
                } else {
                    emulator.list_beacons()
                };
                
                if compact {
                    print_compact_monitor(&beacons);
                } else {
                    print_detailed_monitor(&beacons, show_messages);
                }
                
                io::stdout().flush().unwrap();
            }
            _ = rx.recv() => {
                println!("\nMonitoring stopped.");
                break;
            }
        }
    }
    
    Ok(())
}

fn print_compact_monitor(beacons: &[beacon_emulator::VirtualBeaconStatus]) {
    println!("Beacon Monitor - {} beacons", beacons.len());
    println!("{:<8} {:<36} {:<8} {:<10}", "Status", "ID", "Messages", "Last TX");
    println!("{}", "-".repeat(70));
    
    for beacon in beacons {
        let status = if beacon.is_running { "RUN" } else { "STOP" };
        let last_tx = beacon.stats.last_transmission
            .and_then(|t| t.elapsed().ok())
            .map(|d| format!("{:.1}s", d.as_secs_f64()))
            .unwrap_or_else(|| "Never".to_string());
        
        println!("{:<8} {:<36} {:<8} {:<10}", 
                 status, beacon.id, beacon.stats.messages_sent, last_tx);
    }
}

fn print_detailed_monitor(beacons: &[beacon_emulator::VirtualBeaconStatus], _show_messages: bool) {
    println!("Detailed Beacon Monitor - {} beacons", beacons.len());
    println!();
    
    for beacon in beacons {
        println!("Beacon: {}", beacon.id);
        println!("  Status: {}", if beacon.is_running { "Running" } else { "Stopped" });
        println!("  Position: {:.6}, {:.6}, {:.1}m", 
                 beacon.position.latitude, beacon.position.longitude, beacon.position.depth);
        println!("  Movement: {}", beacon.movement_pattern);
        println!("  Messages: {}", beacon.stats.messages_sent);
        println!("  Interval: {}ms", beacon.config.transmission.interval_ms);
        
        if let Some(last_tx) = beacon.stats.last_transmission {
            if let Ok(elapsed) = last_tx.elapsed() {
                println!("  Last TX: {:.1}s ago", elapsed.as_secs_f64());
            }
        }
        println!();
    }
}

async fn execute_export_command(
    emulator: &EmulatorManager,
    output: std::path::PathBuf,
    format: beacon_emulator::ExportFormat,
    duration: u64,
    all_time: bool,
    beacon_id: Option<Uuid>,
    _include_messages: bool,
) -> Result<(), EmulatorError> {
    let _actual_duration = if all_time { u64::MAX } else { duration };
    
    // For now, just export beacon status since log export is not yet implemented
    let beacons = if let Some(id) = beacon_id {
        vec![emulator.get_beacon_status(id)?]
    } else {
        emulator.list_beacons()
    };
    
    match format {
        beacon_emulator::ExportFormat::Json => {
            let json = serde_json::to_string_pretty(&beacons)?;
            tokio::fs::write(&output, json).await?;
        }
        beacon_emulator::ExportFormat::Csv => {
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
    
    println!("Exported {} beacon(s) to {} (format: {})", 
             beacons.len(), output.display(), format);
    
    if !all_time {
        println!("Note: Full log export with message history not yet implemented.");
        println!("Currently exporting beacon status only.");
    }
    
    Ok(())
}

async fn execute_status_command(
    emulator: &EmulatorManager,
    detailed: bool,
) -> Result<(), EmulatorError> {
    let stats = emulator.get_manager_stats();
    
    println!("Beacon Emulator Status");
    println!("=====================");
    println!("Channel: {}", stats.current_channel);
    println!("Total beacons: {}", stats.total_beacons);
    println!("Running beacons: {}", stats.running_beacons);
    println!("Stopped beacons: {}", stats.stopped_beacons);
    println!("Active channels: {}", stats.active_channels);
    
    if detailed {
        println!();
        println!("Channel names: {:?}", stats.channel_names);
        
        let beacons = emulator.list_beacons();
        if !beacons.is_empty() {
            println!();
            println!("Beacon Details:");
            for beacon in beacons {
                println!("  {} - {} - {} messages", 
                         beacon.id, 
                         if beacon.is_running { "Running" } else { "Stopped" },
                         beacon.stats.messages_sent);
            }
        }
    }
    
    Ok(())
}