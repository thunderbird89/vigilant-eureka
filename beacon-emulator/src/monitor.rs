use std::collections::HashMap;
use std::io::{self, Write};
use std::time::{Duration, SystemTime};
use tokio::time::interval;
use uuid::Uuid;
use chrono::{DateTime, Utc};
use crate::{EmulatorManager, EmulatorError, VirtualBeaconStatus};

/// Enhanced monitoring interface for beacon emulator
pub struct BeaconMonitor<'a> {
    emulator: &'a EmulatorManager,
    filter_beacon: Option<Uuid>,
    filter_channel: Option<String>,
    update_interval: Duration,
    compact_mode: bool,
    show_messages: bool,
    show_health: bool,
    show_position_history: bool,
    max_history_entries: usize,
    position_history: HashMap<Uuid, Vec<PositionHistoryEntry>>,
}

#[derive(Debug, Clone)]
struct PositionHistoryEntry {
    timestamp: SystemTime,
    position: shared_positioning::GeodeticPosition,
    messages_sent: u64,
}

#[derive(Debug, Clone)]
pub struct BeaconHealthIndicator {
    pub beacon_id: Uuid,
    pub health_status: HealthStatus,
    pub last_transmission_age: Option<Duration>,
    pub transmission_rate: f64, // messages per second
    pub position_drift: Option<f64>, // meters from initial position
    pub error_rate: f64, // transmission failures per total attempts
}

#[derive(Debug, Clone, PartialEq)]
pub enum HealthStatus {
    Healthy,
    Warning,
    Critical,
    Offline,
}

impl<'a> BeaconMonitor<'a> {
    pub fn new(emulator: &'a EmulatorManager) -> Self {
        Self {
            emulator,
            filter_beacon: None,
            filter_channel: None,
            update_interval: Duration::from_secs(1),
            compact_mode: false,
            show_messages: false,
            show_health: true,
            show_position_history: false,
            max_history_entries: 100,
            position_history: HashMap::new(),
        }
    }

    pub fn with_beacon_filter(mut self, beacon_id: Uuid) -> Self {
        self.filter_beacon = Some(beacon_id);
        self
    }

    pub fn with_channel_filter(mut self, channel: String) -> Self {
        self.filter_channel = Some(channel);
        self
    }

    pub fn with_update_interval(mut self, interval: Duration) -> Self {
        self.update_interval = interval;
        self
    }

    pub fn with_compact_mode(mut self, compact: bool) -> Self {
        self.compact_mode = compact;
        self
    }

    pub fn with_message_display(mut self, show_messages: bool) -> Self {
        self.show_messages = show_messages;
        self
    }

    pub fn with_health_indicators(mut self, show_health: bool) -> Self {
        self.show_health = show_health;
        self
    }

    pub fn with_position_history(mut self, show_history: bool) -> Self {
        self.show_position_history = show_history;
        self
    }

    /// Start the monitoring loop
    pub async fn start_monitoring(&mut self) -> Result<(), EmulatorError> {
        println!("Enhanced Beacon Monitor Starting...");
        println!("Press Ctrl+C to exit");
        
        if let Some(beacon_id) = self.filter_beacon {
            println!("Filtering: Beacon {}", beacon_id);
        }
        
        if let Some(channel) = &self.filter_channel {
            println!("Filtering: Channel '{}'", channel);
        }
        
        println!("Update interval: {:.1}s", self.update_interval.as_secs_f64());
        println!("{}", "=".repeat(80));

        // Set up Ctrl+C handler
        let (tx, mut rx) = tokio::sync::mpsc::channel(1);
        tokio::spawn(async move {
            tokio::signal::ctrl_c().await.expect("Failed to listen for Ctrl+C");
            let _ = tx.send(()).await;
        });

        let mut interval_timer = interval(self.update_interval);
        let mut last_update = SystemTime::now();

        loop {
            tokio::select! {
                _ = interval_timer.tick() => {
                    let current_time = SystemTime::now();
                    
                    // Clear screen and move cursor to top
                    print!("\x1B[2J\x1B[1;1H");
                    
                    // Update display
                    if let Err(e) = self.update_display(current_time, last_update).await {
                        eprintln!("Monitor error: {}", e);
                        break;
                    }
                    
                    last_update = current_time;
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

    async fn update_display(&mut self, current_time: SystemTime, _last_update: SystemTime) -> Result<(), EmulatorError> {
        // Get beacon data
        let beacons = self.get_filtered_beacons().await?;
        
        // Update position history
        self.update_position_history(&beacons, current_time);
        
        // Display header
        self.display_header(current_time, &beacons).await?;
        
        if beacons.is_empty() {
            println!("No beacons match the current filter criteria.");
            return Ok(());
        }

        // Display beacon information
        if self.compact_mode {
            self.display_compact_view(&beacons, current_time).await?;
        } else {
            self.display_detailed_view(&beacons, current_time).await?;
        }

        // Display channel statistics if not filtering by beacon
        if self.filter_beacon.is_none() {
            self.display_channel_stats().await?;
        }

        Ok(())
    }

    async fn get_filtered_beacons(&self) -> Result<Vec<VirtualBeaconStatus>, EmulatorError> {
        let beacons = if let Some(beacon_id) = self.filter_beacon {
            vec![self.emulator.get_beacon_status(beacon_id)?]
        } else {
            self.emulator.list_beacons()
        };

        // Filter by channel if specified
        if let Some(_channel) = &self.filter_channel {
            // Note: Channel filtering would require additional implementation
            // in the emulator to track which beacons are on which channels
            // For now, we'll show all beacons as the current implementation
            // doesn't separate beacons by channel in the status
        }

        Ok(beacons)
    }

    fn update_position_history(&mut self, beacons: &[VirtualBeaconStatus], timestamp: SystemTime) {
        for beacon in beacons {
            let history = self.position_history.entry(beacon.id).or_insert_with(Vec::new);
            
            history.push(PositionHistoryEntry {
                timestamp,
                position: beacon.position,
                messages_sent: beacon.stats.messages_sent,
            });

            // Keep only recent entries
            if history.len() > self.max_history_entries {
                history.drain(0..history.len() - self.max_history_entries);
            }
        }
    }

    async fn display_header(&self, current_time: SystemTime, beacons: &[VirtualBeaconStatus]) -> Result<(), EmulatorError> {
        let stats = self.emulator.get_manager_stats().await;
        let timestamp: DateTime<Utc> = current_time.into();
        
        println!("┌─ Enhanced Beacon Monitor ─────────────────────────────────────────────────┐");
        println!("│ Time: {} │ Channel: {} │", 
                 timestamp.format("%Y-%m-%d %H:%M:%S UTC"), 
                 stats.current_channel);
        println!("│ Total: {} │ Running: {} │ Stopped: {} │ Filtered: {} │",
                 stats.total_beacons,
                 stats.running_beacons,
                 stats.stopped_beacons,
                 beacons.len());
        println!("└────────────────────────────────────────────────────────────────────────────┘");
        println!();

        Ok(())
    }

    async fn display_compact_view(&self, beacons: &[VirtualBeaconStatus], current_time: SystemTime) -> Result<(), EmulatorError> {
        println!("┌─ Compact View ─────────────────────────────────────────────────────────────┐");
        println!("│{:<8} │{:<36} │{:<8} │{:<10} │{:<12}│", 
                 "Status", "Beacon ID", "Messages", "Last TX", "Health");
        println!("├─────────┼────────────────────────────────────────┼─────────┼───────────┼────────────┤");
        
        for beacon in beacons {
            let status_icon = if beacon.is_running { "●" } else { "○" };
            let status_color = if beacon.is_running { "\x1b[32m" } else { "\x1b[31m" }; // Green/Red
            let reset_color = "\x1b[0m";
            
            let last_tx = beacon.stats.last_transmission
                .and_then(|t| current_time.duration_since(t).ok())
                .map(|d| format!("{:.1}s", d.as_secs_f64()))
                .unwrap_or_else(|| "Never".to_string());
            
            let health = self.calculate_health_indicator(beacon, current_time);
            let health_icon = match health.health_status {
                HealthStatus::Healthy => "✓",
                HealthStatus::Warning => "⚠",
                HealthStatus::Critical => "✗",
                HealthStatus::Offline => "○",
            };
            let health_color = match health.health_status {
                HealthStatus::Healthy => "\x1b[32m", // Green
                HealthStatus::Warning => "\x1b[33m", // Yellow
                HealthStatus::Critical => "\x1b[31m", // Red
                HealthStatus::Offline => "\x1b[90m", // Gray
            };
            
            println!("│{}{:<8}{} │{:<36} │{:<8} │{:<10} │{}{:<12}{}│",
                     status_color, status_icon, reset_color,
                     beacon.id,
                     beacon.stats.messages_sent,
                     last_tx,
                     health_color, health_icon, reset_color);
        }
        
        println!("└─────────┴────────────────────────────────────────┴─────────┴───────────┴────────────┘");
        println!();

        Ok(())
    }

    async fn display_detailed_view(&self, beacons: &[VirtualBeaconStatus], current_time: SystemTime) -> Result<(), EmulatorError> {
        println!("┌─ Detailed View ────────────────────────────────────────────────────────────┐");
        
        for (i, beacon) in beacons.iter().enumerate() {
            if i > 0 {
                println!("├────────────────────────────────────────────────────────────────────────────┤");
            }
            
            let status_text = if beacon.is_running { "RUNNING" } else { "STOPPED" };
            let status_color = if beacon.is_running { "\x1b[32m" } else { "\x1b[31m" };
            let reset_color = "\x1b[0m";
            
            println!("│ Beacon: {} │", beacon.id);
            println!("│ Status: {}{}{} │ Messages: {} │ Interval: {}ms │",
                     status_color, status_text, reset_color,
                     beacon.stats.messages_sent,
                     beacon.config.transmission.interval_ms);
            
            println!("│ Position: {:.6}°, {:.6}°, {:.1}m │",
                     beacon.position.latitude,
                     beacon.position.longitude,
                     beacon.position.depth);
            
            println!("│ Movement: {} │", beacon.movement_pattern);
            
            // Health indicators
            if self.show_health {
                let health = self.calculate_health_indicator(beacon, current_time);
                let health_color = match health.health_status {
                    HealthStatus::Healthy => "\x1b[32m",
                    HealthStatus::Warning => "\x1b[33m",
                    HealthStatus::Critical => "\x1b[31m",
                    HealthStatus::Offline => "\x1b[90m",
                };
                
                println!("│ Health: {}{:?}{} │ Rate: {:.2} msg/s │ Errors: {:.1}% │",
                         health_color, health.health_status, reset_color,
                         health.transmission_rate,
                         health.error_rate * 100.0);
                
                if let Some(drift) = health.position_drift {
                    println!("│ Position drift: {:.1}m from initial │", drift);
                }
            }
            
            // Last transmission info
            if let Some(last_tx) = beacon.stats.last_transmission {
                if let Ok(elapsed) = current_time.duration_since(last_tx) {
                    println!("│ Last transmission: {:.1}s ago │", elapsed.as_secs_f64());
                }
            } else {
                println!("│ Last transmission: Never │");
            }
            
            // Position history
            if self.show_position_history {
                if let Some(history) = self.position_history.get(&beacon.id) {
                    if history.len() > 1 {
                        let recent_entries = history.iter().rev().take(5).collect::<Vec<_>>();
                        println!("│ Recent positions: │");
                        for entry in recent_entries {
                            let timestamp: DateTime<Utc> = entry.timestamp.into();
                            println!("│   {}: {:.6}°, {:.6}° │",
                                     timestamp.format("%H:%M:%S"),
                                     entry.position.latitude,
                                     entry.position.longitude);
                        }
                    }
                }
            }
            
            // Recent messages if enabled
            if self.show_messages {
                // This would require additional implementation to track actual message content
                println!("│ Message display not yet implemented │");
            }
        }
        
        println!("└────────────────────────────────────────────────────────────────────────────┘");
        println!();

        Ok(())
    }

    async fn display_channel_stats(&self) -> Result<(), EmulatorError> {
        // Get channel statistics
        let comm_space_arc = self.emulator.get_communication_space();
        let comm_space = comm_space_arc.read().await;
        let channel_stats = comm_space.get_all_channel_stats().await;
        
        if !channel_stats.is_empty() {
            println!("┌─ Channel Statistics ───────────────────────────────────────────────────────┐");
            println!("│{:<15} │{:<10} │{:<10} │{:<15} │{:<15}│",
                     "Channel", "Messages", "Beacons", "Oldest", "Newest");
            println!("├────────────────┼───────────┼───────────┼────────────────┼────────────────┤");
            
            for stats in channel_stats {
                let oldest = stats.oldest_message
                    .map(|t| {
                        let dt: DateTime<Utc> = t.into();
                        dt.format("%H:%M:%S").to_string()
                    })
                    .unwrap_or_else(|| "N/A".to_string());
                
                let newest = stats.newest_message
                    .map(|t| {
                        let dt: DateTime<Utc> = t.into();
                        dt.format("%H:%M:%S").to_string()
                    })
                    .unwrap_or_else(|| "N/A".to_string());
                
                println!("│{:<15} │{:<10} │{:<10} │{:<15} │{:<15}│",
                         stats.name,
                         stats.total_messages,
                         stats.unique_beacons,
                         oldest,
                         newest);
            }
            
            println!("└────────────────┴───────────┴───────────┴────────────────┴────────────────┘");
            println!();
        }

        Ok(())
    }

    fn calculate_health_indicator(&self, beacon: &VirtualBeaconStatus, current_time: SystemTime) -> BeaconHealthIndicator {
        let last_transmission_age = beacon.stats.last_transmission
            .and_then(|t| current_time.duration_since(t).ok());
        
        // Calculate transmission rate (messages per second)
        let transmission_rate = if beacon.stats.uptime.as_secs() > 0 {
            beacon.stats.messages_sent as f64 / beacon.stats.uptime.as_secs_f64()
        } else {
            0.0
        };
        
        // Calculate error rate
        let total_attempts = beacon.stats.messages_sent + beacon.stats.transmission_failures as u64;
        let error_rate = if total_attempts > 0 {
            beacon.stats.transmission_failures as f64 / total_attempts as f64
        } else {
            0.0
        };
        
        // Calculate position drift if we have history
        let position_drift = self.position_history.get(&beacon.id)
            .and_then(|history| history.first())
            .map(|initial| {
                let lat_diff = beacon.position.latitude - initial.position.latitude;
                let lon_diff = beacon.position.longitude - initial.position.longitude;
                let depth_diff = beacon.position.depth - initial.position.depth;
                
                // Simple distance calculation (not accounting for Earth's curvature)
                let horizontal_distance = ((lat_diff * 111_132.0).powi(2) + 
                                         (lon_diff * 111_320.0 * beacon.position.latitude.to_radians().cos()).powi(2)).sqrt();
                (horizontal_distance.powi(2) + depth_diff.powi(2)).sqrt()
            });
        
        // Determine health status
        let health_status = if !beacon.is_running {
            HealthStatus::Offline
        } else if let Some(age) = last_transmission_age {
            let expected_interval = Duration::from_millis(beacon.config.transmission.interval_ms as u64);
            if age > expected_interval * 3 {
                HealthStatus::Critical
            } else if age > expected_interval * 2 {
                HealthStatus::Warning
            } else if error_rate > 0.1 {
                HealthStatus::Warning
            } else {
                HealthStatus::Healthy
            }
        } else {
            HealthStatus::Critical
        };
        
        BeaconHealthIndicator {
            beacon_id: beacon.id,
            health_status,
            last_transmission_age,
            transmission_rate,
            position_drift,
            error_rate,
        }
    }
}

/// Enhanced monitoring configuration
#[derive(Debug, Clone)]
pub struct MonitorConfig {
    pub beacon_filter: Option<Uuid>,
    pub channel_filter: Option<String>,
    pub update_interval_secs: u64,
    pub compact_mode: bool,
    pub show_messages: bool,
    pub show_health: bool,
    pub show_position_history: bool,
    pub max_history_entries: usize,
}

impl Default for MonitorConfig {
    fn default() -> Self {
        Self {
            beacon_filter: None,
            channel_filter: None,
            update_interval_secs: 1,
            compact_mode: false,
            show_messages: false,
            show_health: true,
            show_position_history: false,
            max_history_entries: 100,
        }
    }
}

impl MonitorConfig {
    pub fn from_cli_args(
        beacon: Option<Uuid>,
        interval: u64,
        compact: bool,
        show_messages: bool,
    ) -> Self {
        Self {
            beacon_filter: beacon,
            update_interval_secs: interval,
            compact_mode: compact,
            show_messages,
            show_health: true,
            show_position_history: !compact, // Show history in detailed mode
            ..Default::default()
        }
    }
}