use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use std::path::PathBuf;
use tracing::{info, warn};

use crate::config::ConfigManager;
use crate::beacon_controller::BeaconController;
use shared_positioning::{
    MockGpsManager, MockPowerManager, MockCommunicationManager, MockTransceiver,
};

// Type alias for the concrete beacon controller type
type ConcreteBeaconController = BeaconController<
    MockGpsManager,
    MockPowerManager,
    MockCommunicationManager,
    MockTransceiver,
>;

#[derive(Subcommand)]
pub enum CliCommands {
    /// Show beacon status
    Status(StatusCommand),
    /// Run diagnostic checks
    Diagnostic(DiagnosticCommand),
    /// Validate configuration file
    ValidateConfig,
    /// Generate default configuration file
    GenerateConfig {
        /// Output file path
        #[arg(short, long, default_value = "beacon-default.toml")]
        output: PathBuf,
    },
}

#[derive(Parser)]
pub struct StatusCommand {
    /// Show detailed status information
    #[arg(short, long)]
    pub detailed: bool,
    
    /// Show GPS status
    #[arg(long)]
    pub gps: bool,
    
    /// Show power status
    #[arg(long)]
    pub power: bool,
    
    /// Show communication status
    #[arg(long)]
    pub communication: bool,
    
    /// Show transmission statistics
    #[arg(long)]
    pub transmission: bool,
}

impl StatusCommand {
    pub async fn execute(&self, config_path: &PathBuf) -> Result<()> {
        info!("Executing status command");
        
        // Load configuration and create temporary beacon controller for status
        let config_manager = ConfigManager::new(config_path.clone());
        let config = config_manager.load_config().await
            .context("Failed to load configuration")?;
        
        // Create temporary beacon controller for status checking
        let beacon_controller = create_temp_beacon_controller(config).await
            .context("Failed to create beacon controller for status check")?;
        
        let status = beacon_controller.get_status();
        
        println!("=== BEACON STATUS ===");
        println!("Beacon ID: {}", status.beacon_id);
        println!("Operational State: {:?}", status.operational_state);
        println!("Uptime: {:?}", status.uptime);
        
        if let Some(error) = &status.last_error {
            println!("Last Error: {}", error);
        }
        
        if self.detailed || self.gps {
            println!("\n--- GPS Status ---");
            println!("GPS Status: {:?}", status.gps_status);
            if let Some(pos) = &status.current_position {
                println!("Position: {:.6}, {:.6} (±{:.1}m)", 
                    pos.latitude, pos.longitude, pos.accuracy_m);
                println!("Satellites: {}", pos.satellite_count);
                println!("Last Update: {:?}", pos.timestamp);
            } else {
                println!("Position: Not available");
            }
        }
        
        if self.detailed || self.power {
            println!("\n--- Power Status ---");
            println!("Battery Level: {:.1}%", status.battery_status.capacity_percent);
            println!("Voltage: {:.2}V", status.battery_status.voltage_v);
            println!("Current: {:.0}mA", status.battery_status.current_ma);
            println!("Temperature: {:.1}°C", status.battery_status.temperature_c);
            println!("Health: {:?}", status.battery_status.health);
        }
        
        if self.detailed || self.communication {
            println!("\n--- Communication Status ---");
            println!("Connection Status: {:?}", status.communication_status);
            if let Some(signal) = status.communication_status.signal_strength {
                println!("Signal Strength: {}%", signal);
            }
            if let Some(last_contact) = status.communication_status.last_successful_contact {
                println!("Last Contact: {:?}", last_contact);
            }
        }
        
        if self.detailed || self.transmission {
            println!("\n--- Transmission Statistics ---");
            println!("Messages Sent: {}", status.transmission_stats.messages_sent);
            println!("Transmission Failures: {}", status.transmission_stats.transmission_failures);
            println!("Average Interval: {}ms", status.transmission_stats.average_transmission_interval_ms);
            if let Some(last_tx) = status.transmission_stats.last_transmission_time {
                println!("Last Transmission: {:?}", last_tx);
            }
        }
        
        if self.detailed {
            println!("\n--- System Health ---");
            println!("CPU Usage: {:.1}%", status.system_health.cpu_usage_percent);
            println!("Memory Usage: {:.1}%", status.system_health.memory_usage_percent);
            println!("GPS Signal Quality: {}", status.system_health.gps_signal_quality);
            println!("Communication Signal Quality: {}", status.system_health.comm_signal_quality);
        }
        
        Ok(())
    }
}

#[derive(Parser)]
pub struct DiagnosticCommand {
    /// Run GPS diagnostics
    #[arg(long)]
    pub gps: bool,
    
    /// Run power system diagnostics
    #[arg(long)]
    pub power: bool,
    
    /// Run communication diagnostics
    #[arg(long)]
    pub communication: bool,
    
    /// Run transmission diagnostics
    #[arg(long)]
    pub transmission: bool,
    
    /// Run all diagnostics
    #[arg(short, long)]
    pub all: bool,
}

impl DiagnosticCommand {
    pub async fn execute(&self, config_path: &PathBuf) -> Result<()> {
        info!("Executing diagnostic command");
        
        // Load configuration
        let config_manager = ConfigManager::new(config_path.clone());
        let config = config_manager.load_config().await
            .context("Failed to load configuration")?;
        
        // Create temporary beacon controller for diagnostics
        let beacon_controller = create_temp_beacon_controller(config).await
            .context("Failed to create beacon controller for diagnostics")?;
        
        println!("=== BEACON DIAGNOSTICS ===");
        
        let run_all = self.all;
        
        if run_all || self.gps {
            self.run_gps_diagnostics(&beacon_controller).await?;
        }
        
        if run_all || self.power {
            self.run_power_diagnostics(&beacon_controller).await?;
        }
        
        if run_all || self.communication {
            self.run_communication_diagnostics(&beacon_controller).await?;
        }
        
        if run_all || self.transmission {
            self.run_transmission_diagnostics(&beacon_controller).await?;
        }
        
        if !run_all && !self.gps && !self.power && !self.communication && !self.transmission {
            println!("No specific diagnostics requested. Use --all or specify individual systems.");
            println!("Available options: --gps, --power, --communication, --transmission, --all");
        }
        
        Ok(())
    }
    
    async fn run_gps_diagnostics(&self, controller: &ConcreteBeaconController) -> Result<()> {
        println!("\n--- GPS Diagnostics ---");
        
        let status = controller.get_status();
        let gps_status = &status.gps_status;
        
        // Check GPS lock status
        match gps_status {
            shared_positioning::GpsStatus::Locked => println!("✓ GPS Lock: Acquired"),
            _ => println!("✗ GPS Lock: Not acquired ({:?})", gps_status),
        }
        
        // Check position availability
        match &status.current_position {
            Some(pos) => {
                println!("✓ Position: Available");
                println!("  Coordinates: {:.6}, {:.6}", pos.latitude, pos.longitude);
                println!("  Accuracy: ±{:.1}m", pos.accuracy_m);
                println!("  Satellites: {}", pos.satellite_count);
                
                // Check accuracy
                if pos.accuracy_m <= 5.0 {
                    println!("✓ Accuracy: Excellent (≤5m)");
                } else if pos.accuracy_m <= 10.0 {
                    println!("⚠ Accuracy: Good (≤10m)");
                } else {
                    println!("✗ Accuracy: Poor (>10m)");
                }
                
                // Check satellite count
                if pos.satellite_count >= 8 {
                    println!("✓ Satellites: Excellent (≥8)");
                } else if pos.satellite_count >= 4 {
                    println!("⚠ Satellites: Adequate (≥4)");
                } else {
                    println!("✗ Satellites: Insufficient (<4)");
                }
            }
            None => println!("✗ Position: Not available"),
        }
        
        // Check signal quality
        let signal_quality = status.system_health.gps_signal_quality;
        if signal_quality >= 80 {
            println!("✓ Signal Quality: Excellent ({}%)", signal_quality);
        } else if signal_quality >= 60 {
            println!("⚠ Signal Quality: Good ({}%)", signal_quality);
        } else {
            println!("✗ Signal Quality: Poor ({}%)", signal_quality);
        }
        
        Ok(())
    }
    
    async fn run_power_diagnostics(&self, controller: &ConcreteBeaconController) -> Result<()> {
        println!("\n--- Power System Diagnostics ---");
        
        let status = controller.get_status();
        let battery = &status.battery_status;
        
        // Check battery level
        if battery.capacity_percent >= 50.0 {
            println!("✓ Battery Level: Good ({:.1}%)", battery.capacity_percent);
        } else if battery.capacity_percent >= 20.0 {
            println!("⚠ Battery Level: Low ({:.1}%)", battery.capacity_percent);
        } else {
            println!("✗ Battery Level: Critical ({:.1}%)", battery.capacity_percent);
        }
        
        // Check voltage
        if battery.voltage_v >= 3.6 {
            println!("✓ Voltage: Normal ({:.2}V)", battery.voltage_v);
        } else if battery.voltage_v >= 3.3 {
            println!("⚠ Voltage: Low ({:.2}V)", battery.voltage_v);
        } else {
            println!("✗ Voltage: Critical ({:.2}V)", battery.voltage_v);
        }
        
        // Check current draw
        if battery.current_ma <= 100.0 {
            println!("✓ Current Draw: Normal ({:.0}mA)", battery.current_ma);
        } else if battery.current_ma <= 200.0 {
            println!("⚠ Current Draw: High ({:.0}mA)", battery.current_ma);
        } else {
            println!("✗ Current Draw: Excessive ({:.0}mA)", battery.current_ma);
        }
        
        // Check temperature
        if battery.temperature_c >= 0.0 && battery.temperature_c <= 40.0 {
            println!("✓ Temperature: Normal ({:.1}°C)", battery.temperature_c);
        } else if battery.temperature_c >= -10.0 && battery.temperature_c <= 50.0 {
            println!("⚠ Temperature: Marginal ({:.1}°C)", battery.temperature_c);
        } else {
            println!("✗ Temperature: Extreme ({:.1}°C)", battery.temperature_c);
        }
        
        // Check battery health
        println!("Battery Health: {:?}", battery.health);
        
        Ok(())
    }
    
    async fn run_communication_diagnostics(&self, controller: &ConcreteBeaconController) -> Result<()> {
        println!("\n--- Communication Diagnostics ---");
        
        let status = controller.get_status();
        let comm = &status.communication_status;
        
        // Check connection status
        match comm.is_connected {
            true => println!("✓ Connection: Active"),
            false => println!("✗ Connection: Inactive"),
        }
        
        // Check signal strength
        match comm.signal_strength {
            Some(strength) => {
                if strength >= 80 {
                    println!("✓ Signal Strength: Excellent ({}%)", strength);
                } else if strength >= 60 {
                    println!("⚠ Signal Strength: Good ({}%)", strength);
                } else {
                    println!("✗ Signal Strength: Poor ({}%)", strength);
                }
            }
            None => println!("? Signal Strength: Unknown"),
        }
        
        // Check last successful contact
        match comm.last_successful_contact {
            Some(time) => {
                let elapsed = std::time::SystemTime::now()
                    .duration_since(time)
                    .unwrap_or_default();
                
                if elapsed.as_secs() < 3600 {
                    println!("✓ Last Contact: Recent ({:.0} minutes ago)", 
                        elapsed.as_secs() as f64 / 60.0);
                } else if elapsed.as_secs() < 86400 {
                    println!("⚠ Last Contact: Stale ({:.0} hours ago)", 
                        elapsed.as_secs() as f64 / 3600.0);
                } else {
                    println!("✗ Last Contact: Very stale ({:.0} days ago)", 
                        elapsed.as_secs() as f64 / 86400.0);
                }
            }
            None => println!("✗ Last Contact: Never"),
        }
        
        Ok(())
    }
    
    async fn run_transmission_diagnostics(&self, controller: &ConcreteBeaconController) -> Result<()> {
        println!("\n--- Transmission Diagnostics ---");
        
        let status = controller.get_status();
        let tx = &status.transmission_stats;
        
        // Check message transmission
        if tx.messages_sent > 0 {
            println!("✓ Messages Sent: {}", tx.messages_sent);
        } else {
            println!("⚠ Messages Sent: None yet");
        }
        
        // Check failure rate
        let failure_rate = if tx.messages_sent > 0 {
            (tx.transmission_failures as f64 / tx.messages_sent as f64) * 100.0
        } else {
            0.0
        };
        
        if failure_rate < 5.0 {
            println!("✓ Failure Rate: Low ({:.1}%)", failure_rate);
        } else if failure_rate < 15.0 {
            println!("⚠ Failure Rate: Moderate ({:.1}%)", failure_rate);
        } else {
            println!("✗ Failure Rate: High ({:.1}%)", failure_rate);
        }
        
        // Check transmission interval
        let expected_interval = 5000; // Default 5 seconds
        let actual_interval = tx.average_transmission_interval_ms;
        let interval_diff = ((actual_interval as f64 - expected_interval as f64) / expected_interval as f64).abs() * 100.0;
        
        if interval_diff < 10.0 {
            println!("✓ Transmission Interval: On schedule ({}ms)", actual_interval);
        } else if interval_diff < 25.0 {
            println!("⚠ Transmission Interval: Slightly off ({}ms)", actual_interval);
        } else {
            println!("✗ Transmission Interval: Significantly off ({}ms)", actual_interval);
        }
        
        // Check last transmission
        match tx.last_transmission_time {
            Some(time) => {
                let elapsed = std::time::SystemTime::now()
                    .duration_since(time)
                    .unwrap_or_default();
                
                if elapsed.as_secs() < 30 {
                    println!("✓ Last Transmission: Recent ({:.0}s ago)", elapsed.as_secs());
                } else if elapsed.as_secs() < 300 {
                    println!("⚠ Last Transmission: Stale ({:.0}s ago)", elapsed.as_secs());
                } else {
                    println!("✗ Last Transmission: Very stale ({:.0}s ago)", elapsed.as_secs());
                }
            }
            None => println!("✗ Last Transmission: Never"),
        }
        
        Ok(())
    }
}

async fn create_temp_beacon_controller(config: crate::beacon_controller::BeaconConfig) -> Result<ConcreteBeaconController> {
    // Create mock managers for status checking
    let gps_manager = MockGpsManager::with_test_positions(config.gps_config.clone())
        .context("Failed to create GPS manager")?;
    
    let power_manager = MockPowerManager::new();
    let communication_manager = MockCommunicationManager::new();
    let transceiver = MockTransceiver::new(1);
    
    BeaconController::new(
        config,
        gps_manager,
        power_manager,
        communication_manager,
        transceiver,
    ).context("Failed to create beacon controller")
}