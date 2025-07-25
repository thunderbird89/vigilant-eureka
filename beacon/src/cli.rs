use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use std::path::PathBuf;
use std::time::Duration;
use tracing::{error, info};
use uuid::Uuid;

use crate::config::ConfigManager;
use crate::beacon_controller::BeaconController;
use crate::deployment::{
    ConfigGenerator, BeaconConfigTemplate, DeploymentSite, DeploymentType,
    EnvironmentalConditions, FleetManager, DeploymentValidator, RemoteConfigManager,
    StatusMonitor, BeaconFleet, BeaconDeployment, BeaconHealth, SerializableGeodeticPosition,
};
use shared_positioning::{
    MockGpsManager, MockPowerManager, MockCommunicationManager, MockTransceiver,
    GeodeticPosition,
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
    /// Deployment and operational tools
    Deploy(DeployCommand),
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

async fn create_temp_beacon_controller(config: shared_positioning::BeaconConfig) -> Result<ConcreteBeaconController> {
    // Create mock managers for status checking
    let gps_config = shared_positioning::GpsConfig {
        acquisition_timeout_s: config.gps.acquisition_timeout_s,
        update_interval_s: config.gps.update_interval_s,
        min_satellite_count: config.gps.min_satellite_count,
        accuracy_threshold_m: config.gps.accuracy_threshold_m,
        cold_start_timeout_s: config.gps.cold_start_timeout_s,
    };
    let gps_manager = MockGpsManager::with_test_positions(gps_config)
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
#[
derive(Parser)]
pub struct DeployCommand {
    #[command(subcommand)]
    pub action: DeployAction,
}

#[derive(Subcommand)]
pub enum DeployAction {
    /// Generate beacon configurations for deployment
    GenerateConfigs {
        /// Number of beacon configurations to generate
        #[arg(short, long, default_value = "1")]
        count: u32,
        /// Output directory for configurations
        #[arg(short, long, default_value = "deployment")]
        output: PathBuf,
        /// Site configuration file
        #[arg(short, long)]
        site_config: Option<PathBuf>,
        /// Configuration template file
        #[arg(short, long)]
        template: Option<PathBuf>,
    },
    /// Validate deployment configurations
    ValidateDeployment {
        /// Deployment manifest file or directory
        #[arg(short, long)]
        deployment: PathBuf,
        /// Test duration in seconds
        #[arg(short, long, default_value = "30")]
        test_duration: u64,
    },
    /// Monitor fleet status
    MonitorFleet {
        /// Fleet configuration file
        #[arg(short, long)]
        fleet_config: PathBuf,
        /// Continuous monitoring interval in seconds
        #[arg(short, long)]
        interval: Option<u64>,
    },
    /// Update beacon configurations remotely
    UpdateConfigs {
        /// Fleet configuration file
        #[arg(short, long)]
        fleet_config: PathBuf,
        /// Configuration template to apply
        #[arg(short, long)]
        template: PathBuf,
        /// Specific beacon IDs to update (comma-separated)
        #[arg(short, long)]
        beacons: Option<String>,
    },
    /// Manage beacon fleet
    Fleet {
        #[command(subcommand)]
        action: FleetAction,
    },
}

#[derive(Subcommand)]
pub enum FleetAction {
    /// Create a new fleet
    Create {
        /// Fleet ID
        #[arg(short, long)]
        id: String,
        /// Fleet name
        #[arg(short, long)]
        name: String,
        /// Data directory
        #[arg(short, long, default_value = "fleet_data")]
        data_dir: PathBuf,
    },
    /// Add a site to the fleet
    AddSite {
        /// Fleet ID
        #[arg(short, long)]
        fleet_id: String,
        /// Site configuration file
        #[arg(short, long)]
        site_config: PathBuf,
        /// Data directory
        #[arg(short, long, default_value = "fleet_data")]
        data_dir: PathBuf,
    },
    /// Add beacons to the fleet
    AddBeacons {
        /// Fleet ID
        #[arg(short, long)]
        fleet_id: String,
        /// Deployment manifest file
        #[arg(short, long)]
        deployment: PathBuf,
        /// Data directory
        #[arg(short, long, default_value = "fleet_data")]
        data_dir: PathBuf,
    },
    /// Show fleet summary
    Summary {
        /// Fleet ID
        #[arg(short, long)]
        fleet_id: String,
        /// Data directory
        #[arg(short, long, default_value = "fleet_data")]
        data_dir: PathBuf,
    },
    /// Generate deployment report
    Report {
        /// Fleet ID
        #[arg(short, long)]
        fleet_id: String,
        /// Output file
        #[arg(short, long)]
        output: Option<PathBuf>,
        /// Data directory
        #[arg(short, long, default_value = "fleet_data")]
        data_dir: PathBuf,
    },
}

impl DeployCommand {
    pub async fn execute(&self, _config_path: &PathBuf) -> Result<()> {
        match &self.action {
            DeployAction::GenerateConfigs { count, output, site_config, template } => {
                self.generate_configs(*count, output, site_config.as_ref(), template.as_ref()).await
            }
            DeployAction::ValidateDeployment { deployment, test_duration } => {
                self.validate_deployment(deployment, *test_duration).await
            }
            DeployAction::MonitorFleet { fleet_config, interval } => {
                self.monitor_fleet(fleet_config, *interval).await
            }
            DeployAction::UpdateConfigs { fleet_config, template, beacons } => {
                self.update_configs(fleet_config, template, beacons.as_ref()).await
            }
            DeployAction::Fleet { action } => {
                self.handle_fleet_action(action).await
            }
        }
    }
    
    async fn generate_configs(
        &self,
        count: u32,
        output: &PathBuf,
        site_config: Option<&PathBuf>,
        template: Option<&PathBuf>,
    ) -> Result<()> {
        info!("Generating {} beacon configurations", count);
        
        // Load or create default site configuration
        let site = if let Some(site_path) = site_config {
            let site_content = tokio::fs::read_to_string(site_path).await
                .context("Failed to read site configuration")?;
            serde_json::from_str(&site_content)
                .context("Failed to parse site configuration")?
        } else {
            // Create default site
            DeploymentSite {
                site_id: "default_site".to_string(),
                name: "Default Deployment Site".to_string(),
                location: SerializableGeodeticPosition {
                    latitude: 40.7128,
                    longitude: -74.0060,
                    depth: 0.0,
                },
                deployment_type: DeploymentType::Surface,
                environmental_conditions: EnvironmentalConditions {
                    temperature_range_c: (5.0, 25.0),
                    wave_height_m: 1.0,
                    current_speed_ms: 0.5,
                    salinity_ppt: 35.0,
                },
                expected_beacons: count,
            }
        };
        
        // Load or create default template
        let template = if let Some(template_path) = template {
            let template_content = tokio::fs::read_to_string(template_path).await
                .context("Failed to read template configuration")?;
            serde_json::from_str(&template_content)
                .context("Failed to parse template configuration")?
        } else {
            BeaconConfigTemplate::default()
        };
        
        let config_generator = ConfigGenerator::with_template(template);
        let deployments = config_generator.generate_deployment_configs(&site, count, output).await
            .context("Failed to generate deployment configurations")?;
        
        println!("✓ Generated {} beacon configurations in: {}", deployments.len(), output.display());
        println!("  Site: {} ({})", site.name, site.site_id);
        
        for deployment in &deployments {
            println!("  - Beacon {}: Expected battery life {} days", 
                deployment.beacon_id, deployment.expected_battery_life_days);
        }
        
        Ok(())
    }
    
    async fn validate_deployment(&self, deployment_path: &PathBuf, test_duration: u64) -> Result<()> {
        info!("Validating deployment: {}", deployment_path.display());
        
        let validator = DeploymentValidator::new(Duration::from_secs(test_duration));
        
        // Load deployment manifest
        let deployments = if deployment_path.is_file() {
            let manifest_content = tokio::fs::read_to_string(deployment_path).await
                .context("Failed to read deployment manifest")?;
            let deployments: Vec<BeaconDeployment> = serde_json::from_str(&manifest_content)
                .context("Failed to parse deployment manifest")?;
            deployments
        } else {
            // Look for manifest in directory
            let manifest_path = deployment_path.join("deployment_manifest.json");
            let manifest_content = tokio::fs::read_to_string(&manifest_path).await
                .context("Failed to read deployment manifest from directory")?;
            serde_json::from_str(&manifest_content)
                .context("Failed to parse deployment manifest")?
        };
        
        let validation_report = validator.validate_fleet_deployment(&deployments).await
            .context("Failed to validate deployment")?;
        
        println!("=== DEPLOYMENT VALIDATION REPORT ===");
        println!("Total Beacons: {}", validation_report.total_beacons);
        println!("Passed: {}", validation_report.passed_count);
        println!("Failed: {}", validation_report.failed_count);
        println!("Success Rate: {:.1}%", 
            (validation_report.passed_count as f64 / validation_report.total_beacons as f64) * 100.0);
        
        for report in &validation_report.validation_reports {
            println!("\n--- Beacon {} ---", report.beacon_id);
            println!("Overall Result: {}", if report.overall_result { "✓ PASS" } else { "✗ FAIL" });
            
            for test in &report.tests {
                let status = if test.passed { "✓" } else { "✗" };
                println!("  {} {}: {} ({:.2}s)", status, test.test_name, test.details, test.duration.as_secs_f64());
            }
            
            if !report.recommendations.is_empty() {
                println!("  Recommendations:");
                for rec in &report.recommendations {
                    println!("    - {}", rec);
                }
            }
        }
        
        Ok(())
    }
    
    async fn monitor_fleet(&self, fleet_config: &PathBuf, interval: Option<u64>) -> Result<()> {
        info!("Monitoring fleet: {}", fleet_config.display());
        
        // Load fleet configuration
        let fleet_content = tokio::fs::read_to_string(fleet_config).await
            .context("Failed to read fleet configuration")?;
        let fleet: BeaconFleet = serde_json::from_str(&fleet_content)
            .context("Failed to parse fleet configuration")?;
        
        let monitor = StatusMonitor::new(fleet);
        
        if let Some(interval_secs) = interval {
            // Continuous monitoring
            println!("Starting continuous monitoring (interval: {}s)", interval_secs);
            println!("Press Ctrl+C to stop");
            
            loop {
                match monitor.monitor_fleet_status().await {
                    Ok(report) => {
                        println!("\n=== FLEET STATUS ({:?}) ===", report.timestamp);
                        println!("Fleet: {}", report.fleet_id);
                        println!("Total Beacons: {}", report.total_beacons);
                        println!("Healthy: {} | Warning: {} | Critical: {}", 
                            report.healthy_count, report.warning_count, report.critical_count);
                        
                        for (beacon_id, status) in &report.beacon_statuses {
                            let health_icon = match status.health {
                                BeaconHealth::Healthy => "✓",
                                BeaconHealth::Warning => "⚠",
                                BeaconHealth::Critical => "✗",
                            };
                            println!("  {} {}: Battery {:.1}% | GPS {:?}", 
                                health_icon, beacon_id, status.battery_level, status.gps_status);
                        }
                    }
                    Err(e) => {
                        error!("Failed to get fleet status: {}", e);
                    }
                }
                
                tokio::time::sleep(Duration::from_secs(interval_secs)).await;
            }
        } else {
            // Single status check
            let report = monitor.monitor_fleet_status().await
                .context("Failed to get fleet status")?;
            
            println!("=== FLEET STATUS REPORT ===");
            println!("Fleet: {}", report.fleet_id);
            println!("Timestamp: {:?}", report.timestamp);
            println!("Total Beacons: {}", report.total_beacons);
            println!("Healthy: {} ({:.1}%)", report.healthy_count, 
                (report.healthy_count as f64 / report.total_beacons as f64) * 100.0);
            println!("Warning: {} ({:.1}%)", report.warning_count,
                (report.warning_count as f64 / report.total_beacons as f64) * 100.0);
            println!("Critical: {} ({:.1}%)", report.critical_count,
                (report.critical_count as f64 / report.total_beacons as f64) * 100.0);
            
            println!("\n--- Beacon Details ---");
            for (beacon_id, status) in &report.beacon_statuses {
                println!("Beacon {}: {:?}", beacon_id, status.health);
                println!("  Battery: {:.1}%", status.battery_level);
                println!("  GPS: {:?}", status.gps_status);
                if let Some(last_contact) = status.last_contact {
                    println!("  Last Contact: {:?}", last_contact);
                } else {
                    println!("  Last Contact: Never");
                }
                if let Some(position) = &status.position {
                    println!("  Position: {:.6}, {:.6}", position.latitude, position.longitude);
                }
            }
        }
        
        Ok(())
    }
    
    async fn update_configs(
        &self,
        fleet_config: &PathBuf,
        template: &PathBuf,
        beacons: Option<&String>,
    ) -> Result<()> {
        info!("Updating beacon configurations");
        
        // Load fleet configuration
        let fleet_content = tokio::fs::read_to_string(fleet_config).await
            .context("Failed to read fleet configuration")?;
        let fleet: BeaconFleet = serde_json::from_str(&fleet_content)
            .context("Failed to parse fleet configuration")?;
        
        // Load configuration template
        let template_content = tokio::fs::read_to_string(template).await
            .context("Failed to read template configuration")?;
        let template: BeaconConfigTemplate = serde_json::from_str(&template_content)
            .context("Failed to parse template configuration")?;
        
        // Create remote configuration manager
        let mut config_manager = RemoteConfigManager::new(fleet);
        
        // Determine target beacons
        let target_beacons: Vec<Uuid> = if let Some(beacon_list) = beacons {
            beacon_list.split(',')
                .map(|s| s.trim().parse::<Uuid>())
                .collect::<Result<Vec<_>, _>>()
                .context("Failed to parse beacon IDs")?
        } else {
            config_manager.get_fleet().beacons.keys().cloned().collect()
        };
        
        println!("Updating configuration for {} beacons", target_beacons.len());
        
        let update_report = config_manager.apply_template_to_beacons(target_beacons, template).await
            .context("Failed to update beacon configurations")?;
        
        println!("=== CONFIGURATION UPDATE REPORT ===");
        println!("Timestamp: {:?}", update_report.timestamp);
        println!("Total Attempted: {}", update_report.total_attempted);
        println!("Successful: {}", update_report.successful_updates.len());
        println!("Failed: {}", update_report.failed_updates.len());
        
        if !update_report.successful_updates.is_empty() {
            println!("\nSuccessful Updates:");
            for beacon_id in &update_report.successful_updates {
                println!("  ✓ {}", beacon_id);
            }
        }
        
        if !update_report.failed_updates.is_empty() {
            println!("\nFailed Updates:");
            for (beacon_id, error) in &update_report.failed_updates {
                println!("  ✗ {}: {}", beacon_id, error);
            }
        }
        
        Ok(())
    }
    

    async fn handle_fleet_action(&self, action: &FleetAction) -> Result<()> {
        match action {
            FleetAction::Create { id, name, data_dir } => {
                let fleet_manager = FleetManager::create_fleet(id.clone(), name.clone(), data_dir.clone()).await
                    .context("Failed to create fleet")?;
                
                println!("✓ Created fleet: {} ({})", name, id);
                println!("  Data directory: {}", data_dir.display());
                
                Ok(())
            }
            FleetAction::AddSite { fleet_id, site_config, data_dir } => {
                let mut fleet_manager = FleetManager::load_fleet(fleet_id, data_dir.clone()).await
                    .context("Failed to load fleet")?;
                
                let site_content = tokio::fs::read_to_string(site_config).await
                    .context("Failed to read site configuration")?;
                let site: DeploymentSite = serde_json::from_str(&site_content)
                    .context("Failed to parse site configuration")?;
                
                fleet_manager.add_site(site.clone()).await
                    .context("Failed to add site to fleet")?;
                
                println!("✓ Added site: {} ({}) to fleet {}", site.name, site.site_id, fleet_id);
                
                Ok(())
            }
            FleetAction::AddBeacons { fleet_id, deployment, data_dir } => {
                let mut fleet_manager = FleetManager::load_fleet(fleet_id, data_dir.clone()).await
                    .context("Failed to load fleet")?;
                
                let deployment_content = tokio::fs::read_to_string(deployment).await
                    .context("Failed to read deployment manifest")?;
                let deployments: Vec<BeaconDeployment> = serde_json::from_str(&deployment_content)
                    .context("Failed to parse deployment manifest")?;
                
                let deployment_count = deployments.len();
                
                for deployment in deployments {
                    fleet_manager.add_beacon(deployment.clone()).await
                        .context(format!("Failed to add beacon {}", deployment.beacon_id))?;
                }
                
                println!("✓ Added {} beacons to fleet {}", deployment_count, fleet_id);
                
                Ok(())
            }
            FleetAction::Summary { fleet_id, data_dir } => {
                let fleet_manager = FleetManager::load_fleet(fleet_id, data_dir.clone()).await
                    .context("Failed to load fleet")?;
                
                let summary = fleet_manager.get_fleet_summary();
                
                println!("=== FLEET SUMMARY ===");
                println!("Fleet: {} ({})", summary.name, summary.fleet_id);
                println!("Created: {:?}", summary.created_at);
                println!("Last Updated: {:?}", summary.last_updated);
                println!("Total Beacons: {}", summary.total_beacons);
                println!("Total Sites: {}", summary.total_sites);
                
                if !summary.sites_by_type.is_empty() {
                    println!("\nSites by Type:");
                    for (site_type, count) in &summary.sites_by_type {
                        println!("  {}: {}", site_type, count);
                    }
                }
                
                Ok(())
            }
            FleetAction::Report { fleet_id, output, data_dir } => {
                let fleet_manager = FleetManager::load_fleet(fleet_id, data_dir.clone()).await
                    .context("Failed to load fleet")?;
                
                let report = fleet_manager.generate_deployment_report().await
                    .context("Failed to generate deployment report")?;
                
                let report_content = serde_json::to_string_pretty(&report)
                    .context("Failed to serialize deployment report")?;
                
                if let Some(output_path) = output {
                    tokio::fs::write(output_path, &report_content).await
                        .context("Failed to write deployment report")?;
                    println!("✓ Deployment report saved to: {}", output_path.display());
                } else {
                    println!("=== FLEET DEPLOYMENT REPORT ===");
                    println!("{}", report_content);
                }
                
                Ok(())
            }
        }
    }
}