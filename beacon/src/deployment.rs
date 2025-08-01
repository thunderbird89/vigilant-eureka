// Beacon deployment and operational tools
// Provides utilities for beacon configuration generation, status monitoring,
// remote configuration updates, deployment validation, and fleet management

use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime};
use tokio::fs;
use tracing::{info, warn, error};
use uuid::Uuid;
use rand;

use shared_positioning::{BeaconConfig, BeaconMessageVersion as MessageVersion, EmergencyConfig};
use crate::beacon_controller::BeaconStatus;
use crate::config::{ConfigManager, validate_config};
use shared_positioning::{
    GpsConfig, PowerConfig, CommunicationConfig, GeodeticPosition,
};

/// Serializable wrapper for GeodeticPosition
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SerializableGeodeticPosition {
    pub latitude: f64,
    pub longitude: f64,
    pub depth: f64,
}

impl From<GeodeticPosition> for SerializableGeodeticPosition {
    fn from(pos: GeodeticPosition) -> Self {
        Self {
            latitude: pos.latitude,
            longitude: pos.longitude,
            depth: pos.depth,
        }
    }
}

impl From<SerializableGeodeticPosition> for GeodeticPosition {
    fn from(pos: SerializableGeodeticPosition) -> Self {
        Self {
            latitude: pos.latitude,
            longitude: pos.longitude,
            depth: pos.depth,
        }
    }
}

/// Configuration template for generating beacon configurations
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BeaconConfigTemplate {
    pub transmission_interval_ms: u32,
    pub message_version: MessageVersion,
    pub gps_config: GpsConfig,
    pub power_config: PowerConfig,
    pub communication_config: CommunicationConfig,
    pub emergency_config: EmergencyConfig,
}

impl Default for BeaconConfigTemplate {
    fn default() -> Self {
        Self {
            transmission_interval_ms: 5000,
            message_version: MessageVersion::V3,
            gps_config: GpsConfig::default(),
            power_config: PowerConfig::default(),
            communication_config: CommunicationConfig::default(),
            emergency_config: EmergencyConfig::default(),
        }
    }
}

/// Deployment site information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeploymentSite {
    pub site_id: String,
    pub name: String,
    pub location: SerializableGeodeticPosition,
    pub deployment_type: DeploymentType,
    pub environmental_conditions: EnvironmentalConditions,
    pub expected_beacons: u32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DeploymentType {
    Surface,
    Anchored { depth_m: f32 },
    Drifting,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct EnvironmentalConditions {
    pub temperature_range_c: (f32, f32),
    pub wave_height_m: f32,
    pub current_speed_ms: f32,
    pub salinity_ppt: f32,
}

/// Beacon deployment configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BeaconDeployment {
    pub beacon_id: Uuid,
    pub site_id: String,
    pub deployment_position: Option<SerializableGeodeticPosition>,
    pub deployment_time: SystemTime,
    pub expected_battery_life_days: u32,
    pub maintenance_schedule_days: u32,
    pub config: BeaconConfig,
}

/// Fleet management information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BeaconFleet {
    pub fleet_id: String,
    pub name: String,
    pub beacons: HashMap<Uuid, BeaconDeployment>,
    pub sites: HashMap<String, DeploymentSite>,
    pub created_at: SystemTime,
    pub last_updated: SystemTime,
}

/// Configuration generation tools
pub struct ConfigGenerator {
    template: BeaconConfigTemplate,
}

impl ConfigGenerator {
    pub fn new() -> Self {
        Self {
            template: BeaconConfigTemplate::default(),
        }
    }
    
    pub fn with_template(template: BeaconConfigTemplate) -> Self {
        Self { template }
    }
    
    /// Generate a single beacon configuration
    pub fn generate_beacon_config(&self, beacon_id: Option<Uuid>) -> BeaconConfig {
        BeaconConfig::new(beacon_id.unwrap_or_else(Uuid::new_v4))
    }
    
    /// Generate multiple beacon configurations for a deployment
    pub async fn generate_deployment_configs(
        &self,
        site: &DeploymentSite,
        count: u32,
        output_dir: &Path,
    ) -> Result<Vec<BeaconDeployment>> {
        info!("Generating {} beacon configurations for site: {}", count, site.name);
        
        // Create output directory
        fs::create_dir_all(output_dir).await
            .context("Failed to create output directory")?;
        
        let mut deployments = Vec::new();
        
        for i in 0..count {
            let beacon_id = Uuid::new_v4();
            let mut config = self.generate_beacon_config(Some(beacon_id));
            
            // Adjust configuration based on site conditions
            self.adjust_config_for_site(&mut config, site);
            
            // Create deployment record
            let deployment = BeaconDeployment {
                beacon_id,
                site_id: site.site_id.clone(),
                deployment_position: None, // Will be set during actual deployment
                deployment_time: SystemTime::now(),
                expected_battery_life_days: self.estimate_battery_life(&config, site),
                maintenance_schedule_days: 30, // Default monthly maintenance
                config: config.clone(),
            };
            
            // Save individual configuration file
            let config_filename = format!("beacon_{}.toml", beacon_id);
            let config_path = output_dir.join(&config_filename);
            let config_manager = ConfigManager::new(config_path);
            config_manager.save_config(&config).await
                .context(format!("Failed to save config for beacon {}", beacon_id))?;
            
            deployments.push(deployment);
            
            info!("Generated configuration for beacon {} ({}/{})", beacon_id, i + 1, count);
        }
        
        // Save deployment manifest
        let manifest_path = output_dir.join("deployment_manifest.json");
        let manifest_content = serde_json::to_string_pretty(&deployments)
            .context("Failed to serialize deployment manifest")?;
        fs::write(&manifest_path, manifest_content).await
            .context("Failed to write deployment manifest")?;
        
        info!("Deployment configurations generated successfully in: {}", output_dir.display());
        Ok(deployments)
    }
    
    fn adjust_config_for_site(&self, config: &mut BeaconConfig, site: &DeploymentSite) {
        // Adjust GPS configuration based on deployment type
        match site.deployment_type {
            DeploymentType::Surface => {
                // Surface beacons have better GPS reception
                config.gps.acquisition_timeout_s = 60;
                config.gps.accuracy_threshold_m = 2.0;
            }
            DeploymentType::Anchored { depth_m } => {
                // Anchored beacons may have intermittent GPS
                if depth_m > 10.0 {
                    config.gps.acquisition_timeout_s = 120;
                    config.gps.accuracy_threshold_m = 5.0;
                }
            }
            DeploymentType::Drifting => {
                // Drifting beacons need frequent GPS updates
                config.gps.update_interval_s = 30;
                config.gps.accuracy_threshold_m = 3.0;
            }
        }
        
        // Adjust power configuration based on environmental conditions
        let temp_range = &site.environmental_conditions.temperature_range_c;
        if temp_range.0 < 0.0 || temp_range.1 > 40.0 {
            // Extreme temperatures affect battery performance
            config.power.low_battery_threshold_percent = 25.0;
            config.power.critical_battery_threshold_percent = 15.0;
        }
        
        // Adjust transmission interval based on wave conditions
        if site.environmental_conditions.wave_height_m > 2.0 {
            // Rough seas may affect transmission reliability
            config.transmission.interval_ms = 3000; // More frequent transmissions
        }
    }
    
    fn estimate_battery_life(&self, config: &BeaconConfig, site: &DeploymentSite) -> u32 {
        // Simple battery life estimation based on configuration and conditions
        let base_life_days = 30; // Base 30 days
        
        // Adjust for transmission interval
        let tx_factor = 5000.0 / config.transmission.interval_ms as f32;
        
        // Adjust for GPS update frequency
        let gps_factor = config.gps.update_interval_s as f32 / 30.0;
        
        // Adjust for environmental conditions
        let env_factor = if site.environmental_conditions.temperature_range_c.0 < 0.0 {
            0.8 // Cold reduces battery life
        } else {
            1.0
        };
        
        ((base_life_days as f32) * tx_factor * gps_factor * env_factor) as u32
    }
}

/// Status monitoring utilities
pub struct StatusMonitor {
    fleet: BeaconFleet,
}

impl StatusMonitor {
    pub fn new(fleet: BeaconFleet) -> Self {
        Self { fleet }
    }
    
    /// Monitor status of all beacons in the fleet
    pub async fn monitor_fleet_status(&self) -> Result<FleetStatusReport> {
        info!("Monitoring status for fleet: {}", self.fleet.name);
        
        let mut beacon_statuses = HashMap::new();
        let mut healthy_count = 0;
        let mut warning_count = 0;
        let mut critical_count = 0;
        
        for (beacon_id, deployment) in &self.fleet.beacons {
            match self.get_beacon_status(*beacon_id).await {
                Ok(status) => {
                    let health = self.assess_beacon_health(&status);
                    match health {
                        BeaconHealth::Healthy => healthy_count += 1,
                        BeaconHealth::Warning => warning_count += 1,
                        BeaconHealth::Critical => critical_count += 1,
                    }
                    beacon_statuses.insert(*beacon_id, BeaconStatusSummary {
                        beacon_id: *beacon_id,
                        health,
                        last_contact: status.transmission_stats.last_transmission_time,
                        battery_level: status.battery_status.capacity_percent,
                        gps_status: status.gps_status.clone(),
                        position: status.current_position.clone(),
                    });
                }
                Err(e) => {
                    warn!("Failed to get status for beacon {}: {}", beacon_id, e);
                    critical_count += 1;
                    beacon_statuses.insert(*beacon_id, BeaconStatusSummary {
                        beacon_id: *beacon_id,
                        health: BeaconHealth::Critical,
                        last_contact: None,
                        battery_level: 0.0,
                        gps_status: shared_positioning::GpsStatus::Acquiring,
                        position: None,
                    });
                }
            }
        }
        
        Ok(FleetStatusReport {
            fleet_id: self.fleet.fleet_id.clone(),
            timestamp: SystemTime::now(),
            total_beacons: self.fleet.beacons.len() as u32,
            healthy_count,
            warning_count,
            critical_count,
            beacon_statuses,
        })
    }
    
    async fn get_beacon_status(&self, beacon_id: Uuid) -> Result<BeaconStatus> {
        // In a real implementation, this would connect to the beacon
        // For now, we'll simulate status retrieval
        Err(anyhow::anyhow!("Status retrieval not implemented for beacon {}", beacon_id))
    }
    
    fn assess_beacon_health(&self, status: &BeaconStatus) -> BeaconHealth {
        // Assess overall beacon health based on various factors
        if status.battery_status.capacity_percent < 10.0 {
            return BeaconHealth::Critical;
        }
        
        if status.battery_status.capacity_percent < 20.0 {
            return BeaconHealth::Warning;
        }
        
        match status.gps_status {
            shared_positioning::GpsStatus::Initializing => BeaconHealth::Warning,
            shared_positioning::GpsStatus::Acquiring => BeaconHealth::Warning,
            shared_positioning::GpsStatus::Locked => BeaconHealth::Healthy,
            shared_positioning::GpsStatus::SignalLost => BeaconHealth::Critical,
            shared_positioning::GpsStatus::DegradedMode => BeaconHealth::Warning,
            shared_positioning::GpsStatus::HardwareFault => BeaconHealth::Critical,
        }
    }
}

#[derive(Debug, Clone)]
pub enum BeaconHealth {
    Healthy,
    Warning,
    Critical,
}

#[derive(Debug, Clone)]
pub struct BeaconStatusSummary {
    pub beacon_id: Uuid,
    pub health: BeaconHealth,
    pub last_contact: Option<SystemTime>,
    pub battery_level: f32,
    pub gps_status: shared_positioning::GpsStatus,
    pub position: Option<shared_positioning::GpsPosition>,
}

#[derive(Debug, Clone)]
pub struct FleetStatusReport {
    pub fleet_id: String,
    pub timestamp: SystemTime,
    pub total_beacons: u32,
    pub healthy_count: u32,
    pub warning_count: u32,
    pub critical_count: u32,
    pub beacon_statuses: HashMap<Uuid, BeaconStatusSummary>,
}

/// Remote configuration update tools
pub struct RemoteConfigManager {
    fleet: BeaconFleet,
}

impl RemoteConfigManager {
    pub fn new(fleet: BeaconFleet) -> Self {
        Self { fleet }
    }
    
    /// Update configuration for a single beacon
    pub async fn update_beacon_config(
        &mut self,
        beacon_id: Uuid,
        new_config: BeaconConfig,
    ) -> Result<()> {
        info!("Updating configuration for beacon: {}", beacon_id);
        
        // Validate new configuration
        validate_config(&new_config)
            .context("New configuration validation failed")?;
        
        // In a real implementation, this would send the configuration to the beacon
        // For now, we'll simulate the update process
        self.simulate_config_update(beacon_id, &new_config).await?;
        
        // Update local deployment record
        let deployment = self.fleet.beacons.get_mut(&beacon_id)
            .ok_or_else(|| anyhow::anyhow!("Beacon {} not found in fleet", beacon_id))?;
        deployment.config = new_config;
        self.fleet.last_updated = SystemTime::now();
        
        info!("Configuration updated successfully for beacon: {}", beacon_id);
        Ok(())
    }
    
    /// Update configuration for multiple beacons
    pub async fn update_fleet_config(
        &mut self,
        config_updates: HashMap<Uuid, BeaconConfig>,
    ) -> Result<ConfigUpdateReport> {
        info!("Updating configuration for {} beacons", config_updates.len());
        
        let mut successful_updates = Vec::new();
        let mut failed_updates = Vec::new();
        
        for (beacon_id, new_config) in config_updates {
            match self.update_beacon_config(beacon_id, new_config).await {
                Ok(()) => successful_updates.push(beacon_id),
                Err(e) => {
                    error!("Failed to update config for beacon {}: {}", beacon_id, e);
                    failed_updates.push((beacon_id, e.to_string()));
                }
            }
        }
        
        Ok(ConfigUpdateReport {
            timestamp: SystemTime::now(),
            total_attempted: successful_updates.len() + failed_updates.len(),
            successful_updates,
            failed_updates,
        })
    }
    
    /// Apply configuration template to multiple beacons
    pub async fn apply_template_to_beacons(
        &mut self,
        beacon_ids: Vec<Uuid>,
        template: BeaconConfigTemplate,
    ) -> Result<ConfigUpdateReport> {
        info!("Applying template to {} beacons", beacon_ids.len());
        
        let mut config_updates = HashMap::new();
        let config_generator = ConfigGenerator::with_template(template);
        
        for beacon_id in beacon_ids {
            // Get existing beacon configuration to preserve beacon ID
            if let Some(deployment) = self.fleet.beacons.get(&beacon_id) {
                let mut new_config = config_generator.generate_beacon_config(Some(beacon_id));
                // Preserve any beacon-specific settings if needed
                config_updates.insert(beacon_id, new_config);
            }
        }
        
        self.update_fleet_config(config_updates).await
    }
    
    async fn simulate_config_update(&self, beacon_id: Uuid, config: &BeaconConfig) -> Result<()> {
        // Simulate network delay and potential failures
        tokio::time::sleep(Duration::from_millis(100)).await;
        
        // Simulate 5% failure rate
        if rand::random::<f32>() < 0.05 {
            return Err(anyhow::anyhow!("Simulated network failure"));
        }
        
        info!("Configuration sent to beacon {} successfully", beacon_id);
        Ok(())
    }
    
    pub fn get_fleet(&self) -> &BeaconFleet {
        &self.fleet
    }
}

#[derive(Debug, Clone)]
pub struct ConfigUpdateReport {
    pub timestamp: SystemTime,
    pub total_attempted: usize,
    pub successful_updates: Vec<Uuid>,
    pub failed_updates: Vec<(Uuid, String)>,
}

/// Deployment validation and testing tools
pub struct DeploymentValidator {
    test_duration: Duration,
}

impl DeploymentValidator {
    pub fn new(test_duration: Duration) -> Self {
        Self { test_duration }
    }
    
    /// Validate a beacon deployment configuration
    pub async fn validate_deployment(&self, deployment: &BeaconDeployment) -> Result<ValidationReport> {
        info!("Validating deployment for beacon: {}", deployment.beacon_id);
        
        let mut tests = Vec::new();
        
        // Configuration validation
        tests.push(self.validate_configuration(&deployment.config).await);
        
        // GPS functionality test
        tests.push(self.test_gps_functionality(&deployment.config).await);
        
        // Power system test
        tests.push(self.test_power_system(&deployment.config).await);
        
        // Communication test
        tests.push(self.test_communication(&deployment.config).await);
        
        // Transmission test
        tests.push(self.test_transmission(&deployment.config).await);
        
        // Environmental compatibility test
        tests.push(self.test_environmental_compatibility(deployment).await);
        
        let passed_tests = tests.iter().filter(|t| t.passed).count();
        let total_tests = tests.len();
        
        let recommendations = self.generate_recommendations(&tests);
        
        Ok(ValidationReport {
            beacon_id: deployment.beacon_id,
            timestamp: SystemTime::now(),
            overall_result: passed_tests == total_tests,
            tests,
            recommendations,
        })
    }
    
    /// Run comprehensive deployment validation for multiple beacons
    pub async fn validate_fleet_deployment(
        &self,
        deployments: &[BeaconDeployment],
    ) -> Result<FleetValidationReport> {
        info!("Validating deployment for {} beacons", deployments.len());
        
        let mut validation_reports = Vec::new();
        let mut passed_count = 0;
        
        for deployment in deployments {
            match self.validate_deployment(deployment).await {
                Ok(report) => {
                    if report.overall_result {
                        passed_count += 1;
                    }
                    validation_reports.push(report);
                }
                Err(e) => {
                    error!("Validation failed for beacon {}: {}", deployment.beacon_id, e);
                    validation_reports.push(ValidationReport {
                        beacon_id: deployment.beacon_id,
                        timestamp: SystemTime::now(),
                        overall_result: false,
                        tests: vec![ValidationTest {
                            test_name: "Validation Error".to_string(),
                            passed: false,
                            details: e.to_string(),
                            duration: Duration::from_secs(0),
                        }],
                        recommendations: vec!["Fix validation errors before deployment".to_string()],
                    });
                }
            }
        }
        
        Ok(FleetValidationReport {
            timestamp: SystemTime::now(),
            total_beacons: deployments.len(),
            passed_count,
            failed_count: deployments.len() - passed_count,
            validation_reports,
        })
    }
    
    async fn validate_configuration(&self, config: &BeaconConfig) -> ValidationTest {
        let start_time = std::time::Instant::now();
        
        let result = validate_config(config);
        let duration = start_time.elapsed();
        
        ValidationTest {
            test_name: "Configuration Validation".to_string(),
            passed: result.is_ok(),
            details: match result {
                Ok(()) => "Configuration is valid".to_string(),
                Err(e) => format!("Configuration validation failed: {}", e),
            },
            duration,
        }
    }
    
    async fn test_gps_functionality(&self, config: &BeaconConfig) -> ValidationTest {
        let start_time = std::time::Instant::now();
        
        // Simulate GPS test
        tokio::time::sleep(Duration::from_millis(500)).await;
        
        let duration = start_time.elapsed();
        
        // Simulate GPS test results
        let passed = config.gps.acquisition_timeout_s >= 30;
        
        ValidationTest {
            test_name: "GPS Functionality".to_string(),
            passed,
            details: if passed {
                "GPS configuration is suitable for deployment".to_string()
            } else {
                "GPS acquisition timeout may be too short".to_string()
            },
            duration,
        }
    }
    
    async fn test_power_system(&self, config: &BeaconConfig) -> ValidationTest {
        let start_time = std::time::Instant::now();
        
        // Simulate power system test
        tokio::time::sleep(Duration::from_millis(300)).await;
        
        let duration = start_time.elapsed();
        
        // Check power configuration
        let passed = config.power.low_battery_threshold_percent > 
                    config.power.critical_battery_threshold_percent;
        
        ValidationTest {
            test_name: "Power System".to_string(),
            passed,
            details: if passed {
                "Power management configuration is valid".to_string()
            } else {
                "Power threshold configuration is invalid".to_string()
            },
            duration,
        }
    }
    
    async fn test_communication(&self, config: &BeaconConfig) -> ValidationTest {
        let start_time = std::time::Instant::now();
        
        // Simulate communication test
        tokio::time::sleep(Duration::from_millis(800)).await;
        
        let duration = start_time.elapsed();
        
        // Check communication configuration
        let passed = config.communication.connection_timeout_s >= 30;
        
        ValidationTest {
            test_name: "Communication".to_string(),
            passed,
            details: if passed {
                "Communication configuration is suitable".to_string()
            } else {
                "Communication timeout may be too short".to_string()
            },
            duration,
        }
    }
    
    async fn test_transmission(&self, config: &BeaconConfig) -> ValidationTest {
        let start_time = std::time::Instant::now();
        
        // Simulate transmission test
        tokio::time::sleep(Duration::from_millis(400)).await;
        
        let duration = start_time.elapsed();
        
        // Check transmission configuration
        let passed = config.transmission.interval_ms >= 1000 && 
                    config.transmission.interval_ms <= 60000;
        
        ValidationTest {
            test_name: "Transmission".to_string(),
            passed,
            details: if passed {
                "Transmission interval is within acceptable range".to_string()
            } else {
                "Transmission interval is outside recommended range".to_string()
            },
            duration,
        }
    }
    
    async fn test_environmental_compatibility(&self, deployment: &BeaconDeployment) -> ValidationTest {
        let start_time = std::time::Instant::now();
        
        // Simulate environmental compatibility test
        tokio::time::sleep(Duration::from_millis(200)).await;
        
        let duration = start_time.elapsed();
        
        // Simple environmental compatibility check
        let passed = deployment.expected_battery_life_days >= 7; // At least 1 week
        
        ValidationTest {
            test_name: "Environmental Compatibility".to_string(),
            passed,
            details: if passed {
                format!("Expected battery life: {} days", deployment.expected_battery_life_days)
            } else {
                "Expected battery life is too short for deployment".to_string()
            },
            duration,
        }
    }
    
    fn generate_recommendations(&self, tests: &[ValidationTest]) -> Vec<String> {
        let mut recommendations = Vec::new();
        
        for test in tests {
            if !test.passed {
                match test.test_name.as_str() {
                    "Configuration Validation" => {
                        recommendations.push("Review and fix configuration parameters".to_string());
                    }
                    "GPS Functionality" => {
                        recommendations.push("Increase GPS acquisition timeout for better reliability".to_string());
                    }
                    "Power System" => {
                        recommendations.push("Adjust power management thresholds".to_string());
                    }
                    "Communication" => {
                        recommendations.push("Increase communication timeout for better reliability".to_string());
                    }
                    "Transmission" => {
                        recommendations.push("Adjust transmission interval to recommended range (1-60 seconds)".to_string());
                    }
                    "Environmental Compatibility" => {
                        recommendations.push("Consider environmental factors and adjust configuration accordingly".to_string());
                    }
                    _ => {}
                }
            }
        }
        
        if recommendations.is_empty() {
            recommendations.push("All tests passed - deployment ready".to_string());
        }
        
        recommendations
    }
}

#[derive(Debug, Clone)]
pub struct ValidationTest {
    pub test_name: String,
    pub passed: bool,
    pub details: String,
    pub duration: Duration,
}

#[derive(Debug, Clone)]
pub struct ValidationReport {
    pub beacon_id: Uuid,
    pub timestamp: SystemTime,
    pub overall_result: bool,
    pub tests: Vec<ValidationTest>,
    pub recommendations: Vec<String>,
}

#[derive(Debug, Clone)]
pub struct FleetValidationReport {
    pub timestamp: SystemTime,
    pub total_beacons: usize,
    pub passed_count: usize,
    pub failed_count: usize,
    pub validation_reports: Vec<ValidationReport>,
}

/// Fleet management utilities
pub struct FleetManager {
    fleet: BeaconFleet,
    data_dir: PathBuf,
}

impl FleetManager {
    pub fn new(fleet: BeaconFleet, data_dir: PathBuf) -> Self {
        Self { fleet, data_dir }
    }
    
    /// Create a new fleet
    pub async fn create_fleet(
        fleet_id: String,
        name: String,
        data_dir: PathBuf,
    ) -> Result<Self> {
        info!("Creating new fleet: {} ({})", name, fleet_id);
        
        let fleet = BeaconFleet {
            fleet_id: fleet_id.clone(),
            name,
            beacons: HashMap::new(),
            sites: HashMap::new(),
            created_at: SystemTime::now(),
            last_updated: SystemTime::now(),
        };
        
        let manager = Self::new(fleet, data_dir);
        manager.save_fleet().await?;
        
        Ok(manager)
    }
    
    /// Load existing fleet from storage
    pub async fn load_fleet(fleet_id: &str, data_dir: PathBuf) -> Result<Self> {
        info!("Loading fleet: {}", fleet_id);
        
        let fleet_file = data_dir.join(format!("{}.json", fleet_id));
        let fleet_content = fs::read_to_string(&fleet_file).await
            .context(format!("Failed to read fleet file: {}", fleet_file.display()))?;
        
        let fleet: BeaconFleet = serde_json::from_str(&fleet_content)
            .context("Failed to parse fleet data")?;
        
        Ok(Self::new(fleet, data_dir))
    }
    
    /// Add a deployment site to the fleet
    pub async fn add_site(&mut self, site: DeploymentSite) -> Result<()> {
        info!("Adding site: {} ({})", site.name, site.site_id);
        
        self.fleet.sites.insert(site.site_id.clone(), site);
        self.fleet.last_updated = SystemTime::now();
        self.save_fleet().await?;
        
        Ok(())
    }
    
    /// Add a beacon deployment to the fleet
    pub async fn add_beacon(&mut self, deployment: BeaconDeployment) -> Result<()> {
        info!("Adding beacon: {} to site: {}", deployment.beacon_id, deployment.site_id);
        
        // Verify site exists
        if !self.fleet.sites.contains_key(&deployment.site_id) {
            return Err(anyhow::anyhow!("Site {} not found in fleet", deployment.site_id));
        }
        
        self.fleet.beacons.insert(deployment.beacon_id, deployment);
        self.fleet.last_updated = SystemTime::now();
        self.save_fleet().await?;
        
        Ok(())
    }
    
    /// Remove a beacon from the fleet
    pub async fn remove_beacon(&mut self, beacon_id: Uuid) -> Result<()> {
        info!("Removing beacon: {}", beacon_id);
        
        self.fleet.beacons.remove(&beacon_id)
            .ok_or_else(|| anyhow::anyhow!("Beacon {} not found in fleet", beacon_id))?;
        
        self.fleet.last_updated = SystemTime::now();
        self.save_fleet().await?;
        
        Ok(())
    }
    
    /// Get fleet summary
    pub fn get_fleet_summary(&self) -> FleetSummary {
        let mut sites_by_type = HashMap::new();
        for site in self.fleet.sites.values() {
            let type_name = match &site.deployment_type {
                DeploymentType::Surface => "Surface",
                DeploymentType::Anchored { .. } => "Anchored",
                DeploymentType::Drifting => "Drifting",
            };
            *sites_by_type.entry(type_name.to_string()).or_insert(0) += 1;
        }
        
        FleetSummary {
            fleet_id: self.fleet.fleet_id.clone(),
            name: self.fleet.name.clone(),
            total_beacons: self.fleet.beacons.len(),
            total_sites: self.fleet.sites.len(),
            sites_by_type,
            created_at: self.fleet.created_at,
            last_updated: self.fleet.last_updated,
        }
    }
    
    /// Generate fleet deployment report
    pub async fn generate_deployment_report(&self) -> Result<FleetDeploymentReport> {
        info!("Generating deployment report for fleet: {}", self.fleet.name);
        
        let mut site_reports = HashMap::new();
        
        for (site_id, site) in &self.fleet.sites {
            let beacons_at_site: Vec<_> = self.fleet.beacons.values()
                .filter(|d| d.site_id == *site_id)
                .cloned()
                .collect();
            
            site_reports.insert(site_id.clone(), SiteDeploymentReport {
                site: site.clone(),
                deployed_beacons: beacons_at_site.len(),
                expected_beacons: site.expected_beacons,
                deployments: beacons_at_site,
            });
        }
        
        Ok(FleetDeploymentReport {
            fleet_summary: self.get_fleet_summary(),
            site_reports,
            generated_at: SystemTime::now(),
        })
    }
    
    /// Save fleet data to storage
    async fn save_fleet(&self) -> Result<()> {
        fs::create_dir_all(&self.data_dir).await
            .context("Failed to create data directory")?;
        
        let fleet_file = self.data_dir.join(format!("{}.json", self.fleet.fleet_id));
        let fleet_content = serde_json::to_string_pretty(&self.fleet)
            .context("Failed to serialize fleet data")?;
        
        fs::write(&fleet_file, fleet_content).await
            .context("Failed to write fleet file")?;
        
        Ok(())
    }
    
    pub fn get_fleet(&self) -> &BeaconFleet {
        &self.fleet
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FleetSummary {
    pub fleet_id: String,
    pub name: String,
    pub total_beacons: usize,
    pub total_sites: usize,
    pub sites_by_type: HashMap<String, u32>,
    pub created_at: SystemTime,
    pub last_updated: SystemTime,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SiteDeploymentReport {
    pub site: DeploymentSite,
    pub deployed_beacons: usize,
    pub expected_beacons: u32,
    pub deployments: Vec<BeaconDeployment>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FleetDeploymentReport {
    pub fleet_summary: FleetSummary,
    pub site_reports: HashMap<String, SiteDeploymentReport>,
    pub generated_at: SystemTime,
}