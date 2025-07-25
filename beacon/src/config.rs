use anyhow::{Context, Result};
use serde::{Deserialize, Serialize};
use std::path::PathBuf;
use tokio::fs;
use tracing::{info, warn};

use shared_positioning::BeaconConfig;

pub struct ConfigManager {
    config_path: PathBuf,
}

impl ConfigManager {
    pub fn new(config_path: PathBuf) -> Self {
        Self { config_path }
    }
    
    pub async fn load_config(&self) -> Result<BeaconConfig> {
        info!("Loading configuration from: {}", self.config_path.display());
        
        if !self.config_path.exists() {
            return Err(anyhow::anyhow!(
                "Configuration file not found: {}. Use 'beacon generate-config' to create one.",
                self.config_path.display()
            ));
        }
        
        let config_content = fs::read_to_string(&self.config_path).await
            .context("Failed to read configuration file")?;
        
        let config: BeaconConfig = match self.config_path.extension().and_then(|s| s.to_str()) {
            Some("toml") => {
                toml::from_str(&config_content)
                    .context("Failed to parse TOML configuration")?
            }
            Some("yaml") | Some("yml") => {
                serde_yaml::from_str(&config_content)
                    .context("Failed to parse YAML configuration")?
            }
            Some("json") => {
                serde_json::from_str(&config_content)
                    .context("Failed to parse JSON configuration")?
            }
            _ => {
                // Default to TOML
                toml::from_str(&config_content)
                    .context("Failed to parse configuration (assumed TOML format)")?
            }
        };
        
        info!("Configuration loaded successfully");
        Ok(config)
    }
    
    pub async fn save_config(&self, config: &BeaconConfig) -> Result<()> {
        info!("Saving configuration to: {}", self.config_path.display());
        
        // Create parent directory if it doesn't exist
        if let Some(parent) = self.config_path.parent() {
            fs::create_dir_all(parent).await
                .context("Failed to create configuration directory")?;
        }
        
        let config_content = match self.config_path.extension().and_then(|s| s.to_str()) {
            Some("yaml") | Some("yml") => {
                serde_yaml::to_string(config)
                    .context("Failed to serialize configuration to YAML")?
            }
            Some("json") => {
                serde_json::to_string_pretty(config)
                    .context("Failed to serialize configuration to JSON")?
            }
            _ => {
                // Default to TOML
                toml::to_string_pretty(config)
                    .context("Failed to serialize configuration to TOML")?
            }
        };
        
        fs::write(&self.config_path, config_content).await
            .context("Failed to write configuration file")?;
        
        info!("Configuration saved successfully");
        Ok(())
    }
    
    pub async fn reload_config(&self) -> Result<BeaconConfig> {
        info!("Reloading configuration");
        self.load_config().await
    }
    
    pub fn get_config_path(&self) -> &PathBuf {
        &self.config_path
    }
}

pub fn validate_config(config: &BeaconConfig) -> Result<()> {
    // Validate beacon ID
    if config.beacon_id.is_nil() {
        return Err(anyhow::anyhow!("Beacon ID cannot be nil"));
    }
    
    // Validate transmission interval
    if config.transmission.interval_ms < 1000 {
        return Err(anyhow::anyhow!(
            "Transmission interval too short: {}ms (minimum: 1000ms)",
            config.transmission.interval_ms
        ));
    }
    
    if config.transmission.interval_ms > 60000 {
        return Err(anyhow::anyhow!(
            "Transmission interval too long: {}ms (maximum: 60000ms)",
            config.transmission.interval_ms
        ));
    }
    
    // Validate GPS configuration
    validate_gps_config(&config.gps)?;
    
    // Validate power configuration
    validate_power_config(&config.power)?;
    
    // Validate communication configuration
    validate_communication_config(&config.communication)?;
    
    // Validate emergency configuration
    validate_emergency_config(&config.emergency)?;
    
    Ok(())
}

fn validate_gps_config(config: &shared_positioning::BeaconGpsConfig) -> Result<()> {
    if config.acquisition_timeout_s < 10 {
        return Err(anyhow::anyhow!(
            "GPS acquisition timeout too short: {}s (minimum: 10s)",
            config.acquisition_timeout_s
        ));
    }
    
    if config.acquisition_timeout_s > 300 {
        return Err(anyhow::anyhow!(
            "GPS acquisition timeout too long: {}s (maximum: 300s)",
            config.acquisition_timeout_s
        ));
    }
    
    if config.update_interval_s < 1 {
        return Err(anyhow::anyhow!(
            "GPS update interval too short: {}s (minimum: 1s)",
            config.update_interval_s
        ));
    }
    
    if config.min_satellite_count < 3 {
        return Err(anyhow::anyhow!(
            "Minimum satellite count too low: {} (minimum: 3)",
            config.min_satellite_count
        ));
    }
    
    if config.accuracy_threshold_m < 1.0 {
        return Err(anyhow::anyhow!(
            "GPS accuracy threshold too strict: {:.1}m (minimum: 1.0m)",
            config.accuracy_threshold_m
        ));
    }
    
    Ok(())
}

fn validate_power_config(config: &shared_positioning::BeaconPowerConfig) -> Result<()> {
    if config.low_battery_threshold_percent <= config.critical_battery_threshold_percent {
        return Err(anyhow::anyhow!(
            "Low battery threshold ({:.1}%) must be higher than critical threshold ({:.1}%)",
            config.low_battery_threshold_percent,
            config.critical_battery_threshold_percent
        ));
    }
    
    if config.critical_battery_threshold_percent <= config.emergency_battery_threshold_percent {
        return Err(anyhow::anyhow!(
            "Critical battery threshold ({:.1}%) must be higher than emergency threshold ({:.1}%)",
            config.critical_battery_threshold_percent,
            config.emergency_battery_threshold_percent
        ));
    }
    
    if config.emergency_battery_threshold_percent < 1.0 {
        return Err(anyhow::anyhow!(
            "Emergency battery threshold too low: {:.1}% (minimum: 1.0%)",
            config.emergency_battery_threshold_percent
        ));
    }
    
    if config.low_battery_threshold_percent > 50.0 {
        return Err(anyhow::anyhow!(
            "Low battery threshold too high: {:.1}% (maximum: 50.0%)",
            config.low_battery_threshold_percent
        ));
    }
    
    Ok(())
}

fn validate_communication_config(config: &shared_positioning::BeaconCommunicationConfig) -> Result<()> {
    if config.connection_interval_hours < 1 {
        return Err(anyhow::anyhow!(
            "Connection interval too short: {}h (minimum: 1h)",
            config.connection_interval_hours
        ));
    }
    
    if config.connection_interval_hours > 168 {
        return Err(anyhow::anyhow!(
            "Connection interval too long: {}h (maximum: 168h/1 week)",
            config.connection_interval_hours
        ));
    }
    
    if config.max_retry_attempts > 10 {
        return Err(anyhow::anyhow!(
            "Too many retry attempts: {} (maximum: 10)",
            config.max_retry_attempts
        ));
    }
    
    if config.connection_timeout_s < 10 {
        return Err(anyhow::anyhow!(
            "Connection timeout too short: {}s (minimum: 10s)",
            config.connection_timeout_s
        ));
    }
    
    if config.connection_timeout_s > 300 {
        return Err(anyhow::anyhow!(
            "Connection timeout too long: {}s (maximum: 300s)",
            config.connection_timeout_s
        ));
    }
    
    Ok(())
}

fn validate_emergency_config(config: &shared_positioning::EmergencyConfig) -> Result<()> {
    if config.emergency_interval_ms < 500 {
        return Err(anyhow::anyhow!(
            "Emergency transmission interval too short: {}ms (minimum: 500ms)",
            config.emergency_interval_ms
        ));
    }
    
    if config.emergency_interval_ms > 10000 {
        return Err(anyhow::anyhow!(
            "Emergency transmission interval too long: {}ms (maximum: 10000ms)",
            config.emergency_interval_ms
        ));
    }
    
    if config.shutdown_conditions.shutdown_grace_period_s < 5 {
        return Err(anyhow::anyhow!(
            "Shutdown delay too short: {}s (minimum: 5s)",
            config.shutdown_conditions.shutdown_grace_period_s
        ));
    }
    
    if config.shutdown_conditions.shutdown_grace_period_s > 300 {
        return Err(anyhow::anyhow!(
            "Shutdown delay too long: {}s (maximum: 300s)",
            config.shutdown_conditions.shutdown_grace_period_s
        ));
    }
    
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::NamedTempFile;
    use uuid::Uuid;
    
    #[tokio::test]
    async fn test_config_save_load_toml() {
        let temp_file = NamedTempFile::new().unwrap();
        let config_path = temp_file.path().with_extension("toml");
        
        let config_manager = ConfigManager::new(config_path);
        
        let original_config = BeaconConfig {
            beacon_id: Uuid::new_v4(),
            transmission_interval_ms: 5000,
            message_version: crate::beacon_controller::MessageVersion::V3,
            gps_config: shared_positioning::GpsConfig::default(),
            power_config: shared_positioning::PowerConfig::default(),
            communication_config: shared_positioning::CommunicationConfig::default(),
            emergency_config: crate::beacon_controller::EmergencyConfig::default(),
        };
        
        // Save configuration
        config_manager.save_config(&original_config).await.unwrap();
        
        // Load configuration
        let loaded_config = config_manager.load_config().await.unwrap();
        
        assert_eq!(original_config.beacon_id, loaded_config.beacon_id);
        assert_eq!(original_config.transmission_interval_ms, loaded_config.transmission_interval_ms);
    }
    
    #[test]
    fn test_config_validation() {
        let mut config = BeaconConfig {
            beacon_id: Uuid::new_v4(),
            transmission_interval_ms: 5000,
            message_version: crate::beacon_controller::MessageVersion::V3,
            gps_config: shared_positioning::GpsConfig::default(),
            power_config: shared_positioning::PowerConfig::default(),
            communication_config: shared_positioning::CommunicationConfig::default(),
            emergency_config: crate::beacon_controller::EmergencyConfig::default(),
        };
        
        // Valid configuration should pass
        assert!(validate_config(&config).is_ok());
        
        // Invalid transmission interval should fail
        config.transmission_interval_ms = 500;
        assert!(validate_config(&config).is_err());
        
        // Nil beacon ID should fail
        config.transmission_interval_ms = 5000;
        config.beacon_id = Uuid::nil();
        assert!(validate_config(&config).is_err());
    }
}