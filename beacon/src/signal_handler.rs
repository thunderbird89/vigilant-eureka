use anyhow::{Context, Result};
use std::sync::Arc;
use tokio::sync::RwLock;
use tracing::{error, info, warn};

#[cfg(unix)]
use signal_hook::consts::signal::*;
#[cfg(unix)]
use signal_hook_tokio::Signals;
#[cfg(unix)]
use tokio_stream::StreamExt;

use crate::beacon_controller::BeaconController;
use crate::config::{ConfigManager, validate_config};

// Type alias for the concrete beacon controller type
type ConcreteBeaconController = BeaconController<
    shared_positioning::MockGpsManager,
    shared_positioning::MockPowerManager,
    shared_positioning::MockCommunicationManager,
    shared_positioning::MockTransceiver,
>;

pub struct SignalHandler {
    beacon_controller: Arc<RwLock<ConcreteBeaconController>>,
    config_manager: ConfigManager,
}

impl SignalHandler {
    pub fn new(
        beacon_controller: Arc<RwLock<ConcreteBeaconController>>,
        config_manager: ConfigManager,
    ) -> Self {
        Self {
            beacon_controller,
            config_manager,
        }
    }
    
    pub async fn run(&self) -> Result<()> {
        info!("Signal handler started");
        
        #[cfg(unix)]
        {
            self.run_unix_signals().await
        }
        
        #[cfg(windows)]
        {
            self.run_windows_signals().await
        }
    }
    
    #[cfg(unix)]
    async fn run_unix_signals(&self) -> Result<()> {
        // Set up signal handling for SIGTERM, SIGINT, and SIGHUP
        let mut signals = Signals::new(&[SIGTERM, SIGINT, SIGHUP])
            .context("Failed to create signal handler")?;
        
        while let Some(signal) = signals.next().await {
            match signal {
                SIGTERM | SIGINT => {
                    info!("Received shutdown signal ({}), initiating graceful shutdown", signal);
                    self.handle_shutdown().await?;
                    break;
                }
                SIGHUP => {
                    info!("Received SIGHUP, reloading configuration");
                    if let Err(e) = self.handle_config_reload().await {
                        error!("Failed to reload configuration: {}", e);
                    }
                }
                _ => {
                    warn!("Received unexpected signal: {}", signal);
                }
            }
        }
        
        info!("Signal handler completed");
        Ok(())
    }
    
    #[cfg(windows)]
    async fn run_windows_signals(&self) -> Result<()> {
        // On Windows, we primarily handle Ctrl+C
        tokio::select! {
            _ = tokio::signal::ctrl_c() => {
                info!("Received Ctrl+C, initiating graceful shutdown");
                self.handle_shutdown().await?;
            }
        }
        
        info!("Signal handler completed");
        Ok(())
    }
    
    async fn handle_shutdown(&self) -> Result<()> {
        info!("Initiating graceful shutdown sequence");
        
        // Stop the beacon controller
        {
            let mut controller = self.beacon_controller.write().await;
            if let Err(e) = controller.stop() {
                error!("Error stopping beacon controller: {}", e);
            } else {
                info!("Beacon controller stopped successfully");
            }
        }
        
        // Give some time for cleanup
        tokio::time::sleep(tokio::time::Duration::from_millis(500)).await;
        
        info!("Graceful shutdown completed");
        Ok(())
    }
    
    async fn handle_config_reload(&self) -> Result<()> {
        info!("Reloading configuration from: {}", self.config_manager.get_config_path().display());
        
        // Load new configuration
        let new_config = self.config_manager.reload_config().await
            .context("Failed to reload configuration file")?;
        
        // Validate new configuration
        validate_config(&new_config)
            .context("New configuration validation failed")?;
        
        info!("New configuration loaded and validated successfully");
        
        // Update beacon controller configuration
        {
            let mut controller = self.beacon_controller.write().await;
            controller.update_configuration(new_config)
                .context("Failed to update beacon controller configuration")?;
        }
        
        info!("Configuration reloaded successfully");
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::NamedTempFile;
    use uuid::Uuid;
    
    #[tokio::test]
    async fn test_config_reload() {
        // Create temporary config file
        let temp_file = NamedTempFile::new().unwrap();
        let config_path = temp_file.path().with_extension("toml");
        
        let config_manager = ConfigManager::new(config_path);
        
        // Create initial configuration
        let initial_config = shared_positioning::BeaconConfig {
            beacon_id: Uuid::new_v4(),
            transmission: shared_positioning::beacon_config::TransmissionConfig {
                interval_ms: 5000,
                message_version: shared_positioning::beacon_config::MessageVersion::V3,
                power_level: 128,
                max_retries: 3,
                retry_delay_ms: 1000,
                adaptive_power: true,
                sequence_rollover: 65535,
            },
            gps: shared_positioning::beacon_config::GpsConfig::default(),
            power: shared_positioning::beacon_config::PowerConfig::default(),
            communication: shared_positioning::beacon_config::CommunicationConfig::default(),
            emergency: shared_positioning::beacon_config::EmergencyConfig::default(),
            hardware: shared_positioning::beacon_config::HardwareConfig::default(),
            metadata: shared_positioning::beacon_config::BeaconConfigMetadata::default(),
        };
        
        // Save initial configuration
        config_manager.save_config(&initial_config).await.unwrap();
        
        // Create beacon controller
        let gps_manager = shared_positioning::MockGpsManager::with_test_positions(
            shared_positioning::GpsConfig::default()
        ).unwrap();
        let power_manager = shared_positioning::MockPowerManager::new();
        let communication_manager = shared_positioning::MockCommunicationManager::new();
        let transceiver = shared_positioning::MockTransceiver::new(1);
        
        let beacon_controller: ConcreteBeaconController = crate::beacon_controller::BeaconController::new(
            initial_config,
            gps_manager,
            power_manager,
            communication_manager,
            transceiver,
        ).unwrap();
        
        let beacon_controller = Arc::new(RwLock::new(beacon_controller));
        
        // Create signal handler
        let signal_handler = SignalHandler::new(beacon_controller.clone(), config_manager);
        
        // Test configuration reload
        let result = signal_handler.handle_config_reload().await;
        assert!(result.is_ok(), "Config reload should succeed: {:?}", result);
    }
}