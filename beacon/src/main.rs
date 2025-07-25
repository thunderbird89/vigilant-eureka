use anyhow::{Context, Result};
use clap::{Parser, Subcommand};
use serde::{Deserialize, Serialize};
use std::path::PathBuf;
use std::sync::Arc;
use tokio::sync::RwLock;
use tokio::signal;
use tracing::{error, info, warn};
use uuid::Uuid;

mod beacon_controller;
mod cli;
mod config;
mod signal_handler;

use beacon_controller::{BeaconController, BeaconConfig, MessageVersion, EmergencyConfig};

// Type alias for the concrete beacon controller type
type ConcreteBeaconController = BeaconController<
    shared_positioning::MockGpsManager,
    shared_positioning::MockPowerManager,
    shared_positioning::MockCommunicationManager,
    shared_positioning::MockTransceiver,
>;
use cli::{CliCommands, StatusCommand, DiagnosticCommand};
use config::{ConfigManager, validate_config};
use signal_handler::SignalHandler;
use shared_positioning::{
    MockGpsManager, MockPowerManager, MockCommunicationManager, MockTransceiver,
    GpsConfig, PowerConfig, CommunicationConfig,
};

#[derive(Parser)]
#[command(name = "beacon")]
#[command(about = "Underwater positioning beacon system")]
#[command(version = "0.1.0")]
struct Cli {
    /// Configuration file path
    #[arg(short, long, default_value = "beacon.toml")]
    config: PathBuf,
    
    /// Log level (trace, debug, info, warn, error)
    #[arg(short, long, default_value = "info")]
    log_level: String,
    
    /// Enable verbose output
    #[arg(short, long)]
    verbose: bool,
    
    #[command(subcommand)]
    command: Option<CliCommands>,
}

#[tokio::main]
async fn main() -> Result<()> {
    let cli = Cli::parse();
    
    // Initialize logging
    init_logging(&cli.log_level, cli.verbose)?;
    
    info!("Starting beacon application");
    
    match cli.command {
        Some(command) => {
            // Handle CLI commands
            handle_cli_command(command, &cli.config).await
        }
        None => {
            // Run main beacon service
            run_beacon_service(&cli.config).await
        }
    }
}

fn init_logging(level: &str, verbose: bool) -> Result<()> {
    let level = match level.to_lowercase().as_str() {
        "trace" => tracing::Level::TRACE,
        "debug" => tracing::Level::DEBUG,
        "info" => tracing::Level::INFO,
        "warn" => tracing::Level::WARN,
        "error" => tracing::Level::ERROR,
        _ => return Err(anyhow::anyhow!("Invalid log level: {}", level)),
    };
    
    let subscriber = tracing_subscriber::fmt()
        .with_max_level(level)
        .with_target(verbose)
        .with_thread_ids(verbose)
        .with_file(verbose)
        .with_line_number(verbose)
        .finish();
    
    tracing::subscriber::set_global_default(subscriber)
        .context("Failed to set tracing subscriber")?;
    
    Ok(())
}

async fn handle_cli_command(command: CliCommands, config_path: &PathBuf) -> Result<()> {
    match command {
        CliCommands::Status(status_cmd) => {
            status_cmd.execute(config_path).await
        }
        CliCommands::Diagnostic(diag_cmd) => {
            diag_cmd.execute(config_path).await
        }
        CliCommands::ValidateConfig => {
            validate_config_file(config_path).await
        }
        CliCommands::GenerateConfig { output } => {
            generate_default_config(&output).await
        }
    }
}

async fn validate_config_file(config_path: &PathBuf) -> Result<()> {
    info!("Validating configuration file: {}", config_path.display());
    
    let config_manager = ConfigManager::new(config_path.clone());
    let config = config_manager.load_config().await
        .context("Failed to load configuration")?;
    
    match validate_config(&config) {
        Ok(()) => {
            println!("✓ Configuration is valid");
            println!("  Beacon ID: {}", config.beacon_id);
            println!("  Transmission interval: {}ms", config.transmission_interval_ms);
            println!("  Message version: {:?}", config.message_version);
            Ok(())
        }
        Err(e) => {
            println!("✗ Configuration validation failed: {}", e);
            Err(e)
        }
    }
}

async fn generate_default_config(output_path: &PathBuf) -> Result<()> {
    info!("Generating default configuration: {}", output_path.display());
    
    let default_config = BeaconConfig {
        beacon_id: Uuid::new_v4(),
        transmission_interval_ms: 5000,
        message_version: MessageVersion::V3,
        gps_config: GpsConfig::default(),
        power_config: PowerConfig::default(),
        communication_config: CommunicationConfig::default(),
        emergency_config: EmergencyConfig::default(),
    };
    
    let config_manager = ConfigManager::new(output_path.clone());
    config_manager.save_config(&default_config).await
        .context("Failed to save default configuration")?;
    
    println!("✓ Default configuration generated: {}", output_path.display());
    println!("  Beacon ID: {}", default_config.beacon_id);
    
    Ok(())
}

async fn run_beacon_service(config_path: &PathBuf) -> Result<()> {
    info!("Starting beacon service with config: {}", config_path.display());
    
    // Load configuration
    let config_manager = ConfigManager::new(config_path.clone());
    let config = config_manager.load_config().await
        .context("Failed to load configuration")?;
    
    // Validate configuration
    validate_config(&config)
        .context("Configuration validation failed")?;
    
    info!("Configuration loaded and validated successfully");
    info!("Beacon ID: {}", config.beacon_id);
    
    // Create beacon controller
    let beacon_controller = create_beacon_controller(config).await
        .context("Failed to create beacon controller")?;
    
    let beacon_controller: Arc<RwLock<ConcreteBeaconController>> = Arc::new(RwLock::new(beacon_controller));
    
    // Set up signal handling
    let signal_handler = SignalHandler::new(
        beacon_controller.clone(),
        config_manager,
    );
    
    // Start beacon controller
    {
        let mut controller = beacon_controller.write().await;
        controller.start()
            .context("Failed to start beacon controller")?;
    }
    
    info!("Beacon controller started successfully");
    
    // Run main service loop with signal handling
    tokio::select! {
        result = signal_handler.run() => {
            match result {
                Ok(()) => info!("Signal handler completed successfully"),
                Err(e) => error!("Signal handler error: {}", e),
            }
        }
        _ = signal::ctrl_c() => {
            info!("Received Ctrl+C, initiating graceful shutdown");
        }
    }
    
    // Graceful shutdown
    info!("Initiating graceful shutdown");
    {
        let mut controller = beacon_controller.write().await;
        if let Err(e) = controller.stop() {
            error!("Error during beacon controller shutdown: {}", e);
        }
    }
    
    info!("Beacon service shutdown complete");
    Ok(())
}

async fn create_beacon_controller(config: BeaconConfig) -> Result<ConcreteBeaconController> {
    // Create mock managers for demonstration
    // In a real implementation, these would be actual hardware interfaces
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