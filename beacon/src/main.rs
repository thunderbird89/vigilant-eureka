use anyhow::{Context, Result};
use clap::Parser;
use std::path::PathBuf;
use std::sync::Arc;
use tokio::sync::RwLock;
use tokio::signal;
use tracing::{error, info};

mod beacon_controller;
mod cli;
mod config;
mod deployment;
mod signal_handler;

use beacon_controller::BeaconController;
use shared_positioning::BeaconConfig;
use config::ConfigManager;

// Type alias for the concrete beacon controller type
type ConcreteBeaconController = BeaconController<
    shared_positioning::MockGpsManager,
    shared_positioning::MockPowerManager,
    shared_positioning::MockCommunicationManager,
    shared_positioning::MockTransceiver,
>;
use cli::CliCommands;
use signal_handler::SignalHandler;
use shared_positioning::{
    MockGpsManager, MockPowerManager, MockCommunicationManager, MockTransceiver,
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
        CliCommands::Deploy(deploy_cmd) => {
            deploy_cmd.execute(config_path).await
        }
    }
}

async fn validate_config_file(config_path: &PathBuf) -> Result<()> {
    info!("Validating configuration file: {}", config_path.display());
    
    let config = BeaconConfig::load_from_file(config_path)
        .map_err(|e| anyhow::anyhow!("Failed to load configuration: {}", e))?;
    
    match config.validate() {
        Ok(()) => {
            println!("✓ Configuration is valid");
            println!("  Beacon ID: {}", config.beacon_id);
            println!("  Transmission interval: {}ms", config.transmission.interval_ms);
            println!("  Message version: {:?}", config.transmission.message_version);
            Ok(())
        }
        Err(e) => {
            println!("✗ Configuration validation failed: {}", e);
            Err(anyhow::anyhow!("Configuration validation failed: {}", e))
        }
    }
}

async fn generate_default_config(output_path: &PathBuf) -> Result<()> {
    info!("Generating default configuration: {}", output_path.display());
    
    let beacon_id = uuid::Uuid::new_v4();
    let mut default_config = BeaconConfig::new(beacon_id);
    
    default_config.save_to_file(output_path)
        .map_err(|e| anyhow::anyhow!("Failed to save default configuration: {}", e))?;
    
    println!("✓ Default configuration generated: {}", output_path.display());
    println!("  Beacon ID: {}", default_config.beacon_id);
    
    Ok(())
}

async fn run_beacon_service(config_path: &PathBuf) -> Result<()> {
    info!("Starting beacon service with config: {}", config_path.display());
    
    // Load configuration
    let config = BeaconConfig::load_from_file(config_path)
        .map_err(|e| anyhow::anyhow!("Failed to load configuration: {}", e))?;
    
    // Validate configuration
    config.validate()
        .map_err(|e| anyhow::anyhow!("Configuration validation failed: {}", e))?;
    
    info!("Configuration loaded and validated successfully");
    info!("Beacon ID: {}", config.beacon_id);
    
    // Create beacon controller
    let beacon_controller = create_beacon_controller(config).await
        .context("Failed to create beacon controller")?;
    
    let beacon_controller: Arc<RwLock<ConcreteBeaconController>> = Arc::new(RwLock::new(beacon_controller));
    
    // Create config manager
    let config_manager = ConfigManager::new(config_path.clone());
    
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