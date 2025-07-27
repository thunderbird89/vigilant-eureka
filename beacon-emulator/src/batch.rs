use serde::{Deserialize, Serialize};
use std::path::PathBuf;
use std::time::Duration;
use uuid::Uuid;
use shared_positioning::GeodeticPosition;
use crate::{EmulatorManager, EmulatorError, MovementPattern, ScenarioType, ExportFormat};
use tokio::time::sleep;
use tracing::{info, warn, error, debug};

/// Batch operation configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BatchConfig {
    pub operations: Vec<BatchOperation>,
    #[serde(default)]
    pub settings: BatchSettings,
}

/// Settings for batch execution
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BatchSettings {
    #[serde(default = "default_timeout")]
    pub timeout_seconds: u64,
    #[serde(default)]
    pub continue_on_error: bool,
    #[serde(default)]
    pub verbose: bool,
}

impl Default for BatchSettings {
    fn default() -> Self {
        Self {
            timeout_seconds: default_timeout(),
            continue_on_error: false,
            verbose: false,
        }
    }
}

fn default_timeout() -> u64 {
    600
}

/// Individual batch operation
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum BatchOperation {
    CreateBeacon {
        id: Option<Uuid>,
        position: [f64; 3], // [lat, lon, depth]
        #[serde(default = "default_interval")]
        interval: u32,
        #[serde(default)]
        movement: MovementPattern,
        #[serde(default)]
        start: bool,
        config_file: Option<PathBuf>,
    },
    CreateScenario {
        scenario_type: ScenarioType,
        count: usize,
        spacing: f64,
        center: [f64; 3], // [lat, lon, depth]
        #[serde(default)]
        start_all: bool,
        #[serde(default = "default_interval")]
        interval: u32,
    },
    StartBeacon {
        id: Uuid,
    },
    StartAllBeacons,
    StopBeacon {
        id: Uuid,
        #[serde(default)]
        remove: bool,
    },
    StopAllBeacons {
        #[serde(default)]
        remove: bool,
    },
    UpdateBeacon {
        id: Uuid,
        position: Option<[f64; 3]>,
        interval: Option<u32>,
        movement: Option<MovementPattern>,
        #[serde(default)]
        restart: bool,
    },
    RemoveBeacon {
        id: Uuid,
        #[serde(default)]
        force: bool,
    },
    ExportLogs {
        output: PathBuf,
        #[serde(default)]
        format: ExportFormat,
        #[serde(default = "default_duration")]
        duration: u64,
        #[serde(default)]
        all_time: bool,
        beacon: Option<Uuid>,
        #[serde(default)]
        include_messages: bool,
    },
    Wait {
        duration: u64, // seconds
    },
    ClearState {
        #[serde(default)]
        confirm: bool,
    },
}

fn default_interval() -> u32 {
    5000
}

fn default_duration() -> u64 {
    3600
}

/// Result of batch operation execution
#[derive(Debug, Clone)]
pub struct BatchExecutionResult {
    pub total_operations: usize,
    pub successful_operations: usize,
    pub failed_operations: usize,
    pub execution_time: Duration,
    pub operation_results: Vec<OperationResult>,
}

/// Result of individual operation
#[derive(Debug, Clone)]
pub struct OperationResult {
    pub operation_index: usize,
    pub operation_type: String,
    pub success: bool,
    pub message: String,
    pub execution_time: Duration,
}

/// Batch operations executor
pub struct BatchExecutor<'a> {
    emulator: &'a mut EmulatorManager,
    verbose: bool,
}

impl<'a> BatchExecutor<'a> {
    pub fn new(emulator: &'a mut EmulatorManager) -> Self {
        Self {
            emulator,
            verbose: false,
        }
    }

    pub fn with_verbose(mut self, verbose: bool) -> Self {
        self.verbose = verbose;
        self
    }

    /// Execute batch operations from configuration
    pub async fn execute_batch(&mut self, config: BatchConfig) -> Result<BatchExecutionResult, EmulatorError> {
        let start_time = std::time::Instant::now();
        let total_operations = config.operations.len();
        let mut operation_results = Vec::new();
        let mut successful_operations = 0;
        let mut failed_operations = 0;

        info!("Starting batch execution with {} operations", total_operations);

        for (index, operation) in config.operations.into_iter().enumerate() {
            let op_start_time = std::time::Instant::now();
            let operation_type = self.get_operation_type(&operation);

            if self.verbose {
                info!("Executing operation {}/{}: {}", index + 1, total_operations, operation_type);
            }

            let result = self.execute_operation(operation).await;
            let execution_time = op_start_time.elapsed();

            let operation_result = match result {
                Ok(message) => {
                    successful_operations += 1;
                    if self.verbose {
                        info!("Operation {} completed successfully: {}", index + 1, message);
                    }
                    OperationResult {
                        operation_index: index,
                        operation_type: operation_type.clone(),
                        success: true,
                        message,
                        execution_time,
                    }
                }
                Err(e) => {
                    failed_operations += 1;
                    let error_message = format!("Operation failed: {}", e);
                    
                    if config.settings.continue_on_error {
                        warn!("Operation {} failed but continuing: {}", index + 1, error_message);
                    } else {
                        error!("Operation {} failed, stopping batch execution: {}", index + 1, error_message);
                        operation_results.push(OperationResult {
                            operation_index: index,
                            operation_type,
                            success: false,
                            message: error_message,
                            execution_time,
                        });
                        break;
                    }

                    OperationResult {
                        operation_index: index,
                        operation_type,
                        success: false,
                        message: error_message,
                        execution_time,
                    }
                }
            };

            operation_results.push(operation_result);
        }

        let total_execution_time = start_time.elapsed();

        info!("Batch execution completed: {}/{} operations successful", 
              successful_operations, total_operations);

        Ok(BatchExecutionResult {
            total_operations,
            successful_operations,
            failed_operations,
            execution_time: total_execution_time,
            operation_results,
        })
    }

    /// Validate batch configuration without executing
    pub fn validate_batch(&self, config: &BatchConfig) -> Result<Vec<String>, EmulatorError> {
        let mut warnings = Vec::new();

        for (index, operation) in config.operations.iter().enumerate() {
            match self.validate_operation(operation) {
                Ok(Some(warning)) => warnings.push(format!("Operation {}: {}", index + 1, warning)),
                Err(e) => return Err(EmulatorError::ConfigError(
                    format!("Operation {} validation failed: {}", index + 1, e)
                )),
                _ => {}
            }
        }

        Ok(warnings)
    }

    fn get_operation_type(&self, operation: &BatchOperation) -> String {
        match operation {
            BatchOperation::CreateBeacon { .. } => "create_beacon".to_string(),
            BatchOperation::CreateScenario { .. } => "create_scenario".to_string(),
            BatchOperation::StartBeacon { .. } => "start_beacon".to_string(),
            BatchOperation::StartAllBeacons => "start_all_beacons".to_string(),
            BatchOperation::StopBeacon { .. } => "stop_beacon".to_string(),
            BatchOperation::StopAllBeacons { .. } => "stop_all_beacons".to_string(),
            BatchOperation::UpdateBeacon { .. } => "update_beacon".to_string(),
            BatchOperation::RemoveBeacon { .. } => "remove_beacon".to_string(),
            BatchOperation::ExportLogs { .. } => "export_logs".to_string(),
            BatchOperation::Wait { .. } => "wait".to_string(),
            BatchOperation::ClearState { .. } => "clear_state".to_string(),
        }
    }

    async fn execute_operation(&mut self, operation: BatchOperation) -> Result<String, EmulatorError> {
        match operation {
            BatchOperation::CreateBeacon { id, position, interval: _, movement, start, config_file } => {
                let pos = GeodeticPosition {
                    latitude: position[0],
                    longitude: position[1],
                    depth: position[2],
                };

                let config = if let Some(config_path) = config_file {
                    Some(self.emulator.load_beacon_config(&config_path).await?)
                } else {
                    None
                };

                let beacon_id = self.emulator.create_beacon(id, pos, config).await?;
                self.emulator.update_beacon_movement_pattern(beacon_id, movement).await?;

                if start {
                    self.emulator.start_beacon(beacon_id).await?;
                    Ok(format!("Created and started beacon {}", beacon_id))
                } else {
                    Ok(format!("Created beacon {}", beacon_id))
                }
            }

            BatchOperation::CreateScenario { scenario_type, count, spacing, center, start_all, interval: _ } => {
                let center_pos = GeodeticPosition {
                    latitude: center[0],
                    longitude: center[1],
                    depth: center[2],
                };

                let beacon_ids = self.emulator.create_scenario(scenario_type.clone(), count, spacing, center_pos).await?;

                if start_all {
                    let mut started_count = 0;
                    for beacon_id in &beacon_ids {
                        if self.emulator.start_beacon(*beacon_id).await.is_ok() {
                            started_count += 1;
                        }
                    }
                    Ok(format!("Created {} scenario with {} beacons, started {}", 
                              scenario_type, beacon_ids.len(), started_count))
                } else {
                    Ok(format!("Created {} scenario with {} beacons", scenario_type, beacon_ids.len()))
                }
            }

            BatchOperation::StartBeacon { id } => {
                self.emulator.start_beacon(id).await?;
                Ok(format!("Started beacon {}", id))
            }

            BatchOperation::StartAllBeacons => {
                let started_beacons = self.emulator.start_all_beacons().await?;
                Ok(format!("Started {} beacons", started_beacons.len()))
            }

            BatchOperation::StopBeacon { id, remove } => {
                self.emulator.stop_beacon(id).await?;
                if remove {
                    self.emulator.remove_beacon(id).await?;
                    Ok(format!("Stopped and removed beacon {}", id))
                } else {
                    Ok(format!("Stopped beacon {}", id))
                }
            }

            BatchOperation::StopAllBeacons { remove } => {
                let stopped_beacons = self.emulator.stop_all_beacons().await?;
                if remove {
                    let mut removed_count = 0;
                    for beacon_id in &stopped_beacons {
                        if self.emulator.remove_beacon(*beacon_id).await.is_ok() {
                            removed_count += 1;
                        }
                    }
                    Ok(format!("Stopped {} beacons and removed {}", stopped_beacons.len(), removed_count))
                } else {
                    Ok(format!("Stopped {} beacons", stopped_beacons.len()))
                }
            }

            BatchOperation::UpdateBeacon { id, position, interval, movement, restart } => {
                let beacon_status = self.emulator.get_beacon_status(id)?;
                let was_running = beacon_status.is_running;

                if let Some(pos) = position {
                    let new_position = GeodeticPosition {
                        latitude: pos[0],
                        longitude: pos[1],
                        depth: pos[2],
                    };
                    self.emulator.update_beacon_position(id, new_position).await?;
                }

                if let Some(new_movement) = movement {
                    self.emulator.update_beacon_movement_pattern(id, new_movement).await?;
                }

                if let Some(new_interval) = interval {
                    let mut config = beacon_status.config;
                    config.transmission.interval_ms = new_interval;
                    self.emulator.update_beacon_config(id, config).await?;
                }

                if restart && was_running {
                    self.emulator.stop_beacon(id).await?;
                    sleep(Duration::from_millis(100)).await;
                    self.emulator.start_beacon(id).await?;
                }

                Ok(format!("Updated beacon {}", id))
            }

            BatchOperation::RemoveBeacon { id, force } => {
                if force {
                    let status = self.emulator.get_beacon_status(id)?;
                    if status.is_running {
                        self.emulator.stop_beacon(id).await?;
                    }
                }
                self.emulator.remove_beacon(id).await?;
                Ok(format!("Removed beacon {}", id))
            }

            BatchOperation::ExportLogs { output, format, duration, all_time, beacon, include_messages } => {
                let exported_count = self.emulator.export_logs(
                    &output, format, duration, all_time, beacon, include_messages
                ).await?;
                Ok(format!("Exported {} log entries to {}", exported_count, output.display()))
            }

            BatchOperation::Wait { duration } => {
                debug!("Waiting for {} seconds", duration);
                sleep(Duration::from_secs(duration)).await;
                Ok(format!("Waited {} seconds", duration))
            }

            BatchOperation::ClearState { confirm: _ } => {
                let beacon_count = self.emulator.get_total_beacon_count();
                self.emulator.clear_state().await?;
                Ok(format!("Cleared {} beacons and reset state", beacon_count))
            }
        }
    }

    fn validate_operation(&self, operation: &BatchOperation) -> Result<Option<String>, EmulatorError> {
        match operation {
            BatchOperation::CreateBeacon { position, interval, .. } => {
                // Validate position
                if position[0] < -90.0 || position[0] > 90.0 {
                    return Err(EmulatorError::ConfigError("Invalid latitude".to_string()));
                }
                if position[1] < -180.0 || position[1] > 180.0 {
                    return Err(EmulatorError::ConfigError("Invalid longitude".to_string()));
                }
                if position[2] < 0.0 || position[2] > 11000.0 {
                    return Err(EmulatorError::ConfigError("Invalid depth".to_string()));
                }

                // Validate interval
                if *interval < 100 || *interval > 300_000 {
                    return Err(EmulatorError::ConfigError("Invalid transmission interval".to_string()));
                }

                Ok(None)
            }

            BatchOperation::CreateScenario { count, spacing, center, .. } => {
                // Validate center position
                if center[0] < -90.0 || center[0] > 90.0 {
                    return Err(EmulatorError::ConfigError("Invalid center latitude".to_string()));
                }
                if center[1] < -180.0 || center[1] > 180.0 {
                    return Err(EmulatorError::ConfigError("Invalid center longitude".to_string()));
                }
                if center[2] < 0.0 || center[2] > 11000.0 {
                    return Err(EmulatorError::ConfigError("Invalid center depth".to_string()));
                }

                // Validate parameters
                if *count == 0 || *count > 100 {
                    return Err(EmulatorError::ConfigError("Invalid beacon count".to_string()));
                }
                if *spacing <= 0.0 || *spacing > 10000.0 {
                    return Err(EmulatorError::ConfigError("Invalid spacing".to_string()));
                }

                Ok(None)
            }

            BatchOperation::Wait { duration } => {
                if *duration > 3600 {
                    Ok(Some("Wait duration longer than 1 hour".to_string()))
                } else {
                    Ok(None)
                }
            }

            _ => Ok(None),
        }
    }
}

/// Load batch configuration from file
pub async fn load_batch_config(file_path: &PathBuf) -> Result<BatchConfig, EmulatorError> {
    let content = tokio::fs::read_to_string(file_path).await
        .map_err(|e| EmulatorError::IoError(e))?;

    let config: BatchConfig = serde_json::from_str(&content)
        .map_err(|e| EmulatorError::ConfigError(format!("Invalid batch configuration: {}", e)))?;

    Ok(config)
}

/// Generate example batch configuration
pub fn generate_example_batch_config() -> BatchConfig {
    BatchConfig {
        operations: vec![
            BatchOperation::CreateBeacon {
                id: None,
                position: [32.123, 45.476, 10.0],
                interval: 5000,
                movement: MovementPattern::Stationary,
                start: true,
                config_file: None,
            },
            BatchOperation::CreateScenario {
                scenario_type: ScenarioType::Triangle,
                count: 3,
                spacing: 100.0,
                center: [32.0, 45.0, 15.0],
                start_all: true,
                interval: 5000,
            },
            BatchOperation::Wait { duration: 10 },
            BatchOperation::ExportLogs {
                output: PathBuf::from("test_results.json"),
                format: ExportFormat::Json,
                duration: 60,
                all_time: false,
                beacon: None,
                include_messages: true,
            },
            BatchOperation::StopAllBeacons { remove: true },
            BatchOperation::ClearState { confirm: true },
        ],
        settings: BatchSettings::default(),
    }
}