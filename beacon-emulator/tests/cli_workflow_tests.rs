//! End-to-end CLI workflow tests
//! 
//! This module contains tests that simulate complete CLI workflows and
//! scenario management operations.

use beacon_emulator::*;
use beacon_emulator::cli::{Cli, EmulatorCommand, MessageVersion, ListFormat, ConfigFormat as CliConfigFormat};
use shared_positioning::GeodeticPosition;
use tokio::time::{sleep, Duration};
use tempfile::TempDir;

/// Test utilities for CLI workflow tests
mod cli_test_utils {
    use super::*;
    
    pub fn create_test_position() -> GeodeticPosition {
        GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        }
    }
    
    pub async fn create_test_manager_with_temp_dir() -> (EmulatorManager, TempDir) {
        let temp_dir = TempDir::new().unwrap();
        let state_file = temp_dir.path().join("test_state.json");
        
        let mut manager = EmulatorManager::new("test_channel");
        manager.set_state_file_path(state_file);
        
        (manager, temp_dir)
    }
    
    pub fn create_test_cli_args() -> Cli {
        Cli {
            channel: "test_channel".to_string(),
            log_level: "info".to_string(),
            state_file: None,
            ipc_port: 8765,
            quiet: false,
            non_interactive: false,
            automation_mode: false,
            command: EmulatorCommand::List {
                detailed: false,
                running_only: false,
                format: ListFormat::Table,
            },
        }
    }
    
    /// Simulate CLI argument parsing for create command
    pub fn create_beacon_command(
        lat: f64,
        lon: f64,
        depth: f64,
        movement: Option<MovementPattern>,
        start: bool,
    ) -> EmulatorCommand {
        EmulatorCommand::Create {
            id: None,
            lat,
            lon,
            depth,
            config: None,
            interval: 5000,
            version: MessageVersion::V3,
            movement: movement.unwrap_or(MovementPattern::Stationary),
            start,
        }
    }
    
    /// Simulate CLI argument parsing for scenario command
    pub fn create_scenario_command(
        scenario_type: ScenarioType,
        count: usize,
        spacing: f64,
        center: GeodeticPosition,
        start_all: bool,
    ) -> EmulatorCommand {
        EmulatorCommand::Scenario {
            scenario_type,
            count,
            spacing,
            center,
            start_all,
            interval: 5000,
        }
    }
}

/// End-to-end tests for complete CLI workflows
mod cli_workflow_tests {
    use super::*;
    use cli_test_utils::*;
    
    #[tokio::test]
    async fn test_create_list_workflow() {
        let (mut manager, _temp_dir) = create_test_manager_with_temp_dir().await;
        let position = create_test_position();
        
        // Simulate CLI create command
        let create_cmd = create_beacon_command(
            position.latitude,
            position.longitude,
            position.depth,
            None,
            false,
        );
        
        // Execute create command
        let beacon_id = match create_cmd {
            EmulatorCommand::Create { lat, lon, depth, start, .. } => {
                let pos = GeodeticPosition {
                    latitude: lat,
                    longitude: lon,
                    depth,
                };
                let id = manager.create_beacon(None, pos, None).await.unwrap();
                if start {
                    manager.start_beacon(id).await.unwrap();
                }
                id
            }
            _ => panic!("Expected Create command"),
        };
        
        // Verify beacon was created
        assert!(manager.beacon_exists(beacon_id));
        assert_eq!(manager.get_total_beacon_count(), 1);
        
        // Simulate CLI list command
        let list_cmd = EmulatorCommand::List {
            detailed: true,
            running_only: false,
            format: ListFormat::Table,
        };
        
        // Execute list command
        match list_cmd {
            EmulatorCommand::List { detailed, running_only, .. } => {
                let beacons = manager.list_beacons();
                assert_eq!(beacons.len(), 1);
                
                let beacon_status = &beacons[0];
                assert_eq!(beacon_status.id, beacon_id);
                assert_eq!(beacon_status.position.latitude, position.latitude);
                
                if running_only {
                    // Would filter only running beacons
                    let running_count = beacons.iter().filter(|b| b.is_running).count();
                    assert_eq!(running_count, 0); // None running yet
                }
                
                if detailed {
                    // Would show detailed information
                    assert!(beacon_status.stats.messages_sent == 0);
                    assert!(beacon_status.stats.last_transmission.is_none());
                }
            }
            _ => panic!("Expected List command"),
        }
    }
    
    #[tokio::test]
    async fn test_create_start_stop_workflow() {
        let (mut manager, _temp_dir) = create_test_manager_with_temp_dir().await;
        let position = create_test_position();
        
        // Create beacon with auto-start
        let create_cmd = create_beacon_command(
            position.latitude,
            position.longitude,
            position.depth,
            Some(MovementPattern::Linear {
                speed_m_per_s: 1.0,
                bearing_deg: 45.0,
            }),
            true, // Auto-start
        );
        
        let beacon_id = match create_cmd {
            EmulatorCommand::Create { lat, lon, depth, movement, start, .. } => {
                let pos = GeodeticPosition {
                    latitude: lat,
                    longitude: lon,
                    depth,
                };
                let id = manager.create_beacon(None, pos, None).await.unwrap();
                manager.update_beacon_movement_pattern(id, movement).await.unwrap();
                if start {
                    manager.start_beacon(id).await.unwrap();
                }
                id
            }
            _ => panic!("Expected Create command"),
        };
        
        // Verify beacon is running
        assert_eq!(manager.get_active_beacon_count(), 1);
        let status = manager.get_beacon_status(beacon_id).unwrap();
        assert!(status.is_running);
        
        // Simulate CLI stop command
        let stop_cmd = EmulatorCommand::Stop {
            id: beacon_id,
            remove: false,
        };
        
        match stop_cmd {
            EmulatorCommand::Stop { id, remove } => {
                manager.stop_beacon(id).await.unwrap();
                if remove {
                    manager.remove_beacon(id).await.unwrap();
                }
            }
            _ => panic!("Expected Stop command"),
        }
        
        // Verify beacon is stopped
        assert_eq!(manager.get_active_beacon_count(), 0);
        let status = manager.get_beacon_status(beacon_id).unwrap();
        assert!(!status.is_running);
        
        // Simulate CLI start command
        let start_cmd = EmulatorCommand::Start {
            id: Some(beacon_id),
            all: false,
        };
        
        match start_cmd {
            EmulatorCommand::Start { id: Some(id), all: false } => {
                manager.start_beacon(id).await.unwrap();
            }
            EmulatorCommand::Start { id: None, all: true } => {
                manager.start_all_beacons().await.unwrap();
            }
            _ => panic!("Expected Start command"),
        }
        
        // Verify beacon is running again
        assert_eq!(manager.get_active_beacon_count(), 1);
        let status = manager.get_beacon_status(beacon_id).unwrap();
        assert!(status.is_running);
        
        // Clean up
        manager.stop_beacon(beacon_id).await.unwrap();
    }
    
    #[tokio::test]
    async fn test_update_beacon_workflow() {
        let (mut manager, _temp_dir) = create_test_manager_with_temp_dir().await;
        let position = create_test_position();
        
        // Create beacon
        let beacon_id = manager.create_beacon(None, position, None).await.unwrap();
        
        // Simulate CLI update command
        let new_position = GeodeticPosition {
            latitude: 33.456,
            longitude: 46.789,
            depth: 15.0,
        };
        
        let update_cmd = EmulatorCommand::Update {
            id: beacon_id,
            position: Some(new_position),
            interval: Some(3000),
            movement: Some(MovementPattern::Circular {
                radius_m: 50.0,
                period_s: 30.0,
            }),
            restart: false,
        };
        
        match update_cmd {
            EmulatorCommand::Update { id, position, interval, movement, restart } => {
                if let Some(pos) = position {
                    manager.update_beacon_position(id, pos).await.unwrap();
                }
                
                if let Some(pattern) = movement {
                    manager.update_beacon_movement_pattern(id, pattern).await.unwrap();
                }
                
                if let Some(new_interval) = interval {
                    // Would update beacon config with new interval
                    let mut config = manager.get_beacon_status(id).unwrap().config;
                    config.transmission.interval_ms = new_interval;
                    manager.update_beacon_config(id, config).await.unwrap();
                    
                    if restart {
                        // Would restart beacon to apply interval changes
                        if manager.get_beacon_status(id).unwrap().is_running {
                            manager.stop_beacon(id).await.unwrap();
                            manager.start_beacon(id).await.unwrap();
                        }
                    }
                }
            }
            _ => panic!("Expected Update command"),
        }
        
        // Verify updates were applied
        let status = manager.get_beacon_status(beacon_id).unwrap();
        assert_eq!(status.position.latitude, new_position.latitude);
        assert_eq!(status.position.longitude, new_position.longitude);
        assert_eq!(status.position.depth, new_position.depth);
        assert_eq!(status.config.transmission.interval_ms, 3000);
        
        match status.movement_pattern {
            MovementPattern::Circular { radius_m, period_s } => {
                assert_eq!(radius_m, 50.0);
                assert_eq!(period_s, 30.0);
            }
            _ => panic!("Expected Circular movement pattern"),
        }
    }
    
    #[tokio::test]
    async fn test_remove_beacon_workflow() {
        let (mut manager, _temp_dir) = create_test_manager_with_temp_dir().await;
        let position = create_test_position();
        
        // Create and start beacon
        let beacon_id = manager.create_beacon(None, position, None).await.unwrap();
        manager.start_beacon(beacon_id).await.unwrap();
        
        assert_eq!(manager.get_total_beacon_count(), 1);
        assert_eq!(manager.get_active_beacon_count(), 1);
        
        // Simulate CLI remove command with force
        let remove_cmd = EmulatorCommand::Remove {
            id: beacon_id,
            force: true,
        };
        
        match remove_cmd {
            EmulatorCommand::Remove { id, force } => {
                if force {
                    // Force removal stops beacon first if running
                    if manager.get_beacon_status(id).unwrap().is_running {
                        manager.stop_beacon(id).await.unwrap();
                    }
                }
                manager.remove_beacon(id).await.unwrap();
            }
            _ => panic!("Expected Remove command"),
        }
        
        // Verify beacon was removed
        assert_eq!(manager.get_total_beacon_count(), 0);
        assert_eq!(manager.get_active_beacon_count(), 0);
        assert!(!manager.beacon_exists(beacon_id));
    }
    
    #[tokio::test]
    async fn test_stop_all_workflow() {
        let (mut manager, _temp_dir) = create_test_manager_with_temp_dir().await;
        
        // Create multiple beacons
        let mut beacon_ids = Vec::new();
        for i in 0..3 {
            let position = GeodeticPosition {
                latitude: 32.0 + (i as f64) * 0.01,
                longitude: 45.0 + (i as f64) * 0.01,
                depth: 10.0,
            };
            let id = manager.create_beacon(None, position, None).await.unwrap();
            manager.start_beacon(id).await.unwrap();
            beacon_ids.push(id);
        }
        
        assert_eq!(manager.get_active_beacon_count(), 3);
        
        // Simulate CLI stop-all command
        let stop_all_cmd = EmulatorCommand::StopAll {
            remove: true,
        };
        
        match stop_all_cmd {
            EmulatorCommand::StopAll { remove } => {
                manager.stop_all_beacons().await.unwrap();
                if remove {
                    for beacon_id in beacon_ids {
                        manager.remove_beacon(beacon_id).await.unwrap();
                    }
                }
            }
            _ => panic!("Expected StopAll command"),
        }
        
        // Verify all beacons were stopped and removed
        assert_eq!(manager.get_active_beacon_count(), 0);
        assert_eq!(manager.get_total_beacon_count(), 0);
    }
    
    #[tokio::test]
    async fn test_status_command_workflow() {
        let (mut manager, _temp_dir) = create_test_manager_with_temp_dir().await;
        
        // Create some beacons in different states
        let position1 = create_test_position();
        let position2 = GeodeticPosition {
            latitude: 33.456,
            longitude: 46.789,
            depth: 15.0,
        };
        
        let beacon1 = manager.create_beacon(None, position1, None).await.unwrap();
        let _beacon2 = manager.create_beacon(None, position2, None).await.unwrap();
        
        // Start only one beacon
        manager.start_beacon(beacon1).await.unwrap();
        
        // Simulate CLI status command
        let status_cmd = EmulatorCommand::Status {
            detailed: true,
        };
        
        match status_cmd {
            EmulatorCommand::Status { detailed } => {
                let stats = manager.get_manager_stats().await;
                assert_eq!(stats.total_beacons, 2);
                assert_eq!(stats.running_beacons, 1);
                assert_eq!(stats.stopped_beacons, 1);
                assert_eq!(stats.current_channel, "test_channel");
                
                if detailed {
                    // Would show detailed statistics
                    assert!(stats.channel_names.contains(&"test_channel".to_string()));
                    assert_eq!(stats.active_channels, 1);
                }
            }
            _ => panic!("Expected Status command"),
        }
        
        // Clean up
        manager.stop_beacon(beacon1).await.unwrap();
    }
    
    #[tokio::test]
    async fn test_clear_state_workflow() {
        let (mut manager, _temp_dir) = create_test_manager_with_temp_dir().await;
        
        // Create some beacons and save state
        let position = create_test_position();
        let beacon_id = manager.create_beacon(None, position, None).await.unwrap();
        manager.start_beacon(beacon_id).await.unwrap();
        manager.save_state().await.unwrap();
        
        assert_eq!(manager.get_total_beacon_count(), 1);
        assert_eq!(manager.get_active_beacon_count(), 1);
        
        // Simulate CLI clear command
        let clear_cmd = EmulatorCommand::Clear {
            confirm: true,
        };
        
        match clear_cmd {
            EmulatorCommand::Clear { confirm } => {
                if confirm {
                    manager.clear_state().await.unwrap();
                }
            }
            _ => panic!("Expected Clear command"),
        }
        
        // Verify state was cleared
        assert_eq!(manager.get_total_beacon_count(), 0);
        assert_eq!(manager.get_active_beacon_count(), 0);
    }
}

/// Tests for scenario management workflows
mod scenario_workflow_tests {
    use super::*;
    use cli_test_utils::*;
    
    #[tokio::test]
    async fn test_triangle_scenario_workflow() {
        let (mut manager, _temp_dir) = create_test_manager_with_temp_dir().await;
        let center = create_test_position();
        
        // Simulate CLI scenario command
        let scenario_cmd = create_scenario_command(
            ScenarioType::Triangle,
            3,
            100.0,
            center,
            true, // Start all beacons
        );
        
        let beacon_ids = match scenario_cmd {
            EmulatorCommand::Scenario { scenario_type, count, spacing, center, start_all, .. } => {
                let ids = manager.create_scenario(scenario_type, count, spacing, center).await.unwrap();
                
                if start_all {
                    for id in &ids {
                        manager.start_beacon(*id).await.unwrap();
                    }
                }
                
                ids
            }
            _ => panic!("Expected Scenario command"),
        };
        
        // Verify scenario was created correctly
        assert_eq!(beacon_ids.len(), 3);
        assert_eq!(manager.get_total_beacon_count(), 3);
        assert_eq!(manager.get_active_beacon_count(), 3);
        
        // Verify beacon positions
        for beacon_id in &beacon_ids {
            let status = manager.get_beacon_status(*beacon_id).unwrap();
            assert_eq!(status.position.depth, center.depth);
            assert!(status.is_running);
        }
        
        // Clean up
        for beacon_id in beacon_ids {
            manager.stop_beacon(beacon_id).await.unwrap();
        }
    }
    
    #[tokio::test]
    async fn test_square_scenario_workflow() {
        let (mut manager, _temp_dir) = create_test_manager_with_temp_dir().await;
        let center = create_test_position();
        
        let scenario_cmd = create_scenario_command(
            ScenarioType::Square,
            4,
            200.0,
            center,
            false, // Don't auto-start
        );
        
        let beacon_ids = match scenario_cmd {
            EmulatorCommand::Scenario { scenario_type, count, spacing, center, start_all, .. } => {
                let ids = manager.create_scenario(scenario_type, count, spacing, center).await.unwrap();
                
                if start_all {
                    for id in &ids {
                        manager.start_beacon(*id).await.unwrap();
                    }
                }
                
                ids
            }
            _ => panic!("Expected Scenario command"),
        };
        
        assert_eq!(beacon_ids.len(), 4);
        assert_eq!(manager.get_total_beacon_count(), 4);
        assert_eq!(manager.get_active_beacon_count(), 0); // Not auto-started
        
        // Manually start all beacons using start-all workflow
        let start_all_cmd = EmulatorCommand::Start {
            id: None,
            all: true,
        };
        
        match start_all_cmd {
            EmulatorCommand::Start { id: None, all: true } => {
                manager.start_all_beacons().await.unwrap();
            }
            _ => panic!("Expected Start all command"),
        }
        
        assert_eq!(manager.get_active_beacon_count(), 4);
        
        // Clean up
        manager.stop_all_beacons().await.unwrap();
    }
    
    #[tokio::test]
    async fn test_line_scenario_workflow() {
        let (mut manager, _temp_dir) = create_test_manager_with_temp_dir().await;
        let center = create_test_position();
        
        let scenario_cmd = create_scenario_command(
            ScenarioType::Line,
            5,
            50.0,
            center,
            true,
        );
        
        let beacon_ids = match scenario_cmd {
            EmulatorCommand::Scenario { scenario_type, count, spacing, center, start_all, .. } => {
                let ids = manager.create_scenario(scenario_type, count, spacing, center).await.unwrap();
                
                if start_all {
                    for id in &ids {
                        manager.start_beacon(*id).await.unwrap();
                    }
                }
                
                ids
            }
            _ => panic!("Expected Scenario command"),
        };
        
        assert_eq!(beacon_ids.len(), 5);
        assert_eq!(manager.get_active_beacon_count(), 5);
        
        // Verify beacons are arranged in a line
        let mut positions = Vec::new();
        for beacon_id in &beacon_ids {
            let status = manager.get_beacon_status(*beacon_id).unwrap();
            positions.push(status.position);
        }
        
        // All should have same longitude and depth
        for pos in &positions {
            assert_eq!(pos.longitude, center.longitude);
            assert_eq!(pos.depth, center.depth);
        }
        
        // Clean up
        for beacon_id in beacon_ids {
            manager.stop_beacon(beacon_id).await.unwrap();
        }
    }
    
    #[tokio::test]
    async fn test_grid_scenario_workflow() {
        let (mut manager, _temp_dir) = create_test_manager_with_temp_dir().await;
        let center = create_test_position();
        
        let scenario_cmd = create_scenario_command(
            ScenarioType::Grid,
            9, // 3x3 grid
            75.0,
            center,
            true,
        );
        
        let beacon_ids = match scenario_cmd {
            EmulatorCommand::Scenario { scenario_type, count, spacing, center, start_all, .. } => {
                let ids = manager.create_scenario(scenario_type, count, spacing, center).await.unwrap();
                
                if start_all {
                    for id in &ids {
                        manager.start_beacon(*id).await.unwrap();
                    }
                }
                
                ids
            }
            _ => panic!("Expected Scenario command"),
        };
        
        assert_eq!(beacon_ids.len(), 9);
        assert_eq!(manager.get_active_beacon_count(), 9);
        
        // Verify all beacons are at the same depth
        for beacon_id in &beacon_ids {
            let status = manager.get_beacon_status(*beacon_id).unwrap();
            assert_eq!(status.position.depth, center.depth);
        }
        
        // Clean up
        for beacon_id in beacon_ids {
            manager.stop_beacon(beacon_id).await.unwrap();
        }
    }
    
    #[tokio::test]
    async fn test_scenario_validation_workflow() {
        let (mut manager, _temp_dir) = create_test_manager_with_temp_dir().await;
        let center = create_test_position();
        
        // Test invalid triangle scenario
        let invalid_scenario_cmd = create_scenario_command(
            ScenarioType::Triangle,
            4, // Wrong count for triangle
            100.0,
            center,
            false,
        );
        
        let result = match invalid_scenario_cmd {
            EmulatorCommand::Scenario { scenario_type, count, spacing, center, .. } => {
                manager.create_scenario(scenario_type, count, spacing, center).await
            }
            _ => panic!("Expected Scenario command"),
        };
        
        assert!(result.is_err());
        assert_eq!(manager.get_total_beacon_count(), 0);
    }
}

/// Tests for configuration file workflows
mod config_workflow_tests {
    use super::*;
    use cli_test_utils::*;
    
    #[tokio::test]
    async fn test_generate_template_workflow() {
        let (manager, temp_dir) = create_test_manager_with_temp_dir().await;
        let template_path = temp_dir.path().join("template.toml");
        
        // Simulate CLI generate-template command
        let generate_cmd = EmulatorCommand::GenerateTemplate {
            output: template_path.clone(),
            format: CliConfigFormat::Toml,
            emulator: false,
            lat: None,
            lon: None,
            depth: 0.0,
        };
        
        match generate_cmd {
            EmulatorCommand::GenerateTemplate { output, format, emulator, lat, lon, depth } => {
                if emulator {
                    let position = GeodeticPosition {
                        latitude: lat.unwrap(),
                        longitude: lon.unwrap(),
                        depth,
                    };
                    manager.generate_emulator_config_template(&output, position, format.into()).await.unwrap();
                } else {
                    manager.generate_config_template(&output, format.into()).await.unwrap();
                }
            }
            _ => panic!("Expected GenerateTemplate command"),
        }
        
        // Verify template file was created
        assert!(template_path.exists());
        
        // Verify template content is valid
        let content = tokio::fs::read_to_string(&template_path).await.unwrap();
        assert!(!content.is_empty());
        assert!(content.contains("beacon_id"));
    }
    
    #[tokio::test]
    async fn test_generate_emulator_template_workflow() {
        let (manager, temp_dir) = create_test_manager_with_temp_dir().await;
        let template_path = temp_dir.path().join("emulator_template.json");
        let position = create_test_position();
        
        // Simulate CLI generate-template command for emulator config
        let generate_cmd = EmulatorCommand::GenerateTemplate {
            output: template_path.clone(),
            format: CliConfigFormat::Json,
            emulator: true,
            lat: Some(position.latitude),
            lon: Some(position.longitude),
            depth: position.depth,
        };
        
        match generate_cmd {
            EmulatorCommand::GenerateTemplate { output, format, emulator, lat, lon, depth } => {
                if emulator {
                    let pos = GeodeticPosition {
                        latitude: lat.unwrap(),
                        longitude: lon.unwrap(),
                        depth,
                    };
                    manager.generate_emulator_config_template(&output, pos, format.into()).await.unwrap();
                } else {
                    manager.generate_config_template(&output, format.into()).await.unwrap();
                }
            }
            _ => panic!("Expected GenerateTemplate command"),
        }
        
        // Verify template file was created
        assert!(template_path.exists());
        
        // Verify template content is valid JSON
        let content = tokio::fs::read_to_string(&template_path).await.unwrap();
        let _json: serde_json::Value = serde_json::from_str(&content).unwrap();
    }
    
    #[tokio::test]
    async fn test_validate_config_workflow() {
        let (manager, temp_dir) = create_test_manager_with_temp_dir().await;
        let config_path = temp_dir.path().join("test_config.toml");
        
        // First generate a config template
        manager.generate_config_template(&config_path, crate::config::ConfigFormat::Toml).await.unwrap();
        
        // Simulate CLI validate-config command
        let validate_cmd = EmulatorCommand::ValidateConfig {
            config: config_path.clone(),
            emulator: false,
            verbose: true,
        };
        
        match validate_cmd {
            EmulatorCommand::ValidateConfig { config, emulator, verbose: _ } => {
                if emulator {
                    let emulator_config = manager.load_emulator_beacon_config(&config).await.unwrap();
                    manager.validate_emulator_specific_config(&emulator_config).unwrap();
                } else {
                    let beacon_config = manager.load_beacon_config(&config).await.unwrap();
                    manager.validate_emulator_config(&beacon_config).unwrap();
                }
            }
            _ => panic!("Expected ValidateConfig command"),
        }
        
        // If we reach here, validation passed
    }
    
    #[tokio::test]
    async fn test_convert_config_workflow() {
        let (manager, temp_dir) = create_test_manager_with_temp_dir().await;
        let toml_path = temp_dir.path().join("config.toml");
        let json_path = temp_dir.path().join("config.json");
        
        // Generate TOML config
        manager.generate_config_template(&toml_path, crate::config::ConfigFormat::Toml).await.unwrap();
        
        // Simulate CLI convert-config command
        let convert_cmd = EmulatorCommand::ConvertConfig {
            input: toml_path.clone(),
            output: json_path.clone(),
            validate: true,
        };
        
        match convert_cmd {
            EmulatorCommand::ConvertConfig { input, output, validate } => {
                // Load config from input format
                let config = manager.load_beacon_config(&input).await.unwrap();
                
                if validate {
                    manager.validate_emulator_config(&config).unwrap();
                }
                
                // Save config in output format (determined by file extension)
                manager.save_beacon_config(&config, &output).await.unwrap();
            }
            _ => panic!("Expected ConvertConfig command"),
        }
        
        // Verify both files exist and contain valid configs
        assert!(toml_path.exists());
        assert!(json_path.exists());
        
        let toml_config = manager.load_beacon_config(&toml_path).await.unwrap();
        let json_config = manager.load_beacon_config(&json_path).await.unwrap();
        
        // Configs should be equivalent
        assert_eq!(toml_config.beacon_id, json_config.beacon_id);
        assert_eq!(toml_config.transmission.interval_ms, json_config.transmission.interval_ms);
    }
    
    #[tokio::test]
    async fn test_create_from_config_workflow() {
        let (mut manager, temp_dir) = create_test_manager_with_temp_dir().await;
        let config_path = temp_dir.path().join("beacon_config.toml");
        let position = create_test_position();
        
        // Generate config template
        manager.generate_config_template(&config_path, crate::config::ConfigFormat::Toml).await.unwrap();
        
        // Simulate CLI create command with config file
        let create_cmd = EmulatorCommand::Create {
            id: None,
            lat: position.latitude,
            lon: position.longitude,
            depth: position.depth,
            config: Some(config_path.clone()),
            interval: 5000,
            version: MessageVersion::V3,
            movement: MovementPattern::Stationary,
            start: false,
        };
        
        let beacon_id = match create_cmd {
            EmulatorCommand::Create { lat, lon, depth, config, .. } => {
                let pos = GeodeticPosition {
                    latitude: lat,
                    longitude: lon,
                    depth,
                };
                
                if let Some(config_file) = config {
                    manager.create_beacon_from_config(None, pos, &config_file).await.unwrap()
                } else {
                    manager.create_beacon(None, pos, None).await.unwrap()
                }
            }
            _ => panic!("Expected Create command"),
        };
        
        // Verify beacon was created with config from file
        assert!(manager.beacon_exists(beacon_id));
        let status = manager.get_beacon_status(beacon_id).unwrap();
        assert_eq!(status.position.latitude, position.latitude);
    }
}

/// Tests for monitoring and export workflows
mod monitoring_workflow_tests {
    use super::*;
    use cli_test_utils::*;
    
    #[tokio::test]
    async fn test_monitor_workflow_simulation() {
        let (mut manager, _temp_dir) = create_test_manager_with_temp_dir().await;
        let position = create_test_position();
        
        // Create and start beacon
        let beacon_id = manager.create_beacon(None, position, None).await.unwrap();
        manager.start_beacon(beacon_id).await.unwrap();
        
        // Simulate CLI monitor command
        let monitor_cmd = EmulatorCommand::Monitor {
            beacon: Some(beacon_id),
            interval: 1,
            compact: false,
            show_messages: true,
        };
        
        match monitor_cmd {
            EmulatorCommand::Monitor { beacon, interval: _, compact: _, show_messages: _ } => {
                if let Some(specific_beacon) = beacon {
                    // Monitor specific beacon
                    let status = manager.get_beacon_status(specific_beacon).unwrap();
                    assert!(status.is_running);
                    
                    // In real implementation, this would run in a loop with periodic updates
                    // Here we just verify we can get the status
                } else {
                    // Monitor all beacons
                    let all_beacons = manager.list_beacons();
                    assert_eq!(all_beacons.len(), 1);
                }
            }
            _ => panic!("Expected Monitor command"),
        }
        
        // Let beacon run briefly to generate some activity
        sleep(Duration::from_millis(100)).await;
        
        // Verify beacon has activity
        let status = manager.get_beacon_status(beacon_id).unwrap();
        assert!(status.is_running);
        
        // Clean up
        manager.stop_beacon(beacon_id).await.unwrap();
    }
    
    #[tokio::test]
    async fn test_export_workflow_simulation() {
        let (mut manager, temp_dir) = create_test_manager_with_temp_dir().await;
        let position = create_test_position();
        let export_path = temp_dir.path().join("export.json");
        
        // Create and start beacon
        let beacon_id = manager.create_beacon(None, position, None).await.unwrap();
        manager.start_beacon(beacon_id).await.unwrap();
        
        // Let beacon run to generate some messages
        sleep(Duration::from_millis(200)).await;
        
        // Simulate CLI export command
        let export_cmd = EmulatorCommand::Export {
            output: export_path.clone(),
            format: ExportFormat::Json,
            duration: 3600,
            all_time: false,
            beacon: Some(beacon_id),
            include_messages: true,
        };
        
        match export_cmd {
            EmulatorCommand::Export { output, format, duration, all_time, beacon, include_messages: _ } => {
                // In real implementation, this would export logs
                // For now, we simulate by checking that we can access the data
                
                let comm_space = manager.get_communication_space();
                let channel = {
                    let space = comm_space.read().await;
                    space.get_channel("test_channel").cloned()
                };
                
                if let Some(ch) = channel {
                    let messages = if all_time {
                        ch.get_recent_messages(1000).await
                    } else {
                        let since = std::time::SystemTime::now() - std::time::Duration::from_secs(duration);
                        ch.get_messages_since(since).await
                    };
                    
                    let filtered_messages = if let Some(beacon_filter) = beacon {
                        messages.into_iter()
                            .filter(|msg| msg.beacon_id == beacon_filter)
                            .collect::<Vec<_>>()
                    } else {
                        messages
                    };
                    
                    // Simulate export by writing a simple JSON file
                    let export_data = serde_json::json!({
                        "format": format!("{:?}", format),
                        "message_count": filtered_messages.len(),
                        "beacon_filter": beacon,
                        "export_time": std::time::SystemTime::now()
                            .duration_since(std::time::UNIX_EPOCH)
                            .unwrap()
                            .as_secs()
                    });
                    
                    tokio::fs::write(&output, serde_json::to_string_pretty(&export_data).unwrap()).await.unwrap();
                }
            }
            _ => panic!("Expected Export command"),
        }
        
        // Verify export file was created
        assert!(export_path.exists());
        
        // Clean up
        manager.stop_beacon(beacon_id).await.unwrap();
    }
}

/// Tests for daemon mode workflow
mod daemon_workflow_tests {
    use super::*;
    use cli_test_utils::*;
    
    #[tokio::test]
    async fn test_daemon_mode_simulation() {
        let (mut manager, _temp_dir) = create_test_manager_with_temp_dir().await;
        let position = create_test_position();
        
        // Create beacon and mark it as intended to run (simulate persistent state)
        let beacon_id = manager.create_beacon(None, position, None).await.unwrap();
        manager.save_state().await.unwrap();
        
        // Simulate CLI daemon command
        let daemon_cmd = EmulatorCommand::Daemon {
            status_interval: 5,
            auto_start: true,
            background: false,
        };
        
        match daemon_cmd {
            EmulatorCommand::Daemon { status_interval: _, auto_start, background: _ } => {
                if auto_start {
                    // Auto-start all beacons marked as intended-running
                    // In this test, we'll start all existing beacons
                    manager.start_all_beacons().await.unwrap();
                }
                
                // In real daemon mode, this would run indefinitely
                // Here we simulate a brief daemon run
                sleep(Duration::from_millis(100)).await;
                
                // Verify beacon is running
                assert_eq!(manager.get_active_beacon_count(), 1);
                let status = manager.get_beacon_status(beacon_id).unwrap();
                assert!(status.is_running);
            }
            _ => panic!("Expected Daemon command"),
        }
        
        // Clean up
        manager.stop_beacon(beacon_id).await.unwrap();
    }
    
    #[tokio::test]
    async fn test_daemon_with_multiple_beacons() {
        let (mut manager, _temp_dir) = create_test_manager_with_temp_dir().await;
        
        // Create multiple beacons
        let mut beacon_ids = Vec::new();
        for i in 0..3 {
            let position = GeodeticPosition {
                latitude: 32.0 + (i as f64) * 0.01,
                longitude: 45.0 + (i as f64) * 0.01,
                depth: 10.0,
            };
            let id = manager.create_beacon(None, position, None).await.unwrap();
            beacon_ids.push(id);
        }
        
        // Save state
        manager.save_state().await.unwrap();
        
        // Simulate daemon startup with auto-start
        let daemon_cmd = EmulatorCommand::Daemon {
            status_interval: 1,
            auto_start: true,
            background: true,
        };
        
        match daemon_cmd {
            EmulatorCommand::Daemon { status_interval: _, auto_start, background: _ } => {
                if auto_start {
                    manager.start_all_beacons().await.unwrap();
                }
                
                // Simulate daemon running
                sleep(Duration::from_millis(100)).await;
                
                // Check status
                let stats = manager.get_manager_stats().await;
                assert_eq!(stats.total_beacons, 3);
                assert_eq!(stats.running_beacons, 3);
            }
            _ => panic!("Expected Daemon command"),
        }
        
        // Clean up
        manager.stop_all_beacons().await.unwrap();
    }
}