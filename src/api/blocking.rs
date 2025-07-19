//! Blocking API for simple positioning operations
//! 
//! This module provides a simple, synchronous API suitable for basic applications
//! where blocking operations are acceptable.

use crate::api::types::{ApiResult, ApiError, PositionRequest, PositionResponse, SystemState, ApiConfig};
use crate::core::{Position, Anchor};
use crate::hardware::{TransceiverInterface, TransceiverConfig};
use crate::algorithms::embedded_trilateration::EmbeddedTrilateration;
use crate::processing::parser::MessageParser;
use crate::validation::data::DataValidator;
use std::collections::HashMap;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

/// Simple blocking API for underwater positioning
pub struct BlockingPositioningApi {
    /// Trilateration engine
    trilateration: EmbeddedTrilateration,
    /// Message parser
    parser: MessageParser,
    /// Data validator
    validator: DataValidator,
    /// Transceiver interfaces
    transceivers: HashMap<u8, Box<dyn TransceiverInterface>>,
    /// API configuration
    config: ApiConfig,
    /// System state
    state: SystemState,
    /// Sequence counter
    sequence_counter: u32,
    /// Start time for uptime calculation
    start_time: Instant,
}

impl BlockingPositioningApi {
    /// Create a new blocking positioning API
    pub fn new(config: ApiConfig) -> Self {
        Self {
            trilateration: EmbeddedTrilateration::default(),
            parser: MessageParser::new(),
            validator: DataValidator::new(),
            transceivers: HashMap::new(),
            config,
            state: SystemState::default(),
            sequence_counter: 0,
            start_time: Instant::now(),
        }
    }
    
    /// Initialize the positioning system
    pub fn initialize(&mut self) -> ApiResult<()> {
        // Validate configuration
        if self.config.default_min_anchors < 3 {
            return Err(ApiError::ConfigurationError {
                parameter: "default_min_anchors".to_string(),
                value: self.config.default_min_anchors.to_string(),
            });
        }
        
        // Initialize components
        self.trilateration = EmbeddedTrilateration::default();
        self.parser = MessageParser::new();
        self.validator = DataValidator::new();
        
        // Update state
        self.state.initialized = true;
        self.state.active_transceivers = self.transceivers.len() as u8;
        
        Ok(())
    }
    
    /// Add a transceiver interface
    pub fn add_transceiver(&mut self, transceiver: Box<dyn TransceiverInterface>) -> ApiResult<()> {
        let id = transceiver.get_id();
        
        if self.transceivers.contains_key(&id) {
            return Err(ApiError::ConfigurationError {
                parameter: "transceiver_id".to_string(),
                value: id.to_string(),
            });
        }
        
        self.transceivers.insert(id, transceiver);
        self.state.active_transceivers = self.transceivers.len() as u8;
        
        Ok(())
    }
    
    /// Remove a transceiver interface
    pub fn remove_transceiver(&mut self, id: u8) -> ApiResult<()> {
        if self.transceivers.remove(&id).is_none() {
            return Err(ApiError::ConfigurationError {
                parameter: "transceiver_id".to_string(),
                value: id.to_string(),
            });
        }
        
        self.state.active_transceivers = self.transceivers.len() as u8;
        Ok(())
    }
    
    /// Get current position using default parameters
    pub fn get_position(&mut self) -> ApiResult<PositionResponse> {
        let request = PositionRequest::default();
        self.get_position_with_request(&request)
    }
    
    /// Get formatted position using default parameters
    pub fn get_formatted_position(&mut self) -> ApiResult<crate::api::formatting::FormattedPosition> {
        let response = self.get_position()?;
        let formatter = crate::api::formatting::PositionFormatter::new();
        Ok(formatter.format(&response))
    }
    
    /// Get formatted position with specific format
    pub fn get_formatted_position_with_format(&mut self, format: crate::api::types::OutputFormat) -> ApiResult<crate::api::formatting::FormattedPosition> {
        let response = self.get_position()?;
        let formatter = crate::api::formatting::PositionFormatter::new().with_format(format);
        Ok(formatter.format(&response))
    }
    
    /// Get position as human-readable text
    pub fn get_position_text(&mut self, compact: bool) -> ApiResult<String> {
        let response = self.get_position()?;
        let formatter = crate::api::formatting::PositionFormatter::new();
        let formatted = formatter.format(&response);
        let text_formatter = crate::api::formatting::TextFormatter { 
            include_diagnostics: false, 
            compact 
        };
        Ok(text_formatter.format_text(&formatted))
    }
    
    /// Get position as JSON string
    pub fn get_position_json(&mut self, pretty: bool) -> ApiResult<String> {
        let response = self.get_position()?;
        let formatter = crate::api::formatting::PositionFormatter::new();
        let formatted = formatter.format(&response);
        let json_formatter = if pretty {
            crate::api::formatting::JsonFormatter::pretty()
        } else {
            crate::api::formatting::JsonFormatter::new()
        };
        json_formatter.format_json(&formatted).map_err(|e| ApiError::InvalidRequest {
            reason: format!("JSON formatting error: {}", e),
        })
    }
    
    /// Get position as CSV row
    pub fn get_position_csv(&mut self, include_header: bool) -> ApiResult<String> {
        let response = self.get_position()?;
        let formatter = crate::api::formatting::PositionFormatter::new();
        let formatted = formatter.format(&response);
        let csv_formatter = crate::api::formatting::CsvFormatter { include_header };
        
        let mut result = String::new();
        if include_header {
            result.push_str(&csv_formatter.header());
            result.push('\n');
        }
        result.push_str(&csv_formatter.format_csv(&formatted));
        
        Ok(result)
    }
    
    /// Get current position with specific request parameters
    pub fn get_position_with_request(&mut self, request: &PositionRequest) -> ApiResult<PositionResponse> {
        if !self.state.initialized {
            return Err(ApiError::NotInitialized);
        }
        
        let start_time = Instant::now();
        let timeout = std::time::Duration::from_millis(request.timeout_ms as u64);
        
        // Collect anchor messages from all transceivers
        let mut anchors = Vec::new();
        let deadline = start_time + timeout;
        
        while Instant::now() < deadline && anchors.len() < request.min_anchors as usize {
            for transceiver in self.transceivers.values_mut() {
                if let Ok(Some(raw_message)) = transceiver.read_message() {
                    // Convert hardware RawMessage to parser RawMessage
                    let parser_message = crate::processing::parser::RawMessage {
                        data: raw_message.data,
                        timestamp_received: raw_message.timestamp_ms,
                        transceiver_id: transceiver.get_id(),
                    };
                    match self.parser.parse_message(&parser_message) {
                        Ok(anchor_msg) => {
                            // Validate message (validate single message by wrapping in Vec)
                            let validation_result = self.validator.validate_messages(vec![anchor_msg.clone()]);
                            if validation_result.valid_messages.is_empty() {
                                if self.config.log_level as u8 >= 3 { // Info level
                                    eprintln!("Invalid anchor message: {:?}", validation_result.rejected_messages);
                                }
                                continue;
                            }
                            let anchor_msg = validation_result.valid_messages[0].clone();
                            
                            // Check message age
                            let current_time = SystemTime::now()
                                .duration_since(UNIX_EPOCH)
                                .unwrap()
                                .as_millis() as u64;
                            
                            if current_time.saturating_sub(anchor_msg.timestamp_ms) > request.max_anchor_age_ms as u64 {
                                continue;
                            }
                            
                            // Convert to core Anchor type
                            let anchor = Anchor {
                                id: anchor_msg.anchor_id.to_string(),
                                timestamp: anchor_msg.timestamp_ms,
                                position: Position {
                                    lat: anchor_msg.position.latitude,
                                    lon: anchor_msg.position.longitude,
                                    depth: anchor_msg.position.depth,
                                },
                            };
                            
                            anchors.push(anchor);
                        }
                        Err(e) => {
                            if self.config.log_level as u8 >= 2 { // Warn level
                                eprintln!("Failed to parse message: {:?}", e);
                            }
                        }
                    }
                }
            }
            
            // Small delay to avoid busy waiting
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
        
        // Check if we have enough anchors
        if anchors.len() < request.min_anchors as usize {
            self.state.error_count += 1;
            return Err(ApiError::InsufficientAnchors {
                available: anchors.len() as u8,
                required: request.min_anchors,
            });
        }
        
        // Perform trilateration
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;
        match self.trilateration.calculate_position_embedded(&anchors, current_time) {
            Ok(position) => {
                let computation_time = start_time.elapsed();
                self.sequence_counter += 1;
                
                // Update statistics
                self.state.positions_calculated += 1;
                self.state.last_position_time = Some(
                    SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .unwrap()
                        .as_millis() as u64
                );
                self.state.uptime_ms = self.start_time.elapsed().as_millis() as u64;
                
                // Update average computation time
                let comp_time_us = computation_time.as_micros() as u32;
                if self.state.positions_calculated == 1 {
                    self.state.avg_computation_time_us = comp_time_us;
                } else {
                    self.state.avg_computation_time_us = 
                        (self.state.avg_computation_time_us + comp_time_us) / 2;
                }
                
                // Create response
                let response = PositionResponse {
                    position,
                    local_position: None, // TODO: Add local coordinate conversion
                    accuracy_estimate: 2.0, // TODO: Calculate actual accuracy estimate
                    timestamp_ms: self.state.last_position_time.unwrap(),
                    anchor_count: anchors.len() as u8,
                    geometry_quality: crate::api::types::GeometryQuality::Good, // TODO: Calculate actual quality
                    computation_time_us: comp_time_us,
                    sequence_number: self.sequence_counter,
                };
                
                Ok(response)
            }
            Err(e) => {
                self.state.error_count += 1;
                Err(ApiError::ComputationFailure {
                    details: format!("Trilateration failed: {:?}", e),
                })
            }
        }
    }
    
    /// Get system state information
    pub fn get_system_state(&mut self) -> SystemState {
        self.state.uptime_ms = self.start_time.elapsed().as_millis() as u64;
        self.state.clone()
    }
    
    /// Update API configuration
    pub fn update_config(&mut self, config: ApiConfig) -> ApiResult<()> {
        // Validate new configuration
        if config.default_min_anchors < 3 {
            return Err(ApiError::ConfigurationError {
                parameter: "default_min_anchors".to_string(),
                value: config.default_min_anchors.to_string(),
            });
        }
        
        self.config = config;
        Ok(())
    }
    
    /// Get current configuration
    pub fn get_config(&self) -> &ApiConfig {
        &self.config
    }
    
    /// Reset system statistics
    pub fn reset_statistics(&mut self) {
        self.state.positions_calculated = 0;
        self.state.error_count = 0;
        self.state.avg_computation_time_us = 0;
        self.sequence_counter = 0;
        self.start_time = Instant::now();
    }
    
    /// Shutdown the positioning system
    pub fn shutdown(&mut self) -> ApiResult<()> {
        // Clear transceivers
        self.transceivers.clear();
        
        // Reset state
        self.state = SystemState::default();
        self.sequence_counter = 0;
        
        Ok(())
    }
}

impl Default for BlockingPositioningApi {
    fn default() -> Self {
        Self::new(ApiConfig::default())
    }
}