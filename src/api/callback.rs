//! Callback-based API for event-driven systems
//! 
//! This module provides callback-based interfaces suitable for event-driven
//! embedded systems and interrupt-based architectures.

use crate::api::types::{ApiResult, ApiError, PositionRequest, PositionResponse, SystemState, ApiConfig};
use crate::core::{Position, Anchor};
use crate::hardware::{TransceiverInterface, TransceiverConfig};
use crate::algorithms::embedded_trilateration::EmbeddedTrilateration;
use crate::processing::parser::MessageParser;
use crate::validation::data::DataValidator;
use std::collections::HashMap;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

/// Callback function type for position updates
pub type PositionCallback = Box<dyn Fn(PositionResponse) + Send>;

/// Callback function type for system events
pub type EventCallback = Box<dyn Fn(SystemEvent) + Send>;

/// System events that can trigger callbacks
#[derive(Debug, Clone)]
pub enum SystemEvent {
    /// New anchor message received
    AnchorMessageReceived {
        anchor_id: String,
        timestamp_ms: u64,
        signal_quality: u8,
    },
    /// Position calculation completed
    PositionCalculated {
        sequence_number: u32,
        computation_time_us: u32,
        anchor_count: u8,
    },
    /// Error occurred during processing
    ErrorOccurred {
        error: ApiError,
        context: String,
    },
    /// System state changed
    StateChanged {
        old_state: SystemState,
        new_state: SystemState,
    },
    /// Transceiver status changed
    TransceiverStatusChanged {
        transceiver_id: u8,
        connected: bool,
        error_count: u32,
    },
}

/// Callback registration handle
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CallbackHandle(u32);

impl CallbackHandle {
    fn new(id: u32) -> Self {
        CallbackHandle(id)
    }
    
    pub fn id(&self) -> u32 {
        self.0
    }
}

/// Callback-based positioning API
pub struct CallbackPositioningApi {
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
    /// Callback handle counter
    callback_counter: u32,
    /// Position callbacks
    position_callbacks: HashMap<CallbackHandle, PositionCallback>,
    /// Event callbacks
    event_callbacks: HashMap<CallbackHandle, EventCallback>,
    /// Automatic positioning configuration
    auto_positioning: Option<PositionRequest>,
    /// Collected anchors for automatic positioning
    collected_anchors: Vec<Anchor>,
    /// Last anchor collection time
    last_collection_time: Instant,
    /// Start time for uptime calculation
    start_time: Instant,
}

impl CallbackPositioningApi {
    /// Create a new callback-based positioning API
    pub fn new(config: ApiConfig) -> Self {
        Self {
            trilateration: EmbeddedTrilateration::default(),
            parser: MessageParser::new(),
            validator: DataValidator::new(),
            transceivers: HashMap::new(),
            config,
            state: SystemState::default(),
            sequence_counter: 0,
            callback_counter: 0,
            position_callbacks: HashMap::new(),
            event_callbacks: HashMap::new(),
            auto_positioning: None,
            collected_anchors: Vec::new(),
            last_collection_time: Instant::now(),
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
        
        let old_state = self.state.clone();
        
        // Initialize components
        self.trilateration = EmbeddedTrilateration::default();
        self.parser = MessageParser::new();
        self.validator = DataValidator::new();
        
        // Update state
        self.state.initialized = true;
        self.state.active_transceivers = self.transceivers.len() as u8;
        
        // Trigger state change event
        self.trigger_event(SystemEvent::StateChanged {
            old_state,
            new_state: self.state.clone(),
        });
        
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
        
        // Trigger transceiver status event
        self.trigger_event(SystemEvent::TransceiverStatusChanged {
            transceiver_id: id,
            connected: true,
            error_count: 0,
        });
        
        Ok(())
    }
    
    /// Register a position callback
    pub fn register_position_callback(&mut self, callback: PositionCallback) -> CallbackHandle {
        self.callback_counter += 1;
        let handle = CallbackHandle::new(self.callback_counter);
        self.position_callbacks.insert(handle, callback);
        handle
    }
    
    /// Register an event callback
    pub fn register_event_callback(&mut self, callback: EventCallback) -> CallbackHandle {
        self.callback_counter += 1;
        let handle = CallbackHandle::new(self.callback_counter);
        self.event_callbacks.insert(handle, callback);
        handle
    }
    
    /// Unregister a callback
    pub fn unregister_callback(&mut self, handle: CallbackHandle) -> ApiResult<()> {
        let removed = self.position_callbacks.remove(&handle).is_some() ||
                     self.event_callbacks.remove(&handle).is_some();
        
        if removed {
            Ok(())
        } else {
            Err(ApiError::InvalidRequest {
                reason: "Invalid callback handle".to_string(),
            })
        }
    }
    
    /// Enable automatic positioning with callbacks
    pub fn enable_auto_positioning(&mut self, request: PositionRequest) {
        self.auto_positioning = Some(request);
        self.collected_anchors.clear();
        self.last_collection_time = Instant::now();
    }
    
    /// Disable automatic positioning
    pub fn disable_auto_positioning(&mut self) {
        self.auto_positioning = None;
        self.collected_anchors.clear();
    }
    
    /// Process incoming data and trigger callbacks (call this regularly)
    pub fn process(&mut self) -> ApiResult<u32> {
        if !self.state.initialized {
            return Err(ApiError::NotInitialized);
        }
        
        let mut messages_processed = 0;
        
        // Collect messages from all transceivers first
        let mut collected_messages = Vec::new();
        let mut transceiver_statuses = Vec::new();
        
        for (transceiver_id, transceiver) in &mut self.transceivers {
            let mut transceiver_errors = 0;
            
            while let Ok(Some(raw_message)) = transceiver.read_message() {
                // Convert hardware RawMessage to parser RawMessage
                let parser_message = crate::processing::parser::RawMessage {
                    data: raw_message.data,
                    timestamp_received: raw_message.timestamp_ms,
                    transceiver_id: *transceiver_id,
                };
                
                collected_messages.push((parser_message, *transceiver_id));
                messages_processed += 1;
            }
            
            // Collect transceiver status
            let status = transceiver.get_status();
            if !status.is_healthy() || transceiver_errors > 0 {
                transceiver_statuses.push((*transceiver_id, status, transceiver_errors));
            }
        }
        
        // Process collected messages
        for (parser_message, transceiver_id) in collected_messages {
            match self.parser.parse_message(&parser_message) {
                Ok(anchor_msg) => {
                    // Validate message
                    let validation_result = self.validator.validate_messages(vec![anchor_msg.clone()]);
                    if validation_result.valid_messages.is_empty() {
                        self.trigger_event(SystemEvent::ErrorOccurred {
                            error: ApiError::InvalidRequest {
                                reason: format!("Invalid anchor message: {:?}", validation_result.rejected_messages),
                            },
                            context: format!("Transceiver {}", transceiver_id),
                        });
                        continue;
                    }
                    let anchor_msg = validation_result.valid_messages[0].clone();
                    
                    // Trigger anchor message event
                    self.trigger_event(SystemEvent::AnchorMessageReceived {
                        anchor_id: anchor_msg.anchor_id.to_string(),
                        timestamp_ms: anchor_msg.timestamp_ms,
                        signal_quality: anchor_msg.signal_quality,
                    });
                    
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
                    
                    // Handle automatic positioning
                    if let Some(ref auto_request) = self.auto_positioning.clone() {
                        self.handle_auto_positioning(anchor, auto_request);
                    }
                }
                Err(e) => {
                    self.trigger_event(SystemEvent::ErrorOccurred {
                        error: ApiError::InvalidRequest {
                            reason: format!("Parse error: {:?}", e),
                        },
                        context: format!("Message parsing on transceiver {}", transceiver_id),
                    });
                }
            }
        }
        
        // Process transceiver status updates
        for (transceiver_id, status, error_count) in transceiver_statuses {
            self.trigger_event(SystemEvent::TransceiverStatusChanged {
                transceiver_id,
                connected: status.connected,
                error_count: status.error_count + error_count,
            });
        }
        
        // Update uptime
        self.state.uptime_ms = self.start_time.elapsed().as_millis() as u64;
        
        Ok(messages_processed)
    }
    
    /// Handle automatic positioning logic
    fn handle_auto_positioning(&mut self, anchor: Anchor, request: &PositionRequest) {
        let current_time = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_millis() as u64;
        
        // Check anchor age
        if current_time.saturating_sub(anchor.timestamp) > request.max_anchor_age_ms as u64 {
            return;
        }
        
        // Add or update anchor in collection
        if let Some(pos) = self.collected_anchors.iter().position(|a| a.id == anchor.id) {
            self.collected_anchors[pos] = anchor;
        } else {
            self.collected_anchors.push(anchor);
        }
        
        // Remove stale anchors
        self.collected_anchors.retain(|a| {
            current_time.saturating_sub(a.timestamp) <= request.max_anchor_age_ms as u64
        });
        
        // Check if we can calculate position
        if self.collected_anchors.len() >= request.min_anchors as usize {
            let start_time = Instant::now();
            
            match self.trilateration.calculate_position_embedded(&self.collected_anchors, current_time) {
                Ok(position) => {
                    let computation_time = start_time.elapsed();
                    self.sequence_counter += 1;
                    
                    // Update statistics
                    self.state.positions_calculated += 1;
                    self.state.last_position_time = Some(current_time);
                    
                    // Update average computation time
                    let comp_time_us = computation_time.as_micros() as u32;
                    if self.state.positions_calculated == 1 {
                        self.state.avg_computation_time_us = comp_time_us;
                    } else {
                        self.state.avg_computation_time_us = 
                            (self.state.avg_computation_time_us + comp_time_us) / 2;
                    }
                    
                    let response = PositionResponse {
                        position,
                        local_position: None,
                        accuracy_estimate: 2.0,
                        timestamp_ms: current_time,
                        anchor_count: self.collected_anchors.len() as u8,
                        geometry_quality: crate::api::types::GeometryQuality::Good,
                        computation_time_us: comp_time_us,
                        sequence_number: self.sequence_counter,
                    };
                    
                    // Trigger position callbacks
                    self.trigger_position_callbacks(response.clone());
                    
                    // Trigger position calculated event
                    self.trigger_event(SystemEvent::PositionCalculated {
                        sequence_number: self.sequence_counter,
                        computation_time_us: comp_time_us,
                        anchor_count: self.collected_anchors.len() as u8,
                    });
                }
                Err(e) => {
                    self.state.error_count += 1;
                    self.trigger_event(SystemEvent::ErrorOccurred {
                        error: ApiError::ComputationFailure {
                            details: format!("Auto-positioning failed: {:?}", e),
                        },
                        context: "Automatic positioning".to_string(),
                    });
                }
            }
        }
    }
    
    /// Trigger position callbacks
    fn trigger_position_callbacks(&self, response: PositionResponse) {
        for callback in self.position_callbacks.values() {
            callback(response.clone());
        }
    }
    
    /// Trigger event callbacks
    fn trigger_event(&self, event: SystemEvent) {
        for callback in self.event_callbacks.values() {
            callback(event.clone());
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
    
    /// Get number of registered callbacks
    pub fn callback_count(&self) -> (usize, usize) {
        (self.position_callbacks.len(), self.event_callbacks.len())
    }
    
    /// Shutdown the positioning system
    pub fn shutdown(&mut self) -> ApiResult<()> {
        // Clear callbacks
        self.position_callbacks.clear();
        self.event_callbacks.clear();
        
        // Clear transceivers
        self.transceivers.clear();
        
        // Reset state
        self.state = SystemState::default();
        self.sequence_counter = 0;
        self.callback_counter = 0;
        self.auto_positioning = None;
        self.collected_anchors.clear();
        
        Ok(())
    }
}

impl Default for CallbackPositioningApi {
    fn default() -> Self {
        Self::new(ApiConfig::default())
    }
}