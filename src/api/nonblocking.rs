//! Non-blocking API for real-time applications
//! 
//! This module provides an asynchronous API suitable for real-time applications
//! where blocking operations are not acceptable.

use crate::api::types::{ApiResult, ApiError, PositionRequest, PositionResponse, SystemState, ApiConfig};
use crate::core::{Position, Anchor};
use crate::hardware::{TransceiverInterface, TransceiverConfig};
use crate::algorithms::embedded_trilateration::EmbeddedTrilateration;
use crate::processing::parser::MessageParser;
use crate::validation::data::DataValidator;
use std::collections::{HashMap, VecDeque};
use std::time::{Instant, SystemTime, UNIX_EPOCH};

/// Request handle for tracking non-blocking operations
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct RequestHandle(u32);

impl RequestHandle {
    fn new(id: u32) -> Self {
        RequestHandle(id)
    }
    
    pub fn id(&self) -> u32 {
        self.0
    }
}

/// Status of a non-blocking request
#[derive(Debug, Clone, PartialEq)]
pub enum RequestStatus {
    /// Request is pending
    Pending,
    /// Request completed successfully
    Completed(PositionResponse),
    /// Request failed with error
    Failed(ApiError),
    /// Request timed out
    TimedOut,
}

/// Internal request state
#[derive(Debug)]
struct PendingRequest {
    handle: RequestHandle,
    request: PositionRequest,
    start_time: Instant,
    collected_anchors: Vec<Anchor>,
}

/// Non-blocking API for underwater positioning
pub struct NonBlockingPositioningApi {
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
    /// Request handle counter
    request_counter: u32,
    /// Pending requests
    pending_requests: HashMap<RequestHandle, PendingRequest>,
    /// Completed requests queue
    completed_requests: VecDeque<(RequestHandle, RequestStatus)>,
    /// Start time for uptime calculation
    start_time: Instant,
}

impl NonBlockingPositioningApi {
    /// Create a new non-blocking positioning API
    pub fn new(config: ApiConfig) -> Self {
        Self {
            trilateration: EmbeddedTrilateration::default(),
            parser: MessageParser::new(),
            validator: DataValidator::new(),
            transceivers: HashMap::new(),
            config,
            state: SystemState::default(),
            sequence_counter: 0,
            request_counter: 0,
            pending_requests: HashMap::new(),
            completed_requests: VecDeque::new(),
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
    
    /// Request position calculation (non-blocking)
    pub fn request_position(&mut self) -> ApiResult<RequestHandle> {
        let request = PositionRequest::default();
        self.request_position_with_params(request)
    }
    
    /// Request position calculation with specific parameters (non-blocking)
    pub fn request_position_with_params(&mut self, request: PositionRequest) -> ApiResult<RequestHandle> {
        if !self.state.initialized {
            return Err(ApiError::NotInitialized);
        }
        
        // Check queue size limit
        if self.pending_requests.len() >= self.config.max_queue_size {
            return Err(ApiError::InvalidRequest {
                reason: "Request queue full".to_string(),
            });
        }
        
        // Create new request handle
        self.request_counter += 1;
        let handle = RequestHandle::new(self.request_counter);
        
        // Create pending request
        let pending = PendingRequest {
            handle,
            request,
            start_time: Instant::now(),
            collected_anchors: Vec::new(),
        };
        
        self.pending_requests.insert(handle, pending);
        
        Ok(handle)
    }
    
    /// Check the status of a request
    pub fn check_request_status(&mut self, handle: RequestHandle) -> ApiResult<RequestStatus> {
        // Check completed requests first
        if let Some(pos) = self.completed_requests.iter().position(|(h, _)| *h == handle) {
            let (_, status) = self.completed_requests.remove(pos).unwrap();
            return Ok(status);
        }
        
        // Check if request is still pending
        if self.pending_requests.contains_key(&handle) {
            Ok(RequestStatus::Pending)
        } else {
            Err(ApiError::InvalidRequest {
                reason: "Invalid request handle".to_string(),
            })
        }
    }
    
    /// Process pending requests (call this regularly in your main loop)
    pub fn process_requests(&mut self) -> ApiResult<u32> {
        let mut processed_count = 0;
        
        // Collect new anchor messages
        let mut new_anchors = Vec::new();
        for transceiver in self.transceivers.values_mut() {
            while let Ok(Some(raw_message)) = transceiver.read_message() {
                // Convert hardware RawMessage to parser RawMessage
                let parser_message = crate::processing::parser::RawMessage {
                    data: raw_message.data,
                    timestamp_received: raw_message.timestamp_ms,
                    transceiver_id: transceiver.get_id(),
                };
                match self.parser.parse_message(&parser_message) {
                    Ok(anchor_msg) => {
                        // Validate message
                        let validation_result = self.validator.validate_messages(vec![anchor_msg.clone()]);
                        if !validation_result.valid_messages.is_empty() {
                            let anchor_msg = validation_result.valid_messages[0].clone();
                            let anchor = Anchor {
                                id: anchor_msg.anchor_id.to_string(),
                                timestamp: anchor_msg.timestamp_ms,
                                position: Position {
                                    lat: anchor_msg.position.latitude,
                                    lon: anchor_msg.position.longitude,
                                    depth: anchor_msg.position.depth,
                                },
                            };
                            new_anchors.push(anchor);
                        }
                    }
                    Err(_) => {
                        // Ignore parsing errors in non-blocking mode
                    }
                }
            }
        }
        
        // Process each pending request
        let mut completed_handles = Vec::new();
        
        for (handle, pending) in &mut self.pending_requests {
            // Check for timeout
            if pending.start_time.elapsed().as_millis() > pending.request.timeout_ms as u128 {
                completed_handles.push((*handle, RequestStatus::TimedOut));
                continue;
            }
            
            // Add new anchors that match age requirements
            let current_time = SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap()
                .as_millis() as u64;
            
            for anchor in &new_anchors {
                if current_time.saturating_sub(anchor.timestamp) <= pending.request.max_anchor_age_ms as u64 {
                    // Check if we already have this anchor
                    if !pending.collected_anchors.iter().any(|a| a.id == anchor.id) {
                        pending.collected_anchors.push(anchor.clone());
                    }
                }
            }
            
            // Check if we have enough anchors to calculate position
            if pending.collected_anchors.len() >= pending.request.min_anchors as usize {
                match self.trilateration.calculate_position_embedded(&pending.collected_anchors, current_time) {
                    Ok(position) => {
                        let computation_time = pending.start_time.elapsed();
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
                            anchor_count: pending.collected_anchors.len() as u8,
                            geometry_quality: crate::api::types::GeometryQuality::Good,
                            computation_time_us: comp_time_us,
                            sequence_number: self.sequence_counter,
                        };
                        
                        completed_handles.push((*handle, RequestStatus::Completed(response)));
                    }
                    Err(e) => {
                        self.state.error_count += 1;
                        let error = ApiError::ComputationFailure {
                            details: format!("Trilateration failed: {:?}", e),
                        };
                        completed_handles.push((*handle, RequestStatus::Failed(error)));
                    }
                }
            }
        }
        
        // Move completed requests to completed queue
        for (handle, status) in completed_handles {
            self.pending_requests.remove(&handle);
            
            // Limit completed queue size
            if self.completed_requests.len() >= self.config.max_queue_size {
                self.completed_requests.pop_front();
            }
            
            self.completed_requests.push_back((handle, status));
            processed_count += 1;
        }
        
        // Update uptime
        self.state.uptime_ms = self.start_time.elapsed().as_millis() as u64;
        
        Ok(processed_count)
    }
    
    /// Cancel a pending request
    pub fn cancel_request(&mut self, handle: RequestHandle) -> ApiResult<()> {
        if self.pending_requests.remove(&handle).is_some() {
            Ok(())
        } else {
            Err(ApiError::InvalidRequest {
                reason: "Request not found or already completed".to_string(),
            })
        }
    }
    
    /// Get number of pending requests
    pub fn pending_request_count(&self) -> usize {
        self.pending_requests.len()
    }
    
    /// Get number of completed requests waiting to be retrieved
    pub fn completed_request_count(&self) -> usize {
        self.completed_requests.len()
    }
    
    /// Clear all completed requests
    pub fn clear_completed_requests(&mut self) {
        self.completed_requests.clear();
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
    
    /// Shutdown the positioning system
    pub fn shutdown(&mut self) -> ApiResult<()> {
        // Clear all requests
        self.pending_requests.clear();
        self.completed_requests.clear();
        
        // Clear transceivers
        self.transceivers.clear();
        
        // Reset state
        self.state = SystemState::default();
        self.sequence_counter = 0;
        self.request_counter = 0;
        
        Ok(())
    }
}

impl Default for NonBlockingPositioningApi {
    fn default() -> Self {
        Self::new(ApiConfig::default())
    }
}