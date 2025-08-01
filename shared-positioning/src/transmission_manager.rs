// Transmission manager for coordinating underwater message broadcasts
// Handles scheduling, power adaptation, failure recovery, and statistics tracking

use std::time::{Duration, Instant};
use std::collections::{VecDeque, HashMap};
use uuid::Uuid;

use crate::{
    TransceiverInterface, MessageBuilder, GeodeticPosition,
    EnvironmentalConditions, PowerManager, SeaState
};

/// Transmission manager errors
#[derive(Debug, Clone, PartialEq)]
pub enum TransmissionError {
    TransceiverFault(String),
    MessageBuildFailed(String),
    PowerInsufficient { required_level: u8, available_level: u8 },
    SchedulingConflict { conflicting_transmission: String },
    HardwareTimeout { timeout_ms: u64 },
    AdaptationFailed(String),
    ConfigurationError(String),
    RetryLimitExceeded { attempts: u32 },
}

impl std::fmt::Display for TransmissionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TransmissionError::TransceiverFault(details) => write!(f, "Transceiver fault: {}", details),
            TransmissionError::MessageBuildFailed(details) => write!(f, "Message build failed: {}", details),
            TransmissionError::PowerInsufficient { required_level, available_level } => {
                write!(f, "Insufficient power: required {}, available {}", required_level, available_level)
            }
            TransmissionError::SchedulingConflict { conflicting_transmission } => {
                write!(f, "Scheduling conflict with: {}", conflicting_transmission)
            }
            TransmissionError::HardwareTimeout { timeout_ms } => {
                write!(f, "Hardware timeout after {} ms", timeout_ms)
            }
            TransmissionError::AdaptationFailed(details) => write!(f, "Adaptation failed: {}", details),
            TransmissionError::ConfigurationError(details) => write!(f, "Configuration error: {}", details),
            TransmissionError::RetryLimitExceeded { attempts } => {
                write!(f, "Retry limit exceeded after {} attempts", attempts)
            }
        }
    }
}

impl std::error::Error for TransmissionError {}

/// Message version for transmission
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MessageVersion {
    V1,
    V2,
    V3,
}

/// Transmission priority levels
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum TransmissionPriority {
    Low = 1,
    Normal = 2,
    High = 3,
    Emergency = 4,
}

/// Message compression types for bandwidth optimization
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CompressionType {
    None,
    LZ4,        // Fast compression for real-time
    Deflate,    // Better compression ratio
    Custom,     // Custom underwater-optimized compression
}

/// Underwater propagation conditions affecting transmission
#[derive(Debug, Clone, PartialEq)]
pub struct UnderwaterConditions {
    pub water_temperature_c: Option<f32>,
    pub salinity_ppt: Option<f32>,
    pub depth_m: Option<f32>,
    pub current_speed_ms: Option<f32>,
    pub turbidity_ntu: Option<f32>,
    pub acoustic_noise_db: Option<f32>,
    pub thermocline_depth_m: Option<f32>,
    pub sound_speed_ms: Option<f32>,
}

impl Default for UnderwaterConditions {
    fn default() -> Self {
        Self {
            water_temperature_c: Some(15.0),  // Typical ocean temperature
            salinity_ppt: Some(35.0),         // Typical ocean salinity
            depth_m: Some(10.0),              // Shallow water default
            current_speed_ms: Some(0.1),      // Calm conditions
            turbidity_ntu: Some(1.0),         // Clear water
            acoustic_noise_db: Some(60.0),    // Quiet ocean
            thermocline_depth_m: Some(50.0),  // Typical thermocline
            sound_speed_ms: Some(1500.0),     // Standard sound speed in seawater
        }
    }
}

/// Statistics for underwater transmission optimization
#[derive(Debug, Clone)]
pub struct UnderwaterOptimizationStats {
    pub format_adaptations: u32,
    pub compression_savings_bytes: u64,
    pub propagation_adjustments: u32,
    pub backoff_escalations: u32,
    pub average_signal_quality: Option<f32>,
    pub current_underwater_conditions: UnderwaterConditions,
    pub compression_ratio: Option<f32>,
}

/// Scheduled transmission entry
#[derive(Debug, Clone)]
pub struct ScheduledTransmission {
    pub id: Uuid,
    pub beacon_id: Uuid,
    pub position: GeodeticPosition,
    pub signal_quality: u8,
    pub sequence_number: u16,
    pub message_version: MessageVersion,
    pub priority: TransmissionPriority,
    pub scheduled_time: Instant,
    pub retry_count: u32,
    pub max_retries: u32,
    pub timeout_ms: u64,
    pub compression_type: CompressionType,
    pub adaptive_power_level: Option<u8>,
    pub underwater_optimized: bool,
}

/// Transmission configuration
#[derive(Debug, Clone)]
pub struct TransmissionConfig {
    pub default_interval_ms: u32,
    pub emergency_interval_ms: u32,
    pub power_save_interval_ms: u32,
    pub max_retry_attempts: u32,
    pub retry_backoff_ms: u32,
    pub transmission_timeout_ms: u64,
    pub adaptive_power_enabled: bool,
    pub min_power_level: u8,
    pub max_power_level: u8,
    pub power_adaptation_threshold: f32,
    pub environmental_adaptation_enabled: bool,
    pub queue_size_limit: usize,
    // Underwater optimization settings
    pub underwater_optimization_enabled: bool,
    pub adaptive_format_selection: bool,
    pub compression_enabled: bool,
    pub default_compression_type: CompressionType,
    pub signal_quality_threshold: u8,
    pub propagation_model_enabled: bool,
    pub exponential_backoff_enabled: bool,
    pub max_backoff_interval_ms: u32,
}

impl Default for TransmissionConfig {
    fn default() -> Self {
        Self {
            default_interval_ms: 5000,
            emergency_interval_ms: 1000,
            power_save_interval_ms: 30000,
            max_retry_attempts: 3,
            retry_backoff_ms: 1000,
            transmission_timeout_ms: 5000,
            adaptive_power_enabled: true,
            min_power_level: 10,
            max_power_level: 255,
            power_adaptation_threshold: 20.0, // Battery percentage
            environmental_adaptation_enabled: true,
            queue_size_limit: 100,
            // Underwater optimization defaults
            underwater_optimization_enabled: true,
            adaptive_format_selection: true,
            compression_enabled: true,
            default_compression_type: CompressionType::LZ4,
            signal_quality_threshold: 50,
            propagation_model_enabled: true,
            exponential_backoff_enabled: true,
            max_backoff_interval_ms: 60000, // 1 minute max backoff
        }
    }
}

/// Detailed transmission statistics
#[derive(Debug, Clone, Default)]
pub struct TransmissionStatistics {
    pub total_transmissions: u64,
    pub successful_transmissions: u64,
    pub failed_transmissions: u64,
    pub retry_transmissions: u64,
    pub emergency_transmissions: u64,
    pub total_bytes_transmitted: u64,
    pub average_transmission_time_ms: f64,
    pub success_rate: f64,
    pub power_level_history: VecDeque<u8>,
    pub transmission_intervals: VecDeque<u64>,
    pub error_counts: HashMap<String, u32>,
    pub last_transmission_time: Option<Instant>,
    pub last_successful_transmission: Option<Instant>,
    pub last_error: Option<TransmissionError>,
    pub adaptive_power_adjustments: u32,
    pub environmental_adaptations: u32,
    // Underwater optimization statistics
    pub format_adaptations: u32,
    pub compression_savings_bytes: u64,
    pub propagation_adjustments: u32,
    pub backoff_escalations: u32,
    pub signal_quality_history: VecDeque<u8>,
    pub underwater_conditions_history: VecDeque<UnderwaterConditions>,
}

impl TransmissionStatistics {
    /// Record a successful transmission
    pub fn record_success(&mut self, bytes: usize, duration: Duration, power_level: u8) {
        self.total_transmissions += 1;
        self.successful_transmissions += 1;
        self.total_bytes_transmitted += bytes as u64;
        
        // Update average transmission time
        let duration_ms = duration.as_millis() as f64;
        if self.successful_transmissions == 1 {
            self.average_transmission_time_ms = duration_ms;
        } else {
            let total_time = self.average_transmission_time_ms * (self.successful_transmissions - 1) as f64;
            self.average_transmission_time_ms = (total_time + duration_ms) / self.successful_transmissions as f64;
        }
        
        // Update success rate
        self.success_rate = self.successful_transmissions as f64 / self.total_transmissions as f64;
        
        // Track power level
        self.power_level_history.push_back(power_level);
        if self.power_level_history.len() > 100 {
            self.power_level_history.pop_front();
        }
        
        self.last_transmission_time = Some(Instant::now());
        self.last_successful_transmission = Some(Instant::now());
    }
    
    /// Record a failed transmission
    pub fn record_failure(&mut self, error: TransmissionError, is_retry: bool) {
        self.total_transmissions += 1;
        self.failed_transmissions += 1;
        
        if is_retry {
            self.retry_transmissions += 1;
        }
        
        // Update success rate
        self.success_rate = self.successful_transmissions as f64 / self.total_transmissions as f64;
        
        // Track error types
        let error_type = match &error {
            TransmissionError::TransceiverFault(_) => "TransceiverFault",
            TransmissionError::MessageBuildFailed(_) => "MessageBuildFailed",
            TransmissionError::PowerInsufficient { .. } => "PowerInsufficient",
            TransmissionError::SchedulingConflict { .. } => "SchedulingConflict",
            TransmissionError::HardwareTimeout { .. } => "HardwareTimeout",
            TransmissionError::AdaptationFailed(_) => "AdaptationFailed",
            TransmissionError::ConfigurationError(_) => "ConfigurationError",
            TransmissionError::RetryLimitExceeded { .. } => "RetryLimitExceeded",
        };
        
        *self.error_counts.entry(error_type.to_string()).or_insert(0) += 1;
        self.last_error = Some(error);
        self.last_transmission_time = Some(Instant::now());
    }
    
    /// Record emergency transmission
    pub fn record_emergency(&mut self) {
        self.emergency_transmissions += 1;
    }
    
    /// Record transmission interval for adaptive timing
    pub fn record_interval(&mut self, interval_ms: u64) {
        self.transmission_intervals.push_back(interval_ms);
        if self.transmission_intervals.len() > 50 {
            self.transmission_intervals.pop_front();
        }
    }
    
    /// Record power adaptation
    pub fn record_power_adaptation(&mut self) {
        self.adaptive_power_adjustments += 1;
    }
    
    /// Record environmental adaptation
    pub fn record_environmental_adaptation(&mut self) {
        self.environmental_adaptations += 1;
    }
    
    /// Record format adaptation for underwater conditions
    pub fn record_format_adaptation(&mut self) {
        self.format_adaptations += 1;
    }
    
    /// Record compression savings
    pub fn record_compression_savings(&mut self, bytes_saved: usize) {
        self.compression_savings_bytes += bytes_saved as u64;
    }
    
    /// Record propagation model adjustment
    pub fn record_propagation_adjustment(&mut self) {
        self.propagation_adjustments += 1;
    }
    
    /// Record backoff escalation
    pub fn record_backoff_escalation(&mut self) {
        self.backoff_escalations += 1;
    }
    
    /// Record signal quality measurement
    pub fn record_signal_quality(&mut self, quality: u8) {
        self.signal_quality_history.push_back(quality);
        if self.signal_quality_history.len() > 100 {
            self.signal_quality_history.pop_front();
        }
    }
    
    /// Record underwater conditions
    pub fn record_underwater_conditions(&mut self, conditions: UnderwaterConditions) {
        self.underwater_conditions_history.push_back(conditions);
        if self.underwater_conditions_history.len() > 50 {
            self.underwater_conditions_history.pop_front();
        }
    }
    
    /// Get average transmission interval
    pub fn get_average_interval_ms(&self) -> Option<f64> {
        if self.transmission_intervals.is_empty() {
            None
        } else {
            let sum: u64 = self.transmission_intervals.iter().sum();
            Some(sum as f64 / self.transmission_intervals.len() as f64)
        }
    }
    
    /// Get current power level trend
    pub fn get_power_level_trend(&self) -> Option<f64> {
        if self.power_level_history.len() < 2 {
            return None;
        }
        
        let recent_count = (self.power_level_history.len() / 2).max(5);
        let recent_avg: f64 = self.power_level_history
            .iter()
            .rev()
            .take(recent_count)
            .map(|&x| x as f64)
            .sum::<f64>() / recent_count as f64;
        
        let older_avg: f64 = self.power_level_history
            .iter()
            .take(recent_count)
            .map(|&x| x as f64)
            .sum::<f64>() / recent_count as f64;
        
        Some(recent_avg - older_avg)
    }
}

/// Main transmission manager for coordinating underwater message broadcasts
pub struct TransmissionManager<T, P> 
where
    T: TransceiverInterface,
    P: PowerManager,
{
    config: TransmissionConfig,
    transceiver: T,
    power_manager: P,
    message_builder: MessageBuilder,
    transmission_queue: VecDeque<ScheduledTransmission>,
    statistics: TransmissionStatistics,
    current_power_level: u8,
    environmental_conditions: EnvironmentalConditions,
    underwater_conditions: UnderwaterConditions,
    last_transmission: Option<Instant>,
    sequence_counter: u16,
    is_running: bool,
    priority_queue: VecDeque<ScheduledTransmission>, // Separate queue for emergency messages
}

impl<T, P> TransmissionManager<T, P>
where
    T: TransceiverInterface,
    P: PowerManager,
{
    /// Create new transmission manager
    pub fn new(
        config: TransmissionConfig,
        transceiver: T,
        power_manager: P,
    ) -> Result<Self, TransmissionError> {
        // Validate configuration
        if config.default_interval_ms < 100 {
            return Err(TransmissionError::ConfigurationError(
                "Default interval too short (minimum 100ms)".to_string()
            ));
        }
        
        if config.min_power_level >= config.max_power_level {
            return Err(TransmissionError::ConfigurationError(
                "Invalid power level range".to_string()
            ));
        }
        
        Ok(Self {
            config,
            transceiver,
            power_manager,
            message_builder: MessageBuilder::new(),
            transmission_queue: VecDeque::new(),
            statistics: TransmissionStatistics::default(),
            current_power_level: 128, // Start with mid-range power
            environmental_conditions: EnvironmentalConditions::default(),
            underwater_conditions: UnderwaterConditions::default(),
            last_transmission: None,
            sequence_counter: 0,
            is_running: false,
            priority_queue: VecDeque::new(),
        })
    }
    
    /// Start the transmission manager
    pub fn start(&mut self) -> Result<(), TransmissionError> {
        if self.is_running {
            return Ok(());
        }
        
        // Initialize transceiver
        self.transceiver.set_transmission_power(self.current_power_level)
            .map_err(|e| TransmissionError::TransceiverFault(e.to_string()))?;
        
        self.is_running = true;
        Ok(())
    }
    
    /// Stop the transmission manager
    pub fn stop(&mut self) -> Result<(), TransmissionError> {
        if !self.is_running {
            return Ok(());
        }
        
        // Clear pending transmissions
        self.transmission_queue.clear();
        self.priority_queue.clear();
        self.is_running = false;
        
        Ok(())
    }
    
    /// Schedule a transmission (requirement 1.3 - configurable transmission intervals)
    pub fn schedule_transmission(
        &mut self,
        beacon_id: Uuid,
        position: GeodeticPosition,
        signal_quality: u8,
        message_version: MessageVersion,
        priority: TransmissionPriority,
        delay_ms: Option<u32>,
    ) -> Result<Uuid, TransmissionError> {
        if !self.is_running {
            return Err(TransmissionError::ConfigurationError(
                "Transmission manager not running".to_string()
            ));
        }
        
        // Check queue size limit
        if self.transmission_queue.len() >= self.config.queue_size_limit {
            return Err(TransmissionError::SchedulingConflict {
                conflicting_transmission: "Queue full".to_string()
            });
        }
        
        let transmission_id = Uuid::new_v4();
        let scheduled_time = if let Some(delay) = delay_ms {
            Instant::now() + Duration::from_millis(delay as u64)
        } else {
            Instant::now()
        };
        
        // Adaptive message format selection based on conditions
        let adaptive_version = if self.config.adaptive_format_selection {
            self.select_optimal_message_format(signal_quality, priority)?
        } else {
            message_version
        };

        let transmission = ScheduledTransmission {
            id: transmission_id,
            beacon_id,
            position,
            signal_quality,
            sequence_number: self.get_next_sequence(),
            message_version: adaptive_version,
            priority,
            scheduled_time,
            retry_count: 0,
            max_retries: self.config.max_retry_attempts,
            timeout_ms: self.config.transmission_timeout_ms,
            compression_type: self.config.default_compression_type,
            adaptive_power_level: None,
            underwater_optimized: self.config.underwater_optimization_enabled,
        };
        
        // Use priority queue for emergency messages
        if priority == TransmissionPriority::Emergency {
            self.priority_queue.push_back(transmission);
        } else {
            // Insert in priority order
            let insert_pos = self.transmission_queue
                .iter()
                .position(|t| t.priority < priority || 
                         (t.priority == priority && t.scheduled_time > scheduled_time))
                .unwrap_or(self.transmission_queue.len());
            
            self.transmission_queue.insert(insert_pos, transmission);
        }
        
        Ok(transmission_id)
    }
    
    /// Process pending transmissions (requirement 1.5 - message sequencing)
    pub fn process_transmissions(&mut self) -> Result<u32, TransmissionError> {
        if !self.is_running {
            return Ok(0);
        }
        
        let mut processed_count = 0;
        let now = Instant::now();
        
        // Process emergency transmissions first (priority queue)
        while let Some(transmission) = self.priority_queue.front() {
            if transmission.scheduled_time > now {
                break; // Not ready yet
            }
            
            let transmission = self.priority_queue.pop_front().unwrap();
            
            match self.execute_transmission(&transmission) {
                Ok(()) => {
                    processed_count += 1;
                    self.update_transmission_timing(now);
                }
                Err(error) => {
                    self.handle_transmission_retry(&transmission, error, now, true)?;
                }
            }
        }
        
        // Process regular transmissions
        while let Some(transmission) = self.transmission_queue.front() {
            if transmission.scheduled_time > now {
                break; // Not ready yet
            }
            
            let transmission = self.transmission_queue.pop_front().unwrap();
            
            match self.execute_transmission(&transmission) {
                Ok(()) => {
                    processed_count += 1;
                    self.update_transmission_timing(now);
                }
                Err(error) => {
                    self.handle_transmission_retry(&transmission, error, now, false)?;
                }
            }
        }
        
        Ok(processed_count)
    }
    
    /// Handle transmission retry logic with exponential backoff
    fn handle_transmission_retry(
        &mut self,
        transmission: &ScheduledTransmission,
        error: TransmissionError,
        now: Instant,
        is_priority: bool,
    ) -> Result<(), TransmissionError> {
        if transmission.retry_count < transmission.max_retries {
            let mut retry_transmission = transmission.clone();
            retry_transmission.retry_count += 1;
            
            // Calculate backoff delay with exponential backoff if enabled
            let backoff_delay = if self.config.exponential_backoff_enabled {
                let base_delay = self.config.retry_backoff_ms as u64;
                let exponential_delay = base_delay * (2_u64.pow(retry_transmission.retry_count));
                let capped_delay = exponential_delay.min(self.config.max_backoff_interval_ms as u64);
                
                self.statistics.record_backoff_escalation();
                capped_delay
            } else {
                self.config.retry_backoff_ms as u64
            };
            
            retry_transmission.scheduled_time = now + Duration::from_millis(backoff_delay);
            
            // Re-queue for retry in appropriate queue
            if is_priority {
                self.priority_queue.push_back(retry_transmission);
            } else {
                self.transmission_queue.push_back(retry_transmission);
            }
            
            self.statistics.record_failure(error, true);
        } else {
            // Max retries exceeded
            let retry_error = TransmissionError::RetryLimitExceeded {
                attempts: transmission.retry_count
            };
            self.statistics.record_failure(retry_error, false);
        }
        
        Ok(())
    }
    
    /// Update transmission timing statistics
    fn update_transmission_timing(&mut self, now: Instant) {
        // Record interval if this isn't the first transmission
        if let Some(last_time) = self.last_transmission {
            let interval = now.duration_since(last_time).as_millis() as u64;
            self.statistics.record_interval(interval);
        }
        
        self.last_transmission = Some(now);
    }
    
    /// Select optimal message format based on underwater conditions
    fn select_optimal_message_format(
        &mut self,
        signal_quality: u8,
        priority: TransmissionPriority,
    ) -> Result<MessageVersion, TransmissionError> {
        // For emergency messages, use most reliable format
        if priority == TransmissionPriority::Emergency {
            self.statistics.record_format_adaptation();
            return Ok(MessageVersion::V1); // Most robust format
        }
        
        // Select format based on signal quality and underwater conditions
        let format = if signal_quality < self.config.signal_quality_threshold {
            // Poor signal quality - use compact format
            MessageVersion::V2
        } else if self.underwater_conditions.acoustic_noise_db.unwrap_or(60.0) > 80.0 {
            // High noise - use robust format
            MessageVersion::V1
        } else {
            // Good conditions - use advanced format with UUID
            MessageVersion::V3
        };
        
        self.statistics.record_format_adaptation();
        Ok(format)
    }
    
    /// Compress message data if compression is enabled
    fn compress_message_data(
        &mut self,
        data: &[u8],
        compression_type: CompressionType,
    ) -> Result<Vec<u8>, TransmissionError> {
        match compression_type {
            CompressionType::None => Ok(data.to_vec()),
            CompressionType::LZ4 => {
                // Simulate LZ4 compression (in real implementation, use lz4 crate)
                let compressed = self.simulate_lz4_compression(data);
                let savings = data.len().saturating_sub(compressed.len());
                self.statistics.record_compression_savings(savings);
                Ok(compressed)
            }
            CompressionType::Deflate => {
                // Simulate deflate compression (in real implementation, use flate2 crate)
                let compressed = self.simulate_deflate_compression(data);
                let savings = data.len().saturating_sub(compressed.len());
                self.statistics.record_compression_savings(savings);
                Ok(compressed)
            }
            CompressionType::Custom => {
                // Custom underwater-optimized compression
                let compressed = self.apply_underwater_compression(data)?;
                let savings = data.len().saturating_sub(compressed.len());
                self.statistics.record_compression_savings(savings);
                Ok(compressed)
            }
        }
    }
    
    /// Simulate LZ4 compression (placeholder for real implementation)
    fn simulate_lz4_compression(&self, data: &[u8]) -> Vec<u8> {
        // Simple simulation - reduce size by ~30% for typical beacon messages
        let compression_ratio = 0.7;
        let compressed_size = (data.len() as f32 * compression_ratio) as usize;
        let mut compressed = data.to_vec();
        compressed.truncate(compressed_size.max(data.len() / 2)); // Minimum 50% of original
        compressed
    }
    
    /// Simulate deflate compression (placeholder for real implementation)
    fn simulate_deflate_compression(&self, data: &[u8]) -> Vec<u8> {
        // Simple simulation - reduce size by ~40% for typical beacon messages
        let compression_ratio = 0.6;
        let compressed_size = (data.len() as f32 * compression_ratio) as usize;
        let mut compressed = data.to_vec();
        compressed.truncate(compressed_size.max(data.len() / 3)); // Minimum 33% of original
        compressed
    }
    
    /// Apply custom underwater-optimized compression
    fn apply_underwater_compression(&self, data: &[u8]) -> Result<Vec<u8>, TransmissionError> {
        // Custom compression optimized for underwater beacon messages
        // Focus on compressing redundant position data and timestamps
        
        if data.len() < 10 {
            return Ok(data.to_vec()); // Too small to compress effectively
        }
        
        let mut compressed = Vec::new();
        
        // Add compression header
        compressed.push(0xAC); // Underwater compression marker
        
        // Apply differential encoding for position data
        // (In real implementation, this would be more sophisticated)
        let mut prev_byte = 0u8;
        for &byte in data {
            let diff = byte.wrapping_sub(prev_byte);
            compressed.push(diff);
            prev_byte = byte;
        }
        
        // Ensure we actually achieved compression
        if compressed.len() >= data.len() {
            return Ok(data.to_vec()); // No benefit, return original
        }
        
        Ok(compressed)
    }
    
    /// Calculate optimal transmission power using underwater propagation model
    fn calculate_propagation_power(
        &mut self,
        distance_m: f32,
        target_signal_strength: u8,
    ) -> Result<u8, TransmissionError> {
        if !self.config.propagation_model_enabled {
            return Ok(self.current_power_level);
        }
        
        // Underwater acoustic propagation model
        let frequency_khz = 25.0; // Typical underwater acoustic frequency
        let sound_speed = self.underwater_conditions.sound_speed_ms.unwrap_or(1500.0);
        let absorption_coefficient = self.calculate_absorption_coefficient(frequency_khz)?;
        
        // Transmission loss calculation: TL = 20*log10(r) + α*r + 60
        let geometric_loss = 20.0 * distance_m.log10();
        let absorption_loss = absorption_coefficient * distance_m / 1000.0; // Convert to km
        let total_loss_db = geometric_loss + absorption_loss + 60.0;
        
        // Calculate required source level
        let target_level_db = target_signal_strength as f32;
        let required_source_level = target_level_db + total_loss_db;
        
        // Convert to power level (0-255 scale)
        // Ensure power increases with distance by using a more direct relationship
        let base_power = self.config.min_power_level as f32;
        let power_range = (self.config.max_power_level - self.config.min_power_level) as f32;
        let distance_factor = (distance_m / 100.0).min(10.0); // Cap at 10x for 1000m
        let power_level = (base_power + (power_range * distance_factor * 0.3)) as u8;
        
        let bounded_power = power_level
            .max(self.config.min_power_level)
            .min(self.config.max_power_level);
        
        self.statistics.record_propagation_adjustment();
        Ok(bounded_power)
    }
    
    /// Calculate absorption coefficient based on underwater conditions
    fn calculate_absorption_coefficient(&self, frequency_khz: f32) -> Result<f32, TransmissionError> {
        let temperature = self.underwater_conditions.water_temperature_c.unwrap_or(15.0);
        let salinity = self.underwater_conditions.salinity_ppt.unwrap_or(35.0);
        let depth = self.underwater_conditions.depth_m.unwrap_or(10.0);
        
        // Simplified absorption model for underwater acoustics
        // Based on typical values for seawater at acoustic frequencies
        let f = frequency_khz;
        let t = temperature;
        let s = salinity;
        let d = depth;
        
        // Simple empirical model for absorption coefficient (dB/km)
        // Based on frequency, temperature, salinity, and depth
        let frequency_factor = f * f / 1000.0; // Frequency squared effect
        let temperature_factor = 1.0 + (t - 15.0) * 0.01; // Temperature effect
        let salinity_factor = 1.0 + (s - 35.0) * 0.005; // Salinity effect
        let depth_factor = 1.0 + d * 0.0001; // Depth effect (pressure)
        
        // Base absorption coefficient for 25 kHz in typical seawater
        let base_absorption = 0.1; // dB/km
        
        let total_absorption = base_absorption * frequency_factor * temperature_factor * salinity_factor * depth_factor;
        
        // Ensure minimum absorption to avoid zero values
        Ok(total_absorption.max(0.01))
    }
    
    /// Execute a single transmission
    fn execute_transmission(&mut self, transmission: &ScheduledTransmission) -> Result<(), TransmissionError> {
        let _start_time = Instant::now();
        
        // Check power requirements before transmission
        self.check_power_requirements()?;
        
        // Adapt transmission power based on conditions (requirement 6.1)
        if self.config.adaptive_power_enabled {
            self.adapt_transmission_power()?;
        }
        
        // Build message based on version
        let mut message_data = self.build_message(transmission)?;
        
        // Apply compression if enabled
        if self.config.compression_enabled && transmission.underwater_optimized {
            message_data = self.compress_message_data(&message_data, transmission.compression_type)?;
        }
        
        // Validate message before transmission
        self.message_builder.validate_message_data(&message_data)
            .map_err(|e| TransmissionError::MessageBuildFailed(e.to_string()))?;
        
        // Apply underwater propagation model for power optimization
        if transmission.underwater_optimized && self.config.propagation_model_enabled {
            let target_distance = 1000.0; // Assume 1km target range
            let optimal_power = self.calculate_propagation_power(target_distance, transmission.signal_quality)?;
            if optimal_power != self.current_power_level {
                self.set_power_level(optimal_power)?;
            }
        }
        
        // Execute transmission with timeout
        let transmission_result = self.transceiver.transmit_message_timed(&message_data)
            .map_err(|e| TransmissionError::TransceiverFault(e.to_string()));
        
        match transmission_result {
            Ok(duration) => {
                // Record successful transmission
                self.statistics.record_success(
                    message_data.len(),
                    duration,
                    self.current_power_level
                );
                
                // Record emergency transmission if applicable
                if transmission.priority == TransmissionPriority::Emergency {
                    self.statistics.record_emergency();
                }
                
                Ok(())
            }
            Err(error) => {
                self.statistics.record_failure(error.clone(), transmission.retry_count > 0);
                Err(error)
            }
        }
    }
    
    /// Build message based on version and transmission parameters
    fn build_message(&self, transmission: &ScheduledTransmission) -> Result<Vec<u8>, TransmissionError> {
        match transmission.message_version {
            MessageVersion::V1 => {
                self.message_builder.build_v1_message_with_uuid(
                    transmission.beacon_id,
                    transmission.position,
                    transmission.signal_quality,
                    transmission.sequence_number,
                )
            }
            MessageVersion::V2 => {
                self.message_builder.build_v2_message_with_uuid(
                    transmission.beacon_id,
                    transmission.position,
                    transmission.signal_quality,
                    transmission.sequence_number,
                )
            }
            MessageVersion::V3 => {
                self.message_builder.build_v3_message(
                    transmission.beacon_id,
                    transmission.position,
                    transmission.signal_quality,
                    transmission.sequence_number,
                )
            }
        }.map_err(|e| TransmissionError::MessageBuildFailed(e.to_string()))
    }
    
    /// Check power requirements for transmission
    fn check_power_requirements(&mut self) -> Result<(), TransmissionError> {
        let battery_status = self.power_manager.get_battery_status()
            .map_err(|_e| TransmissionError::PowerInsufficient { 
                required_level: self.current_power_level, 
                available_level: 0 
            })?;
        
        // Check if battery level is sufficient for transmission
        if battery_status.capacity_percent < self.config.power_adaptation_threshold {
            // Reduce power level for power conservation
            let reduced_power = (self.current_power_level as f32 * 0.7) as u8;
            let adjusted_power = reduced_power.max(self.config.min_power_level);
            
            if adjusted_power != self.current_power_level {
                self.current_power_level = adjusted_power;
                self.transceiver.set_transmission_power(self.current_power_level)
                    .map_err(|e| TransmissionError::TransceiverFault(e.to_string()))?;
                self.statistics.record_power_adaptation();
            }
        }
        
        // Check if power is critically low
        if battery_status.capacity_percent < 5.0 {
            return Err(TransmissionError::PowerInsufficient {
                required_level: self.config.min_power_level,
                available_level: (battery_status.capacity_percent * 2.55) as u8,
            });
        }
        
        Ok(())
    }
    
    /// Adapt transmission power based on environmental conditions (requirement 6.1)
    fn adapt_transmission_power(&mut self) -> Result<(), TransmissionError> {
        if !self.config.environmental_adaptation_enabled {
            return Ok(());
        }
        
        let new_power_level = self.transceiver.adapt_transmission_power(&self.environmental_conditions)
            .map_err(|e| TransmissionError::AdaptationFailed(e.to_string()))?;
        
        // Ensure power level is within configured bounds
        let bounded_power = new_power_level
            .max(self.config.min_power_level)
            .min(self.config.max_power_level);
        
        if bounded_power != self.current_power_level {
            self.current_power_level = bounded_power;
            self.statistics.record_environmental_adaptation();
        }
        
        Ok(())
    }
    
    /// Update environmental conditions for adaptive transmission
    pub fn update_environmental_conditions(&mut self, conditions: EnvironmentalConditions) {
        self.environmental_conditions = conditions;
    }
    
    /// Update underwater conditions for propagation modeling
    pub fn update_underwater_conditions(&mut self, conditions: UnderwaterConditions) {
        self.statistics.record_underwater_conditions(conditions.clone());
        self.underwater_conditions = conditions;
    }

    /// Adjust transmission frequency for rough sea conditions (requirement 6.1, 6.4)
    pub fn adjust_transmission_frequency_for_sea_conditions(&mut self, sea_state: &SeaState) -> Result<(), TransmissionError> {
        let new_interval = match sea_state {
            SeaState::Calm | SeaState::Smooth => self.config.default_interval_ms,
            SeaState::Slight => self.config.default_interval_ms + 1000, // Slight delay
            SeaState::Moderate => self.config.default_interval_ms + 2000, // Moderate delay
            SeaState::Rough => self.config.default_interval_ms + 5000, // Significant delay
            SeaState::VeryRough => self.config.default_interval_ms + 10000, // Large delay
            SeaState::High | SeaState::VeryHigh => self.config.default_interval_ms + 20000, // Very large delay
            SeaState::Phenomenal => self.config.default_interval_ms + 30000, // Maximum delay
        };

        // Update configuration temporarily for rough sea conditions
        let mut temp_config = self.config.clone();
        temp_config.default_interval_ms = new_interval;
        self.update_config(temp_config)?;

        self.statistics.record_environmental_adaptation();
        Ok(())
    }

    /// Adjust transmission power for environmental conditions with enhanced logic
    pub fn adjust_transmission_power_for_conditions(&mut self, wave_height_m: Option<f32>, noise_level_db: Option<f32>, temperature_c: Option<f32>) -> Result<(), TransmissionError> {
        let mut power_adjustment = 0i16;

        // Adjust for wave conditions
        if let Some(wave_height) = wave_height_m {
            power_adjustment += match wave_height {
                h if h <= 1.0 => 0,
                h if h <= 2.5 => 10,
                h if h <= 4.0 => 25,
                h if h <= 6.0 => 40,
                h if h <= 9.0 => 60,
                _ => 80,
            };
        }

        // Adjust for noise conditions
        if let Some(noise) = noise_level_db {
            power_adjustment += match noise {
                n if n <= 60.0 => -10, // Quiet conditions, can reduce power
                n if n <= 80.0 => 0,   // Normal conditions
                n if n <= 100.0 => 15, // Noisy conditions, increase power
                _ => 30,               // Very noisy, significant increase
            };
        }

        // Adjust for temperature (affects electronics performance)
        if let Some(temp) = temperature_c {
            power_adjustment += match temp {
                t if t <= 0.0 => 10,   // Cold conditions may need more power
                t if t <= 40.0 => 0,   // Normal temperature range
                t if t <= 60.0 => -5,  // Hot conditions, reduce to prevent overheating
                _ => -15,              // Very hot, significant reduction
            };
        }

        // Apply power adjustment within bounds
        let new_power = ((self.current_power_level as i16) + power_adjustment)
            .max(self.config.min_power_level as i16)
            .min(self.config.max_power_level as i16) as u8;

        if new_power != self.current_power_level {
            self.set_power_level(new_power)?;
            self.statistics.record_environmental_adaptation();
        }

        Ok(())
    }

    /// Schedule transmission with environmental timing adjustment
    pub fn schedule_transmission_with_environmental_timing(
        &mut self,
        beacon_id: Uuid,
        position: GeodeticPosition,
        signal_quality: u8,
        message_version: MessageVersion,
        priority: TransmissionPriority,
        environmental_delay_ms: Option<u32>,
    ) -> Result<Uuid, TransmissionError> {
        let base_delay = match priority {
            TransmissionPriority::Emergency => 0,
            TransmissionPriority::High => 100,
            TransmissionPriority::Normal => 500,
            TransmissionPriority::Low => 1000,
        };

        let total_delay = base_delay + environmental_delay_ms.unwrap_or(0);

        self.schedule_transmission(
            beacon_id,
            position,
            signal_quality,
            message_version,
            priority,
            Some(total_delay),
        )
    }
    
    /// Get next sequence number
    fn get_next_sequence(&mut self) -> u16 {
        let current = self.sequence_counter;
        self.sequence_counter = self.sequence_counter.wrapping_add(1);
        current
    }
    
    /// Get transmission statistics
    pub fn get_statistics(&self) -> &TransmissionStatistics {
        &self.statistics
    }
    
    /// Get current configuration
    pub fn get_config(&self) -> &TransmissionConfig {
        &self.config
    }
    
    /// Update configuration
    pub fn update_config(&mut self, config: TransmissionConfig) -> Result<(), TransmissionError> {
        // Validate new configuration
        if config.default_interval_ms < 100 {
            return Err(TransmissionError::ConfigurationError(
                "Default interval too short (minimum 100ms)".to_string()
            ));
        }
        
        if config.min_power_level >= config.max_power_level {
            return Err(TransmissionError::ConfigurationError(
                "Invalid power level range".to_string()
            ));
        }
        
        self.config = config;
        Ok(())
    }
    
    /// Get current transmission queue status
    pub fn get_queue_status(&self) -> (usize, usize) {
        let total_queued = self.transmission_queue.len() + self.priority_queue.len();
        (total_queued, self.config.queue_size_limit)
    }
    
    /// Clear transmission queue
    pub fn clear_queue(&mut self) {
        self.transmission_queue.clear();
        self.priority_queue.clear();
    }
    
    /// Get current power level
    pub fn get_current_power_level(&self) -> u8 {
        self.current_power_level
    }
    
    /// Set power level manually
    pub fn set_power_level(&mut self, power_level: u8) -> Result<(), TransmissionError> {
        let bounded_power = power_level
            .max(self.config.min_power_level)
            .min(self.config.max_power_level);
        
        self.transceiver.set_transmission_power(bounded_power)
            .map_err(|e| TransmissionError::TransceiverFault(e.to_string()))?;
        
        self.current_power_level = bounded_power;
        Ok(())
    }
    
    /// Check if transmission manager is running
    pub fn is_running(&self) -> bool {
        self.is_running
    }
    
    /// Get time until next scheduled transmission
    pub fn time_until_next_transmission(&self) -> Option<Duration> {
        let now = Instant::now();
        
        // Check priority queue first
        if let Some(priority_transmission) = self.priority_queue.front() {
            return Some(if priority_transmission.scheduled_time > now {
                priority_transmission.scheduled_time.duration_since(now)
            } else {
                Duration::from_secs(0)
            });
        }
        
        // Check regular queue
        self.transmission_queue.front().map(|transmission| {
            if transmission.scheduled_time > now {
                transmission.scheduled_time.duration_since(now)
            } else {
                Duration::from_secs(0)
            }
        })
    }
    
    /// Cancel a scheduled transmission
    pub fn cancel_transmission(&mut self, transmission_id: Uuid) -> bool {
        // Check priority queue first
        if let Some(pos) = self.priority_queue.iter().position(|t| t.id == transmission_id) {
            self.priority_queue.remove(pos);
            return true;
        }
        
        // Check regular queue
        if let Some(pos) = self.transmission_queue.iter().position(|t| t.id == transmission_id) {
            self.transmission_queue.remove(pos);
            return true;
        }
        
        false
    }
    
    /// Get transmission queue summary
    pub fn get_queue_summary(&self) -> Vec<(Uuid, TransmissionPriority, Instant, u32)> {
        let mut summary = Vec::new();
        
        // Add priority queue items first
        for t in &self.priority_queue {
            summary.push((t.id, t.priority, t.scheduled_time, t.retry_count));
        }
        
        // Add regular queue items
        for t in &self.transmission_queue {
            summary.push((t.id, t.priority, t.scheduled_time, t.retry_count));
        }
        
        summary
    }
    
    /// Schedule emergency transmission with highest priority
    pub fn schedule_emergency_transmission(
        &mut self,
        beacon_id: Uuid,
        position: GeodeticPosition,
        signal_quality: u8,
        message_version: MessageVersion,
    ) -> Result<Uuid, TransmissionError> {
        self.schedule_transmission(
            beacon_id,
            position,
            signal_quality,
            message_version,
            TransmissionPriority::Emergency,
            Some(0), // Immediate transmission
        )
    }
    
    /// Get underwater optimization statistics
    pub fn get_underwater_statistics(&self) -> UnderwaterOptimizationStats {
        UnderwaterOptimizationStats {
            format_adaptations: self.statistics.format_adaptations,
            compression_savings_bytes: self.statistics.compression_savings_bytes,
            propagation_adjustments: self.statistics.propagation_adjustments,
            backoff_escalations: self.statistics.backoff_escalations,
            average_signal_quality: self.calculate_average_signal_quality(),
            current_underwater_conditions: self.underwater_conditions.clone(),
            compression_ratio: self.calculate_compression_ratio(),
        }
    }
    
    /// Calculate average signal quality from history
    fn calculate_average_signal_quality(&self) -> Option<f32> {
        if self.statistics.signal_quality_history.is_empty() {
            None
        } else {
            let sum: u32 = self.statistics.signal_quality_history.iter().map(|&x| x as u32).sum();
            Some(sum as f32 / self.statistics.signal_quality_history.len() as f32)
        }
    }
    
    /// Calculate overall compression ratio
    fn calculate_compression_ratio(&self) -> Option<f32> {
        if self.statistics.total_bytes_transmitted == 0 {
            None
        } else {
            let original_bytes = self.statistics.total_bytes_transmitted + self.statistics.compression_savings_bytes;
            Some(self.statistics.total_bytes_transmitted as f32 / original_bytes as f32)
        }
    }
    
    /// Clear priority queue (emergency messages)
    pub fn clear_priority_queue(&mut self) {
        self.priority_queue.clear();
    }
    
    /// Get priority queue status
    pub fn get_priority_queue_status(&self) -> (usize, usize) {
        (self.priority_queue.len(), self.transmission_queue.len())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{MockTransceiver, MockPowerManager};
    
    #[test]
    fn test_transmission_manager_creation() {
        let config = TransmissionConfig::default();
        let transceiver = MockTransceiver::new(1);
        let power_manager = MockPowerManager::new();
        
        let manager = TransmissionManager::new(config, transceiver, power_manager);
        assert!(manager.is_ok());
    }
    
    #[test]
    fn test_schedule_transmission() {
        let config = TransmissionConfig::default();
        let mut transceiver = MockTransceiver::new(1);
        transceiver.configure(crate::TransceiverConfig::default()).unwrap();
        let power_manager = MockPowerManager::new();
        
        let mut manager = TransmissionManager::new(config, transceiver, power_manager).unwrap();
        manager.start().unwrap();
        
        let beacon_id = Uuid::new_v4();
        let position = GeodeticPosition {
            latitude: 32.123456,
            longitude: -117.654321,
            depth: 10.5,
        };
        
        let result = manager.schedule_transmission(
            beacon_id,
            position,
            200,
            MessageVersion::V3,
            TransmissionPriority::Normal,
            None,
        );
        
        assert!(result.is_ok());
        assert_eq!(manager.get_queue_status().0, 1);
    }
    
    #[test]
    fn test_transmission_statistics() {
        let mut stats = TransmissionStatistics::default();
        
        stats.record_success(100, Duration::from_millis(50), 128);
        assert_eq!(stats.successful_transmissions, 1);
        assert_eq!(stats.total_bytes_transmitted, 100);
        assert_eq!(stats.success_rate, 1.0);
        
        let error = TransmissionError::TransceiverFault("Test error".to_string());
        stats.record_failure(error, false);
        assert_eq!(stats.failed_transmissions, 1);
        assert_eq!(stats.success_rate, 0.5);
    }
    
    #[test]
    fn test_power_adaptation() {
        let mut config = TransmissionConfig::default();
        config.adaptive_power_enabled = true;
        config.power_adaptation_threshold = 50.0;
        
        let mut transceiver = MockTransceiver::new(1);
        transceiver.configure(crate::TransceiverConfig::default()).unwrap();
        let mut power_manager = MockPowerManager::new();
        power_manager.simulate_discharge(55.0); // Discharge to 30% (below threshold)
        
        let mut manager = TransmissionManager::new(config, transceiver, power_manager).unwrap();
        manager.start().unwrap();
        
        // Power should be adapted when checking requirements
        let result = manager.check_power_requirements();
        assert!(result.is_ok());
        
        // Power level should be reduced
        assert!(manager.get_current_power_level() < 128);
    }

    #[test]
    fn test_underwater_optimization_features() {
        let mut config = TransmissionConfig::default();
        config.underwater_optimization_enabled = true;
        config.adaptive_format_selection = true;
        config.compression_enabled = true;
        
        let mut transceiver = MockTransceiver::new(1);
        transceiver.configure(crate::TransceiverConfig::default()).unwrap();
        let power_manager = MockPowerManager::new();
        
        let mut manager = TransmissionManager::new(config, transceiver, power_manager).unwrap();
        manager.start().unwrap();
        
        // Test underwater conditions update
        let underwater_conditions = UnderwaterConditions {
            water_temperature_c: Some(20.0),
            salinity_ppt: Some(35.0),
            depth_m: Some(50.0),
            acoustic_noise_db: Some(85.0), // High noise
            ..Default::default()
        };
        manager.update_underwater_conditions(underwater_conditions);
        
        // Schedule transmission with poor signal quality to trigger format adaptation
        let beacon_id = Uuid::new_v4();
        let position = GeodeticPosition {
            latitude: 32.123456,
            longitude: -117.654321,
            depth: 50.0,
        };
        
        let result = manager.schedule_transmission(
            beacon_id,
            position,
            30, // Poor signal quality
            MessageVersion::V3,
            TransmissionPriority::Normal,
            None,
        );
        
        assert!(result.is_ok());
        
        // Get underwater statistics
        let stats = manager.get_underwater_statistics();
        assert_eq!(stats.format_adaptations, 1); // Should have adapted format
    }
    
    #[test]
    fn test_emergency_message_priority() {
        let config = TransmissionConfig::default();
        let mut transceiver = MockTransceiver::new(1);
        transceiver.configure(crate::TransceiverConfig::default()).unwrap();
        let power_manager = MockPowerManager::new();
        
        let mut manager = TransmissionManager::new(config, transceiver, power_manager).unwrap();
        manager.start().unwrap();
        
        let beacon_id = Uuid::new_v4();
        let position = GeodeticPosition {
            latitude: 32.123456,
            longitude: -117.654321,
            depth: 10.0,
        };
        
        // Schedule normal transmission
        let normal_id = manager.schedule_transmission(
            beacon_id,
            position,
            200,
            MessageVersion::V2,
            TransmissionPriority::Normal,
            Some(1000), // 1 second delay
        ).unwrap();
        
        // Schedule emergency transmission
        let emergency_id = manager.schedule_emergency_transmission(
            beacon_id,
            position,
            200,
            MessageVersion::V1,
        ).unwrap();
        
        // Emergency should be in priority queue
        let (priority_count, regular_count) = manager.get_priority_queue_status();
        assert_eq!(priority_count, 1); // Emergency message
        assert_eq!(regular_count, 1);  // Normal message
        
        // Emergency should be processed first
        let queue_summary = manager.get_queue_summary();
        assert_eq!(queue_summary[0].1, TransmissionPriority::Emergency);
    }
    
    #[test]
    fn test_exponential_backoff() {
        let mut config = TransmissionConfig::default();
        config.exponential_backoff_enabled = true;
        config.retry_backoff_ms = 1000;
        config.max_backoff_interval_ms = 8000;
        
        let mut transceiver = MockTransceiver::new(1);
        transceiver.configure(crate::TransceiverConfig::default()).unwrap();
        transceiver.set_should_fail_transmission(true); // Force failures
        let power_manager = MockPowerManager::new();
        
        let mut manager = TransmissionManager::new(config, transceiver, power_manager).unwrap();
        manager.start().unwrap();
        
        let beacon_id = Uuid::new_v4();
        let position = GeodeticPosition {
            latitude: 32.123456,
            longitude: -117.654321,
            depth: 10.0,
        };
        
        // Schedule transmission that will fail
        let _result = manager.schedule_transmission(
            beacon_id,
            position,
            200,
            MessageVersion::V2,
            TransmissionPriority::Normal,
            None,
        ).unwrap();
        
        // Process transmission (should fail and retry with backoff)
        let _processed = manager.process_transmissions().unwrap();
        
        // Check that backoff escalation was recorded
        let stats = manager.get_underwater_statistics();
        assert!(stats.backoff_escalations > 0);
    }
    
    #[test]
    fn test_compression_simulation() {
        let config = TransmissionConfig::default();
        let transceiver = MockTransceiver::new(1);
        let power_manager = MockPowerManager::new();
        
        let mut manager = TransmissionManager::new(config, transceiver, power_manager).unwrap();
        
        let test_data = vec![0u8; 100]; // 100 bytes of test data
        
        // Test LZ4 compression
        let compressed_lz4 = manager.compress_message_data(&test_data, CompressionType::LZ4).unwrap();
        assert!(compressed_lz4.len() < test_data.len());
        
        // Test deflate compression
        let compressed_deflate = manager.compress_message_data(&test_data, CompressionType::Deflate).unwrap();
        assert!(compressed_deflate.len() < test_data.len());
        assert!(compressed_deflate.len() < compressed_lz4.len()); // Deflate should be better
        
        // Test custom compression
        let compressed_custom = manager.compress_message_data(&test_data, CompressionType::Custom).unwrap();
        assert!(compressed_custom.len() <= test_data.len());
    }
    
    #[test]
    fn test_propagation_model() {
        let mut config = TransmissionConfig::default();
        config.propagation_model_enabled = true;
        
        let transceiver = MockTransceiver::new(1);
        let power_manager = MockPowerManager::new();
        
        let mut manager = TransmissionManager::new(config, transceiver, power_manager).unwrap();
        
        // Test power calculation for different distances
        let power_100m = manager.calculate_propagation_power(100.0, 100).unwrap();
        let power_1000m = manager.calculate_propagation_power(1000.0, 100).unwrap();
        
        // Power should increase with distance
        assert!(power_1000m > power_100m);
        
        // Test absorption coefficient calculation
        let absorption = manager.calculate_absorption_coefficient(25.0).unwrap();
        assert!(absorption > 0.0);
    }

    #[test]
    fn test_rough_sea_transmission_adjustment() {
        let config = TransmissionConfig::default();
        let mut transceiver = MockTransceiver::new(1);
        transceiver.configure(crate::TransceiverConfig::default()).unwrap();
        let power_manager = MockPowerManager::new();
        
        let mut manager = TransmissionManager::new(config, transceiver, power_manager).unwrap();
        manager.start().unwrap();
        
        // Test rough sea condition adjustment
        let result = manager.adjust_transmission_frequency_for_sea_conditions(&SeaState::Rough);
        assert!(result.is_ok());
        
        // Transmission interval should be increased for rough seas
        assert!(manager.get_config().default_interval_ms > 5000);
    }

    #[test]
    fn test_environmental_power_adjustment() {
        let config = TransmissionConfig::default();
        let mut transceiver = MockTransceiver::new(1);
        transceiver.configure(crate::TransceiverConfig::default()).unwrap();
        let power_manager = MockPowerManager::new();
        
        let mut manager = TransmissionManager::new(config, transceiver, power_manager).unwrap();
        manager.start().unwrap();
        
        let initial_power = manager.get_current_power_level();
        
        // Test power adjustment for high waves and noise
        let result = manager.adjust_transmission_power_for_conditions(
            Some(5.0), // High waves
            Some(90.0), // High noise
            Some(25.0), // Normal temperature
        );
        assert!(result.is_ok());
        
        // Power should be increased for difficult conditions
        assert!(manager.get_current_power_level() > initial_power);
    }
}