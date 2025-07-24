// Communication manager for long-range connectivity
// Handles connection management, status reporting, and configuration updates

use std::time::{Duration, SystemTime, Instant};
use std::collections::VecDeque;
use uuid::Uuid;
use serde::{Serialize, Deserialize};
use rand;


use crate::transceiver_interface::CommError;
use crate::gps_manager::GpsPosition;
use crate::power_manager::BatteryStatus;

/// Communication manager trait for long-range connectivity
pub trait CommunicationManager {
    /// Establish connection to remote services
    fn connect(&mut self) -> Result<(), CommError>;
    
    /// Disconnect from remote services
    fn disconnect(&mut self) -> Result<(), CommError>;
    
    /// Send status report to remote services
    fn send_status_report(&mut self, report: StatusReport) -> Result<(), CommError>;
    
    /// Check for and download configuration updates
    fn check_for_updates(&mut self) -> Result<Option<ConfigUpdate>, CommError>;
    
    /// Check if currently connected
    fn is_connected(&self) -> bool;
    
    /// Get current signal strength (0-100, None if not available)
    fn get_signal_strength(&self) -> Option<u8>;
    
    /// Get connection statistics
    fn get_connection_stats(&self) -> ConnectionStats;
    
    /// Configure communication parameters
    fn configure(&mut self, config: CommunicationConfig) -> Result<(), CommError>;
}

/// Configuration for communication manager
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CommunicationConfig {
    /// Connection interval in hours (1-24)
    pub connection_interval_hours: u32,
    /// Number of retry attempts before giving up
    pub retry_attempts: u32,
    /// Initial retry backoff in milliseconds
    pub retry_backoff_ms: u32,
    /// Maximum retry interval in hours
    pub max_retry_interval_hours: u32,
    /// Connection timeout in seconds
    pub connection_timeout_s: u32,
    /// Enable data compression for transmission
    pub data_compression_enabled: bool,
    /// Server endpoint URL
    pub server_endpoint: String,
    /// Authentication token or key
    pub auth_token: String,
}

impl Default for CommunicationConfig {
    fn default() -> Self {
        Self {
            connection_interval_hours: 6,
            retry_attempts: 3,
            retry_backoff_ms: 1000,
            max_retry_interval_hours: 6,
            connection_timeout_s: 30,
            data_compression_enabled: true,
            server_endpoint: "https://beacon-api.example.com".to_string(),
            auth_token: String::new(),
        }
    }
}

/// Status report sent to remote services
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct StatusReport {
    /// Unique beacon identifier
    pub beacon_id: Uuid,
    /// Report generation timestamp
    pub timestamp: SystemTime,
    /// Recent position history
    pub position_history: Vec<GpsPosition>,
    /// Current battery status
    pub battery_status: BatteryStatus,
    /// System health metrics
    pub system_health: SystemHealth,
    /// Transmission statistics
    pub transmission_stats: TransmissionStats,
    /// Uptime since last restart
    pub uptime: Duration,
    /// Recent error log entries
    pub recent_errors: Vec<ErrorLogEntry>,
}

/// System health metrics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SystemHealth {
    /// CPU usage percentage (0-100)
    pub cpu_usage_percent: f32,
    /// Memory usage percentage (0-100)
    pub memory_usage_percent: f32,
    /// Internal temperature in Celsius
    pub temperature_c: f32,
    /// GPS signal quality (0-100)
    pub gps_signal_quality: u8,
    /// Communication signal quality (0-100)
    pub comm_signal_quality: u8,
    /// Number of system restarts
    pub restart_count: u32,
    /// Last restart reason
    pub last_restart_reason: String,
}

/// Transmission statistics
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TransmissionStats {
    /// Total messages sent
    pub messages_sent: u64,
    /// Number of transmission failures
    pub transmission_failures: u32,
    /// Last successful transmission time
    pub last_transmission_time: Option<SystemTime>,
    /// Average transmission interval in milliseconds
    pub average_transmission_interval_ms: u32,
    /// Recent signal quality measurements
    pub signal_quality_history: Vec<u8>,
    /// Transmission power levels used
    pub power_level_history: Vec<u8>,
}

/// Error log entry for reporting
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ErrorLogEntry {
    /// Error occurrence timestamp
    pub timestamp: SystemTime,
    /// Error severity level
    pub severity: ErrorSeverity,
    /// Error message
    pub message: String,
    /// Error context information
    pub context: String,
    /// Recovery action taken
    pub recovery_action: Option<String>,
}

/// Error severity levels
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ErrorSeverity {
    Info,
    Warning,
    Error,
    Critical,
}

/// Configuration update from remote services
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConfigUpdate {
    /// Update identifier
    pub update_id: String,
    /// Update version
    pub version: String,
    /// Update timestamp
    pub timestamp: SystemTime,
    /// Configuration data
    pub config_data: Vec<u8>,
    /// Update checksum for validation
    pub checksum: String,
    /// Update description
    pub description: String,
    /// Whether update requires restart
    pub requires_restart: bool,
}

/// Connection statistics
#[derive(Debug, Clone)]
pub struct ConnectionStats {
    /// Total connection attempts
    pub connection_attempts: u32,
    /// Successful connections
    pub successful_connections: u32,
    /// Failed connections
    pub failed_connections: u32,
    /// Last connection time
    pub last_connection_time: Option<SystemTime>,
    /// Last connection duration
    pub last_connection_duration: Option<Duration>,
    /// Average connection duration
    pub average_connection_duration: Duration,
    /// Total data sent (bytes)
    pub total_data_sent: u64,
    /// Total data received (bytes)
    pub total_data_received: u64,
    /// Current retry count
    pub current_retry_count: u32,
    /// Next retry time
    pub next_retry_time: Option<SystemTime>,
}

impl Default for ConnectionStats {
    fn default() -> Self {
        Self {
            connection_attempts: 0,
            successful_connections: 0,
            failed_connections: 0,
            last_connection_time: None,
            last_connection_duration: None,
            average_connection_duration: Duration::from_secs(0),
            total_data_sent: 0,
            total_data_received: 0,
            current_retry_count: 0,
            next_retry_time: None,
        }
    }
}

/// Communication manager with retry logic and exponential backoff
pub struct BasicCommunicationManager {
    config: CommunicationConfig,
    connected: bool,
    stats: ConnectionStats,
    last_connection_attempt: Option<Instant>,
    retry_backoff: Duration,
    connection_history: VecDeque<ConnectionAttempt>,
}

/// Record of a connection attempt
#[derive(Debug, Clone)]
struct ConnectionAttempt {
    timestamp: Instant,
    success: bool,
    duration: Option<Duration>,
    error: Option<String>,
}

impl BasicCommunicationManager {
    /// Create new communication manager with configuration
    pub fn new(config: CommunicationConfig) -> Self {
        Self {
            config,
            connected: false,
            stats: ConnectionStats::default(),
            last_connection_attempt: None,
            retry_backoff: Duration::from_millis(1000),
            connection_history: VecDeque::with_capacity(100),
        }
    }
    
    /// Calculate next retry delay using exponential backoff
    fn calculate_retry_delay(&self) -> Duration {
        let base_delay = Duration::from_millis(self.config.retry_backoff_ms as u64);
        let max_delay = Duration::from_secs(self.config.max_retry_interval_hours as u64 * 3600);
        
        let exponential_delay = base_delay * (2_u32.pow(self.stats.current_retry_count.min(10)));
        exponential_delay.min(max_delay)
    }
    
    /// Record connection attempt result
    fn record_connection_attempt(&mut self, success: bool, duration: Option<Duration>, error: Option<String>) {
        let attempt = ConnectionAttempt {
            timestamp: Instant::now(),
            success,
            duration,
            error,
        };
        
        self.connection_history.push_back(attempt);
        if self.connection_history.len() > 100 {
            self.connection_history.pop_front();
        }
        
        self.stats.connection_attempts += 1;
        if success {
            self.stats.successful_connections += 1;
            self.stats.current_retry_count = 0;
            self.retry_backoff = Duration::from_millis(self.config.retry_backoff_ms as u64);
        } else {
            self.stats.failed_connections += 1;
            self.stats.current_retry_count += 1;
            self.retry_backoff = self.calculate_retry_delay();
            self.stats.next_retry_time = Some(SystemTime::now() + self.retry_backoff);
        }
        
        if let Some(duration) = duration {
            self.stats.last_connection_duration = Some(duration);
            
            // Update average connection duration
            let total_duration = self.stats.average_connection_duration * self.stats.successful_connections.saturating_sub(1)
                + duration;
            self.stats.average_connection_duration = total_duration / self.stats.successful_connections.max(1);
        }
    }
    
    /// Check if enough time has passed for retry
    fn can_retry(&self) -> bool {
        if let Some(next_retry) = self.stats.next_retry_time {
            SystemTime::now() >= next_retry
        } else {
            true
        }
    }
    
    /// Simulate network connection (placeholder for actual implementation)
    fn establish_network_connection(&mut self) -> Result<(), CommError> {
        // This would contain actual network connection logic
        // For now, simulate connection based on configuration
        
        if self.config.server_endpoint.is_empty() {
            return Err(CommError::ConfigurationError("Server endpoint not configured".to_string()));
        }
        
        if self.config.auth_token.is_empty() {
            return Err(CommError::AuthenticationFailed);
        }
        
        // Simulate connection timeout
        std::thread::sleep(Duration::from_millis(100));
        
        // Simulate occasional connection failures for testing
        if self.stats.connection_attempts % 5 == 4 {
            return Err(CommError::ConnectionFailed("Simulated network error".to_string()));
        }
        
        Ok(())
    }
    
    /// Simulate data transmission (placeholder for actual implementation)
    fn transmit_data(&mut self, data: &[u8]) -> Result<(), CommError> {
        if !self.connected {
            return Err(CommError::NotConnected);
        }
        
        // Simulate data compression if enabled
        let compressed_size = if self.config.data_compression_enabled {
            data.len() * 7 / 10  // Simulate 30% compression
        } else {
            data.len()
        };
        
        self.stats.total_data_sent += compressed_size as u64;
        
        // Simulate transmission delay
        std::thread::sleep(Duration::from_millis(10));
        
        Ok(())
    }
    
    /// Simulate data reception (placeholder for actual implementation)
    fn receive_data(&mut self) -> Result<Vec<u8>, CommError> {
        if !self.connected {
            return Err(CommError::NotConnected);
        }
        
        // Simulate receiving configuration update data
        let data = b"mock_config_update_data".to_vec();
        self.stats.total_data_received += data.len() as u64;
        
        Ok(data)
    }
}

impl CommunicationManager for BasicCommunicationManager {
    fn connect(&mut self) -> Result<(), CommError> {
        let start_time = Instant::now();
        
        // Check if we can retry (respects exponential backoff)
        if !self.can_retry() {
            return Err(CommError::RetryLimitExceeded);
        }
        
        // Check retry limit
        if self.stats.current_retry_count >= self.config.retry_attempts {
            return Err(CommError::RetryLimitExceeded);
        }
        
        self.last_connection_attempt = Some(start_time);
        
        // Attempt to establish connection
        match self.establish_network_connection() {
            Ok(()) => {
                self.connected = true;
                let duration = start_time.elapsed();
                self.stats.last_connection_time = Some(SystemTime::now());
                self.record_connection_attempt(true, Some(duration), None);
                Ok(())
            }
            Err(e) => {
                self.connected = false;
                let duration = start_time.elapsed();
                self.record_connection_attempt(false, Some(duration), Some(e.to_string()));
                Err(e)
            }
        }
    }
    
    fn disconnect(&mut self) -> Result<(), CommError> {
        if !self.connected {
            return Ok(());
        }
        
        self.connected = false;
        Ok(())
    }
    
    fn send_status_report(&mut self, report: StatusReport) -> Result<(), CommError> {
        if !self.connected {
            // Try to connect first
            self.connect()?;
        }
        
        // Serialize status report
        let serialized = serde_json::to_vec(&report)
            .map_err(|e| CommError::SerializationError(e.to_string()))?;
        
        // Transmit data
        self.transmit_data(&serialized)?;
        
        Ok(())
    }
    
    fn check_for_updates(&mut self) -> Result<Option<ConfigUpdate>, CommError> {
        if !self.connected {
            // Try to connect first
            self.connect()?;
        }
        
        // Request updates from server (simulated)
        let response_data = self.receive_data()?;
        
        // Check if there's actually an update
        if response_data.is_empty() || response_data == b"no_updates" {
            return Ok(None);
        }
        
        // Parse configuration update
        let update = ConfigUpdate {
            update_id: "update_001".to_string(),
            version: "1.2.0".to_string(),
            timestamp: SystemTime::now(),
            config_data: response_data,
            checksum: "mock_checksum".to_string(),
            description: "Mock configuration update".to_string(),
            requires_restart: false,
        };
        
        Ok(Some(update))
    }
    
    fn is_connected(&self) -> bool {
        self.connected
    }
    
    fn get_signal_strength(&self) -> Option<u8> {
        if self.connected {
            // Simulate signal strength based on connection history
            let success_rate = if self.stats.connection_attempts > 0 {
                (self.stats.successful_connections * 100) / self.stats.connection_attempts
            } else {
                100
            };
            Some(success_rate as u8)
        } else {
            None
        }
    }
    
    fn get_connection_stats(&self) -> ConnectionStats {
        self.stats.clone()
    }
    
    fn configure(&mut self, config: CommunicationConfig) -> Result<(), CommError> {
        // Validate configuration
        if config.connection_interval_hours == 0 || config.connection_interval_hours > 24 {
            return Err(CommError::ConfigurationError(
                "Connection interval must be between 1 and 24 hours".to_string()
            ));
        }
        
        if config.retry_attempts == 0 {
            return Err(CommError::ConfigurationError(
                "Retry attempts must be greater than 0".to_string()
            ));
        }
        
        if config.connection_timeout_s == 0 {
            return Err(CommError::ConfigurationError(
                "Connection timeout must be greater than 0".to_string()
            ));
        }
        
        if config.server_endpoint.is_empty() {
            return Err(CommError::ConfigurationError(
                "Server endpoint cannot be empty".to_string()
            ));
        }
        
        // Disconnect if configuration changes require it
        if self.connected && (
            config.server_endpoint != self.config.server_endpoint ||
            config.auth_token != self.config.auth_token
        ) {
            self.disconnect()?;
        }
        
        self.config = config;
        Ok(())
    }
}

/// Mock communication manager for testing
pub struct MockCommunicationManager {
    connected: bool,
    config: CommunicationConfig,
    stats: ConnectionStats,
    connection_success_rate: f64,
    simulated_updates: VecDeque<ConfigUpdate>,
    transmission_delay: Duration,
    force_connection_failure: bool,
    signal_strength: Option<u8>,
}

impl MockCommunicationManager {
    /// Create new mock communication manager
    pub fn new() -> Self {
        Self {
            connected: false,
            config: CommunicationConfig::default(),
            stats: ConnectionStats::default(),
            connection_success_rate: 0.9, // 90% success rate by default
            simulated_updates: VecDeque::new(),
            transmission_delay: Duration::from_millis(10),
            force_connection_failure: false,
            signal_strength: Some(85),
        }
    }
    
    /// Set connection success rate for testing (0.0 to 1.0)
    pub fn set_connection_success_rate(&mut self, rate: f64) {
        self.connection_success_rate = rate.clamp(0.0, 1.0);
    }
    
    /// Add simulated configuration update
    pub fn add_simulated_update(&mut self, update: ConfigUpdate) {
        self.simulated_updates.push_back(update);
    }
    
    /// Set transmission delay for testing
    pub fn set_transmission_delay(&mut self, delay: Duration) {
        self.transmission_delay = delay;
    }
    
    /// Force connection failures for testing
    pub fn force_connection_failure(&mut self, force: bool) {
        self.force_connection_failure = force;
    }
    
    /// Set simulated signal strength
    pub fn set_signal_strength(&mut self, strength: Option<u8>) {
        self.signal_strength = strength;
    }
    
    /// Get number of status reports sent
    pub fn get_reports_sent(&self) -> u32 {
        (self.stats.total_data_sent / 1000) as u32 // Approximate based on data sent
    }
    
    /// Get number of updates checked
    pub fn get_updates_checked(&self) -> u32 {
        self.stats.total_data_received as u32
    }
}

impl Default for MockCommunicationManager {
    fn default() -> Self {
        Self::new()
    }
}

impl CommunicationManager for MockCommunicationManager {
    fn connect(&mut self) -> Result<(), CommError> {
        self.stats.connection_attempts += 1;
        
        if self.force_connection_failure {
            self.stats.failed_connections += 1;
            return Err(CommError::ConnectionFailed("Forced failure for testing".to_string()));
        }
        
        // Simulate connection success/failure based on success rate
        let success = rand::random::<f64>() < self.connection_success_rate;
        
        if success {
            self.connected = true;
            self.stats.successful_connections += 1;
            self.stats.last_connection_time = Some(SystemTime::now());
            Ok(())
        } else {
            self.connected = false;
            self.stats.failed_connections += 1;
            Err(CommError::ConnectionFailed("Simulated connection failure".to_string()))
        }
    }
    
    fn disconnect(&mut self) -> Result<(), CommError> {
        self.connected = false;
        Ok(())
    }
    
    fn send_status_report(&mut self, _report: StatusReport) -> Result<(), CommError> {
        if !self.connected {
            return Err(CommError::NotConnected);
        }
        
        // Simulate transmission delay
        std::thread::sleep(self.transmission_delay);
        
        // Simulate data transmission
        self.stats.total_data_sent += 1000; // Approximate report size
        
        Ok(())
    }
    
    fn check_for_updates(&mut self) -> Result<Option<ConfigUpdate>, CommError> {
        if !self.connected {
            return Err(CommError::NotConnected);
        }
        
        // Simulate data reception
        self.stats.total_data_received += 1;
        
        // Return simulated update if available
        Ok(self.simulated_updates.pop_front())
    }
    
    fn is_connected(&self) -> bool {
        self.connected
    }
    
    fn get_signal_strength(&self) -> Option<u8> {
        if self.connected {
            self.signal_strength
        } else {
            None
        }
    }
    
    fn get_connection_stats(&self) -> ConnectionStats {
        self.stats.clone()
    }
    
    fn configure(&mut self, config: CommunicationConfig) -> Result<(), CommError> {
        self.config = config;
        Ok(())
    }
}



#[cfg(test)]
mod tests {
    use super::*;
    use std::time::SystemTime;
    
    #[test]
    fn test_communication_config_default() {
        let config = CommunicationConfig::default();
        assert_eq!(config.connection_interval_hours, 6);
        assert_eq!(config.retry_attempts, 3);
        assert!(config.data_compression_enabled);
    }
    
    #[test]
    fn test_basic_communication_manager_creation() {
        let config = CommunicationConfig::default();
        let manager = BasicCommunicationManager::new(config);
        assert!(!manager.is_connected());
        assert_eq!(manager.get_connection_stats().connection_attempts, 0);
    }
    
    #[test]
    fn test_mock_communication_manager() {
        let mut manager = MockCommunicationManager::new();
        assert!(!manager.is_connected());
        
        // Test connection
        manager.set_connection_success_rate(1.0);
        assert!(manager.connect().is_ok());
        assert!(manager.is_connected());
        
        // Test status report
        let report = create_test_status_report();
        assert!(manager.send_status_report(report).is_ok());
        
        // Test disconnect
        assert!(manager.disconnect().is_ok());
        assert!(!manager.is_connected());
    }
    
    #[test]
    fn test_mock_connection_failure() {
        let mut manager = MockCommunicationManager::new();
        manager.set_connection_success_rate(0.0); // Force failure
        
        assert!(manager.connect().is_err());
        assert!(!manager.is_connected());
    }
    
    #[test]
    fn test_configuration_validation() {
        let mut manager = BasicCommunicationManager::new(CommunicationConfig::default());
        
        // Test invalid connection interval
        let mut invalid_config = CommunicationConfig::default();
        invalid_config.connection_interval_hours = 0;
        assert!(manager.configure(invalid_config).is_err());
        
        // Test invalid retry attempts
        let mut invalid_config = CommunicationConfig::default();
        invalid_config.retry_attempts = 0;
        assert!(manager.configure(invalid_config).is_err());
        
        // Test valid configuration
        let valid_config = CommunicationConfig::default();
        assert!(manager.configure(valid_config).is_ok());
    }
    
    #[test]
    fn test_exponential_backoff() {
        let config = CommunicationConfig {
            retry_backoff_ms: 1000,
            max_retry_interval_hours: 1,
            ..Default::default()
        };
        let mut manager = BasicCommunicationManager::new(config);
        
        // Simulate multiple failures to test backoff
        for i in 0..5 {
            manager.stats.current_retry_count = i;
            let delay = manager.calculate_retry_delay();
            
            if i == 0 {
                assert_eq!(delay, Duration::from_millis(1000));
            } else {
                assert!(delay >= Duration::from_millis(1000));
                assert!(delay <= Duration::from_secs(3600)); // Max 1 hour
            }
        }
    }
    
    #[test]
    fn test_simulated_updates() {
        let mut manager = MockCommunicationManager::new();
        manager.set_connection_success_rate(1.0);
        assert!(manager.connect().is_ok());
        
        // Add simulated update
        let update = ConfigUpdate {
            update_id: "test_001".to_string(),
            version: "1.0.0".to_string(),
            timestamp: SystemTime::now(),
            config_data: b"test_config".to_vec(),
            checksum: "test_checksum".to_string(),
            description: "Test update".to_string(),
            requires_restart: false,
        };
        manager.add_simulated_update(update.clone());
        
        // Check for updates
        let received_update = manager.check_for_updates().unwrap();
        assert!(received_update.is_some());
        assert_eq!(received_update.unwrap().update_id, "test_001");
        
        // No more updates
        let no_update = manager.check_for_updates().unwrap();
        assert!(no_update.is_none());
    }
    
    fn create_test_status_report() -> StatusReport {
        StatusReport {
            beacon_id: Uuid::new_v4(),
            timestamp: SystemTime::now(),
            position_history: vec![],
            battery_status: crate::power_manager::BatteryStatus {
                voltage_v: 12.0,
                current_ma: 100.0,
                capacity_percent: 75.0,
                temperature_c: 25.0,
                health: crate::power_manager::BatteryHealth::Good,
                timestamp: SystemTime::now(),
                cycles: 100,
                time_to_empty: Some(Duration::from_secs(3600)),
                time_to_full: None,
            },
            system_health: SystemHealth {
                cpu_usage_percent: 25.0,
                memory_usage_percent: 60.0,
                temperature_c: 30.0,
                gps_signal_quality: 85,
                comm_signal_quality: 90,
                restart_count: 0,
                last_restart_reason: "Normal startup".to_string(),
            },
            transmission_stats: TransmissionStats {
                messages_sent: 1000,
                transmission_failures: 5,
                last_transmission_time: Some(SystemTime::now()),
                average_transmission_interval_ms: 5000,
                signal_quality_history: vec![85, 87, 90, 88, 92],
                power_level_history: vec![75, 75, 80, 75, 75],
            },
            uptime: Duration::from_secs(3600),
            recent_errors: vec![],
        }
    }
}