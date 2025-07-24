use std::time::{Duration, SystemTime, Instant};
use serde::{Deserialize, Serialize};

/// GPS position data with accuracy and timing information
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GpsPosition {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    pub timestamp: SystemTime,
    pub accuracy_m: f32,
    pub satellite_count: u8,
}

/// GPS configuration parameters
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GpsConfig {
    pub acquisition_timeout_s: u32,
    pub update_interval_s: u32,
    pub min_satellite_count: u8,
    pub accuracy_threshold_m: f32,
    pub cold_start_timeout_s: u32,
}

impl Default for GpsConfig {
    fn default() -> Self {
        Self {
            acquisition_timeout_s: 60,
            update_interval_s: 5,
            min_satellite_count: 4,
            accuracy_threshold_m: 10.0,
            cold_start_timeout_s: 120,
        }
    }
}

/// GPS-specific error types
#[derive(Debug, Clone, PartialEq)]
pub enum GpsError {
    AcquisitionTimeout,
    SignalLost,
    AccuracyTooLow { current: f32, required: f32 },
    HardwareFault,
    ConfigurationInvalid,
}

impl std::fmt::Display for GpsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            GpsError::AcquisitionTimeout => write!(f, "GPS acquisition timeout"),
            GpsError::SignalLost => write!(f, "GPS signal lost"),
            GpsError::AccuracyTooLow { current, required } => {
                write!(f, "GPS accuracy too low: {}m (required: {}m)", current, required)
            }
            GpsError::HardwareFault => write!(f, "GPS hardware fault"),
            GpsError::ConfigurationInvalid => write!(f, "GPS configuration invalid"),
        }
    }
}

impl std::error::Error for GpsError {}

/// GPS status information
#[derive(Debug, Clone, PartialEq)]
pub enum GpsStatus {
    Initializing,
    Acquiring,
    Locked,
    SignalLost,
    DegradedMode,
    HardwareFault,
}

/// Main GPS manager trait for position acquisition and monitoring
pub trait GpsManager {
    /// Start GPS acquisition process
    fn start_acquisition(&mut self) -> Result<(), GpsError>;
    
    /// Get current GPS position if available
    fn get_current_position(&self) -> Option<GpsPosition>;
    
    /// Get current position accuracy in meters
    fn get_position_accuracy(&self) -> Option<f32>;
    
    /// Check if GPS has a valid lock
    fn is_locked(&self) -> bool;
    
    /// Get current satellite count
    fn get_satellite_count(&self) -> u8;
    
    /// Configure GPS parameters
    fn configure(&mut self, config: GpsConfig) -> Result<(), GpsError>;
    
    /// Get current GPS status
    fn get_status(&self) -> GpsStatus;
    
    /// Update GPS state (should be called periodically)
    fn update(&mut self) -> Result<(), GpsError>;
    
    /// Stop GPS acquisition
    fn stop(&mut self) -> Result<(), GpsError>;
}

/// Basic GPS manager implementation
pub struct BasicGpsManager {
    config: GpsConfig,
    current_position: Option<GpsPosition>,
    status: GpsStatus,
    acquisition_start: Option<Instant>,
    last_update: Option<Instant>,
    signal_lost_time: Option<Instant>,
    satellite_count: u8,
}

impl BasicGpsManager {
    pub fn new(config: GpsConfig) -> Result<Self, GpsError> {
        // Validate configuration
        if config.acquisition_timeout_s == 0 || config.update_interval_s == 0 {
            return Err(GpsError::ConfigurationInvalid);
        }
        
        if config.min_satellite_count < 3 {
            return Err(GpsError::ConfigurationInvalid);
        }
        
        Ok(Self {
            config,
            current_position: None,
            status: GpsStatus::Initializing,
            acquisition_start: None,
            last_update: None,
            signal_lost_time: None,
            satellite_count: 0,
        })
    }
    
    /// Check if GPS signal has been lost for too long (>5 minutes)
    fn is_degraded_mode(&self) -> bool {
        if let Some(signal_lost_time) = self.signal_lost_time {
            signal_lost_time.elapsed() > Duration::from_secs(300) // 5 minutes
        } else {
            false
        }
    }
    
    /// Simulate GPS hardware interaction (placeholder for real implementation)
    fn read_gps_data(&mut self) -> Result<Option<GpsPosition>, GpsError> {
        // This would interface with actual GPS hardware
        // For now, return None to simulate no data available
        Ok(None)
    }
    
    /// Check if position meets accuracy requirements
    fn is_position_accurate(&self, position: &GpsPosition) -> bool {
        position.accuracy_m <= self.config.accuracy_threshold_m &&
        position.satellite_count >= self.config.min_satellite_count
    }
}

impl GpsManager for BasicGpsManager {
    fn start_acquisition(&mut self) -> Result<(), GpsError> {
        self.status = GpsStatus::Acquiring;
        self.acquisition_start = Some(Instant::now());
        self.signal_lost_time = None;
        Ok(())
    }
    
    fn get_current_position(&self) -> Option<GpsPosition> {
        self.current_position.clone()
    }
    
    fn get_position_accuracy(&self) -> Option<f32> {
        self.current_position.as_ref().map(|pos| pos.accuracy_m)
    }
    
    fn is_locked(&self) -> bool {
        matches!(self.status, GpsStatus::Locked)
    }
    
    fn get_satellite_count(&self) -> u8 {
        self.satellite_count
    }
    
    fn configure(&mut self, config: GpsConfig) -> Result<(), GpsError> {
        // Validate new configuration
        if config.acquisition_timeout_s == 0 || config.update_interval_s == 0 {
            return Err(GpsError::ConfigurationInvalid);
        }
        
        if config.min_satellite_count < 3 {
            return Err(GpsError::ConfigurationInvalid);
        }
        
        self.config = config;
        Ok(())
    }
    
    fn get_status(&self) -> GpsStatus {
        self.status.clone()
    }
    
    fn update(&mut self) -> Result<(), GpsError> {
        match self.status {
            GpsStatus::Initializing => {
                // Ready to start acquisition
                Ok(())
            }
            GpsStatus::Acquiring => {
                // Check for acquisition timeout
                if let Some(start_time) = self.acquisition_start {
                    if start_time.elapsed() > Duration::from_secs(self.config.acquisition_timeout_s as u64) {
                        self.status = GpsStatus::HardwareFault;
                        return Err(GpsError::AcquisitionTimeout);
                    }
                }
                
                // Try to read GPS data
                match self.read_gps_data()? {
                    Some(position) => {
                        if self.is_position_accurate(&position) {
                            self.current_position = Some(position.clone());
                            self.satellite_count = position.satellite_count;
                            self.status = GpsStatus::Locked;
                            self.last_update = Some(Instant::now());
                            self.signal_lost_time = None;
                        } else {
                            // Position available but not accurate enough
                            self.satellite_count = position.satellite_count;
                        }
                    }
                    None => {
                        // No GPS data available yet
                    }
                }
                Ok(())
            }
            GpsStatus::Locked => {
                // Check if we need to update position
                let should_update = if let Some(last_update) = self.last_update {
                    last_update.elapsed() >= Duration::from_secs(self.config.update_interval_s as u64)
                } else {
                    true
                };
                
                if should_update {
                    match self.read_gps_data()? {
                        Some(position) => {
                            if self.is_position_accurate(&position) {
                                self.current_position = Some(position.clone());
                                self.satellite_count = position.satellite_count;
                                self.last_update = Some(Instant::now());
                                self.signal_lost_time = None;
                            } else {
                                // Signal quality degraded
                                self.satellite_count = position.satellite_count;
                                if self.signal_lost_time.is_none() {
                                    self.signal_lost_time = Some(Instant::now());
                                }
                                
                                if self.is_degraded_mode() {
                                    self.status = GpsStatus::DegradedMode;
                                }
                            }
                        }
                        None => {
                            // Signal lost
                            if self.signal_lost_time.is_none() {
                                self.signal_lost_time = Some(Instant::now());
                            }
                            
                            self.status = GpsStatus::SignalLost;
                            
                            if self.is_degraded_mode() {
                                self.status = GpsStatus::DegradedMode;
                            }
                        }
                    }
                }
                Ok(())
            }
            GpsStatus::SignalLost => {
                // Try to reacquire signal
                match self.read_gps_data()? {
                    Some(position) => {
                        if self.is_position_accurate(&position) {
                            self.current_position = Some(position.clone());
                            self.satellite_count = position.satellite_count;
                            self.status = GpsStatus::Locked;
                            self.last_update = Some(Instant::now());
                            self.signal_lost_time = None;
                        } else {
                            self.satellite_count = position.satellite_count;
                        }
                    }
                    None => {
                        if self.is_degraded_mode() {
                            self.status = GpsStatus::DegradedMode;
                        }
                    }
                }
                Ok(())
            }
            GpsStatus::DegradedMode => {
                // Continue trying to reacquire signal
                match self.read_gps_data()? {
                    Some(position) => {
                        if self.is_position_accurate(&position) {
                            self.current_position = Some(position.clone());
                            self.satellite_count = position.satellite_count;
                            self.status = GpsStatus::Locked;
                            self.last_update = Some(Instant::now());
                            self.signal_lost_time = None;
                        } else {
                            self.satellite_count = position.satellite_count;
                        }
                    }
                    None => {
                        // Still no signal
                    }
                }
                Ok(())
            }
            GpsStatus::HardwareFault => {
                // Hardware fault - cannot recover automatically
                Err(GpsError::HardwareFault)
            }
        }
    }
    
    fn stop(&mut self) -> Result<(), GpsError> {
        self.status = GpsStatus::Initializing;
        self.acquisition_start = None;
        self.last_update = None;
        self.signal_lost_time = None;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_gps_config_default() {
        let config = GpsConfig::default();
        assert_eq!(config.acquisition_timeout_s, 60);
        assert_eq!(config.update_interval_s, 5);
        assert_eq!(config.min_satellite_count, 4);
        assert_eq!(config.accuracy_threshold_m, 10.0);
        assert_eq!(config.cold_start_timeout_s, 120);
    }
    
    #[test]
    fn test_gps_config_validation() {
        // Valid configuration
        let valid_config = GpsConfig {
            acquisition_timeout_s: 60,
            update_interval_s: 5,
            min_satellite_count: 4,
            accuracy_threshold_m: 10.0,
            cold_start_timeout_s: 120,
        };
        
        let manager = BasicGpsManager::new(valid_config);
        assert!(manager.is_ok());
        
        // Invalid configuration - zero timeout
        let invalid_config = GpsConfig {
            acquisition_timeout_s: 0,
            update_interval_s: 5,
            min_satellite_count: 4,
            accuracy_threshold_m: 10.0,
            cold_start_timeout_s: 120,
        };
        
        let manager = BasicGpsManager::new(invalid_config);
        assert!(matches!(manager, Err(GpsError::ConfigurationInvalid)));
        
        // Invalid configuration - too few satellites
        let invalid_config = GpsConfig {
            acquisition_timeout_s: 60,
            update_interval_s: 5,
            min_satellite_count: 2,
            accuracy_threshold_m: 10.0,
            cold_start_timeout_s: 120,
        };
        
        let manager = BasicGpsManager::new(invalid_config);
        assert!(matches!(manager, Err(GpsError::ConfigurationInvalid)));
    }
    
    #[test]
    fn test_basic_gps_manager_initialization() {
        let config = GpsConfig::default();
        let manager = BasicGpsManager::new(config).unwrap();
        
        assert_eq!(manager.get_status(), GpsStatus::Initializing);
        assert!(!manager.is_locked());
        assert_eq!(manager.get_satellite_count(), 0);
        assert!(manager.get_current_position().is_none());
        assert!(manager.get_position_accuracy().is_none());
    }
    
    #[test]
    fn test_start_acquisition() {
        let config = GpsConfig::default();
        let mut manager = BasicGpsManager::new(config).unwrap();
        
        let result = manager.start_acquisition();
        assert!(result.is_ok());
        assert_eq!(manager.get_status(), GpsStatus::Acquiring);
    }
    
    #[test]
    fn test_stop_gps() {
        let config = GpsConfig::default();
        let mut manager = BasicGpsManager::new(config).unwrap();
        
        manager.start_acquisition().unwrap();
        assert_eq!(manager.get_status(), GpsStatus::Acquiring);
        
        let result = manager.stop();
        assert!(result.is_ok());
        assert_eq!(manager.get_status(), GpsStatus::Initializing);
    }
    
    #[test]
    fn test_configure() {
        let config = GpsConfig::default();
        let mut manager = BasicGpsManager::new(config).unwrap();
        
        let new_config = GpsConfig {
            acquisition_timeout_s: 120,
            update_interval_s: 10,
            min_satellite_count: 6,
            accuracy_threshold_m: 5.0,
            cold_start_timeout_s: 180,
        };
        
        let result = manager.configure(new_config.clone());
        assert!(result.is_ok());
        assert_eq!(manager.config.acquisition_timeout_s, 120);
        assert_eq!(manager.config.update_interval_s, 10);
        assert_eq!(manager.config.min_satellite_count, 6);
        assert_eq!(manager.config.accuracy_threshold_m, 5.0);
        assert_eq!(manager.config.cold_start_timeout_s, 180);
    }
    
    #[test]
    fn test_position_accuracy_check() {
        let config = GpsConfig {
            acquisition_timeout_s: 60,
            update_interval_s: 5,
            min_satellite_count: 4,
            accuracy_threshold_m: 10.0,
            cold_start_timeout_s: 120,
        };
        let manager = BasicGpsManager::new(config).unwrap();
        
        // Accurate position
        let accurate_position = GpsPosition {
            latitude: 37.7749,
            longitude: -122.4194,
            altitude: 10.0,
            timestamp: SystemTime::now(),
            accuracy_m: 5.0,
            satellite_count: 6,
        };
        assert!(manager.is_position_accurate(&accurate_position));
        
        // Inaccurate position - poor accuracy
        let inaccurate_position = GpsPosition {
            latitude: 37.7749,
            longitude: -122.4194,
            altitude: 10.0,
            timestamp: SystemTime::now(),
            accuracy_m: 15.0,
            satellite_count: 6,
        };
        assert!(!manager.is_position_accurate(&inaccurate_position));
        
        // Inaccurate position - too few satellites
        let inaccurate_position = GpsPosition {
            latitude: 37.7749,
            longitude: -122.4194,
            altitude: 10.0,
            timestamp: SystemTime::now(),
            accuracy_m: 5.0,
            satellite_count: 3,
        };
        assert!(!manager.is_position_accurate(&inaccurate_position));
    }
}

/// Mock GPS manager for testing without hardware
pub struct MockGpsManager {
    config: GpsConfig,
    current_position: Option<GpsPosition>,
    status: GpsStatus,
    satellite_count: u8,
    simulated_positions: std::collections::VecDeque<GpsPosition>,
    acquisition_delay: Duration,
    signal_loss_simulation: bool,
    hardware_fault_simulation: bool,
    accuracy_variation: f32,
    acquisition_start: Option<Instant>,
    last_update: Option<Instant>,
    signal_lost_time: Option<Instant>,
}

impl MockGpsManager {
    pub fn new(config: GpsConfig) -> Result<Self, GpsError> {
        // Validate configuration
        if config.acquisition_timeout_s == 0 || config.update_interval_s == 0 {
            return Err(GpsError::ConfigurationInvalid);
        }
        
        if config.min_satellite_count < 3 {
            return Err(GpsError::ConfigurationInvalid);
        }
        
        Ok(Self {
            config,
            current_position: None,
            status: GpsStatus::Initializing,
            satellite_count: 0,
            simulated_positions: std::collections::VecDeque::new(),
            acquisition_delay: Duration::from_secs(5),
            signal_loss_simulation: false,
            hardware_fault_simulation: false,
            accuracy_variation: 0.0,
            acquisition_start: None,
            last_update: None,
            signal_lost_time: None,
        })
    }
    
    /// Add simulated GPS positions for testing
    pub fn add_simulated_position(&mut self, position: GpsPosition) {
        self.simulated_positions.push_back(position);
    }
    
    /// Set acquisition delay for testing
    pub fn set_acquisition_delay(&mut self, delay: Duration) {
        self.acquisition_delay = delay;
    }
    
    /// Simulate signal loss for testing
    pub fn simulate_signal_loss(&mut self, enable: bool) {
        self.signal_loss_simulation = enable;
    }
    
    /// Simulate hardware fault for testing
    pub fn simulate_hardware_fault(&mut self, enable: bool) {
        self.hardware_fault_simulation = enable;
    }
    
    /// Set accuracy variation for testing
    pub fn set_accuracy_variation(&mut self, variation: f32) {
        self.accuracy_variation = variation;
    }
    
    /// Create a mock GPS manager with default test positions
    pub fn with_test_positions(config: GpsConfig) -> Result<Self, GpsError> {
        let mut manager = Self::new(config)?;
        
        // Add some test positions
        let test_positions = vec![
            GpsPosition {
                latitude: 37.7749,
                longitude: -122.4194,
                altitude: 10.0,
                timestamp: SystemTime::now(),
                accuracy_m: 3.0,
                satellite_count: 8,
            },
            GpsPosition {
                latitude: 37.7750,
                longitude: -122.4195,
                altitude: 11.0,
                timestamp: SystemTime::now(),
                accuracy_m: 4.0,
                satellite_count: 7,
            },
            GpsPosition {
                latitude: 37.7751,
                longitude: -122.4196,
                altitude: 12.0,
                timestamp: SystemTime::now(),
                accuracy_m: 5.0,
                satellite_count: 6,
            },
        ];
        
        for position in test_positions {
            manager.add_simulated_position(position);
        }
        
        Ok(manager)
    }
    
    /// Simulate GPS data reading
    fn read_simulated_gps_data(&mut self) -> Result<Option<GpsPosition>, GpsError> {
        if self.hardware_fault_simulation {
            return Err(GpsError::HardwareFault);
        }
        
        if self.signal_loss_simulation {
            return Ok(None);
        }
        
        // Check if enough time has passed for acquisition
        if let Some(start_time) = self.acquisition_start {
            if start_time.elapsed() < self.acquisition_delay {
                return Ok(None);
            }
        }
        
        // Get next simulated position
        if let Some(mut position) = self.simulated_positions.pop_front() {
            // Apply accuracy variation
            position.accuracy_m += self.accuracy_variation;
            position.timestamp = SystemTime::now();
            
            // Cycle the position back for continuous simulation
            self.simulated_positions.push_back(position.clone());
            
            Ok(Some(position))
        } else {
            // No simulated positions available
            Ok(None)
        }
    }
    
    /// Check if position meets accuracy requirements
    fn is_position_accurate(&self, position: &GpsPosition) -> bool {
        position.accuracy_m <= self.config.accuracy_threshold_m &&
        position.satellite_count >= self.config.min_satellite_count
    }
    
    /// Check if GPS signal has been lost for too long (>5 minutes)
    fn is_degraded_mode(&self) -> bool {
        if let Some(signal_lost_time) = self.signal_lost_time {
            signal_lost_time.elapsed() > Duration::from_secs(300) // 5 minutes
        } else {
            false
        }
    }
}

impl GpsManager for MockGpsManager {
    fn start_acquisition(&mut self) -> Result<(), GpsError> {
        if self.hardware_fault_simulation {
            self.status = GpsStatus::HardwareFault;
            return Err(GpsError::HardwareFault);
        }
        
        self.status = GpsStatus::Acquiring;
        self.acquisition_start = Some(Instant::now());
        self.signal_lost_time = None;
        Ok(())
    }
    
    fn get_current_position(&self) -> Option<GpsPosition> {
        self.current_position.clone()
    }
    
    fn get_position_accuracy(&self) -> Option<f32> {
        self.current_position.as_ref().map(|pos| pos.accuracy_m)
    }
    
    fn is_locked(&self) -> bool {
        matches!(self.status, GpsStatus::Locked)
    }
    
    fn get_satellite_count(&self) -> u8 {
        self.satellite_count
    }
    
    fn configure(&mut self, config: GpsConfig) -> Result<(), GpsError> {
        // Validate new configuration
        if config.acquisition_timeout_s == 0 || config.update_interval_s == 0 {
            return Err(GpsError::ConfigurationInvalid);
        }
        
        if config.min_satellite_count < 3 {
            return Err(GpsError::ConfigurationInvalid);
        }
        
        self.config = config;
        Ok(())
    }
    
    fn get_status(&self) -> GpsStatus {
        self.status.clone()
    }
    
    fn update(&mut self) -> Result<(), GpsError> {
        match self.status {
            GpsStatus::Initializing => {
                // Ready to start acquisition
                Ok(())
            }
            GpsStatus::Acquiring => {
                // Check for acquisition timeout
                if let Some(start_time) = self.acquisition_start {
                    if start_time.elapsed() > Duration::from_secs(self.config.acquisition_timeout_s as u64) {
                        self.status = GpsStatus::HardwareFault;
                        return Err(GpsError::AcquisitionTimeout);
                    }
                }
                
                // Try to read GPS data
                match self.read_simulated_gps_data()? {
                    Some(position) => {
                        if self.is_position_accurate(&position) {
                            self.current_position = Some(position.clone());
                            self.satellite_count = position.satellite_count;
                            self.status = GpsStatus::Locked;
                            self.last_update = Some(Instant::now());
                            self.signal_lost_time = None;
                        } else {
                            // Position available but not accurate enough
                            self.satellite_count = position.satellite_count;
                        }
                    }
                    None => {
                        // No GPS data available yet
                    }
                }
                Ok(())
            }
            GpsStatus::Locked => {
                // Check if we need to update position
                let should_update = if let Some(last_update) = self.last_update {
                    last_update.elapsed() >= Duration::from_secs(self.config.update_interval_s as u64)
                } else {
                    true
                };
                
                if should_update {
                    match self.read_simulated_gps_data()? {
                        Some(position) => {
                            if self.is_position_accurate(&position) {
                                self.current_position = Some(position.clone());
                                self.satellite_count = position.satellite_count;
                                self.last_update = Some(Instant::now());
                                self.signal_lost_time = None;
                            } else {
                                // Signal quality degraded
                                self.satellite_count = position.satellite_count;
                                if self.signal_lost_time.is_none() {
                                    self.signal_lost_time = Some(Instant::now());
                                }
                                
                                if self.is_degraded_mode() {
                                    self.status = GpsStatus::DegradedMode;
                                }
                            }
                        }
                        None => {
                            // Signal lost
                            if self.signal_lost_time.is_none() {
                                self.signal_lost_time = Some(Instant::now());
                            }
                            
                            self.status = GpsStatus::SignalLost;
                            
                            if self.is_degraded_mode() {
                                self.status = GpsStatus::DegradedMode;
                            }
                        }
                    }
                }
                Ok(())
            }
            GpsStatus::SignalLost => {
                // Try to reacquire signal
                match self.read_simulated_gps_data()? {
                    Some(position) => {
                        if self.is_position_accurate(&position) {
                            self.current_position = Some(position.clone());
                            self.satellite_count = position.satellite_count;
                            self.status = GpsStatus::Locked;
                            self.last_update = Some(Instant::now());
                            self.signal_lost_time = None;
                        } else {
                            self.satellite_count = position.satellite_count;
                        }
                    }
                    None => {
                        if self.is_degraded_mode() {
                            self.status = GpsStatus::DegradedMode;
                        }
                    }
                }
                Ok(())
            }
            GpsStatus::DegradedMode => {
                // Continue trying to reacquire signal
                match self.read_simulated_gps_data()? {
                    Some(position) => {
                        if self.is_position_accurate(&position) {
                            self.current_position = Some(position.clone());
                            self.satellite_count = position.satellite_count;
                            self.status = GpsStatus::Locked;
                            self.last_update = Some(Instant::now());
                            self.signal_lost_time = None;
                        } else {
                            self.satellite_count = position.satellite_count;
                        }
                    }
                    None => {
                        // Still no signal
                    }
                }
                Ok(())
            }
            GpsStatus::HardwareFault => {
                // Hardware fault - cannot recover automatically
                Err(GpsError::HardwareFault)
            }
        }
    }
    
    fn stop(&mut self) -> Result<(), GpsError> {
        self.status = GpsStatus::Initializing;
        self.acquisition_start = None;
        self.last_update = None;
        self.signal_lost_time = None;
        Ok(())
    }
}

#[cfg(test)]
mod mock_tests {
    use super::*;
    
    #[test]
    fn test_mock_gps_manager_initialization() {
        let config = GpsConfig::default();
        let manager = MockGpsManager::new(config).unwrap();
        
        assert_eq!(manager.get_status(), GpsStatus::Initializing);
        assert!(!manager.is_locked());
        assert_eq!(manager.get_satellite_count(), 0);
        assert!(manager.get_current_position().is_none());
        assert!(manager.get_position_accuracy().is_none());
    }
    
    #[test]
    fn test_mock_gps_with_test_positions() {
        let config = GpsConfig::default();
        let manager = MockGpsManager::with_test_positions(config).unwrap();
        
        assert_eq!(manager.simulated_positions.len(), 3);
    }
    
    #[test]
    fn test_mock_gps_acquisition() {
        let config = GpsConfig::default();
        let mut manager = MockGpsManager::with_test_positions(config).unwrap();
        
        // Set short acquisition delay for testing
        manager.set_acquisition_delay(Duration::from_millis(100));
        
        // Start acquisition
        manager.start_acquisition().unwrap();
        assert_eq!(manager.get_status(), GpsStatus::Acquiring);
        
        // Wait for acquisition delay and update
        std::thread::sleep(Duration::from_millis(150));
        manager.update().unwrap();
        
        // Should have acquired lock
        assert_eq!(manager.get_status(), GpsStatus::Locked);
        assert!(manager.is_locked());
        assert!(manager.get_current_position().is_some());
        assert!(manager.get_satellite_count() > 0);
    }
    
    #[test]
    fn test_mock_gps_signal_loss_simulation() {
        let config = GpsConfig {
            update_interval_s: 1, // Short update interval for testing
            ..GpsConfig::default()
        };
        let mut manager = MockGpsManager::with_test_positions(config).unwrap();
        
        // Set short acquisition delay
        manager.set_acquisition_delay(Duration::from_millis(100));
        
        // Start and acquire lock
        manager.start_acquisition().unwrap();
        std::thread::sleep(Duration::from_millis(150));
        manager.update().unwrap();
        assert_eq!(manager.get_status(), GpsStatus::Locked);
        
        // Wait for update interval to pass, then simulate signal loss
        std::thread::sleep(Duration::from_secs(1));
        manager.simulate_signal_loss(true);
        manager.update().unwrap();
        
        // Should detect signal loss
        assert_eq!(manager.get_status(), GpsStatus::SignalLost);
        
        // Restore signal
        manager.simulate_signal_loss(false);
        manager.update().unwrap();
        
        // Should reacquire lock
        assert_eq!(manager.get_status(), GpsStatus::Locked);
    }
    
    #[test]
    fn test_mock_gps_hardware_fault_simulation() {
        let config = GpsConfig::default();
        let mut manager = MockGpsManager::with_test_positions(config).unwrap();
        
        // Simulate hardware fault
        manager.simulate_hardware_fault(true);
        
        // Should fail to start acquisition
        let result = manager.start_acquisition();
        assert!(matches!(result, Err(GpsError::HardwareFault)));
        assert_eq!(manager.get_status(), GpsStatus::HardwareFault);
    }
    
    #[test]
    fn test_mock_gps_accuracy_variation() {
        let config = GpsConfig {
            accuracy_threshold_m: 5.0,
            ..GpsConfig::default()
        };
        let mut manager = MockGpsManager::with_test_positions(config).unwrap();
        
        // Set high accuracy variation to make positions inaccurate
        manager.set_accuracy_variation(10.0);
        manager.set_acquisition_delay(Duration::from_millis(100));
        
        // Start acquisition
        manager.start_acquisition().unwrap();
        std::thread::sleep(Duration::from_millis(150));
        manager.update().unwrap();
        
        // Should still be acquiring due to poor accuracy
        assert_eq!(manager.get_status(), GpsStatus::Acquiring);
        
        // Reset accuracy variation
        manager.set_accuracy_variation(0.0);
        manager.update().unwrap();
        
        // Should now acquire lock
        assert_eq!(manager.get_status(), GpsStatus::Locked);
    }
    
    #[test]
    fn test_mock_gps_acquisition_timeout() {
        let config = GpsConfig {
            acquisition_timeout_s: 1, // Very short timeout
            ..GpsConfig::default()
        };
        let mut manager = MockGpsManager::new(config).unwrap();
        
        // Set long acquisition delay to trigger timeout
        manager.set_acquisition_delay(Duration::from_secs(2));
        
        // Start acquisition
        manager.start_acquisition().unwrap();
        
        // Wait for timeout and update
        std::thread::sleep(Duration::from_secs(2));
        let result = manager.update();
        
        // Should timeout
        assert!(matches!(result, Err(GpsError::AcquisitionTimeout)));
        assert_eq!(manager.get_status(), GpsStatus::HardwareFault);
    }
}