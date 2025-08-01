// Watchdog timer system for beacon reliability and fault tolerance
// Implements redundant system monitoring with automatic recovery and emergency procedures

use std::time::{Duration, SystemTime, Instant};
use std::sync::{Arc, Mutex, atomic::{AtomicBool, Ordering}};
use std::thread::{self, JoinHandle};
use std::sync::mpsc::{self, Receiver, Sender};
use std::collections::hash_map::DefaultHasher;
use std::hash::{Hash, Hasher};
use serde::{Serialize, Deserialize};

use crate::{
    BeaconError, SystemErrorType, HardwareComponent, ErrorSeverity,
    ReliabilityError, FailureEvent
};

/// Watchdog timer errors
#[derive(Debug, Clone, PartialEq)]
pub enum WatchdogError {
    TimerExpired { component: String, timeout_ms: u64 },
    SystemUnresponsive { last_heartbeat: SystemTime },
    RecoveryFailed { component: String, attempts: u32 },
    ConfigurationInvalid { reason: String },
    MonitoringFailed { reason: String },
}

impl std::fmt::Display for WatchdogError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            WatchdogError::TimerExpired { component, timeout_ms } => {
                write!(f, "Watchdog timer expired for {} after {} ms", component, timeout_ms)
            }
            WatchdogError::SystemUnresponsive { last_heartbeat } => {
                write!(f, "System unresponsive since {:?}", last_heartbeat)
            }
            WatchdogError::RecoveryFailed { component, attempts } => {
                write!(f, "Recovery failed for {} after {} attempts", component, attempts)
            }
            WatchdogError::ConfigurationInvalid { reason } => {
                write!(f, "Invalid watchdog configuration: {}", reason)
            }
            WatchdogError::MonitoringFailed { reason } => {
                write!(f, "Watchdog monitoring failed: {}", reason)
            }
        }
    }
}

impl std::error::Error for WatchdogError {}

/// Watchdog timer configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WatchdogConfig {
    /// Enable watchdog monitoring
    pub enabled: bool,
    /// Global system timeout in milliseconds
    pub system_timeout_ms: u64,
    /// Component-specific timeouts
    pub component_timeouts: std::collections::HashMap<String, u64>,
    /// Maximum recovery attempts before emergency shutdown
    pub max_recovery_attempts: u32,
    /// Enable automatic recovery
    pub auto_recovery_enabled: bool,
    /// Enable emergency shutdown on critical failures
    pub emergency_shutdown_enabled: bool,
    /// Heartbeat interval for system monitoring
    pub heartbeat_interval_ms: u64,
}

impl Default for WatchdogConfig {
    fn default() -> Self {
        let mut component_timeouts = std::collections::HashMap::new();
        component_timeouts.insert("gps".to_string(), 30000);  // 30 seconds
        component_timeouts.insert("transceiver".to_string(), 10000);  // 10 seconds
        component_timeouts.insert("power".to_string(), 5000);  // 5 seconds
        component_timeouts.insert("communication".to_string(), 60000);  // 60 seconds
        component_timeouts.insert("transmission".to_string(), 15000);  // 15 seconds

        Self {
            enabled: true,
            system_timeout_ms: 120000,  // 2 minutes
            component_timeouts,
            max_recovery_attempts: 3,
            auto_recovery_enabled: true,
            emergency_shutdown_enabled: true,
            heartbeat_interval_ms: 5000,  // 5 seconds
        }
    }
}

/// Watchdog timer event types
#[derive(Debug, Clone)]
pub enum WatchdogEvent {
    Heartbeat { component: String, timestamp: SystemTime },
    Timeout { component: String, timeout_ms: u64 },
    RecoveryAttempt { component: String, attempt: u32 },
    RecoverySuccess { component: String, attempts: u32 },
    RecoveryFailure { component: String, attempts: u32 },
    EmergencyShutdown { reason: String },
    SystemReset { reason: String },
}

/// Watchdog monitoring statistics
#[derive(Debug, Clone, Default)]
pub struct WatchdogStats {
    pub total_heartbeats: u64,
    pub timeout_events: u32,
    pub recovery_attempts: u32,
    pub successful_recoveries: u32,
    pub failed_recoveries: u32,
    pub emergency_shutdowns: u32,
    pub system_resets: u32,
    pub uptime: Duration,
    pub last_heartbeat: Option<SystemTime>,
    pub component_health: std::collections::HashMap<String, ComponentWatchdogHealth>,
}

/// Component watchdog health status
#[derive(Debug, Clone)]
pub struct ComponentWatchdogHealth {
    pub last_heartbeat: SystemTime,
    pub timeout_count: u32,
    pub recovery_count: u32,
    pub health_score: f64,  // 0.0 to 1.0
    pub is_responsive: bool,
}

/// Watchdog timer system for redundant monitoring
pub struct WatchdogTimer {
    config: WatchdogConfig,
    statistics: Arc<Mutex<WatchdogStats>>,
    component_heartbeats: Arc<Mutex<std::collections::HashMap<String, SystemTime>>>,
    recovery_attempts: Arc<Mutex<std::collections::HashMap<String, u32>>>,
    event_sender: Option<Sender<WatchdogEvent>>,
    event_receiver: Option<Receiver<WatchdogEvent>>,
    monitoring_thread: Option<JoinHandle<()>>,
    running: Arc<AtomicBool>,
    start_time: SystemTime,
}

impl WatchdogTimer {
    /// Create new watchdog timer
    pub fn new(config: WatchdogConfig) -> Result<Self, WatchdogError> {
        Self::validate_config(&config)?;

        let (sender, receiver) = mpsc::channel();

        Ok(Self {
            config,
            statistics: Arc::new(Mutex::new(WatchdogStats::default())),
            component_heartbeats: Arc::new(Mutex::new(std::collections::HashMap::new())),
            recovery_attempts: Arc::new(Mutex::new(std::collections::HashMap::new())),
            event_sender: Some(sender),
            event_receiver: Some(receiver),
            monitoring_thread: None,
            running: Arc::new(AtomicBool::new(false)),
            start_time: SystemTime::now(),
        })
    }

    /// Validate watchdog configuration
    fn validate_config(config: &WatchdogConfig) -> Result<(), WatchdogError> {
        if config.system_timeout_ms < 1000 {
            return Err(WatchdogError::ConfigurationInvalid {
                reason: "System timeout must be at least 1000ms".to_string(),
            });
        }

        if config.heartbeat_interval_ms < 100 {
            return Err(WatchdogError::ConfigurationInvalid {
                reason: "Heartbeat interval must be at least 100ms".to_string(),
            });
        }

        if config.heartbeat_interval_ms >= config.system_timeout_ms / 2 {
            return Err(WatchdogError::ConfigurationInvalid {
                reason: "Heartbeat interval must be less than half of system timeout".to_string(),
            });
        }

        for (component, timeout) in &config.component_timeouts {
            if *timeout < 1000 {
                return Err(WatchdogError::ConfigurationInvalid {
                    reason: format!("Component timeout for {} must be at least 1000ms", component),
                });
            }
        }

        Ok(())
    }

    /// Start watchdog monitoring
    pub fn start(&mut self) -> Result<(), WatchdogError> {
        if !self.config.enabled {
            return Ok(());
        }

        if self.running.load(Ordering::Relaxed) {
            return Ok(());
        }

        self.running.store(true, Ordering::Relaxed);
        self.start_time = SystemTime::now();

        // Initialize component heartbeats
        {
            let mut heartbeats = self.component_heartbeats.lock().unwrap();
            let now = SystemTime::now();
            for component in self.config.component_timeouts.keys() {
                heartbeats.insert(component.clone(), now);
            }
        }

        // Start monitoring thread
        let running = Arc::clone(&self.running);
        let config = self.config.clone();
        let statistics = Arc::clone(&self.statistics);
        let heartbeats = Arc::clone(&self.component_heartbeats);
        let recovery_attempts = Arc::clone(&self.recovery_attempts);
        let event_sender = self.event_sender.take().unwrap();

        let monitoring_thread = thread::spawn(move || {
            Self::monitoring_loop(running, config, statistics, heartbeats, recovery_attempts, event_sender);
        });

        self.monitoring_thread = Some(monitoring_thread);

        Ok(())
    }

    /// Stop watchdog monitoring
    pub fn stop(&mut self) -> Result<(), WatchdogError> {
        if !self.running.load(Ordering::Relaxed) {
            return Ok(());
        }

        self.running.store(false, Ordering::Relaxed);

        if let Some(thread) = self.monitoring_thread.take() {
            if let Err(_) = thread.join() {
                return Err(WatchdogError::MonitoringFailed {
                    reason: "Failed to stop monitoring thread".to_string(),
                });
            }
        }

        // Update final statistics
        {
            let mut stats = self.statistics.lock().unwrap();
            stats.uptime = SystemTime::now().duration_since(self.start_time)
                .unwrap_or(Duration::from_secs(0));
        }

        Ok(())
    }

    /// Send heartbeat for a component
    pub fn heartbeat(&self, component: &str) -> Result<(), WatchdogError> {
        if !self.config.enabled || !self.running.load(Ordering::Relaxed) {
            return Ok(());
        }

        let now = SystemTime::now();

        // Update heartbeat timestamp
        {
            let mut heartbeats = self.component_heartbeats.lock().unwrap();
            heartbeats.insert(component.to_string(), now);
        }

        // Update statistics
        {
            let mut stats = self.statistics.lock().unwrap();
            stats.total_heartbeats += 1;
            stats.last_heartbeat = Some(now);

            // Update component health
            let health = stats.component_health.entry(component.to_string())
                .or_insert_with(|| ComponentWatchdogHealth {
                    last_heartbeat: now,
                    timeout_count: 0,
                    recovery_count: 0,
                    health_score: 1.0,
                    is_responsive: true,
                });

            health.last_heartbeat = now;
            health.is_responsive = true;
            
            // Improve health score on successful heartbeat
            health.health_score = (health.health_score + 0.1).min(1.0);
        }

        Ok(())
    }

    /// Main monitoring loop (runs in separate thread)
    fn monitoring_loop(
        running: Arc<AtomicBool>,
        config: WatchdogConfig,
        statistics: Arc<Mutex<WatchdogStats>>,
        heartbeats: Arc<Mutex<std::collections::HashMap<String, SystemTime>>>,
        recovery_attempts: Arc<Mutex<std::collections::HashMap<String, u32>>>,
        event_sender: Sender<WatchdogEvent>,
    ) {
        let heartbeat_interval = Duration::from_millis(config.heartbeat_interval_ms);

        while running.load(Ordering::Relaxed) {
            let now = SystemTime::now();

            // Check component timeouts
            {
                let heartbeats_guard = heartbeats.lock().unwrap();
                for (component, timeout_ms) in &config.component_timeouts {
                    if let Some(last_heartbeat) = heartbeats_guard.get(component) {
                        let elapsed = now.duration_since(*last_heartbeat)
                            .unwrap_or(Duration::from_secs(0));

                        if elapsed.as_millis() > *timeout_ms as u128 {
                            // Component timeout detected
                            let _ = event_sender.send(WatchdogEvent::Timeout {
                                component: component.clone(),
                                timeout_ms: *timeout_ms,
                            });

                            // Update statistics
                            {
                                let mut stats = statistics.lock().unwrap();
                                stats.timeout_events += 1;

                                // Update component health
                                if let Some(health) = stats.component_health.get_mut(component) {
                                    health.timeout_count += 1;
                                    health.is_responsive = false;
                                    health.health_score = (health.health_score - 0.2).max(0.0);
                                }
                            }

                            // Attempt recovery if enabled
                            if config.auto_recovery_enabled {
                                let mut attempts_guard = recovery_attempts.lock().unwrap();
                                let attempts = attempts_guard.entry(component.clone()).or_insert(0);
                                *attempts += 1;

                                if *attempts <= config.max_recovery_attempts {
                                    let _ = event_sender.send(WatchdogEvent::RecoveryAttempt {
                                        component: component.clone(),
                                        attempt: *attempts,
                                    });

                                    // Simulate recovery attempt
                                    if Self::attempt_component_recovery(component) {
                                        let _ = event_sender.send(WatchdogEvent::RecoverySuccess {
                                            component: component.clone(),
                                            attempts: *attempts,
                                        });

                                        // Reset recovery attempts on success
                                        *attempts = 0;

                                        // Update statistics
                                        {
                                            let mut stats = statistics.lock().unwrap();
                                            stats.successful_recoveries += 1;

                                            if let Some(health) = stats.component_health.get_mut(component) {
                                                health.recovery_count += 1;
                                                health.health_score = (health.health_score + 0.3).min(1.0);
                                            }
                                        }
                                    } else {
                                        let _ = event_sender.send(WatchdogEvent::RecoveryFailure {
                                            component: component.clone(),
                                            attempts: *attempts,
                                        });

                                        // Update statistics
                                        {
                                            let mut stats = statistics.lock().unwrap();
                                            stats.failed_recoveries += 1;
                                        }

                                        // Check if we've exceeded max attempts
                                        if *attempts >= config.max_recovery_attempts {
                                            if config.emergency_shutdown_enabled {
                                                let _ = event_sender.send(WatchdogEvent::EmergencyShutdown {
                                                    reason: format!("Component {} failed recovery after {} attempts", 
                                                                  component, attempts),
                                                });

                                                // Update statistics
                                                {
                                                    let mut stats = statistics.lock().unwrap();
                                                    stats.emergency_shutdowns += 1;
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // Check system-wide timeout
            {
                let stats = statistics.lock().unwrap();
                if let Some(last_heartbeat) = stats.last_heartbeat {
                    let elapsed = now.duration_since(last_heartbeat)
                        .unwrap_or(Duration::from_secs(0));

                    if elapsed.as_millis() > config.system_timeout_ms as u128 {
                        let _ = event_sender.send(WatchdogEvent::SystemReset {
                            reason: format!("System unresponsive for {} ms", elapsed.as_millis()),
                        });

                        // This would trigger a system reset in a real implementation
                        break;
                    }
                }
            }

            thread::sleep(heartbeat_interval);
        }
    }

    /// Attempt to recover a component (simplified implementation)
    fn attempt_component_recovery(component: &str) -> bool {
        // In a real implementation, this would perform actual recovery procedures
        // For now, we simulate recovery with a success rate based on component type
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};
        
        // Use a deterministic "random" based on component name and current time
        let mut hasher = DefaultHasher::new();
        component.hash(&mut hasher);
        SystemTime::now().duration_since(SystemTime::UNIX_EPOCH)
            .unwrap_or(Duration::from_secs(0))
            .as_millis()
            .hash(&mut hasher);
        
        let pseudo_random = (hasher.finish() % 100) as f64 / 100.0;
        
        match component {
            "gps" => pseudo_random > 0.3,  // 70% success rate
            "transceiver" => pseudo_random > 0.2,  // 80% success rate
            "power" => pseudo_random > 0.1,  // 90% success rate
            "communication" => pseudo_random > 0.4,  // 60% success rate
            _ => pseudo_random > 0.5,  // 50% success rate
        }
    }

    /// Process watchdog events
    pub fn process_events(&mut self) -> Vec<WatchdogEvent> {
        let mut events = Vec::new();

        if let Some(receiver) = &self.event_receiver {
            while let Ok(event) = receiver.try_recv() {
                events.push(event);
            }
        }

        events
    }

    /// Get watchdog statistics
    pub fn get_statistics(&self) -> WatchdogStats {
        let stats = self.statistics.lock().unwrap();
        let mut result = stats.clone();
        
        // Update uptime
        result.uptime = SystemTime::now().duration_since(self.start_time)
            .unwrap_or(Duration::from_secs(0));

        result
    }

    /// Get component health status
    pub fn get_component_health(&self, component: &str) -> Option<ComponentWatchdogHealth> {
        let stats = self.statistics.lock().unwrap();
        stats.component_health.get(component).cloned()
    }

    /// Check if watchdog is running
    pub fn is_running(&self) -> bool {
        self.running.load(Ordering::Relaxed)
    }

    /// Force emergency shutdown
    pub fn force_emergency_shutdown(&self, reason: String) -> Result<(), WatchdogError> {
        if let Some(sender) = &self.event_sender {
            sender.send(WatchdogEvent::EmergencyShutdown { reason })
                .map_err(|_| WatchdogError::MonitoringFailed {
                    reason: "Failed to send emergency shutdown event".to_string(),
                })?;
        }

        Ok(())
    }

    /// Reset recovery attempts for a component
    pub fn reset_recovery_attempts(&self, component: &str) {
        let mut attempts = self.recovery_attempts.lock().unwrap();
        attempts.insert(component.to_string(), 0);
    }
}

impl Drop for WatchdogTimer {
    fn drop(&mut self) {
        let _ = self.stop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_watchdog_creation() {
        let config = WatchdogConfig::default();
        let watchdog = WatchdogTimer::new(config).unwrap();
        
        assert!(!watchdog.is_running());
    }

    #[test]
    fn test_config_validation() {
        let mut config = WatchdogConfig::default();
        config.system_timeout_ms = 500;  // Too short
        
        let result = WatchdogTimer::new(config);
        assert!(result.is_err());
    }

    #[test]
    fn test_heartbeat() {
        let config = WatchdogConfig::default();
        let mut watchdog = WatchdogTimer::new(config).unwrap();
        
        watchdog.start().unwrap();
        
        let result = watchdog.heartbeat("gps");
        assert!(result.is_ok());
        
        watchdog.stop().unwrap();
    }
}