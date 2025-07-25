// Hardware fault detection and recovery system for beacon reliability
// Implements comprehensive hardware monitoring, fault detection, and automated recovery procedures

use std::time::{Duration, SystemTime, Instant};
use std::collections::{HashMap, VecDeque};
use serde::{Serialize, Deserialize};

use crate::{
    GpsManager, PowerManager, CommunicationManager, TransceiverInterface,
    BeaconError, HardwareComponent, HardwareFaultType, ErrorSeverity
};

/// Hardware fault detection errors
#[derive(Debug, Clone, PartialEq)]
pub enum HardwareFaultError {
    DiagnosticFailed { component: String, reason: String },
    RecoveryFailed { component: String, attempts: u32 },
    ComponentUnresponsive { component: String, timeout_ms: u64 },
    CalibrationFailed { component: String, error: String },
    SelfTestFailed { component: String, test_name: String },
    CriticalFaultDetected { component: String, fault_type: String },
}

impl std::fmt::Display for HardwareFaultError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            HardwareFaultError::DiagnosticFailed { component, reason } => {
                write!(f, "Diagnostic failed for {}: {}", component, reason)
            }
            HardwareFaultError::RecoveryFailed { component, attempts } => {
                write!(f, "Recovery failed for {} after {} attempts", component, attempts)
            }
            HardwareFaultError::ComponentUnresponsive { component, timeout_ms } => {
                write!(f, "Component {} unresponsive after {} ms", component, timeout_ms)
            }
            HardwareFaultError::CalibrationFailed { component, error } => {
                write!(f, "Calibration failed for {}: {}", component, error)
            }
            HardwareFaultError::SelfTestFailed { component, test_name } => {
                write!(f, "Self-test '{}' failed for {}", test_name, component)
            }
            HardwareFaultError::CriticalFaultDetected { component, fault_type } => {
                write!(f, "Critical fault in {}: {}", component, fault_type)
            }
        }
    }
}

impl std::error::Error for HardwareFaultError {}

/// Hardware component health status
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum ComponentHealth {
    Healthy,
    Degraded { performance_impact: f32 },
    Faulty { fault_description: String },
    Failed,
    Unknown,
}

/// Hardware diagnostic result
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiagnosticResult {
    pub component: HardwareComponent,
    pub health: ComponentHealth,
    pub response_time_ms: Option<u64>,
    pub error_rate: f32,
    pub last_successful_operation: Option<SystemTime>,
    pub diagnostic_data: Vec<u8>,
    pub recommended_action: RecommendedAction,
    pub timestamp: SystemTime,
}

/// Recommended actions for hardware issues
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum RecommendedAction {
    None,
    Monitor,
    Recalibrate,
    SoftReset,
    HardReset,
    PowerCycle,
    ReplaceComponent,
    EmergencyShutdown,
}

/// Hardware recovery strategy
#[derive(Debug, Clone)]
pub struct RecoveryStrategy {
    pub component: HardwareComponent,
    pub steps: Vec<RecoveryStep>,
    pub max_attempts: u32,
    pub timeout_per_step: Duration,
    pub success_criteria: SuccessCriteria,
}

/// Individual recovery step
#[derive(Debug, Clone)]
pub enum RecoveryStep {
    SoftReset,
    HardReset,
    PowerCycle { duration_ms: u64 },
    Recalibrate,
    ReloadConfiguration,
    RunSelfTest { test_name: String },
    WaitForStabilization { duration_ms: u64 },
    VerifyOperation,
}

/// Success criteria for recovery
#[derive(Debug, Clone)]
pub struct SuccessCriteria {
    pub response_time_threshold_ms: u64,
    pub error_rate_threshold: f32,
    pub required_operations: Vec<String>,
}

/// Hardware monitoring configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HardwareMonitorConfig {
    pub diagnostic_interval_ms: u64,
    pub fault_detection_enabled: bool,
    pub auto_recovery_enabled: bool,
    pub max_recovery_attempts: u32,
    pub component_timeouts: HashMap<String, u64>,
    pub error_rate_thresholds: HashMap<String, f32>,
    pub health_check_intervals: HashMap<String, u64>,
}

impl Default for HardwareMonitorConfig {
    fn default() -> Self {
        let mut component_timeouts = HashMap::new();
        component_timeouts.insert("gps".to_string(), 30000);  // 30 seconds
        component_timeouts.insert("transceiver".to_string(), 5000);  // 5 seconds
        component_timeouts.insert("power".to_string(), 1000);  // 1 second
        component_timeouts.insert("communication".to_string(), 10000);  // 10 seconds

        let mut error_rate_thresholds = HashMap::new();
        error_rate_thresholds.insert("gps".to_string(), 0.1);  // 10% error rate
        error_rate_thresholds.insert("transceiver".to_string(), 0.05);  // 5% error rate
        error_rate_thresholds.insert("power".to_string(), 0.01);  // 1% error rate
        error_rate_thresholds.insert("communication".to_string(), 0.2);  // 20% error rate

        let mut health_check_intervals = HashMap::new();
        health_check_intervals.insert("gps".to_string(), 60000);  // 1 minute
        health_check_intervals.insert("transceiver".to_string(), 30000);  // 30 seconds
        health_check_intervals.insert("power".to_string(), 10000);  // 10 seconds
        health_check_intervals.insert("communication".to_string(), 120000);  // 2 minutes

        Self {
            diagnostic_interval_ms: 30000,  // 30 seconds
            fault_detection_enabled: true,
            auto_recovery_enabled: true,
            max_recovery_attempts: 3,
            component_timeouts,
            error_rate_thresholds,
            health_check_intervals,
        }
    }
}

/// Hardware monitoring statistics
#[derive(Debug, Clone, Default)]
pub struct HardwareMonitorStats {
    pub total_diagnostics: u64,
    pub faults_detected: u32,
    pub recovery_attempts: u32,
    pub successful_recoveries: u32,
    pub failed_recoveries: u32,
    pub component_uptime: HashMap<String, Duration>,
    pub component_error_counts: HashMap<String, u32>,
    pub last_diagnostic_time: Option<SystemTime>,
    pub average_diagnostic_time_ms: f64,
}

/// Main hardware monitoring and fault detection system
pub struct HardwareMonitor {
    config: HardwareMonitorConfig,
    diagnostic_results: HashMap<HardwareComponent, DiagnosticResult>,
    recovery_strategies: HashMap<HardwareComponent, RecoveryStrategy>,
    fault_history: VecDeque<(SystemTime, HardwareComponent, HardwareFaultType)>,
    recovery_history: VecDeque<RecoveryAttempt>,
    statistics: HardwareMonitorStats,
    component_start_times: HashMap<HardwareComponent, SystemTime>,
    last_health_checks: HashMap<HardwareComponent, SystemTime>,
    monitoring_active: bool,
}

/// Recovery attempt record
#[derive(Debug, Clone)]
pub struct RecoveryAttempt {
    pub timestamp: SystemTime,
    pub component: HardwareComponent,
    pub fault_type: HardwareFaultType,
    pub strategy_used: String,
    pub steps_completed: u32,
    pub success: bool,
    pub duration: Duration,
    pub error_message: Option<String>,
}

impl HardwareMonitor {
    /// Create new hardware monitor
    pub fn new(config: HardwareMonitorConfig) -> Self {
        let mut recovery_strategies = HashMap::new();
        
        // GPS recovery strategy
        recovery_strategies.insert(HardwareComponent::GpsReceiver, RecoveryStrategy {
            component: HardwareComponent::GpsReceiver,
            steps: vec![
                RecoveryStep::SoftReset,
                RecoveryStep::WaitForStabilization { duration_ms: 5000 },
                RecoveryStep::RunSelfTest { test_name: "gps_acquisition".to_string() },
                RecoveryStep::Recalibrate,
                RecoveryStep::VerifyOperation,
            ],
            max_attempts: 3,
            timeout_per_step: Duration::from_secs(30),
            success_criteria: SuccessCriteria {
                response_time_threshold_ms: 30000,
                error_rate_threshold: 0.1,
                required_operations: vec!["position_fix".to_string()],
            },
        });

        // Transceiver recovery strategy
        recovery_strategies.insert(HardwareComponent::Transceiver, RecoveryStrategy {
            component: HardwareComponent::Transceiver,
            steps: vec![
                RecoveryStep::SoftReset,
                RecoveryStep::ReloadConfiguration,
                RecoveryStep::RunSelfTest { test_name: "transmission_test".to_string() },
                RecoveryStep::VerifyOperation,
            ],
            max_attempts: 3,
            timeout_per_step: Duration::from_secs(10),
            success_criteria: SuccessCriteria {
                response_time_threshold_ms: 5000,
                error_rate_threshold: 0.05,
                required_operations: vec!["transmit_message".to_string()],
            },
        });

        // Power management recovery strategy
        recovery_strategies.insert(HardwareComponent::PowerManagement, RecoveryStrategy {
            component: HardwareComponent::PowerManagement,
            steps: vec![
                RecoveryStep::Recalibrate,
                RecoveryStep::RunSelfTest { test_name: "battery_monitor".to_string() },
                RecoveryStep::VerifyOperation,
            ],
            max_attempts: 2,
            timeout_per_step: Duration::from_secs(5),
            success_criteria: SuccessCriteria {
                response_time_threshold_ms: 1000,
                error_rate_threshold: 0.01,
                required_operations: vec!["battery_status".to_string()],
            },
        });

        // Communication module recovery strategy
        recovery_strategies.insert(HardwareComponent::CommunicationModule, RecoveryStrategy {
            component: HardwareComponent::CommunicationModule,
            steps: vec![
                RecoveryStep::SoftReset,
                RecoveryStep::WaitForStabilization { duration_ms: 10000 },
                RecoveryStep::ReloadConfiguration,
                RecoveryStep::RunSelfTest { test_name: "connectivity_test".to_string() },
                RecoveryStep::VerifyOperation,
            ],
            max_attempts: 3,
            timeout_per_step: Duration::from_secs(30),
            success_criteria: SuccessCriteria {
                response_time_threshold_ms: 10000,
                error_rate_threshold: 0.2,
                required_operations: vec!["connect".to_string(), "send_data".to_string()],
            },
        });

        Self {
            config,
            diagnostic_results: HashMap::new(),
            recovery_strategies,
            fault_history: VecDeque::with_capacity(100),
            recovery_history: VecDeque::with_capacity(50),
            statistics: HardwareMonitorStats::default(),
            component_start_times: HashMap::new(),
            last_health_checks: HashMap::new(),
            monitoring_active: false,
        }
    }

    /// Start hardware monitoring
    pub fn start_monitoring(&mut self) {
        if self.monitoring_active {
            return;
        }

        let now = SystemTime::now();
        
        // Initialize component start times
        self.component_start_times.insert(HardwareComponent::GpsReceiver, now);
        self.component_start_times.insert(HardwareComponent::Transceiver, now);
        self.component_start_times.insert(HardwareComponent::PowerManagement, now);
        self.component_start_times.insert(HardwareComponent::CommunicationModule, now);

        self.monitoring_active = true;
    }

    /// Stop hardware monitoring
    pub fn stop_monitoring(&mut self) {
        if !self.monitoring_active {
            return;
        }

        // Update component uptime statistics
        let now = SystemTime::now();
        for (component, start_time) in &self.component_start_times {
            if let Ok(uptime) = now.duration_since(*start_time) {
                let component_name = format!("{:?}", component);
                *self.statistics.component_uptime.entry(component_name).or_insert(Duration::from_secs(0)) += uptime;
            }
        }

        self.monitoring_active = false;
    }

    /// Run comprehensive hardware diagnostics
    pub fn run_diagnostics<G, P, C, T>(&mut self, 
        gps_manager: &mut G,
        power_manager: &mut P,
        communication_manager: &mut C,
        transceiver: &mut T,
    ) -> Result<Vec<DiagnosticResult>, HardwareFaultError>
    where
        G: GpsManager,
        P: PowerManager,
        C: CommunicationManager,
        T: TransceiverInterface,
    {
        if !self.monitoring_active {
            return Err(HardwareFaultError::DiagnosticFailed {
                component: "system".to_string(),
                reason: "Monitoring not active".to_string(),
            });
        }

        let start_time = Instant::now();
        let mut results = Vec::new();

        // Diagnose GPS
        if let Ok(gps_result) = self.diagnose_gps(gps_manager) {
            results.push(gps_result);
        }

        // Diagnose Power Management
        if let Ok(power_result) = self.diagnose_power_management(power_manager) {
            results.push(power_result);
        }

        // Diagnose Communication
        if let Ok(comm_result) = self.diagnose_communication(communication_manager) {
            results.push(comm_result);
        }

        // Diagnose Transceiver
        if let Ok(transceiver_result) = self.diagnose_transceiver(transceiver) {
            results.push(transceiver_result);
        }

        // Update statistics
        self.statistics.total_diagnostics += 1;
        let diagnostic_duration = start_time.elapsed().as_millis() as f64;
        
        if self.statistics.total_diagnostics == 1 {
            self.statistics.average_diagnostic_time_ms = diagnostic_duration;
        } else {
            let total_time = self.statistics.average_diagnostic_time_ms * (self.statistics.total_diagnostics - 1) as f64;
            self.statistics.average_diagnostic_time_ms = (total_time + diagnostic_duration) / self.statistics.total_diagnostics as f64;
        }

        self.statistics.last_diagnostic_time = Some(SystemTime::now());

        // Store diagnostic results
        for result in &results {
            self.diagnostic_results.insert(result.component.clone(), result.clone());
            
            // Check for faults
            if matches!(result.health, ComponentHealth::Faulty { .. } | ComponentHealth::Failed) {
                self.statistics.faults_detected += 1;
                
                // Record fault in history
                let fault_type = match result.health {
                    ComponentHealth::Faulty { .. } => HardwareFaultType::ComponentFailure,
                    ComponentHealth::Failed => HardwareFaultType::ComponentFailure,
                    _ => HardwareFaultType::ComponentFailure,
                };
                
                self.fault_history.push_back((SystemTime::now(), result.component.clone(), fault_type));
                if self.fault_history.len() > 100 {
                    self.fault_history.pop_front();
                }

                // Attempt recovery if enabled
                if self.config.auto_recovery_enabled {
                    if let Err(e) = self.attempt_recovery(result.component.clone()) {
                        eprintln!("Recovery failed for {:?}: {}", result.component, e);
                    }
                }
            }
        }

        Ok(results)
    }

    /// Diagnose GPS receiver
    fn diagnose_gps<G>(&mut self, gps_manager: &mut G) -> Result<DiagnosticResult, HardwareFaultError>
    where
        G: GpsManager,
    {
        let start_time = Instant::now();
        let component = HardwareComponent::GpsReceiver;

        // Check GPS status
        let gps_status = gps_manager.get_status();
        let position = gps_manager.get_current_position();
        let satellite_count = gps_manager.get_satellite_count();

        let health = match gps_status {
            crate::GpsStatus::Locked => {
                if satellite_count >= 4 {
                    ComponentHealth::Healthy
                } else {
                    ComponentHealth::Degraded { performance_impact: 0.3 }
                }
            }
            crate::GpsStatus::Acquiring => ComponentHealth::Degraded { performance_impact: 0.5 },
            crate::GpsStatus::SignalLost => ComponentHealth::Faulty { 
                fault_description: "GPS signal lost".to_string() 
            },
            crate::GpsStatus::HardwareFault => ComponentHealth::Failed,
            _ => ComponentHealth::Unknown,
        };

        let response_time = start_time.elapsed().as_millis() as u64;
        
        // Calculate error rate based on recent GPS performance
        let error_rate = if position.is_some() { 0.0 } else { 1.0 };

        let recommended_action = match health {
            ComponentHealth::Healthy => RecommendedAction::None,
            ComponentHealth::Degraded { .. } => RecommendedAction::Monitor,
            ComponentHealth::Faulty { .. } => RecommendedAction::SoftReset,
            ComponentHealth::Failed => RecommendedAction::HardReset,
            ComponentHealth::Unknown => RecommendedAction::Recalibrate,
        };

        Ok(DiagnosticResult {
            component,
            health,
            response_time_ms: Some(response_time),
            error_rate,
            last_successful_operation: position.map(|_| SystemTime::now()),
            diagnostic_data: vec![satellite_count],
            recommended_action,
            timestamp: SystemTime::now(),
        })
    }

    /// Diagnose power management system
    fn diagnose_power_management<P>(&mut self, power_manager: &mut P) -> Result<DiagnosticResult, HardwareFaultError>
    where
        P: PowerManager,
    {
        let start_time = Instant::now();
        let component = HardwareComponent::PowerManagement;

        // Check power status
        let battery_result = power_manager.get_battery_status();
        let charging_result = power_manager.get_charging_status();

        let health = match (&battery_result, &charging_result) {
            (Ok(battery), Ok(_charging)) => {
                if battery.voltage_v > 0.0 && battery.capacity_percent >= 0.0 {
                    ComponentHealth::Healthy
                } else {
                    ComponentHealth::Faulty { 
                        fault_description: "Invalid battery readings".to_string() 
                    }
                }
            }
            _ => ComponentHealth::Failed,
        };

        let response_time = start_time.elapsed().as_millis() as u64;
        let error_rate = if battery_result.is_ok() && charging_result.is_ok() { 0.0 } else { 1.0 };

        let recommended_action = match health {
            ComponentHealth::Healthy => RecommendedAction::None,
            ComponentHealth::Degraded { .. } => RecommendedAction::Recalibrate,
            ComponentHealth::Faulty { .. } => RecommendedAction::Recalibrate,
            ComponentHealth::Failed => RecommendedAction::PowerCycle,
            ComponentHealth::Unknown => RecommendedAction::Monitor,
        };

        Ok(DiagnosticResult {
            component,
            health,
            response_time_ms: Some(response_time),
            error_rate,
            last_successful_operation: if battery_result.is_ok() { Some(SystemTime::now()) } else { None },
            diagnostic_data: vec![],
            recommended_action,
            timestamp: SystemTime::now(),
        })
    }

    /// Diagnose communication module
    fn diagnose_communication<C>(&mut self, communication_manager: &mut C) -> Result<DiagnosticResult, HardwareFaultError>
    where
        C: CommunicationManager,
    {
        let start_time = Instant::now();
        let component = HardwareComponent::CommunicationModule;

        // Check communication status
        let is_connected = communication_manager.is_connected();
        let signal_strength = communication_manager.get_signal_strength();

        let health = if is_connected {
            match signal_strength {
                Some(strength) if strength > 50 => ComponentHealth::Healthy,
                Some(strength) if strength > 20 => ComponentHealth::Degraded { performance_impact: 0.3 },
                Some(_) => ComponentHealth::Degraded { performance_impact: 0.6 },
                None => ComponentHealth::Faulty { 
                    fault_description: "No signal strength reading".to_string() 
                },
            }
        } else {
            ComponentHealth::Faulty { 
                fault_description: "Not connected".to_string() 
            }
        };

        let response_time = start_time.elapsed().as_millis() as u64;
        let error_rate = if is_connected { 0.0 } else { 1.0 };

        let recommended_action = match health {
            ComponentHealth::Healthy => RecommendedAction::None,
            ComponentHealth::Degraded { .. } => RecommendedAction::Monitor,
            ComponentHealth::Faulty { .. } => RecommendedAction::SoftReset,
            ComponentHealth::Failed => RecommendedAction::HardReset,
            ComponentHealth::Unknown => RecommendedAction::Monitor,
        };

        Ok(DiagnosticResult {
            component,
            health,
            response_time_ms: Some(response_time),
            error_rate,
            last_successful_operation: if is_connected { Some(SystemTime::now()) } else { None },
            diagnostic_data: signal_strength.map(|s| vec![s]).unwrap_or_default(),
            recommended_action,
            timestamp: SystemTime::now(),
        })
    }

    /// Diagnose transceiver
    fn diagnose_transceiver<T>(&mut self, transceiver: &mut T) -> Result<DiagnosticResult, HardwareFaultError>
    where
        T: TransceiverInterface,
    {
        let start_time = Instant::now();
        let component = HardwareComponent::Transceiver;

        // Test transceiver by attempting to set power level
        let power_test_result = transceiver.set_transmission_power(128);

        let health = match power_test_result {
            Ok(()) => ComponentHealth::Healthy,
            Err(_) => ComponentHealth::Faulty { 
                fault_description: "Power control failed".to_string() 
            },
        };

        let response_time = start_time.elapsed().as_millis() as u64;
        let error_rate = if power_test_result.is_ok() { 0.0 } else { 1.0 };

        let recommended_action = match health {
            ComponentHealth::Healthy => RecommendedAction::None,
            ComponentHealth::Degraded { .. } => RecommendedAction::Recalibrate,
            ComponentHealth::Faulty { .. } => RecommendedAction::SoftReset,
            ComponentHealth::Failed => RecommendedAction::HardReset,
            ComponentHealth::Unknown => RecommendedAction::Monitor,
        };

        Ok(DiagnosticResult {
            component,
            health,
            response_time_ms: Some(response_time),
            error_rate,
            last_successful_operation: if power_test_result.is_ok() { Some(SystemTime::now()) } else { None },
            diagnostic_data: vec![],
            recommended_action,
            timestamp: SystemTime::now(),
        })
    }

    /// Attempt hardware recovery for a component
    pub fn attempt_recovery(&mut self, component: HardwareComponent) -> Result<(), HardwareFaultError> {
        if !self.config.auto_recovery_enabled {
            return Err(HardwareFaultError::RecoveryFailed {
                component: format!("{:?}", component),
                attempts: 0,
            });
        }

        let strategy = self.recovery_strategies.get(&component)
            .ok_or_else(|| HardwareFaultError::RecoveryFailed {
                component: format!("{:?}", component),
                attempts: 0,
            })?
            .clone();

        let start_time = SystemTime::now();
        let mut attempts = 0;
        let mut success = false;
        let mut error_message = None;

        while attempts < strategy.max_attempts && !success {
            attempts += 1;
            self.statistics.recovery_attempts += 1;

            match self.execute_recovery_strategy(&strategy) {
                Ok(()) => {
                    success = true;
                    self.statistics.successful_recoveries += 1;
                }
                Err(e) => {
                    error_message = Some(e.to_string());
                    if attempts >= strategy.max_attempts {
                        self.statistics.failed_recoveries += 1;
                    }
                }
            }
        }

        // Record recovery attempt
        let recovery_attempt = RecoveryAttempt {
            timestamp: start_time,
            component: component.clone(),
            fault_type: HardwareFaultType::ComponentFailure, // Simplified for now
            strategy_used: format!("{:?}_recovery", component),
            steps_completed: attempts,
            success,
            duration: SystemTime::now().duration_since(start_time).unwrap_or(Duration::from_secs(0)),
            error_message,
        };

        self.recovery_history.push_back(recovery_attempt);
        if self.recovery_history.len() > 50 {
            self.recovery_history.pop_front();
        }

        if success {
            Ok(())
        } else {
            Err(HardwareFaultError::RecoveryFailed {
                component: format!("{:?}", component),
                attempts,
            })
        }
    }

    /// Execute recovery strategy steps
    fn execute_recovery_strategy(&self, strategy: &RecoveryStrategy) -> Result<(), HardwareFaultError> {
        for step in &strategy.steps {
            match step {
                RecoveryStep::SoftReset => {
                    // Simulate soft reset
                    std::thread::sleep(Duration::from_millis(100));
                }
                RecoveryStep::HardReset => {
                    // Simulate hard reset
                    std::thread::sleep(Duration::from_millis(500));
                }
                RecoveryStep::PowerCycle { duration_ms } => {
                    std::thread::sleep(Duration::from_millis(*duration_ms));
                }
                RecoveryStep::Recalibrate => {
                    // Simulate recalibration
                    std::thread::sleep(Duration::from_millis(1000));
                }
                RecoveryStep::ReloadConfiguration => {
                    // Simulate configuration reload
                    std::thread::sleep(Duration::from_millis(200));
                }
                RecoveryStep::RunSelfTest { test_name: _ } => {
                    // Simulate self-test
                    std::thread::sleep(Duration::from_millis(2000));
                }
                RecoveryStep::WaitForStabilization { duration_ms } => {
                    std::thread::sleep(Duration::from_millis(*duration_ms));
                }
                RecoveryStep::VerifyOperation => {
                    // Simulate operation verification
                    std::thread::sleep(Duration::from_millis(500));
                }
            }
        }

        Ok(())
    }

    /// Get hardware monitoring statistics
    pub fn get_statistics(&self) -> &HardwareMonitorStats {
        &self.statistics
    }

    /// Get recent diagnostic results
    pub fn get_diagnostic_results(&self) -> &HashMap<HardwareComponent, DiagnosticResult> {
        &self.diagnostic_results
    }

    /// Get recent fault history
    pub fn get_fault_history(&self, limit: usize) -> Vec<(SystemTime, HardwareComponent, HardwareFaultType)> {
        self.fault_history.iter()
            .rev()
            .take(limit)
            .cloned()
            .collect()
    }

    /// Get recent recovery history
    pub fn get_recovery_history(&self, limit: usize) -> Vec<RecoveryAttempt> {
        self.recovery_history.iter()
            .rev()
            .take(limit)
            .cloned()
            .collect()
    }

    /// Check if monitoring is active
    pub fn is_monitoring_active(&self) -> bool {
        self.monitoring_active
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hardware_monitor_creation() {
        let config = HardwareMonitorConfig::default();
        let monitor = HardwareMonitor::new(config);
        
        assert!(!monitor.is_monitoring_active());
        assert_eq!(monitor.get_statistics().total_diagnostics, 0);
    }

    #[test]
    fn test_recovery_strategy_creation() {
        let config = HardwareMonitorConfig::default();
        let monitor = HardwareMonitor::new(config);
        
        // Check that recovery strategies are created for all components
        assert!(monitor.recovery_strategies.contains_key(&HardwareComponent::GpsReceiver));
        assert!(monitor.recovery_strategies.contains_key(&HardwareComponent::Transceiver));
        assert!(monitor.recovery_strategies.contains_key(&HardwareComponent::PowerManagement));
        assert!(monitor.recovery_strategies.contains_key(&HardwareComponent::CommunicationModule));
    }
}