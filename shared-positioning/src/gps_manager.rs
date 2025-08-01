use std::time::{Duration, SystemTime, Instant};
use std::collections::{HashMap, VecDeque};
use serde::{Deserialize, Serialize};
use nalgebra::Vector3;

/// GNSS constellation types
#[derive(Debug, Clone, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum GnssConstellation {
    GPS,
    GLONASS,
    Galileo,
    BeiDou,
    QZSS,
}

/// Satellite information for constellation management
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SatelliteInfo {
    pub constellation: GnssConstellation,
    pub satellite_id: u8,
    pub elevation: f32,
    pub azimuth: f32,
    pub signal_strength: f32,
    pub used_in_fix: bool,
}

/// GPS position data with accuracy and timing information
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GpsPosition {
    pub latitude: f64,
    pub longitude: f64,
    pub altitude: f64,
    pub timestamp: SystemTime,
    pub accuracy_m: f32,
    pub satellite_count: u8,
    pub velocity: Option<Vector3<f64>>, // m/s in NED frame
    pub satellites: Vec<SatelliteInfo>,
    pub hdop: f32, // Horizontal dilution of precision
    pub vdop: f32, // Vertical dilution of precision
    pub quality_score: f32, // 0.0 to 1.0
}

/// Dead reckoning state for GPS-denied navigation
#[derive(Debug, Clone, PartialEq)]
pub struct DeadReckoningState {
    pub last_known_position: GpsPosition,
    pub estimated_position: GpsPosition,
    pub velocity: Vector3<f64>, // m/s in NED frame
    pub acceleration: Vector3<f64>, // m/s² in NED frame
    pub confidence: f32, // 0.0 to 1.0, decreases over time
    pub time_since_last_fix: Duration,
}

/// Position prediction parameters
#[derive(Debug, Clone, PartialEq)]
pub struct PositionPrediction {
    pub predicted_position: GpsPosition,
    pub confidence: f32,
    pub prediction_horizon: Duration,
    pub uncertainty_ellipse: (f32, f32, f32), // semi-major, semi-minor, orientation
}

/// GPS spoofing detection metrics
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct SpoofingDetectionMetrics {
    pub signal_power_anomaly: f32,
    pub timing_inconsistency: f32,
    pub position_jump_magnitude: f32,
    pub satellite_geometry_score: f32,
    pub spoofing_probability: f32, // 0.0 to 1.0
    pub detection_confidence: f32,
}

/// GPS configuration parameters
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct GpsConfig {
    pub acquisition_timeout_s: u32,
    pub update_interval_s: u32,
    pub min_satellite_count: u8,
    pub accuracy_threshold_m: f32,
    pub cold_start_timeout_s: u32,
    pub enabled_constellations: Vec<GnssConstellation>,
    pub constellation_priorities: HashMap<GnssConstellation, f32>,
    pub min_elevation_deg: f32,
    pub max_hdop: f32,
    pub max_vdop: f32,
    pub spoofing_detection_enabled: bool,
    pub dead_reckoning_enabled: bool,
    pub position_prediction_enabled: bool,
    pub quality_scoring_enabled: bool,
}

impl Default for GpsConfig {
    fn default() -> Self {
        let mut constellation_priorities = HashMap::new();
        constellation_priorities.insert(GnssConstellation::GPS, 1.0);
        constellation_priorities.insert(GnssConstellation::GLONASS, 0.8);
        constellation_priorities.insert(GnssConstellation::Galileo, 0.9);
        constellation_priorities.insert(GnssConstellation::BeiDou, 0.7);
        constellation_priorities.insert(GnssConstellation::QZSS, 0.6);
        
        Self {
            acquisition_timeout_s: 60,
            update_interval_s: 5,
            min_satellite_count: 4,
            accuracy_threshold_m: 10.0,
            cold_start_timeout_s: 120,
            enabled_constellations: vec![
                GnssConstellation::GPS,
                GnssConstellation::GLONASS,
                GnssConstellation::Galileo,
            ],
            constellation_priorities,
            min_elevation_deg: 10.0,
            max_hdop: 5.0,
            max_vdop: 8.0,
            spoofing_detection_enabled: true,
            dead_reckoning_enabled: true,
            position_prediction_enabled: true,
            quality_scoring_enabled: true,
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
    SpoofingDetected { confidence: f32 },
    ConstellationUnavailable { constellation: GnssConstellation },
    DeadReckoningFailed,
    PredictionFailed,
    QualityTooLow { score: f32, threshold: f32 },
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
            GpsError::SpoofingDetected { confidence } => {
                write!(f, "GPS spoofing detected with confidence: {:.2}", confidence)
            }
            GpsError::ConstellationUnavailable { constellation } => {
                write!(f, "GNSS constellation unavailable: {:?}", constellation)
            }
            GpsError::DeadReckoningFailed => write!(f, "Dead reckoning navigation failed"),
            GpsError::PredictionFailed => write!(f, "Position prediction failed"),
            GpsError::QualityTooLow { score, threshold } => {
                write!(f, "Position quality too low: {:.2} (required: {:.2})", score, threshold)
            }
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
    
    // Advanced GPS features
    
    /// Get available satellites for constellation optimization
    fn get_available_satellites(&self) -> Vec<SatelliteInfo>;
    
    /// Optimize constellation selection based on geometry and signal strength
    fn optimize_constellation_selection(&mut self) -> Result<(), GpsError>;
    
    /// Get dead reckoning state when GPS is unavailable
    fn get_dead_reckoning_state(&self) -> Option<DeadReckoningState>;
    
    /// Update dead reckoning with motion sensors (accelerometer, gyroscope)
    fn update_dead_reckoning(&mut self, acceleration: Vector3<f64>, angular_velocity: Vector3<f64>) -> Result<(), GpsError>;
    
    /// Predict future position based on current trajectory
    fn predict_position(&self, time_horizon: Duration) -> Result<PositionPrediction, GpsError>;
    
    /// Interpolate position between two known positions
    fn interpolate_position(&self, pos1: &GpsPosition, pos2: &GpsPosition, interpolation_time: SystemTime) -> Result<GpsPosition, GpsError>;
    
    /// Detect GPS spoofing attempts
    fn detect_spoofing(&mut self) -> Result<SpoofingDetectionMetrics, GpsError>;
    
    /// Calculate position quality score
    fn calculate_quality_score(&self, position: &GpsPosition) -> f32;
    
    /// Validate position accuracy and quality
    fn validate_position_quality(&self, position: &GpsPosition) -> Result<bool, GpsError>;
    
    /// Get multi-GNSS constellation status
    fn get_constellation_status(&self) -> HashMap<GnssConstellation, bool>;
    
    /// Enable/disable specific GNSS constellations
    fn configure_constellations(&mut self, enabled_constellations: Vec<GnssConstellation>) -> Result<(), GpsError>;
}

/// Advanced GPS manager implementation with multi-GNSS and advanced features
pub struct AdvancedGpsManager {
    config: GpsConfig,
    current_position: Option<GpsPosition>,
    status: GpsStatus,
    acquisition_start: Option<Instant>,
    last_update: Option<Instant>,
    signal_lost_time: Option<Instant>,
    satellite_count: u8,
    
    // Advanced features
    available_satellites: Vec<SatelliteInfo>,
    position_history: VecDeque<GpsPosition>,
    dead_reckoning_state: Option<DeadReckoningState>,
    spoofing_metrics: SpoofingDetectionMetrics,
    constellation_status: HashMap<GnssConstellation, bool>,
    quality_history: VecDeque<f32>,
    
    // Performance tracking
    constellation_performance: HashMap<GnssConstellation, f32>,
    last_spoofing_check: Option<Instant>,
    prediction_accuracy_history: VecDeque<f32>,
}

/// Basic GPS manager implementation (legacy compatibility)
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

impl AdvancedGpsManager {
    pub fn new(config: GpsConfig) -> Result<Self, GpsError> {
        // Validate configuration
        if config.acquisition_timeout_s == 0 || config.update_interval_s == 0 {
            return Err(GpsError::ConfigurationInvalid);
        }
        
        if config.min_satellite_count < 3 {
            return Err(GpsError::ConfigurationInvalid);
        }
        
        let mut constellation_status = HashMap::new();
        for constellation in &config.enabled_constellations {
            constellation_status.insert(constellation.clone(), true);
        }
        
        let mut constellation_performance = HashMap::new();
        for constellation in &config.enabled_constellations {
            constellation_performance.insert(constellation.clone(), 0.5); // Start with neutral performance
        }
        
        Ok(Self {
            config,
            current_position: None,
            status: GpsStatus::Initializing,
            acquisition_start: None,
            last_update: None,
            signal_lost_time: None,
            satellite_count: 0,
            available_satellites: Vec::new(),
            position_history: VecDeque::with_capacity(100),
            dead_reckoning_state: None,
            spoofing_metrics: SpoofingDetectionMetrics {
                signal_power_anomaly: 0.0,
                timing_inconsistency: 0.0,
                position_jump_magnitude: 0.0,
                satellite_geometry_score: 0.0,
                spoofing_probability: 0.0,
                detection_confidence: 0.0,
            },
            constellation_status,
            quality_history: VecDeque::with_capacity(50),
            constellation_performance,
            last_spoofing_check: None,
            prediction_accuracy_history: VecDeque::with_capacity(20),
        })
    }
    
    /// Optimize satellite selection based on geometry and signal strength
    fn optimize_satellite_selection(&mut self) -> Vec<SatelliteInfo> {
        let mut selected_satellites = Vec::new();
        
        // Filter satellites by elevation and enabled constellations
        let mut candidates: Vec<_> = self.available_satellites.iter()
            .filter(|sat| {
                sat.elevation >= self.config.min_elevation_deg &&
                self.config.enabled_constellations.contains(&sat.constellation)
            })
            .cloned()
            .collect();
        
        // Sort by signal strength and constellation priority
        candidates.sort_by(|a, b| {
            let priority_a = self.config.constellation_priorities.get(&a.constellation).unwrap_or(&0.5);
            let priority_b = self.config.constellation_priorities.get(&b.constellation).unwrap_or(&0.5);
            let score_a = a.signal_strength * priority_a;
            let score_b = b.signal_strength * priority_b;
            score_b.partial_cmp(&score_a).unwrap_or(std::cmp::Ordering::Equal)
        });
        
        // Select best satellites ensuring good geometric diversity
        let mut constellation_counts = HashMap::new();
        for satellite in candidates {
            let constellation_count = constellation_counts.get(&satellite.constellation).unwrap_or(&0);
            
            // Limit satellites per constellation for diversity
            if *constellation_count < 4 && selected_satellites.len() < 12 {
                selected_satellites.push(satellite.clone());
                constellation_counts.insert(satellite.constellation.clone(), constellation_count + 1);
            }
        }
        
        selected_satellites
    }
    
    /// Calculate geometric dilution of precision (GDOP)
    fn calculate_gdop(&self, satellites: &[SatelliteInfo]) -> (f32, f32) {
        if satellites.len() < 4 {
            return (99.0, 99.0); // Invalid GDOP
        }
        
        // Simplified GDOP calculation based on satellite geometry
        let mut elevation_sum = 0.0;
        let mut azimuth_spread = 0.0;
        
        for satellite in satellites {
            elevation_sum += satellite.elevation;
        }
        
        let avg_elevation = elevation_sum / satellites.len() as f32;
        
        // Calculate azimuth spread (simplified)
        if satellites.len() >= 4 {
            let mut azimuths: Vec<f32> = satellites.iter().map(|s| s.azimuth).collect();
            azimuths.sort_by(|a, b| a.partial_cmp(b).unwrap());
            
            let mut max_gap: f32 = 0.0;
            for i in 0..azimuths.len() {
                let next_idx = (i + 1) % azimuths.len();
                let gap = if next_idx == 0 {
                    360.0 - azimuths[i] + azimuths[next_idx]
                } else {
                    azimuths[next_idx] - azimuths[i]
                };
                max_gap = max_gap.max(gap);
            }
            azimuth_spread = 360.0 - max_gap;
        }
        
        // Calculate HDOP and VDOP based on geometry
        let hdop = if azimuth_spread > 180.0 && avg_elevation > 30.0 {
            1.0 + (90.0 - avg_elevation) / 90.0
        } else {
            2.0 + (180.0 - azimuth_spread) / 180.0 + (90.0 - avg_elevation) / 90.0
        };
        
        let vdop = hdop * 1.5; // VDOP is typically higher than HDOP
        
        (hdop.min(20.0), vdop.min(30.0))
    }
    
    /// Update dead reckoning state
    fn update_dead_reckoning_internal(&mut self, acceleration: Vector3<f64>, angular_velocity: Vector3<f64>) -> Result<(), GpsError> {
        if !self.config.dead_reckoning_enabled {
            return Ok(());
        }
        
        let current_time = SystemTime::now();
        
        if let Some(ref mut dr_state) = self.dead_reckoning_state {
            let dt = current_time.duration_since(dr_state.estimated_position.timestamp)
                .unwrap_or(Duration::from_millis(100)).as_secs_f64();
            
            // Update velocity with acceleration
            dr_state.velocity += acceleration * dt;
            
            // Update position with velocity
            let position_delta = dr_state.velocity * dt;
            
            // Convert local displacement to lat/lon (simplified)
            let lat_delta = position_delta.y / 111320.0; // meters to degrees latitude
            let lon_delta = position_delta.x / (111320.0 * dr_state.estimated_position.latitude.to_radians().cos());
            
            dr_state.estimated_position.latitude += lat_delta;
            dr_state.estimated_position.longitude += lon_delta;
            dr_state.estimated_position.altitude += position_delta.z;
            dr_state.estimated_position.timestamp = current_time;
            
            // Decrease confidence over time
            let time_factor = (dt / 60.0) as f32; // Decrease confidence over minutes
            dr_state.confidence = (dr_state.confidence - time_factor * 0.1).max(0.0);
            
            dr_state.time_since_last_fix += Duration::from_secs_f64(dt);
            
            // Update acceleration for next iteration
            dr_state.acceleration = acceleration;
        }
        
        Ok(())
    }
    
    /// Detect potential GPS spoofing using multiple detection algorithms
    fn detect_spoofing_internal(&mut self) -> SpoofingDetectionMetrics {
        let mut metrics = SpoofingDetectionMetrics {
            signal_power_anomaly: 0.0,
            timing_inconsistency: 0.0,
            position_jump_magnitude: 0.0,
            satellite_geometry_score: 0.0,
            spoofing_probability: 0.0,
            detection_confidence: 0.0,
        };
        
        if !self.config.spoofing_detection_enabled {
            return metrics;
        }
        
        // 1. Signal power anomaly detection
        metrics.signal_power_anomaly = self.detect_signal_power_anomaly();
        
        // 2. Position jump detection
        metrics.position_jump_magnitude = self.detect_position_jumps();
        
        // 3. Satellite geometry analysis
        metrics.satellite_geometry_score = self.analyze_satellite_geometry();
        
        // 4. Timing consistency check
        metrics.timing_inconsistency = self.check_timing_consistency();
        
        // Calculate overall spoofing probability using weighted combination
        metrics.spoofing_probability = (
            metrics.signal_power_anomaly * 0.25 +
            metrics.position_jump_magnitude * 0.35 +
            metrics.satellite_geometry_score * 0.25 +
            metrics.timing_inconsistency * 0.15
        ).min(1.0);
        
        // Calculate detection confidence based on available data and consistency
        metrics.detection_confidence = self.calculate_detection_confidence(&metrics);
        
        self.spoofing_metrics = metrics.clone();
        metrics
    }
    
    /// Detect signal power anomalies
    fn detect_signal_power_anomaly(&self) -> f32 {
        if self.available_satellites.is_empty() {
            return 0.0;
        }
        
        let signal_strengths: Vec<f32> = self.available_satellites.iter()
            .map(|s| s.signal_strength)
            .collect();
        
        let avg_signal = signal_strengths.iter().sum::<f32>() / signal_strengths.len() as f32;
        let variance = signal_strengths.iter()
            .map(|s| (s - avg_signal).powi(2))
            .sum::<f32>() / signal_strengths.len() as f32;
        let std_dev = variance.sqrt();
        
        let mut anomaly_score = 0.0;
        
        // Check for unusually high average signal strength
        if avg_signal > 50.0 {
            anomaly_score += (avg_signal - 50.0) / 50.0;
        }
        
        // Check for unusually low signal variation (all signals too similar)
        if std_dev < 2.0 && signal_strengths.len() > 4 {
            anomaly_score += (2.0 - std_dev) / 2.0;
        }
        
        // Check for signals that are too strong for their elevation
        for satellite in &self.available_satellites {
            let expected_strength = 25.0 + (satellite.elevation - 10.0) * 0.5;
            if satellite.signal_strength > expected_strength + 15.0 {
                anomaly_score += 0.2;
            }
        }
        
        anomaly_score.min(1.0)
    }
    
    /// Detect unrealistic position jumps
    fn detect_position_jumps(&self) -> f32 {
        if self.position_history.len() < 2 {
            return 0.0;
        }
        
        let current_pos = match &self.current_position {
            Some(pos) => pos,
            None => return 0.0,
        };
        
        let mut max_jump_score: f32 = 0.0;
        
        // Check against recent positions
        for prev_pos in self.position_history.iter().rev().take(3) {
            let time_diff = current_pos.timestamp.duration_since(prev_pos.timestamp)
                .unwrap_or(Duration::from_secs(1)).as_secs_f64();
            
            if time_diff > 0.0 && time_diff < 300.0 { // Within 5 minutes
                let lat_diff = (current_pos.latitude - prev_pos.latitude) * 111320.0;
                let lon_diff = (current_pos.longitude - prev_pos.longitude) * 111320.0 * current_pos.latitude.to_radians().cos();
                let distance = (lat_diff * lat_diff + lon_diff * lon_diff).sqrt();
                let speed = distance / time_diff;
                
                // Different thresholds for different time intervals
                let max_reasonable_speed = if time_diff < 10.0 {
                    50.0 // 50 m/s for very short intervals
                } else if time_diff < 60.0 {
                    30.0 // 30 m/s for short intervals
                } else {
                    15.0 // 15 m/s for longer intervals
                };
                
                if speed > max_reasonable_speed {
                    let jump_score = ((speed - max_reasonable_speed) / max_reasonable_speed).min(2.0);
                    max_jump_score = max_jump_score.max(jump_score as f32);
                }
            }
        }
        
        max_jump_score.min(1.0)
    }
    
    /// Analyze satellite geometry for spoofing indicators
    fn analyze_satellite_geometry(&self) -> f32 {
        if self.available_satellites.len() < 4 {
            return 0.0;
        }
        
        let (hdop, vdop) = self.calculate_gdop(&self.available_satellites);
        let mut geometry_score = 0.0;
        
        // Check for suspiciously good geometry (too perfect)
        if hdop < 0.8 && vdop < 1.2 {
            geometry_score += (0.8 - hdop) / 0.8 * 0.5;
            geometry_score += (1.2 - vdop) / 1.2 * 0.3;
        }
        
        // Check for unusual satellite distribution
        let mut azimuth_sectors = [0; 8]; // 8 sectors of 45 degrees each
        for satellite in &self.available_satellites {
            if satellite.elevation >= self.config.min_elevation_deg {
                let sector = ((satellite.azimuth / 45.0) as usize).min(7);
                azimuth_sectors[sector] += 1;
            }
        }
        
        // Check for clustering (too many satellites in one sector)
        let max_in_sector = *azimuth_sectors.iter().max().unwrap_or(&0);
        let total_satellites = azimuth_sectors.iter().sum::<i32>();
        
        if total_satellites > 0 && max_in_sector as f32 / total_satellites as f32 > 0.6 {
            geometry_score += 0.3;
        }
        
        // Check for constellation diversity anomalies
        let mut constellation_counts = std::collections::HashMap::new();
        for satellite in &self.available_satellites {
            *constellation_counts.entry(&satellite.constellation).or_insert(0) += 1;
        }
        
        // Suspicious if only one constellation is visible when multiple are enabled
        if constellation_counts.len() == 1 && self.config.enabled_constellations.len() > 1 {
            geometry_score += 0.2;
        }
        
        geometry_score.min(1.0)
    }
    
    /// Check timing consistency across measurements
    fn check_timing_consistency(&self) -> f32 {
        if self.position_history.len() < 3 {
            return 0.0;
        }
        
        let mut timing_score: f32 = 0.0;
        let positions: Vec<_> = self.position_history.iter().rev().take(5).collect();
        
        // Check for consistent time intervals
        let mut intervals = Vec::new();
        for i in 0..positions.len()-1 {
            let interval = positions[i].timestamp.duration_since(positions[i+1].timestamp)
                .unwrap_or(Duration::from_secs(0)).as_secs_f64();
            intervals.push(interval);
        }
        
        if !intervals.is_empty() {
            let avg_interval = intervals.iter().sum::<f64>() / intervals.len() as f64;
            let variance = intervals.iter()
                .map(|i| (i - avg_interval).powi(2))
                .sum::<f64>() / intervals.len() as f64;
            let std_dev = variance.sqrt();
            
            // Suspicious if intervals are too regular (exactly the same)
            if std_dev < 0.1 && avg_interval > 1.0 {
                timing_score += 0.3;
            }
            
            // Suspicious if there are sudden changes in update rate
            for interval in &intervals {
                if (*interval - avg_interval).abs() > avg_interval * 2.0 {
                    timing_score += 0.2;
                }
            }
        }
        
        timing_score.min(1.0)
    }
    
    /// Calculate detection confidence based on available data
    fn calculate_detection_confidence(&self, metrics: &SpoofingDetectionMetrics) -> f32 {
        let mut confidence = 0.0;
        
        // Base confidence on amount of historical data
        let history_factor = (self.position_history.len() as f32 / 10.0).min(1.0);
        confidence += history_factor * 0.3;
        
        // Confidence based on satellite count
        let satellite_factor = (self.available_satellites.len() as f32 / 8.0).min(1.0);
        confidence += satellite_factor * 0.2;
        
        // Confidence based on consistency of detection across metrics
        let detection_consistency = if metrics.spoofing_probability > 0.3 {
            let active_detectors = [
                metrics.signal_power_anomaly > 0.2,
                metrics.position_jump_magnitude > 0.2,
                metrics.satellite_geometry_score > 0.2,
                metrics.timing_inconsistency > 0.2,
            ].iter().filter(|&&x| x).count();
            
            (active_detectors as f32 / 4.0) * 0.5
        } else {
            0.3 // Lower confidence for low probability detections
        };
        
        confidence += detection_consistency;
        
        confidence.min(1.0).max(0.1)
    }
    
    /// Calculate comprehensive position quality score
    fn calculate_quality_score_internal(&self, position: &GpsPosition) -> f32 {
        if !self.config.quality_scoring_enabled {
            return 1.0;
        }
        
        let mut weighted_score = 0.0;
        
        // 1. Accuracy score (25% weight)
        let accuracy_score = if position.accuracy_m <= 1.0 {
            1.0
        } else if position.accuracy_m <= 3.0 {
            0.9
        } else if position.accuracy_m <= 5.0 {
            0.8
        } else if position.accuracy_m <= 10.0 {
            0.6
        } else if position.accuracy_m <= 20.0 {
            0.4
        } else {
            0.2
        };
        weighted_score += accuracy_score * 0.25;
        
        // 2. Satellite count score (20% weight)
        let sat_score = if position.satellite_count >= 12 {
            1.0
        } else if position.satellite_count >= 8 {
            0.9
        } else if position.satellite_count >= 6 {
            0.8
        } else if position.satellite_count >= 4 {
            0.6
        } else if position.satellite_count >= 3 {
            0.4
        } else {
            0.1
        };
        weighted_score += sat_score * 0.20;
        
        // 3. HDOP/VDOP score (20% weight)
        let dop_score = if position.hdop <= 1.0 && position.vdop <= 1.5 {
            1.0
        } else if position.hdop <= 2.0 && position.vdop <= 3.0 {
            0.9
        } else if position.hdop <= 3.0 && position.vdop <= 4.0 {
            0.8
        } else if position.hdop <= 5.0 && position.vdop <= 8.0 {
            0.6
        } else if position.hdop <= 10.0 && position.vdop <= 15.0 {
            0.4
        } else {
            0.2
        };
        weighted_score += dop_score * 0.20;
        
        // 4. Constellation diversity score (15% weight)
        let mut constellations_used = std::collections::HashSet::new();
        for satellite in &position.satellites {
            if satellite.used_in_fix {
                constellations_used.insert(&satellite.constellation);
            }
        }
        
        let diversity_score = match constellations_used.len() {
            0 => 0.0,
            1 => 0.5,
            2 => 0.7,
            3 => 0.9,
            _ => 1.0,
        };
        weighted_score += diversity_score * 0.15;
        
        // 5. Signal strength score (10% weight)
        let avg_signal_strength = if !position.satellites.is_empty() {
            position.satellites.iter()
                .filter(|s| s.used_in_fix)
                .map(|s| s.signal_strength)
                .sum::<f32>() / position.satellites.iter().filter(|s| s.used_in_fix).count().max(1) as f32
        } else {
            0.0
        };
        
        let signal_score = if avg_signal_strength >= 45.0 {
            1.0
        } else if avg_signal_strength >= 35.0 {
            0.8
        } else if avg_signal_strength >= 25.0 {
            0.6
        } else if avg_signal_strength >= 15.0 {
            0.4
        } else {
            0.2
        };
        weighted_score += signal_score * 0.10;
        
        // 6. Satellite elevation score (10% weight)
        let avg_elevation = if !position.satellites.is_empty() {
            position.satellites.iter()
                .filter(|s| s.used_in_fix)
                .map(|s| s.elevation)
                .sum::<f32>() / position.satellites.iter().filter(|s| s.used_in_fix).count().max(1) as f32
        } else {
            0.0
        };
        
        let elevation_score = if avg_elevation >= 45.0 {
            1.0
        } else if avg_elevation >= 30.0 {
            0.8
        } else if avg_elevation >= 20.0 {
            0.6
        } else if avg_elevation >= 10.0 {
            0.4
        } else {
            0.2
        };
        weighted_score += elevation_score * 0.10;
        
        // Apply consistency bonus if we have position history
        if self.position_history.len() >= 3 {
            let consistency_bonus = self.calculate_position_consistency_bonus(position);
            weighted_score *= 1.0 + consistency_bonus * 0.1; // Up to 10% bonus
        }
        
        // Apply spoofing penalty
        if self.spoofing_metrics.spoofing_probability > 0.3 {
            let spoofing_penalty = self.spoofing_metrics.spoofing_probability * 0.5;
            weighted_score *= 1.0 - spoofing_penalty;
        }
        
        weighted_score.max(0.0).min(1.0)
    }
    
    /// Calculate position consistency bonus based on historical data
    fn calculate_position_consistency_bonus(&self, current_position: &GpsPosition) -> f32 {
        if self.position_history.len() < 3 {
            return 0.0;
        }
        
        let recent_positions: Vec<_> = self.position_history.iter().rev().take(5).collect();
        let mut consistency_factors = Vec::new();
        
        // Check accuracy consistency
        let accuracies: Vec<f32> = recent_positions.iter().map(|p| p.accuracy_m).collect();
        let avg_accuracy = accuracies.iter().sum::<f32>() / accuracies.len() as f32;
        let accuracy_variance = accuracies.iter()
            .map(|a| (a - avg_accuracy).powi(2))
            .sum::<f32>() / accuracies.len() as f32;
        let accuracy_consistency = 1.0 / (1.0 + accuracy_variance.sqrt() / avg_accuracy);
        consistency_factors.push(accuracy_consistency);
        
        // Check satellite count consistency
        let sat_counts: Vec<u8> = recent_positions.iter().map(|p| p.satellite_count).collect();
        let avg_sat_count = sat_counts.iter().sum::<u8>() as f32 / sat_counts.len() as f32;
        let sat_count_variance = sat_counts.iter()
            .map(|c| (*c as f32 - avg_sat_count).powi(2))
            .sum::<f32>() / sat_counts.len() as f32;
        let sat_count_consistency = 1.0 / (1.0 + sat_count_variance.sqrt() / avg_sat_count);
        consistency_factors.push(sat_count_consistency);
        
        // Check HDOP consistency
        let hdops: Vec<f32> = recent_positions.iter().map(|p| p.hdop).collect();
        let avg_hdop = hdops.iter().sum::<f32>() / hdops.len() as f32;
        let hdop_variance = hdops.iter()
            .map(|h| (h - avg_hdop).powi(2))
            .sum::<f32>() / hdops.len() as f32;
        let hdop_consistency = 1.0 / (1.0 + hdop_variance.sqrt() / avg_hdop);
        consistency_factors.push(hdop_consistency);
        
        // Average consistency score
        let avg_consistency = consistency_factors.iter().sum::<f32>() / consistency_factors.len() as f32;
        
        // Convert to bonus (0.0 to 1.0)
        (avg_consistency - 0.5).max(0.0) * 2.0
    }
}

impl GpsManager for AdvancedGpsManager {
    fn start_acquisition(&mut self) -> Result<(), GpsError> {
        self.status = GpsStatus::Acquiring;
        self.acquisition_start = Some(Instant::now());
        self.signal_lost_time = None;
        
        // Initialize constellation status
        for constellation in &self.config.enabled_constellations {
            self.constellation_status.insert(constellation.clone(), true);
        }
        
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
        
        // Update constellation status based on new config
        self.constellation_status.clear();
        for constellation in &config.enabled_constellations {
            self.constellation_status.insert(constellation.clone(), true);
        }
        
        self.config = config;
        Ok(())
    }
    
    fn get_status(&self) -> GpsStatus {
        self.status.clone()
    }
    
    fn update(&mut self) -> Result<(), GpsError> {
        // Perform spoofing detection periodically
        if self.config.spoofing_detection_enabled {
            let should_check_spoofing = if let Some(last_check) = self.last_spoofing_check {
                last_check.elapsed() > Duration::from_secs(30) // Check every 30 seconds
            } else {
                true
            };
            
            if should_check_spoofing {
                let spoofing_metrics = self.detect_spoofing_internal();
                if spoofing_metrics.spoofing_probability > 0.7 {
                    return Err(GpsError::SpoofingDetected { 
                        confidence: spoofing_metrics.detection_confidence 
                    });
                }
                self.last_spoofing_check = Some(Instant::now());
            }
        }
        
        match self.status {
            GpsStatus::Initializing => {
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
                
                // Simulate GPS data acquisition (in real implementation, this would read from hardware)
                if let Some(position) = self.simulate_gps_acquisition()? {
                    if self.is_position_accurate(&position) {
                        // Calculate quality score
                        let quality_score = self.calculate_quality_score_internal(&position);
                        let mut enhanced_position = position.clone();
                        enhanced_position.quality_score = quality_score;
                        
                        self.current_position = Some(enhanced_position.clone());
                        self.satellite_count = enhanced_position.satellite_count;
                        self.status = GpsStatus::Locked;
                        self.last_update = Some(Instant::now());
                        self.signal_lost_time = None;
                        
                        // Add to position history
                        self.position_history.push_back(enhanced_position.clone());
                        if self.position_history.len() > 100 {
                            self.position_history.pop_front();
                        }
                        
                        // Initialize dead reckoning if enabled
                        if self.config.dead_reckoning_enabled && self.dead_reckoning_state.is_none() {
                            self.dead_reckoning_state = Some(DeadReckoningState {
                                last_known_position: enhanced_position.clone(),
                                estimated_position: enhanced_position.clone(),
                                velocity: Vector3::zeros(),
                                acceleration: Vector3::zeros(),
                                confidence: 1.0,
                                time_since_last_fix: Duration::from_secs(0),
                            });
                        }
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
                    if let Some(position) = self.simulate_gps_acquisition()? {
                        if self.is_position_accurate(&position) {
                            let quality_score = self.calculate_quality_score_internal(&position);
                            let mut enhanced_position = position.clone();
                            enhanced_position.quality_score = quality_score;
                            
                            self.current_position = Some(enhanced_position.clone());
                            self.satellite_count = enhanced_position.satellite_count;
                            self.last_update = Some(Instant::now());
                            self.signal_lost_time = None;
                            
                            // Update position history
                            self.position_history.push_back(enhanced_position.clone());
                            if self.position_history.len() > 100 {
                                self.position_history.pop_front();
                            }
                            
                            // Update dead reckoning state
                            if let Some(ref mut dr_state) = self.dead_reckoning_state {
                                dr_state.last_known_position = enhanced_position;
                                dr_state.confidence = 1.0;
                                dr_state.time_since_last_fix = Duration::from_secs(0);
                            }
                        } else {
                            // Signal quality degraded
                            if self.signal_lost_time.is_none() {
                                self.signal_lost_time = Some(Instant::now());
                            }
                            
                            if self.is_degraded_mode() {
                                self.status = GpsStatus::DegradedMode;
                            }
                        }
                    } else {
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
                Ok(())
            }
            GpsStatus::SignalLost => {
                // Try to reacquire signal
                if let Some(position) = self.simulate_gps_acquisition()? {
                    if self.is_position_accurate(&position) {
                        let quality_score = self.calculate_quality_score_internal(&position);
                        let mut enhanced_position = position.clone();
                        enhanced_position.quality_score = quality_score;
                        
                        self.current_position = Some(enhanced_position.clone());
                        self.satellite_count = enhanced_position.satellite_count;
                        self.status = GpsStatus::Locked;
                        self.last_update = Some(Instant::now());
                        self.signal_lost_time = None;
                        
                        self.position_history.push_back(enhanced_position);
                        if self.position_history.len() > 100 {
                            self.position_history.pop_front();
                        }
                    }
                } else {
                    if self.is_degraded_mode() {
                        self.status = GpsStatus::DegradedMode;
                    }
                }
                Ok(())
            }
            GpsStatus::DegradedMode => {
                // Continue trying to reacquire signal
                if let Some(position) = self.simulate_gps_acquisition()? {
                    if self.is_position_accurate(&position) {
                        let quality_score = self.calculate_quality_score_internal(&position);
                        let mut enhanced_position = position.clone();
                        enhanced_position.quality_score = quality_score;
                        
                        self.current_position = Some(enhanced_position.clone());
                        self.satellite_count = enhanced_position.satellite_count;
                        self.status = GpsStatus::Locked;
                        self.last_update = Some(Instant::now());
                        self.signal_lost_time = None;
                        
                        self.position_history.push_back(enhanced_position);
                        if self.position_history.len() > 100 {
                            self.position_history.pop_front();
                        }
                    }
                }
                Ok(())
            }
            GpsStatus::HardwareFault => {
                Err(GpsError::HardwareFault)
            }
        }
    }
    
    fn stop(&mut self) -> Result<(), GpsError> {
        self.status = GpsStatus::Initializing;
        self.acquisition_start = None;
        self.last_update = None;
        self.signal_lost_time = None;
        self.dead_reckoning_state = None;
        Ok(())
    }
    
    // Advanced GPS features implementation
    
    fn get_available_satellites(&self) -> Vec<SatelliteInfo> {
        self.available_satellites.clone()
    }
    
    fn optimize_constellation_selection(&mut self) -> Result<(), GpsError> {
        let optimized_satellites = self.optimize_satellite_selection();
        
        // Update available satellites with optimized selection
        for satellite in &mut self.available_satellites {
            satellite.used_in_fix = optimized_satellites.iter()
                .any(|opt_sat| opt_sat.satellite_id == satellite.satellite_id && 
                     opt_sat.constellation == satellite.constellation);
        }
        
        Ok(())
    }
    
    fn get_dead_reckoning_state(&self) -> Option<DeadReckoningState> {
        self.dead_reckoning_state.clone()
    }
    
    fn update_dead_reckoning(&mut self, acceleration: Vector3<f64>, angular_velocity: Vector3<f64>) -> Result<(), GpsError> {
        self.update_dead_reckoning_internal(acceleration, angular_velocity)
    }
    
    fn predict_position(&self, time_horizon: Duration) -> Result<PositionPrediction, GpsError> {
        if !self.config.position_prediction_enabled {
            return Err(GpsError::PredictionFailed);
        }
        
        let current_pos = self.current_position.as_ref()
            .ok_or(GpsError::PredictionFailed)?;
        
        // Use multiple position history points for better prediction
        if self.position_history.len() < 2 {
            return Err(GpsError::PredictionFailed);
        }
        
        let dt = time_horizon.as_secs_f64();
        
        // Calculate velocity and acceleration from position history
        let (velocity, acceleration) = self.calculate_motion_parameters();
        
        // Kinematic prediction with acceleration
        let position_delta = velocity * dt + acceleration * (dt * dt * 0.5);
        
        // Convert local displacement to lat/lon
        let lat_delta = position_delta.y / 111320.0; // meters to degrees latitude
        let lon_delta = position_delta.x / (111320.0 * current_pos.latitude.to_radians().cos());
        
        // Predict velocity at future time
        let predicted_velocity = velocity + acceleration * dt;
        
        let predicted_position = GpsPosition {
            latitude: current_pos.latitude + lat_delta,
            longitude: current_pos.longitude + lon_delta,
            altitude: current_pos.altitude + position_delta.z,
            timestamp: current_pos.timestamp + time_horizon,
            accuracy_m: current_pos.accuracy_m * (1.0 + dt as f32 * 0.1), // Accuracy degrades over time
            satellite_count: current_pos.satellite_count,
            velocity: Some(predicted_velocity),
            satellites: current_pos.satellites.clone(),
            hdop: current_pos.hdop * (1.0 + dt as f32 * 0.05),
            vdop: current_pos.vdop * (1.0 + dt as f32 * 0.05),
            quality_score: current_pos.quality_score * (1.0 - dt as f32 * 0.05).max(0.1),
        };
        
        // Calculate confidence based on time horizon, motion consistency, and current accuracy
        let motion_consistency = self.calculate_motion_consistency();
        let time_factor = (1.0 - (dt / 3600.0) as f32).max(0.1); // Decrease over hours
        let accuracy_factor = (10.0 / current_pos.accuracy_m).min(1.0);
        let confidence = motion_consistency * time_factor * accuracy_factor;
        
        // Calculate uncertainty ellipse based on motion and time
        let base_uncertainty = current_pos.accuracy_m * (1.0 + dt as f32 * 0.2);
        let motion_uncertainty = velocity.magnitude() as f32 * dt as f32 * 0.1;
        let total_uncertainty = base_uncertainty + motion_uncertainty;
        
        // Ellipse orientation based on velocity direction
        let velocity_angle = if velocity.magnitude() > 0.1 {
            velocity.y.atan2(velocity.x) as f32
        } else {
            0.0
        };
        
        let uncertainty_ellipse = (
            total_uncertainty,
            total_uncertainty * 0.6, // Minor axis
            velocity_angle,
        );
        
        Ok(PositionPrediction {
            predicted_position,
            confidence,
            prediction_horizon: time_horizon,
            uncertainty_ellipse,
        })
    }
    

    
    fn interpolate_position(&self, pos1: &GpsPosition, pos2: &GpsPosition, interpolation_time: SystemTime) -> Result<GpsPosition, GpsError> {
        let t1 = pos1.timestamp.duration_since(SystemTime::UNIX_EPOCH).unwrap().as_secs_f64();
        let t2 = pos2.timestamp.duration_since(SystemTime::UNIX_EPOCH).unwrap().as_secs_f64();
        let t_interp = interpolation_time.duration_since(SystemTime::UNIX_EPOCH).unwrap().as_secs_f64();
        
        if t_interp < t1 || t_interp > t2 || t1 >= t2 {
            return Err(GpsError::PredictionFailed);
        }
        
        let alpha = (t_interp - t1) / (t2 - t1);
        
        let interpolated_pos = GpsPosition {
            latitude: pos1.latitude + alpha * (pos2.latitude - pos1.latitude),
            longitude: pos1.longitude + alpha * (pos2.longitude - pos1.longitude),
            altitude: pos1.altitude + alpha * (pos2.altitude - pos1.altitude),
            timestamp: interpolation_time,
            accuracy_m: pos1.accuracy_m.max(pos2.accuracy_m) * (1.0 + alpha as f32 * 0.1),
            satellite_count: ((pos1.satellite_count as f32 + pos2.satellite_count as f32) / 2.0) as u8,
            velocity: match (&pos1.velocity, &pos2.velocity) {
                (Some(v1), Some(v2)) => Some(v1 + alpha * (v2 - v1)),
                _ => None,
            },
            satellites: pos1.satellites.clone(), // Use pos1 satellites for simplicity
            hdop: pos1.hdop + alpha as f32 * (pos2.hdop - pos1.hdop),
            vdop: pos1.vdop + alpha as f32 * (pos2.vdop - pos1.vdop),
            quality_score: pos1.quality_score.min(pos2.quality_score) * (1.0 - alpha as f32 * 0.1),
        };
        
        Ok(interpolated_pos)
    }
    
    fn detect_spoofing(&mut self) -> Result<SpoofingDetectionMetrics, GpsError> {
        Ok(self.detect_spoofing_internal())
    }
    
    fn calculate_quality_score(&self, position: &GpsPosition) -> f32 {
        self.calculate_quality_score_internal(position)
    }
    
    fn validate_position_quality(&self, position: &GpsPosition) -> Result<bool, GpsError> {
        let quality_score = self.calculate_quality_score_internal(position);
        
        let min_quality_threshold = 0.3; // Configurable threshold
        
        if quality_score < min_quality_threshold {
            return Err(GpsError::QualityTooLow { 
                score: quality_score, 
                threshold: min_quality_threshold 
            });
        }
        
        Ok(quality_score >= min_quality_threshold)
    }
    
    fn get_constellation_status(&self) -> HashMap<GnssConstellation, bool> {
        self.constellation_status.clone()
    }
    
    fn configure_constellations(&mut self, enabled_constellations: Vec<GnssConstellation>) -> Result<(), GpsError> {
        // Update constellation status
        self.constellation_status.clear();
        for constellation in &enabled_constellations {
            self.constellation_status.insert(constellation.clone(), true);
        }
        
        // Update config
        self.config.enabled_constellations = enabled_constellations;
        
        Ok(())
    }
}

impl AdvancedGpsManager {
    /// Calculate velocity and acceleration from position history
    fn calculate_motion_parameters(&self) -> (Vector3<f64>, Vector3<f64>) {
        if self.position_history.len() < 3 {
            return (Vector3::zeros(), Vector3::zeros());
        }
        
        let positions: Vec<_> = self.position_history.iter().rev().take(5).collect();
        let mut velocities = Vec::new();
        
        // Calculate velocities between consecutive positions
        for i in 0..positions.len()-1 {
            let pos1 = positions[i+1];
            let pos2 = positions[i];
            
            let dt = pos2.timestamp.duration_since(pos1.timestamp)
                .unwrap_or(Duration::from_secs(1)).as_secs_f64();
            
            if dt > 0.0 {
                let lat_diff = (pos2.latitude - pos1.latitude) * 111320.0;
                let lon_diff = (pos2.longitude - pos1.longitude) * 111320.0 * pos1.latitude.to_radians().cos();
                let alt_diff = pos2.altitude - pos1.altitude;
                
                let velocity = Vector3::new(
                    lon_diff / dt,
                    lat_diff / dt,
                    alt_diff / dt,
                );
                
                velocities.push(velocity);
            }
        }
        
        if velocities.is_empty() {
            return (Vector3::zeros(), Vector3::zeros());
        }
        
        // Average velocity
        let avg_velocity = velocities.iter().fold(Vector3::zeros(), |acc, v| acc + v) / velocities.len() as f64;
        
        // Calculate acceleration from velocity changes
        let acceleration = if velocities.len() >= 2 {
            let recent_velocity = velocities[0];
            let older_velocity = velocities[velocities.len()-1];
            let dt = (velocities.len() - 1) as f64 * 5.0; // Assume 5 second intervals
            
            if dt > 0.0 {
                (recent_velocity - older_velocity) / dt
            } else {
                Vector3::zeros()
            }
        } else {
            Vector3::zeros()
        };
        
        (avg_velocity, acceleration)
    }
    
    /// Calculate motion consistency score for prediction confidence
    fn calculate_motion_consistency(&self) -> f32 {
        if self.position_history.len() < 3 {
            return 0.5;
        }
        
        let positions: Vec<_> = self.position_history.iter().rev().take(5).collect();
        let mut velocity_variations: Vec<f64> = Vec::new();
        let mut prev_velocity: Option<Vector3<f64>> = None;
        
        for i in 0..positions.len()-1 {
            let pos1 = positions[i+1];
            let pos2 = positions[i];
            
            let dt = pos2.timestamp.duration_since(pos1.timestamp)
                .unwrap_or(Duration::from_secs(1)).as_secs_f64();
            
            if dt > 0.0 {
                let lat_diff = (pos2.latitude - pos1.latitude) * 111320.0;
                let lon_diff = (pos2.longitude - pos1.longitude) * 111320.0 * pos1.latitude.to_radians().cos();
                
                let velocity: Vector3<f64> = Vector3::new(lon_diff / dt, lat_diff / dt, 0.0);
                
                if let Some(prev_vel) = prev_velocity {
                    let velocity_change: f64 = (velocity - prev_vel).magnitude();
                    velocity_variations.push(velocity_change);
                }
                
                prev_velocity = Some(velocity);
            }
        }
        
        if velocity_variations.is_empty() {
            return 0.5;
        }
        
        // Calculate coefficient of variation
        let mean_variation: f64 = velocity_variations.iter().sum::<f64>() / velocity_variations.len() as f64;
        let variance: f64 = velocity_variations.iter()
            .map(|v| (v - mean_variation).powi(2))
            .sum::<f64>() / velocity_variations.len() as f64;
        let std_dev = variance.sqrt();
        
        let coefficient_of_variation = if mean_variation > 0.0 {
            std_dev / mean_variation
        } else {
            0.0
        };
        
        // Convert to consistency score (lower variation = higher consistency)
        (1.0f32 / (1.0 + coefficient_of_variation as f32)).max(0.1).min(1.0)
    }
}

impl AdvancedGpsManager {
    /// Simulate GPS data acquisition (placeholder for real hardware interface)
    fn simulate_gps_acquisition(&mut self) -> Result<Option<GpsPosition>, GpsError> {
        // This would interface with actual GPS hardware
        // For testing purposes, simulate realistic GPS data
        
        // Simulate satellite acquisition process
        if self.available_satellites.is_empty() {
            self.simulate_satellite_constellation();
        }
        
        // Check if we have enough satellites for a fix
        let usable_satellites: Vec<_> = self.available_satellites.iter()
            .filter(|sat| {
                sat.elevation >= self.config.min_elevation_deg &&
                sat.signal_strength > 30.0 &&
                self.config.enabled_constellations.contains(&sat.constellation)
            })
            .cloned()
            .collect();
        
        if usable_satellites.len() < self.config.min_satellite_count as usize {
            return Ok(None);
        }
        
        // Calculate GDOP
        let (hdop, vdop) = self.calculate_gdop(&usable_satellites);
        
        if hdop > self.config.max_hdop || vdop > self.config.max_vdop {
            return Ok(None);
        }
        
        // Simulate position calculation
        let base_lat = 37.7749; // San Francisco Bay area for testing
        let base_lon = -122.4194;
        let base_alt = 10.0;
        
        // Add some realistic variation
        let time_offset = SystemTime::now().duration_since(SystemTime::UNIX_EPOCH)
            .unwrap().as_secs_f64();
        let lat_variation = (time_offset * 0.001).sin() * 0.0001; // ~10m variation
        let lon_variation = (time_offset * 0.0015).cos() * 0.0001;
        let alt_variation = (time_offset * 0.002).sin() * 2.0;
        
        // Calculate accuracy based on HDOP and satellite count
        let base_accuracy = hdop * 2.0;
        let satellite_bonus = (usable_satellites.len() as f32 - 4.0) * 0.5;
        let accuracy = (base_accuracy - satellite_bonus).max(1.0);
        
        // Create velocity estimate if we have position history
        let velocity = if let Some(last_pos) = self.position_history.back() {
            let dt = SystemTime::now().duration_since(last_pos.timestamp)
                .unwrap_or(Duration::from_secs(1)).as_secs_f64();
            
            if dt > 0.0 {
                let lat_diff = (base_lat + lat_variation - last_pos.latitude) * 111320.0;
                let lon_diff = (base_lon + lon_variation - last_pos.longitude) * 111320.0 * base_lat.to_radians().cos();
                let alt_diff = base_alt + alt_variation - last_pos.altitude;
                
                Some(Vector3::new(
                    lon_diff / dt,
                    lat_diff / dt,
                    alt_diff / dt,
                ))
            } else {
                None
            }
        } else {
            None
        };
        
        let position = GpsPosition {
            latitude: base_lat + lat_variation,
            longitude: base_lon + lon_variation,
            altitude: base_alt + alt_variation,
            timestamp: SystemTime::now(),
            accuracy_m: accuracy,
            satellite_count: usable_satellites.len() as u8,
            velocity,
            satellites: usable_satellites,
            hdop,
            vdop,
            quality_score: 0.0, // Will be calculated later
        };
        
        Ok(Some(position))
    }
    
    /// Simulate satellite constellation for testing
    fn simulate_satellite_constellation(&mut self) {
        let mut satellites = Vec::new();
        
        // Simulate GPS satellites
        if self.config.enabled_constellations.contains(&GnssConstellation::GPS) {
            for i in 1..=12 {
                satellites.push(SatelliteInfo {
                    constellation: GnssConstellation::GPS,
                    satellite_id: i,
                    elevation: 15.0 + (i as f32 * 7.0) % 60.0,
                    azimuth: (i as f32 * 30.0) % 360.0,
                    signal_strength: 35.0 + (i as f32 * 3.0) % 20.0,
                    used_in_fix: true,
                });
            }
        }
        
        // Simulate GLONASS satellites
        if self.config.enabled_constellations.contains(&GnssConstellation::GLONASS) {
            for i in 1..=8 {
                satellites.push(SatelliteInfo {
                    constellation: GnssConstellation::GLONASS,
                    satellite_id: i + 64, // GLONASS PRN offset
                    elevation: 20.0 + (i as f32 * 8.0) % 50.0,
                    azimuth: (i as f32 * 45.0 + 22.5) % 360.0,
                    signal_strength: 30.0 + (i as f32 * 4.0) % 25.0,
                    used_in_fix: true,
                });
            }
        }
        
        // Simulate Galileo satellites
        if self.config.enabled_constellations.contains(&GnssConstellation::Galileo) {
            for i in 1..=8 {
                satellites.push(SatelliteInfo {
                    constellation: GnssConstellation::Galileo,
                    satellite_id: i + 200, // Galileo PRN offset
                    elevation: 25.0 + (i as f32 * 6.0) % 45.0,
                    azimuth: (i as f32 * 40.0 + 10.0) % 360.0,
                    signal_strength: 32.0 + (i as f32 * 3.5) % 22.0,
                    used_in_fix: true,
                });
            }
        }
        
        // Simulate BeiDou satellites
        if self.config.enabled_constellations.contains(&GnssConstellation::BeiDou) {
            for i in 1..=6 {
                satellites.push(SatelliteInfo {
                    constellation: GnssConstellation::BeiDou,
                    satellite_id: i + 140, // BeiDou PRN offset
                    elevation: 18.0 + (i as f32 * 9.0) % 55.0,
                    azimuth: (i as f32 * 60.0 + 30.0) % 360.0,
                    signal_strength: 28.0 + (i as f32 * 5.0) % 27.0,
                    used_in_fix: true,
                });
            }
        }
        
        self.available_satellites = satellites;
    }
    
    /// Check if position meets accuracy requirements
    fn is_position_accurate(&self, position: &GpsPosition) -> bool {
        position.accuracy_m <= self.config.accuracy_threshold_m &&
        position.satellite_count >= self.config.min_satellite_count &&
        position.hdop <= self.config.max_hdop &&
        position.vdop <= self.config.max_vdop
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
    
    // Advanced GPS features - basic implementations
    
    fn get_available_satellites(&self) -> Vec<SatelliteInfo> {
        Vec::new() // Basic implementation returns empty list
    }
    
    fn optimize_constellation_selection(&mut self) -> Result<(), GpsError> {
        Ok(()) // Basic implementation does nothing
    }
    
    fn get_dead_reckoning_state(&self) -> Option<DeadReckoningState> {
        None // Basic implementation doesn't support dead reckoning
    }
    
    fn update_dead_reckoning(&mut self, _acceleration: Vector3<f64>, _angular_velocity: Vector3<f64>) -> Result<(), GpsError> {
        Err(GpsError::DeadReckoningFailed) // Basic implementation doesn't support dead reckoning
    }
    
    fn predict_position(&self, _time_horizon: Duration) -> Result<PositionPrediction, GpsError> {
        Err(GpsError::PredictionFailed) // Basic implementation doesn't support prediction
    }
    
    fn interpolate_position(&self, _pos1: &GpsPosition, _pos2: &GpsPosition, _interpolation_time: SystemTime) -> Result<GpsPosition, GpsError> {
        Err(GpsError::PredictionFailed) // Basic implementation doesn't support interpolation
    }
    
    fn detect_spoofing(&mut self) -> Result<SpoofingDetectionMetrics, GpsError> {
        Ok(SpoofingDetectionMetrics {
            signal_power_anomaly: 0.0,
            timing_inconsistency: 0.0,
            position_jump_magnitude: 0.0,
            satellite_geometry_score: 0.0,
            spoofing_probability: 0.0,
            detection_confidence: 0.0,
        })
    }
    
    fn calculate_quality_score(&self, position: &GpsPosition) -> f32 {
        // Simple quality score based on accuracy and satellite count
        let accuracy_score = if position.accuracy_m <= 5.0 { 1.0 } else { 0.5 };
        let sat_score = if position.satellite_count >= 6 { 1.0 } else { 0.7 };
        accuracy_score * sat_score
    }
    
    fn validate_position_quality(&self, position: &GpsPosition) -> Result<bool, GpsError> {
        let quality_score = self.calculate_quality_score(position);
        Ok(quality_score >= 0.3)
    }
    
    fn get_constellation_status(&self) -> HashMap<GnssConstellation, bool> {
        let mut status = HashMap::new();
        status.insert(GnssConstellation::GPS, true);
        status
    }
    
    fn configure_constellations(&mut self, _enabled_constellations: Vec<GnssConstellation>) -> Result<(), GpsError> {
        Ok(()) // Basic implementation ignores constellation configuration
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
        let valid_config = GpsConfig::default();
        
        let manager = BasicGpsManager::new(valid_config);
        assert!(manager.is_ok());
        
        // Invalid configuration - zero timeout
        let mut invalid_config = GpsConfig::default();
        invalid_config.acquisition_timeout_s = 0;
        
        let manager = BasicGpsManager::new(invalid_config);
        assert!(matches!(manager, Err(GpsError::ConfigurationInvalid)));
        
        // Invalid configuration - too few satellites
        let mut invalid_config = GpsConfig::default();
        invalid_config.min_satellite_count = 2;
        
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
        
        let mut new_config = GpsConfig::default();
        new_config.acquisition_timeout_s = 120;
        new_config.update_interval_s = 10;
        new_config.min_satellite_count = 6;
        new_config.accuracy_threshold_m = 5.0;
        new_config.cold_start_timeout_s = 180;
        
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
        let config = GpsConfig::default();
        let manager = BasicGpsManager::new(config).unwrap();
        
        // Accurate position
        let accurate_position = GpsPosition {
            latitude: 37.7749,
            longitude: -122.4194,
            altitude: 10.0,
            timestamp: SystemTime::now(),
            accuracy_m: 5.0,
            satellite_count: 6,
            velocity: None,
            satellites: Vec::new(),
            hdop: 2.0,
            vdop: 3.0,
            quality_score: 0.8,
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
            velocity: None,
            satellites: Vec::new(),
            hdop: 8.0,
            vdop: 10.0,
            quality_score: 0.3,
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
            velocity: None,
            satellites: Vec::new(),
            hdop: 3.0,
            vdop: 4.0,
            quality_score: 0.4,
        };
        assert!(!manager.is_position_accurate(&inaccurate_position));
    }
    
    #[test]
    fn test_advanced_gps_manager_initialization() {
        let config = GpsConfig::default();
        let manager = AdvancedGpsManager::new(config).unwrap();
        
        assert_eq!(manager.get_status(), GpsStatus::Initializing);
        assert!(!manager.is_locked());
        assert_eq!(manager.get_satellite_count(), 0);
        assert!(manager.get_current_position().is_none());
        assert!(manager.get_dead_reckoning_state().is_none());
        assert_eq!(manager.get_available_satellites().len(), 0);
    }
    
    #[test]
    fn test_constellation_configuration() {
        let mut config = GpsConfig::default();
        config.enabled_constellations = vec![
            GnssConstellation::GPS,
            GnssConstellation::Galileo,
        ];
        
        let mut manager = AdvancedGpsManager::new(config).unwrap();
        
        let constellation_status = manager.get_constellation_status();
        assert_eq!(constellation_status.len(), 2);
        assert_eq!(constellation_status[&GnssConstellation::GPS], true);
        assert_eq!(constellation_status[&GnssConstellation::Galileo], true);
        
        // Test reconfiguration
        let new_constellations = vec![
            GnssConstellation::GPS,
            GnssConstellation::GLONASS,
            GnssConstellation::BeiDou,
        ];
        
        let result = manager.configure_constellations(new_constellations);
        assert!(result.is_ok());
        
        let updated_status = manager.get_constellation_status();
        assert_eq!(updated_status.len(), 3);
        assert_eq!(updated_status[&GnssConstellation::GPS], true);
        assert_eq!(updated_status[&GnssConstellation::GLONASS], true);
        assert_eq!(updated_status[&GnssConstellation::BeiDou], true);
    }
    
    #[test]
    fn test_satellite_simulation() {
        let config = GpsConfig::default();
        let mut manager = AdvancedGpsManager::new(config).unwrap();
        
        // Start acquisition to trigger satellite simulation
        manager.start_acquisition().unwrap();
        manager.update().unwrap(); // This should trigger satellite simulation
        
        let satellites = manager.get_available_satellites();
        assert!(!satellites.is_empty());
        
        // Check that we have satellites from enabled constellations
        let mut has_gps = false;
        let mut has_glonass = false;
        let mut has_galileo = false;
        
        for satellite in &satellites {
            match satellite.constellation {
                GnssConstellation::GPS => has_gps = true,
                GnssConstellation::GLONASS => has_glonass = true,
                GnssConstellation::Galileo => has_galileo = true,
                _ => {}
            }
        }
        
        assert!(has_gps);
        assert!(has_glonass);
        assert!(has_galileo);
    }
    
    #[test]
    fn test_gdop_calculation() {
        let config = GpsConfig::default();
        let manager = AdvancedGpsManager::new(config).unwrap();
        
        // Create test satellites with good geometry
        let satellites = vec![
            SatelliteInfo {
                constellation: GnssConstellation::GPS,
                satellite_id: 1,
                elevation: 45.0,
                azimuth: 0.0,
                signal_strength: 40.0,
                used_in_fix: true,
            },
            SatelliteInfo {
                constellation: GnssConstellation::GPS,
                satellite_id: 2,
                elevation: 45.0,
                azimuth: 90.0,
                signal_strength: 40.0,
                used_in_fix: true,
            },
            SatelliteInfo {
                constellation: GnssConstellation::GPS,
                satellite_id: 3,
                elevation: 45.0,
                azimuth: 180.0,
                signal_strength: 40.0,
                used_in_fix: true,
            },
            SatelliteInfo {
                constellation: GnssConstellation::GPS,
                satellite_id: 4,
                elevation: 45.0,
                azimuth: 270.0,
                signal_strength: 40.0,
                used_in_fix: true,
            },
        ];
        
        let (hdop, vdop) = manager.calculate_gdop(&satellites);
        
        // Good geometry should result in reasonable GDOP values
        assert!(hdop > 0.0 && hdop < 10.0);
        assert!(vdop > 0.0 && vdop < 15.0);
        assert!(vdop > hdop); // VDOP should typically be higher than HDOP
    }
    
    #[test]
    fn test_spoofing_detection() {
        let config = GpsConfig::default();
        let mut manager = AdvancedGpsManager::new(config).unwrap();
        
        // Test with normal conditions
        let normal_metrics = manager.detect_spoofing().unwrap();
        assert!(normal_metrics.spoofing_probability < 0.3);
        
        // Simulate suspicious signal conditions
        manager.available_satellites = vec![
            SatelliteInfo {
                constellation: GnssConstellation::GPS,
                satellite_id: 1,
                elevation: 45.0,
                azimuth: 0.0,
                signal_strength: 60.0, // Unusually high
                used_in_fix: true,
            },
            SatelliteInfo {
                constellation: GnssConstellation::GPS,
                satellite_id: 2,
                elevation: 45.0,
                azimuth: 10.0, // Clustered azimuth
                signal_strength: 60.0, // Unusually high
                used_in_fix: true,
            },
        ];
        
        let suspicious_metrics = manager.detect_spoofing().unwrap();
        assert!(suspicious_metrics.signal_power_anomaly > 0.0);
        assert!(suspicious_metrics.spoofing_probability > normal_metrics.spoofing_probability);
    }
    
    #[test]
    fn test_quality_scoring() {
        let config = GpsConfig::default();
        let manager = AdvancedGpsManager::new(config).unwrap();
        
        // High quality position
        let high_quality_position = GpsPosition {
            latitude: 37.7749,
            longitude: -122.4194,
            altitude: 10.0,
            timestamp: SystemTime::now(),
            accuracy_m: 2.0,
            satellite_count: 10,
            velocity: Some(Vector3::new(1.0, 0.5, 0.0)),
            satellites: vec![
                SatelliteInfo {
                    constellation: GnssConstellation::GPS,
                    satellite_id: 1,
                    elevation: 45.0,
                    azimuth: 0.0,
                    signal_strength: 45.0,
                    used_in_fix: true,
                },
                SatelliteInfo {
                    constellation: GnssConstellation::Galileo,
                    satellite_id: 201,
                    elevation: 50.0,
                    azimuth: 90.0,
                    signal_strength: 42.0,
                    used_in_fix: true,
                },
            ],
            hdop: 1.5,
            vdop: 2.0,
            quality_score: 0.0, // Will be calculated
        };
        
        let quality_score = manager.calculate_quality_score(&high_quality_position);
        assert!(quality_score > 0.7);
        
        // Low quality position
        let low_quality_position = GpsPosition {
            latitude: 37.7749,
            longitude: -122.4194,
            altitude: 10.0,
            timestamp: SystemTime::now(),
            accuracy_m: 25.0,
            satellite_count: 3,
            velocity: None,
            satellites: vec![
                SatelliteInfo {
                    constellation: GnssConstellation::GPS,
                    satellite_id: 1,
                    elevation: 15.0,
                    azimuth: 0.0,
                    signal_strength: 20.0,
                    used_in_fix: true,
                },
            ],
            hdop: 8.0,
            vdop: 12.0,
            quality_score: 0.0,
        };
        
        let low_quality_score = manager.calculate_quality_score(&low_quality_position);
        assert!(low_quality_score < 0.4);
        assert!(low_quality_score < quality_score);
    }
    
    #[test]
    fn test_position_interpolation() {
        let config = GpsConfig::default();
        let manager = AdvancedGpsManager::new(config).unwrap();
        
        let pos1 = GpsPosition {
            latitude: 37.7749,
            longitude: -122.4194,
            altitude: 10.0,
            timestamp: SystemTime::UNIX_EPOCH + Duration::from_secs(1000),
            accuracy_m: 5.0,
            satellite_count: 6,
            velocity: Some(Vector3::new(1.0, 0.5, 0.1)),
            satellites: Vec::new(),
            hdop: 2.0,
            vdop: 3.0,
            quality_score: 0.8,
        };
        
        let pos2 = GpsPosition {
            latitude: 37.7750,
            longitude: -122.4195,
            altitude: 10.1,
            timestamp: SystemTime::UNIX_EPOCH + Duration::from_secs(1010),
            accuracy_m: 5.0,
            satellite_count: 6,
            velocity: Some(Vector3::new(1.0, 0.5, 0.1)),
            satellites: Vec::new(),
            hdop: 2.0,
            vdop: 3.0,
            quality_score: 0.8,
        };
        
        let interpolation_time = SystemTime::UNIX_EPOCH + Duration::from_secs(1005);
        let interpolated = manager.interpolate_position(&pos1, &pos2, interpolation_time).unwrap();
        
        // Should be approximately halfway between pos1 and pos2
        assert!((interpolated.latitude - 37.77495).abs() < 0.0001);
        assert!((interpolated.longitude - (-122.41945)).abs() < 0.0001);
        assert!((interpolated.altitude - 10.05).abs() < 0.1);
    }
    
    #[test]
    fn test_motion_parameter_calculation() {
        let mut config = GpsConfig::default();
        config.position_prediction_enabled = true;
        let mut manager = AdvancedGpsManager::new(config).unwrap();
        
        // Add position history to simulate motion
        let base_time = SystemTime::now();
        for i in 0..5 {
            let position = GpsPosition {
                latitude: 37.7749 + (i as f64 * 0.0001),
                longitude: -122.4194 + (i as f64 * 0.0001),
                altitude: 10.0 + (i as f64 * 0.1),
                timestamp: base_time + Duration::from_secs(i * 5),
                accuracy_m: 3.0,
                satellite_count: 8,
                velocity: Some(Vector3::new(1.0, 1.0, 0.02)),
                satellites: Vec::new(),
                hdop: 2.0,
                vdop: 3.0,
                quality_score: 0.8,
            };
            manager.position_history.push_back(position);
        }
        
        let (velocity, acceleration) = manager.calculate_motion_parameters();
        
        // Should detect consistent motion
        assert!(velocity.magnitude() > 0.0);
        assert!(velocity.x > 0.0); // Moving east
        assert!(velocity.y > 0.0); // Moving north
    }
    
    #[test]
    fn test_position_prediction() {
        let mut config = GpsConfig::default();
        config.position_prediction_enabled = true;
        let mut manager = AdvancedGpsManager::new(config).unwrap();
        
        // Set up current position
        let current_position = GpsPosition {
            latitude: 37.7749,
            longitude: -122.4194,
            altitude: 10.0,
            timestamp: SystemTime::now(),
            accuracy_m: 3.0,
            satellite_count: 8,
            velocity: Some(Vector3::new(10.0, 5.0, 0.1)), // 10 m/s east, 5 m/s north
            satellites: Vec::new(),
            hdop: 2.0,
            vdop: 3.0,
            quality_score: 0.8,
        };
        
        manager.current_position = Some(current_position.clone());
        
        // Add some position history
        for i in 0..3 {
            let position = GpsPosition {
                latitude: 37.7749 - (i as f64 * 0.0001),
                longitude: -122.4194 - (i as f64 * 0.0001),
                altitude: 10.0,
                timestamp: SystemTime::now() - Duration::from_secs((i + 1) * 5),
                accuracy_m: 3.0,
                satellite_count: 8,
                velocity: Some(Vector3::new(10.0, 5.0, 0.1)),
                satellites: Vec::new(),
                hdop: 2.0,
                vdop: 3.0,
                quality_score: 0.8,
            };
            manager.position_history.push_back(position);
        }
        
        // Predict position 60 seconds in the future
        let prediction = manager.predict_position(Duration::from_secs(60)).unwrap();
        
        // Should predict movement based on velocity
        assert!(prediction.predicted_position.latitude > current_position.latitude);
        assert!(prediction.predicted_position.longitude > current_position.longitude);
        assert!(prediction.confidence > 0.0 && prediction.confidence <= 1.0);
        assert!(prediction.uncertainty_ellipse.0 > current_position.accuracy_m);
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
                velocity: Some(Vector3::new(1.0, 0.5, 0.0)),
                satellites: Vec::new(),
                hdop: 1.5,
                vdop: 2.0,
                quality_score: 0.9,
            },
            GpsPosition {
                latitude: 37.7750,
                longitude: -122.4195,
                altitude: 11.0,
                timestamp: SystemTime::now(),
                accuracy_m: 4.0,
                satellite_count: 7,
                velocity: Some(Vector3::new(1.1, 0.6, 0.1)),
                satellites: Vec::new(),
                hdop: 2.0,
                vdop: 2.5,
                quality_score: 0.8,
            },
            GpsPosition {
                latitude: 37.7751,
                longitude: -122.4196,
                altitude: 12.0,
                timestamp: SystemTime::now(),
                accuracy_m: 5.0,
                satellite_count: 6,
                velocity: Some(Vector3::new(1.2, 0.7, 0.2)),
                satellites: Vec::new(),
                hdop: 2.5,
                vdop: 3.0,
                quality_score: 0.7,
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
    
    // Advanced GPS features - mock implementations
    
    fn get_available_satellites(&self) -> Vec<SatelliteInfo> {
        Vec::new() // Mock implementation returns empty list
    }
    
    fn optimize_constellation_selection(&mut self) -> Result<(), GpsError> {
        Ok(()) // Mock implementation does nothing
    }
    
    fn get_dead_reckoning_state(&self) -> Option<DeadReckoningState> {
        None // Mock implementation doesn't support dead reckoning
    }
    
    fn update_dead_reckoning(&mut self, _acceleration: Vector3<f64>, _angular_velocity: Vector3<f64>) -> Result<(), GpsError> {
        Err(GpsError::DeadReckoningFailed) // Mock implementation doesn't support dead reckoning
    }
    
    fn predict_position(&self, _time_horizon: Duration) -> Result<PositionPrediction, GpsError> {
        Err(GpsError::PredictionFailed) // Mock implementation doesn't support prediction
    }
    
    fn interpolate_position(&self, _pos1: &GpsPosition, _pos2: &GpsPosition, _interpolation_time: SystemTime) -> Result<GpsPosition, GpsError> {
        Err(GpsError::PredictionFailed) // Mock implementation doesn't support interpolation
    }
    
    fn detect_spoofing(&mut self) -> Result<SpoofingDetectionMetrics, GpsError> {
        Ok(SpoofingDetectionMetrics {
            signal_power_anomaly: 0.0,
            timing_inconsistency: 0.0,
            position_jump_magnitude: 0.0,
            satellite_geometry_score: 0.0,
            spoofing_probability: 0.0,
            detection_confidence: 0.0,
        })
    }
    
    fn calculate_quality_score(&self, position: &GpsPosition) -> f32 {
        // Simple quality score based on accuracy and satellite count
        let accuracy_score = if position.accuracy_m <= 5.0 { 1.0 } else { 0.5 };
        let sat_score = if position.satellite_count >= 6 { 1.0 } else { 0.7 };
        accuracy_score * sat_score
    }
    
    fn validate_position_quality(&self, position: &GpsPosition) -> Result<bool, GpsError> {
        let quality_score = self.calculate_quality_score(position);
        Ok(quality_score >= 0.3)
    }
    
    fn get_constellation_status(&self) -> HashMap<GnssConstellation, bool> {
        let mut status = HashMap::new();
        status.insert(GnssConstellation::GPS, true);
        status
    }
    
    fn configure_constellations(&mut self, _enabled_constellations: Vec<GnssConstellation>) -> Result<(), GpsError> {
        Ok(()) // Mock implementation ignores constellation configuration
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