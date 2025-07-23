// Graceful degradation capabilities for underwater positioning system
// Implements fallback positioning modes, accuracy estimation, and system health monitoring

use std::collections::HashMap;
use std::time::{Duration, Instant, SystemTime};
use crate::error_handling::{PositioningError, ErrorSeverity, ErrorContext};
use crate::{Anchor, Position, trilaterate, SPEED_OF_SOUND_WATER};
use nalgebra::Vector3;

/// System operating modes with different capability levels
#[derive(Debug, Clone, PartialEq)]
pub enum OperatingMode {
    /// Full 3D positioning with 4+ anchors
    Full3D {
        anchor_count: u8,
        expected_accuracy_m: f32,
    },
    
    /// 2D positioning with depth estimation (3 anchors)
    Positioning2D {
        anchor_count: u8,
        depth_estimation_method: DepthEstimationMethod,
        expected_accuracy_m: f32,
    },
    
    /// Range and bearing to nearest anchor (2 anchors)
    RangeBearing {
        primary_anchor_id: u16,
        reference_anchor_id: u16,
        expected_accuracy_m: f32,
    },
    
    /// Dead reckoning using last known position
    DeadReckoning {
        last_position: Position,
        last_position_time: SystemTime,
        drift_rate_m_per_s: f32,
    },
    
    /// Emergency mode - basic status only
    Emergency {
        reason: String,
        limited_functionality: Vec<String>,
    },
    
    /// System offline
    Offline {
        reason: String,
    },
}

/// Methods for depth estimation in 2D mode
#[derive(Debug, Clone, PartialEq)]
pub enum DepthEstimationMethod {
    /// Use average depth of available anchors
    AverageAnchorDepth,
    /// Use weighted average based on signal strength
    WeightedAverage,
    /// Use last known depth
    LastKnownDepth { depth: f64 },
    /// Use configured default depth
    DefaultDepth { depth: f64 },
    /// Use depth from external sensor
    ExternalSensor { sensor_id: String },
}

/// Positioning result with uncertainty information
#[derive(Debug, Clone)]
pub struct PositioningResult {
    pub position: Position,
    pub local_position: Vector3<f64>,
    pub accuracy_estimate: AccuracyEstimate,
    pub operating_mode: OperatingMode,
    pub timestamp: SystemTime,
    pub anchor_count: u8,
    pub computation_time_ms: f64,
    pub warnings: Vec<String>,
}

/// Comprehensive accuracy estimation
#[derive(Debug, Clone)]
pub struct AccuracyEstimate {
    pub horizontal_accuracy_m: f32,
    pub vertical_accuracy_m: f32,
    pub overall_accuracy_m: f32,
    pub confidence_level: f32, // 0.0 to 1.0
    pub uncertainty_ellipse: UncertaintyEllipse,
    pub quality_indicators: QualityIndicators,
}

/// Uncertainty ellipse for position error visualization
#[derive(Debug, Clone)]
pub struct UncertaintyEllipse {
    pub semi_major_axis_m: f32,
    pub semi_minor_axis_m: f32,
    pub orientation_deg: f32,
    pub confidence_level: f32,
}

/// Quality indicators for positioning solution
#[derive(Debug, Clone)]
pub struct QualityIndicators {
    pub geometric_dilution_of_precision: f32,
    pub anchor_geometry_quality: GeometryQuality,
    pub signal_quality_average: f32,
    pub timing_precision_ms: f32,
    pub solution_convergence: bool,
    pub residual_error_m: f32,
}

/// Geometry quality assessment
#[derive(Debug, Clone, PartialEq, PartialOrd)]
pub enum GeometryQuality {
    Excellent,  // GDOP < 2.0
    Good,       // GDOP < 5.0
    Acceptable, // GDOP < 10.0
    Poor,       // GDOP < 20.0
    Degenerate, // GDOP >= 20.0 or singular
}

/// System health monitoring
#[derive(Debug, Clone)]
pub struct SystemHealth {
    pub overall_status: HealthStatus,
    pub subsystem_health: HashMap<String, SubsystemHealth>,
    pub performance_metrics: PerformanceMetrics,
    pub resource_usage: ResourceUsage,
    pub last_update: SystemTime,
}

/// Health status levels
#[derive(Debug, Clone, PartialEq, PartialOrd, Ord, Eq)]
pub enum HealthStatus {
    Healthy,
    Warning,
    Degraded,
    Critical,
    Failed,
}

/// Subsystem health information
#[derive(Debug, Clone)]
pub struct SubsystemHealth {
    pub status: HealthStatus,
    pub last_check: SystemTime,
    pub error_count: u32,
    pub performance_score: f32, // 0.0 to 1.0
    pub details: String,
}

/// Performance metrics
#[derive(Debug, Clone)]
pub struct PerformanceMetrics {
    pub positioning_success_rate: f32,
    pub average_computation_time_ms: f32,
    pub average_accuracy_m: f32,
    pub throughput_positions_per_second: f32,
    pub uptime_percentage: f32,
}

/// Resource usage monitoring
#[derive(Debug, Clone)]
pub struct ResourceUsage {
    pub memory_usage_percent: f32,
    pub cpu_usage_percent: f32,
    pub battery_level_percent: Option<f32>,
    pub storage_usage_percent: f32,
    pub network_bandwidth_usage_percent: f32,
}

/// Graceful degradation manager
pub struct GracefulDegradationManager {
    current_mode: OperatingMode,
    mode_history: Vec<(OperatingMode, SystemTime)>,
    system_health: SystemHealth,
    degradation_thresholds: DegradationThresholds,
    fallback_strategies: HashMap<String, FallbackStrategy>,
    last_successful_position: Option<(Position, SystemTime)>,
    performance_history: Vec<PerformanceSnapshot>,
    max_history_size: usize,
}

/// Thresholds for triggering degradation
#[derive(Debug, Clone)]
pub struct DegradationThresholds {
    pub min_anchors_full_3d: u8,
    pub min_anchors_2d: u8,
    pub max_anchor_age_ms: u32,
    pub min_geometry_quality: GeometryQuality,
    pub max_computation_time_ms: f64,
    pub min_accuracy_threshold_m: f32,
    pub max_error_rate: f32,
    pub min_signal_quality: f32,
}

impl Default for DegradationThresholds {
    fn default() -> Self {
        Self {
            min_anchors_full_3d: 4,
            min_anchors_2d: 3,
            max_anchor_age_ms: 5000,
            min_geometry_quality: GeometryQuality::Acceptable,
            max_computation_time_ms: 200.0,
            min_accuracy_threshold_m: 10.0,
            max_error_rate: 0.1,
            min_signal_quality: 100.0,
        }
    }
}

/// Fallback strategy configuration
#[derive(Debug, Clone)]
pub struct FallbackStrategy {
    pub trigger_conditions: Vec<String>,
    pub target_mode: OperatingMode,
    pub parameter_adjustments: HashMap<String, f64>,
    pub expected_performance_impact: f32,
    pub recovery_conditions: Vec<String>,
}

/// Performance snapshot for trend analysis
#[derive(Debug, Clone)]
pub struct PerformanceSnapshot {
    pub timestamp: SystemTime,
    pub mode: OperatingMode,
    pub success_rate: f32,
    pub accuracy: f32,
    pub computation_time: f32,
    pub anchor_count: u8,
}

impl GracefulDegradationManager {
    pub fn new() -> Self {
        let mut fallback_strategies = HashMap::new();
        
        // Define default fallback strategies
        fallback_strategies.insert("insufficient_anchors_3d".to_string(), FallbackStrategy {
            trigger_conditions: vec!["anchor_count < 4".to_string()],
            target_mode: OperatingMode::Positioning2D {
                anchor_count: 3,
                depth_estimation_method: DepthEstimationMethod::AverageAnchorDepth,
                expected_accuracy_m: 3.0,
            },
            parameter_adjustments: HashMap::from([
                ("timeout_multiplier".to_string(), 1.5),
                ("accuracy_threshold".to_string(), 3.0),
            ]),
            expected_performance_impact: 0.3,
            recovery_conditions: vec!["anchor_count >= 4".to_string(), "geometry_quality >= Good".to_string()],
        });
        
        fallback_strategies.insert("insufficient_anchors_2d".to_string(), FallbackStrategy {
            trigger_conditions: vec!["anchor_count < 3".to_string()],
            target_mode: OperatingMode::RangeBearing {
                primary_anchor_id: 0,
                reference_anchor_id: 0,
                expected_accuracy_m: 10.0,
            },
            parameter_adjustments: HashMap::from([
                ("timeout_multiplier".to_string(), 2.0),
                ("accuracy_threshold".to_string(), 10.0),
            ]),
            expected_performance_impact: 0.7,
            recovery_conditions: vec!["anchor_count >= 3".to_string()],
        });
        
        fallback_strategies.insert("poor_geometry".to_string(), FallbackStrategy {
            trigger_conditions: vec!["geometry_quality == Poor".to_string()],
            target_mode: OperatingMode::Positioning2D {
                anchor_count: 3,
                depth_estimation_method: DepthEstimationMethod::LastKnownDepth { depth: 0.0 },
                expected_accuracy_m: 5.0,
            },
            parameter_adjustments: HashMap::from([
                ("regularization_factor".to_string(), 1e-3),
                ("accuracy_threshold".to_string(), 5.0),
            ]),
            expected_performance_impact: 0.4,
            recovery_conditions: vec!["geometry_quality >= Acceptable".to_string()],
        });
        
        Self {
            current_mode: OperatingMode::Full3D {
                anchor_count: 4,
                expected_accuracy_m: 1.0,
            },
            mode_history: Vec::new(),
            system_health: SystemHealth::new(),
            degradation_thresholds: DegradationThresholds::default(),
            fallback_strategies,
            last_successful_position: None,
            performance_history: Vec::new(),
            max_history_size: 1000,
        }
    }
    
    /// Attempt positioning with graceful degradation
    pub fn attempt_positioning(&mut self, anchors: &[Anchor], receiver_time_ms: u64) -> Result<PositioningResult, PositioningError> {
        let start_time = Instant::now();
        
        // Update system health
        self.update_system_health(anchors);
        
        // Determine appropriate operating mode
        let target_mode = self.determine_operating_mode(anchors, receiver_time_ms)?;
        
        // Switch mode if necessary
        if target_mode != self.current_mode {
            self.switch_operating_mode(target_mode.clone());
        }
        
        // Attempt positioning based on current mode
        let result = match &self.current_mode {
            OperatingMode::Full3D { .. } => {
                self.attempt_full_3d_positioning(anchors, receiver_time_ms)
            }
            OperatingMode::Positioning2D { depth_estimation_method, .. } => {
                self.attempt_2d_positioning(anchors, receiver_time_ms, depth_estimation_method)
            }
            OperatingMode::RangeBearing { primary_anchor_id, reference_anchor_id, .. } => {
                self.attempt_range_bearing_positioning(anchors, receiver_time_ms, *primary_anchor_id, *reference_anchor_id)
            }
            OperatingMode::DeadReckoning { last_position, last_position_time, drift_rate_m_per_s } => {
                self.attempt_dead_reckoning_positioning(last_position.clone(), *last_position_time, *drift_rate_m_per_s, receiver_time_ms)
            }
            OperatingMode::Emergency { reason, .. } => {
                Err(PositioningError::CriticalFailure {
                    failure_type: crate::error_handling::CriticalFailureType::SystemOverload,
                    system_state: format!("Emergency mode: {}", reason),
                    recovery_possible: true,
                })
            }
            OperatingMode::Offline { reason } => {
                Err(PositioningError::CriticalFailure {
                    failure_type: crate::error_handling::CriticalFailureType::SystemOverload,
                    system_state: format!("System offline: {}", reason),
                    recovery_possible: false,
                })
            }
        };
        
        let computation_time = start_time.elapsed().as_secs_f64() * 1000.0;
        
        // Record performance metrics
        self.record_performance_snapshot(&result, computation_time, anchors.len() as u8);
        
        // Update last successful position if applicable
        if let Ok(ref pos_result) = result {
            self.last_successful_position = Some((pos_result.position.clone(), SystemTime::now()));
        }
        
        result
    }
    
    /// Determine appropriate operating mode based on current conditions
    fn determine_operating_mode(&self, anchors: &[Anchor], receiver_time_ms: u64) -> Result<OperatingMode, PositioningError> {
        let anchor_count = anchors.len() as u8;
        let geometry_quality = self.assess_geometry_quality(anchors);
        
        // Check for critical failures first
        if anchor_count == 0 {
            return Ok(OperatingMode::Emergency {
                reason: "No anchors available".to_string(),
                limited_functionality: vec!["positioning".to_string()],
            });
        }
        
        // Check anchor data freshness
        let current_time = receiver_time_ms;
        let stale_anchors: Vec<_> = anchors.iter()
            .filter(|a| current_time.saturating_sub(a.timestamp) > self.degradation_thresholds.max_anchor_age_ms as u64)
            .collect();
        
        if stale_anchors.len() == anchors.len() {
            // All anchors are stale, try dead reckoning
            if let Some((last_pos, last_time)) = &self.last_successful_position {
                return Ok(OperatingMode::DeadReckoning {
                    last_position: last_pos.clone(),
                    last_position_time: *last_time,
                    drift_rate_m_per_s: 0.1, // Conservative drift estimate
                });
            } else {
                return Ok(OperatingMode::Emergency {
                    reason: "All anchor data is stale and no previous position available".to_string(),
                    limited_functionality: vec!["positioning".to_string()],
                });
            }
        }
        
        // Filter out stale anchors for mode determination
        let fresh_anchor_count = (anchors.len() - stale_anchors.len()) as u8;
        
        // Determine mode based on available fresh anchors and geometry
        if fresh_anchor_count >= self.degradation_thresholds.min_anchors_full_3d && 
           geometry_quality >= self.degradation_thresholds.min_geometry_quality {
            Ok(OperatingMode::Full3D {
                anchor_count: fresh_anchor_count,
                expected_accuracy_m: 1.0,
            })
        } else if fresh_anchor_count >= self.degradation_thresholds.min_anchors_2d {
            Ok(OperatingMode::Positioning2D {
                anchor_count: fresh_anchor_count,
                depth_estimation_method: DepthEstimationMethod::AverageAnchorDepth,
                expected_accuracy_m: 3.0,
            })
        } else if fresh_anchor_count >= 2 {
            // Find two best anchors for range/bearing
            let mut sorted_anchors: Vec<_> = anchors.iter().enumerate().collect();
            sorted_anchors.sort_by(|a, b| {
                let age_a = current_time.saturating_sub(a.1.timestamp);
                let age_b = current_time.saturating_sub(b.1.timestamp);
                age_a.cmp(&age_b)
            });
            
            Ok(OperatingMode::RangeBearing {
                primary_anchor_id: sorted_anchors[0].1.id.parse().unwrap_or(0),
                reference_anchor_id: sorted_anchors[1].1.id.parse().unwrap_or(1),
                expected_accuracy_m: 10.0,
            })
        } else {
            // Single anchor or no fresh anchors - try dead reckoning
            if let Some((last_pos, last_time)) = &self.last_successful_position {
                Ok(OperatingMode::DeadReckoning {
                    last_position: last_pos.clone(),
                    last_position_time: *last_time,
                    drift_rate_m_per_s: 0.2, // Higher drift rate due to uncertainty
                })
            } else {
                Ok(OperatingMode::Emergency {
                    reason: format!("Insufficient fresh anchors ({}) and no previous position", fresh_anchor_count),
                    limited_functionality: vec!["positioning".to_string()],
                })
            }
        }
    }
    
    /// Switch to new operating mode
    fn switch_operating_mode(&mut self, new_mode: OperatingMode) {
        let now = SystemTime::now();
        self.mode_history.push((self.current_mode.clone(), now));
        self.current_mode = new_mode;
        
        // Limit history size
        if self.mode_history.len() > 100 {
            self.mode_history.remove(0);
        }
    }
    
    /// Attempt full 3D positioning
    fn attempt_full_3d_positioning(&self, anchors: &[Anchor], receiver_time_ms: u64) -> Result<PositioningResult, PositioningError> {
        // Use existing trilateration function
        match trilaterate(anchors, receiver_time_ms) {
            Ok((position, local_position)) => {
                let accuracy = self.estimate_accuracy(anchors, &local_position);
                let warnings = self.generate_warnings(anchors, &accuracy);
                
                Ok(PositioningResult {
                    position,
                    local_position,
                    accuracy_estimate: accuracy,
                    operating_mode: self.current_mode.clone(),
                    timestamp: SystemTime::now(),
                    anchor_count: anchors.len() as u8,
                    computation_time_ms: 0.0, // Will be filled by caller
                    warnings,
                })
            }
            Err(e) => Err(PositioningError::ComputationFailure {
                operation: "full_3d_trilateration".to_string(),
                details: e,
                input_data_summary: format!("{} anchors", anchors.len()),
            })
        }
    }
    
    /// Attempt 2D positioning with depth estimation
    fn attempt_2d_positioning(&self, anchors: &[Anchor], receiver_time_ms: u64, 
                              depth_method: &DepthEstimationMethod) -> Result<PositioningResult, PositioningError> {
        // Use first 3 anchors for 2D positioning
        let anchors_2d = if anchors.len() >= 3 { &anchors[0..3] } else { anchors };
        
        match trilaterate(anchors_2d, receiver_time_ms) {
            Ok((mut position, mut local_position)) => {
                // Apply depth estimation
                let estimated_depth = self.estimate_depth(anchors_2d, depth_method);
                position.depth = estimated_depth;
                local_position.z = estimated_depth;
                
                let accuracy = self.estimate_accuracy_2d(anchors_2d, &local_position);
                let mut warnings = self.generate_warnings(anchors_2d, &accuracy);
                warnings.push("Using 2D positioning mode - depth estimated".to_string());
                
                Ok(PositioningResult {
                    position,
                    local_position,
                    accuracy_estimate: accuracy,
                    operating_mode: self.current_mode.clone(),
                    timestamp: SystemTime::now(),
                    anchor_count: anchors_2d.len() as u8,
                    computation_time_ms: 0.0,
                    warnings,
                })
            }
            Err(e) => Err(PositioningError::ComputationFailure {
                operation: "2d_positioning".to_string(),
                details: e,
                input_data_summary: format!("{} anchors", anchors_2d.len()),
            })
        }
    }
    
    /// Attempt range and bearing positioning
    fn attempt_range_bearing_positioning(&self, anchors: &[Anchor], receiver_time_ms: u64,
                                        primary_anchor_id: u16, reference_anchor_id: u16) -> Result<PositioningResult, PositioningError> {
        // Find the specified anchors
        let primary_anchor = anchors.iter().find(|a| a.id.parse().unwrap_or(0) == primary_anchor_id)
            .ok_or_else(|| PositioningError::InsufficientAnchors {
                available: anchors.len() as u8,
                required: 1,
                anchor_ids: vec![primary_anchor_id],
            })?;
        
        let reference_anchor = anchors.iter().find(|a| a.id.parse().unwrap_or(0) == reference_anchor_id);
        
        // Calculate range to primary anchor
        let dt_ms = receiver_time_ms as i64 - primary_anchor.timestamp as i64;
        if dt_ms < 0 {
            return Err(PositioningError::TimingError {
                error_type: crate::error_handling::TimingIssue::TimestampInvalid,
                time_offset_ms: dt_ms,
                affected_anchors: vec![primary_anchor_id],
            });
        }
        
        let range_m = SPEED_OF_SOUND_WATER * (dt_ms as f64 / 1000.0);
        
        // Estimate position based on range and bearing
        let estimated_position = if let Some(ref_anchor) = reference_anchor {
            // Use bearing from reference anchor to primary anchor
            let bearing = self.calculate_bearing(&ref_anchor.position, &primary_anchor.position);
            self.estimate_position_from_range_bearing(&primary_anchor.position, range_m, bearing)
        } else {
            // Use primary anchor position as rough estimate
            Position {
                lat: primary_anchor.position.lat,
                lon: primary_anchor.position.lon,
                depth: primary_anchor.position.depth,
            }
        };
        
        let local_position = Vector3::new(0.0, 0.0, estimated_position.depth);
        let accuracy = AccuracyEstimate {
            horizontal_accuracy_m: 10.0,
            vertical_accuracy_m: 5.0,
            overall_accuracy_m: 12.0,
            confidence_level: 0.5,
            uncertainty_ellipse: UncertaintyEllipse {
                semi_major_axis_m: 15.0,
                semi_minor_axis_m: 8.0,
                orientation_deg: 0.0,
                confidence_level: 0.68,
            },
            quality_indicators: QualityIndicators {
                geometric_dilution_of_precision: 20.0,
                anchor_geometry_quality: GeometryQuality::Poor,
                signal_quality_average: 150.0,
                timing_precision_ms: 5.0,
                solution_convergence: false,
                residual_error_m: 8.0,
            },
        };
        
        let warnings = vec![
            "Using range/bearing mode - accuracy significantly reduced".to_string(),
            format!("Range to anchor {}: {:.1} m", primary_anchor_id, range_m),
        ];
        
        Ok(PositioningResult {
            position: estimated_position,
            local_position,
            accuracy_estimate: accuracy,
            operating_mode: self.current_mode.clone(),
            timestamp: SystemTime::now(),
            anchor_count: anchors.len() as u8,
            computation_time_ms: 0.0,
            warnings,
        })
    }
    
    /// Attempt dead reckoning positioning
    fn attempt_dead_reckoning_positioning(&self, last_position: Position, last_time: SystemTime,
                                         drift_rate: f32, receiver_time_ms: u64) -> Result<PositioningResult, PositioningError> {
        let current_time = SystemTime::UNIX_EPOCH + Duration::from_millis(receiver_time_ms);
        let time_elapsed = current_time.duration_since(last_time)
            .map_err(|_| PositioningError::TimingError {
                error_type: crate::error_handling::TimingIssue::TimestampInvalid,
                time_offset_ms: -1000,
                affected_anchors: vec![],
            })?;
        
        let drift_distance = drift_rate * time_elapsed.as_secs_f32();
        
        // Simple dead reckoning - assume random drift
        let estimated_position = Position {
            lat: last_position.lat,
            lon: last_position.lon,
            depth: last_position.depth,
        };
        
        let local_position = Vector3::new(0.0, 0.0, estimated_position.depth);
        let accuracy = AccuracyEstimate {
            horizontal_accuracy_m: drift_distance,
            vertical_accuracy_m: drift_distance * 0.5,
            overall_accuracy_m: drift_distance * 1.2,
            confidence_level: 0.3,
            uncertainty_ellipse: UncertaintyEllipse {
                semi_major_axis_m: drift_distance * 1.5,
                semi_minor_axis_m: drift_distance,
                orientation_deg: 0.0,
                confidence_level: 0.68,
            },
            quality_indicators: QualityIndicators {
                geometric_dilution_of_precision: 100.0,
                anchor_geometry_quality: GeometryQuality::Degenerate,
                signal_quality_average: 0.0,
                timing_precision_ms: 1000.0,
                solution_convergence: false,
                residual_error_m: drift_distance,
            },
        };
        
        let warnings = vec![
            "Using dead reckoning - position estimate only".to_string(),
            format!("Time since last fix: {:.1} s", time_elapsed.as_secs_f32()),
            format!("Estimated drift: {:.1} m", drift_distance),
        ];
        
        Ok(PositioningResult {
            position: estimated_position,
            local_position,
            accuracy_estimate: accuracy,
            operating_mode: self.current_mode.clone(),
            timestamp: SystemTime::now(),
            anchor_count: 0,
            computation_time_ms: 0.0,
            warnings,
        })
    }
    
    /// Assess geometry quality of anchor configuration
    fn assess_geometry_quality(&self, anchors: &[Anchor]) -> GeometryQuality {
        if anchors.len() < 3 {
            return GeometryQuality::Degenerate;
        }
        
        // Simple geometry assessment based on anchor spread
        let mut min_distance = f64::INFINITY;
        let mut max_distance: f64 = 0.0;
        
        for i in 0..anchors.len() {
            for j in i+1..anchors.len() {
                let distance = self.calculate_distance(&anchors[i].position, &anchors[j].position);
                min_distance = min_distance.min(distance);
                max_distance = max_distance.max(distance);
            }
        }
        
        if min_distance == 0.0 || max_distance == 0.0 {
            return GeometryQuality::Degenerate;
        }
        
        let spread_ratio = max_distance / min_distance;
        
        match spread_ratio {
            r if r > 10.0 => GeometryQuality::Excellent,
            r if r > 5.0 => GeometryQuality::Good,
            r if r > 2.0 => GeometryQuality::Acceptable,
            r if r > 1.0 => GeometryQuality::Poor,
            _ => GeometryQuality::Degenerate,
        }
    }
    
    /// Estimate accuracy for positioning result
    fn estimate_accuracy(&self, anchors: &[Anchor], _position: &Vector3<f64>) -> AccuracyEstimate {
        let geometry_quality = self.assess_geometry_quality(anchors);
        
        let (horizontal_acc, vertical_acc) = match geometry_quality {
            GeometryQuality::Excellent => (0.5, 0.3),
            GeometryQuality::Good => (1.0, 0.8),
            GeometryQuality::Acceptable => (2.0, 1.5),
            GeometryQuality::Poor => (5.0, 3.0),
            GeometryQuality::Degenerate => (20.0, 15.0),
        };
        
        AccuracyEstimate {
            horizontal_accuracy_m: horizontal_acc,
            vertical_accuracy_m: vertical_acc,
            overall_accuracy_m: (horizontal_acc.powi(2) + vertical_acc.powi(2)).sqrt(),
            confidence_level: match geometry_quality {
                GeometryQuality::Excellent => 0.95,
                GeometryQuality::Good => 0.85,
                GeometryQuality::Acceptable => 0.70,
                GeometryQuality::Poor => 0.50,
                GeometryQuality::Degenerate => 0.20,
            },
            uncertainty_ellipse: UncertaintyEllipse {
                semi_major_axis_m: horizontal_acc * 1.5,
                semi_minor_axis_m: horizontal_acc * 0.8,
                orientation_deg: 0.0,
                confidence_level: 0.68,
            },
            quality_indicators: QualityIndicators {
                geometric_dilution_of_precision: match geometry_quality {
                    GeometryQuality::Excellent => 1.5,
                    GeometryQuality::Good => 3.0,
                    GeometryQuality::Acceptable => 7.0,
                    GeometryQuality::Poor => 15.0,
                    GeometryQuality::Degenerate => 50.0,
                },
                anchor_geometry_quality: geometry_quality,
                signal_quality_average: 180.0,
                timing_precision_ms: 1.0,
                solution_convergence: true,
                residual_error_m: horizontal_acc * 0.5,
            },
        }
    }
    
    /// Estimate accuracy for 2D positioning
    fn estimate_accuracy_2d(&self, anchors: &[Anchor], _position: &Vector3<f64>) -> AccuracyEstimate {
        let mut base_accuracy = self.estimate_accuracy(anchors, _position);
        
        // Increase uncertainty for 2D mode
        base_accuracy.horizontal_accuracy_m *= 1.5;
        base_accuracy.vertical_accuracy_m *= 3.0; // Much higher vertical uncertainty
        base_accuracy.overall_accuracy_m = (base_accuracy.horizontal_accuracy_m.powi(2) + 
                                           base_accuracy.vertical_accuracy_m.powi(2)).sqrt();
        base_accuracy.confidence_level *= 0.8;
        
        base_accuracy
    }
    
    /// Estimate depth using specified method
    fn estimate_depth(&self, anchors: &[Anchor], method: &DepthEstimationMethod) -> f64 {
        match method {
            DepthEstimationMethod::AverageAnchorDepth => {
                anchors.iter().map(|a| a.position.depth).sum::<f64>() / anchors.len() as f64
            }
            DepthEstimationMethod::WeightedAverage => {
                // Weight by signal quality (simulated)
                let total_weight: f64 = anchors.len() as f64;
                anchors.iter().map(|a| a.position.depth).sum::<f64>() / total_weight
            }
            DepthEstimationMethod::LastKnownDepth { depth } => *depth,
            DepthEstimationMethod::DefaultDepth { depth } => *depth,
            DepthEstimationMethod::ExternalSensor { .. } => {
                // Simulate external sensor reading
                0.0
            }
        }
    }
    
    /// Generate warnings based on positioning conditions
    fn generate_warnings(&self, anchors: &[Anchor], accuracy: &AccuracyEstimate) -> Vec<String> {
        let mut warnings = Vec::new();
        
        if accuracy.overall_accuracy_m > 5.0 {
            warnings.push(format!("Low accuracy: {:.1} m", accuracy.overall_accuracy_m));
        }
        
        if accuracy.confidence_level < 0.7 {
            warnings.push(format!("Low confidence: {:.0}%", accuracy.confidence_level * 100.0));
        }
        
        if anchors.len() < 4 {
            warnings.push(format!("Limited anchors: {} available", anchors.len()));
        }
        
        match accuracy.quality_indicators.anchor_geometry_quality {
            GeometryQuality::Poor | GeometryQuality::Degenerate => {
                warnings.push("Poor anchor geometry detected".to_string());
            }
            _ => {}
        }
        
        warnings
    }
    
    /// Update system health monitoring
    fn update_system_health(&mut self, anchors: &[Anchor]) {
        let now = SystemTime::now();
        
        // Update subsystem health
        self.system_health.subsystem_health.insert("positioning".to_string(), SubsystemHealth {
            status: if anchors.len() >= 3 { HealthStatus::Healthy } else { HealthStatus::Degraded },
            last_check: now,
            error_count: 0,
            performance_score: (anchors.len() as f32 / 4.0).min(1.0),
            details: format!("{} anchors available", anchors.len()),
        });
        
        self.system_health.subsystem_health.insert("communication".to_string(), SubsystemHealth {
            status: HealthStatus::Healthy,
            last_check: now,
            error_count: 0,
            performance_score: 0.95,
            details: "All transceivers operational".to_string(),
        });
        
        // Update overall status
        let worst_status = self.system_health.subsystem_health.values()
            .map(|h| &h.status)
            .max()
            .unwrap_or(&HealthStatus::Healthy);
        
        self.system_health.overall_status = worst_status.clone();
        self.system_health.last_update = now;
    }
    
    /// Record performance snapshot
    fn record_performance_snapshot(&mut self, result: &Result<PositioningResult, PositioningError>, 
                                  computation_time: f64, anchor_count: u8) {
        let snapshot = PerformanceSnapshot {
            timestamp: SystemTime::now(),
            mode: self.current_mode.clone(),
            success_rate: if result.is_ok() { 1.0 } else { 0.0 },
            accuracy: result.as_ref().map(|r| r.accuracy_estimate.overall_accuracy_m).unwrap_or(f32::INFINITY),
            computation_time: computation_time as f32,
            anchor_count,
        };
        
        self.performance_history.push(snapshot);
        
        // Limit history size
        if self.performance_history.len() > self.max_history_size {
            self.performance_history.remove(0);
        }
    }
    
    /// Get current system health
    pub fn get_system_health(&self) -> &SystemHealth {
        &self.system_health
    }
    
    /// Get current operating mode
    pub fn get_current_mode(&self) -> &OperatingMode {
        &self.current_mode
    }
    
    /// Get performance statistics
    pub fn get_performance_statistics(&self) -> PerformanceStatistics {
        if self.performance_history.is_empty() {
            return PerformanceStatistics::default();
        }
        
        let total_attempts = self.performance_history.len();
        let successful_attempts = self.performance_history.iter().filter(|s| s.success_rate > 0.0).count();
        
        let avg_accuracy = self.performance_history.iter()
            .filter(|s| s.accuracy.is_finite())
            .map(|s| s.accuracy)
            .sum::<f32>() / successful_attempts.max(1) as f32;
        
        let avg_computation_time = self.performance_history.iter()
            .map(|s| s.computation_time)
            .sum::<f32>() / total_attempts as f32;
        
        PerformanceStatistics {
            total_attempts,
            successful_attempts,
            success_rate: successful_attempts as f32 / total_attempts as f32,
            average_accuracy_m: avg_accuracy,
            average_computation_time_ms: avg_computation_time,
            mode_distribution: self.calculate_mode_distribution(),
        }
    }
    
    /// Calculate mode distribution
    fn calculate_mode_distribution(&self) -> HashMap<String, f32> {
        let mut distribution = HashMap::new();
        let total = self.performance_history.len() as f32;
        
        if total == 0.0 {
            return distribution;
        }
        
        for snapshot in &self.performance_history {
            let mode_name = format!("{:?}", snapshot.mode).split('{').next().unwrap_or("Unknown").to_string();
            *distribution.entry(mode_name).or_insert(0.0) += 1.0 / total;
        }
        
        distribution
    }
    
    /// Helper functions
    fn calculate_distance(&self, pos1: &Position, pos2: &Position) -> f64 {
        let lat1_rad = pos1.lat.to_radians();
        let lat2_rad = pos2.lat.to_radians();
        let delta_lat = (pos2.lat - pos1.lat).to_radians();
        let delta_lon = (pos2.lon - pos1.lon).to_radians();
        
        let a = (delta_lat / 2.0).sin().powi(2) + 
                lat1_rad.cos() * lat2_rad.cos() * (delta_lon / 2.0).sin().powi(2);
        let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
        
        6371000.0 * c // Earth radius in meters
    }
    
    fn calculate_bearing(&self, from: &Position, to: &Position) -> f64 {
        let lat1_rad = from.lat.to_radians();
        let lat2_rad = to.lat.to_radians();
        let delta_lon = (to.lon - from.lon).to_radians();
        
        let y = delta_lon.sin() * lat2_rad.cos();
        let x = lat1_rad.cos() * lat2_rad.sin() - lat1_rad.sin() * lat2_rad.cos() * delta_lon.cos();
        
        y.atan2(x).to_degrees()
    }
    
    fn estimate_position_from_range_bearing(&self, anchor_pos: &Position, range_m: f64, bearing_deg: f64) -> Position {
        let bearing_rad = bearing_deg.to_radians();
        let earth_radius = 6371000.0;
        
        let lat1_rad = anchor_pos.lat.to_radians();
        let lon1_rad = anchor_pos.lon.to_radians();
        
        let lat2_rad = (lat1_rad.sin() * (range_m / earth_radius).cos() + 
                       lat1_rad.cos() * (range_m / earth_radius).sin() * bearing_rad.cos()).asin();
        
        let lon2_rad = lon1_rad + (bearing_rad.sin() * (range_m / earth_radius).sin() / lat2_rad.cos()).atan2(
            (range_m / earth_radius).cos() - lat1_rad.sin() * lat2_rad.sin());
        
        Position {
            lat: lat2_rad.to_degrees(),
            lon: lon2_rad.to_degrees(),
            depth: anchor_pos.depth,
        }
    }
}

impl Default for GracefulDegradationManager {
    fn default() -> Self {
        Self::new()
    }
}

impl SystemHealth {
    fn new() -> Self {
        Self {
            overall_status: HealthStatus::Healthy,
            subsystem_health: HashMap::new(),
            performance_metrics: PerformanceMetrics {
                positioning_success_rate: 1.0,
                average_computation_time_ms: 50.0,
                average_accuracy_m: 1.0,
                throughput_positions_per_second: 10.0,
                uptime_percentage: 100.0,
            },
            resource_usage: ResourceUsage {
                memory_usage_percent: 25.0,
                cpu_usage_percent: 15.0,
                battery_level_percent: Some(85.0),
                storage_usage_percent: 10.0,
                network_bandwidth_usage_percent: 5.0,
            },
            last_update: SystemTime::now(),
        }
    }
}

/// Performance statistics
#[derive(Debug, Clone, Default)]
pub struct PerformanceStatistics {
    pub total_attempts: usize,
    pub successful_attempts: usize,
    pub success_rate: f32,
    pub average_accuracy_m: f32,
    pub average_computation_time_ms: f32,
    pub mode_distribution: HashMap<String, f32>,
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_operating_mode_determination() {
        let mut manager = GracefulDegradationManager::new();
        
        // Test with 4 anchors - should use Full3D
        let anchors = vec![
            Anchor { id: "1".to_string(), timestamp: 1000, position: Position { lat: 0.0, lon: 0.0, depth: 0.0 } },
            Anchor { id: "2".to_string(), timestamp: 1000, position: Position { lat: 0.001, lon: 0.0, depth: 0.0 } },
            Anchor { id: "3".to_string(), timestamp: 1000, position: Position { lat: 0.0, lon: 0.001, depth: 0.0 } },
            Anchor { id: "4".to_string(), timestamp: 1000, position: Position { lat: 0.001, lon: 0.001, depth: 10.0 } },
        ];
        
        let mode = manager.determine_operating_mode(&anchors, 1000).unwrap();
        match mode {
            OperatingMode::Full3D { anchor_count, .. } => assert_eq!(anchor_count, 4),
            _ => panic!("Expected Full3D mode"),
        }
    }
    
    #[test]
    fn test_graceful_degradation() {
        let mut manager = GracefulDegradationManager::new();
        
        // Test with 2 anchors - should degrade to RangeBearing
        let anchors = vec![
            Anchor { id: "1".to_string(), timestamp: 1000, position: Position { lat: 0.0, lon: 0.0, depth: 0.0 } },
            Anchor { id: "2".to_string(), timestamp: 1000, position: Position { lat: 0.001, lon: 0.0, depth: 0.0 } },
        ];
        
        let mode = manager.determine_operating_mode(&anchors, 1000).unwrap();
        match mode {
            OperatingMode::RangeBearing { .. } => {}, // Expected
            _ => panic!("Expected RangeBearing mode, got {:?}", mode),
        }
    }
    
    #[test]
    fn test_geometry_quality_assessment() {
        let manager = GracefulDegradationManager::new();
        
        // Test with well-spread anchors
        let good_anchors = vec![
            Anchor { id: "1".to_string(), timestamp: 1000, position: Position { lat: 0.0, lon: 0.0, depth: 0.0 } },
            Anchor { id: "2".to_string(), timestamp: 1000, position: Position { lat: 0.01, lon: 0.0, depth: 0.0 } },
            Anchor { id: "3".to_string(), timestamp: 1000, position: Position { lat: 0.0, lon: 0.01, depth: 0.0 } },
        ];
        
        let quality = manager.assess_geometry_quality(&good_anchors);
        assert!(quality >= GeometryQuality::Good);
        
        // Test with collinear anchors
        let bad_anchors = vec![
            Anchor { id: "1".to_string(), timestamp: 1000, position: Position { lat: 0.0, lon: 0.0, depth: 0.0 } },
            Anchor { id: "2".to_string(), timestamp: 1000, position: Position { lat: 0.001, lon: 0.0, depth: 0.0 } },
            Anchor { id: "3".to_string(), timestamp: 1000, position: Position { lat: 0.002, lon: 0.0, depth: 0.0 } },
        ];
        
        let quality = manager.assess_geometry_quality(&bad_anchors);
        assert!(quality <= GeometryQuality::Poor);
    }
}