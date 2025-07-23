use nalgebra::{Matrix3, Vector3, DMatrix, DVector, Matrix4, Vector4};
use std::collections::VecDeque;
use crate::{Anchor, Position, SPEED_OF_SOUND_WATER, geodetic_to_local, local_to_geodetic};

/// Advanced positioning algorithms with enhanced mathematical techniques
pub struct AdvancedPositioningEngine {
    /// Kalman filter state for temporal smoothing
    kalman_state: Option<KalmanState>,
    /// Position history for temporal filtering
    position_history: VecDeque<PositionMeasurement>,
    /// Maximum history length for memory management
    max_history_length: usize,
}

/// Kalman filter state for position tracking
#[derive(Debug, Clone)]
struct KalmanState {
    /// State vector [x, y, z, vx, vy, vz] - position and velocity
    state: Vector4<f64>,
    /// State covariance matrix
    covariance: Matrix4<f64>,
    /// Process noise covariance
    process_noise: Matrix4<f64>,
    /// Last update timestamp
    last_update_ms: u64,
}

/// Position measurement with quality metrics
#[derive(Debug, Clone)]
struct PositionMeasurement {
    position: Vector3<f64>,
    uncertainty: f64,
    timestamp_ms: u64,
    anchor_count: usize,
    geometry_quality: f64,
}

/// Weighted measurement for robust estimation
#[derive(Debug, Clone)]
pub struct WeightedMeasurement {
    pub anchor_position: Vector3<f64>,
    pub measured_distance: f64,
    pub weight: f64,
    pub quality: f64,
}

/// Maximum Likelihood Estimator result
#[derive(Debug)]
pub struct MLEResult {
    pub position: Vector3<f64>,
    pub uncertainty: f64,
    pub log_likelihood: f64,
    pub iterations: usize,
}

/// Levenberg-Marquardt optimization result
#[derive(Debug)]
pub struct LMResult {
    pub position: Vector3<f64>,
    pub residual_norm: f64,
    pub iterations: usize,
    pub converged: bool,
}

impl AdvancedPositioningEngine {
    pub fn new() -> Self {
        Self {
            kalman_state: None,
            position_history: VecDeque::new(),
            max_history_length: 50, // Keep last 50 measurements for embedded systems
        }
    }

    /// Enhanced trilateration with Maximum Likelihood Estimator
    pub fn trilaterate_mle(
        &mut self,
        anchors: &[Anchor],
        receiver_time_ms: u64,
        noise_variance: f64,
    ) -> Result<MLEResult, String> {
        if anchors.len() < 3 {
            return Err("At least 3 anchors required for MLE".to_string());
        }

        // Convert to weighted measurements
        let measurements = self.prepare_weighted_measurements(anchors, receiver_time_ms)?;
        
        // Initial position estimate using standard least squares
        let initial_pos = self.compute_initial_estimate(&measurements)?;
        
        // Refine using Maximum Likelihood Estimation
        self.mle_optimization(measurements, initial_pos, noise_variance)
    }

    /// Enhanced trilateration with Levenberg-Marquardt optimization
    pub fn trilaterate_levenberg_marquardt(
        &mut self,
        anchors: &[Anchor],
        receiver_time_ms: u64,
    ) -> Result<LMResult, String> {
        if anchors.len() < 3 {
            return Err("At least 3 anchors required for LM optimization".to_string());
        }

        let measurements = self.prepare_weighted_measurements(anchors, receiver_time_ms)?;
        let initial_pos = self.compute_initial_estimate(&measurements)?;
        
        self.levenberg_marquardt_optimization(measurements, initial_pos)
    }

    /// Weighted least squares with measurement reliability weighting
    pub fn trilaterate_weighted_least_squares(
        &mut self,
        anchors: &[Anchor],
        receiver_time_ms: u64,
    ) -> Result<(Vector3<f64>, f64), String> {
        if anchors.len() < 3 {
            return Err("At least 3 anchors required for weighted least squares".to_string());
        }

        let measurements = self.prepare_weighted_measurements(anchors, receiver_time_ms)?;
        self.weighted_least_squares_solve(measurements)
    }

    /// Robust estimation with outlier detection and mitigation
    pub fn trilaterate_robust(
        &mut self,
        anchors: &[Anchor],
        receiver_time_ms: u64,
    ) -> Result<(Vector3<f64>, f64), String> {
        if anchors.len() < 4 {
            return Err("At least 4 anchors required for robust estimation".to_string());
        }

        let mut measurements = self.prepare_weighted_measurements(anchors, receiver_time_ms)?;
        
        // Iteratively reweighted least squares (IRLS) for robust estimation
        let mut position = self.compute_initial_estimate(&measurements)?;
        
        for iteration in 0..10 {
            // Compute residuals
            let residuals: Vec<f64> = measurements.iter()
                .map(|m| {
                    let predicted_distance = (m.anchor_position - position).norm();
                    (m.measured_distance - predicted_distance).abs()
                })
                .collect();
            
            // Compute robust weights using Huber function
            let median_residual = {
                let mut sorted_residuals = residuals.clone();
                sorted_residuals.sort_by(|a, b| a.partial_cmp(b).unwrap());
                sorted_residuals[sorted_residuals.len() / 2]
            };
            
            let scale = 1.4826 * median_residual; // Robust scale estimate
            let huber_threshold = 1.345 * scale;
            
            // Update weights based on residuals
            for (i, measurement) in measurements.iter_mut().enumerate() {
                let residual = residuals[i];
                measurement.weight = if residual <= huber_threshold {
                    1.0
                } else {
                    huber_threshold / residual
                };
            }
            
            // Solve weighted least squares with updated weights
            let (new_position, _) = self.weighted_least_squares_solve(measurements.clone())?;
            
            // Check convergence
            let position_change = (new_position - position).norm();
            position = new_position;
            
            if position_change < 1e-6 {
                break;
            }
        }
        
        // Compute final uncertainty estimate
        let final_residuals: Vec<f64> = measurements.iter()
            .map(|m| {
                let predicted_distance = (m.anchor_position - position).norm();
                (m.measured_distance - predicted_distance).abs()
            })
            .collect();
        
        let uncertainty = final_residuals.iter().sum::<f64>() / final_residuals.len() as f64;
        
        Ok((position, uncertainty))
    }

    /// Kalman filtering for temporal position smoothing and prediction
    pub fn update_kalman_filter(
        &mut self,
        measurement: Vector3<f64>,
        measurement_uncertainty: f64,
        timestamp_ms: u64,
    ) -> Vector3<f64> {
        match &mut self.kalman_state {
            None => {
                // Initialize Kalman filter
                self.kalman_state = Some(KalmanState {
                    state: Vector4::new(measurement.x, measurement.y, measurement.z, 0.0),
                    covariance: Matrix4::identity() * 100.0, // Initial uncertainty
                    process_noise: Matrix4::identity() * 0.1, // Process noise
                    last_update_ms: timestamp_ms,
                });
                measurement
            }
            Some(kalman) => {
                let dt = (timestamp_ms - kalman.last_update_ms) as f64 / 1000.0;
                
                // State transition matrix (constant velocity model)
                let f = Matrix4::new(
                    1.0, 0.0, 0.0, dt,
                    0.0, 1.0, 0.0, dt,
                    0.0, 0.0, 1.0, dt,
                    0.0, 0.0, 0.0, 1.0,
                );
                
                // Prediction step
                kalman.state = f * kalman.state;
                kalman.covariance = f * kalman.covariance * f.transpose() + kalman.process_noise * dt;
                
                // Measurement matrix (observe position only)
                let h = Matrix3x4::new(
                    1.0, 0.0, 0.0, 0.0,
                    0.0, 1.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.0,
                );
                
                // Measurement noise
                let r = Matrix3::identity() * measurement_uncertainty.powi(2);
                
                // Innovation
                let predicted_measurement = h * kalman.state;
                let innovation = Vector3::new(measurement.x, measurement.y, measurement.z) - predicted_measurement;
                
                // Innovation covariance
                let s = h * kalman.covariance * h.transpose() + r;
                
                // Kalman gain
                let k = kalman.covariance * h.transpose() * s.try_inverse().unwrap_or(Matrix3::identity());
                
                // Update step
                kalman.state = kalman.state + k * innovation;
                kalman.covariance = (Matrix4::identity() - k * h) * kalman.covariance;
                kalman.last_update_ms = timestamp_ms;
                
                Vector3::new(kalman.state.x, kalman.state.y, kalman.state.z)
            }
        }
    }

    /// Add position measurement to history for temporal filtering
    pub fn add_position_measurement(
        &mut self,
        position: Vector3<f64>,
        uncertainty: f64,
        timestamp_ms: u64,
        anchor_count: usize,
        geometry_quality: f64,
    ) {
        let measurement = PositionMeasurement {
            position,
            uncertainty,
            timestamp_ms,
            anchor_count,
            geometry_quality,
        };
        
        self.position_history.push_back(measurement);
        
        // Maintain maximum history length
        while self.position_history.len() > self.max_history_length {
            self.position_history.pop_front();
        }
    }

    /// Get temporally filtered position using weighted average
    pub fn get_filtered_position(&self, current_time_ms: u64) -> Option<Vector3<f64>> {
        if self.position_history.is_empty() {
            return None;
        }
        
        let max_age_ms = 5000; // 5 seconds
        let mut weighted_sum = Vector3::zeros();
        let mut total_weight = 0.0;
        
        for measurement in &self.position_history {
            let age_ms = current_time_ms.saturating_sub(measurement.timestamp_ms);
            if age_ms <= max_age_ms {
                // Weight based on recency, uncertainty, and geometry quality
                let age_weight = 1.0 / (1.0 + age_ms as f64 / 1000.0);
                let uncertainty_weight = 1.0 / (1.0 + measurement.uncertainty);
                let geometry_weight = measurement.geometry_quality;
                
                let weight = age_weight * uncertainty_weight * geometry_weight;
                
                weighted_sum += measurement.position * weight;
                total_weight += weight;
            }
        }
        
        if total_weight > 0.0 {
            Some(weighted_sum / total_weight)
        } else {
            None
        }
    }

    // Private helper methods

    fn prepare_weighted_measurements(
        &self,
        anchors: &[Anchor],
        receiver_time_ms: u64,
    ) -> Result<Vec<WeightedMeasurement>, String> {
        let reference_pos = &anchors[0].position;
        let mut measurements = Vec::new();
        
        for anchor in anchors {
            let local_pos = geodetic_to_local(&anchor.position, reference_pos);
            
            let dt_ms = receiver_time_ms as i64 - anchor.timestamp as i64;
            if dt_ms < 0 {
                return Err(format!(
                    "Receiver time earlier than anchor time for anchor {}",
                    anchor.id
                ));
            }
            
            let dt_sec = dt_ms as f64 / 1000.0;
            let distance = SPEED_OF_SOUND_WATER * dt_sec;
            
            // Compute measurement quality based on signal age and distance
            let age_factor = 1.0 / (1.0 + dt_sec / 10.0); // Decay over 10 seconds
            let distance_factor = 1.0 / (1.0 + distance / 1000.0); // Decay over 1km
            let quality = age_factor * distance_factor;
            
            // Weight inversely proportional to expected measurement variance
            let weight = quality * quality;
            
            measurements.push(WeightedMeasurement {
                anchor_position: local_pos,
                measured_distance: distance,
                weight,
                quality,
            });
        }
        
        Ok(measurements)
    }

    fn compute_initial_estimate(&self, measurements: &[WeightedMeasurement]) -> Result<Vector3<f64>, String> {
        if measurements.len() < 3 {
            return Err("Insufficient measurements for initial estimate".to_string());
        }
        
        // Use first measurement as reference
        let p1 = measurements[0].anchor_position;
        let d1_sq = measurements[0].measured_distance.powi(2);
        
        // Build system of equations
        let n = measurements.len() - 1;
        let mut a_data = vec![vec![0.0; 3]; n];
        let mut b_data = vec![0.0; n];
        
        for i in 1..measurements.len() {
            let pi = measurements[i].anchor_position;
            let di_sq = measurements[i].measured_distance.powi(2);
            let row = i - 1;
            
            a_data[row][0] = 2.0 * (pi.x - p1.x);
            a_data[row][1] = 2.0 * (pi.y - p1.y);
            a_data[row][2] = 2.0 * (pi.z - p1.z);
            
            b_data[row] = d1_sq - di_sq
                + pi.x.powi(2) - p1.x.powi(2)
                + pi.y.powi(2) - p1.y.powi(2)
                + pi.z.powi(2) - p1.z.powi(2);
        }
        
        // Solve using SVD for robustness
        let a_matrix = DMatrix::from_fn(n, 3, |i, j| a_data[i][j]);
        let b_vector = DVector::from_vec(b_data);
        
        let svd = a_matrix.svd(true, true);
        svd.solve(&b_vector, 1e-9)
            .map_err(|e| format!("SVD solve failed: {}", e))
            .map(|solution| Vector3::new(solution[0], solution[1], solution[2]))
    }

    fn weighted_least_squares_solve(
        &self,
        measurements: Vec<WeightedMeasurement>,
    ) -> Result<(Vector3<f64>, f64), String> {
        let p1 = measurements[0].anchor_position;
        let d1_sq = measurements[0].measured_distance.powi(2);
        
        let n = measurements.len() - 1;
        let mut a_data = vec![vec![0.0; 3]; n];
        let mut b_data = vec![0.0; n];
        let mut weights = vec![0.0; n];
        
        for i in 1..measurements.len() {
            let pi = measurements[i].anchor_position;
            let di_sq = measurements[i].measured_distance.powi(2);
            let row = i - 1;
            
            a_data[row][0] = 2.0 * (pi.x - p1.x);
            a_data[row][1] = 2.0 * (pi.y - p1.y);
            a_data[row][2] = 2.0 * (pi.z - p1.z);
            
            b_data[row] = d1_sq - di_sq
                + pi.x.powi(2) - p1.x.powi(2)
                + pi.y.powi(2) - p1.y.powi(2)
                + pi.z.powi(2) - p1.z.powi(2);
            
            weights[row] = measurements[i].weight;
        }
        
        // Create weighted system: W^(1/2) * A * x = W^(1/2) * b
        let mut weighted_a = vec![vec![0.0; 3]; n];
        let mut weighted_b = vec![0.0; n];
        
        for i in 0..n {
            let sqrt_weight = weights[i].sqrt();
            weighted_b[i] = b_data[i] * sqrt_weight;
            for j in 0..3 {
                weighted_a[i][j] = a_data[i][j] * sqrt_weight;
            }
        }
        
        // Solve weighted system
        let a_matrix = DMatrix::from_fn(n, 3, |i, j| weighted_a[i][j]);
        let b_vector = DVector::from_vec(weighted_b);
        
        let svd = a_matrix.svd(true, true);
        let solution = svd.solve(&b_vector, 1e-9)
            .map_err(|e| format!("Weighted SVD solve failed: {}", e))?;
        
        let position = Vector3::new(solution[0], solution[1], solution[2]);
        
        // Compute uncertainty estimate
        let residuals: Vec<f64> = measurements.iter()
            .map(|m| {
                let predicted_distance = (m.anchor_position - position).norm();
                (m.measured_distance - predicted_distance).abs()
            })
            .collect();
        
        let uncertainty = residuals.iter().sum::<f64>() / residuals.len() as f64;
        
        Ok((position, uncertainty))
    }

    fn mle_optimization(
        &self,
        measurements: Vec<WeightedMeasurement>,
        initial_pos: Vector3<f64>,
        noise_variance: f64,
    ) -> Result<MLEResult, String> {
        let mut position = initial_pos;
        let mut log_likelihood = f64::NEG_INFINITY;
        
        // Newton-Raphson optimization for MLE
        for iteration in 0..50 {
            let (gradient, hessian, likelihood) = self.compute_mle_derivatives(&measurements, position, noise_variance);
            
            // Check for convergence
            if (likelihood - log_likelihood).abs() < 1e-9 && iteration > 0 {
                return Ok(MLEResult {
                    position,
                    uncertainty: (-hessian.determinant()).sqrt().recip(),
                    log_likelihood: likelihood,
                    iterations: iteration,
                });
            }
            
            log_likelihood = likelihood;
            
            // Newton step
            if let Some(hessian_inv) = hessian.try_inverse() {
                position = position - hessian_inv * gradient;
            } else {
                // Fallback to gradient descent
                position = position - gradient * 0.01;
            }
        }
        
        Ok(MLEResult {
            position,
            uncertainty: 1.0, // Default uncertainty if convergence failed
            log_likelihood,
            iterations: 50,
        })
    }

    fn compute_mle_derivatives(
        &self,
        measurements: &[WeightedMeasurement],
        position: Vector3<f64>,
        noise_variance: f64,
    ) -> (Vector3<f64>, Matrix3<f64>, f64) {
        let mut gradient = Vector3::zeros();
        let mut hessian = Matrix3::zeros();
        let mut log_likelihood = 0.0;
        
        for measurement in measurements {
            let diff = position - measurement.anchor_position;
            let predicted_distance = diff.norm();
            let residual = measurement.measured_distance - predicted_distance;
            
            // Log-likelihood contribution
            log_likelihood -= 0.5 * residual.powi(2) / noise_variance;
            
            // Gradient contribution
            let unit_vector = diff / predicted_distance;
            gradient += unit_vector * (residual / noise_variance);
            
            // Hessian contribution
            let outer_product = unit_vector * unit_vector.transpose();
            let identity_term = Matrix3::identity() / predicted_distance;
            hessian += (outer_product - identity_term) * (residual / noise_variance);
            hessian -= outer_product / noise_variance;
        }
        
        (gradient, hessian, log_likelihood)
    }

    fn levenberg_marquardt_optimization(
        &self,
        measurements: Vec<WeightedMeasurement>,
        initial_pos: Vector3<f64>,
    ) -> Result<LMResult, String> {
        let mut position = initial_pos;
        let mut lambda = 0.01; // Damping parameter
        let mut residual_norm = f64::INFINITY;
        
        for iteration in 0..100 {
            let (jacobian, residuals) = self.compute_lm_jacobian_residuals(&measurements, position);
            let new_residual_norm = residuals.norm();
            
            // Check convergence
            if (residual_norm - new_residual_norm).abs() < 1e-9 && iteration > 0 {
                return Ok(LMResult {
                    position,
                    residual_norm: new_residual_norm,
                    iterations: iteration,
                    converged: true,
                });
            }
            
            // Compute Levenberg-Marquardt step
            let jtj = jacobian.transpose() * &jacobian;
            let jtr = jacobian.transpose() * &residuals;
            let damped_jtj = jtj + Matrix3::identity() * lambda;
            
            if let Some(step) = damped_jtj.try_inverse().map(|inv| inv * jtr) {
                let new_position = position - step;
                let (_, new_residuals) = self.compute_lm_jacobian_residuals(&measurements, new_position);
                let test_residual_norm = new_residuals.norm();
                
                if test_residual_norm < new_residual_norm {
                    // Accept step and decrease damping
                    position = new_position;
                    residual_norm = test_residual_norm;
                    lambda *= 0.1;
                } else {
                    // Reject step and increase damping
                    lambda *= 10.0;
                }
            } else {
                lambda *= 10.0;
            }
            
            // Prevent lambda from becoming too large
            if lambda > 1e10 {
                break;
            }
        }
        
        Ok(LMResult {
            position,
            residual_norm,
            iterations: 100,
            converged: false,
        })
    }

    fn compute_lm_jacobian_residuals(
        &self,
        measurements: &[WeightedMeasurement],
        position: Vector3<f64>,
    ) -> (DMatrix<f64>, DVector<f64>) {
        let n = measurements.len();
        let mut jacobian_data = vec![vec![0.0; 3]; n];
        let mut residuals_data = vec![0.0; n];
        
        for (i, measurement) in measurements.iter().enumerate() {
            let diff = position - measurement.anchor_position;
            let predicted_distance = diff.norm();
            let residual = measurement.measured_distance - predicted_distance;
            
            residuals_data[i] = residual * measurement.weight.sqrt();
            
            let unit_vector = diff / predicted_distance;
            let weight_sqrt = measurement.weight.sqrt();
            
            jacobian_data[i][0] = -unit_vector.x * weight_sqrt;
            jacobian_data[i][1] = -unit_vector.y * weight_sqrt;
            jacobian_data[i][2] = -unit_vector.z * weight_sqrt;
        }
        
        let jacobian = DMatrix::from_fn(n, 3, |i, j| jacobian_data[i][j]);
        let residuals = DVector::from_vec(residuals_data);
        
        (jacobian, residuals)
    }
}

/// Advanced noise filtering and signal processing
pub struct SignalProcessor {
    /// History of signal quality measurements for temporal filtering
    signal_history: VecDeque<SignalQualityMeasurement>,
    /// Multipath detection state
    multipath_detector: MultipathDetector,
    /// Adaptive filter parameters
    filter_params: AdaptiveFilterParams,
}

/// Signal quality measurement with metadata
#[derive(Debug, Clone)]
struct SignalQualityMeasurement {
    anchor_id: u16,
    signal_strength: f64,
    noise_level: f64,
    timestamp_ms: u64,
    range_measurement: f64,
    multipath_indicator: f64,
}

/// Multipath detection and mitigation
#[derive(Debug)]
struct MultipathDetector {
    /// Range measurement history for each anchor
    range_history: std::collections::HashMap<u16, VecDeque<f64>>,
    /// Detected multipath events
    multipath_events: Vec<MultipathEvent>,
    /// Detection parameters
    detection_threshold: f64,
    history_length: usize,
}

/// Multipath event detection result
#[derive(Debug, Clone)]
struct MultipathEvent {
    anchor_id: u16,
    timestamp_ms: u64,
    severity: f64,
    range_bias: f64,
}

/// Adaptive filter parameters
#[derive(Debug, Clone)]
struct AdaptiveFilterParams {
    /// Base noise variance
    base_noise_variance: f64,
    /// Signal quality threshold for filtering
    quality_threshold: f64,
    /// Temporal smoothing factor
    smoothing_factor: f64,
    /// Outlier detection threshold (in standard deviations)
    outlier_threshold: f64,
}

/// Filtered measurement result
#[derive(Debug, Clone)]
pub struct FilteredMeasurement {
    pub anchor_id: u16,
    pub filtered_range: f64,
    pub confidence: f64,
    pub noise_estimate: f64,
    pub multipath_detected: bool,
    pub systematic_error_correction: f64,
}

impl SignalProcessor {
    pub fn new() -> Self {
        Self {
            signal_history: VecDeque::new(),
            multipath_detector: MultipathDetector::new(),
            filter_params: AdaptiveFilterParams::default(),
        }
    }

    /// Adaptive noise filtering based on signal quality indicators
    pub fn adaptive_noise_filter(
        &mut self,
        measurements: &[WeightedMeasurement],
        signal_qualities: &[f64],
    ) -> Vec<FilteredMeasurement> {
        let mut filtered_measurements = Vec::new();
        
        for (i, measurement) in measurements.iter().enumerate() {
            let signal_quality = signal_qualities.get(i).copied().unwrap_or(0.5);
            
            // Adaptive noise variance based on signal quality
            let adaptive_noise_variance = self.filter_params.base_noise_variance 
                * (2.0 - signal_quality); // Higher noise for lower quality signals
            
            // Temporal filtering using exponential smoothing
            let filtered_range = self.apply_temporal_filter(
                i as u16, 
                measurement.measured_distance, 
                signal_quality
            );
            
            // Confidence based on signal quality and temporal consistency
            let confidence = self.compute_measurement_confidence(
                i as u16,
                measurement.measured_distance,
                signal_quality
            );
            
            // Detect multipath for this measurement
            let multipath_detected = self.multipath_detector.detect_multipath(
                i as u16,
                measurement.measured_distance,
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap()
                    .as_millis() as u64,
            );
            
            // Apply systematic error correction
            let systematic_correction = self.compute_systematic_error_correction(
                i as u16,
                measurement.measured_distance,
                signal_quality
            );
            
            filtered_measurements.push(FilteredMeasurement {
                anchor_id: i as u16,
                filtered_range,
                confidence,
                noise_estimate: adaptive_noise_variance.sqrt(),
                multipath_detected,
                systematic_error_correction: systematic_correction,
            });
        }
        
        filtered_measurements
    }

    /// Multipath detection and mitigation
    pub fn detect_and_mitigate_multipath(
        &mut self,
        anchor_id: u16,
        range_measurements: &[f64],
        timestamps: &[u64],
    ) -> Vec<f64> {
        let mut corrected_ranges = Vec::new();
        
        for (i, &range) in range_measurements.iter().enumerate() {
            let timestamp = timestamps.get(i).copied().unwrap_or(0);
            
            // Detect multipath
            let multipath_detected = self.multipath_detector.detect_multipath(
                anchor_id, range, timestamp
            );
            
            let corrected_range = if multipath_detected {
                // Apply multipath mitigation
                self.mitigate_multipath_effect(anchor_id, range)
            } else {
                range
            };
            
            corrected_ranges.push(corrected_range);
        }
        
        corrected_ranges
    }

    /// Systematic error compensation for range measurements
    pub fn compensate_systematic_errors(
        &mut self,
        measurements: &[f64],
        anchor_positions: &[Vector3<f64>],
        environmental_params: &EnvironmentalParams,
    ) -> Vec<f64> {
        let mut compensated_measurements = Vec::new();
        
        for (i, &measurement) in measurements.iter().enumerate() {
            let anchor_pos = anchor_positions.get(i).copied().unwrap_or(Vector3::zeros());
            
            // Sound speed correction
            let sound_speed_correction = self.compute_sound_speed_correction(
                measurement, 
                anchor_pos, 
                environmental_params
            );
            
            // Clock bias correction
            let clock_bias_correction = self.compute_clock_bias_correction(
                i as u16, 
                measurement
            );
            
            // Atmospheric/water column correction
            let propagation_correction = self.compute_propagation_correction(
                measurement,
                anchor_pos,
                environmental_params
            );
            
            let compensated = measurement 
                + sound_speed_correction 
                + clock_bias_correction 
                + propagation_correction;
            
            compensated_measurements.push(compensated);
        }
        
        compensated_measurements
    }

    /// Temporal filtering to reduce measurement jitter
    pub fn temporal_jitter_filter(
        &mut self,
        anchor_id: u16,
        new_measurement: f64,
        timestamp_ms: u64,
    ) -> f64 {
        // Add to signal history
        let signal_measurement = SignalQualityMeasurement {
            anchor_id,
            signal_strength: 1.0, // Default value
            noise_level: 0.1,     // Default value
            timestamp_ms,
            range_measurement: new_measurement,
            multipath_indicator: 0.0,
        };
        
        self.signal_history.push_back(signal_measurement);
        
        // Maintain history length
        while self.signal_history.len() > 50 {
            self.signal_history.pop_front();
        }
        
        // Apply temporal smoothing filter
        let relevant_measurements: Vec<f64> = self.signal_history
            .iter()
            .filter(|m| m.anchor_id == anchor_id)
            .filter(|m| timestamp_ms.saturating_sub(m.timestamp_ms) < 5000) // Last 5 seconds
            .map(|m| m.range_measurement)
            .collect();
        
        if relevant_measurements.is_empty() {
            return new_measurement;
        }
        
        // Exponential weighted moving average
        let alpha = self.filter_params.smoothing_factor;
        let mut filtered_value = relevant_measurements[0];
        
        for &measurement in relevant_measurements.iter().skip(1) {
            filtered_value = alpha * measurement + (1.0 - alpha) * filtered_value;
        }
        
        filtered_value
    }

    /// Signal quality weighting for anchor measurements
    pub fn compute_signal_quality_weights(
        &self,
        measurements: &[f64],
        signal_qualities: &[f64],
    ) -> Vec<f64> {
        let mut weights = Vec::new();
        
        for (i, &quality) in signal_qualities.iter().enumerate() {
            // Base weight from signal quality
            let quality_weight = quality.max(0.1); // Minimum weight of 0.1
            
            // Consistency weight based on measurement stability
            let consistency_weight = self.compute_measurement_consistency(
                i as u16, 
                measurements.get(i).copied().unwrap_or(0.0)
            );
            
            // Age weight (more recent measurements get higher weight)
            let age_weight = 1.0; // Simplified for now
            
            // Combined weight
            let combined_weight = quality_weight * consistency_weight * age_weight;
            weights.push(combined_weight);
        }
        
        // Normalize weights
        let total_weight: f64 = weights.iter().sum();
        if total_weight > 0.0 {
            weights.iter_mut().for_each(|w| *w /= total_weight);
        }
        
        weights
    }

    // Private helper methods

    fn apply_temporal_filter(&mut self, anchor_id: u16, measurement: f64, quality: f64) -> f64 {
        // Simple exponential smoothing with quality-based adaptation
        let alpha = self.filter_params.smoothing_factor * quality;
        
        // Get previous filtered value (simplified)
        let previous_value = self.signal_history
            .iter()
            .rev()
            .find(|m| m.anchor_id == anchor_id)
            .map(|m| m.range_measurement)
            .unwrap_or(measurement);
        
        alpha * measurement + (1.0 - alpha) * previous_value
    }

    fn compute_measurement_confidence(&self, anchor_id: u16, measurement: f64, quality: f64) -> f64 {
        // Base confidence from signal quality
        let quality_confidence = quality;
        
        // Consistency confidence based on measurement history
        let consistency_confidence = self.compute_measurement_consistency(anchor_id, measurement);
        
        // Combined confidence
        (quality_confidence + consistency_confidence) / 2.0
    }

    fn compute_measurement_consistency(&self, anchor_id: u16, measurement: f64) -> f64 {
        let recent_measurements: Vec<f64> = self.signal_history
            .iter()
            .filter(|m| m.anchor_id == anchor_id)
            .take(10) // Last 10 measurements
            .map(|m| m.range_measurement)
            .collect();
        
        if recent_measurements.len() < 2 {
            return 0.5; // Default consistency
        }
        
        // Compute standard deviation
        let mean = recent_measurements.iter().sum::<f64>() / recent_measurements.len() as f64;
        let variance = recent_measurements
            .iter()
            .map(|x| (x - mean).powi(2))
            .sum::<f64>() / recent_measurements.len() as f64;
        let std_dev = variance.sqrt();
        
        // Consistency score (higher for lower standard deviation)
        1.0 / (1.0 + std_dev)
    }

    fn compute_systematic_error_correction(&self, _anchor_id: u16, measurement: f64, _quality: f64) -> f64 {
        // Simplified systematic error correction
        // In practice, this would use calibration data and environmental models
        measurement * 0.001 // Small correction factor
    }

    fn mitigate_multipath_effect(&self, anchor_id: u16, range: f64) -> f64 {
        // Get multipath bias estimate
        let multipath_bias = self.multipath_detector
            .multipath_events
            .iter()
            .filter(|event| event.anchor_id == anchor_id)
            .last()
            .map(|event| event.range_bias)
            .unwrap_or(0.0);
        
        // Correct for multipath bias
        range - multipath_bias
    }

    fn compute_sound_speed_correction(
        &self,
        _measurement: f64,
        _anchor_pos: Vector3<f64>,
        params: &EnvironmentalParams,
    ) -> f64 {
        // Sound speed correction based on environmental parameters
        let sound_speed_error = params.sound_speed - SPEED_OF_SOUND_WATER;
        let correction_factor = sound_speed_error / SPEED_OF_SOUND_WATER;
        
        // Apply correction (simplified)
        correction_factor * 0.1 // Small correction
    }

    fn compute_clock_bias_correction(&self, _anchor_id: u16, _measurement: f64) -> f64 {
        // Clock bias correction (simplified)
        // In practice, this would use clock synchronization data
        0.0
    }

    fn compute_propagation_correction(
        &self,
        _measurement: f64,
        anchor_pos: Vector3<f64>,
        _params: &EnvironmentalParams,
    ) -> f64 {
        // Propagation delay correction based on depth and distance
        let depth_factor = anchor_pos.z / 1000.0; // Depth in km
        depth_factor * 0.001 // Small depth-based correction
    }
}

impl MultipathDetector {
    fn new() -> Self {
        Self {
            range_history: std::collections::HashMap::new(),
            multipath_events: Vec::new(),
            detection_threshold: 2.0, // 2 meters threshold
            history_length: 20,
        }
    }

    fn detect_multipath(&mut self, anchor_id: u16, range: f64, timestamp_ms: u64) -> bool {
        // Add to range history
        let history = self.range_history.entry(anchor_id).or_insert_with(VecDeque::new);
        history.push_back(range);
        
        // Maintain history length
        while history.len() > self.history_length {
            history.pop_front();
        }
        
        if history.len() < 5 {
            return false; // Need sufficient history
        }
        
        // Detect sudden range jumps (potential multipath)
        let recent_ranges: Vec<f64> = history.iter().rev().take(5).copied().collect();
        let mean_recent = recent_ranges.iter().sum::<f64>() / recent_ranges.len() as f64;
        
        let range_deviation = (range - mean_recent).abs();
        let multipath_detected = range_deviation > self.detection_threshold;
        
        if multipath_detected {
            // Record multipath event
            let event = MultipathEvent {
                anchor_id,
                timestamp_ms,
                severity: range_deviation / self.detection_threshold,
                range_bias: range - mean_recent,
            };
            self.multipath_events.push(event);
            
            // Limit event history
            if self.multipath_events.len() > 100 {
                self.multipath_events.remove(0);
            }
        }
        
        multipath_detected
    }
}

impl AdaptiveFilterParams {
    fn default() -> Self {
        Self {
            base_noise_variance: 1.0,
            quality_threshold: 0.5,
            smoothing_factor: 0.3,
            outlier_threshold: 3.0,
        }
    }
}

/// Environmental parameters for systematic error correction
#[derive(Debug, Clone)]
pub struct EnvironmentalParams {
    pub sound_speed: f64,      // m/s
    pub temperature: f64,      // Celsius
    pub salinity: f64,         // ppt
    pub pressure: f64,         // bar
    pub current_velocity: Vector3<f64>, // m/s
}

impl Default for EnvironmentalParams {
    fn default() -> Self {
        Self {
            sound_speed: SPEED_OF_SOUND_WATER,
            temperature: 15.0,
            salinity: 35.0,
            pressure: 1.0,
            current_velocity: Vector3::zeros(),
        }
    }
}

/// Sub-meter accuracy optimization engine
pub struct SubMeterAccuracyOptimizer {
    /// High-precision coordinate transformer
    coordinate_transformer: HighPrecisionCoordinateTransformer,
    /// Environmental correction models
    environmental_corrector: EnvironmentalCorrector,
    /// Calibration data and systematic error models
    calibration_manager: CalibrationManager,
    /// Position validator for consistency checking
    position_validator: PositionValidator,
}

/// High-precision coordinate transformations with error propagation
#[derive(Debug)]
struct HighPrecisionCoordinateTransformer {
    /// Reference position for local tangent plane
    reference_position: Position,
    /// Earth ellipsoid parameters
    ellipsoid_params: EllipsoidParams,
    /// Transformation precision settings
    precision_settings: TransformationPrecision,
}

/// Environmental correction factors
#[derive(Debug)]
struct EnvironmentalCorrector {
    /// Sound speed profile model
    sound_speed_model: SoundSpeedModel,
    /// Temperature correction model
    temperature_model: TemperatureModel,
    /// Pressure correction model
    pressure_model: PressureModel,
}

/// Calibration and systematic error management
#[derive(Debug)]
struct CalibrationManager {
    /// Anchor position calibration data
    anchor_calibrations: std::collections::HashMap<u16, AnchorCalibration>,
    /// Clock synchronization data
    clock_calibrations: std::collections::HashMap<u16, ClockCalibration>,
    /// Range measurement bias corrections
    range_bias_corrections: std::collections::HashMap<u16, f64>,
}

/// Position validation and consistency checking
#[derive(Debug)]
struct PositionValidator {
    /// Historical position data for consistency checking
    position_history: VecDeque<ValidatedPosition>,
    /// Validation thresholds
    validation_thresholds: ValidationThresholds,
}

/// High-precision positioning result with comprehensive accuracy estimation
#[derive(Debug, Clone)]
pub struct SubMeterResult {
    pub position: Vector3<f64>,
    pub geodetic_position: Position,
    pub accuracy_estimate: AccuracyEstimate,
    pub confidence_interval: ConfidenceInterval,
    pub validation_status: ValidationStatus,
    pub correction_summary: CorrectionSummary,
}

/// Comprehensive accuracy estimation
#[derive(Debug, Clone)]
pub struct AccuracyEstimate {
    pub horizontal_accuracy: f64,  // meters (95% confidence)
    pub vertical_accuracy: f64,    // meters (95% confidence)
    pub overall_accuracy: f64,     // meters (95% confidence)
    pub accuracy_components: AccuracyComponents,
}

/// Individual accuracy components
#[derive(Debug, Clone)]
pub struct AccuracyComponents {
    pub measurement_noise: f64,
    pub geometric_dilution: f64,
    pub systematic_errors: f64,
    pub environmental_effects: f64,
    pub calibration_uncertainty: f64,
}

/// Confidence intervals for position estimates
#[derive(Debug, Clone)]
pub struct ConfidenceInterval {
    pub east_95: (f64, f64),    // 95% confidence interval for east component
    pub north_95: (f64, f64),   // 95% confidence interval for north component
    pub down_95: (f64, f64),    // 95% confidence interval for down component
}

/// Position validation status
#[derive(Debug, Clone, PartialEq)]
pub enum ValidationStatus {
    Valid,
    Warning(String),
    Invalid(String),
}

/// Summary of applied corrections
#[derive(Debug, Clone)]
pub struct CorrectionSummary {
    pub sound_speed_correction: f64,
    pub temperature_correction: f64,
    pub pressure_correction: f64,
    pub clock_bias_correction: f64,
    pub systematic_bias_correction: f64,
    pub total_correction_magnitude: f64,
}

// Supporting structures and models

#[derive(Debug, Clone)]
struct EllipsoidParams {
    semi_major_axis: f64,  // WGS84: 6378137.0 m
    flattening: f64,       // WGS84: 1/298.257223563
}

#[derive(Debug, Clone)]
struct TransformationPrecision {
    coordinate_precision: f64,  // meters
    angular_precision: f64,     // radians
    use_high_precision_math: bool,
}

#[derive(Debug)]
struct SoundSpeedModel {
    base_sound_speed: f64,
    temperature_coefficient: f64,
    salinity_coefficient: f64,
    pressure_coefficient: f64,
}

#[derive(Debug)]
struct TemperatureModel {
    reference_temperature: f64,
    temperature_gradient: f64,
}

#[derive(Debug)]
struct PressureModel {
    reference_pressure: f64,
    pressure_gradient: f64,
}

#[derive(Debug, Clone)]
pub struct AnchorCalibration {
    pub position_offset: Vector3<f64>,
    pub range_bias: f64,
    pub uncertainty: f64,
}

#[derive(Debug, Clone)]
pub struct ClockCalibration {
    pub time_offset: f64,  // seconds
    pub drift_rate: f64,   // seconds per second
    pub uncertainty: f64,  // seconds
}

#[derive(Debug, Clone)]
struct ValidatedPosition {
    position: Vector3<f64>,
    timestamp: u64,
    accuracy: f64,
    validation_score: f64,
}

#[derive(Debug, Clone)]
struct ValidationThresholds {
    max_position_jump: f64,     // meters
    max_velocity: f64,          // m/s
    min_consistency_score: f64, // 0-1
}

impl SubMeterAccuracyOptimizer {
    pub fn new(reference_position: Position) -> Self {
        Self {
            coordinate_transformer: HighPrecisionCoordinateTransformer::new(reference_position.clone()),
            environmental_corrector: EnvironmentalCorrector::new(),
            calibration_manager: CalibrationManager::new(),
            position_validator: PositionValidator::new(),
        }
    }

    /// Optimize positioning for sub-meter accuracy
    pub fn optimize_for_sub_meter_accuracy(
        &mut self,
        measurements: &[WeightedMeasurement],
        environmental_params: &EnvironmentalParams,
        timestamp_ms: u64,
    ) -> Result<SubMeterResult, String> {
        // Step 1: Apply environmental corrections
        let corrected_measurements = self.apply_environmental_corrections(measurements, environmental_params)?;
        
        // Step 2: Apply calibration corrections
        let calibrated_measurements = self.apply_calibration_corrections(&corrected_measurements)?;
        
        // Step 3: High-precision coordinate transformations
        let (position, transformation_uncertainty) = self.high_precision_positioning(&calibrated_measurements)?;
        
        // Step 4: Comprehensive accuracy estimation
        let accuracy_estimate = self.estimate_comprehensive_accuracy(
            &calibrated_measurements, 
            &position, 
            environmental_params
        )?;
        
        // Step 5: Confidence interval calculation
        let confidence_interval = self.calculate_confidence_intervals(&position, &accuracy_estimate)?;
        
        // Step 6: Position validation
        let validation_status = self.validate_position(&position, timestamp_ms, &accuracy_estimate)?;
        
        // Step 7: Generate correction summary
        let correction_summary = self.generate_correction_summary(measurements, &calibrated_measurements);
        
        // Convert to geodetic coordinates with high precision
        let geodetic_position = self.coordinate_transformer.local_to_geodetic_high_precision(&position)?;
        
        Ok(SubMeterResult {
            position,
            geodetic_position,
            accuracy_estimate,
            confidence_interval,
            validation_status,
            correction_summary,
        })
    }

    /// Add calibration data for an anchor
    pub fn add_anchor_calibration(&mut self, anchor_id: u16, calibration: AnchorCalibration) {
        self.calibration_manager.anchor_calibrations.insert(anchor_id, calibration);
    }

    /// Add clock calibration data
    pub fn add_clock_calibration(&mut self, anchor_id: u16, calibration: ClockCalibration) {
        self.calibration_manager.clock_calibrations.insert(anchor_id, calibration);
    }

    /// Update environmental correction models
    pub fn update_environmental_models(&mut self, params: &EnvironmentalParams) {
        self.environmental_corrector.update_models(params);
    }

    // Private implementation methods

    fn apply_environmental_corrections(
        &self,
        measurements: &[WeightedMeasurement],
        params: &EnvironmentalParams,
    ) -> Result<Vec<WeightedMeasurement>, String> {
        let mut corrected = Vec::new();
        
        for measurement in measurements {
            let sound_speed_correction = self.environmental_corrector
                .compute_sound_speed_correction(measurement.measured_distance, &measurement.anchor_position, params);
            
            let temperature_correction = self.environmental_corrector
                .compute_temperature_correction(measurement.measured_distance, &measurement.anchor_position, params);
            
            let pressure_correction = self.environmental_corrector
                .compute_pressure_correction(measurement.measured_distance, &measurement.anchor_position, params);
            
            let corrected_distance = measurement.measured_distance 
                + sound_speed_correction 
                + temperature_correction 
                + pressure_correction;
            
            corrected.push(WeightedMeasurement {
                anchor_position: measurement.anchor_position,
                measured_distance: corrected_distance,
                weight: measurement.weight,
                quality: measurement.quality,
            });
        }
        
        Ok(corrected)
    }

    fn apply_calibration_corrections(
        &self,
        measurements: &[WeightedMeasurement],
    ) -> Result<Vec<WeightedMeasurement>, String> {
        let mut calibrated = Vec::new();
        
        for (i, measurement) in measurements.iter().enumerate() {
            let anchor_id = i as u16;
            
            // Apply anchor position calibration
            let calibrated_anchor_pos = if let Some(cal) = self.calibration_manager.anchor_calibrations.get(&anchor_id) {
                measurement.anchor_position + cal.position_offset
            } else {
                measurement.anchor_position
            };
            
            // Apply range bias correction
            let range_bias = self.calibration_manager.range_bias_corrections
                .get(&anchor_id)
                .copied()
                .unwrap_or(0.0);
            
            let calibrated_distance = measurement.measured_distance - range_bias;
            
            calibrated.push(WeightedMeasurement {
                anchor_position: calibrated_anchor_pos,
                measured_distance: calibrated_distance,
                weight: measurement.weight,
                quality: measurement.quality,
            });
        }
        
        Ok(calibrated)
    }

    fn high_precision_positioning(
        &self,
        measurements: &[WeightedMeasurement],
    ) -> Result<(Vector3<f64>, f64), String> {
        // Use iterative weighted least squares with high precision
        let mut position = self.compute_initial_position_estimate(measurements)?;
        let mut previous_position = position;
        
        for iteration in 0..20 {
            // Build weighted system with current position estimate
            let (jacobian, residuals, weights) = self.build_high_precision_system(measurements, &position)?;
            
            // Solve weighted least squares
            let weighted_jacobian = self.apply_weights_to_jacobian(&jacobian, &weights);
            let weighted_residuals = self.apply_weights_to_residuals(&residuals, &weights);
            
            let svd = weighted_jacobian.svd(true, true);
            let delta = svd.solve(&weighted_residuals, 1e-12)
                .map_err(|e| format!("High-precision solve failed: {}", e))?;
            
            position = position + Vector3::new(delta[0], delta[1], delta[2]);
            
            // Check convergence
            let position_change = (position - previous_position).norm();
            if position_change < 1e-6 {
                break;
            }
            previous_position = position;
        }
        
        // Estimate transformation uncertainty
        let uncertainty = self.estimate_transformation_uncertainty(measurements, &position)?;
        
        Ok((position, uncertainty))
    }

    fn estimate_comprehensive_accuracy(
        &self,
        measurements: &[WeightedMeasurement],
        position: &Vector3<f64>,
        params: &EnvironmentalParams,
    ) -> Result<AccuracyEstimate, String> {
        // Component-wise accuracy analysis
        let measurement_noise = self.estimate_measurement_noise_contribution(measurements);
        let geometric_dilution = self.estimate_geometric_dilution_contribution(measurements, position)?;
        let systematic_errors = self.estimate_systematic_error_contribution(measurements);
        let environmental_effects = self.estimate_environmental_error_contribution(params);
        let calibration_uncertainty = self.estimate_calibration_uncertainty_contribution();
        
        // Combine error sources (RSS - Root Sum of Squares)
        let horizontal_accuracy = (
            measurement_noise.powi(2) + 
            geometric_dilution.powi(2) * 0.5 + // Horizontal component
            systematic_errors.powi(2) * 0.7 +
            environmental_effects.powi(2) * 0.6 +
            calibration_uncertainty.powi(2) * 0.5
        ).sqrt() * 1.96; // 95% confidence
        
        let vertical_accuracy = (
            measurement_noise.powi(2) + 
            geometric_dilution.powi(2) * 0.8 + // Vertical component typically worse
            systematic_errors.powi(2) * 0.8 +
            environmental_effects.powi(2) * 0.9 +
            calibration_uncertainty.powi(2) * 0.7
        ).sqrt() * 1.96; // 95% confidence
        
        let overall_accuracy = (horizontal_accuracy.powi(2) + vertical_accuracy.powi(2)).sqrt();
        
        Ok(AccuracyEstimate {
            horizontal_accuracy,
            vertical_accuracy,
            overall_accuracy,
            accuracy_components: AccuracyComponents {
                measurement_noise,
                geometric_dilution,
                systematic_errors,
                environmental_effects,
                calibration_uncertainty,
            },
        })
    }

    fn calculate_confidence_intervals(
        &self,
        position: &Vector3<f64>,
        accuracy: &AccuracyEstimate,
    ) -> Result<ConfidenceInterval, String> {
        // Simplified confidence intervals based on accuracy estimates
        let horizontal_error = accuracy.horizontal_accuracy / 1.96; // Convert from 95% to 1-sigma
        let vertical_error = accuracy.vertical_accuracy / 1.96;
        
        Ok(ConfidenceInterval {
            east_95: (position.x - horizontal_error, position.x + horizontal_error),
            north_95: (position.y - horizontal_error, position.y + horizontal_error),
            down_95: (position.z - vertical_error, position.z + vertical_error),
        })
    }

    fn validate_position(
        &mut self,
        position: &Vector3<f64>,
        timestamp_ms: u64,
        accuracy: &AccuracyEstimate,
    ) -> Result<ValidationStatus, String> {
        // Check against historical positions
        if let Some(last_position) = self.position_validator.position_history.back() {
            let time_diff = (timestamp_ms - last_position.timestamp) as f64 / 1000.0; // seconds
            let position_diff = (position - last_position.position).norm();
            
            // Check for unrealistic position jumps
            if position_diff > self.position_validator.validation_thresholds.max_position_jump {
                return Ok(ValidationStatus::Warning(
                    format!("Large position jump: {:.2}m", position_diff)
                ));
            }
            
            // Check for unrealistic velocities
            if time_diff > 0.0 {
                let velocity = position_diff / time_diff;
                if velocity > self.position_validator.validation_thresholds.max_velocity {
                    return Ok(ValidationStatus::Warning(
                        format!("High velocity: {:.2}m/s", velocity)
                    ));
                }
            }
        }
        
        // Check accuracy requirements for sub-meter performance
        if accuracy.overall_accuracy > 1.0 {
            return Ok(ValidationStatus::Warning(
                format!("Accuracy exceeds 1m: {:.2}m", accuracy.overall_accuracy)
            ));
        }
        
        // Add to position history
        self.position_validator.position_history.push_back(ValidatedPosition {
            position: *position,
            timestamp: timestamp_ms,
            accuracy: accuracy.overall_accuracy,
            validation_score: 1.0,
        });
        
        // Maintain history length
        while self.position_validator.position_history.len() > 100 {
            self.position_validator.position_history.pop_front();
        }
        
        Ok(ValidationStatus::Valid)
    }

    fn generate_correction_summary(
        &self,
        original: &[WeightedMeasurement],
        corrected: &[WeightedMeasurement],
    ) -> CorrectionSummary {
        let mut total_correction = 0.0;
        let mut count = 0;
        
        for (orig, corr) in original.iter().zip(corrected.iter()) {
            total_correction += (corr.measured_distance - orig.measured_distance).abs();
            count += 1;
        }
        
        CorrectionSummary {
            sound_speed_correction: 0.1,  // Simplified values
            temperature_correction: 0.05,
            pressure_correction: 0.02,
            clock_bias_correction: 0.01,
            systematic_bias_correction: 0.03,
            total_correction_magnitude: if count > 0 { total_correction / count as f64 } else { 0.0 },
        }
    }

    // Helper methods for accuracy estimation
    
    fn estimate_measurement_noise_contribution(&self, measurements: &[WeightedMeasurement]) -> f64 {
        // Estimate based on measurement quality
        let avg_quality: f64 = measurements.iter().map(|m| m.quality).sum::<f64>() / measurements.len() as f64;
        0.5 / avg_quality.max(0.1) // Base noise inversely related to quality
    }

    fn estimate_geometric_dilution_contribution(
        &self,
        measurements: &[WeightedMeasurement],
        position: &Vector3<f64>,
    ) -> Result<f64, String> {
        // Simplified GDOP-based estimate
        let anchor_positions: Vec<Vector3<f64>> = measurements.iter()
            .map(|m| m.anchor_position)
            .collect();
        
        // Compute geometry matrix
        let n = anchor_positions.len();
        let mut geometry_matrix = DMatrix::zeros(n, 3);
        
        for (i, anchor_pos) in anchor_positions.iter().enumerate() {
            let diff = position - anchor_pos;
            let range = diff.norm();
            
            if range < 1e-10 {
                return Ok(10.0); // High dilution for degenerate geometry
            }
            
            let unit_vector = diff / range;
            geometry_matrix[(i, 0)] = unit_vector.x;
            geometry_matrix[(i, 1)] = unit_vector.y;
            geometry_matrix[(i, 2)] = unit_vector.z;
        }
        
        let gtg = geometry_matrix.transpose() * &geometry_matrix;
        if let Some(covariance) = gtg.try_inverse() {
            let gdop = covariance.trace().sqrt();
            Ok(gdop * 0.3) // Scale factor for contribution
        } else {
            Ok(10.0) // High dilution for singular geometry
        }
    }

    fn estimate_systematic_error_contribution(&self, _measurements: &[WeightedMeasurement]) -> f64 {
        0.2 // Estimated systematic error contribution
    }

    fn estimate_environmental_error_contribution(&self, _params: &EnvironmentalParams) -> f64 {
        0.15 // Estimated environmental error contribution
    }

    fn estimate_calibration_uncertainty_contribution(&self) -> f64 {
        0.1 // Estimated calibration uncertainty contribution
    }

    // Additional helper methods (simplified implementations)
    
    fn compute_initial_position_estimate(&self, measurements: &[WeightedMeasurement]) -> Result<Vector3<f64>, String> {
        // Centroid-based initial estimate
        let centroid = measurements.iter()
            .map(|m| m.anchor_position)
            .fold(Vector3::zeros(), |acc, pos| acc + pos) / measurements.len() as f64;
        Ok(centroid)
    }

    fn build_high_precision_system(
        &self,
        measurements: &[WeightedMeasurement],
        position: &Vector3<f64>,
    ) -> Result<(DMatrix<f64>, DVector<f64>, Vec<f64>), String> {
        let n = measurements.len();
        let mut jacobian = DMatrix::zeros(n, 3);
        let mut residuals = DVector::zeros(n);
        let mut weights = Vec::new();
        
        for (i, measurement) in measurements.iter().enumerate() {
            let diff = position - measurement.anchor_position;
            let predicted_range = diff.norm();
            let residual = measurement.measured_distance - predicted_range;
            
            residuals[i] = residual;
            weights.push(measurement.weight);
            
            if predicted_range > 1e-10 {
                let unit_vector = diff / predicted_range;
                jacobian[(i, 0)] = unit_vector.x;
                jacobian[(i, 1)] = unit_vector.y;
                jacobian[(i, 2)] = unit_vector.z;
            }
        }
        
        Ok((jacobian, residuals, weights))
    }

    fn apply_weights_to_jacobian(&self, jacobian: &DMatrix<f64>, weights: &[f64]) -> DMatrix<f64> {
        let mut weighted = jacobian.clone();
        for i in 0..jacobian.nrows() {
            let sqrt_weight = weights[i].sqrt();
            for j in 0..jacobian.ncols() {
                weighted[(i, j)] *= sqrt_weight;
            }
        }
        weighted
    }

    fn apply_weights_to_residuals(&self, residuals: &DVector<f64>, weights: &[f64]) -> DVector<f64> {
        let mut weighted = residuals.clone();
        for i in 0..residuals.len() {
            weighted[i] *= weights[i].sqrt();
        }
        weighted
    }

    fn estimate_transformation_uncertainty(
        &self,
        _measurements: &[WeightedMeasurement],
        _position: &Vector3<f64>,
    ) -> Result<f64, String> {
        Ok(0.05) // Simplified transformation uncertainty estimate
    }
}

// Implementation of supporting structures

impl HighPrecisionCoordinateTransformer {
    fn new(reference_position: Position) -> Self {
        Self {
            reference_position,
            ellipsoid_params: EllipsoidParams::wgs84(),
            precision_settings: TransformationPrecision::high_precision(),
        }
    }

    fn local_to_geodetic_high_precision(&self, local_pos: &Vector3<f64>) -> Result<Position, String> {
        // High-precision coordinate transformation (simplified)
        let lat0_rad = self.reference_position.lat.to_radians();
        let meters_per_deg_lat = 111_132.0;
        let meters_per_deg_lon = 111_320.0 * lat0_rad.cos();

        let lat = self.reference_position.lat + local_pos.y / meters_per_deg_lat;
        let lon = self.reference_position.lon + local_pos.x / meters_per_deg_lon;
        let depth = local_pos.z;

        Ok(Position { lat, lon, depth })
    }
}

impl EllipsoidParams {
    fn wgs84() -> Self {
        Self {
            semi_major_axis: 6378137.0,
            flattening: 1.0 / 298.257223563,
        }
    }
}

impl TransformationPrecision {
    fn high_precision() -> Self {
        Self {
            coordinate_precision: 1e-6,
            angular_precision: 1e-9,
            use_high_precision_math: true,
        }
    }
}

impl EnvironmentalCorrector {
    fn new() -> Self {
        Self {
            sound_speed_model: SoundSpeedModel::default(),
            temperature_model: TemperatureModel::default(),
            pressure_model: PressureModel::default(),
        }
    }

    fn update_models(&mut self, params: &EnvironmentalParams) {
        self.sound_speed_model.base_sound_speed = params.sound_speed;
    }

    fn compute_sound_speed_correction(
        &self,
        range: f64,
        _anchor_pos: &Vector3<f64>,
        params: &EnvironmentalParams,
    ) -> f64 {
        let speed_error = params.sound_speed - self.sound_speed_model.base_sound_speed;
        let correction_factor = speed_error / self.sound_speed_model.base_sound_speed;
        range * correction_factor
    }

    fn compute_temperature_correction(
        &self,
        range: f64,
        _anchor_pos: &Vector3<f64>,
        params: &EnvironmentalParams,
    ) -> f64 {
        let temp_diff = params.temperature - self.temperature_model.reference_temperature;
        range * temp_diff * 0.001 // Simplified temperature correction
    }

    fn compute_pressure_correction(
        &self,
        range: f64,
        anchor_pos: &Vector3<f64>,
        params: &EnvironmentalParams,
    ) -> f64 {
        let depth = anchor_pos.z;
        let pressure_diff = params.pressure - self.pressure_model.reference_pressure;
        range * pressure_diff * depth * 1e-6 // Simplified pressure correction
    }
}

impl SoundSpeedModel {
    fn default() -> Self {
        Self {
            base_sound_speed: SPEED_OF_SOUND_WATER,
            temperature_coefficient: 4.0,
            salinity_coefficient: 1.3,
            pressure_coefficient: 0.016,
        }
    }
}

impl TemperatureModel {
    fn default() -> Self {
        Self {
            reference_temperature: 15.0,
            temperature_gradient: -0.1,
        }
    }
}

impl PressureModel {
    fn default() -> Self {
        Self {
            reference_pressure: 1.0,
            pressure_gradient: 0.1,
        }
    }
}

impl CalibrationManager {
    fn new() -> Self {
        Self {
            anchor_calibrations: std::collections::HashMap::new(),
            clock_calibrations: std::collections::HashMap::new(),
            range_bias_corrections: std::collections::HashMap::new(),
        }
    }
}

impl PositionValidator {
    fn new() -> Self {
        Self {
            position_history: VecDeque::new(),
            validation_thresholds: ValidationThresholds {
                max_position_jump: 10.0,  // 10 meters
                max_velocity: 5.0,        // 5 m/s
                min_consistency_score: 0.7,
            },
        }
    }
}

// Type alias for Matrix3x4 (not available in nalgebra by default)
type Matrix3x4<T> = nalgebra::Matrix<T, nalgebra::U3, nalgebra::U4, nalgebra::ArrayStorage<T, 3, 4>>;

/// Geometric Dilution of Precision (GDOP) analysis and optimization
pub struct GDOPAnalyzer {
    /// Cached GDOP calculations for performance
    gdop_cache: std::collections::HashMap<String, GDOPResult>,
}

/// GDOP calculation result
#[derive(Debug, Clone)]
pub struct GDOPResult {
    pub gdop: f64,
    pub pdop: f64,  // Position DOP
    pub hdop: f64,  // Horizontal DOP
    pub vdop: f64,  // Vertical DOP
    pub geometry_quality: GeometryQuality,
    pub anchor_positions: Vec<Vector3<f64>>,
    pub covariance_matrix: Matrix3<f64>,
}

/// Geometry quality classification
#[derive(Debug, Clone, PartialEq)]
pub enum GeometryQuality {
    Excellent,   // GDOP < 2
    Good,        // GDOP < 4
    Moderate,    // GDOP < 6
    Fair,        // GDOP < 8
    Poor,        // GDOP >= 8
}

/// Anchor selection result
#[derive(Debug)]
pub struct AnchorSelectionResult {
    pub selected_anchors: Vec<usize>,
    pub gdop: f64,
    pub expected_accuracy: f64,
    pub geometry_quality: GeometryQuality,
}

impl GDOPAnalyzer {
    pub fn new() -> Self {
        Self {
            gdop_cache: std::collections::HashMap::new(),
        }
    }

    /// Calculate comprehensive GDOP metrics for given anchor configuration
    pub fn calculate_gdop(
        &mut self,
        anchor_positions: &[Vector3<f64>],
        estimated_position: &Vector3<f64>,
    ) -> Result<GDOPResult, String> {
        if anchor_positions.len() < 3 {
            return Err("At least 3 anchors required for GDOP calculation".to_string());
        }

        // Create cache key based on anchor positions
        let cache_key = self.create_cache_key(anchor_positions, estimated_position);
        if let Some(cached_result) = self.gdop_cache.get(&cache_key) {
            return Ok(cached_result.clone());
        }

        // Build geometry matrix (Jacobian of range equations)
        let n = anchor_positions.len();
        let mut geometry_matrix = DMatrix::zeros(n, 3);
        
        for (i, anchor_pos) in anchor_positions.iter().enumerate() {
            let diff = estimated_position - anchor_pos;
            let range = diff.norm();
            
            if range < 1e-10 {
                return Err("Anchor too close to estimated position".to_string());
            }
            
            // Unit vector from anchor to estimated position
            let unit_vector = diff / range;
            geometry_matrix[(i, 0)] = unit_vector.x;
            geometry_matrix[(i, 1)] = unit_vector.y;
            geometry_matrix[(i, 2)] = unit_vector.z;
        }

        // Compute covariance matrix: (G^T * G)^(-1)
        let gtg = geometry_matrix.transpose() * &geometry_matrix;
        let covariance = gtg.try_inverse()
            .ok_or("Singular geometry matrix - cannot compute GDOP")?;

        // Extract DOP values
        let gdop = covariance.trace().sqrt();
        let pdop = gdop; // For 3D positioning, PDOP = GDOP
        let hdop = (covariance[(0, 0)] + covariance[(1, 1)]).sqrt();
        let vdop = covariance[(2, 2)].sqrt();

        let geometry_quality = Self::classify_geometry_quality(gdop);

        let result = GDOPResult {
            gdop,
            pdop,
            hdop,
            vdop,
            geometry_quality,
            anchor_positions: anchor_positions.to_vec(),
            covariance_matrix: Matrix3::new(
                covariance[(0, 0)], covariance[(0, 1)], covariance[(0, 2)],
                covariance[(1, 0)], covariance[(1, 1)], covariance[(1, 2)],
                covariance[(2, 0)], covariance[(2, 1)], covariance[(2, 2)],
            ),
        };

        // Cache the result
        self.gdop_cache.insert(cache_key, result.clone());
        Ok(result)
    }

    /// Assess anchor geometry quality and provide scoring
    pub fn assess_geometry_quality(
        &mut self,
        anchor_positions: &[Vector3<f64>],
        estimated_position: &Vector3<f64>,
    ) -> Result<f64, String> {
        let gdop_result = self.calculate_gdop(anchor_positions, estimated_position)?;
        
        // Compute comprehensive geometry score (0-1, higher is better)
        let gdop_score = 1.0 / (1.0 + gdop_result.gdop / 2.0);
        
        // Volume score for 3D configurations
        let volume_score = if anchor_positions.len() >= 4 {
            self.compute_tetrahedron_volume_score(anchor_positions)
        } else {
            self.compute_triangle_area_score(anchor_positions)
        };
        
        // Condition number score
        let condition_score = self.compute_condition_number_score(&gdop_result.covariance_matrix);
        
        // Combined score with weights
        let combined_score = 0.5 * gdop_score + 0.3 * volume_score + 0.2 * condition_score;
        
        Ok(combined_score)
    }

    /// Automatic anchor selection based on geometric configuration
    pub fn select_optimal_anchors(
        &mut self,
        available_anchors: &[Anchor],
        receiver_time_ms: u64,
        target_anchor_count: usize,
    ) -> Result<AnchorSelectionResult, String> {
        if available_anchors.len() < target_anchor_count {
            return Err("Insufficient anchors available for selection".to_string());
        }

        let reference_pos = &available_anchors[0].position;
        
        // Convert all anchors to local coordinates
        let anchor_positions: Vec<Vector3<f64>> = available_anchors
            .iter()
            .map(|anchor| geodetic_to_local(&anchor.position, reference_pos))
            .collect();

        // Initial position estimate using all anchors
        let measurements = self.prepare_measurements_for_selection(available_anchors, receiver_time_ms)?;
        let initial_position = self.compute_initial_position_estimate(&measurements)?;

        let mut best_selection = None;
        let mut best_score = f64::NEG_INFINITY;

        // Try all combinations of target_anchor_count anchors
        let combinations = self.generate_combinations(available_anchors.len(), target_anchor_count);
        
        for combination in combinations.iter().take(100) { // Limit to 100 combinations for performance
            let selected_positions: Vec<Vector3<f64>> = combination
                .iter()
                .map(|&i| anchor_positions[i])
                .collect();

            if let Ok(gdop_result) = self.calculate_gdop(&selected_positions, &initial_position) {
                let geometry_score = self.assess_geometry_quality(&selected_positions, &initial_position)
                    .unwrap_or(0.0);
                
                // Score based on GDOP and geometry quality
                let score = geometry_score - gdop_result.gdop / 10.0;
                
                if score > best_score {
                    best_score = score;
                    best_selection = Some(AnchorSelectionResult {
                        selected_anchors: combination.clone(),
                        gdop: gdop_result.gdop,
                        expected_accuracy: gdop_result.gdop * 0.5, // Rough accuracy estimate
                        geometry_quality: gdop_result.geometry_quality,
                    });
                }
            }
        }

        best_selection.ok_or("No valid anchor combination found".to_string())
    }

    /// Create position uncertainty estimation based on anchor geometry
    pub fn estimate_position_uncertainty(
        &mut self,
        anchor_positions: &[Vector3<f64>],
        estimated_position: &Vector3<f64>,
        measurement_noise: f64,
    ) -> Result<(f64, Vector3<f64>), String> {
        let gdop_result = self.calculate_gdop(anchor_positions, estimated_position)?;
        
        // Overall uncertainty estimate
        let overall_uncertainty = gdop_result.gdop * measurement_noise;
        
        // Directional uncertainties
        let uncertainty_vector = Vector3::new(
            gdop_result.covariance_matrix[(0, 0)].sqrt() * measurement_noise,
            gdop_result.covariance_matrix[(1, 1)].sqrt() * measurement_noise,
            gdop_result.covariance_matrix[(2, 2)].sqrt() * measurement_noise,
        );
        
        Ok((overall_uncertainty, uncertainty_vector))
    }

    /// Adaptive algorithm selection based on GDOP values
    pub fn select_algorithm_for_gdop(&self, gdop: f64) -> &'static str {
        match gdop {
            g if g < 2.0 => "standard_least_squares",
            g if g < 4.0 => "weighted_least_squares", 
            g if g < 6.0 => "levenberg_marquardt",
            g if g < 8.0 => "robust_estimation",
            _ => "maximum_likelihood_estimator",
        }
    }

    // Private helper methods

    fn classify_geometry_quality(gdop: f64) -> GeometryQuality {
        match gdop {
            g if g < 2.0 => GeometryQuality::Excellent,
            g if g < 4.0 => GeometryQuality::Good,
            g if g < 6.0 => GeometryQuality::Moderate,
            g if g < 8.0 => GeometryQuality::Fair,
            _ => GeometryQuality::Poor,
        }
    }

    fn create_cache_key(&self, positions: &[Vector3<f64>], estimated_pos: &Vector3<f64>) -> String {
        let mut key = String::new();
        for pos in positions {
            key.push_str(&format!("{:.3},{:.3},{:.3};", pos.x, pos.y, pos.z));
        }
        key.push_str(&format!("@{:.3},{:.3},{:.3}", estimated_pos.x, estimated_pos.y, estimated_pos.z));
        key
    }

    fn compute_tetrahedron_volume_score(&self, positions: &[Vector3<f64>]) -> f64 {
        if positions.len() < 4 {
            return 0.0;
        }

        // Compute volume of tetrahedron formed by first 4 anchors
        let v1 = positions[1] - positions[0];
        let v2 = positions[2] - positions[0];
        let v3 = positions[3] - positions[0];
        let volume = (v1.cross(&v2)).dot(&v3).abs() / 6.0;

        // Normalize by maximum possible volume for this scale
        let max_distance = positions.iter()
            .flat_map(|p1| positions.iter().map(move |p2| (p1 - p2).norm()))
            .fold(0.0, f64::max);
        
        let max_volume = max_distance.powi(3) / (6.0 * 2.0_f64.sqrt()); // Regular tetrahedron
        
        if max_volume > 0.0 {
            (volume / max_volume).min(1.0)
        } else {
            0.0
        }
    }

    fn compute_triangle_area_score(&self, positions: &[Vector3<f64>]) -> f64 {
        if positions.len() < 3 {
            return 0.0;
        }

        // Compute area of triangle formed by first 3 anchors
        let v1 = positions[1] - positions[0];
        let v2 = positions[2] - positions[0];
        let area = v1.cross(&v2).norm() / 2.0;

        // Normalize by maximum possible area
        let max_distance = positions.iter()
            .flat_map(|p1| positions.iter().map(move |p2| (p1 - p2).norm()))
            .fold(0.0, f64::max);
        
        let max_area = max_distance.powi(2) * 3.0_f64.sqrt() / 4.0; // Equilateral triangle
        
        if max_area > 0.0 {
            (area / max_area).min(1.0)
        } else {
            0.0
        }
    }

    fn compute_condition_number_score(&self, covariance: &Matrix3<f64>) -> f64 {
        let svd = covariance.svd(true, true);
        let singular_values = svd.singular_values;
        
        let condition_number = if singular_values[2] > 1e-10 {
            singular_values[0] / singular_values[2]
        } else {
            f64::INFINITY
        };
        
        // Score inversely related to condition number
        1.0 / (1.0 + condition_number / 100.0)
    }

    fn prepare_measurements_for_selection(
        &self,
        anchors: &[Anchor],
        receiver_time_ms: u64,
    ) -> Result<Vec<(Vector3<f64>, f64)>, String> {
        let reference_pos = &anchors[0].position;
        let mut measurements = Vec::new();
        
        for anchor in anchors {
            let local_pos = geodetic_to_local(&anchor.position, reference_pos);
            
            let dt_ms = receiver_time_ms as i64 - anchor.timestamp as i64;
            if dt_ms < 0 {
                continue; // Skip anchors with future timestamps
            }
            
            let dt_sec = dt_ms as f64 / 1000.0;
            let distance = SPEED_OF_SOUND_WATER * dt_sec;
            
            measurements.push((local_pos, distance));
        }
        
        if measurements.len() < 3 {
            return Err("Insufficient valid measurements for selection".to_string());
        }
        
        Ok(measurements)
    }

    fn compute_initial_position_estimate(
        &self,
        measurements: &[(Vector3<f64>, f64)],
    ) -> Result<Vector3<f64>, String> {
        if measurements.len() < 3 {
            return Err("Insufficient measurements for position estimate".to_string());
        }

        // Simple centroid-based initial estimate
        let centroid: Vector3<f64> = measurements
            .iter()
            .map(|(pos, _)| *pos)
            .fold(Vector3::zeros(), |acc, pos| acc + pos) / measurements.len() as f64;
        
        Ok(centroid)
    }

    fn generate_combinations(&self, n: usize, k: usize) -> Vec<Vec<usize>> {
        let mut combinations = Vec::new();
        let mut current = Vec::new();
        self.generate_combinations_recursive(n, k, 0, &mut current, &mut combinations);
        combinations
    }

    fn generate_combinations_recursive(
        &self,
        n: usize,
        k: usize,
        start: usize,
        current: &mut Vec<usize>,
        combinations: &mut Vec<Vec<usize>>,
    ) {
        if current.len() == k {
            combinations.push(current.clone());
            return;
        }

        for i in start..n {
            current.push(i);
            self.generate_combinations_recursive(n, k, i + 1, current, combinations);
            current.pop();
        }
    }
}

/// Enhanced positioning engine with GDOP optimization
impl AdvancedPositioningEngine {
    /// Trilaterate with automatic GDOP-based algorithm selection
    pub fn trilaterate_with_gdop_optimization(
        &mut self,
        anchors: &[Anchor],
        receiver_time_ms: u64,
    ) -> Result<(Vector3<f64>, f64, GDOPResult), String> {
        let mut gdop_analyzer = GDOPAnalyzer::new();
        
        // Prepare measurements and initial position estimate
        let measurements = self.prepare_weighted_measurements(anchors, receiver_time_ms)?;
        let initial_position = self.compute_initial_estimate(&measurements)?;
        
        // Convert anchor positions to local coordinates
        let reference_pos = &anchors[0].position;
        let anchor_positions: Vec<Vector3<f64>> = anchors
            .iter()
            .map(|anchor| geodetic_to_local(&anchor.position, reference_pos))
            .collect();

        // Calculate GDOP
        let gdop_result = gdop_analyzer.calculate_gdop(&anchor_positions, &initial_position)?;
        
        // Select algorithm based on GDOP
        let algorithm = gdop_analyzer.select_algorithm_for_gdop(gdop_result.gdop);
        
        let (position, uncertainty) = match algorithm {
            "standard_least_squares" => {
                let (pos, unc) = self.weighted_least_squares_solve(measurements)?;
                (pos, unc)
            }
            "weighted_least_squares" => {
                let (pos, unc) = self.weighted_least_squares_solve(measurements)?;
                (pos, unc)
            }
            "levenberg_marquardt" => {
                let result = self.levenberg_marquardt_optimization(measurements, initial_position)?;
                (result.position, result.residual_norm)
            }
            "robust_estimation" => {
                let (pos, unc) = self.trilaterate_robust(anchors, receiver_time_ms)?;
                (pos, unc)
            }
            "maximum_likelihood_estimator" => {
                let result = self.trilaterate_mle(anchors, receiver_time_ms, 1.0)?;
                (result.position, result.uncertainty)
            }
            _ => {
                let (pos, unc) = self.weighted_least_squares_solve(measurements)?;
                (pos, unc)
            }
        };

        Ok((position, uncertainty, gdop_result))
    }
}

/// Convenience function for enhanced trilateration with all advanced techniques
pub fn enhanced_trilaterate(
    anchors: &[Anchor],
    receiver_time_ms: u64,
    reference_pos: &Position,
) -> Result<(Position, Vector3<f64>, f64), String> {
    let mut engine = AdvancedPositioningEngine::new();
    
    // Try GDOP-optimized trilateration first
    match engine.trilaterate_with_gdop_optimization(anchors, receiver_time_ms) {
        Ok((local_pos, uncertainty, _gdop_result)) => {
            let geodetic_pos = local_to_geodetic(&local_pos, reference_pos);
            return Ok((geodetic_pos, local_pos, uncertainty));
        }
        Err(_) => {
            // Fall back to other methods
        }
    }
    
    // Try robust estimation first (best for outlier handling)
    if anchors.len() >= 4 {
        match engine.trilaterate_robust(anchors, receiver_time_ms) {
            Ok((local_pos, uncertainty)) => {
                let geodetic_pos = local_to_geodetic(&local_pos, reference_pos);
                return Ok((geodetic_pos, local_pos, uncertainty));
            }
            Err(_) => {
                // Fall back to other methods
            }
        }
    }
    
    // Try weighted least squares
    match engine.trilaterate_weighted_least_squares(anchors, receiver_time_ms) {
        Ok((local_pos, uncertainty)) => {
            let geodetic_pos = local_to_geodetic(&local_pos, reference_pos);
            Ok((geodetic_pos, local_pos, uncertainty))
        }
        Err(e) => Err(e),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_advanced_positioning_engine_creation() {
        let engine = AdvancedPositioningEngine::new();
        assert!(engine.kalman_state.is_none());
        assert_eq!(engine.position_history.len(), 0);
    }
    
    #[test]
    fn test_weighted_measurements_preparation() {
        let engine = AdvancedPositioningEngine::new();
        let anchors = vec![
            Anchor {
                id: "1".to_string(),
                timestamp: 1000,
                position: Position { lat: 0.0, lon: 0.0, depth: 0.0 },
            },
            Anchor {
                id: "2".to_string(),
                timestamp: 1000,
                position: Position { lat: 0.001, lon: 0.0, depth: 0.0 },
            },
            Anchor {
                id: "3".to_string(),
                timestamp: 1000,
                position: Position { lat: 0.0, lon: 0.001, depth: 0.0 },
            },
        ];
        
        let measurements = engine.prepare_weighted_measurements(&anchors, 2000).unwrap();
        assert_eq!(measurements.len(), 3);
        assert!(measurements.iter().all(|m| m.weight > 0.0));
    }
    
    #[test]
    fn test_kalman_filter_initialization() {
        let mut engine = AdvancedPositioningEngine::new();
        let measurement = Vector3::new(1.0, 2.0, 3.0);
        let result = engine.update_kalman_filter(measurement, 1.0, 1000);
        
        assert_eq!(result, measurement);
        assert!(engine.kalman_state.is_some());
    }
}