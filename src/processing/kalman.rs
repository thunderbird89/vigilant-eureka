use nalgebra::{Matrix3, Matrix6, Vector3, Vector6, Matrix3x6};
use crate::core::Position;
use std::time::{Duration, Instant};

/// Kalman filter for temporal position smoothing and prediction
/// State vector: [x, y, z, vx, vy, vz] (position and velocity)
#[derive(Clone)]
pub struct PositionKalmanFilter {
    /// Current state estimate [position, velocity]
    pub state: Vector6<f64>,
    /// State covariance matrix
    pub covariance: Matrix6<f64>,
    /// Process noise covariance matrix
    pub process_noise: Matrix6<f64>,
    /// Measurement noise covariance matrix
    pub measurement_noise: Matrix3<f64>,
    /// Last update timestamp
    pub last_update: Option<Instant>,
    /// Maximum time between updates before reset (seconds)
    pub max_time_gap: f64,
}

impl PositionKalmanFilter {
    /// Create new Kalman filter with default parameters
    pub fn new() -> Self {
        Self {
            state: Vector6::zeros(),
            covariance: Matrix6::identity() * 100.0, // Initial uncertainty
            process_noise: Self::default_process_noise(),
            measurement_noise: Self::default_measurement_noise(),
            last_update: None,
            max_time_gap: 10.0, // 10 seconds
        }
    }

    /// Create Kalman filter with custom noise parameters
    pub fn with_noise_parameters(
        position_process_noise: f64,
        velocity_process_noise: f64,
        measurement_noise: f64,
    ) -> Self {
        let mut filter = Self::new();
        filter.process_noise = Self::create_process_noise(position_process_noise, velocity_process_noise);
        filter.measurement_noise = Matrix3::identity() * measurement_noise;
        filter
    }

    /// Initialize filter with first position measurement
    pub fn initialize(&mut self, position: &Vector3<f64>, initial_velocity: Option<Vector3<f64>>) {
        let velocity = initial_velocity.unwrap_or_else(Vector3::zeros);
        
        self.state[0] = position.x;
        self.state[1] = position.y;
        self.state[2] = position.z;
        self.state[3] = velocity.x;
        self.state[4] = velocity.y;
        self.state[5] = velocity.z;
        
        // Reset covariance with reasonable initial values
        self.covariance = Matrix6::identity();
        self.covariance[(0, 0)] = 1.0;  // 1m position uncertainty
        self.covariance[(1, 1)] = 1.0;
        self.covariance[(2, 2)] = 1.0;
        self.covariance[(3, 3)] = 0.1;  // 0.1 m/s velocity uncertainty
        self.covariance[(4, 4)] = 0.1;
        self.covariance[(5, 5)] = 0.1;
        
        self.last_update = Some(Instant::now());
    }

    /// Predict next state based on motion model
    pub fn predict(&mut self, dt: f64) -> Vector3<f64> {
        if dt <= 0.0 {
            return Vector3::new(self.state[0], self.state[1], self.state[2]);
        }

        // State transition matrix (constant velocity model)
        let f = self.create_state_transition_matrix(dt);
        
        // Predict state: x_k = F * x_{k-1}
        self.state = f * self.state;
        
        // Predict covariance: P_k = F * P_{k-1} * F^T + Q
        self.covariance = f * self.covariance * f.transpose() + self.process_noise * dt;
        
        Vector3::new(self.state[0], self.state[1], self.state[2])
    }

    /// Update filter with new position measurement
    pub fn update(&mut self, measurement: &Vector3<f64>, measurement_uncertainty: Option<Matrix3<f64>>) -> Vector3<f64> {
        let now = Instant::now();
        
        // Check if too much time has passed - reset if necessary
        if let Some(last_time) = self.last_update {
            let elapsed = now.duration_since(last_time).as_secs_f64();
            if elapsed > self.max_time_gap {
                // Reset filter with new measurement
                self.initialize(measurement, None);
                return *measurement;
            }
            
            // Predict to current time
            self.predict(elapsed);
        } else {
            // First measurement - initialize
            self.initialize(measurement, None);
            return *measurement;
        }

        // Measurement matrix (we observe position directly)
        let h = Matrix3x6::new(
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        );

        // Use provided uncertainty or default
        let r = measurement_uncertainty.unwrap_or(self.measurement_noise);

        // Innovation (measurement residual)
        let predicted_measurement = h * self.state;
        let innovation = measurement - predicted_measurement;

        // Innovation covariance
        let s = h * self.covariance * h.transpose() + r;

        // Kalman gain
        if let Some(s_inv) = s.try_inverse() {
            let k = self.covariance * h.transpose() * s_inv;

            // Update state estimate
            self.state = self.state + k * innovation;

            // Update covariance estimate (Joseph form for numerical stability)
            let i_kh = Matrix6::identity() - k * h;
            self.covariance = i_kh * self.covariance * i_kh.transpose() + k * r * k.transpose();
        }

        self.last_update = Some(now);
        Vector3::new(self.state[0], self.state[1], self.state[2])
    }

    /// Get current position estimate
    pub fn get_position(&self) -> Vector3<f64> {
        Vector3::new(self.state[0], self.state[1], self.state[2])
    }

    /// Get current velocity estimate
    pub fn get_velocity(&self) -> Vector3<f64> {
        Vector3::new(self.state[3], self.state[4], self.state[5])
    }

    /// Get position uncertainty (standard deviations)
    pub fn get_position_uncertainty(&self) -> Vector3<f64> {
        Vector3::new(
            self.covariance[(0, 0)].sqrt(),
            self.covariance[(1, 1)].sqrt(),
            self.covariance[(2, 2)].sqrt(),
        )
    }

    /// Get velocity uncertainty (standard deviations)
    pub fn get_velocity_uncertainty(&self) -> Vector3<f64> {
        Vector3::new(
            self.covariance[(3, 3)].sqrt(),
            self.covariance[(4, 4)].sqrt(),
            self.covariance[(5, 5)].sqrt(),
        )
    }

    /// Predict position at future time
    pub fn predict_future_position(&self, future_dt: f64) -> Vector3<f64> {
        if future_dt <= 0.0 {
            return self.get_position();
        }

        // Simple constant velocity prediction
        let current_pos = self.get_position();
        let current_vel = self.get_velocity();
        
        current_pos + current_vel * future_dt
    }

    /// Check if filter is initialized
    pub fn is_initialized(&self) -> bool {
        self.last_update.is_some()
    }

    /// Reset filter state
    pub fn reset(&mut self) {
        self.state = Vector6::zeros();
        self.covariance = Matrix6::identity() * 100.0;
        self.last_update = None;
    }

    /// Set process noise parameters
    pub fn set_process_noise(&mut self, position_noise: f64, velocity_noise: f64) {
        self.process_noise = Self::create_process_noise(position_noise, velocity_noise);
    }

    /// Set measurement noise
    pub fn set_measurement_noise(&mut self, noise: f64) {
        self.measurement_noise = Matrix3::identity() * noise;
    }

    /// Create state transition matrix for constant velocity model
    fn create_state_transition_matrix(&self, dt: f64) -> Matrix6<f64> {
        Matrix6::new(
            1.0, 0.0, 0.0, dt,  0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, dt,  0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, dt,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        )
    }

    /// Create default process noise matrix
    fn default_process_noise() -> Matrix6<f64> {
        Self::create_process_noise(0.1, 0.01) // 0.1 m/s² position, 0.01 m/s² velocity
    }

    /// Create process noise matrix with specified parameters
    fn create_process_noise(position_noise: f64, velocity_noise: f64) -> Matrix6<f64> {
        let mut q = Matrix6::zeros();
        
        // Position process noise
        q[(0, 0)] = position_noise;
        q[(1, 1)] = position_noise;
        q[(2, 2)] = position_noise;
        
        // Velocity process noise
        q[(3, 3)] = velocity_noise;
        q[(4, 4)] = velocity_noise;
        q[(5, 5)] = velocity_noise;
        
        q
    }

    /// Create default measurement noise matrix
    fn default_measurement_noise() -> Matrix3<f64> {
        Matrix3::identity() * 1.0 // 1 meter measurement noise
    }
}

/// Multi-hypothesis Kalman filter for handling multiple possible tracks
pub struct MultiHypothesisKalmanFilter {
    /// Multiple filter hypotheses
    pub filters: Vec<PositionKalmanFilter>,
    /// Hypothesis weights (probabilities)
    pub weights: Vec<f64>,
    /// Maximum number of hypotheses to maintain
    pub max_hypotheses: usize,
    /// Minimum weight threshold for hypothesis pruning
    pub min_weight_threshold: f64,
}

impl MultiHypothesisKalmanFilter {
    pub fn new(max_hypotheses: usize) -> Self {
        Self {
            filters: Vec::new(),
            weights: Vec::new(),
            max_hypotheses,
            min_weight_threshold: 0.01,
        }
    }

    /// Initialize with first measurement
    pub fn initialize(&mut self, measurement: &Vector3<f64>) {
        let mut filter = PositionKalmanFilter::new();
        filter.initialize(measurement, None);
        
        self.filters.clear();
        self.weights.clear();
        self.filters.push(filter);
        self.weights.push(1.0);
    }

    /// Update with new measurement, handling potential ambiguity
    pub fn update(&mut self, measurement: &Vector3<f64>, gate_threshold: f64) -> Vector3<f64> {
        if self.filters.is_empty() {
            self.initialize(measurement);
            return *measurement;
        }

        let mut new_filters = Vec::new();
        let mut new_weights = Vec::new();

        // Update each hypothesis
        for (i, filter) in self.filters.iter_mut().enumerate() {
            let predicted_pos = filter.get_position();
            let distance = (measurement - predicted_pos).norm();
            
            if distance <= gate_threshold {
                // Measurement is compatible with this hypothesis
                // Create a new filter with the same parameters
                let mut updated_filter = PositionKalmanFilter::new();
                updated_filter.state = filter.state;
                updated_filter.covariance = filter.covariance;
                updated_filter.process_noise = filter.process_noise;
                updated_filter.measurement_noise = filter.measurement_noise;
                updated_filter.last_update = filter.last_update;
                updated_filter.max_time_gap = filter.max_time_gap;
                updated_filter.update(measurement, None);
                
                // Calculate likelihood (simplified)
                let likelihood = (-0.5 * (distance / gate_threshold).powi(2)).exp();
                let updated_weight = self.weights[i] * likelihood;
                
                new_filters.push(updated_filter);
                new_weights.push(updated_weight);
            }
        }

        // If no hypotheses were compatible, create new one
        if new_filters.is_empty() {
            let mut new_filter = PositionKalmanFilter::new();
            new_filter.initialize(measurement, None);
            new_filters.push(new_filter);
            new_weights.push(0.1); // Low initial weight for new hypothesis
        }

        // Normalize weights
        let total_weight: f64 = new_weights.iter().sum();
        if total_weight > 0.0 {
            for weight in &mut new_weights {
                *weight /= total_weight;
            }
        }

        // Prune low-weight hypotheses
        let mut pruned_filters = Vec::new();
        let mut pruned_weights = Vec::new();
        
        for (filter, weight) in new_filters.into_iter().zip(new_weights.into_iter()) {
            if weight >= self.min_weight_threshold {
                pruned_filters.push(filter);
                pruned_weights.push(weight);
            }
        }

        // Limit number of hypotheses
        if pruned_filters.len() > self.max_hypotheses {
            // Sort by weight and keep top hypotheses
            let mut indexed_weights: Vec<(usize, f64)> = pruned_weights.iter().enumerate().map(|(i, &w)| (i, w)).collect();
            indexed_weights.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
            
            let mut final_filters = Vec::new();
            let mut final_weights = Vec::new();
            
            for i in 0..self.max_hypotheses {
                let idx = indexed_weights[i].0;
                final_filters.push(pruned_filters[idx].clone());
                final_weights.push(pruned_weights[idx]);
            }
            
            self.filters = final_filters;
            self.weights = final_weights;
        } else {
            self.filters = pruned_filters;
            self.weights = pruned_weights;
        }

        // Return weighted average position
        self.get_weighted_position()
    }

    /// Get weighted average position from all hypotheses
    pub fn get_weighted_position(&self) -> Vector3<f64> {
        if self.filters.is_empty() {
            return Vector3::zeros();
        }

        let mut weighted_pos = Vector3::zeros();
        for (filter, weight) in self.filters.iter().zip(self.weights.iter()) {
            weighted_pos += filter.get_position() * (*weight);
        }
        
        weighted_pos
    }

    /// Get most likely hypothesis
    pub fn get_best_hypothesis(&self) -> Option<&PositionKalmanFilter> {
        if self.filters.is_empty() {
            return None;
        }

        let max_weight_idx = self.weights.iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .map(|(i, _)| i)?;

        Some(&self.filters[max_weight_idx])
    }

    /// Get number of active hypotheses
    pub fn num_hypotheses(&self) -> usize {
        self.filters.len()
    }
}

/// Adaptive Kalman filter that adjusts noise parameters based on innovation
pub struct AdaptiveKalmanFilter {
    /// Base Kalman filter
    pub base_filter: PositionKalmanFilter,
    /// Innovation history for adaptation
    innovation_history: Vec<Vector3<f64>>,
    /// Maximum history length
    max_history_length: usize,
    /// Adaptation rate
    adaptation_rate: f64,
}

impl AdaptiveKalmanFilter {
    pub fn new() -> Self {
        Self {
            base_filter: PositionKalmanFilter::new(),
            innovation_history: Vec::new(),
            max_history_length: 10,
            adaptation_rate: 0.1,
        }
    }

    /// Update filter and adapt noise parameters
    pub fn update(&mut self, measurement: &Vector3<f64>) -> Vector3<f64> {
        // Get predicted position before update
        let predicted_pos = self.base_filter.get_position();
        
        // Update base filter
        let result = self.base_filter.update(measurement, None);
        
        // Calculate innovation
        let innovation = measurement - predicted_pos;
        self.innovation_history.push(innovation);
        
        // Limit history length
        if self.innovation_history.len() > self.max_history_length {
            self.innovation_history.remove(0);
        }
        
        // Adapt noise parameters based on innovation statistics
        if self.innovation_history.len() >= 5 {
            self.adapt_noise_parameters();
        }
        
        result
    }

    /// Adapt noise parameters based on innovation statistics
    fn adapt_noise_parameters(&mut self) {
        if self.innovation_history.is_empty() {
            return;
        }

        // Calculate innovation variance
        let mean_innovation = self.innovation_history.iter()
            .fold(Vector3::zeros(), |acc, &inn| acc + inn) / self.innovation_history.len() as f64;
        
        let innovation_variance = self.innovation_history.iter()
            .map(|&inn| (inn - mean_innovation).norm_squared())
            .sum::<f64>() / self.innovation_history.len() as f64;

        // Adapt measurement noise based on innovation variance
        let current_measurement_noise = self.base_filter.measurement_noise[(0, 0)];
        let target_measurement_noise = innovation_variance.sqrt();
        let new_measurement_noise = current_measurement_noise * (1.0 - self.adaptation_rate) 
            + target_measurement_noise * self.adaptation_rate;
        
        self.base_filter.set_measurement_noise(new_measurement_noise);
        
        // Adapt process noise based on innovation trend
        let innovation_trend = if self.innovation_history.len() >= 2 {
            let recent = &self.innovation_history[self.innovation_history.len()-2..];
            (recent[1] - recent[0]).norm()
        } else {
            0.0
        };
        
        let process_noise_adjustment = innovation_trend * self.adaptation_rate;
        let current_process_noise = self.base_filter.process_noise[(0, 0)];
        let new_process_noise = (current_process_noise + process_noise_adjustment).max(0.001);
        
        self.base_filter.set_process_noise(new_process_noise, new_process_noise * 0.1);
    }

    /// Get current position estimate
    pub fn get_position(&self) -> Vector3<f64> {
        self.base_filter.get_position()
    }

    /// Get current velocity estimate
    pub fn get_velocity(&self) -> Vector3<f64> {
        self.base_filter.get_velocity()
    }

    /// Check if filter is initialized
    pub fn is_initialized(&self) -> bool {
        self.base_filter.is_initialized()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread;

    #[test]
    fn test_kalman_filter_initialization() {
        let mut filter = PositionKalmanFilter::new();
        let initial_pos = Vector3::new(10.0, 20.0, 5.0);
        
        filter.initialize(&initial_pos, None);
        
        assert!(filter.is_initialized());
        let pos = filter.get_position();
        assert!((pos - initial_pos).norm() < 1e-10);
    }

    #[test]
    fn test_kalman_filter_prediction() {
        let mut filter = PositionKalmanFilter::new();
        let initial_pos = Vector3::new(0.0, 0.0, 0.0);
        let initial_vel = Vector3::new(1.0, 0.0, 0.0); // 1 m/s in x direction
        
        filter.initialize(&initial_pos, Some(initial_vel));
        
        // Predict 1 second into future
        let predicted_pos = filter.predict(1.0);
        
        // Should move 1 meter in x direction
        assert!((predicted_pos.x - 1.0).abs() < 1e-10);
        assert!(predicted_pos.y.abs() < 1e-10);
        assert!(predicted_pos.z.abs() < 1e-10);
    }

    #[test]
    fn test_kalman_filter_update() {
        let mut filter = PositionKalmanFilter::new();
        let initial_pos = Vector3::new(0.0, 0.0, 0.0);
        
        filter.initialize(&initial_pos, None);
        
        // Update with new measurement
        let measurement = Vector3::new(1.0, 1.0, 0.0);
        let updated_pos = filter.update(&measurement, None);
        
        // Position should be between initial and measurement
        assert!(updated_pos.x > 0.0 && updated_pos.x < 1.0);
        assert!(updated_pos.y > 0.0 && updated_pos.y < 1.0);
    }

    #[test]
    fn test_multi_hypothesis_filter() {
        let mut mh_filter = MultiHypothesisKalmanFilter::new(3);
        
        // Initialize with first measurement
        let first_measurement = Vector3::new(0.0, 0.0, 0.0);
        mh_filter.initialize(&first_measurement);
        
        assert_eq!(mh_filter.num_hypotheses(), 1);
        
        // Add compatible measurement
        let second_measurement = Vector3::new(1.0, 0.0, 0.0);
        let result = mh_filter.update(&second_measurement, 5.0); // Large gate
        
        assert!(mh_filter.num_hypotheses() >= 1);
        assert!(result.x > 0.0);
    }

    #[test]
    fn test_adaptive_filter() {
        let mut adaptive_filter = AdaptiveKalmanFilter::new();
        
        // Initialize with measurements
        let measurements = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(2.0, 0.0, 0.0),
            Vector3::new(3.0, 0.0, 0.0),
        ];
        
        for measurement in measurements {
            adaptive_filter.update(&measurement);
        }
        
        assert!(adaptive_filter.is_initialized());
        let final_pos = adaptive_filter.get_position();
        assert!(final_pos.x > 0.0);
    }

    #[test]
    fn test_future_prediction() {
        let mut filter = PositionKalmanFilter::new();
        let initial_pos = Vector3::new(0.0, 0.0, 0.0);
        let initial_vel = Vector3::new(2.0, 1.0, 0.0);
        
        filter.initialize(&initial_pos, Some(initial_vel));
        
        // Predict 2 seconds into future
        let future_pos = filter.predict_future_position(2.0);
        
        // Should be at (4, 2, 0) based on constant velocity
        assert!((future_pos.x - 4.0).abs() < 1e-10);
        assert!((future_pos.y - 2.0).abs() < 1e-10);
        assert!(future_pos.z.abs() < 1e-10);
    }

    #[test]
    fn test_uncertainty_tracking() {
        let mut filter = PositionKalmanFilter::new();
        let initial_pos = Vector3::new(0.0, 0.0, 0.0);
        
        filter.initialize(&initial_pos, None);
        
        let initial_uncertainty = filter.get_position_uncertainty();
        
        // Add measurement with high uncertainty
        let high_noise = Matrix3::identity() * 100.0;
        filter.update(&Vector3::new(10.0, 0.0, 0.0), Some(high_noise));
        
        let final_uncertainty = filter.get_position_uncertainty();
        
        // Uncertainty should be reasonable (not too high or too low)
        assert!(final_uncertainty.x > 0.1);
        assert!(final_uncertainty.x < 100.0);
    }
}