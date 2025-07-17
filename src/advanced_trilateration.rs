use nalgebra::{Matrix3, Vector3, DMatrix, DVector, Matrix4};
use crate::{Anchor, Position, SPEED_OF_SOUND_WATER};
use crate::kalman_filter::PositionKalmanFilter;
use std::f64::consts::PI;
use std::cmp::Ordering;

/// Advanced trilateration algorithms with enhanced mathematical techniques
pub struct AdvancedTrilateration {
    /// Maximum number of iterations for iterative algorithms
    pub max_iterations: usize,
    /// Convergence tolerance for iterative algorithms
    pub convergence_tolerance: f64,
    /// Regularization parameter for ill-conditioned systems
    pub regularization_lambda: f64,
    /// Outlier detection threshold (in standard deviations)
    pub outlier_threshold: f64,
    /// Kalman filter for temporal smoothing
    pub kalman_filter: Option<PositionKalmanFilter>,
}

impl Default for AdvancedTrilateration {
    fn default() -> Self {
        Self {
            max_iterations: 50,
            convergence_tolerance: 1e-6,
            regularization_lambda: 1e-6,
            outlier_threshold: 2.5,
            kalman_filter: None,
        }
    }
}

impl AdvancedTrilateration {
    pub fn new() -> Self {
        Self::default()
    }

    /// Enhanced Maximum Likelihood Estimator (MLE) for trilateration with advanced noise handling
    /// Supports multiple noise models: Gaussian, Rayleigh, and mixed distributions
    pub fn mle_trilateration(
        &self,
        anchors: &[Anchor],
        receiver_time_ms: u64,
        measurement_variances: &[f64],
        noise_model: NoiseModel,
    ) -> Result<(Position, Vector3<f64>, f64), String> {
        if anchors.len() < 3 {
            return Err("At least 3 anchors required for MLE trilateration".to_string());
        }

        if measurement_variances.len() != anchors.len() {
            return Err("Measurement variances must match number of anchors".to_string());
        }

        // Convert to local coordinates using first anchor as reference
        let reference_pos = &anchors[0].position;
        let mut positions = Vec::new();
        let mut ranges = Vec::new();
        let mut weights = Vec::new();

        for (i, anchor) in anchors.iter().enumerate() {
            let local = self.geodetic_to_local(&anchor.position, reference_pos);
            positions.push(local);

            let dt_ms = receiver_time_ms as i64 - anchor.timestamp as i64;
            if dt_ms < 0 {
                return Err(format!("Receiver time earlier than anchor time for anchor {}", anchor.id));
            }
            let dt_sec = dt_ms as f64 / 1000.0;
            let range = SPEED_OF_SOUND_WATER * dt_sec;
            ranges.push(range);

            // Enhanced weighting with adaptive variance estimation
            let adaptive_variance = self.estimate_adaptive_variance(measurement_variances[i], &ranges, i);
            weights.push(1.0 / adaptive_variance);
        }

        // Multi-stage MLE optimization for better convergence
        let initial_estimate = self.weighted_least_squares(&positions, &ranges, &weights)?;
        
        // Stage 1: Coarse optimization with robust loss function
        let robust_estimate = self.robust_mle_optimization(&positions, &ranges, &weights, initial_estimate, noise_model)?;
        
        // Stage 2: Fine optimization with standard MLE
        let mle_estimate = self.mle_optimization(&positions, &ranges, &weights, robust_estimate, noise_model)?;
        
        // Calculate enhanced log-likelihood with model selection criteria
        let log_likelihood = self.calculate_enhanced_log_likelihood(&positions, &ranges, &weights, &mle_estimate, noise_model);
        
        // Convert back to geodetic coordinates
        let geodetic_pos = self.local_to_geodetic(&mle_estimate, reference_pos);
        
        Ok((geodetic_pos, mle_estimate, log_likelihood))
    }

    /// Enhanced Levenberg-Marquardt optimization with adaptive damping and trust region
    pub fn levenberg_marquardt_trilateration(
        &self,
        anchors: &[Anchor],
        receiver_time_ms: u64,
        initial_guess: Option<Vector3<f64>>,
    ) -> Result<(Position, Vector3<f64>, f64), String> {
        if anchors.len() < 3 {
            return Err("At least 3 anchors required for LM trilateration".to_string());
        }

        let reference_pos = &anchors[0].position;
        let mut positions = Vec::new();
        let mut ranges = Vec::new();

        for anchor in anchors {
            let local = self.geodetic_to_local(&anchor.position, reference_pos);
            positions.push(local);

            let dt_ms = receiver_time_ms as i64 - anchor.timestamp as i64;
            if dt_ms < 0 {
                return Err(format!("Receiver time earlier than anchor time for anchor {}", anchor.id));
            }
            let dt_sec = dt_ms as f64 / 1000.0;
            let range = SPEED_OF_SOUND_WATER * dt_sec;
            ranges.push(range);
        }

        // Enhanced initial guess with multiple strategies
        let mut current_estimate = match initial_guess {
            Some(guess) => guess,
            None => self.compute_robust_initial_guess(&positions, &ranges)?,
        };

        let mut lambda = 0.001; // Initial damping parameter
        let mut prev_cost = f64::INFINITY;
        let mut nu = 2.0; // Damping adjustment factor
        let mut trust_radius = 1000.0; // Trust region radius in meters

        for iteration in 0..self.max_iterations {
            // Compute residuals and Jacobian
            let (residuals, jacobian) = self.compute_residuals_and_jacobian(&positions, &ranges, &current_estimate);
            
            // Compute cost (sum of squared residuals)
            let current_cost = residuals.dot(&residuals);
            
            // Check for convergence (multiple criteria)
            if (prev_cost - current_cost).abs() < self.convergence_tolerance {
                break;
            }
            
            // Check gradient convergence
            let gradient = jacobian.transpose() * &residuals;
            if gradient.norm() < self.convergence_tolerance * 10.0 {
                break;
            }

            // Enhanced Levenberg-Marquardt step with trust region
            let jt_j = jacobian.transpose() * &jacobian;
            let jt_r = jacobian.transpose() * &residuals;
            
            // Adaptive damping based on condition number
            let svd = jt_j.clone().svd(true, true);
            let condition_number = if svd.singular_values[2] > 1e-12 {
                svd.singular_values[0] / svd.singular_values[2]
            } else {
                1e12
            };
            
            let adaptive_lambda = lambda * (1.0 + condition_number / 1000.0);
            
            // Add damping to diagonal with adaptive scaling
            let mut augmented_matrix = jt_j;
            for i in 0..3 {
                let diagonal_scaling = 1.0 + jt_j[(i, i)].abs() / (jt_j.trace() / 3.0 + 1e-12);
                augmented_matrix[(i, i)] += adaptive_lambda * diagonal_scaling;
            }

            // Solve for step with trust region constraint
            if let Some(step) = self.solve_linear_system_dmatrix(&augmented_matrix, &jt_r) {
                let step_vec3 = Vector3::new(step[0], step[1], step[2]);
                
                // Apply trust region constraint
                let step_norm = step_vec3.norm();
                let constrained_step = if step_norm > trust_radius {
                    step_vec3 * (trust_radius / step_norm)
                } else {
                    step_vec3
                };
                
                let new_estimate = current_estimate - constrained_step;
                let (new_residuals, _) = self.compute_residuals_and_jacobian(&positions, &ranges, &new_estimate);
                let new_cost = new_residuals.dot(&new_residuals);

                // Compute gain ratio for trust region adjustment
                let predicted_reduction = self.compute_predicted_reduction(&residuals, &jacobian, &constrained_step);
                let actual_reduction = current_cost - new_cost;
                let gain_ratio = if predicted_reduction.abs() > 1e-12 {
                    actual_reduction / predicted_reduction
                } else {
                    0.0
                };

                if gain_ratio > 0.25 {
                    // Accept step
                    current_estimate = new_estimate;
                    prev_cost = current_cost;
                    
                    // Adjust damping parameter
                    if gain_ratio > 0.75 {
                        lambda = (lambda / nu).max(1e-12);
                        trust_radius = (trust_radius * 2.0).min(10000.0);
                    }
                } else {
                    // Reject step and increase damping
                    lambda = (lambda * nu).min(1e6);
                    trust_radius *= 0.5;
                    nu = (nu * 2.0).min(10.0);
                }
                
                // Update nu for next iteration
                if gain_ratio > 0.75 {
                    nu = 2.0;
                } else if gain_ratio < 0.25 {
                    nu = (nu * 1.5).min(10.0);
                }
            } else {
                return Err("Failed to solve linear system in Levenberg-Marquardt".to_string());
            }

            // Prevent excessive damping or trust region collapse
            if lambda > 1e6 || trust_radius < 0.01 {
                break;
            }
        }

        let final_cost = {
            let (residuals, _) = self.compute_residuals_and_jacobian(&positions, &ranges, &current_estimate);
            residuals.dot(&residuals)
        };

        let geodetic_pos = self.local_to_geodetic(&current_estimate, reference_pos);
        Ok((geodetic_pos, current_estimate, final_cost))
    }

    /// Weighted least squares with measurement reliability weighting
    pub fn weighted_least_squares_trilateration(
        &self,
        anchors: &[Anchor],
        receiver_time_ms: u64,
        reliability_weights: &[f64],
    ) -> Result<(Position, Vector3<f64>, f64), String> {
        if anchors.len() < 3 {
            return Err("At least 3 anchors required for weighted LS trilateration".to_string());
        }

        if reliability_weights.len() != anchors.len() {
            return Err("Reliability weights must match number of anchors".to_string());
        }

        let reference_pos = &anchors[0].position;
        let mut positions = Vec::new();
        let mut ranges = Vec::new();

        for anchor in anchors {
            let local = self.geodetic_to_local(&anchor.position, reference_pos);
            positions.push(local);

            let dt_ms = receiver_time_ms as i64 - anchor.timestamp as i64;
            if dt_ms < 0 {
                return Err(format!("Receiver time earlier than anchor time for anchor {}", anchor.id));
            }
            let dt_sec = dt_ms as f64 / 1000.0;
            let range = SPEED_OF_SOUND_WATER * dt_sec;
            ranges.push(range);
        }

        let result = self.weighted_least_squares(&positions, &ranges, reliability_weights)?;
        
        // Calculate weighted residual sum of squares for quality assessment
        let residual_cost = self.calculate_weighted_residuals(&positions, &ranges, reliability_weights, &result);
        
        let geodetic_pos = self.local_to_geodetic(&result, reference_pos);
        Ok((geodetic_pos, result, residual_cost))
    }

    /// Robust estimation with outlier detection and mitigation
    pub fn robust_trilateration(
        &self,
        anchors: &[Anchor],
        receiver_time_ms: u64,
    ) -> Result<(Position, Vector3<f64>, Vec<bool>), String> {
        if anchors.len() < 4 {
            return Err("At least 4 anchors required for robust trilateration".to_string());
        }

        let reference_pos = &anchors[0].position;
        let mut positions = Vec::new();
        let mut ranges = Vec::new();

        for anchor in anchors {
            let local = self.geodetic_to_local(&anchor.position, reference_pos);
            positions.push(local);

            let dt_ms = receiver_time_ms as i64 - anchor.timestamp as i64;
            if dt_ms < 0 {
                return Err(format!("Receiver time earlier than anchor time for anchor {}", anchor.id));
            }
            let dt_sec = dt_ms as f64 / 1000.0;
            let range = SPEED_OF_SOUND_WATER * dt_sec;
            ranges.push(range);
        }

        // Initial estimate using all measurements
        let initial_estimate = self.linear_least_squares(&positions, &ranges)?;
        
        // Detect outliers using residual analysis
        let outlier_mask = self.detect_outliers(&positions, &ranges, &initial_estimate);
        
        // Re-estimate using only inlier measurements
        let inlier_positions: Vec<_> = positions.iter().enumerate()
            .filter(|(i, _)| !outlier_mask[*i])
            .map(|(_, pos)| *pos)
            .collect();
        let inlier_ranges: Vec<_> = ranges.iter().enumerate()
            .filter(|(i, _)| !outlier_mask[*i])
            .map(|(_, range)| *range)
            .collect();

        if inlier_positions.len() < 3 {
            return Err("Too many outliers detected - insufficient inlier measurements".to_string());
        }

        // Use uniform weights for inliers
        let weights = vec![1.0; inlier_positions.len()];
        let robust_estimate = self.weighted_least_squares(&inlier_positions, &inlier_ranges, &weights)?;
        
        let geodetic_pos = self.local_to_geodetic(&robust_estimate, reference_pos);
        Ok((geodetic_pos, robust_estimate, outlier_mask))
    }

    /// Kalman filter-enhanced trilateration with temporal smoothing and prediction
    pub fn kalman_enhanced_trilateration(
        &mut self,
        anchors: &[Anchor],
        receiver_time_ms: u64,
        measurement_uncertainty: Option<nalgebra::Matrix3<f64>>,
    ) -> Result<(Position, Vector3<f64>, Vector3<f64>), String> {
        if anchors.len() < 3 {
            return Err("At least 3 anchors required for Kalman-enhanced trilateration".to_string());
        }

        // First, get raw trilateration result using robust estimation
        let (raw_position, raw_local_pos, _): (Position, Vector3<f64>, Vector3<bool>) = if anchors.len() >= 4 {
            let (pos, local, _outliers) = self.robust_trilateration(anchors, receiver_time_ms)?;
            (pos, local, Vector3::new(false, false, false)) // Placeholder for outliers
        } else {
            // Use weighted least squares for 3 anchors
            let weights = vec![1.0; anchors.len()];
            let (pos, local, _cost) = self.weighted_least_squares_trilateration(anchors, receiver_time_ms, &weights)?;
            (pos, local, Vector3::zeros())
        };

        // Initialize Kalman filter if not already done
        if self.kalman_filter.is_none() {
            let mut filter = PositionKalmanFilter::new();
            filter.initialize(&raw_local_pos, None);
            self.kalman_filter = Some(filter);
        }

        // Update Kalman filter with new measurement
        let filter = self.kalman_filter.as_mut().unwrap();
        let filtered_local_pos = filter.update(&raw_local_pos, measurement_uncertainty);
        
        // Get velocity estimate from Kalman filter
        let velocity_estimate = filter.get_velocity();
        
        // Convert filtered position back to geodetic coordinates
        let reference_pos = &anchors[0].position;
        let filtered_geodetic_pos = self.local_to_geodetic(&filtered_local_pos, reference_pos);
        
        Ok((filtered_geodetic_pos, filtered_local_pos, velocity_estimate))
    }

    /// Predict future position using Kalman filter
    pub fn predict_future_position(
        &self,
        future_time_seconds: f64,
        reference_position: &Position,
    ) -> Result<(Position, Vector3<f64>), String> {
        if let Some(ref filter) = self.kalman_filter {
            if !filter.is_initialized() {
                return Err("Kalman filter not initialized".to_string());
            }
            
            let predicted_local_pos = filter.predict_future_position(future_time_seconds);
            let predicted_geodetic_pos = self.local_to_geodetic(&predicted_local_pos, reference_position);
            
            Ok((predicted_geodetic_pos, predicted_local_pos))
        } else {
            Err("Kalman filter not available".to_string())
        }
    }

    /// Get current velocity estimate from Kalman filter
    pub fn get_velocity_estimate(&self) -> Option<Vector3<f64>> {
        self.kalman_filter.as_ref().map(|filter| filter.get_velocity())
    }

    /// Get position uncertainty estimate from Kalman filter
    pub fn get_position_uncertainty(&self) -> Option<Vector3<f64>> {
        self.kalman_filter.as_ref().map(|filter| filter.get_position_uncertainty())
    }

    /// Reset Kalman filter state
    pub fn reset_kalman_filter(&mut self) {
        if let Some(ref mut filter) = self.kalman_filter {
            filter.reset();
        }
    }

    /// Enable Kalman filtering with custom noise parameters
    pub fn enable_kalman_filtering(
        &mut self,
        position_process_noise: f64,
        velocity_process_noise: f64,
        measurement_noise: f64,
    ) {
        let filter = PositionKalmanFilter::with_noise_parameters(
            position_process_noise,
            velocity_process_noise,
            measurement_noise,
        );
        self.kalman_filter = Some(filter);
    }

    /// Disable Kalman filtering
    pub fn disable_kalman_filtering(&mut self) {
        self.kalman_filter = None;
    }

    /// Helper function: Weighted least squares implementation
    fn weighted_least_squares(
        &self,
        positions: &[Vector3<f64>],
        ranges: &[f64],
        weights: &[f64],
    ) -> Result<Vector3<f64>, String> {
        if positions.len() < 3 {
            return Err("At least 3 positions required".to_string());
        }

        let n = positions.len();
        let p1 = positions[0];
        
        // Handle 3-anchor case (2D solution)
        if n == 3 {
            let mut a_matrix = nalgebra::Matrix2::zeros();
            let mut b_vector = nalgebra::Vector2::zeros();
            let mut weight_matrix = nalgebra::Matrix2::zeros();

            for i in 1..3 {
                let pi = positions[i];
                let row = i - 1;
                
                a_matrix[(row, 0)] = 2.0 * (pi.x - p1.x);
                a_matrix[(row, 1)] = 2.0 * (pi.y - p1.y);
                
                b_vector[row] = ranges[0].powi(2) - ranges[i].powi(2)
                    + pi.x.powi(2) - p1.x.powi(2)
                    + pi.y.powi(2) - p1.y.powi(2)
                    + pi.z.powi(2) - p1.z.powi(2);
                
                weight_matrix[(row, row)] = weights[i];
            }

            // Weighted least squares: (A^T W A)^-1 A^T W b
            let at_w = a_matrix.transpose() * weight_matrix;
            let at_w_a = &at_w * &a_matrix;
            let at_w_b = &at_w * &b_vector;

            if let Some(inv_at_w_a) = at_w_a.try_inverse() {
                let xy_solution = inv_at_w_a * at_w_b;
                
                // Estimate depth as weighted average
                let total_weight: f64 = weights.iter().sum();
                let estimated_depth: f64 = positions.iter()
                    .zip(weights.iter())
                    .map(|(p, w)| p.z * w / total_weight)
                    .sum();
                
                return Ok(Vector3::new(xy_solution.x, xy_solution.y, estimated_depth));
            } else {
                return Err("Singular matrix in 2D weighted least squares".to_string());
            }
        }

        // 3D case with 4+ anchors
        let mut a_matrix = DMatrix::zeros(n - 1, 3);
        let mut b_vector = DVector::zeros(n - 1);
        let mut weight_matrix = DMatrix::zeros(n - 1, n - 1);

        for i in 1..n {
            let pi = positions[i];
            let row = i - 1;
            
            a_matrix[(row, 0)] = 2.0 * (pi.x - p1.x);
            a_matrix[(row, 1)] = 2.0 * (pi.y - p1.y);
            a_matrix[(row, 2)] = 2.0 * (pi.z - p1.z);
            
            b_vector[row] = ranges[0].powi(2) - ranges[i].powi(2)
                + pi.x.powi(2) - p1.x.powi(2)
                + pi.y.powi(2) - p1.y.powi(2)
                + pi.z.powi(2) - p1.z.powi(2);
            
            weight_matrix[(row, row)] = weights[i];
        }

        // Weighted least squares solution
        let at_w = a_matrix.transpose() * &weight_matrix;
        let at_w_a = &at_w * &a_matrix;
        let at_w_b = &at_w * &b_vector;

        // Add regularization for numerical stability
        let mut regularized_matrix = at_w_a;
        for i in 0..3 {
            regularized_matrix[(i, i)] += self.regularization_lambda;
        }

        if let Some(solution) = self.solve_linear_system_dmatrix(&regularized_matrix, &at_w_b) {
            Ok(Vector3::new(solution[0], solution[1], solution[2]))
        } else {
            Err("Failed to solve weighted least squares system".to_string())
        }
    }

    /// Helper function: Linear least squares for initial estimates
    fn linear_least_squares(
        &self,
        positions: &[Vector3<f64>],
        ranges: &[f64],
    ) -> Result<Vector3<f64>, String> {
        let weights = vec![1.0; positions.len()];
        self.weighted_least_squares(positions, ranges, &weights)
    }

    /// Helper function: MLE optimization using gradient descent
    fn mle_optimization(
        &self,
        positions: &[Vector3<f64>],
        ranges: &[f64],
        weights: &[f64],
        initial_estimate: Vector3<f64>,
        noise_model: NoiseModel,
    ) -> Result<Vector3<f64>, String> {
        let mut current_estimate = initial_estimate;
        let mut learning_rate = 0.01;
        
        for _iteration in 0..self.max_iterations {
            let gradient = self.compute_mle_gradient(positions, ranges, weights, &current_estimate, noise_model);
            let gradient_norm = gradient.norm();
            
            if gradient_norm < self.convergence_tolerance {
                break;
            }
            
            // Gradient descent step
            let step = gradient * learning_rate;
            let new_estimate = current_estimate - step;
            
            // Check if step improves likelihood
            let current_likelihood = self.calculate_log_likelihood(positions, ranges, weights, &current_estimate, noise_model);
            let new_likelihood = self.calculate_log_likelihood(positions, ranges, weights, &new_estimate, noise_model);
            
            if new_likelihood > current_likelihood {
                current_estimate = new_estimate;
                learning_rate *= 1.1; // Increase learning rate on success
            } else {
                learning_rate *= 0.5; // Decrease learning rate on failure
            }
            
            // Prevent learning rate from becoming too small
            if learning_rate < 1e-8 {
                break;
            }
        }
        
        Ok(current_estimate)
    }

    /// Helper function: Compute MLE gradient
    fn compute_mle_gradient(
        &self,
        positions: &[Vector3<f64>],
        ranges: &[f64],
        weights: &[f64],
        estimate: &Vector3<f64>,
        noise_model: NoiseModel,
    ) -> Vector3<f64> {
        let mut gradient = Vector3::zeros();
        
        for i in 0..positions.len() {
            let diff = estimate - positions[i];
            let predicted_range = diff.norm();
            let residual = predicted_range - ranges[i];
            
            if predicted_range > 1e-10 {
                let unit_vector = diff / predicted_range;
                
                // Apply different gradient calculations based on noise model
                match noise_model {
                    NoiseModel::Gaussian => {
                        // Standard Gaussian noise model
                        gradient += unit_vector * residual * weights[i];
                    },
                    NoiseModel::Rayleigh => {
                        // Rayleigh noise model (common in underwater acoustics)
                        // For Rayleigh, gradient includes additional terms
                        if ranges[i] > 0.0 {
                            let scale_factor = 1.0 - (ranges[i] / predicted_range);
                            gradient += unit_vector * scale_factor * weights[i];
                        }
                    },
                    NoiseModel::Mixed(gaussian_weight) => {
                        // Mixed Gaussian-Rayleigh model
                        let gaussian_term = unit_vector * residual * weights[i];
                        
                        let rayleigh_term = if ranges[i] > 0.0 {
                            let scale_factor = 1.0 - (ranges[i] / predicted_range);
                            unit_vector * scale_factor * weights[i]
                        } else {
                            Vector3::zeros()
                        };
                        
                        gradient += gaussian_term * gaussian_weight + rayleigh_term * (1.0 - gaussian_weight);
                    },
                    NoiseModel::StudentT(dof) => {
                        // Student's t-distribution for heavy-tailed noise
                        let scale_factor = (dof + 1.0) * residual / (dof + (residual / weights[i]).powi(2));
                        gradient += unit_vector * scale_factor * weights[i];
                    },
                }
            }
        }
        
        gradient
    }

    /// Helper function: Calculate log-likelihood for MLE
    fn calculate_log_likelihood(
        &self,
        positions: &[Vector3<f64>],
        ranges: &[f64],
        weights: &[f64],
        estimate: &Vector3<f64>,
        noise_model: NoiseModel,
    ) -> f64 {
        let mut log_likelihood = 0.0;
        
        for i in 0..positions.len() {
            let diff = estimate - positions[i];
            let predicted_range = diff.norm();
            let residual = predicted_range - ranges[i];
            
            // Calculate log-likelihood based on noise model
            match noise_model {
                NoiseModel::Gaussian => {
                    // Gaussian noise: log p(r|x) = -0.5 * w * residual^2 + const
                    log_likelihood -= 0.5 * weights[i] * residual.powi(2);
                },
                NoiseModel::Rayleigh => {
                    // Rayleigh noise model
                    if ranges[i] > 0.0 && predicted_range > 0.0 {
                        let sigma_sq = 1.0 / weights[i];
                        log_likelihood += (ranges[i] / sigma_sq) * (predicted_range - ranges[i] / 2.0);
                    }
                },
                NoiseModel::Mixed(gaussian_weight) => {
                    // Mixed Gaussian-Rayleigh model
                    let gaussian_ll = -0.5 * weights[i] * residual.powi(2);
                    
                    let rayleigh_ll = if ranges[i] > 0.0 && predicted_range > 0.0 {
                        let sigma_sq = 1.0 / weights[i];
                        (ranges[i] / sigma_sq) * (predicted_range - ranges[i] / 2.0)
                    } else {
                        0.0
                    };
                    
                    log_likelihood += gaussian_weight * gaussian_ll + (1.0 - gaussian_weight) * rayleigh_ll;
                },
                NoiseModel::StudentT(dof) => {
                    // Student's t-distribution for heavy-tailed noise
                    let scaled_residual = residual * weights[i].sqrt();
                    log_likelihood -= ((dof + 1.0) / 2.0) * (1.0 + scaled_residual.powi(2) / dof).ln();
                },
            }
        }
        
        log_likelihood
    }

    /// Enhanced log-likelihood calculation with model selection criteria
    fn calculate_enhanced_log_likelihood(
        &self,
        positions: &[Vector3<f64>],
        ranges: &[f64],
        weights: &[f64],
        estimate: &Vector3<f64>,
        noise_model: NoiseModel,
    ) -> f64 {
        let n = positions.len() as f64;
        let k = 3.0; // Number of parameters (x, y, z)
        
        // Standard log-likelihood based on noise model
        let log_likelihood = self.calculate_log_likelihood(positions, ranges, weights, estimate, noise_model);
        
        // Add AIC (Akaike Information Criterion) penalty for model complexity
        let aic_penalty = k;
        
        // Add BIC (Bayesian Information Criterion) penalty
        let bic_penalty = 0.5 * k * n.ln();
        
        // Enhanced likelihood with model selection penalties
        log_likelihood - aic_penalty - bic_penalty
    }

    /// Estimate adaptive variance based on local measurement consistency
    fn estimate_adaptive_variance(&self, base_variance: f64, ranges: &[f64], current_idx: usize) -> f64 {
        if ranges.len() < 3 {
            return base_variance;
        }
        
        // Calculate local variance based on range consistency with neighbors
        let current_range = ranges[current_idx];
        let mut local_deviations = Vec::new();
        
        for (i, &range) in ranges.iter().enumerate() {
            if i != current_idx {
                local_deviations.push((current_range - range).abs());
            }
        }
        
        if local_deviations.is_empty() {
            return base_variance;
        }
        
        // Calculate median absolute deviation for robust variance estimation
        local_deviations.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mad = local_deviations[local_deviations.len() / 2];
        
        // Adaptive variance combines base variance with local consistency
        let consistency_factor = 1.0 + (mad / (current_range + 1.0)).min(2.0);
        base_variance * consistency_factor
    }

    /// Robust MLE optimization with M-estimator loss function
    fn robust_mle_optimization(
        &self,
        positions: &[Vector3<f64>],
        ranges: &[f64],
        weights: &[f64],
        initial_estimate: Vector3<f64>,
        noise_model: NoiseModel,
    ) -> Result<Vector3<f64>, String> {
        let mut current_estimate = initial_estimate;
        let mut learning_rate = 0.005; // Smaller learning rate for robust optimization
        let huber_threshold = 2.0; // Threshold for Huber loss function
        
        for iteration in 0..self.max_iterations {
            let gradient = self.compute_robust_mle_gradient(
                positions, ranges, weights, &current_estimate, huber_threshold, noise_model
            );
            let gradient_norm = gradient.norm();
            
            if gradient_norm < self.convergence_tolerance {
                break;
            }
            
            // Adaptive learning rate based on iteration
            let adaptive_rate = learning_rate * (0.95_f64).powi(iteration as i32 / 10);
            
            // Gradient descent step with momentum
            let step = gradient * adaptive_rate;
            let new_estimate = current_estimate - step;
            
            // Check if step improves robust likelihood
            let current_likelihood = self.calculate_robust_log_likelihood(
                positions, ranges, weights, &current_estimate, huber_threshold, noise_model
            );
            let new_likelihood = self.calculate_robust_log_likelihood(
                positions, ranges, weights, &new_estimate, huber_threshold, noise_model
            );
            
            if new_likelihood > current_likelihood {
                current_estimate = new_estimate;
            } else {
                learning_rate *= 0.8; // Reduce learning rate if no improvement
            }
            
            // Prevent learning rate from becoming too small
            if learning_rate < 1e-10 {
                break;
            }
        }
        
        Ok(current_estimate)
    }

    /// Compute robust MLE gradient using Huber loss function
    fn compute_robust_mle_gradient(
        &self,
        positions: &[Vector3<f64>],
        ranges: &[f64],
        weights: &[f64],
        estimate: &Vector3<f64>,
        huber_threshold: f64,
        noise_model: NoiseModel,
    ) -> Vector3<f64> {
        let mut gradient = Vector3::zeros();
        
        for i in 0..positions.len() {
            let diff = estimate - positions[i];
            let predicted_range = diff.norm();
            let residual = predicted_range - ranges[i];
            let weighted_residual = residual * weights[i].sqrt();
            
            if predicted_range > 1e-10 {
                let unit_vector = diff / predicted_range;
                
                // Apply Huber loss function for robustness
                let influence_function = if weighted_residual.abs() <= huber_threshold {
                    // Quadratic region (standard least squares)
                    weighted_residual
                } else {
                    // Linear region (less influence from outliers)
                    huber_threshold * weighted_residual.signum()
                };
                
                // Apply noise model-specific adjustments
                let adjusted_influence = match noise_model {
                    NoiseModel::Gaussian => influence_function,
                    NoiseModel::Rayleigh => {
                        if ranges[i] > 0.0 {
                            influence_function * (1.0 - ranges[i] / predicted_range.max(ranges[i]))
                        } else {
                            influence_function
                        }
                    },
                    NoiseModel::Mixed(gaussian_weight) => {
                        let rayleigh_factor = if ranges[i] > 0.0 {
                            1.0 - ranges[i] / predicted_range.max(ranges[i])
                        } else {
                            1.0
                        };
                        influence_function * (gaussian_weight + (1.0 - gaussian_weight) * rayleigh_factor)
                    },
                    NoiseModel::StudentT(dof) => {
                        influence_function * (dof + 1.0) / (dof + weighted_residual.powi(2))
                    },
                };
                
                gradient += unit_vector * adjusted_influence * weights[i].sqrt();
            }
        }
        
        gradient
    }

    /// Calculate robust log-likelihood with Huber loss
    fn calculate_robust_log_likelihood(
        &self,
        positions: &[Vector3<f64>],
        ranges: &[f64],
        weights: &[f64],
        estimate: &Vector3<f64>,
        huber_threshold: f64,
        noise_model: NoiseModel,
    ) -> f64 {
        let mut log_likelihood = 0.0;
        
        for i in 0..positions.len() {
            let diff = estimate - positions[i];
            let predicted_range = diff.norm();
            let residual = predicted_range - ranges[i];
            let weighted_residual = residual * weights[i].sqrt();
            
            // Huber loss function
            let huber_loss = if weighted_residual.abs() <= huber_threshold {
                0.5 * weighted_residual.powi(2)
            } else {
                huber_threshold * (weighted_residual.abs() - 0.5 * huber_threshold)
            };
            
            // Apply noise model-specific adjustments
            let adjusted_loss = match noise_model {
                NoiseModel::Gaussian => huber_loss,
                NoiseModel::Rayleigh => {
                    if ranges[i] > 0.0 && predicted_range > 0.0 {
                        let rayleigh_factor = ranges[i] / predicted_range;
                        huber_loss * (1.0 - rayleigh_factor.min(1.0))
                    } else {
                        huber_loss
                    }
                },
                NoiseModel::Mixed(gaussian_weight) => {
                    let rayleigh_component = if ranges[i] > 0.0 && predicted_range > 0.0 {
                        let rayleigh_factor = ranges[i] / predicted_range;
                        huber_loss * (1.0 - rayleigh_factor.min(1.0))
                    } else {
                        huber_loss
                    };
                    gaussian_weight * huber_loss + (1.0 - gaussian_weight) * rayleigh_component
                },
                NoiseModel::StudentT(dof) => {
                    (dof + 1.0) * (1.0 + weighted_residual.powi(2) / dof).ln() / 2.0
                },
            };
            
            log_likelihood -= adjusted_loss;
        }
        
        log_likelihood
    }

    /// Compute residuals and Jacobian for Levenberg-Marquardt
    fn compute_residuals_and_jacobian(
        &self,
        positions: &[Vector3<f64>],
        ranges: &[f64],
        estimate: &Vector3<f64>,
    ) -> (DVector<f64>, DMatrix<f64>) {
        let n = positions.len();
        let mut residuals = DVector::zeros(n);
        let mut jacobian = DMatrix::zeros(n, 3);
        
        for i in 0..n {
            let diff = estimate - positions[i];
            let predicted_range = diff.norm();
            
            // Residual: measured range - predicted range
            residuals[i] = ranges[i] - predicted_range;
            
            // Jacobian: derivative of residual with respect to position
            if predicted_range > 1e-10 {
                let unit_vector = diff / predicted_range;
                jacobian[(i, 0)] = -unit_vector.x;
                jacobian[(i, 1)] = -unit_vector.y;
                jacobian[(i, 2)] = -unit_vector.z;
            }
        }
        
        (residuals, jacobian)
    }

    /// Compute predicted reduction in cost for Levenberg-Marquardt
    fn compute_predicted_reduction(
        &self,
        residuals: &DVector<f64>,
        jacobian: &DMatrix<f64>,
        step: &Vector3<f64>,
    ) -> f64 {
        let step_dv = DVector::from_column_slice(&[step.x, step.y, step.z]);
        let linear_term = jacobian * &step_dv;
        
        // Predicted reduction = -2 * residuals^T * J * step + step^T * J^T * J * step
        -2.0 * residuals.dot(&linear_term) + linear_term.dot(&linear_term)
    }

    /// Solve linear system for DMatrix
    fn solve_linear_system_dmatrix(
        &self,
        a: &DMatrix<f64>,
        b: &DVector<f64>,
    ) -> Option<DVector<f64>> {
        let svd = a.svd(true, true);
        svd.solve(b, 1e-10).ok()
    }

    /// Compute robust initial guess for iterative methods
    fn compute_robust_initial_guess(
        &self,
        positions: &[Vector3<f64>],
        ranges: &[f64],
    ) -> Result<Vector3<f64>, String> {
        // Try multiple initial guesses and pick the best one
        
        // Method 1: Centroid of anchor positions
        let centroid = positions.iter().fold(Vector3::zeros(), |acc, p| acc + p) / positions.len() as f64;
        
        // Method 2: Linear least squares
        let linear_ls = match self.linear_least_squares(positions, ranges) {
            Ok(pos) => pos,
            Err(_) => centroid, // Fallback to centroid if linear LS fails
        };
        
        // Method 3: Weighted centroid based on ranges
        let total_inverse_range: f64 = ranges.iter().map(|r| 1.0 / r.max(0.1)).sum();
        let weighted_centroid = positions.iter().zip(ranges.iter())
            .fold(Vector3::zeros(), |acc, (p, r)| acc + p * (1.0 / r.max(0.1) / total_inverse_range));
        
        // Evaluate each guess and pick the best one
        let candidates = [centroid, linear_ls, weighted_centroid];
        let mut best_cost = f64::INFINITY;
        let mut best_guess = centroid;
        
        for guess in &candidates {
            let mut cost = 0.0;
            for (i, pos) in positions.iter().enumerate() {
                let diff = guess - pos;
                let predicted_range = diff.norm();
                let residual = predicted_range - ranges[i];
                cost += residual.powi(2);
            }
            
            if cost < best_cost {
                best_cost = cost;
                best_guess = *guess;
            }
        }
        
        Ok(best_guess)
    }

    /// Detect outliers based on residual analysis
    fn detect_outliers(
        &self,
        positions: &[Vector3<f64>],
        ranges: &[f64],
        estimate: &Vector3<f64>,
    ) -> Vec<bool> {
        let mut residuals = Vec::with_capacity(positions.len());
        
        // Calculate residuals
        for (i, pos) in positions.iter().enumerate() {
            let diff = estimate - pos;
            let predicted_range = diff.norm();
            let residual = (predicted_range - ranges[i]).abs();
            residuals.push(residual);
        }
        
        // Calculate median and MAD (Median Absolute Deviation)
        let mut sorted_residuals = residuals.clone();
        sorted_residuals.sort_by(|a, b| a.partial_cmp(b).unwrap());
        
        let median = if sorted_residuals.len() % 2 == 0 {
            (sorted_residuals[sorted_residuals.len() / 2 - 1] + sorted_residuals[sorted_residuals.len() / 2]) / 2.0
        } else {
            sorted_residuals[sorted_residuals.len() / 2]
        };
        
        let mut abs_deviations: Vec<f64> = residuals.iter().map(|r| (r - median).abs()).collect();
        abs_deviations.sort_by(|a, b| a.partial_cmp(b).unwrap());
        
        let mad = if abs_deviations.len() % 2 == 0 {
            (abs_deviations[abs_deviations.len() / 2 - 1] + abs_deviations[abs_deviations.len() / 2]) / 2.0
        } else {
            abs_deviations[abs_deviations.len() / 2]
        };
        
        // Normalize MAD (for normal distribution)
        let mad_normalized = mad / 0.6745;
        
        // Mark outliers
        residuals.iter().map(|r| {
            let z_score = (r - median).abs() / mad_normalized.max(1e-10);
            z_score > self.outlier_threshold
        }).collect()
    }

    /// Calculate weighted residuals for quality assessment
    fn calculate_weighted_residuals(
        &self,
        positions: &[Vector3<f64>],
        ranges: &[f64],
        weights: &[f64],
        estimate: &Vector3<f64>,
    ) -> f64 {
        let mut weighted_rss = 0.0;
        
        for i in 0..positions.len() {
            let diff = estimate - positions[i];
            let predicted_range = diff.norm();
            let residual = predicted_range - ranges[i];
            weighted_rss += weights[i] * residual.powi(2);
        }
        
        weighted_rss
    }

    /// Convert geodetic coordinates to local tangent plane
    fn geodetic_to_local(&self, pos: &Position, reference: &Position) -> Vector3<f64> {
        // Approx meters per degree at reference latitude (valid for very small areas)
        let lat0_rad = pos.lat * PI / 180.0;
        let meters_per_deg_lat = 111_132.0; // roughly constant
        let meters_per_deg_lon = 111_320.0 * lat0_rad.cos();

        let dlat = pos.lat - reference.lat;
        let dlon = pos.lon - reference.lon;

        let north = dlat * meters_per_deg_lat;
        let east = dlon * meters_per_deg_lon;
        let down = pos.depth;

        Vector3::new(east, north, down)
    }

    /// Convert local tangent plane coordinates to geodetic
    fn local_to_geodetic(&self, local_pos: &Vector3<f64>, reference: &Position) -> Position {
        let lat0_rad = reference.lat * PI / 180.0;
        let meters_per_deg_lat = 111_132.0;
        let meters_per_deg_lon = 111_320.0 * lat0_rad.cos();

        let east = local_pos.x;
        let north = local_pos.y;
        let down = local_pos.z;

        let lat_est = reference.lat + north / meters_per_deg_lat;
        let lon_est = reference.lon + east / meters_per_deg_lon;
        let depth_est = down;

        Position {
            lat: lat_est,
            lon: lon_est,
            depth: depth_est,
        }
    }
}

/// Noise models for maximum likelihood estimation
#[derive(Debug, Clone, Copy)]
pub enum NoiseModel {
    /// Standard Gaussian noise model
    Gaussian,
    /// Rayleigh noise model (common in underwater acoustics)
    Rayleigh,
    /// Mixed Gaussian-Rayleigh model with weight for Gaussian component
    Mixed(f64),
    /// Student's t-distribution for heavy-tailed noise (degrees of freedom)
    StudentT(f64),
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;

    #[test]
    fn test_mle_trilateration() {
        let trilateration = AdvancedTrilateration::new();
        
        // Create test anchors
        let anchors = vec![
            Anchor {
                id: "1".to_string(),
                timestamp: 1000,
                position: Position { lat: 0.0, lon: 0.0, depth: 0.0 },
            },
            Anchor {
                id: "2".to_string(),
                timestamp: 1010,
                position: Position { lat: 0.001, lon: 0.0, depth: 0.0 },
            },
            Anchor {
                id: "3".to_string(),
                timestamp: 1020,
                position: Position { lat: 0.0, lon: 0.001, depth: 0.0 },
            },
            Anchor {
                id: "4".to_string(),
                timestamp: 1030,
                position: Position { lat: 0.0, lon: 0.0, depth: 10.0 },
            },
        ];
        
        let receiver_time = 1050;
        let variances = vec![1.0, 1.0, 1.0, 1.0];
        
        // Test with Gaussian noise model
        let result = trilateration.mle_trilateration(&anchors, receiver_time, &variances, NoiseModel::Gaussian);
        assert!(result.is_ok());
        
        if let Ok((pos, _, _)) = result {
            println!("MLE position: lat={}, lon={}, depth={}", pos.lat, pos.lon, pos.depth);
            assert!(pos.lat.abs() < 0.01);
            assert!(pos.lon.abs() < 0.01);
        }
        
        // Test with Rayleigh noise model
        let result = trilateration.mle_trilateration(&anchors, receiver_time, &variances, NoiseModel::Rayleigh);
        assert!(result.is_ok());
    }

    #[test]
    fn test_levenberg_marquardt() {
        let trilateration = AdvancedTrilateration::new();
        
        // Create test anchors
        let anchors = vec![
            Anchor {
                id: "1".to_string(),
                timestamp: 1000,
                position: Position { lat: 0.0, lon: 0.0, depth: 0.0 },
            },
            Anchor {
                id: "2".to_string(),
                timestamp: 1010,
                position: Position { lat: 0.001, lon: 0.0, depth: 0.0 },
            },
            Anchor {
                id: "3".to_string(),
                timestamp: 1020,
                position: Position { lat: 0.0, lon: 0.001, depth: 0.0 },
            },
            Anchor {
                id: "4".to_string(),
                timestamp: 1030,
                position: Position { lat: 0.0, lon: 0.0, depth: 10.0 },
            },
        ];
        
        let receiver_time = 1050;
        
        // Test with no initial guess
        let result = trilateration.levenberg_marquardt_trilateration(&anchors, receiver_time, None);
        assert!(result.is_ok());
        
        if let Ok((pos, _, cost)) = result {
            println!("LM position: lat={}, lon={}, depth={}, cost={}", pos.lat, pos.lon, pos.depth, cost);
            assert!(pos.lat.abs() < 0.01);
            assert!(pos.lon.abs() < 0.01);
        }
        
        // Test with initial guess
        let initial_guess = Vector3::new(10.0, 10.0, 10.0); // Bad initial guess
        let result = trilateration.levenberg_marquardt_trilateration(&anchors, receiver_time, Some(initial_guess));
        assert!(result.is_ok());
    }

    #[test]
    fn test_weighted_least_squares() {
        let trilateration = AdvancedTrilateration::new();
        
        // Create test anchors
        let anchors = vec![
            Anchor {
                id: "1".to_string(),
                timestamp: 1000,
                position: Position { lat: 0.0, lon: 0.0, depth: 0.0 },
            },
            Anchor {
                id: "2".to_string(),
                timestamp: 1010,
                position: Position { lat: 0.001, lon: 0.0, depth: 0.0 },
            },
            Anchor {
                id: "3".to_string(),
                timestamp: 1020,
                position: Position { lat: 0.0, lon: 0.001, depth: 0.0 },
            },
            Anchor {
                id: "4".to_string(),
                timestamp: 1030,
                position: Position { lat: 0.0, lon: 0.0, depth: 10.0 },
            },
        ];
        
        let receiver_time = 1050;
        
        // Test with uniform weights
        let weights = vec![1.0, 1.0, 1.0, 1.0];
        let result = trilateration.weighted_least_squares_trilateration(&anchors, receiver_time, &weights);
        assert!(result.is_ok());
        
        // Test with non-uniform weights
        let weights = vec![1.0, 0.5, 0.5, 2.0]; // More weight on depth measurement
        let result = trilateration.weighted_least_squares_trilateration(&anchors, receiver_time, &weights);
        assert!(result.is_ok());
        
        if let Ok((pos, _, _)) = result {
            println!("WLS position: lat={}, lon={}, depth={}", pos.lat, pos.lon, pos.depth);
        }
    }

    #[test]
    fn test_robust_trilateration() {
        let trilateration = AdvancedTrilateration::new();
        
        // Create test anchors with one outlier
        let anchors = vec![
            Anchor {
                id: "1".to_string(),
                timestamp: 1000,
                position: Position { lat: 0.0, lon: 0.0, depth: 0.0 },
            },
            Anchor {
                id: "2".to_string(),
                timestamp: 1010,
                position: Position { lat: 0.001, lon: 0.0, depth: 0.0 },
            },
            Anchor {
                id: "3".to_string(),
                timestamp: 1020,
                position: Position { lat: 0.0, lon: 0.001, depth: 0.0 },
            },
            Anchor {
                id: "4".to_string(),
                timestamp: 1030,
                position: Position { lat: 0.0, lon: 0.0, depth: 10.0 },
            },
            // Outlier anchor with inconsistent timing
            Anchor {
                id: "5".to_string(),
                timestamp: 900, // Much earlier timestamp -> much larger range
                position: Position { lat: 0.001, lon: 0.001, depth: 5.0 },
            },
        ];
        
        let receiver_time = 1050;
        
        let result = trilateration.robust_trilateration(&anchors, receiver_time);
        assert!(result.is_ok());
        
        if let Ok((pos, _, outliers)) = result {
            println!("Robust position: lat={}, lon={}, depth={}", pos.lat, pos.lon, pos.depth);
            println!("Outliers detected: {:?}", outliers);
            
            // The last anchor should be detected as an outlier
            assert!(outliers[4]);
        }
    }

    #[test]
    fn test_kalman_enhanced_trilateration() {
        let mut trilateration = AdvancedTrilateration::new();
        
        // Create test anchors
        let anchors = vec![
            Anchor {
                id: "1".to_string(),
                timestamp: 1000,
                position: Position { lat: 0.0, lon: 0.0, depth: 0.0 },
            },
            Anchor {
                id: "2".to_string(),
                timestamp: 1010,
                position: Position { lat: 0.001, lon: 0.0, depth: 0.0 },
            },
            Anchor {
                id: "3".to_string(),
                timestamp: 1020,
                position: Position { lat: 0.0, lon: 0.001, depth: 0.0 },
            },
            Anchor {
                id: "4".to_string(),
                timestamp: 1030,
                position: Position { lat: 0.0, lon: 0.0, depth: 10.0 },
            },
        ];
        
        let receiver_time = 1050;
        
        // First position update
        let result1 = trilateration.kalman_enhanced_trilateration(&anchors, receiver_time, None);
        assert!(result1.is_ok());
        
        // Second position update (should be smoothed)
        let receiver_time2 = 1100;
        let result2 = trilateration.kalman_enhanced_trilateration(&anchors, receiver_time2, None);
        assert!(result2.is_ok());
        
        if let (Ok((pos1, _, _)), Ok((pos2, _, vel))) = (result1, result2) {
            println!("Kalman position 1: lat={}, lon={}, depth={}", pos1.lat, pos1.lon, pos1.depth);
            println!("Kalman position 2: lat={}, lon={}, depth={}", pos2.lat, pos2.lon, pos2.depth);
            println!("Velocity estimate: x={}, y={}, z={} m/s", vel.x, vel.y, vel.z);
        }
    }

    #[test]
    fn test_performance_comparison() {
        let mut trilateration = AdvancedTrilateration::new();
        
        // Create test anchors
        let anchors = vec![
            Anchor {
                id: "1".to_string(),
                timestamp: 1000,
                position: Position { lat: 0.0, lon: 0.0, depth: 0.0 },
            },
            Anchor {
                id: "2".to_string(),
                timestamp: 1010,
                position: Position { lat: 0.001, lon: 0.0, depth: 0.0 },
            },
            Anchor {
                id: "3".to_string(),
                timestamp: 1020,
                position: Position { lat: 0.0, lon: 0.001, depth: 0.0 },
            },
            Anchor {
                id: "4".to_string(),
                timestamp: 1030,
                position: Position { lat: 0.0, lon: 0.0, depth: 10.0 },
            },
        ];
        
        let receiver_time = 1050;
        let variances = vec![1.0, 1.0, 1.0, 1.0];
        
        println!("Performance comparison:");
        
        // Test MLE
        let start = Instant::now();
        let _ = trilateration.mle_trilateration(&anchors, receiver_time, &variances, NoiseModel::Gaussian);
        println!("MLE: {:?}", start.elapsed());
        
        // Test Levenberg-Marquardt
        let start = Instant::now();
        let _ = trilateration.levenberg_marquardt_trilateration(&anchors, receiver_time, None);
        println!("Levenberg-Marquardt: {:?}", start.elapsed());
        
        // Test Weighted Least Squares
        let start = Instant::now();
        let weights = vec![1.0, 1.0, 1.0, 1.0];
        let _ = trilateration.weighted_least_squares_trilateration(&anchors, receiver_time, &weights);
        println!("Weighted Least Squares: {:?}", start.elapsed());
        
        // Test Robust
        let start = Instant::now();
        let _ = trilateration.robust_trilateration(&anchors, receiver_time);
        println!("Robust: {:?}", start.elapsed());
        
        // Test Kalman
        let start = Instant::now();
        let _ = trilateration.kalman_enhanced_trilateration(&anchors, receiver_time, None);
        println!("Kalman: {:?}", start.elapsed());
    }
}

/// Geometric Dilution of Precision (GDOP) types and quality indicators
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum GdopQuality {
    /// Excellent geometry (GDOP < 2)
    Excellent,
    /// Good geometry (GDOP < 5)
    Good,
    /// Moderate geometry (GDOP < 10)
    Moderate,
    /// Fair geometry (GDOP < 20)
    Fair,
    /// Poor geometry (GDOP >= 20)
    Poor,
}

impl GdopQuality {
    /// Create quality assessment from GDOP value
    pub fn from_gdop(gdop: f64) -> Self {
        if gdop < 2.0 {
            GdopQuality::Excellent
        } else if gdop < 5.0 {
            GdopQuality::Good
        } else if gdop < 10.0 {
            GdopQuality::Moderate
        } else if gdop < 20.0 {
            GdopQuality::Fair
        } else {
            GdopQuality::Poor
        }
    }
    
    /// Get description of the quality
    pub fn description(&self) -> &'static str {
        match self {
            GdopQuality::Excellent => "Excellent geometry (ideal for precision positioning)",
            GdopQuality::Good => "Good geometry (suitable for most applications)",
            GdopQuality::Moderate => "Moderate geometry (acceptable for general navigation)",
            GdopQuality::Fair => "Fair geometry (limited accuracy, use with caution)",
            GdopQuality::Poor => "Poor geometry (significant accuracy degradation)",
        }
    }
}

/// Detailed GDOP components for different error dimensions
#[derive(Debug, Clone, Copy)]
pub struct DilutionOfPrecision {
    /// Geometric Dilution of Precision (overall)
    pub gdop: f64,
    /// Position Dilution of Precision (3D)
    pub pdop: f64,
    /// Horizontal Dilution of Precision (2D)
    pub hdop: f64,
    /// Vertical Dilution of Precision (depth/height)
    pub vdop: f64,
    /// Time Dilution of Precision (clock error)
    pub tdop: f64,
    /// Quality assessment based on GDOP
    pub quality: GdopQuality,
}

/// Anchor geometry assessment result with quality metrics
#[derive(Debug, Clone)]
pub struct AnchorGeometryAssessment {
    /// Dilution of precision metrics
    pub dop: DilutionOfPrecision,
    /// Condition number of the geometry matrix
    pub condition_number: f64,
    /// Tetrahedron volume for 3D configurations (normalized)
    pub normalized_volume: f64,
    /// Anchor quality scores (higher is better)
    pub anchor_scores: Vec<f64>,
    /// Recommended anchor subset (indices of best anchors)
    pub recommended_anchors: Vec<usize>,
}

impl AdvancedTrilateration {
    /// Calculate Geometric Dilution of Precision (GDOP) for a given anchor configuration
    /// Returns detailed DOP metrics and quality assessment
    pub fn calculate_gdop(
        &self,
        anchor_positions: &[Vector3<f64>],
        receiver_position: &Vector3<f64>,
    ) -> DilutionOfPrecision {
        if anchor_positions.len() < 3 {
            // Return worst-case values for insufficient anchors
            return DilutionOfPrecision {
                gdop: f64::INFINITY,
                pdop: f64::INFINITY,
                hdop: f64::INFINITY,
                vdop: f64::INFINITY,
                tdop: f64::INFINITY,
                quality: GdopQuality::Poor,
            };
        }
        
        // Build geometry matrix (H) where each row is a unit vector from receiver to anchor
        // plus a 1.0 for the time component
        let mut geometry_matrix = DMatrix::zeros(anchor_positions.len(), 4);
        
        for (i, anchor_pos) in anchor_positions.iter().enumerate() {
            let diff = anchor_pos - receiver_position;
            let distance = diff.norm();
            
            if distance > 1e-10 {
                // Unit vector components from receiver to anchor
                geometry_matrix[(i, 0)] = diff.x / distance;
                geometry_matrix[(i, 1)] = diff.y / distance;
                geometry_matrix[(i, 2)] = diff.z / distance;
                geometry_matrix[(i, 3)] = 1.0; // Time component
            }
        }
        
        // Calculate (H^T * H)^-1 which gives the covariance matrix
        let h_transpose = geometry_matrix.transpose();
        let h_square = &h_transpose * &geometry_matrix;
        
        // Use SVD for numerical stability when inverting
        let svd = h_square.svd(true, true);
        
        // Check if matrix is invertible
        if svd.singular_values[3] < 1e-10 {
            // Singular matrix - return worst-case values
            return DilutionOfPrecision {
                gdop: f64::INFINITY,
                pdop: f64::INFINITY,
                hdop: f64::INFINITY,
                vdop: f64::INFINITY,
                tdop: f64::INFINITY,
                quality: GdopQuality::Poor,
            };
        }
        
        // Reconstruct inverse from SVD components
        let mut s_inv = DMatrix::zeros(4, 4);
        for i in 0..4 {
            if svd.singular_values[i] > 1e-10 {
                s_inv[(i, i)] = 1.0 / svd.singular_values[i];
            } else {
                s_inv[(i, i)] = 0.0;
            }
        }
        
        let covariance = svd.u.unwrap() * s_inv * svd.v_t.unwrap();
        
        // Calculate DOP components from covariance matrix diagonal
        let gdop = (covariance[(0, 0)] + covariance[(1, 1)] + covariance[(2, 2)] + covariance[(3, 3)]).sqrt();
        let pdop = (covariance[(0, 0)] + covariance[(1, 1)] + covariance[(2, 2)]).sqrt();
        let hdop = (covariance[(0, 0)] + covariance[(1, 1)]).sqrt();
        let vdop = covariance[(2, 2)].sqrt();
        let tdop = covariance[(3, 3)].sqrt();
        
        // Create DOP result with quality assessment
        DilutionOfPrecision {
            gdop,
            pdop,
            hdop,
            vdop,
            tdop,
            quality: GdopQuality::from_gdop(gdop),
        }
    }
    
    /// Perform comprehensive anchor geometry assessment
    /// Evaluates GDOP, condition number, and normalized volume
    pub fn assess_anchor_geometry(
        &self,
        anchor_positions: &[Vector3<f64>],
        receiver_position: Option<&Vector3<f64>>,
    ) -> AnchorGeometryAssessment {
        // Use provided receiver position or estimate centroid
        let position = match receiver_position {
            Some(pos) => *pos,
            None => {
                // Calculate centroid as initial position estimate
                let mut centroid = Vector3::zeros();
                for pos in anchor_positions {
                    centroid += pos;
                }
                centroid /= anchor_positions.len() as f64;
                centroid
            }
        };
        
        // Calculate GDOP metrics
        let dop = self.calculate_gdop(anchor_positions, &position);
        
        // Calculate condition number using geometry matrix
        let mut geometry_matrix = DMatrix::zeros(anchor_positions.len(), 3);
        for (i, anchor_pos) in anchor_positions.iter().enumerate() {
            let diff = anchor_pos - &position;
            let distance = diff.norm();
            
            if distance > 1e-10 {
                geometry_matrix[(i, 0)] = diff.x / distance;
                geometry_matrix[(i, 1)] = diff.y / distance;
                geometry_matrix[(i, 2)] = diff.z / distance;
            }
        }
        
        let svd = geometry_matrix.svd(false, false);
        let condition_number = if svd.singular_values[svd.singular_values.len() - 1] > 1e-10 {
            svd.singular_values[0] / svd.singular_values[svd.singular_values.len() - 1]
        } else {
            f64::INFINITY
        };
        
        // Calculate normalized tetrahedron volume for 3D configurations
        let normalized_volume = if anchor_positions.len() >= 4 {
            self.calculate_normalized_tetrahedron_volume(anchor_positions)
        } else {
            0.0
        };
        
        // Calculate individual anchor quality scores
        let anchor_scores = self.calculate_anchor_quality_scores(anchor_positions, &position);
        
        // Determine recommended anchor subset based on scores
        let recommended_anchors = self.select_optimal_anchors(anchor_positions, &anchor_scores);
        
        AnchorGeometryAssessment {
            dop,
            condition_number,
            normalized_volume,
            anchor_scores,
            recommended_anchors,
        }
    }
    
    /// Calculate normalized tetrahedron volume for 3D anchor configurations
    /// Returns a value between 0 (degenerate) and 1 (optimal tetrahedron)
    fn calculate_normalized_tetrahedron_volume(&self, positions: &[Vector3<f64>]) -> f64 {
        if positions.len() < 4 {
            return 0.0;
        }
        
        // Calculate maximum distance between any two anchors for normalization
        let mut max_dist = 0.0;
        for i in 0..positions.len() {
            for j in i+1..positions.len() {
                let dist = (positions[i] - positions[j]).norm();
                max_dist = max_dist.max(dist);
            }
        }
        
        if max_dist < 1e-10 {
            return 0.0;
        }
        
        // Calculate volumes of all possible tetrahedra and take the maximum
        let mut max_volume = 0.0;
        
        for i in 0..positions.len() {
            for j in i+1..positions.len() {
                for k in j+1..positions.len() {
                    for l in k+1..positions.len() {
                        let p1 = positions[i];
                        let p2 = positions[j];
                        let p3 = positions[k];
                        let p4 = positions[l];
                        
                        let v1 = p2 - p1;
                        let v2 = p3 - p1;
                        let v3 = p4 - p1;
                        
                        let volume = (v1.cross(&v2)).dot(&v3).abs() / 6.0;
                        max_volume = max_volume.max(volume);
                    }
                }
            }
        }
        
        // Normalize by the maximum possible volume for a tetrahedron with edge length max_dist
        // For a regular tetrahedron with edge length L, volume = L³/(6√2)
        let ideal_volume = max_dist.powi(3) / (6.0 * 2.0_f64.sqrt());
        
        if ideal_volume > 1e-10 {
            max_volume / ideal_volume
        } else {
            0.0
        }
    }
    
    /// Calculate quality scores for individual anchors based on their contribution to geometry
    fn calculate_anchor_quality_scores(&self, positions: &[Vector3<f64>], receiver_pos: &Vector3<f64>) -> Vec<f64> {
        let n = positions.len();
        if n < 3 {
            return vec![1.0; n]; // Not enough anchors for meaningful scoring
        }
        
        let mut scores = Vec::with_capacity(n);
        
        // Calculate baseline GDOP with all anchors
        let baseline_dop = self.calculate_gdop(positions, receiver_pos);
        
        // Calculate how much each anchor contributes to the geometry
        for i in 0..n {
            // Create a subset without this anchor
            let subset: Vec<Vector3<f64>> = positions.iter()
                .enumerate()
                .filter(|&(j, _)| j != i)
                .map(|(_, pos)| *pos)
                .collect();
            
            // Calculate GDOP without this anchor
            let subset_dop = self.calculate_gdop(&subset, receiver_pos);
            
            // Score is based on how much GDOP degrades when this anchor is removed
            // Higher score means more important anchor (removing it causes more degradation)
            let degradation_ratio = if baseline_dop.gdop < 1000.0 && subset_dop.gdop < 1000.0 {
                subset_dop.gdop / baseline_dop.gdop
            } else {
                1.0 // Default for numerical stability
            };
            
            // Normalize score between 0 and 1, with higher being better
            let score = (degradation_ratio - 1.0).min(10.0) / 10.0;
            scores.push(score);
        }
        
        scores
    }
    
    /// Select optimal subset of anchors based on geometry quality
    fn select_optimal_anchors(&self, positions: &[Vector3<f64>], scores: &[f64]) -> Vec<usize> {
        let n = positions.len();
        
        // If we have 4 or fewer anchors, use all of them
        if n <= 4 {
            return (0..n).collect();
        }
        
        // Create indices sorted by score (highest first)
        let mut indexed_scores: Vec<(usize, f64)> = scores.iter()
            .enumerate()
            .map(|(i, &score)| (i, score))
            .collect();
        
        indexed_scores.sort_by(|a, b| {
            b.1.partial_cmp(&a.1).unwrap_or(Ordering::Equal)
        });
        
        // Always include the top 4 anchors
        let mut selected: Vec<usize> = indexed_scores.iter()
            .take(4)
            .map(|&(idx, _)| idx)
            .collect();
        
        // Add additional anchors if they significantly improve geometry
        let base_indices: Vec<usize> = selected.clone();
        let base_positions: Vec<Vector3<f64>> = base_indices.iter()
            .map(|&idx| positions[idx])
            .collect();
        
        // Calculate centroid as approximate receiver position
        let mut centroid = Vector3::zeros();
        for &idx in &base_indices {
            centroid += positions[idx];
        }
        centroid /= base_indices.len() as f64;
        
        let base_dop = self.calculate_gdop(&base_positions, &centroid);
        
        // Consider adding each remaining anchor
        for &(idx, _) in indexed_scores.iter().skip(4) {
            let mut extended_positions = base_positions.clone();
            extended_positions.push(positions[idx]);
            
            let extended_dop = self.calculate_gdop(&extended_positions, &centroid);
            
            // Add anchor if it improves GDOP by at least 15%
            if extended_dop.gdop < base_dop.gdop * 0.85 {
                selected.push(idx);
            }
        }
        
        selected
    }
    
    /// Trilateration with automatic anchor selection based on GDOP
    pub fn gdop_optimized_trilateration(
        &self,
        anchors: &[Anchor],
        receiver_time_ms: u64,
    ) -> Result<(Position, Vector3<f64>, DilutionOfPrecision), String> {
        if anchors.len() < 3 {
            return Err("At least 3 anchors required for GDOP-optimized trilateration".to_string());
        }
        
        // Convert to local coordinates
        let reference_pos = &anchors[0].position;
        let mut positions = Vec::new();
        let mut ranges = Vec::new();
        let mut anchor_indices = Vec::new();
        
        for (i, anchor) in anchors.iter().enumerate() {
            let local = self.geodetic_to_local(&anchor.position, reference_pos);
            positions.push(local);
            
            let dt_ms = receiver_time_ms as i64 - anchor.timestamp as i64;
            if dt_ms < 0 {
                return Err(format!("Receiver time earlier than anchor time for anchor {}", anchor.id));
            }
            let dt_sec = dt_ms as f64 / 1000.0;
            let range = SPEED_OF_SOUND_WATER * dt_sec;
            ranges.push(range);
            
            anchor_indices.push(i);
        }
        
        // Get initial position estimate
        let initial_estimate = if positions.len() >= 3 {
            match self.linear_least_squares(&positions, &ranges) {
                Ok(pos) => pos,
                Err(_) => {
                    // Fallback to centroid if linear least squares fails
                    let mut centroid = Vector3::zeros();
                    for pos in &positions {
                        centroid += pos;
                    }
                    centroid /= positions.len() as f64
                }
            }
        } else {
            return Err("Insufficient anchors for initial position estimate".to_string());
        };
        
        // Assess anchor geometry and get recommended subset
        let assessment = self.assess_anchor_geometry(&positions, Some(&initial_estimate));
        
        // If geometry is already excellent, use all anchors
        if assessment.dop.quality == GdopQuality::Excellent || assessment.dop.quality == GdopQuality::Good {
            // Use weighted least squares with all anchors
            let weights = vec![1.0; positions.len()];
            let (geodetic_pos, local_pos, _) = self.weighted_least_squares_trilateration(anchors, receiver_time_ms, &weights)?;
            return Ok((geodetic_pos, local_pos, assessment.dop));
        }
        
        // Otherwise, use the recommended subset
        if assessment.recommended_anchors.len() >= 3 {
            // Create subset of anchors
            let subset_anchors: Vec<Anchor> = assessment.recommended_anchors.iter()
                .map(|&idx| anchors[idx].clone())
                .collect();
            
            // Use uniform weights for the selected anchors
            let weights = vec![1.0; subset_anchors.len()];
            let (geodetic_pos, local_pos, _) = self.weighted_least_squares_trilateration(&subset_anchors, receiver_time_ms, &weights)?;
            
            // Recalculate DOP with final position
            let subset_positions: Vec<Vector3<f64>> = assessment.recommended_anchors.iter()
                .map(|&idx| positions[idx])
                .collect();
            
            let final_dop = self.calculate_gdop(&subset_positions, &local_pos);
            
            return Ok((geodetic_pos, local_pos, final_dop));
        } else {
            // Fallback to using all anchors if subset is too small
            let weights = vec![1.0; positions.len()];
            let (geodetic_pos, local_pos, _) = self.weighted_least_squares_trilateration(anchors, receiver_time_ms, &weights)?;
            return Ok((geodetic_pos, local_pos, assessment.dop));
        }
    }
    
    /// Calculate position uncertainty based on GDOP and range measurement uncertainty
    pub fn calculate_position_uncertainty(
        &self,
        dop: &DilutionOfPrecision,
        range_uncertainty_m: f64,
    ) -> Vector3<f64> {
        // Convert DOP to position uncertainty by multiplying with range uncertainty
        Vector3::new(
            dop.hdop * range_uncertainty_m,  // East uncertainty
            dop.hdop * range_uncertainty_m,  // North uncertainty
            dop.vdop * range_uncertainty_m,  // Vertical uncertainty
        )
    }
    
    /// Select optimal algorithm based on anchor geometry
    pub fn select_optimal_algorithm(
        &self,
        dop: &DilutionOfPrecision,
        anchor_count: usize,
    ) -> TrilaterationAlgorithm {
        match (dop.quality, anchor_count) {
            // Excellent geometry - use standard least squares
            (GdopQuality::Excellent, _) => TrilaterationAlgorithm::LinearLeastSquares,
            
            // Good geometry with 4+ anchors - use weighted least squares
            (GdopQuality::Good, n) if n >= 4 => TrilaterationAlgorithm::WeightedLeastSquares,
            
            // Moderate geometry with 4+ anchors - use robust estimation
            (GdopQuality::Moderate, n) if n >= 4 => TrilaterationAlgorithm::RobustEstimation,
            
            // Fair geometry with 4+ anchors - use MLE with Gaussian noise model
            (GdopQuality::Fair, n) if n >= 4 => TrilaterationAlgorithm::MaximumLikelihood(NoiseModel::Gaussian),
            
            // Poor geometry with 4+ anchors - use Levenberg-Marquardt with regularization
            (GdopQuality::Poor, n) if n >= 4 => TrilaterationAlgorithm::LevenbergMarquardt,
            
            // Default for 3 anchors - use weighted least squares
            (_, 3) => TrilaterationAlgorithm::WeightedLeastSquares,
            
            // Default fallback
            _ => TrilaterationAlgorithm::LinearLeastSquares,
        }
    }
}

/// Available trilateration algorithms for adaptive selection
#[derive(Debug, Clone, Copy)]
pub enum TrilaterationAlgorithm {
    /// Standard linear least squares (fastest, least robust)
    LinearLeastSquares,
    /// Weighted least squares with measurement reliability weighting
    WeightedLeastSquares,
    /// Maximum likelihood estimation with specified noise model
    MaximumLikelihood(NoiseModel),
    /// Levenberg-Marquardt optimization (slower but more robust)
    LevenbergMarquardt,
    /// Robust estimation with outlier detection
    RobustEstimation,
}


#[cfg(test)]
mod gdop_tests {
    use super::*;
    use crate::Position;
    
    #[test]
    fn test_gdop_calculation() {
        let trilateration = AdvancedTrilateration::new();
        
        // Create a good geometry with anchors in tetrahedron formation
        let positions = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(100.0, 0.0, 0.0),
            Vector3::new(0.0, 100.0, 0.0),
            Vector3::new(0.0, 0.0, 100.0),
        ];
        
        let receiver_pos = Vector3::new(50.0, 50.0, 50.0);
        let dop = trilateration.calculate_gdop(&positions, &receiver_pos);
        
        // Good geometry should have reasonable GDOP
        assert!(dop.gdop < 10.0, "GDOP should be reasonable for good geometry");
        assert!(dop.hdop < dop.vdop, "HDOP should be better than VDOP for this configuration");
        assert_eq!(dop.quality, GdopQuality::from_gdop(dop.gdop));
    }
    
    #[test]
    fn test_poor_geometry_detection() {
        let trilateration = AdvancedTrilateration::new();
        
        // Create a poor geometry with nearly collinear anchors
        let positions = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(100.0, 1.0, 0.0),
            Vector3::new(200.0, 2.0, 0.0),
            Vector3::new(300.0, 3.0, 0.0),
        ];
        
        let receiver_pos = Vector3::new(50.0, 50.0, 50.0);
        let dop = trilateration.calculate_gdop(&positions, &receiver_pos);
        
        // Poor geometry should have high GDOP
        assert!(dop.gdop > 10.0, "GDOP should be high for poor geometry");
        assert!(dop.quality == GdopQuality::Fair || dop.quality == GdopQuality::Poor, 
                "Quality should indicate poor geometry");
    }
    
    #[test]
    fn test_anchor_geometry_assessment() {
        let trilateration = AdvancedTrilateration::new();
        
        // Create a mixed geometry with some good and some poor anchor placements
        let positions = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(100.0, 0.0, 0.0),
            Vector3::new(0.0, 100.0, 0.0),
            Vector3::new(0.0, 0.0, 100.0),
            Vector3::new(1.0, 1.0, 0.0),  // Nearly collinear with first anchor
        ];
        
        let assessment = trilateration.assess_anchor_geometry(&positions, None);
        
        // Check that assessment contains valid data
        assert!(assessment.dop.gdop > 0.0, "GDOP should be positive");
        assert!(assessment.condition_number > 0.0, "Condition number should be positive");
        assert!(assessment.normalized_volume >= 0.0 && assessment.normalized_volume <= 1.0,
                "Normalized volume should be between 0 and 1");
        
        // Check that we have scores for all anchors
        assert_eq!(assessment.anchor_scores.len(), positions.len());
        
        // Check that recommended anchors is not empty
        assert!(!assessment.recommended_anchors.is_empty());
        
        // The nearly collinear anchor should have a lower score
        let min_score = assessment.anchor_scores.iter().copied().fold(f64::INFINITY, f64::min);
        assert!(min_score < 0.5, "Poorest anchor should have a low score");
    }
    
    #[test]
    fn test_gdop_optimized_trilateration() {
        let mut trilateration = AdvancedTrilateration::new();
        
        // Create anchors with timestamps
        let base_time = 1000000;
        let anchors = vec![
            Anchor {
                id: "1".to_string(),
                timestamp: base_time,
                position: Position { lat: 0.0, lon: 0.0, depth: 0.0 },
            },
            Anchor {
                id: "2".to_string(),
                timestamp: base_time + 67,  // 100m away at 1500 m/s
                position: Position { lat: 0.0009, lon: 0.0, depth: 0.0 }, // ~100m north
            },
            Anchor {
                id: "3".to_string(),
                timestamp: base_time + 67,
                position: Position { lat: 0.0, lon: 0.0009, depth: 0.0 }, // ~100m east
            },
            Anchor {
                id: "4".to_string(),
                timestamp: base_time + 67,
                position: Position { lat: 0.0, lon: 0.0, depth: 100.0 }, // 100m deep
            },
            Anchor {
                id: "5".to_string(),
                timestamp: base_time + 67,
                position: Position { lat: 0.00001, lon: 0.00001, depth: 0.0 }, // Very close to first anchor
            },
        ];
        
        // Receiver is at the center, 100m from each good anchor
        let receiver_time = base_time + 167; // 100m / 1500 m/s * 1000 ms/s + base_time
        
        let result = trilateration.gdop_optimized_trilateration(&anchors, receiver_time);
        
        assert!(result.is_ok(), "GDOP-optimized trilateration should succeed");
        
        if let Ok((pos, _, dop)) = result {
            // Position should be close to origin
            assert!(pos.lat.abs() < 0.0001, "Latitude should be close to origin");
            assert!(pos.lon.abs() < 0.0001, "Longitude should be close to origin");
            assert!(pos.depth.abs() < 10.0, "Depth should be close to origin");
            
            // DOP should be reasonable
            assert!(dop.gdop < 20.0, "GDOP should be reasonable");
        }
    }
    
    #[test]
    fn test_position_uncertainty_calculation() {
        let trilateration = AdvancedTrilateration::new();
        
        // Create a DOP with known values
        let dop = DilutionOfPrecision {
            gdop: 5.0,
            pdop: 4.0,
            hdop: 2.0,
            vdop: 3.0,
            tdop: 3.0,
            quality: GdopQuality::Good,
        };
        
        // Range uncertainty of 1 meter
        let range_uncertainty = 1.0;
        
        let uncertainty = trilateration.calculate_position_uncertainty(&dop, range_uncertainty);
        
        // Uncertainty should match DOP values scaled by range uncertainty
        assert_eq!(uncertainty.x, dop.hdop * range_uncertainty);
        assert_eq!(uncertainty.y, dop.hdop * range_uncertainty);
        assert_eq!(uncertainty.z, dop.vdop * range_uncertainty);
    }
    
    #[test]
    fn test_algorithm_selection() {
        let trilateration = AdvancedTrilateration::new();
        
        // Test different quality levels and anchor counts
        let excellent_dop = DilutionOfPrecision {
            gdop: 1.5,
            pdop: 1.2,
            hdop: 0.8,
            vdop: 0.9,
            tdop: 0.7,
            quality: GdopQuality::Excellent,
        };
        
        let poor_dop = DilutionOfPrecision {
            gdop: 25.0,
            pdop: 20.0,
            hdop: 15.0,
            vdop: 18.0,
            tdop: 10.0,
            quality: GdopQuality::Poor,
        };
        
        // Excellent geometry should use linear least squares regardless of anchor count
        let algo1 = trilateration.select_optimal_algorithm(&excellent_dop, 3);
        let algo2 = trilateration.select_optimal_algorithm(&excellent_dop, 5);
        
        match algo1 {
            TrilaterationAlgorithm::LinearLeastSquares => {},
            _ => panic!("Should select LinearLeastSquares for excellent geometry with 3 anchors"),
        }
        
        match algo2 {
            TrilaterationAlgorithm::LinearLeastSquares => {},
            _ => panic!("Should select LinearLeastSquares for excellent geometry with 5 anchors"),
        }
        
        // Poor geometry with many anchors should use Levenberg-Marquardt
        let algo3 = trilateration.select_optimal_algorithm(&poor_dop, 5);
        
        match algo3 {
            TrilaterationAlgorithm::LevenbergMarquardt => {},
            _ => panic!("Should select LevenbergMarquardt for poor geometry with 5 anchors"),
        }
    }
}