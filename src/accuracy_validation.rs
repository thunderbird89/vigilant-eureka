use nalgebra::{Vector3, Matrix3};
use crate::{Anchor, Position, SPEED_OF_SOUND_WATER};
use std::collections::VecDeque;
use rand::prelude::*;
use rand_distr::Normal;

/// Accuracy validation and performance metrics for underwater positioning
pub struct AccuracyValidator {
    /// History of position errors for statistical analysis
    pub position_error_history: VecDeque<PositionError>,
    /// Maximum history size
    max_history_size: usize,
}

/// Position error metrics
#[derive(Debug, Clone)]
pub struct PositionError {
    /// True position
    pub true_position: Vector3<f64>,
    /// Estimated position
    pub estimated_position: Vector3<f64>,
    /// Error vector
    pub error_vector: Vector3<f64>,
    /// Error magnitude (meters)
    pub error_magnitude: f64,
    /// Horizontal error (meters)
    pub horizontal_error: f64,
    /// Vertical error (meters)
    pub vertical_error: f64,
    /// GDOP value at measurement time
    pub gdop: f64,
    /// Number of anchors used
    pub num_anchors: usize,
}

/// Accuracy statistics over multiple measurements
#[derive(Debug, Clone)]
pub struct AccuracyStatistics {
    /// Mean position error (meters)
    pub mean_error: f64,
    /// Standard deviation of position error (meters)
    pub std_dev_error: f64,
    /// 95% confidence error (meters)
    pub error_95_percentile: f64,
    /// Root Mean Square Error (RMSE) (meters)
    pub rmse: f64,
    /// Mean horizontal error (meters)
    pub mean_horizontal_error: f64,
    /// Mean vertical error (meters)
    pub mean_vertical_error: f64,
    /// Maximum observed error (meters)
    pub max_error: f64,
    /// Minimum observed error (meters)
    pub min_error: f64,
    /// Number of samples in statistics
    pub sample_count: usize,
}

/// Accuracy validation result with detailed metrics
#[derive(Debug, Clone)]
pub struct ValidationResult {
    /// Overall accuracy statistics
    pub statistics: AccuracyStatistics,
    /// Sub-meter accuracy achievement rate (0-1)
    pub submeter_accuracy_rate: f64,
    /// Factors affecting accuracy
    pub limiting_factors: Vec<String>,
    /// Recommendations for improvement
    pub recommendations: Vec<String>,
}

impl Default for AccuracyValidator {
    fn default() -> Self {
        Self {
            position_error_history: VecDeque::new(),
            max_history_size: 100,
        }
    }
}

impl AccuracyValidator {
    /// Create a new accuracy validator
    pub fn new() -> Self {
        Self::default()
    }

    /// Calculate position error metrics between true and estimated positions
    pub fn calculate_position_error(
        &self,
        true_position: &Vector3<f64>,
        estimated_position: &Vector3<f64>,
        gdop: f64,
        num_anchors: usize,
    ) -> PositionError {
        let error_vector = estimated_position - true_position;
        let error_magnitude = error_vector.norm();
        
        // Calculate horizontal and vertical components
        let horizontal_error = (error_vector.x.powi(2) + error_vector.y.powi(2)).sqrt();
        let vertical_error = error_vector.z.abs();
        
        PositionError {
            true_position: *true_position,
            estimated_position: *estimated_position,
            error_vector,
            error_magnitude,
            horizontal_error,
            vertical_error,
            gdop,
            num_anchors,
        }
    }

    /// Add position error to history
    pub fn add_error_to_history(&mut self, error: PositionError) {
        self.position_error_history.push_back(error);
        
        // Trim history if needed
        while self.position_error_history.len() > self.max_history_size {
            self.position_error_history.pop_front();
        }
    }

    /// Calculate accuracy statistics from error history
    pub fn calculate_statistics(&self) -> AccuracyStatistics {
        if self.position_error_history.is_empty() {
            return AccuracyStatistics {
                mean_error: 0.0,
                std_dev_error: 0.0,
                error_95_percentile: 0.0,
                rmse: 0.0,
                mean_horizontal_error: 0.0,
                mean_vertical_error: 0.0,
                max_error: 0.0,
                min_error: 0.0,
                sample_count: 0,
            };
        }
        
        let n = self.position_error_history.len();
        
        // Calculate mean error
        let mean_error: f64 = self.position_error_history.iter()
            .map(|e| e.error_magnitude)
            .sum::<f64>() / n as f64;
        
        // Calculate RMSE
        let rmse: f64 = (self.position_error_history.iter()
            .map(|e| e.error_magnitude.powi(2))
            .sum::<f64>() / n as f64)
            .sqrt();
        
        // Calculate standard deviation
        let variance: f64 = self.position_error_history.iter()
            .map(|e| (e.error_magnitude - mean_error).powi(2))
            .sum::<f64>() / n as f64;
        let std_dev_error = variance.sqrt();
        
        // Calculate mean horizontal and vertical errors
        let mean_horizontal_error: f64 = self.position_error_history.iter()
            .map(|e| e.horizontal_error)
            .sum::<f64>() / n as f64;
        
        let mean_vertical_error: f64 = self.position_error_history.iter()
            .map(|e| e.vertical_error)
            .sum::<f64>() / n as f64;
        
        // Find min and max errors
        let max_error = self.position_error_history.iter()
            .map(|e| e.error_magnitude)
            .fold(0.0, f64::max);
        
        let min_error = self.position_error_history.iter()
            .map(|e| e.error_magnitude)
            .fold(f64::INFINITY, f64::min);
        
        // Calculate 95th percentile error
        let mut sorted_errors: Vec<f64> = self.position_error_history.iter()
            .map(|e| e.error_magnitude)
            .collect();
        sorted_errors.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        
        let index_95 = (n as f64 * 0.95).ceil() as usize - 1;
        let error_95_percentile = sorted_errors.get(index_95.min(n - 1)).copied().unwrap_or(0.0);
        
        AccuracyStatistics {
            mean_error,
            std_dev_error,
            error_95_percentile,
            rmse,
            mean_horizontal_error,
            mean_vertical_error,
            max_error,
            min_error,
            sample_count: n,
        }
    }

    /// Simulate positioning with different anchor configurations
    pub fn simulate_accuracy(
        &mut self,
        true_position: &Vector3<f64>,
        anchor_positions: &[Vector3<f64>],
        range_std_dev: f64,
        num_simulations: usize,
    ) -> ValidationResult {
        // Clear previous history
        self.position_error_history.clear();
        
        let mut rng = rand::thread_rng();
        let normal = Normal::new(0.0, range_std_dev).unwrap();
        
        for _ in 0..num_simulations {
            // Generate noisy ranges
            let mut noisy_ranges = Vec::with_capacity(anchor_positions.len());
            
            for anchor_pos in anchor_positions {
                let true_range = (true_position - anchor_pos).norm();
                let noise = normal.sample(&mut rng);
                noisy_ranges.push(true_range + noise);
            }
            
            // Calculate GDOP (simplified)
            let gdop = self.calculate_simple_gdop(anchor_positions);
            
            // Perform trilateration with noisy ranges
            let estimated_position = match self.trilaterate_with_ranges(anchor_positions, &noisy_ranges) {
                Ok(pos) => pos,
                Err(_) => continue, // Skip failed trilaterations
            };
            
            // Calculate error
            let error = self.calculate_position_error(
                true_position,
                &estimated_position,
                gdop,
                anchor_positions.len(),
            );
            
            // Add to history
            self.add_error_to_history(error);
        }
        
        // Validate accuracy
        self.validate_accuracy()
    }

    /// Simple GDOP calculation
    fn calculate_simple_gdop(&self, anchor_positions: &[Vector3<f64>]) -> f64 {
        if anchor_positions.len() < 4 {
            return 10.0; // Default high GDOP for insufficient anchors
        }
        
        // Simple geometric assessment based on anchor spread
        let centroid = anchor_positions.iter().sum::<Vector3<f64>>() / anchor_positions.len() as f64;
        
        let mut total_distance = 0.0;
        for pos in anchor_positions {
            total_distance += (pos - centroid).norm();
        }
        
        let avg_distance = total_distance / anchor_positions.len() as f64;
        
        // Simple GDOP estimate (lower is better)
        if avg_distance > 100.0 {
            2.0 // Good geometry
        } else if avg_distance > 50.0 {
            4.0 // Moderate geometry
        } else {
            8.0 // Poor geometry
        }
    }

    /// Perform trilateration with given ranges
    fn trilaterate_with_ranges(
        &self,
        anchor_positions: &[Vector3<f64>],
        ranges: &[f64],
    ) -> Result<Vector3<f64>, String> {
        if anchor_positions.len() < 3 || ranges.len() != anchor_positions.len() {
            return Err("At least 3 anchors required with matching ranges".to_string());
        }
        
        // Use weighted least squares for simplicity
        let n = anchor_positions.len();
        let p1 = anchor_positions[0];
        
        // Handle 3-anchor case (2D solution)
        if n == 3 {
            let mut a_matrix = nalgebra::Matrix2::zeros();
            let mut b_vector = nalgebra::Vector2::zeros();

            for i in 1..3 {
                let pi = anchor_positions[i];
                let row = i - 1;
                
                a_matrix[(row, 0)] = 2.0 * (pi.x - p1.x);
                a_matrix[(row, 1)] = 2.0 * (pi.y - p1.y);
                
                b_vector[row] = ranges[0].powi(2) - ranges[i].powi(2)
                    + pi.x.powi(2) - p1.x.powi(2)
                    + pi.y.powi(2) - p1.y.powi(2)
                    + pi.z.powi(2) - p1.z.powi(2);
            }

            if let Some(inv_a) = a_matrix.try_inverse() {
                let xy_solution = inv_a * b_vector;
                
                // Estimate depth as average
                let estimated_depth: f64 = anchor_positions.iter()
                    .map(|p| p.z)
                    .sum::<f64>() / n as f64;
                
                return Ok(Vector3::new(xy_solution.x, xy_solution.y, estimated_depth));
            } else {
                return Err("Singular matrix in 2D least squares".to_string());
            }
        }

        // 3D case with 4+ anchors
        let mut a_matrix = nalgebra::DMatrix::zeros(n - 1, 3);
        let mut b_vector = nalgebra::DVector::zeros(n - 1);

        for i in 1..n {
            let pi = anchor_positions[i];
            let row = i - 1;
            
            a_matrix[(row, 0)] = 2.0 * (pi.x - p1.x);
            a_matrix[(row, 1)] = 2.0 * (pi.y - p1.y);
            a_matrix[(row, 2)] = 2.0 * (pi.z - p1.z);
            
            b_vector[row] = ranges[0].powi(2) - ranges[i].powi(2)
                + pi.x.powi(2) - p1.x.powi(2)
                + pi.y.powi(2) - p1.y.powi(2)
                + pi.z.powi(2) - p1.z.powi(2);
        }

        // Least squares solution
        let at_a = a_matrix.transpose() * &a_matrix;
        let at_b = a_matrix.transpose() * &b_vector;

        // Add regularization for numerical stability
        let mut regularized_matrix = at_a;
        for i in 0..3 {
            regularized_matrix[(i, i)] += 1e-6;
        }

        // Solve linear system
        match regularized_matrix.try_inverse() {
            Some(inv) => {
                let solution = inv * at_b;
                Ok(Vector3::new(solution[0], solution[1], solution[2]))
            },
            None => Err("Failed to solve least squares system".to_string()),
        }
    }

    /// Validate positioning accuracy with detailed metrics
    pub fn validate_accuracy(&self) -> ValidationResult {
        // Calculate overall statistics
        let overall_stats = self.calculate_statistics();
        
        // Calculate sub-meter accuracy rate
        let submeter_count = self.position_error_history.iter()
            .filter(|e| e.error_magnitude < 1.0)
            .count();
        
        let submeter_accuracy_rate = if self.position_error_history.is_empty() {
            0.0
        } else {
            submeter_count as f64 / self.position_error_history.len() as f64
        };
        
        // Identify limiting factors
        let mut limiting_factors = Vec::new();
        
        if overall_stats.mean_error > 0.5 {
            if overall_stats.mean_horizontal_error > overall_stats.mean_vertical_error {
                limiting_factors.push("Horizontal positioning accuracy".to_string());
            } else {
                limiting_factors.push("Vertical positioning accuracy".to_string());
            }
        }
        
        // Generate recommendations
        let mut recommendations = Vec::new();
        
        if submeter_accuracy_rate < 0.95 {
            recommendations.push("Improve anchor geometry for better GDOP".to_string());
            recommendations.push("Deploy anchors in tetrahedral configuration when possible".to_string());
            recommendations.push("Increase number of anchors for redundancy".to_string());
        }
        
        if overall_stats.mean_horizontal_error > 0.5 {
            recommendations.push("Improve horizontal anchor distribution".to_string());
        }
        
        if overall_stats.mean_vertical_error > 0.5 {
            recommendations.push("Add anchors at different depths for better vertical accuracy".to_string());
        }
        
        ValidationResult {
            statistics: overall_stats,
            submeter_accuracy_rate,
            limiting_factors,
            recommendations,
        }
    }

    /// Analyze factors affecting sub-meter accuracy achievement
    pub fn analyze_submeter_accuracy_factors(&self) -> Vec<(String, f64)> {
        if self.position_error_history.is_empty() {
            return Vec::new();
        }
        
        let mut factors = Vec::new();
        
        // Analyze GDOP impact
        let gdop_correlation = self.calculate_correlation(|e| e.gdop, |e| e.error_magnitude);
        factors.push(("GDOP".to_string(), gdop_correlation));
        
        // Analyze anchor count impact
        let anchor_correlation = self.calculate_correlation(|e| e.num_anchors as f64, |e| e.error_magnitude);
        factors.push(("Number of anchors".to_string(), -anchor_correlation)); // Negative because more anchors should reduce error
        
        // Analyze horizontal vs vertical error
        let horizontal_ratio = self.position_error_history.iter()
            .map(|e| e.horizontal_error / e.error_magnitude.max(0.001))
            .sum::<f64>() / self.position_error_history.len() as f64;
        
        factors.push(("Horizontal error contribution".to_string(), horizontal_ratio));
        
        // Sort by absolute impact (descending)
        factors.sort_by(|a, b| b.1.abs().partial_cmp(&a.1.abs()).unwrap_or(std::cmp::Ordering::Equal));
        
        factors
    }

    /// Calculate correlation between two metrics from error history
    fn calculate_correlation<F, G>(&self, metric1: F, metric2: G) -> f64
    where
        F: Fn(&PositionError) -> f64,
        G: Fn(&PositionError) -> f64,
    {
        if self.position_error_history.len() < 2 {
            return 0.0;
        }
        
        let n = self.position_error_history.len() as f64;
        
        // Calculate means
        let mean1: f64 = self.position_error_history.iter()
            .map(|e| metric1(e))
            .sum::<f64>() / n;
        
        let mean2: f64 = self.position_error_history.iter()
            .map(|e| metric2(e))
            .sum::<f64>() / n;
        
        // Calculate covariance and variances
        let mut covariance = 0.0;
        let mut variance1 = 0.0;
        let mut variance2 = 0.0;
        
        for error in &self.position_error_history {
            let val1 = metric1(error);
            let val2 = metric2(error);
            
            covariance += (val1 - mean1) * (val2 - mean2);
            variance1 += (val1 - mean1).powi(2);
            variance2 += (val2 - mean2).powi(2);
        }
        
        covariance /= n;
        variance1 /= n;
        variance2 /= n;
        
        // Calculate correlation coefficient
        if variance1 > 0.0 && variance2 > 0.0 {
            covariance / (variance1.sqrt() * variance2.sqrt())
        } else {
            0.0
        }
    }

    /// Optimize for sub-meter accuracy achievement
    pub fn optimize_for_submeter_accuracy(
        &self,
        anchor_positions: &[Vector3<f64>],
        _target_position: &Vector3<f64>,
    ) -> Vec<String> {
        let mut recommendations = Vec::new();
        
        // Calculate current GDOP
        let gdop = self.calculate_simple_gdop(anchor_positions);
        
        // Check if GDOP is good enough for sub-meter accuracy
        if gdop > 5.0 {
            recommendations.push(format!("Current GDOP ({:.2}) is too high for reliable sub-meter accuracy. Target GDOP < 2.0 for optimal results.", gdop));
            
            // Recommend anchor geometry improvements
            recommendations.push("Improve anchor geometry by deploying anchors in a tetrahedral configuration around the target area.".to_string());
            recommendations.push("Ensure anchors are well-distributed in all three dimensions.".to_string());
        }
        
        // Check number of anchors
        if anchor_positions.len() < 4 {
            recommendations.push(format!("Current anchor count ({}) is insufficient. Use at least 4 anchors for 3D positioning, preferably 5-6 for redundancy.", anchor_positions.len()));
        } else if anchor_positions.len() < 6 {
            recommendations.push("Consider adding more anchors for improved redundancy and accuracy.".to_string());
        }
        
        // Recommend environmental corrections
        recommendations.push("Apply sound speed corrections based on water temperature, salinity, and pressure for improved range accuracy.".to_string());
        
        // Recommend signal processing techniques
        recommendations.push("Use adaptive signal processing to mitigate multipath effects and noise.".to_string());
        
        // Recommend Kalman filtering for temporal smoothing
        recommendations.push("Apply Kalman filtering with appropriate process and measurement noise parameters for temporal smoothing.".to_string());
        
        // Recommend anchor selection based on GDOP
        if anchor_positions.len() > 4 {
            recommendations.push("Use GDOP-based anchor selection to choose optimal subset of anchors for each position calculation.".to_string());
        }
        
        recommendations
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_position_error_calculation() {
        let validator = AccuracyValidator::new();
        
        let true_pos = Vector3::new(0.0, 0.0, 0.0);
        let estimated_pos = Vector3::new(0.6, 0.8, 0.0);
        
        let error = validator.calculate_position_error(&true_pos, &estimated_pos, 2.0, 4);
        
        // Error magnitude should be 1.0 (Pythagorean theorem)
        assert!((error.error_magnitude - 1.0).abs() < 1e-10);
        
        // Horizontal error should be 1.0
        assert!((error.horizontal_error - 1.0).abs() < 1e-10);
        
        // Vertical error should be 0.0
        assert!(error.vertical_error.abs() < 1e-10);
    }
    
    #[test]
    fn test_statistics_calculation() {
        let mut validator = AccuracyValidator::new();
        
        // Add some test errors
        let true_pos = Vector3::new(0.0, 0.0, 0.0);
        
        validator.add_error_to_history(validator.calculate_position_error(
            &true_pos, &Vector3::new(0.5, 0.0, 0.0), 2.0, 4
        ));
        
        validator.add_error_to_history(validator.calculate_position_error(
            &true_pos, &Vector3::new(0.0, 1.0, 0.0), 3.0, 4
        ));
        
        validator.add_error_to_history(validator.calculate_position_error(
            &true_pos, &Vector3::new(0.0, 0.0, 1.5), 4.0, 4
        ));
        
        let stats = validator.calculate_statistics();
        
        // Mean error should be (0.5 + 1.0 + 1.5) / 3 = 1.0
        assert!((stats.mean_error - 1.0).abs() < 1e-10);
        
        // Sample count should be 3
        assert_eq!(stats.sample_count, 3);
    }
    
    #[test]
    fn test_submeter_accuracy_rate() {
        let mut validator = AccuracyValidator::new();
        
        // Add some test errors
        let true_pos = Vector3::new(0.0, 0.0, 0.0);
        
        // Two sub-meter errors
        validator.add_error_to_history(validator.calculate_position_error(
            &true_pos, &Vector3::new(0.5, 0.0, 0.0), 2.0, 4
        ));
        
        validator.add_error_to_history(validator.calculate_position_error(
            &true_pos, &Vector3::new(0.0, 0.8, 0.0), 3.0, 4
        ));
        
        // One above-meter error
        validator.add_error_to_history(validator.calculate_position_error(
            &true_pos, &Vector3::new(0.0, 0.0, 1.5), 4.0, 4
        ));
        
        let result = validator.validate_accuracy();
        
        // Sub-meter accuracy rate should be 2/3 = 0.6667
        assert!((result.submeter_accuracy_rate - 2.0/3.0).abs() < 1e-10);
    }
}