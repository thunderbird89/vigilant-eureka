use nalgebra::{Matrix4, Vector3, Vector4, DMatrix, DVector, Matrix3};
use crate::core::{Anchor, Position};
use crate::algorithms::trilateration::{GdopQuality, DilutionOfPrecision, AnchorGeometryAssessment, AdvancedTrilateration, TrilaterationAlgorithm};
use std::cmp::Ordering;
use std::f64::consts::PI;

/// GDOP optimization module for improving positioning accuracy through anchor selection
pub struct GdopOptimizer {
    /// Advanced trilateration engine for calculations
    trilateration: AdvancedTrilateration,
    /// Maximum acceptable GDOP value
    max_acceptable_gdop: f64,
    /// Minimum number of anchors required
    min_anchors: usize,
    /// Maximum number of anchors to use (for computational efficiency)
    max_anchors: usize,
    /// Geometry quality thresholds for GDOP values
    gdop_thresholds: [f64; 5],
    /// Weighting factor for signal quality vs geometry (0-1, higher means more weight on geometry)
    geometry_weight: f64,
}

impl Default for GdopOptimizer {
    fn default() -> Self {
        Self {
            trilateration: AdvancedTrilateration::default(),
            max_acceptable_gdop: 10.0,
            min_anchors: 4,
            max_anchors: 8,
            gdop_thresholds: [2.0, 5.0, 10.0, 20.0, 50.0],
            geometry_weight: 0.7,
        }
    }
}

/// Anchor selection result with quality metrics
#[derive(Debug, Clone)]
pub struct AnchorSelectionResult {
    /// Indices of selected anchors
    pub selected_indices: Vec<usize>,
    /// GDOP value for the selected configuration
    pub gdop: f64,
    /// Quality assessment
    pub quality: GdopQuality,
    /// Estimated position uncertainty (meters)
    pub position_uncertainty: f64,
}

/// Anchor scoring information for selection algorithms
#[derive(Debug, Clone)]
struct AnchorScore {
    /// Index in original anchor array
    index: usize,
    /// Contribution to geometry improvement
    geometry_score: f64,
    /// Signal quality (0-255)
    signal_quality: u8,
    /// Combined score (weighted sum)
    combined_score: f64,
}

impl GdopOptimizer {
    /// Create a new GDOP optimizer with default parameters
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a new GDOP optimizer with custom parameters
    pub fn with_parameters(max_gdop: f64, min_anchors: usize, max_anchors: usize) -> Self {
        Self {
            trilateration: AdvancedTrilateration::default(),
            max_acceptable_gdop: max_gdop,
            min_anchors: min_anchors.max(3), // At least 3 anchors required
            max_anchors: max_anchors,
            gdop_thresholds: [2.0, 5.0, 10.0, 20.0, 50.0],
            geometry_weight: 0.7,
        }
    }

    /// Select optimal anchor subset based on GDOP minimization
    /// 
    /// This function evaluates different anchor combinations to find the subset
    /// that provides the best geometric configuration for accurate positioning.
    pub fn select_optimal_anchors(
        &self,
        anchors: &[Anchor],
        receiver_time_ms: u64,
        range_uncertainty_m: f64,
    ) -> Result<AnchorSelectionResult, String> {
        if anchors.len() < self.min_anchors {
            return Err(format!(
                "Insufficient anchors: {} available, {} required",
                anchors.len(), self.min_anchors
            ));
        }

        // Convert anchors to local coordinates for geometry calculations
        let reference_pos = &anchors[0].position;
        let mut positions = Vec::with_capacity(anchors.len());
        let mut signal_qualities = Vec::with_capacity(anchors.len());

        for anchor in anchors {
            let local = self.trilateration.geodetic_to_local(&anchor.position, reference_pos);
            positions.push(local);
            
            // Extract signal quality from anchor ID (assuming last 2 digits encode quality)
            // In a real system, this would come from signal strength measurements
            let quality = anchor.id.parse::<u16>().map_or(128, |id| (id % 256) as u8);
            signal_qualities.push(quality);
        }

        // If we have exactly the minimum number of anchors, use all of them
        if anchors.len() == self.min_anchors {
            let indices: Vec<usize> = (0..anchors.len()).collect();
            let assessment = self.assess_anchor_subset(&positions, &indices);
            
            return Ok(AnchorSelectionResult {
                selected_indices: indices,
                gdop: assessment.dop.gdop,
                quality: assessment.dop.quality,
                position_uncertainty: assessment.dop.gdop * range_uncertainty_m,
            });
        }

        // For larger sets, use greedy selection algorithm
        let selected = self.greedy_anchor_selection(&positions, &signal_qualities);
        
        // Calculate GDOP for selected anchors
        let assessment = self.assess_anchor_subset(&positions, &selected);
        
        Ok(AnchorSelectionResult {
            selected_indices: selected,
            gdop: assessment.dop.gdop,
            quality: assessment.dop.quality,
            position_uncertainty: assessment.dop.gdop * range_uncertainty_m,
        })
    }

    /// Greedy algorithm for anchor selection
    /// 
    /// Starts with the best 4 anchors and adds more if they improve GDOP
    fn greedy_anchor_selection(
        &self,
        positions: &[Vector3<f64>],
        signal_qualities: &[u8],
    ) -> Vec<usize> {
        // Start by scoring individual anchors
        let mut anchor_scores = self.score_anchors(positions, signal_qualities);
        
        // Sort by combined score (descending)
        anchor_scores.sort_by(|a, b| {
            b.combined_score.partial_cmp(&a.combined_score)
                .unwrap_or(Ordering::Equal)
        });
        
        // Start with the best 4 anchors
        let mut selected: Vec<usize> = anchor_scores.iter()
            .take(4)
            .map(|score| score.index)
            .collect();
        
        // Calculate initial GDOP
        let initial_assessment = self.assess_anchor_subset(positions, &selected);
        let mut best_gdop = initial_assessment.dop.gdop;
        
        // Try adding more anchors if they improve GDOP
        for score in anchor_scores.iter().skip(4) {
            // Skip if we've reached max anchors
            if selected.len() >= self.max_anchors {
                break;
            }
            
            // Try adding this anchor
            let mut test_set = selected.clone();
            test_set.push(score.index);
            
            // Calculate new GDOP
            let test_assessment = self.assess_anchor_subset(positions, &test_set);
            
            // Keep this anchor if it improves GDOP
            if test_assessment.dop.gdop < best_gdop {
                selected.push(score.index);
                best_gdop = test_assessment.dop.gdop;
            }
        }
        
        selected
    }

    /// Score anchors based on geometry contribution and signal quality
    fn score_anchors(
        &self,
        positions: &[Vector3<f64>],
        signal_qualities: &[u8],
    ) -> Vec<AnchorScore> {
        let mut scores = Vec::with_capacity(positions.len());
        
        // Calculate centroid for reference
        let centroid = positions.iter()
            .fold(Vector3::zeros(), |acc, pos| acc + pos) 
            / positions.len() as f64;
        
        // Score each anchor
        for (i, pos) in positions.iter().enumerate() {
            // Geometry score based on distance from centroid and other anchors
            let dist_from_centroid = (pos - centroid).norm();
            
            // Calculate average distance to other anchors
            let mut total_dist = 0.0;
            let mut count = 0;
            for (j, other_pos) in positions.iter().enumerate() {
                if i != j {
                    total_dist += (pos - other_pos).norm();
                    count += 1;
                }
            }
            let avg_dist_to_others = if count > 0 { total_dist / count as f64 } else { 0.0 };
            
            // Combine distance metrics for geometry score
            // We want anchors that are far from centroid and well-separated from others
            let geometry_score = dist_from_centroid * 0.5 + avg_dist_to_others * 0.5;
            
            // Signal quality normalized to 0-1
            let signal_quality = signal_qualities[i];
            let normalized_quality = signal_quality as f64 / 255.0;
            
            // Combined score (70% geometry, 30% signal quality)
            let combined_score = geometry_score * 0.7 + normalized_quality * 0.3;
            
            scores.push(AnchorScore {
                index: i,
                geometry_score,
                signal_quality,
                combined_score,
            });
        }
        
        scores
    }

    /// Assess a subset of anchors for GDOP calculation
    fn assess_anchor_subset(
        &self,
        all_positions: &[Vector3<f64>],
        indices: &[usize],
    ) -> AnchorGeometryAssessment {
        // Extract positions for the selected indices
        let subset_positions: Vec<Vector3<f64>> = indices.iter()
            .map(|&idx| all_positions[idx])
            .collect();
        
        // Calculate centroid as approximate receiver position
        let centroid = subset_positions.iter()
            .fold(Vector3::zeros(), |acc, pos| acc + pos) 
            / subset_positions.len() as f64;
        
        // Perform geometry assessment
        self.trilateration.assess_anchor_geometry(&subset_positions, Some(&centroid))
    }

    /// Calculate position uncertainty based on GDOP and range measurement uncertainty
    pub fn calculate_position_uncertainty(
        &self,
        gdop: f64,
        range_uncertainty_m: f64,
    ) -> f64 {
        gdop * range_uncertainty_m
    }

    /// Determine the best trilateration algorithm based on GDOP value
    pub fn select_algorithm_by_gdop(&self, gdop: f64) -> TrilaterationAlgorithm {
        use crate::algorithms::trilateration::TrilaterationAlgorithm;
        
        if gdop < 2.0 {
            // Excellent geometry - standard least squares is sufficient
            TrilaterationAlgorithm::LinearLeastSquares
        } else if gdop < 5.0 {
            // Good geometry - weighted least squares for better results
            TrilaterationAlgorithm::WeightedLeastSquares
        } else if gdop < 10.0 {
            // Moderate geometry - Levenberg-Marquardt for robustness
            TrilaterationAlgorithm::LevenbergMarquardt
        } else {
            // Poor geometry - maximum likelihood with robust estimation
            TrilaterationAlgorithm::RobustMle
        }
    }
}

/// Extension trait for AdvancedTrilateration to add GDOP-based functionality
pub trait GdopOptimizedTrilateration {
    /// Perform trilateration with automatic algorithm selection based on GDOP
    fn trilaterate_with_gdop_optimization(
        &mut self,
        anchors: &[Anchor],
        receiver_time_ms: u64,
        range_uncertainty_m: f64,
    ) -> Result<(Position, Vector3<f64>, DilutionOfPrecision), String>;
    
    /// Select best subset of anchors based on GDOP and perform trilateration
    fn trilaterate_with_anchor_selection(
        &mut self,
        anchors: &[Anchor],
        receiver_time_ms: u64,
        range_uncertainty_m: f64,
    ) -> Result<(Position, Vector3<f64>, AnchorSelectionResult), String>;
}

impl GdopOptimizedTrilateration for AdvancedTrilateration {
    fn trilaterate_with_gdop_optimization(
        &mut self,
        anchors: &[Anchor],
        receiver_time_ms: u64,
        range_uncertainty_m: f64,
    ) -> Result<(Position, Vector3<f64>, DilutionOfPrecision), String> {
        if anchors.len() < 3 {
            return Err("At least 3 anchors required for trilateration".to_string());
        }

        // Convert to local coordinates
        let reference_pos = &anchors[0].position;
        let mut positions = Vec::with_capacity(anchors.len());
        let mut ranges = Vec::with_capacity(anchors.len());

        for anchor in anchors {
            let local = self.geodetic_to_local(&anchor.position, reference_pos);
            positions.push(local);

            let dt_ms = receiver_time_ms as i64 - anchor.timestamp as i64;
            if dt_ms < 0 {
                return Err(format!("Receiver time earlier than anchor time for anchor {}", anchor.id));
            }
            let dt_sec = dt_ms as f64 / 1000.0;
            let range = crate::SPEED_OF_SOUND_WATER * dt_sec;
            ranges.push(range);
        }

        // Calculate GDOP for current configuration
        // Use centroid as initial position estimate
        let centroid = positions.iter()
            .fold(Vector3::zeros(), |acc, pos| acc + pos) 
            / positions.len() as f64;
        
        let dop = self.calculate_gdop(&positions, &centroid);
        
        // Select algorithm based on GDOP
        let optimizer = GdopOptimizer::new();
        let algorithm = optimizer.select_algorithm_by_gdop(dop.gdop);
        
        // Perform trilateration with selected algorithm
        let (position, local_pos, _) = match algorithm {
            TrilaterationAlgorithm::LinearLeastSquares => {
                // Simple least squares
                let weights = vec![1.0; anchors.len()];
                self.weighted_least_squares_trilateration(anchors, receiver_time_ms, &weights)?
            },
            TrilaterationAlgorithm::WeightedLeastSquares => {
                // Weight by inverse distance
                let weights: Vec<f64> = ranges.iter()
                    .map(|&r| 1.0 / r.max(0.1))
                    .collect();
                self.weighted_least_squares_trilateration(anchors, receiver_time_ms, &weights)?
            },
            TrilaterationAlgorithm::LevenbergMarquardt => {
                // Levenberg-Marquardt with centroid as initial guess
                self.levenberg_marquardt_trilateration(anchors, receiver_time_ms, Some(centroid))?
            },
            TrilaterationAlgorithm::RobustMle => {
                // Robust MLE with outlier detection
                if anchors.len() >= 4 {
                    let (pos, local, _) = self.robust_trilateration(anchors, receiver_time_ms)?;
                    (pos, local, 0.0)
                } else {
                    // Fallback for 3 anchors
                    let weights = vec![1.0; anchors.len()];
                    self.weighted_least_squares_trilateration(anchors, receiver_time_ms, &weights)?
                }
            },
            TrilaterationAlgorithm::MaximumLikelihood(_) => {
                // Use MLE with default noise model
                let weights = vec![1.0; anchors.len()];
                self.weighted_least_squares_trilateration(anchors, receiver_time_ms, &weights)?
            },
            TrilaterationAlgorithm::RobustEstimation => {
                // Use robust estimation
                if anchors.len() >= 4 {
                    let (pos, local, _) = self.robust_trilateration(anchors, receiver_time_ms)?;
                    (pos, local, 0.0)
                } else {
                    let weights = vec![1.0; anchors.len()];
                    self.weighted_least_squares_trilateration(anchors, receiver_time_ms, &weights)?
                }
            },
        };
        
        // Recalculate GDOP with final position for more accuracy
        let final_dop = self.calculate_gdop(&positions, &local_pos);
        
        Ok((position, local_pos, final_dop))
    }
    
    fn trilaterate_with_anchor_selection(
        &mut self,
        anchors: &[Anchor],
        receiver_time_ms: u64,
        range_uncertainty_m: f64,
    ) -> Result<(Position, Vector3<f64>, AnchorSelectionResult), String> {
        if anchors.len() < 3 {
            return Err("At least 3 anchors required for trilateration".to_string());
        }

        // Create optimizer and select optimal anchors
        let optimizer = GdopOptimizer::new();
        let selection = optimizer.select_optimal_anchors(anchors, receiver_time_ms, range_uncertainty_m)?;
        
        // Create subset of selected anchors
        let selected_anchors: Vec<Anchor> = selection.selected_indices.iter()
            .map(|&idx| anchors[idx].clone())
            .collect();
        
        // Perform trilateration with selected anchors and optimized algorithm
        let algorithm = optimizer.select_algorithm_by_gdop(selection.gdop);
        
        // Set algorithm and perform trilateration
        let (position, local_pos, _) = match algorithm {
            TrilaterationAlgorithm::LinearLeastSquares => {
                // Simple least squares
                let weights = vec![1.0; selected_anchors.len()];
                self.weighted_least_squares_trilateration(&selected_anchors, receiver_time_ms, &weights)?
            },
            TrilaterationAlgorithm::WeightedLeastSquares => {
                // Create weights based on anchor quality
                let weights: Vec<f64> = selection.selected_indices.iter()
                    .map(|&idx| {
                        // Extract quality from anchor ID (in real system, use signal quality)
                        let quality = anchors[idx].id.parse::<u16>().map_or(128, |id| (id % 256) as u8);
                        quality as f64 / 128.0 // Normalize around 1.0
                    })
                    .collect();
                self.weighted_least_squares_trilateration(&selected_anchors, receiver_time_ms, &weights)?
            },
            TrilaterationAlgorithm::LevenbergMarquardt => {
                // Levenberg-Marquardt with no initial guess
                self.levenberg_marquardt_trilateration(&selected_anchors, receiver_time_ms, None)?
            },
            TrilaterationAlgorithm::RobustMle => {
                // Robust MLE with outlier detection
                if selected_anchors.len() >= 4 {
                    let (pos, local, _) = self.robust_trilateration(&selected_anchors, receiver_time_ms)?;
                    (pos, local, 0.0)
                } else {
                    // Fallback for 3 anchors
                    let weights = vec![1.0; selected_anchors.len()];
                    self.weighted_least_squares_trilateration(&selected_anchors, receiver_time_ms, &weights)?
                }
            },
            TrilaterationAlgorithm::MaximumLikelihood(_) => {
                // Use MLE with default noise model
                let weights = vec![1.0; selected_anchors.len()];
                self.weighted_least_squares_trilateration(&selected_anchors, receiver_time_ms, &weights)?
            },
            TrilaterationAlgorithm::RobustEstimation => {
                // Use robust estimation
                if selected_anchors.len() >= 4 {
                    let (pos, local, _) = self.robust_trilateration(&selected_anchors, receiver_time_ms)?;
                    (pos, local, 0.0)
                } else {
                    let weights = vec![1.0; selected_anchors.len()];
                    self.weighted_least_squares_trilateration(&selected_anchors, receiver_time_ms, &weights)?
                }
            },
        };
        
        Ok((position, local_pos, selection))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Position;
    
    fn create_test_anchors() -> Vec<Anchor> {
        vec![
            Anchor {
                id: "001".to_string(),
                timestamp: 1000,
                position: Position { lat: 32.12345, lon: 45.47675, depth: 0.0 },
            },
            Anchor {
                id: "002".to_string(),
                timestamp: 1002,
                position: Position { lat: 32.12365, lon: 45.47695, depth: 0.0 },
            },
            Anchor {
                id: "003".to_string(),
                timestamp: 1002,
                position: Position { lat: 32.12365, lon: 45.47655, depth: 0.0 },
            },
            Anchor {
                id: "004".to_string(),
                timestamp: 1000,
                position: Position { lat: 32.12385, lon: 45.47675, depth: 10.0 },
            },
            Anchor {
                id: "005".to_string(),
                timestamp: 1003,
                position: Position { lat: 32.12355, lon: 45.47685, depth: 5.0 },
            },
            Anchor {
                id: "006".to_string(),
                timestamp: 1004,
                position: Position { lat: 32.12375, lon: 45.47665, depth: 7.0 },
            },
        ]
    }
    
    #[test]
    fn test_gdop_calculation() {
        let optimizer = GdopOptimizer::new();
        let trilateration = AdvancedTrilateration::new();
        let anchors = create_test_anchors();
        
        // Convert to local coordinates
        let reference_pos = &anchors[0].position;
        let positions: Vec<Vector3<f64>> = anchors.iter()
            .map(|a| trilateration.geodetic_to_local(&a.position, reference_pos))
            .collect();
        
        // Calculate centroid
        let centroid = positions.iter()
            .fold(Vector3::zeros(), |acc, pos| acc + pos) 
            / positions.len() as f64;
        
        // Calculate GDOP
        let dop = trilateration.calculate_gdop(&positions, &centroid);
        
        // GDOP should be a positive number
        assert!(dop.gdop > 0.0);
        
        // Quality should be valid
        assert!(matches!(
            dop.quality,
            GdopQuality::Excellent | GdopQuality::Good | GdopQuality::Moderate | 
            GdopQuality::Fair | GdopQuality::Poor
        ));
    }
    
    #[test]
    fn test_anchor_selection() {
        let optimizer = GdopOptimizer::new();
        let anchors = create_test_anchors();
        
        // Select optimal anchors
        let result = optimizer.select_optimal_anchors(&anchors, 1010, 1.0);
        
        // Should succeed
        assert!(result.is_ok());
        
        let selection = result.unwrap();
        
        // Should select at least 4 anchors
        assert!(selection.selected_indices.len() >= 4);
        
        // All indices should be valid
        for &idx in &selection.selected_indices {
            assert!(idx < anchors.len());
        }
        
        // GDOP should be positive
        assert!(selection.gdop > 0.0);
        
        // Position uncertainty should be positive
        assert!(selection.position_uncertainty > 0.0);
    }
    
    #[test]
    fn test_gdop_optimized_trilateration() {
        let mut trilateration = AdvancedTrilateration::new();
        let anchors = create_test_anchors();
        
        // Perform GDOP-optimized trilateration
        let result = trilateration.trilaterate_with_gdop_optimization(&anchors, 1010, 1.0);
        
        // Should succeed
        assert!(result.is_ok());
        
        let (position, _, dop) = result.unwrap();
        
        // Position should be valid
        assert!(position.lat > 0.0);
        assert!(position.lon > 0.0);
        
        // GDOP should be positive
        assert!(dop.gdop > 0.0);
    }
    
    #[test]
    fn test_trilateration_with_anchor_selection() {
        let mut trilateration = AdvancedTrilateration::new();
        let anchors = create_test_anchors();
        
        // Perform trilateration with anchor selection
        let result = trilateration.trilaterate_with_anchor_selection(&anchors, 1010, 1.0);
        
        // Should succeed
        assert!(result.is_ok());
        
        let (position, _, selection) = result.unwrap();
        
        // Position should be valid
        assert!(position.lat > 0.0);
        assert!(position.lon > 0.0);
        
        // Should select at least 4 anchors
        assert!(selection.selected_indices.len() >= 4);
    }
}