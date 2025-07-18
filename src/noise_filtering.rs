use nalgebra::{Vector3, Matrix3};
use crate::{Anchor, Position, SPEED_OF_SOUND_WATER};
use std::collections::VecDeque;
use std::f64::consts::PI;

/// Noise filtering and signal processing module for underwater positioning
pub struct NoiseFilter {
    /// Window size for moving average filters
    pub window_size: usize,
    /// Threshold for multipath detection (ratio of expected to actual signal strength)
    pub multipath_threshold: f64,
    /// Threshold for outlier detection (in standard deviations)
    pub outlier_threshold: f64,
    /// Adaptive filter coefficient (0-1, higher means more weight on new measurements)
    pub alpha: f64,
    /// History of recent measurements for temporal filtering
    measurement_history: VecDeque<MeasurementSample>,
    /// Systematic error estimates for each anchor
    systematic_errors: std::collections::HashMap<String, f64>,
    /// Signal quality scores for each anchor (0-1)
    signal_quality_scores: std::collections::HashMap<String, f64>,
}

/// Measurement sample with quality metrics
#[derive(Debug, Clone)]
pub struct MeasurementSample {
    /// Anchor ID
    pub anchor_id: String,
    /// Measured range
    pub range: f64,
    /// Signal quality indicator (0-255)
    pub signal_quality: u8,
    /// Timestamp of measurement
    pub timestamp: u64,
    /// Estimated noise level
    pub noise_level: f64,
    /// Multipath indicator (0-1, higher means more likely multipath)
    pub multipath_indicator: f64,
}

/// Signal quality assessment result
#[derive(Debug, Clone)]
pub struct SignalQualityAssessment {
    /// Overall signal quality score (0-1)
    pub quality_score: f64,
    /// Multipath detection result
    pub multipath_detected: bool,
    /// Signal-to-noise ratio estimate (dB)
    pub snr_db: f64,
    /// Confidence in the measurement (0-1)
    pub confidence: f64,
}

/// Filtered measurement result
#[derive(Debug, Clone)]
pub struct FilteredMeasurement {
    /// Original measurement
    pub original: MeasurementSample,
    /// Filtered range value
    pub filtered_range: f64,
    /// Quality assessment
    pub quality: SignalQualityAssessment,
    /// Applied correction factors
    pub corrections: MeasurementCorrections,
}

/// Measurement correction factors
#[derive(Debug, Clone)]
pub struct MeasurementCorrections {
    /// Systematic error correction (meters)
    pub systematic_error: f64,
    /// Multipath correction (meters)
    pub multipath_correction: f64,
    /// Temporal jitter correction (meters)
    pub jitter_correction: f64,
    /// Total correction applied (meters)
    pub total_correction: f64,
}

impl Default for NoiseFilter {
    fn default() -> Self {
        Self {
            window_size: 5,
            multipath_threshold: 1.5,
            outlier_threshold: 2.5,
            alpha: 0.3,
            measurement_history: VecDeque::with_capacity(10),
            systematic_errors: std::collections::HashMap::new(),
            signal_quality_scores: std::collections::HashMap::new(),
        }
    }
}

impl NoiseFilter {
    /// Create a new noise filter with default parameters
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a noise filter with custom parameters
    pub fn with_parameters(
        window_size: usize,
        multipath_threshold: f64,
        outlier_threshold: f64,
        alpha: f64,
    ) -> Self {
        Self {
            window_size,
            multipath_threshold,
            outlier_threshold,
            alpha,
            measurement_history: VecDeque::with_capacity(window_size * 2),
            systematic_errors: std::collections::HashMap::new(),
            signal_quality_scores: std::collections::HashMap::new(),
        }
    }

    /// Process a batch of anchor measurements with adaptive noise filtering
    pub fn filter_measurements(
        &mut self,
        anchors: &[Anchor],
        receiver_time_ms: u64,
    ) -> Vec<FilteredMeasurement> {
        let mut filtered_measurements = Vec::with_capacity(anchors.len());
        
        for anchor in anchors {
            // Convert to range measurement
            let dt_ms = receiver_time_ms as i64 - anchor.timestamp as i64;
            if dt_ms < 0 {
                // Skip invalid measurements
                continue;
            }
            
            let dt_sec = dt_ms as f64 / 1000.0;
            let range = SPEED_OF_SOUND_WATER * dt_sec;
            
            // Extract signal quality from anchor ID (in a real system, this would come from the transceiver)
            // This is a placeholder implementation
            let signal_quality = anchor.id.parse::<u16>().map_or(128, |id| (id % 256) as u8);
            
            // Create measurement sample
            let sample = MeasurementSample {
                anchor_id: anchor.id.clone(),
                range,
                signal_quality,
                timestamp: anchor.timestamp,
                noise_level: self.estimate_noise_level(anchor),
                multipath_indicator: self.estimate_multipath_probability(anchor, range),
            };
            
            // Apply filtering
            let filtered = self.filter_measurement(sample);
            filtered_measurements.push(filtered);
        }
        
        // Update signal quality scores based on this batch
        self.update_signal_quality_scores(&filtered_measurements);
        
        filtered_measurements
    }

    /// Filter a single measurement with adaptive algorithms
    fn filter_measurement(&mut self, sample: MeasurementSample) -> FilteredMeasurement {
        // Get history for this anchor
        let anchor_history: Vec<&MeasurementSample> = self.measurement_history.iter()
            .filter(|m| m.anchor_id == sample.anchor_id)
            .collect();
        
        // Calculate corrections
        let systematic_error = self.systematic_errors
            .get(&sample.anchor_id)
            .copied()
            .unwrap_or(0.0);
        
        let multipath_correction = if sample.multipath_indicator > self.multipath_threshold {
            // Apply multipath mitigation
            self.calculate_multipath_correction(&sample, &anchor_history)
        } else {
            0.0
        };
        
        let jitter_correction = self.calculate_jitter_correction(&sample, &anchor_history);
        
        let total_correction = systematic_error + multipath_correction + jitter_correction;
        
        // Apply corrections to get filtered range
        let filtered_range = sample.range - total_correction;
        
        // Assess signal quality
        let quality = self.assess_signal_quality(&sample, filtered_range);
        
        // Store corrections
        let corrections = MeasurementCorrections {
            systematic_error,
            multipath_correction,
            jitter_correction,
            total_correction,
        };
        
        // Add to history
        self.add_to_history(sample.clone());
        
        FilteredMeasurement {
            original: sample,
            filtered_range,
            quality,
            corrections,
        }
    }

    /// Add measurement to history, maintaining window size
    fn add_to_history(&mut self, sample: MeasurementSample) {
        self.measurement_history.push_back(sample);
        
        // Trim history if needed
        while self.measurement_history.len() > self.window_size * 2 {
            self.measurement_history.pop_front();
        }
    }

    /// Estimate noise level for an anchor
    fn estimate_noise_level(&self, anchor: &Anchor) -> f64 {
        // In a real system, this would use signal strength indicators from the transceiver
        // For this implementation, we'll use a placeholder based on the anchor ID
        let base_noise = anchor.id.parse::<u16>().map_or(20.0, |id| (id % 10) as f64);
        
        // Add depth-dependent component (deeper = more noise)
        let depth_factor = 1.0 + (anchor.position.depth / 100.0);
        
        base_noise * depth_factor
    }

    /// Estimate probability of multipath for a measurement
    fn estimate_multipath_probability(&self, anchor: &Anchor, measured_range: f64) -> f64 {
        // In a real system, this would analyze signal characteristics
        // For this implementation, we'll use a placeholder based on range and depth
        
        // Deeper water tends to have more multipath
        let depth_factor = anchor.position.depth / 50.0;
        
        // Longer ranges tend to have more multipath
        let range_factor = measured_range / 1000.0;
        
        // Combine factors (clamped to 0-1)
        (depth_factor + range_factor).min(1.0)
    }

    /// Calculate correction for multipath effects
    fn calculate_multipath_correction(
        &self,
        sample: &MeasurementSample,
        history: &[&MeasurementSample],
    ) -> f64 {
        if history.is_empty() {
            return 0.0;
        }
        
        // Multipath typically causes positive bias in range measurements
        // We'll use a simplified model based on multipath indicator
        
        // Calculate expected range from history
        let expected_range = if history.len() >= 2 {
            // Use linear prediction from last two measurements
            let last = history[history.len() - 1];
            let prev = history[history.len() - 2];
            
            let dt = (sample.timestamp - last.timestamp) as f64 / 1000.0;
            let rate = (last.range - prev.range) / 
                ((last.timestamp - prev.timestamp) as f64 / 1000.0);
            
            last.range + rate * dt
        } else {
            // Just use last measurement
            history[0].range
        };
        
        // If measured range is significantly larger than expected, it may be multipath
        let range_diff = sample.range - expected_range;
        
        if range_diff > 0.0 && sample.multipath_indicator > 0.5 {
            // Apply correction proportional to multipath indicator and difference
            range_diff * (sample.multipath_indicator - 0.5) * 2.0
        } else {
            0.0
        }
    }

    /// Calculate correction for temporal jitter
    fn calculate_jitter_correction(
        &self,
        sample: &MeasurementSample,
        history: &[&MeasurementSample],
    ) -> f64 {
        if history.len() < self.window_size {
            return 0.0;
        }
        
        // Use exponential moving average to smooth jitter
        let recent_history = &history[history.len() - self.window_size..];
        
        // Calculate weighted average of recent measurements
        let mut weighted_sum = 0.0;
        let mut weight_sum = 0.0;
        
        for (i, hist) in recent_history.iter().enumerate() {
            // More recent measurements get higher weight
            let weight = (i + 1) as f64;
            weighted_sum += hist.range * weight;
            weight_sum += weight;
        }
        
        let smoothed_range = weighted_sum / weight_sum;
        
        // Calculate jitter correction
        let jitter = sample.range - smoothed_range;
        
        // Apply adaptive filtering based on signal quality
        let quality_factor = sample.signal_quality as f64 / 255.0;
        let correction_factor = self.alpha * (1.0 - quality_factor);
        
        jitter * correction_factor
    }

    /// Assess signal quality for a measurement
    fn assess_signal_quality(
        &self,
        sample: &MeasurementSample,
        filtered_range: f64,
    ) -> SignalQualityAssessment {
        // Calculate SNR in dB (placeholder implementation)
        let signal_level = sample.signal_quality as f64;
        let noise_level = sample.noise_level.max(1.0);
        let snr_linear = signal_level / noise_level;
        let snr_db = 20.0 * snr_linear.log10();
        
        // Detect multipath
        let multipath_detected = sample.multipath_indicator > self.multipath_threshold;
        
        // Calculate confidence based on SNR and multipath
        let snr_factor = (snr_db / 30.0).min(1.0).max(0.0); // Normalize to 0-1
        let multipath_factor = 1.0 - sample.multipath_indicator;
        
        // Combined quality score
        let quality_score = snr_factor * 0.7 + multipath_factor * 0.3;
        
        // Calculate confidence
        let confidence = quality_score * (1.0 - (sample.range - filtered_range).abs() / sample.range);
        
        SignalQualityAssessment {
            quality_score,
            multipath_detected,
            snr_db,
            confidence: confidence.max(0.0).min(1.0),
        }
    }

    /// Update signal quality scores for all anchors
    fn update_signal_quality_scores(&mut self, measurements: &[FilteredMeasurement]) {
        for measurement in measurements {
            let anchor_id = &measurement.original.anchor_id;
            let current_score = self.signal_quality_scores
                .get(anchor_id)
                .copied()
                .unwrap_or(0.5);
            
            // Update score with exponential smoothing
            let new_score = current_score * (1.0 - self.alpha) + 
                measurement.quality.quality_score * self.alpha;
            
            self.signal_quality_scores.insert(anchor_id.clone(), new_score);
            
            // Update systematic error estimate
            let current_error = self.systematic_errors
                .get(anchor_id)
                .copied()
                .unwrap_or(0.0);
            
            // Only update systematic error if confidence is high
            if measurement.quality.confidence > 0.7 {
                let error_diff = measurement.original.range - measurement.filtered_range;
                let new_error = current_error * 0.9 + error_diff * 0.1;
                self.systematic_errors.insert(anchor_id.clone(), new_error);
            }
        }
    }

    /// Get signal quality score for an anchor
    pub fn get_signal_quality(&self, anchor_id: &str) -> f64 {
        self.signal_quality_scores
            .get(anchor_id)
            .copied()
            .unwrap_or(0.5)
    }

    /// Get systematic error estimate for an anchor
    pub fn get_systematic_error(&self, anchor_id: &str) -> f64 {
        self.systematic_errors
            .get(anchor_id)
            .copied()
            .unwrap_or(0.0)
    }

    /// Reset filter state
    pub fn reset(&mut self) {
        self.measurement_history.clear();
        // Keep systematic errors and quality scores
    }

    /// Clear all state including learned parameters
    pub fn clear(&mut self) {
        self.measurement_history.clear();
        self.systematic_errors.clear();
        self.signal_quality_scores.clear();
    }
}

/// Adaptive signal processor for underwater positioning
pub struct AdaptiveSignalProcessor {
    /// Noise filter for measurement processing
    pub noise_filter: NoiseFilter,
    /// Temporal filter window size
    pub temporal_window: usize,
    /// Recent position estimates for temporal filtering
    position_history: VecDeque<(Vector3<f64>, f64)>, // (position, quality)
    /// Measurement weights for each anchor
    anchor_weights: std::collections::HashMap<String, f64>,
}

impl Default for AdaptiveSignalProcessor {
    fn default() -> Self {
        Self {
            noise_filter: NoiseFilter::default(),
            temporal_window: 3,
            position_history: VecDeque::with_capacity(5),
            anchor_weights: std::collections::HashMap::new(),
        }
    }
}

impl AdaptiveSignalProcessor {
    /// Create a new adaptive signal processor
    pub fn new() -> Self {
        Self::default()
    }

    /// Process anchors and apply filtering
    pub fn process_anchors(
        &mut self,
        anchors: &[Anchor],
        receiver_time_ms: u64,
    ) -> (Vec<Anchor>, Vec<f64>) {
        // Filter measurements
        let filtered = self.noise_filter.filter_measurements(anchors, receiver_time_ms);
        
        // Create filtered anchors
        let mut filtered_anchors = Vec::with_capacity(filtered.len());
        let mut weights = Vec::with_capacity(filtered.len());
        
        for measurement in &filtered {
            // Find original anchor
            if let Some(original) = anchors.iter().find(|a| a.id == measurement.original.anchor_id) {
                // Create modified anchor with adjusted timestamp
                let mut filtered_anchor = original.clone();
                
                // Adjust timestamp to reflect filtered range
                let filtered_time_sec = measurement.filtered_range / SPEED_OF_SOUND_WATER;
                let filtered_time_ms = (filtered_time_sec * 1000.0) as u64;
                filtered_anchor.timestamp = receiver_time_ms - filtered_time_ms;
                
                // Calculate weight based on quality
                let weight = measurement.quality.confidence;
                
                // Update anchor weights
                self.anchor_weights.insert(original.id.clone(), weight);
                
                filtered_anchors.push(filtered_anchor);
                weights.push(weight);
            }
        }
        
        (filtered_anchors, weights)
    }

    /// Apply temporal filtering to position estimate
    pub fn apply_temporal_filter(
        &mut self,
        position: Vector3<f64>,
        quality: f64,
    ) -> Vector3<f64> {
        // Add to history
        self.position_history.push_back((position, quality));
        
        // Trim history if needed
        while self.position_history.len() > self.temporal_window {
            self.position_history.pop_front();
        }
        
        // If not enough history, return as is
        if self.position_history.len() < 2 {
            return position;
        }
        
        // Apply weighted average based on quality
        let mut weighted_sum = Vector3::zeros();
        let mut weight_sum = 0.0;
        
        for (pos, qual) in &self.position_history {
            weighted_sum += pos * (*qual);
            weight_sum += qual;
        }
        
        if weight_sum > 0.0 {
            weighted_sum / weight_sum
        } else {
            position
        }
    }

    /// Detect and mitigate multipath effects
    pub fn detect_multipath(
        &self,
        anchors: &[Anchor],
        receiver_time_ms: u64,
    ) -> Vec<bool> {
        let mut multipath_flags = vec![false; anchors.len()];
        
        for (i, anchor) in anchors.iter().enumerate() {
            // Calculate range
            let dt_ms = receiver_time_ms as i64 - anchor.timestamp as i64;
            if dt_ms < 0 {
                continue;
            }
            
            let dt_sec = dt_ms as f64 / 1000.0;
            let range = SPEED_OF_SOUND_WATER * dt_sec;
            
            // Check for multipath indicators
            
            // 1. Excessive range compared to other anchors
            let median_range = self.calculate_median_range(anchors, receiver_time_ms);
            let range_ratio = range / median_range;
            
            // 2. Signal quality from history
            let signal_quality = self.noise_filter.get_signal_quality(&anchor.id);
            
            // 3. Depth-based probability (deeper = more multipath)
            let depth_factor = 1.0 + (anchor.position.depth / 50.0);
            
            // Combined multipath score
            let multipath_score = (range_ratio - 1.0) * depth_factor * (1.0 - signal_quality);
            
            // Flag as multipath if score exceeds threshold
            multipath_flags[i] = multipath_score > self.noise_filter.multipath_threshold;
        }
        
        multipath_flags
    }

    /// Calculate median range from anchors
    fn calculate_median_range(&self, anchors: &[Anchor], receiver_time_ms: u64) -> f64 {
        let mut ranges = Vec::with_capacity(anchors.len());
        
        for anchor in anchors {
            let dt_ms = receiver_time_ms as i64 - anchor.timestamp as i64;
            if dt_ms < 0 {
                continue;
            }
            
            let dt_sec = dt_ms as f64 / 1000.0;
            let range = SPEED_OF_SOUND_WATER * dt_sec;
            ranges.push(range);
        }
        
        if ranges.is_empty() {
            return 1000.0; // Default value if no valid ranges
        }
        
        // Calculate median
        ranges.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        
        if ranges.len() % 2 == 0 {
            let mid = ranges.len() / 2;
            (ranges[mid - 1] + ranges[mid]) / 2.0
        } else {
            ranges[ranges.len() / 2]
        }
    }

    /// Get weights for each anchor based on signal quality
    pub fn get_anchor_weights(&self, anchors: &[Anchor]) -> Vec<f64> {
        anchors.iter()
            .map(|a| self.anchor_weights.get(&a.id).copied().unwrap_or(1.0))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
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
        ]
    }
    
    #[test]
    fn test_noise_filter_initialization() {
        let filter = NoiseFilter::new();
        
        assert_eq!(filter.window_size, 5);
        assert!(filter.multipath_threshold > 0.0);
        assert!(filter.outlier_threshold > 0.0);
        assert!(filter.alpha > 0.0 && filter.alpha < 1.0);
    }
    
    #[test]
    fn test_measurement_filtering() {
        let mut filter = NoiseFilter::new();
        let anchors = create_test_anchors();
        
        let filtered = filter.filter_measurements(&anchors, 1010);
        
        // Should have filtered all anchors
        assert_eq!(filtered.len(), anchors.len());
        
        // Filtered ranges should be reasonable
        for measurement in &filtered {
            assert!(measurement.filtered_range > 0.0);
            assert!(measurement.quality.quality_score >= 0.0);
            assert!(measurement.quality.quality_score <= 1.0);
        }
    }
    
    #[test]
    fn test_adaptive_signal_processor() {
        let mut processor = AdaptiveSignalProcessor::new();
        let anchors = create_test_anchors();
        
        let (filtered_anchors, weights) = processor.process_anchors(&anchors, 1010);
        
        // Should have processed all anchors
        assert_eq!(filtered_anchors.len(), anchors.len());
        assert_eq!(weights.len(), anchors.len());
        
        // Weights should be between 0 and 1
        for weight in weights {
            assert!(weight >= 0.0);
            assert!(weight <= 1.0);
        }
    }
    
    #[test]
    fn test_multipath_detection() {
        let processor = AdaptiveSignalProcessor::new();
        let mut anchors = create_test_anchors();
        
        // Add an anchor with likely multipath (very long range)
        anchors.push(Anchor {
            id: "005".to_string(),
            timestamp: 900, // Very early timestamp = long range
            position: Position { lat: 32.12345, lon: 45.47675, depth: 20.0 }, // Deep = more multipath
        });
        
        let multipath_flags = processor.detect_multipath(&anchors, 1010);
        
        // Should have a flag for each anchor
        assert_eq!(multipath_flags.len(), anchors.len());
        
        // Last anchor should be flagged as multipath
        assert!(multipath_flags[anchors.len() - 1]);
    }
    
    #[test]
    fn test_temporal_filtering() {
        let mut processor = AdaptiveSignalProcessor::new();
        
        // Add some positions
        let pos1 = Vector3::new(0.0, 0.0, 0.0);
        let pos2 = Vector3::new(1.0, 0.0, 0.0);
        let pos3 = Vector3::new(2.0, 0.0, 0.0);
        
        let filtered1 = processor.apply_temporal_filter(pos1, 1.0);
        let filtered2 = processor.apply_temporal_filter(pos2, 1.0);
        let filtered3 = processor.apply_temporal_filter(pos3, 1.0);
        
        // First position should be unchanged
        assert!((filtered1 - pos1).norm() < 1e-10);
        
        // Later positions should be smoothed
        assert!(filtered3.x < pos3.x); // Should be pulled back by history
    }
}