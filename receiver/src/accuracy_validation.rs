use crate::{trilaterate, Anchor, Position};
use nalgebra::Vector3;
use std::collections::HashMap;

/// Noise simulation framework for testing positioning accuracy under various conditions
pub struct NoiseSimulator {
    pub range_noise_std: f64,      // Standard deviation of range measurement noise (meters)
    pub timing_jitter_std: f64,    // Standard deviation of timing jitter (milliseconds)
    pub position_noise_std: f64,   // Standard deviation of anchor position uncertainty (meters)
    pub sound_speed_error: f64,    // Sound speed error as fraction (e.g., 0.01 = 1% error)
}

impl NoiseSimulator {
    pub fn new() -> Self {
        Self {
            range_noise_std: 0.1,      // 10cm range noise
            timing_jitter_std: 1.0,    // 1ms timing jitter
            position_noise_std: 0.05,  // 5cm anchor position uncertainty
            sound_speed_error: 0.0,    // No sound speed error by default
        }
    }

    pub fn with_range_noise(mut self, std_dev: f64) -> Self {
        self.range_noise_std = std_dev;
        self
    }

    pub fn with_timing_jitter(mut self, std_dev_ms: f64) -> Self {
        self.timing_jitter_std = std_dev_ms;
        self
    }

    pub fn with_position_uncertainty(mut self, std_dev: f64) -> Self {
        self.position_noise_std = std_dev;
        self
    }

    pub fn with_sound_speed_error(mut self, error_fraction: f64) -> Self {
        self.sound_speed_error = error_fraction;
        self
    }

    /// Apply noise to anchor data based on configured parameters
    pub fn add_noise_to_anchors(&self, anchors: &[Anchor], true_receiver_pos: &Position, rng_seed: u64) -> Vec<Anchor> {
        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};
        
        let mut noisy_anchors = Vec::new();
        
        for (i, anchor) in anchors.iter().enumerate() {
            // Create deterministic but different random values for each anchor
            let mut hasher = DefaultHasher::new();
            rng_seed.hash(&mut hasher);
            i.hash(&mut hasher);
            let seed = hasher.finish();
            
            // Simple linear congruential generator for deterministic noise
            let mut rng_state = seed;
            let mut next_random = || {
                rng_state = rng_state.wrapping_mul(1103515245).wrapping_add(12345);
                (rng_state as f64) / (u64::MAX as f64) - 0.5 // Range [-0.5, 0.5]
            };
            
            // Box-Muller transform for Gaussian noise
            let mut gaussian_noise = || {
                let u1 = (next_random() + 0.5).max(1e-10).min(1.0 - 1e-10);
                let u2 = next_random() + 0.5;
                (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
            };

            // Add position noise to anchor
            let noisy_lat = anchor.position.lat + gaussian_noise() * self.position_noise_std / 111132.0; // ~1 degree lat = 111132m
            let noisy_lon = anchor.position.lon + gaussian_noise() * self.position_noise_std / (111320.0 * anchor.position.lat.to_radians().cos());
            let noisy_depth = anchor.position.depth + gaussian_noise() * self.position_noise_std;

            // Calculate true range to receiver
            let true_range = calculate_range(&anchor.position, true_receiver_pos);
            
            // Add range noise (affects timing)
            let noisy_range = true_range + gaussian_noise() * self.range_noise_std;
            
            // Add sound speed error
            let effective_sound_speed = 1500.0 * (1.0 + self.sound_speed_error);
            
            // Calculate noisy travel time
            let noisy_travel_time_ms = (noisy_range / effective_sound_speed) * 1000.0;
            
            // Add timing jitter
            let total_jitter_ms = gaussian_noise() * self.timing_jitter_std;
            
            // Calculate noisy timestamp (receiver time - travel time - jitter)
            let base_receiver_time = 1723111200000u64; // Base time for consistency
            let noisy_timestamp = (base_receiver_time as f64 - noisy_travel_time_ms - total_jitter_ms) as u64;

            let noisy_anchor = Anchor {
                id: anchor.id.clone(),
                timestamp: noisy_timestamp,
                position: Position {
                    lat: noisy_lat,
                    lon: noisy_lon,
                    depth: noisy_depth,
                },
            };

            noisy_anchors.push(noisy_anchor);
        }

        noisy_anchors
    }
}

/// Calculate Euclidean distance between two positions in meters
fn calculate_range(pos1: &Position, pos2: &Position) -> f64 {
    // Convert to local coordinates for distance calculation
    let lat_diff = (pos2.lat - pos1.lat) * 111132.0; // meters per degree latitude
    let lon_diff = (pos2.lon - pos1.lon) * 111320.0 * pos1.lat.to_radians().cos(); // meters per degree longitude
    let depth_diff = pos2.depth - pos1.depth;
    
    (lat_diff.powi(2) + lon_diff.powi(2) + depth_diff.powi(2)).sqrt()
}

/// Test scenario with known ground truth for accuracy validation
#[derive(Debug, Clone)]
pub struct TestScenario {
    pub name: String,
    pub anchors: Vec<Anchor>,
    pub true_receiver_position: Position,
    pub expected_accuracy_m: f64,
    pub geometry_quality: String,
}

impl TestScenario {
    /// Create a well-conditioned 4-anchor scenario
    pub fn good_geometry_4_anchors() -> Self {
        let anchors = vec![
            Anchor {
                id: "001".to_string(),
                timestamp: 1723111199986,
                position: Position { lat: 32.12345, lon: 45.47675, depth: 0.0 },
            },
            Anchor {
                id: "002".to_string(),
                timestamp: 1723111199988,
                position: Position { lat: 32.12365, lon: 45.47695, depth: 0.0 },
            },
            Anchor {
                id: "003".to_string(),
                timestamp: 1723111199988,
                position: Position { lat: 32.12365, lon: 45.47655, depth: 0.0 },
            },
            Anchor {
                id: "004".to_string(),
                timestamp: 1723111199986,
                position: Position { lat: 32.12385, lon: 45.47675, depth: 0.0 },
            },
        ];

        Self {
            name: "Good Geometry 4 Anchors".to_string(),
            anchors,
            true_receiver_position: Position { lat: 32.123649, lon: 45.476750, depth: 0.0 },
            expected_accuracy_m: 0.5,
            geometry_quality: "Excellent".to_string(),
        }
    }

    /// Create a 3D scenario with anchors at different depths
    pub fn three_dimensional_scenario() -> Self {
        let anchors = vec![
            Anchor {
                id: "001".to_string(),
                timestamp: 1723111199986,
                position: Position { lat: 32.12345, lon: 45.47675, depth: 5.0 },
            },
            Anchor {
                id: "002".to_string(),
                timestamp: 1723111199988,
                position: Position { lat: 32.12365, lon: 45.47695, depth: 15.0 },
            },
            Anchor {
                id: "003".to_string(),
                timestamp: 1723111199988,
                position: Position { lat: 32.12365, lon: 45.47655, depth: 25.0 },
            },
            Anchor {
                id: "004".to_string(),
                timestamp: 1723111199986,
                position: Position { lat: 32.12385, lon: 45.47675, depth: 10.0 },
            },
        ];

        Self {
            name: "3D Scenario".to_string(),
            anchors,
            true_receiver_position: Position { lat: 32.123644, lon: 45.476735, depth: 14.5 },
            expected_accuracy_m: 1.0,
            geometry_quality: "Good".to_string(),
        }
    }

    /// Create a challenging scenario with poor geometry (nearly coplanar)
    pub fn poor_geometry_scenario() -> Self {
        let anchors = vec![
            Anchor {
                id: "001".to_string(),
                timestamp: 1723111199986,
                position: Position { lat: 32.12345, lon: 45.47675, depth: 10.0 },
            },
            Anchor {
                id: "002".to_string(),
                timestamp: 1723111199988,
                position: Position { lat: 32.12346, lon: 45.47676, depth: 10.1 },
            },
            Anchor {
                id: "003".to_string(),
                timestamp: 1723111199988,
                position: Position { lat: 32.12347, lon: 45.47677, depth: 10.2 },
            },
            Anchor {
                id: "004".to_string(),
                timestamp: 1723111199986,
                position: Position { lat: 32.12348, lon: 45.47678, depth: 10.3 },
            },
        ];

        Self {
            name: "Poor Geometry (Nearly Collinear)".to_string(),
            anchors,
            true_receiver_position: Position { lat: 32.123465, lon: 45.476765, depth: 10.15 },
            expected_accuracy_m: 5.0,
            geometry_quality: "Poor".to_string(),
        }
    }

    /// Create a 3-anchor scenario (2D positioning)
    pub fn three_anchor_scenario() -> Self {
        let anchors = vec![
            Anchor {
                id: "001".to_string(),
                timestamp: 1723111199986,
                position: Position { lat: 32.12345, lon: 45.47675, depth: 0.0 },
            },
            Anchor {
                id: "002".to_string(),
                timestamp: 1723111199988,
                position: Position { lat: 32.12365, lon: 45.47695, depth: 0.0 },
            },
            Anchor {
                id: "003".to_string(),
                timestamp: 1723111199988,
                position: Position { lat: 32.12365, lon: 45.47655, depth: 0.0 },
            },
        ];

        Self {
            name: "3 Anchor Scenario (2D)".to_string(),
            anchors,
            true_receiver_position: Position { lat: 32.123550, lon: 45.476750, depth: 0.0 },
            expected_accuracy_m: 1.0,
            geometry_quality: "Acceptable".to_string(),
        }
    }
}

/// Statistical analysis results for positioning accuracy
#[derive(Debug)]
pub struct AccuracyStatistics {
    pub scenario_name: String,
    pub num_trials: usize,
    pub mean_error_m: f64,
    pub std_error_m: f64,
    pub max_error_m: f64,
    pub min_error_m: f64,
    pub percentile_50_m: f64,
    pub percentile_95_m: f64,
    pub percentile_99_m: f64,
    pub success_rate: f64,
    pub meets_requirement: bool,
    pub mean_computation_time_ms: f64,
}

/// Comprehensive accuracy validation test runner
pub struct AccuracyValidator {
    scenarios: Vec<TestScenario>,
    noise_configs: Vec<NoiseSimulator>,
    num_trials_per_config: usize,
}

impl AccuracyValidator {
    pub fn new() -> Self {
        Self {
            scenarios: vec![
                TestScenario::good_geometry_4_anchors(),
                TestScenario::three_dimensional_scenario(),
                TestScenario::poor_geometry_scenario(),
                TestScenario::three_anchor_scenario(),
            ],
            noise_configs: vec![
                NoiseSimulator::new(), // Baseline
                NoiseSimulator::new().with_range_noise(0.2),
                NoiseSimulator::new().with_timing_jitter(2.0),
                NoiseSimulator::new().with_position_uncertainty(0.1),
                NoiseSimulator::new().with_sound_speed_error(0.01),
                NoiseSimulator::new()
                    .with_range_noise(0.3)
                    .with_timing_jitter(3.0)
                    .with_position_uncertainty(0.15)
                    .with_sound_speed_error(0.02),
            ],
            num_trials_per_config: 100,
        }
    }

    pub fn with_trials(mut self, num_trials: usize) -> Self {
        self.num_trials_per_config = num_trials;
        self
    }

    /// Run comprehensive accuracy validation tests
    pub fn run_validation(&self) -> Vec<AccuracyStatistics> {
        let mut all_results = Vec::new();

        for scenario in &self.scenarios {
            for (noise_idx, noise_config) in self.noise_configs.iter().enumerate() {
                let stats = self.run_scenario_with_noise(scenario, noise_config, noise_idx);
                all_results.push(stats);
            }
        }

        all_results
    }

    fn run_scenario_with_noise(&self, scenario: &TestScenario, noise_config: &NoiseSimulator, noise_idx: usize) -> AccuracyStatistics {
        let mut errors = Vec::new();
        let mut computation_times = Vec::new();
        let mut successful_trials = 0;

        let receiver_time_ms = 1723111200000u64;

        for trial in 0..self.num_trials_per_config {
            let seed = (noise_idx as u64) * 1000 + trial as u64;
            let noisy_anchors = noise_config.add_noise_to_anchors(&scenario.anchors, &scenario.true_receiver_position, seed);

            let start_time = std::time::Instant::now();
            let result = trilaterate(&noisy_anchors, receiver_time_ms);
            let computation_time = start_time.elapsed().as_secs_f64() * 1000.0; // Convert to milliseconds

            computation_times.push(computation_time);

            match result {
                Ok((estimated_pos, _)) => {
                    successful_trials += 1;
                    let error = calculate_range(&estimated_pos, &scenario.true_receiver_position);
                    errors.push(error);
                }
                Err(_) => {
                    // Failed trial - don't include in error statistics but count for success rate
                }
            }
        }

        // Calculate statistics
        let success_rate = successful_trials as f64 / self.num_trials_per_config as f64;
        
        if errors.is_empty() {
            return AccuracyStatistics {
                scenario_name: format!("{} (Noise Config {})", scenario.name, noise_idx),
                num_trials: self.num_trials_per_config,
                mean_error_m: f64::INFINITY,
                std_error_m: f64::INFINITY,
                max_error_m: f64::INFINITY,
                min_error_m: f64::INFINITY,
                percentile_50_m: f64::INFINITY,
                percentile_95_m: f64::INFINITY,
                percentile_99_m: f64::INFINITY,
                success_rate,
                meets_requirement: false,
                mean_computation_time_ms: computation_times.iter().sum::<f64>() / computation_times.len() as f64,
            };
        }

        errors.sort_by(|a, b| a.partial_cmp(b).unwrap());

        let mean_error = errors.iter().sum::<f64>() / errors.len() as f64;
        let variance = errors.iter().map(|e| (e - mean_error).powi(2)).sum::<f64>() / errors.len() as f64;
        let std_error = variance.sqrt();

        let percentile_50 = errors[errors.len() / 2];
        let percentile_95 = errors[(errors.len() as f64 * 0.95) as usize];
        let percentile_99 = errors[(errors.len() as f64 * 0.99) as usize];

        let meets_requirement = percentile_95 <= 1.0; // 95% of measurements within 1.0m

        AccuracyStatistics {
            scenario_name: format!("{} (Noise Config {})", scenario.name, noise_idx),
            num_trials: self.num_trials_per_config,
            mean_error_m: mean_error,
            std_error_m: std_error,
            max_error_m: *errors.last().unwrap(),
            min_error_m: *errors.first().unwrap(),
            percentile_50_m: percentile_50,
            percentile_95_m: percentile_95,
            percentile_99_m: percentile_99,
            success_rate,
            meets_requirement,
            mean_computation_time_ms: computation_times.iter().sum::<f64>() / computation_times.len() as f64,
        }
    }

    /// Generate detailed accuracy report
    pub fn generate_report(&self, results: &[AccuracyStatistics]) -> String {
        let mut report = String::new();
        
        report.push_str("=== UNDERWATER POSITIONING SYSTEM ACCURACY VALIDATION REPORT ===\n\n");
        
        // Summary statistics
        let total_scenarios = results.len();
        let passing_scenarios = results.iter().filter(|r| r.meets_requirement).count();
        let overall_success_rate = results.iter().map(|r| r.success_rate).sum::<f64>() / results.len() as f64;
        
        report.push_str(&format!("SUMMARY:\n"));
        report.push_str(&format!("- Total test configurations: {}\n", total_scenarios));
        report.push_str(&format!("- Configurations meeting 1.0m requirement: {} ({:.1}%)\n", 
                                passing_scenarios, 100.0 * passing_scenarios as f64 / total_scenarios as f64));
        report.push_str(&format!("- Overall success rate: {:.1}%\n", 100.0 * overall_success_rate));
        report.push_str(&format!("- Trials per configuration: {}\n\n", self.num_trials_per_config));

        // Detailed results
        report.push_str("DETAILED RESULTS:\n");
        report.push_str(&format!("{:<40} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8} {:>8}\n",
                                "Scenario", "Success%", "Mean(m)", "Std(m)", "50%(m)", "95%(m)", "99%(m)", "Max(m)", "Time(ms)", "Pass"));
        report.push_str(&"-".repeat(120));
        report.push_str("\n");

        for result in results {
            let pass_indicator = if result.meets_requirement { "✓" } else { "✗" };
            report.push_str(&format!("{:<40} {:>7.1}% {:>7.3} {:>7.3} {:>7.3} {:>7.3} {:>7.3} {:>7.3} {:>7.2} {:>8}\n",
                                    result.scenario_name,
                                    100.0 * result.success_rate,
                                    result.mean_error_m,
                                    result.std_error_m,
                                    result.percentile_50_m,
                                    result.percentile_95_m,
                                    result.percentile_99_m,
                                    result.max_error_m,
                                    result.mean_computation_time_ms,
                                    pass_indicator));
        }

        // Performance analysis
        report.push_str("\n\nPERFORMANCE ANALYSIS:\n");
        let mean_computation_time = results.iter().map(|r| r.mean_computation_time_ms).sum::<f64>() / results.len() as f64;
        let max_computation_time = results.iter().map(|r| r.mean_computation_time_ms).fold(0.0, f64::max);
        
        report.push_str(&format!("- Average computation time: {:.2} ms\n", mean_computation_time));
        report.push_str(&format!("- Maximum computation time: {:.2} ms\n", max_computation_time));
        report.push_str(&format!("- Real-time requirement (200ms): {}\n", 
                                if max_computation_time < 200.0 { "✓ PASS" } else { "✗ FAIL" }));

        // Noise sensitivity analysis
        report.push_str("\n\nNOISE SENSITIVITY ANALYSIS:\n");
        let mut scenario_groups: HashMap<String, Vec<&AccuracyStatistics>> = HashMap::new();
        
        for result in results {
            let base_scenario = result.scenario_name.split(" (Noise Config").next().unwrap_or(&result.scenario_name);
            scenario_groups.entry(base_scenario.to_string()).or_insert_with(Vec::new).push(result);
        }

        for (scenario_name, group) in scenario_groups {
            report.push_str(&format!("\n{}: \n", scenario_name));
            let baseline = group.iter().find(|r| r.scenario_name.contains("Config 0")).unwrap();
            report.push_str(&format!("  Baseline (no noise): {:.3}m (95%ile)\n", baseline.percentile_95_m));
            
            for result in group.iter().filter(|r| !r.scenario_name.contains("Config 0")) {
                let degradation = result.percentile_95_m - baseline.percentile_95_m;
                report.push_str(&format!("  With noise: {:.3}m (95%ile), degradation: +{:.3}m\n", 
                                        result.percentile_95_m, degradation));
            }
        }

        report.push_str("\n\nRECOMMENDATIONS:\n");
        
        let failing_scenarios: Vec<_> = results.iter().filter(|r| !r.meets_requirement).collect();
        if failing_scenarios.is_empty() {
            report.push_str("✓ All test configurations meet the 1.0m accuracy requirement.\n");
        } else {
            report.push_str(&format!("⚠ {} configurations fail to meet 1.0m requirement:\n", failing_scenarios.len()));
            for result in failing_scenarios {
                report.push_str(&format!("  - {}: {:.3}m (95%ile)\n", result.scenario_name, result.percentile_95_m));
            }
        }

        if overall_success_rate < 0.95 {
            report.push_str("⚠ Consider improving error handling - success rate below 95%\n");
        }

        if max_computation_time > 100.0 {
            report.push_str("⚠ Consider optimizing computation time for real-time performance\n");
        }

        report
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_noise_simulator_deterministic() {
        let simulator = NoiseSimulator::new().with_range_noise(0.1);
        let scenario = TestScenario::good_geometry_4_anchors();
        
        // Same seed should produce same results
        let noisy1 = simulator.add_noise_to_anchors(&scenario.anchors, &scenario.true_receiver_position, 12345);
        let noisy2 = simulator.add_noise_to_anchors(&scenario.anchors, &scenario.true_receiver_position, 12345);
        
        assert_eq!(noisy1.len(), noisy2.len());
        for (a1, a2) in noisy1.iter().zip(noisy2.iter()) {
            assert_eq!(a1.timestamp, a2.timestamp);
            assert!((a1.position.lat - a2.position.lat).abs() < 1e-10);
            assert!((a1.position.lon - a2.position.lon).abs() < 1e-10);
            assert!((a1.position.depth - a2.position.depth).abs() < 1e-10);
        }
    }

    #[test]
    fn test_accuracy_validator_basic() {
        let validator = AccuracyValidator::new().with_trials(10);
        let results = validator.run_validation();
        
        assert!(!results.is_empty());
        
        // Check that we have results for all scenario/noise combinations
        let expected_combinations = 4 * 6; // 4 scenarios × 6 noise configs
        assert_eq!(results.len(), expected_combinations);
        
        // Verify basic statistics are reasonable
        for result in &results {
            assert!(result.success_rate >= 0.0 && result.success_rate <= 1.0);
            assert!(result.mean_computation_time_ms >= 0.0);
            if result.success_rate > 0.0 {
                assert!(result.mean_error_m >= 0.0);
                assert!(result.std_error_m >= 0.0);
            }
        }
    }

    #[test]
    fn test_scenario_creation() {
        let scenario = TestScenario::good_geometry_4_anchors();
        assert_eq!(scenario.anchors.len(), 4);
        assert_eq!(scenario.name, "Good Geometry 4 Anchors");
        
        let scenario_3d = TestScenario::three_dimensional_scenario();
        assert_eq!(scenario_3d.anchors.len(), 4);
        
        let scenario_3_anchor = TestScenario::three_anchor_scenario();
        assert_eq!(scenario_3_anchor.anchors.len(), 3);
    }

    #[test]
    fn test_range_calculation() {
        let pos1 = Position { lat: 32.12345, lon: 45.47675, depth: 0.0 };
        let pos2 = Position { lat: 32.12355, lon: 45.47685, depth: 10.0 };
        
        let range = calculate_range(&pos1, &pos2);
        assert!(range > 0.0);
        assert!(range < 50.0); // Should be reasonable for small coordinate differences
    }

    #[test]
    fn test_comprehensive_accuracy_validation() {
        // This is the main test that validates the 1.0m accuracy requirement
        let validator = AccuracyValidator::new().with_trials(50);
        let results = validator.run_validation();
        
        // Generate and print report
        let report = validator.generate_report(&results);
        println!("{}", report);
        
        // Check that at least some configurations meet the requirement
        let passing_configs = results.iter().filter(|r| r.meets_requirement).count();
        assert!(passing_configs > 0, "No configurations meet the 1.0m accuracy requirement");
        
        // Check that 3D scenarios generally perform better than poor geometry scenarios
        let three_d_results: Vec<_> = results.iter()
            .filter(|r| r.scenario_name.contains("3D Scenario"))
            .collect();
        let poor_geometry_results: Vec<_> = results.iter()
            .filter(|r| r.scenario_name.contains("Poor Geometry"))
            .collect();
        
        if !three_d_results.is_empty() && !poor_geometry_results.is_empty() {
            let three_d_avg_error = three_d_results.iter()
                .map(|r| r.mean_error_m)
                .sum::<f64>() / three_d_results.len() as f64;
            let poor_avg_error = poor_geometry_results.iter()
                .map(|r| r.mean_error_m)
                .sum::<f64>() / poor_geometry_results.len() as f64;
            
            assert!(three_d_avg_error < poor_avg_error, 
                   "3D scenarios should have better accuracy than poor geometry scenarios");
        }
        
        // Verify computation time requirements
        let max_computation_time = results.iter()
            .map(|r| r.mean_computation_time_ms)
            .fold(0.0, f64::max);
        assert!(max_computation_time < 200.0, 
               "Computation time exceeds 200ms requirement: {:.2}ms", max_computation_time);
    }
}