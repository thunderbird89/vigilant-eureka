//! Performance benchmark tests for timing validation
//! 
//! Tests cover:
//! - Algorithm execution time measurements
//! - Memory usage validation
//! - Real-time constraint verification
//! - Performance comparison between different algorithms

use crate::algorithms::trilateration::AdvancedTrilateration;
use crate::algorithms::gdop::GdopOptimizer;
use crate::algorithms::precision::HighPrecisionTransformer;
use crate::algorithms::embedded_trilateration::{EmbeddedTrilateration, config};
use crate::algorithms::embedded_coordinates::EmbeddedCoordinateManager;
use crate::core::{Anchor, Position};
use std::time::{Instant, Duration};
use std::collections::HashMap;

/// Performance test utilities
pub struct PerformanceTestUtils;

impl PerformanceTestUtils {
    /// Create performance test anchor sets of different sizes
    pub fn create_performance_test_anchors(count: usize) -> Vec<Anchor> {
        (0..count).map(|i| {
            let angle = i as f64 * 2.0 * std::f64::consts::PI / count as f64;
            let radius = 100.0 + (i as f64 * 10.0); // Varying distances
            let x = radius * angle.cos();
            let y = radius * angle.sin();
            let depth = (i as f64 * 5.0) % 50.0; // Varying depths
            
            Anchor {
                id: format!("PERF{:02}", i + 1),
                timestamp: 1000 + i as u64,
                position: Position {
                    lat: y / 111320.0, // Convert meters to degrees
                    lon: x / 111320.0,
                    depth,
                },
            }
        }).collect()
    }

    /// Measure execution time of a function
    pub fn measure_execution_time<F, R>(operation: F) -> (R, Duration)
    where
        F: FnOnce() -> R,
    {
        let start = Instant::now();
        let result = operation();
        let duration = start.elapsed();
        (result, duration)
    }

    /// Run performance benchmark with multiple iterations
    pub fn benchmark_operation<F, R>(
        operation: F,
        iterations: usize,
        description: &str,
    ) -> BenchmarkResult
    where
        F: Fn() -> R,
    {
        let mut durations = Vec::with_capacity(iterations);
        
        // Warm-up run
        let _ = operation();
        
        // Benchmark runs
        for _ in 0..iterations {
            let (_, duration) = Self::measure_execution_time(&operation);
            durations.push(duration);
        }
        
        // Calculate statistics
        let total_duration: Duration = durations.iter().sum();
        let avg_duration = total_duration / iterations as u32;
        
        let mut sorted_durations = durations.clone();
        sorted_durations.sort();
        
        let min_duration = sorted_durations[0];
        let max_duration = sorted_durations[iterations - 1];
        let median_duration = sorted_durations[iterations / 2];
        
        // Calculate 95th percentile
        let p95_index = (iterations as f64 * 0.95) as usize;
        let p95_duration = sorted_durations[p95_index.min(iterations - 1)];
        
        BenchmarkResult {
            description: description.to_string(),
            iterations,
            min_duration,
            max_duration,
            avg_duration,
            median_duration,
            p95_duration,
            total_duration,
        }
    }

    /// Validate real-time constraint
    pub fn validate_real_time_constraint(duration: Duration, max_duration_ms: u64) -> bool {
        duration.as_millis() <= max_duration_ms as u128
    }

    /// Estimate memory usage of a type
    pub fn estimate_memory_usage<T>() -> usize {
        std::mem::size_of::<T>()
    }

    /// Create stress test scenario with many anchors
    pub fn create_stress_test_anchors() -> Vec<Anchor> {
        Self::create_performance_test_anchors(50) // Large number of anchors
    }
}

/// Benchmark result structure
#[derive(Debug, Clone)]
pub struct BenchmarkResult {
    pub description: String,
    pub iterations: usize,
    pub min_duration: Duration,
    pub max_duration: Duration,
    pub avg_duration: Duration,
    pub median_duration: Duration,
    pub p95_duration: Duration,
    pub total_duration: Duration,
}

impl BenchmarkResult {
    /// Check if benchmark meets real-time constraints
    pub fn meets_real_time_constraint(&self, max_duration_ms: u64) -> bool {
        self.p95_duration.as_millis() <= max_duration_ms as u128
    }

    /// Print benchmark results
    pub fn print_results(&self) {
        println!("Benchmark: {}", self.description);
        println!("  Iterations: {}", self.iterations);
        println!("  Min: {:?}", self.min_duration);
        println!("  Max: {:?}", self.max_duration);
        println!("  Avg: {:?}", self.avg_duration);
        println!("  Median: {:?}", self.median_duration);
        println!("  95th percentile: {:?}", self.p95_duration);
        println!("  Total: {:?}", self.total_duration);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_trilateration_algorithm_performance() {
        let mut trilateration = AdvancedTrilateration::new();
        let anchors = PerformanceTestUtils::create_performance_test_anchors(4);
        let receiver_time = 1010;
        
        // Test weighted least squares performance
        let benchmark = PerformanceTestUtils::benchmark_operation(
            || {
                let weights = vec![1.0; anchors.len()];
                trilateration.weighted_least_squares_trilateration(&anchors, receiver_time, &weights)
            },
            100, // 100 iterations
            "Trilateration - Weighted Least Squares",
        );
        
        // Algorithm should complete within 100ms (real-time constraint)
        assert!(benchmark.meets_real_time_constraint(100),
               "Weighted Least Squares should meet 100ms constraint: 95th percentile = {:?}",
               benchmark.p95_duration);
        
        // Average should be much faster for typical use
        assert!(benchmark.avg_duration.as_millis() < 50,
               "Weighted Least Squares average should be < 50ms: {:?}",
               benchmark.avg_duration);
        
        if cfg!(debug_assertions) {
            benchmark.print_results();
        }
    }

    #[test]
    fn test_embedded_trilateration_performance() {
        let anchors = PerformanceTestUtils::create_performance_test_anchors(4);
        let receiver_time = 1010;
        
        // Benchmark different embedded configurations
        let configurations = vec![
            ("Floating Point", EmbeddedTrilateration::new()),
            ("Fixed Point", EmbeddedTrilateration::with_fixed_point()),
            ("Fast Mode", EmbeddedTrilateration::with_fast_mode()),
            ("Precision Mode", EmbeddedTrilateration::with_precision_mode()),
            ("Ultra Fast", config::ULTRA_FAST),
            ("Balanced", config::BALANCED),
        ];
        
        for (name, trilateration) in configurations {
            let benchmark = PerformanceTestUtils::benchmark_operation(
                || trilateration.calculate_position_embedded(&anchors, receiver_time),
                100,
                &format!("Embedded Trilateration - {}", name),
            );
            
            // All embedded algorithms should be very fast (< 50ms)
            assert!(benchmark.meets_real_time_constraint(50),
                   "Embedded {} should meet 50ms constraint: 95th percentile = {:?}",
                   name, benchmark.p95_duration);
            
            // Ultra-fast mode should be fastest
            if name == "Ultra Fast" {
                assert!(benchmark.avg_duration.as_millis() < 10,
                       "Ultra Fast should be < 10ms: {:?}",
                       benchmark.avg_duration);
            }
            
            if cfg!(debug_assertions) {
                benchmark.print_results();
            }
        }
    }

    #[test]
    fn test_gdop_optimization_performance() {
        let optimizer = GdopOptimizer::new();
        let anchors = PerformanceTestUtils::create_performance_test_anchors(8);
        let receiver_time = 1010;
        
        let benchmark = PerformanceTestUtils::benchmark_operation(
            || optimizer.select_optimal_anchors(&anchors, receiver_time, 1.0),
            50, // Fewer iterations as GDOP optimization is more complex
            "GDOP Anchor Selection",
        );
        
        // GDOP optimization should complete within 200ms
        assert!(benchmark.meets_real_time_constraint(200),
               "GDOP optimization should meet 200ms constraint: 95th percentile = {:?}",
               benchmark.p95_duration);
        
        if cfg!(debug_assertions) {
            benchmark.print_results();
        }
    }

    #[test]
    fn test_coordinate_transformation_performance() {
        let mut transformer = HighPrecisionTransformer::new();
        let mut coord_manager = EmbeddedCoordinateManager::new();
        
        let reference = Position { lat: 37.7749, lon: -122.4194, depth: 0.0 };
        coord_manager.set_reference_point(reference, 1000).unwrap();
        
        let test_position = Position { lat: 37.7759, lon: -122.4184, depth: 50.0 };
        
        // Test basic coordinate transformation performance
        let benchmark = PerformanceTestUtils::benchmark_operation(
            || {
                transformer.geodetic_to_ecef(&test_position)
            },
            1000, // Many iterations for fast operations
            "Coordinate Transformation - Geodetic to ECEF",
        );
        
        // Coordinate transformations should be very fast (< 10ms)
        assert!(benchmark.meets_real_time_constraint(10),
               "Geodetic to ECEF should meet 10ms constraint: 95th percentile = {:?}",
               benchmark.p95_duration);
        
        // Average should be sub-millisecond for simple transformations
        assert!(benchmark.avg_duration.as_micros() < 1000,
               "Geodetic to ECEF average should be < 1ms: {:?}",
               benchmark.avg_duration);
        
        if cfg!(debug_assertions) {
            benchmark.print_results();
        }
    }

    #[test]
    fn test_memory_usage_constraints() {
        // Test memory usage of key structures
        let memory_tests = vec![
            ("AdvancedTrilateration", PerformanceTestUtils::estimate_memory_usage::<AdvancedTrilateration>()),
            ("EmbeddedTrilateration", PerformanceTestUtils::estimate_memory_usage::<EmbeddedTrilateration>()),
            ("GdopOptimizer", PerformanceTestUtils::estimate_memory_usage::<GdopOptimizer>()),
            ("HighPrecisionTransformer", PerformanceTestUtils::estimate_memory_usage::<HighPrecisionTransformer>()),
            ("EmbeddedCoordinateManager", PerformanceTestUtils::estimate_memory_usage::<EmbeddedCoordinateManager>()),
            ("Anchor", PerformanceTestUtils::estimate_memory_usage::<Anchor>()),
            ("Position", PerformanceTestUtils::estimate_memory_usage::<Position>()),
        ];
        
        for (name, size) in memory_tests {
            println!("{}: {} bytes", name, size);
            
            // Validate memory constraints for embedded systems
            match name {
                "EmbeddedTrilateration" => {
                    assert!(size < 1024, "EmbeddedTrilateration should be < 1KB: {} bytes", size);
                },
                "EmbeddedCoordinateManager" => {
                    assert!(size < 4096, "EmbeddedCoordinateManager should be < 4KB: {} bytes", size);
                },
                "Anchor" | "Position" => {
                    assert!(size < 128, "{} should be < 128 bytes: {} bytes", name, size);
                },
                _ => {
                    // General constraint for all structures
                    assert!(size < 10240, "{} should be reasonable size: {} bytes", name, size);
                }
            }
        }
    }

    #[test]
    fn test_scalability_with_anchor_count() {
        let mut trilateration = AdvancedTrilateration::new();
        let anchor_counts = vec![3, 4, 5, 6, 8, 10];
        let mut performance_data = HashMap::new();
        
        for &count in &anchor_counts {
            let anchors = PerformanceTestUtils::create_performance_test_anchors(count);
            let receiver_time = 1010;
            
            let benchmark = PerformanceTestUtils::benchmark_operation(
                || {
                    let weights = vec![1.0; anchors.len()];
                    trilateration.weighted_least_squares_trilateration(&anchors, receiver_time, &weights)
                },
                50,
                &format!("Trilateration with {} anchors", count),
            );
            
            performance_data.insert(count, benchmark.avg_duration);
            
            // Performance should scale reasonably with anchor count
            assert!(benchmark.meets_real_time_constraint(100),
                   "Trilateration with {} anchors should meet 100ms constraint: 95th percentile = {:?}",
                   count, benchmark.p95_duration);
            
            if cfg!(debug_assertions) {
                benchmark.print_results();
            }
        }
        
        // Verify that performance doesn't degrade too much with more anchors
        let min_anchors_time = performance_data[&3];
        let max_anchors_time = performance_data[&10];
        
        // Performance shouldn't degrade by more than 5x
        assert!(max_anchors_time.as_nanos() < min_anchors_time.as_nanos() * 5,
               "Performance degradation too high: 3 anchors = {:?}, 10 anchors = {:?}",
               min_anchors_time, max_anchors_time);
    }

    #[test]
    fn test_stress_test_performance() {
        let mut trilateration = AdvancedTrilateration::new();
        let stress_anchors = PerformanceTestUtils::create_stress_test_anchors();
        let receiver_time = 1010;
        
        println!("Stress test with {} anchors", stress_anchors.len());
        
        // Test that system can handle large number of anchors
        let benchmark = PerformanceTestUtils::benchmark_operation(
            || {
                let weights = vec![1.0; stress_anchors.len()];
                trilateration.weighted_least_squares_trilateration(&stress_anchors, receiver_time, &weights)
            },
            10, // Fewer iterations for stress test
            "Stress Test - Many Anchors",
        );
        
        // Even with many anchors, should complete within 500ms
        assert!(benchmark.meets_real_time_constraint(500),
               "Stress test should meet 500ms constraint: 95th percentile = {:?}",
               benchmark.p95_duration);
        
        if cfg!(debug_assertions) {
            benchmark.print_results();
        }
    }

    #[test]
    fn test_algorithm_comparison_performance() {
        let anchors = PerformanceTestUtils::create_performance_test_anchors(4);
        let receiver_time = 1010;
        
        // Compare different algorithm approaches
        let mut results = Vec::new();
        
        // Standard trilateration
        let mut standard_tril = AdvancedTrilateration::new();
        let standard_benchmark = PerformanceTestUtils::benchmark_operation(
            || {
                let weights = vec![1.0; anchors.len()];
                standard_tril.weighted_least_squares_trilateration(&anchors, receiver_time, &weights)
            },
            100,
            "Standard Trilateration",
        );
        results.push(("Standard", standard_benchmark));
        
        // Embedded trilateration
        let embedded_tril = EmbeddedTrilateration::new();
        let embedded_benchmark = PerformanceTestUtils::benchmark_operation(
            || embedded_tril.calculate_position_embedded(&anchors, receiver_time),
            100,
            "Embedded Trilateration",
        );
        results.push(("Embedded", embedded_benchmark));
        
        // Fixed-point embedded
        let fixed_point_tril = EmbeddedTrilateration::with_fixed_point();
        let fixed_point_benchmark = PerformanceTestUtils::benchmark_operation(
            || fixed_point_tril.calculate_position_embedded(&anchors, receiver_time),
            100,
            "Fixed-Point Embedded",
        );
        results.push(("Fixed-Point", fixed_point_benchmark));
        
        // Print comparison
        if cfg!(debug_assertions) {
            println!("\nPerformance Comparison:");
            for (name, benchmark) in &results {
                println!("{}: avg = {:?}, 95th = {:?}", 
                        name, benchmark.avg_duration, benchmark.p95_duration);
            }
        }
        
        // All algorithms should meet real-time constraints
        for (name, benchmark) in &results {
            assert!(benchmark.meets_real_time_constraint(100),
                   "{} should meet 100ms constraint", name);
        }
        
        // Fixed-point should be fastest or comparable to floating-point
        let embedded_avg = results[1].1.avg_duration;
        let fixed_point_avg = results[2].1.avg_duration;
        
        // Fixed-point shouldn't be more than 2x slower than floating-point
        assert!(fixed_point_avg.as_nanos() < embedded_avg.as_nanos() * 2,
               "Fixed-point shouldn't be much slower: embedded = {:?}, fixed-point = {:?}",
               embedded_avg, fixed_point_avg);
    }

    #[test]
    fn test_real_time_constraint_validation() {
        // Test various real-time constraints
        let constraints = vec![
            (10, "Ultra-fast constraint"),
            (50, "Fast constraint"),
            (100, "Standard constraint"),
            (200, "Relaxed constraint"),
        ];
        
        let anchors = PerformanceTestUtils::create_performance_test_anchors(4);
        let receiver_time = 1010;
        let trilateration = EmbeddedTrilateration::with_fast_mode();
        
        for (max_ms, description) in constraints {
            let benchmark = PerformanceTestUtils::benchmark_operation(
                || trilateration.calculate_position_embedded(&anchors, receiver_time),
                50,
                &format!("Real-time test - {}", description),
            );
            
            let meets_constraint = benchmark.meets_real_time_constraint(max_ms);
            
            if max_ms >= 50 {
                // Should meet reasonable constraints
                assert!(meets_constraint,
                       "Should meet {} constraint: 95th percentile = {:?}",
                       description, benchmark.p95_duration);
            }
            
            if cfg!(debug_assertions) {
                println!("{}: meets {}ms constraint = {}", 
                        description, max_ms, meets_constraint);
            }
        }
    }

    #[test]
    fn test_performance_regression_detection() {
        // This test helps detect performance regressions
        let anchors = PerformanceTestUtils::create_performance_test_anchors(4);
        let receiver_time = 1010;
        let trilateration = EmbeddedTrilateration::new();
        
        let benchmark = PerformanceTestUtils::benchmark_operation(
            || trilateration.calculate_position_embedded(&anchors, receiver_time),
            100,
            "Regression Detection Test",
        );
        
        // Define performance baselines (these should be updated if legitimate improvements are made)
        let expected_max_avg_ms = 20; // Average should be < 20ms
        let expected_max_p95_ms = 50; // 95th percentile should be < 50ms
        
        assert!(benchmark.avg_duration.as_millis() < expected_max_avg_ms,
               "Performance regression detected: average = {:?}, expected < {}ms",
               benchmark.avg_duration, expected_max_avg_ms);
        
        assert!(benchmark.p95_duration.as_millis() < expected_max_p95_ms,
               "Performance regression detected: 95th percentile = {:?}, expected < {}ms",
               benchmark.p95_duration, expected_max_p95_ms);
        
        if cfg!(debug_assertions) {
            println!("Performance baseline check passed:");
            benchmark.print_results();
        }
    }
}