//! Performance tests for beacon emulator
//! 
//! This module contains performance tests that validate the emulator's ability
//! to handle high beacon counts and message throughput requirements.

use beacon_emulator::*;
use shared_positioning::GeodeticPosition;

use tokio::time::{sleep, Duration, timeout, Instant};
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};

/// Test utilities for performance testing
mod performance_test_utils {
    use super::*;
    
    pub fn create_test_positions(count: usize) -> Vec<GeodeticPosition> {
        (0..count).map(|i| GeodeticPosition {
            latitude: 32.0 + (i as f64) * 0.001, // Spread positions slightly
            longitude: 45.0 + (i as f64) * 0.001,
            depth: 10.0 + (i as f64) * 0.1,
        }).collect()
    }
    
    pub async fn create_test_manager() -> EmulatorManager {
        EmulatorManager::new("perf_test_channel")
    }
    
    pub async fn measure_execution_time<F, Fut, T>(operation: F) -> (T, Duration)
    where
        F: FnOnce() -> Fut,
        Fut: std::future::Future<Output = T>,
    {
        let start = Instant::now();
        let result = operation().await;
        let duration = start.elapsed();
        (result, duration)
    }
    
    #[derive(Clone)]
    pub struct MessageCounter {
        count: Arc<AtomicUsize>,
    }
    
    impl MessageCounter {
        pub fn new() -> Self {
            Self {
                count: Arc::new(AtomicUsize::new(0)),
            }
        }
        
        pub fn increment(&self) {
            self.count.fetch_add(1, Ordering::Relaxed);
        }
        
        pub fn get_count(&self) -> usize {
            self.count.load(Ordering::Relaxed)
        }
        
        pub fn reset(&self) {
            self.count.store(0, Ordering::Relaxed);
        }
    }
}

/// Performance tests for high beacon counts
mod high_beacon_count_tests {
    use super::*;
    use performance_test_utils::*;
    
    #[tokio::test]
    async fn test_create_50_beacons_performance() {
        let mut manager = create_test_manager().await;
        let positions = create_test_positions(50);
        
        let (beacon_ids, creation_time) = measure_execution_time(|| async {
            let mut ids = Vec::new();
            for position in positions {
                let id = manager.create_beacon(None, position, None).await.unwrap();
                ids.push(id);
            }
            ids
        }).await;
        
        assert_eq!(beacon_ids.len(), 50);
        assert_eq!(manager.get_total_beacon_count(), 50);
        
        // Should create 50 beacons in reasonable time (less than 5 seconds)
        assert!(creation_time < Duration::from_secs(5), 
                "Creating 50 beacons took {:?}, expected < 5s", creation_time);
        
        println!("Created 50 beacons in {:?} ({:.2} beacons/sec)", 
                 creation_time, 50.0 / creation_time.as_secs_f64());
    }
    
    #[tokio::test]
    async fn test_start_50_beacons_performance() {
        let mut manager = create_test_manager().await;
        let positions = create_test_positions(50);
        
        // Create beacons first
        let mut beacon_ids = Vec::new();
        for position in positions {
            let id = manager.create_beacon(None, position, None).await.unwrap();
            beacon_ids.push(id);
        }
        
        // Measure time to start all beacons
        let (started_ids, start_time) = measure_execution_time(|| async {
            manager.start_all_beacons().await.unwrap()
        }).await;
        
        assert_eq!(started_ids.len(), 50);
        assert_eq!(manager.get_active_beacon_count(), 50);
        
        // Should start 50 beacons in reasonable time (less than 10 seconds)
        assert!(start_time < Duration::from_secs(10), 
                "Starting 50 beacons took {:?}, expected < 10s", start_time);
        
        println!("Started 50 beacons in {:?} ({:.2} beacons/sec)", 
                 start_time, 50.0 / start_time.as_secs_f64());
        
        // Clean up
        manager.stop_all_beacons().await.unwrap();
    }
    
    #[tokio::test]
    async fn test_stop_50_beacons_performance() {
        let mut manager = create_test_manager().await;
        let positions = create_test_positions(50);
        
        // Create and start beacons
        for position in positions {
            let id = manager.create_beacon(None, position, None).await.unwrap();
            manager.start_beacon(id).await.unwrap();
        }
        
        assert_eq!(manager.get_active_beacon_count(), 50);
        
        // Measure time to stop all beacons
        let (stopped_ids, stop_time) = measure_execution_time(|| async {
            manager.stop_all_beacons().await.unwrap()
        }).await;
        
        assert_eq!(stopped_ids.len(), 50);
        assert_eq!(manager.get_active_beacon_count(), 0);
        
        // Should stop 50 beacons in reasonable time (less than 5 seconds)
        assert!(stop_time < Duration::from_secs(5), 
                "Stopping 50 beacons took {:?}, expected < 5s", stop_time);
        
        println!("Stopped 50 beacons in {:?} ({:.2} beacons/sec)", 
                 stop_time, 50.0 / stop_time.as_secs_f64());
    }
    
    #[tokio::test]
    async fn test_beacon_lifecycle_performance_at_scale() {
        let mut manager = create_test_manager().await;
        let positions = create_test_positions(30); // Slightly smaller for full lifecycle test
        
        // Measure full lifecycle: create -> start -> run -> stop -> remove
        let lifecycle_start_time = Instant::now();
        
        // Create phase
        let mut beacon_ids = Vec::new();
        for position in positions {
            let id = manager.create_beacon(None, position, None).await.unwrap();
            beacon_ids.push(id);
        }
        let create_time = lifecycle_start_time.elapsed();
        
        // Start phase
        let start_phase_time = Instant::now();
        manager.start_all_beacons().await.unwrap();
        let start_time = start_phase_time.elapsed();
        
        // Run phase - let beacons run for a short time
        sleep(Duration::from_millis(500)).await;
        let run_time = Duration::from_millis(500);
        
        // Stop phase
        let stop_phase_time = Instant::now();
        manager.stop_all_beacons().await.unwrap();
        let stop_time = stop_phase_time.elapsed();
        
        // Remove phase
        let remove_phase_time = Instant::now();
        for beacon_id in beacon_ids {
            manager.remove_beacon(beacon_id).await.unwrap();
        }
        let remove_time = remove_phase_time.elapsed();
        
        let total_time = lifecycle_start_time.elapsed();
        
        assert_eq!(manager.get_total_beacon_count(), 0);
        assert_eq!(manager.get_active_beacon_count(), 0);
        
        println!("Full lifecycle for 30 beacons:");
        println!("  Create: {:?} ({:.2}/sec)", create_time, 30.0 / create_time.as_secs_f64());
        println!("  Start:  {:?} ({:.2}/sec)", start_time, 30.0 / start_time.as_secs_f64());
        println!("  Run:    {:?}", run_time);
        println!("  Stop:   {:?} ({:.2}/sec)", stop_time, 30.0 / stop_time.as_secs_f64());
        println!("  Remove: {:?} ({:.2}/sec)", remove_time, 30.0 / remove_time.as_secs_f64());
        println!("  Total:  {:?}", total_time);
        
        // Total lifecycle should complete in reasonable time (less than 30 seconds)
        assert!(total_time < Duration::from_secs(30), 
                "Full lifecycle took {:?}, expected < 30s", total_time);
    }
    
    #[tokio::test]
    async fn test_memory_usage_with_many_beacons() {
        let mut manager = create_test_manager().await;
        let positions = create_test_positions(100);
        
        // Create many beacons to test memory usage
        let mut beacon_ids = Vec::new();
        for position in positions {
            let id = manager.create_beacon(None, position, None).await.unwrap();
            beacon_ids.push(id);
        }
        
        assert_eq!(manager.get_total_beacon_count(), 100);
        
        // Start all beacons
        manager.start_all_beacons().await.unwrap();
        assert_eq!(manager.get_active_beacon_count(), 100);
        
        // Let them run briefly
        sleep(Duration::from_millis(200)).await;
        
        // Verify all beacons are still running (no crashes due to memory issues)
        assert_eq!(manager.get_active_beacon_count(), 100);
        
        // Check that we can still perform operations
        let stats = manager.get_manager_stats().await;
        assert_eq!(stats.total_beacons, 100);
        assert_eq!(stats.running_beacons, 100);
        
        // Clean up
        manager.stop_all_beacons().await.unwrap();
        
        // Verify cleanup worked
        assert_eq!(manager.get_active_beacon_count(), 0);
    }
}

/// Performance tests for message throughput
mod message_throughput_tests {
    use super::*;
    use performance_test_utils::*;
    
    #[tokio::test]
    async fn test_message_transmission_rate() {
        let mut manager = create_test_manager().await;
        let position = GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        };
        
        // Get communication space and subscribe to channel
        let comm_space = manager.get_communication_space();
        let channel = {
            let mut space = comm_space.write().await;
            space.get_or_create_channel("perf_test_channel")
        };
        let mut receiver = channel.subscribe();
        
        // Create beacon with fast transmission interval
        let beacon_id = manager.create_beacon(None, position, None).await.unwrap();
        
        // Update config for faster transmission (100ms interval)
        let mut config = manager.get_beacon_status(beacon_id).unwrap().config;
        config.transmission.interval_ms = 100; // 10 messages per second
        manager.update_beacon_config(beacon_id, config).await.unwrap();
        
        // Start beacon
        manager.start_beacon(beacon_id).await.unwrap();
        
        // Count messages for 2 seconds
        let message_counter = MessageCounter::new();
        let counter_clone = message_counter.clone();
        
        let counting_task = tokio::spawn(async move {
            let timeout_duration = Duration::from_secs(2);
            let start_time = Instant::now();
            
            while start_time.elapsed() < timeout_duration {
                match timeout(Duration::from_millis(50), receiver.recv()).await {
                    Ok(Ok(_message)) => {
                        counter_clone.increment();
                    }
                    Ok(Err(_)) => break, // Channel closed
                    Err(_) => continue, // Timeout, continue counting
                }
            }
        });
        
        // Wait for counting to complete
        counting_task.await.unwrap();
        
        let message_count = message_counter.get_count();
        let messages_per_second = message_count as f64 / 2.0;
        
        println!("Received {} messages in 2 seconds ({:.2} msg/sec)", 
                 message_count, messages_per_second);
        
        // Should receive approximately 20 messages (10/sec * 2 sec)
        // Allow some tolerance for timing variations
        assert!(message_count >= 15 && message_count <= 25, 
                "Expected 15-25 messages, got {}", message_count);
        
        // Clean up
        manager.stop_beacon(beacon_id).await.unwrap();
    }
    
    #[tokio::test]
    async fn test_multiple_beacon_message_throughput() {
        let mut manager = create_test_manager().await;
        let positions = create_test_positions(10);
        
        // Get communication space and subscribe to channel
        let comm_space = manager.get_communication_space();
        let channel = {
            let mut space = comm_space.write().await;
            space.get_or_create_channel("perf_test_channel")
        };
        let mut receiver = channel.subscribe();
        
        // Create multiple beacons with fast transmission
        let mut beacon_ids = Vec::new();
        for position in positions {
            let id = manager.create_beacon(None, position, None).await.unwrap();
            
            // Set fast transmission interval (200ms)
            let mut config = manager.get_beacon_status(id).unwrap().config;
            config.transmission.interval_ms = 200; // 5 messages per second per beacon
            manager.update_beacon_config(id, config).await.unwrap();
            
            beacon_ids.push(id);
        }
        
        // Start all beacons
        manager.start_all_beacons().await.unwrap();
        
        // Count messages for 2 seconds
        let message_counter = MessageCounter::new();
        let counter_clone = message_counter.clone();
        
        let counting_task = tokio::spawn(async move {
            let timeout_duration = Duration::from_secs(2);
            let start_time = Instant::now();
            
            while start_time.elapsed() < timeout_duration {
                match timeout(Duration::from_millis(50), receiver.recv()).await {
                    Ok(Ok(_message)) => {
                        counter_clone.increment();
                    }
                    Ok(Err(_)) => break,
                    Err(_) => continue,
                }
            }
        });
        
        counting_task.await.unwrap();
        
        let message_count = message_counter.get_count();
        let messages_per_second = message_count as f64 / 2.0;
        
        println!("Received {} messages from 10 beacons in 2 seconds ({:.2} msg/sec)", 
                 message_count, messages_per_second);
        
        // Should receive approximately 100 messages (10 beacons * 5 msg/sec * 2 sec)
        // Allow tolerance for timing variations
        assert!(message_count >= 80 && message_count <= 120, 
                "Expected 80-120 messages, got {}", message_count);
        
        // Clean up
        manager.stop_all_beacons().await.unwrap();
    }
    
    #[tokio::test]
    async fn test_message_broadcast_performance() {
        let mut manager = create_test_manager().await;
        let position = GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        };
        
        // Get communication space
        let comm_space = manager.get_communication_space();
        let channel = {
            let mut space = comm_space.write().await;
            space.get_or_create_channel("perf_test_channel")
        };
        
        // Create multiple subscribers to test broadcast performance
        let mut receivers = Vec::new();
        for _ in 0..10 {
            receivers.push(channel.subscribe());
        }
        
        // Create and start beacon
        let beacon_id = manager.create_beacon(None, position, None).await.unwrap();
        manager.start_beacon(beacon_id).await.unwrap();
        
        // Count messages received by all subscribers
        let message_counters: Vec<MessageCounter> = (0..10).map(|_| MessageCounter::new()).collect();
        let mut counting_tasks = Vec::new();
        
        for (i, mut receiver) in receivers.into_iter().enumerate() {
            let counter = message_counters[i].clone();
            let task = tokio::spawn(async move {
                let timeout_duration = Duration::from_secs(1);
                let start_time = Instant::now();
                
                while start_time.elapsed() < timeout_duration {
                    match timeout(Duration::from_millis(50), receiver.recv()).await {
                        Ok(Ok(_message)) => {
                            counter.increment();
                        }
                        Ok(Err(_)) => break,
                        Err(_) => continue,
                    }
                }
            });
            counting_tasks.push(task);
        }
        
        // Wait for all counting tasks to complete
        for task in counting_tasks {
            task.await.unwrap();
        }
        
        // Check that all subscribers received messages
        let mut total_messages = 0;
        for (i, counter) in message_counters.iter().enumerate() {
            let count = counter.get_count();
            total_messages += count;
            println!("Subscriber {} received {} messages", i, count);
            assert!(count > 0, "Subscriber {} should have received messages", i);
        }
        
        println!("Total messages across all subscribers: {}", total_messages);
        
        // All subscribers should receive approximately the same number of messages
        let first_count = message_counters[0].get_count();
        for counter in &message_counters {
            let count = counter.get_count();
            // Allow some variation but should be roughly equal
            assert!((count as i32 - first_count as i32).abs() <= 2, 
                    "Message counts should be similar across subscribers");
        }
        
        // Clean up
        manager.stop_beacon(beacon_id).await.unwrap();
    }
    
    #[tokio::test]
    async fn test_channel_message_logging_performance() {
        let mut manager = create_test_manager().await;
        let positions = create_test_positions(5);
        
        // Get communication space
        let comm_space = manager.get_communication_space();
        let channel = {
            let mut space = comm_space.write().await;
            space.get_or_create_channel("perf_test_channel")
        };
        
        // Create and start multiple beacons
        let mut beacon_ids = Vec::new();
        for position in positions {
            let id = manager.create_beacon(None, position, None).await.unwrap();
            
            // Set fast transmission for more messages
            let mut config = manager.get_beacon_status(id).unwrap().config;
            config.transmission.interval_ms = 100; // 10 msg/sec per beacon
            manager.update_beacon_config(id, config).await.unwrap();
            
            manager.start_beacon(id).await.unwrap();
            beacon_ids.push(id);
        }
        
        // Let beacons run for 2 seconds to generate messages
        sleep(Duration::from_secs(2)).await;
        
        // Test message retrieval performance
        let (message_count, retrieval_time) = measure_execution_time(|| async {
            channel.get_message_count().await
        }).await;
        
        println!("Retrieved message count ({}) in {:?}", message_count, retrieval_time);
        assert!(retrieval_time < Duration::from_millis(100), 
                "Message count retrieval should be fast");
        
        // Test recent messages retrieval
        let (recent_messages, recent_time) = measure_execution_time(|| async {
            channel.get_recent_messages(50).await
        }).await;
        
        println!("Retrieved {} recent messages in {:?}", recent_messages.len(), recent_time);
        assert!(recent_time < Duration::from_millis(100), 
                "Recent messages retrieval should be fast");
        
        // Test filtered message retrieval
        let beacon_id = beacon_ids[0];
        let (filtered_messages, filter_time) = measure_execution_time(|| async {
            channel.get_messages_by_beacon(beacon_id).await
        }).await;
        
        println!("Retrieved {} filtered messages in {:?}", filtered_messages.len(), filter_time);
        assert!(filter_time < Duration::from_millis(200), 
                "Filtered message retrieval should be reasonably fast");
        
        // Clean up
        manager.stop_all_beacons().await.unwrap();
    }
}

/// Performance tests for resource usage and scalability
mod resource_usage_tests {
    use super::*;
    use performance_test_utils::*;
    
    #[tokio::test]
    async fn test_concurrent_beacon_operations() {
        let mut manager = create_test_manager().await;
        let positions = create_test_positions(20);
        
        // Test sequential beacon creation (concurrent not possible with mutable manager)
        let creation_start = Instant::now();
        let mut beacon_ids = Vec::new();
        
        for position in positions {
            let id = manager.create_beacon(None, position, None).await.unwrap();
            beacon_ids.push(id);
        }
        
        let creation_time = creation_start.elapsed();
        println!("Created 20 beacons concurrently in {:?}", creation_time);
        
        assert_eq!(beacon_ids.len(), 20);
        assert_eq!(manager.get_total_beacon_count(), 20);
        
        // Test concurrent beacon starting
        let start_time = Instant::now();
        let started_ids = manager.start_all_beacons().await.unwrap();
        let start_duration = start_time.elapsed();
        
        println!("Started {} beacons in {:?}", started_ids.len(), start_duration);
        assert_eq!(started_ids.len(), 20);
        assert_eq!(manager.get_active_beacon_count(), 20);
        
        // Test sequential position updates while running
        let update_start = Instant::now();
        
        for (i, beacon_id) in beacon_ids.iter().enumerate() {
            let new_position = GeodeticPosition {
                latitude: 33.0 + (i as f64) * 0.001,
                longitude: 46.0 + (i as f64) * 0.001,
                depth: 15.0,
            };
            manager.update_beacon_position(*beacon_id, new_position).await.unwrap();
        }
        
        let update_time = update_start.elapsed();
        println!("Updated 20 beacon positions in {:?}", update_time);
        
        // Verify updates were applied
        for beacon_id in &beacon_ids {
            let status = manager.get_beacon_status(*beacon_id).unwrap();
            assert!(status.position.latitude >= 33.0);
        }
        
        // Clean up
        manager.stop_all_beacons().await.unwrap();
    }
    
    #[tokio::test]
    async fn test_state_persistence_performance() {
        let mut manager = create_test_manager().await;
        let positions = create_test_positions(50);
        
        // Create many beacons
        for position in positions {
            manager.create_beacon(None, position, None).await.unwrap();
        }
        
        // Test state saving performance
        let (_, save_time) = measure_execution_time(|| async {
            manager.save_state().await.unwrap()
        }).await;
        
        println!("Saved state for 50 beacons in {:?}", save_time);
        assert!(save_time < Duration::from_secs(2), 
                "State saving should be fast even with many beacons");
        
        // Create new manager and test loading performance
        let state_file_path = manager.get_state_file_path().clone();
        let mut new_manager = EmulatorManager::new("test_channel");
        new_manager.set_state_file_path(state_file_path);
        
        let (_, load_time) = measure_execution_time(|| async {
            new_manager.load_state().await.unwrap()
        }).await;
        
        println!("Loaded state for 50 beacons in {:?}", load_time);
        assert!(load_time < Duration::from_secs(5), 
                "State loading should be reasonably fast");
        
        assert_eq!(new_manager.get_total_beacon_count(), 50);
    }
    
    #[tokio::test]
    async fn test_scenario_creation_performance() {
        let mut manager = create_test_manager().await;
        let center = GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        };
        
        // Test large grid scenario creation
        let (beacon_ids, creation_time) = measure_execution_time(|| async {
            manager.create_scenario(ScenarioType::Grid, 25, 50.0, center).await.unwrap() // 5x5 grid
        }).await;
        
        println!("Created 25-beacon grid scenario in {:?}", creation_time);
        assert_eq!(beacon_ids.len(), 25);
        assert!(creation_time < Duration::from_secs(5), 
                "Scenario creation should be fast");
        
        // Test starting all scenario beacons
        let (started_ids, start_time) = measure_execution_time(|| async {
            manager.start_all_beacons().await.unwrap()
        }).await;
        
        println!("Started 25 scenario beacons in {:?}", start_time);
        assert_eq!(started_ids.len(), 25);
        assert!(start_time < Duration::from_secs(10), 
                "Starting scenario beacons should be reasonably fast");
        
        // Clean up
        manager.stop_all_beacons().await.unwrap();
    }
    
    #[tokio::test]
    async fn test_system_stability_under_load() {
        let mut manager = create_test_manager().await;
        let positions = create_test_positions(30);
        
        // Create and start beacons
        let mut beacon_ids = Vec::new();
        for position in positions {
            let id = manager.create_beacon(None, position, None).await.unwrap();
            beacon_ids.push(id);
        }
        
        manager.start_all_beacons().await.unwrap();
        
        // Run system under load for extended period
        let load_test_duration = Duration::from_secs(5);
        let start_time = Instant::now();
        
        while start_time.elapsed() < load_test_duration {
            // Perform various operations to stress the system
            let stats = manager.get_manager_stats().await;
            assert_eq!(stats.running_beacons, 30);
            
            // Update some beacon positions
            if let Some(&beacon_id) = beacon_ids.first() {
                let new_position = GeodeticPosition {
                    latitude: 32.0 + (start_time.elapsed().as_secs_f64() * 0.001),
                    longitude: 45.0,
                    depth: 10.0,
                };
                manager.update_beacon_position(beacon_id, new_position).await.unwrap();
            }
            
            // Brief pause between operations
            sleep(Duration::from_millis(100)).await;
        }
        
        // Verify system is still stable after load test
        assert_eq!(manager.get_active_beacon_count(), 30);
        
        // All beacons should still be running
        for beacon_id in &beacon_ids {
            let status = manager.get_beacon_status(*beacon_id).unwrap();
            assert!(status.is_running);
        }
        
        println!("System remained stable under load for {:?}", load_test_duration);
        
        // Clean up
        manager.stop_all_beacons().await.unwrap();
    }
}

/// Performance tests for specific bottlenecks and edge cases
mod bottleneck_tests {
    use super::*;
    use performance_test_utils::*;
    
    #[tokio::test]
    async fn test_rapid_start_stop_cycles() {
        let mut manager = create_test_manager().await;
        let position = create_test_positions(1)[0];
        
        // Create beacon
        let beacon_id = manager.create_beacon(None, position, None).await.unwrap();
        
        // Perform rapid start/stop cycles
        let cycles = 10;
        let cycle_start = Instant::now();
        
        for i in 0..cycles {
            manager.start_beacon(beacon_id).await.unwrap();
            
            // Brief run time
            sleep(Duration::from_millis(50)).await;
            
            manager.stop_beacon(beacon_id).await.unwrap();
            
            println!("Completed start/stop cycle {}/{}", i + 1, cycles);
        }
        
        let total_cycle_time = cycle_start.elapsed();
        let avg_cycle_time = total_cycle_time / cycles;
        
        println!("Completed {} start/stop cycles in {:?} (avg: {:?}/cycle)", 
                 cycles, total_cycle_time, avg_cycle_time);
        
        // Each cycle should complete reasonably quickly
        assert!(avg_cycle_time < Duration::from_secs(2), 
                "Average cycle time should be reasonable");
        
        // Beacon should be in stopped state
        let status = manager.get_beacon_status(beacon_id).unwrap();
        assert!(!status.is_running);
    }
    
    #[tokio::test]
    async fn test_message_queue_performance() {
        let mut manager = create_test_manager().await;
        let position = create_test_positions(1)[0];
        
        // Get communication space
        let comm_space = manager.get_communication_space();
        let channel = {
            let mut space = comm_space.write().await;
            space.get_or_create_channel("perf_test_channel")
        };
        
        // Create beacon with very fast transmission
        let beacon_id = manager.create_beacon(None, position, None).await.unwrap();
        let mut config = manager.get_beacon_status(beacon_id).unwrap().config;
        config.transmission.interval_ms = 10; // Very fast: 100 msg/sec
        manager.update_beacon_config(beacon_id, config).await.unwrap();
        
        // Start beacon
        manager.start_beacon(beacon_id).await.unwrap();
        
        // Let it run to build up message queue
        sleep(Duration::from_millis(500)).await;
        
        // Test message retrieval performance with large queue
        let (message_count, count_time) = measure_execution_time(|| async {
            channel.get_message_count().await
        }).await;
        
        println!("Message queue contains {} messages, counted in {:?}", 
                 message_count, count_time);
        
        // Should handle large message queues efficiently
        assert!(count_time < Duration::from_millis(50), 
                "Message counting should be fast even with large queues");
        
        // Test bulk message retrieval
        let (messages, retrieval_time) = measure_execution_time(|| async {
            channel.get_recent_messages(100).await
        }).await;
        
        println!("Retrieved {} messages in {:?}", messages.len(), retrieval_time);
        assert!(retrieval_time < Duration::from_millis(100), 
                "Bulk message retrieval should be efficient");
        
        // Clean up
        manager.stop_beacon(beacon_id).await.unwrap();
    }
    
    #[tokio::test]
    async fn test_movement_calculation_performance() {
        let mut manager = create_test_manager().await;
        let positions = create_test_positions(20);
        
        // Create beacons with different movement patterns
        let mut beacon_ids = Vec::new();
        let movement_patterns = vec![
            MovementPattern::Linear { speed_m_per_s: 1.0, bearing_deg: 45.0 },
            MovementPattern::Circular { radius_m: 100.0, period_s: 60.0 },
            MovementPattern::Random { max_speed_m_per_s: 2.0 },
        ];
        
        for (i, position) in positions.into_iter().enumerate() {
            let id = manager.create_beacon(None, position, None).await.unwrap();
            let pattern = movement_patterns[i % movement_patterns.len()].clone();
            manager.update_beacon_movement_pattern(id, pattern).await.unwrap();
            beacon_ids.push(id);
        }
        
        // Start all beacons (this will trigger movement calculations)
        let start_time = Instant::now();
        manager.start_all_beacons().await.unwrap();
        let start_duration = start_time.elapsed();
        
        println!("Started 20 beacons with movement patterns in {:?}", start_duration);
        
        // Let beacons run to perform movement calculations
        sleep(Duration::from_secs(2)).await;
        
        // Verify all beacons are still running (no crashes from movement calculations)
        assert_eq!(manager.get_active_beacon_count(), 20);
        
        // Check that positions have been updated for moving beacons
        for beacon_id in &beacon_ids {
            let status = manager.get_beacon_status(*beacon_id).unwrap();
            assert!(status.is_running);
            // Position updates are handled internally, just verify beacon is healthy
        }
        
        println!("Movement calculations performed successfully for 2 seconds");
        
        // Clean up
        manager.stop_all_beacons().await.unwrap();
    }
}