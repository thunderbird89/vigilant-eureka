// Transmission manager demonstration
// Shows how to use the transmission manager for coordinating underwater message broadcasts

use shared_positioning::{
    TransmissionManager, TransmissionConfig, TransmissionPriority, TransmissionMessageVersion,
    MockTransceiver, MockPowerManager, TransceiverConfig, TransceiverInterface,
    GeodeticPosition, EnvironmentalConditions
};
use uuid::Uuid;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    println!("=== Transmission Manager Demo ===\n");

    // Create and configure mock transceiver
    let mut transceiver = MockTransceiver::new(1);
    transceiver.configure(TransceiverConfig::default())?;
    println!("✓ Transceiver configured");

    // Create mock power manager
    let power_manager = MockPowerManager::new();
    println!("✓ Power manager initialized");

    // Create transmission configuration
    let config = TransmissionConfig::default();
    println!("✓ Transmission configuration created");

    // Create transmission manager
    let mut transmission_manager = TransmissionManager::new(config, transceiver, power_manager)?;
    println!("✓ Transmission manager created");

    // Start the transmission manager
    transmission_manager.start()?;
    println!("✓ Transmission manager started");

    // Create beacon position
    let beacon_id = Uuid::new_v4();
    let position = GeodeticPosition {
        latitude: 32.123456,
        longitude: -117.654321,
        depth: 10.5,
    };
    println!("✓ Beacon position: lat={:.6}, lon={:.6}, depth={:.1}m", 
             position.latitude, position.longitude, position.depth);

    // Schedule normal priority transmission
    println!("\n--- Scheduling Transmissions ---");
    let transmission_id1 = transmission_manager.schedule_transmission(
        beacon_id,
        position,
        200, // signal quality
        TransmissionMessageVersion::V3,
        TransmissionPriority::Normal,
        None, // immediate
    )?;
    println!("✓ Scheduled normal transmission: {}", transmission_id1);

    // Schedule high priority transmission
    let transmission_id2 = transmission_manager.schedule_transmission(
        beacon_id,
        position,
        180, // signal quality
        TransmissionMessageVersion::V2,
        TransmissionPriority::High,
        Some(2000), // 2 second delay
    )?;
    println!("✓ Scheduled high priority transmission: {}", transmission_id2);

    // Schedule emergency transmission
    let transmission_id3 = transmission_manager.schedule_transmission(
        beacon_id,
        position,
        150, // signal quality
        TransmissionMessageVersion::V1,
        TransmissionPriority::Emergency,
        Some(500), // 0.5 second delay
    )?;
    println!("✓ Scheduled emergency transmission: {}", transmission_id3);

    // Show queue status
    let (queue_size, queue_limit) = transmission_manager.get_queue_status();
    println!("✓ Queue status: {}/{} transmissions", queue_size, queue_limit);

    // Update environmental conditions for adaptive power
    println!("\n--- Environmental Adaptation ---");
    let environmental_conditions = EnvironmentalConditions {
        water_temperature_c: Some(15.0),
        salinity_ppt: Some(35.0),
        depth_m: Some(50.0),
        current_speed_ms: Some(0.5),
        noise_level_db: Some(85.0),
        signal_attenuation_factor: Some(0.3),
    };
    transmission_manager.update_environmental_conditions(environmental_conditions);
    println!("✓ Environmental conditions updated");

    // Process transmissions
    println!("\n--- Processing Transmissions ---");
    let processed_count = transmission_manager.process_transmissions()?;
    println!("✓ Processed {} transmissions", processed_count);

    // Show transmission statistics
    println!("\n--- Transmission Statistics ---");
    let stats = transmission_manager.get_statistics();
    println!("Total transmissions: {}", stats.total_transmissions);
    println!("Successful transmissions: {}", stats.successful_transmissions);
    println!("Failed transmissions: {}", stats.failed_transmissions);
    println!("Success rate: {:.2}%", stats.success_rate * 100.0);
    println!("Emergency transmissions: {}", stats.emergency_transmissions);
    println!("Power adaptations: {}", stats.adaptive_power_adjustments);
    println!("Environmental adaptations: {}", stats.environmental_adaptations);

    if let Some(avg_time) = stats.get_average_interval_ms() {
        println!("Average transmission interval: {:.1}ms", avg_time);
    }

    if let Some(power_trend) = stats.get_power_level_trend() {
        println!("Power level trend: {:.1}", power_trend);
    }

    // Show current power level
    println!("Current power level: {}", transmission_manager.get_current_power_level());

    // Show time until next transmission
    if let Some(time_until_next) = transmission_manager.time_until_next_transmission() {
        println!("Time until next transmission: {}ms", time_until_next.as_millis());
    } else {
        println!("No pending transmissions");
    }

    // Demonstrate configuration update
    println!("\n--- Configuration Update ---");
    let mut new_config = transmission_manager.get_config().clone();
    new_config.default_interval_ms = 3000; // Faster transmission
    new_config.max_retry_attempts = 5; // More retries
    transmission_manager.update_config(new_config)?;
    println!("✓ Configuration updated");

    // Stop the transmission manager
    transmission_manager.stop()?;
    println!("\n✓ Transmission manager stopped");

    println!("\n=== Demo Complete ===");
    Ok(())
}