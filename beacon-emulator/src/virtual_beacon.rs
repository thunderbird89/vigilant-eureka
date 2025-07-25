use uuid::Uuid;
use serde::{Serialize, Deserialize, Serializer, Deserializer};
use std::time::{SystemTime, UNIX_EPOCH, Duration};
use tokio::time::interval;
use tokio::sync::mpsc;
use shared_positioning::{
    BeaconConfig,
    GeodeticPosition,
    MessageBuilder,
    MessageVersion,
};
use crate::{
    EmulatorError, 
    VirtualChannel, 
    VirtualMessage,
    MovementPattern,
    movement::{MovementCoordinateTransformer, MovementPatternValidator}
};

/// Control messages for beacon lifecycle management
#[derive(Debug)]
enum BeaconControl {
    Stop,
    UpdatePosition(GeodeticPosition),
    UpdateMovementPattern(MovementPattern),
    UpdateConfig(BeaconConfig),
}

/// Virtual beacon that emulates real beacon behavior
pub struct VirtualBeacon {
    id: Uuid,
    config: BeaconConfig,
    position: GeodeticPosition,
    initial_position: GeodeticPosition, // Store initial position for circular movement
    movement_pattern: MovementPattern,
    sequence_number: u16,
    is_running: bool,
    virtual_channel: VirtualChannel,
    stats: VirtualBeaconStats,
    message_builder: MessageBuilder,
    start_time: Option<SystemTime>,
    control_tx: Option<mpsc::UnboundedSender<BeaconControl>>,
}

impl VirtualBeacon {
    pub fn new(
        id: Uuid,
        config: BeaconConfig,
        initial_position: GeodeticPosition,
        virtual_channel: VirtualChannel,
    ) -> Result<Self, EmulatorError> {
        Ok(Self {
            id,
            config,
            position: initial_position,
            initial_position,
            movement_pattern: MovementPattern::Stationary,
            sequence_number: 0,
            is_running: false,
            virtual_channel,
            stats: VirtualBeaconStats::new(),
            message_builder: MessageBuilder::new(),
            start_time: None,
            control_tx: None,
        })
    }
    
    pub fn id(&self) -> Uuid {
        self.id
    }
    
    pub fn position(&self) -> GeodeticPosition {
        self.position
    }
    
    pub fn is_running(&self) -> bool {
        self.is_running
    }
    
    pub fn get_status(&self) -> VirtualBeaconStatus {
        let uptime = if let Some(start_time) = self.start_time {
            SystemTime::now().duration_since(start_time).unwrap_or_default()
        } else {
            Duration::new(0, 0)
        };
        
        let mut stats = self.stats.clone();
        stats.uptime = uptime;
        
        VirtualBeaconStatus {
            id: self.id,
            position: self.position,
            is_running: self.is_running,
            movement_pattern: self.movement_pattern.clone(),
            stats,
            config: self.config.clone(),
        }
    }
    
    /// Start the virtual beacon transmission loop
    pub async fn start(&mut self) -> Result<tokio::task::JoinHandle<()>, EmulatorError> {
        if self.is_running {
            return Err(EmulatorError::ConfigError("Beacon is already running".to_string()));
        }
        
        self.is_running = true;
        self.start_time = Some(SystemTime::now());
        self.sequence_number = 0;
        
        // Create control channel
        let (control_tx, control_rx) = mpsc::unbounded_channel();
        self.control_tx = Some(control_tx);
        
        // Clone necessary data for the async task
        let id = self.id;
        let config = self.config.clone();
        let position = self.position;
        let initial_position = self.initial_position;
        let movement_pattern = self.movement_pattern.clone();
        let virtual_channel = self.virtual_channel.clone();
        let message_builder = MessageBuilder::new();
        let sequence_number = self.sequence_number;
        let stats = self.stats.clone();
        
        // Spawn the transmission task
        let task_handle = tokio::spawn(async move {
            Self::transmission_loop(
                id,
                config,
                position,
                initial_position,
                movement_pattern,
                virtual_channel,
                message_builder,
                sequence_number,
                stats,
                control_rx,
            ).await;
        });
        
        Ok(task_handle)
    }
    
    /// Stop the virtual beacon
    pub fn stop(&mut self) -> Result<(), EmulatorError> {
        if !self.is_running {
            return Ok(());
        }
        
        self.is_running = false;
        
        if let Some(control_tx) = &self.control_tx {
            let _ = control_tx.send(BeaconControl::Stop);
        }
        
        self.control_tx = None;
        Ok(())
    }
    
    /// Update beacon position with validation
    pub fn update_position(&mut self, new_position: GeodeticPosition) -> Result<(), EmulatorError> {
        // Validate the new position before setting it
        MovementPatternValidator::validate_position(new_position)?;
        
        self.position = new_position;
        
        if let Some(control_tx) = &self.control_tx {
            control_tx.send(BeaconControl::UpdatePosition(new_position))
                .map_err(|_| EmulatorError::ChannelError("Failed to send position update".to_string()))?;
        }
        
        Ok(())
    }
    
    /// Set movement pattern with validation
    pub fn set_movement_pattern(&mut self, pattern: MovementPattern) -> Result<(), EmulatorError> {
        // Validate the movement pattern before setting it
        MovementPatternValidator::validate_pattern(&pattern)?;
        
        self.movement_pattern = pattern.clone();
        
        if let Some(control_tx) = &self.control_tx {
            control_tx.send(BeaconControl::UpdateMovementPattern(pattern))
                .map_err(|_| EmulatorError::ChannelError("Failed to send movement pattern update".to_string()))?;
        }
        
        Ok(())
    }
    
    /// Update beacon configuration
    pub fn update_config(&mut self, new_config: BeaconConfig) -> Result<(), EmulatorError> {
        self.config = new_config.clone();
        
        if let Some(control_tx) = &self.control_tx {
            control_tx.send(BeaconControl::UpdateConfig(new_config))
                .map_err(|_| EmulatorError::ChannelError("Failed to send config update".to_string()))?;
        }
        
        Ok(())
    }
    
    /// Main transmission loop that runs in a separate task
    async fn transmission_loop(
        id: Uuid,
        mut config: BeaconConfig,
        mut position: GeodeticPosition,
        initial_position: GeodeticPosition,
        mut movement_pattern: MovementPattern,
        virtual_channel: VirtualChannel,
        message_builder: MessageBuilder,
        mut sequence_number: u16,
        mut stats: VirtualBeaconStats,
        mut control_rx: mpsc::UnboundedReceiver<BeaconControl>,
    ) {
        let mut transmission_interval = interval(Duration::from_millis(
            config.transmission.interval_ms as u64
        ));
        let loop_start_time = SystemTime::now();
        
        loop {
            tokio::select! {
                // Handle control messages
                control_msg = control_rx.recv() => {
                    match control_msg {
                        Some(BeaconControl::Stop) => {
                            break;
                        }
                        Some(BeaconControl::UpdatePosition(new_position)) => {
                            position = new_position;
                        }
                        Some(BeaconControl::UpdateMovementPattern(new_pattern)) => {
                            movement_pattern = new_pattern;
                        }
                        Some(BeaconControl::UpdateConfig(new_config)) => {
                            config = new_config;
                            // Update transmission interval
                            transmission_interval = interval(Duration::from_millis(
                                config.transmission.interval_ms as u64
                            ));
                        }
                        None => break, // Channel closed
                    }
                }
                
                // Handle transmission timing
                _ = transmission_interval.tick() => {
                    // Update position based on movement pattern
                    position = Self::update_position_from_movement(
                        position,
                        initial_position,
                        &movement_pattern,
                        loop_start_time,
                        config.transmission.interval_ms as u64,
                        stats.messages_sent,
                    );
                    
                    // Build and transmit message
                    match Self::build_and_transmit_message(
                        &message_builder,
                        id,
                        position,
                        sequence_number,
                        &config,
                        &virtual_channel,
                    ).await {
                        Ok(()) => {
                            stats.messages_sent += 1;
                            stats.last_transmission = Some(SystemTime::now());
                            sequence_number = sequence_number.wrapping_add(1);
                        }
                        Err(_) => {
                            stats.transmission_failures += 1;
                        }
                    }
                }
            }
        }
    }
    
    /// Build and transmit a message based on configuration
    async fn build_and_transmit_message(
        message_builder: &MessageBuilder,
        beacon_id: Uuid,
        position: GeodeticPosition,
        sequence_number: u16,
        config: &BeaconConfig,
        virtual_channel: &VirtualChannel,
    ) -> Result<(), EmulatorError> {
        // Determine message version from config
        let message_version = match config.transmission.message_version {
            shared_positioning::BeaconMessageVersion::V1 => MessageVersion::V1,
            shared_positioning::BeaconMessageVersion::V2 => MessageVersion::V2,
            shared_positioning::BeaconMessageVersion::V3 => MessageVersion::V3,
        };
        
        // Build message based on version
        let message_data = match message_version {
            MessageVersion::V1 => {
                message_builder.build_v1_message_with_uuid(
                    beacon_id,
                    position,
                    255, // Full signal quality for virtual beacons
                    sequence_number,
                ).map_err(|e| EmulatorError::MessageBuildError(e.to_string()))?
            }
            MessageVersion::V2 => {
                message_builder.build_v2_message_with_uuid(
                    beacon_id,
                    position,
                    255,
                    sequence_number,
                ).map_err(|e| EmulatorError::MessageBuildError(e.to_string()))?
            }
            MessageVersion::V3 => {
                message_builder.build_v3_message(
                    beacon_id,
                    position,
                    255,
                    sequence_number,
                ).map_err(|e| EmulatorError::MessageBuildError(e.to_string()))?
            }
        };
        
        // Create virtual message
        let virtual_message = VirtualMessage {
            beacon_id,
            timestamp: SystemTime::now(),
            position,
            message_data,
            signal_quality: 255,
        };
        
        // Transmit to virtual channel
        virtual_channel.broadcast_message(virtual_message).await?;
        
        Ok(())
    }
    
    /// Update position based on movement pattern using high-precision coordinate transformations
    fn update_position_from_movement(
        current_position: GeodeticPosition,
        initial_position: GeodeticPosition,
        movement_pattern: &MovementPattern,
        loop_start_time: SystemTime,
        transmission_interval_ms: u64,
        _message_count: u64,
    ) -> GeodeticPosition {
        // Validate current position
        if let Err(_) = MovementPatternValidator::validate_position(current_position) {
            // If position is invalid, return current position without movement
            return current_position;
        }
        
        let time_delta_s = transmission_interval_ms as f64 / 1000.0;
        
        // Validate time delta
        if let Err(_) = MovementPatternValidator::validate_time_delta(time_delta_s) {
            return current_position;
        }
        
        // Create a thread-local coordinate transformer
        thread_local! {
            static TRANSFORMER: std::cell::RefCell<MovementCoordinateTransformer> = 
                std::cell::RefCell::new(MovementCoordinateTransformer::new());
        }
        
        let result = TRANSFORMER.with(|transformer| {
            let mut transformer = transformer.borrow_mut();
            
            match movement_pattern {
                MovementPattern::Stationary => Ok(current_position),
                
                MovementPattern::Linear { speed_m_per_s, bearing_deg } => {
                    transformer.apply_linear_movement(
                        current_position,
                        *speed_m_per_s,
                        *bearing_deg,
                        time_delta_s,
                    )
                }
                
                MovementPattern::Circular { radius_m, period_s } => {
                    let elapsed = SystemTime::now().duration_since(loop_start_time)
                        .unwrap_or_default().as_secs_f64();
                    
                    transformer.apply_circular_movement(
                        initial_position,
                        *radius_m,
                        *period_s,
                        elapsed,
                    )
                }
                
                MovementPattern::Random { max_speed_m_per_s } => {
                    transformer.apply_random_movement(
                        current_position,
                        *max_speed_m_per_s,
                        time_delta_s,
                    )
                }
            }
        });
        
        // Return the new position or current position if there was an error
        result.unwrap_or(current_position)
    }
}

/// Status information for a virtual beacon
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VirtualBeaconStatus {
    pub id: Uuid,
    pub position: GeodeticPosition,
    pub is_running: bool,
    pub movement_pattern: MovementPattern,
    pub stats: VirtualBeaconStats,
    pub config: BeaconConfig,
}

/// Statistics for virtual beacon operation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VirtualBeaconStats {
    pub messages_sent: u64,
    #[serde(
        serialize_with = "serialize_optional_system_time",
        deserialize_with = "deserialize_optional_system_time"
    )]
    pub last_transmission: Option<SystemTime>,
    pub uptime: Duration,
    pub transmission_failures: u32,
}

impl VirtualBeaconStats {
    pub fn new() -> Self {
        Self {
            messages_sent: 0,
            last_transmission: None,
            uptime: Duration::new(0, 0),
            transmission_failures: 0,
        }
    }
}

// Custom serialization for SystemTime
fn serialize_optional_system_time<S>(
    time: &Option<SystemTime>,
    serializer: S,
) -> Result<S::Ok, S::Error>
where
    S: Serializer,
{
    match time {
        Some(t) => {
            let duration = t.duration_since(UNIX_EPOCH)
                .map_err(serde::ser::Error::custom)?;
            serializer.serialize_some(&duration.as_secs())
        }
        None => serializer.serialize_none(),
    }
}

fn deserialize_optional_system_time<'de, D>(
    deserializer: D,
) -> Result<Option<SystemTime>, D::Error>
where
    D: Deserializer<'de>,
{
    let opt_secs: Option<u64> = Option::deserialize(deserializer)?;
    match opt_secs {
        Some(secs) => {
            let time = UNIX_EPOCH + Duration::from_secs(secs);
            Ok(Some(time))
        }
        None => Ok(None),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use shared_positioning::{BeaconConfig, GeodeticPosition};
    use uuid::Uuid;
    use tokio::time::{sleep, Duration as TokioDuration};
    use crate::{VirtualChannel, MovementPattern};

    fn create_test_beacon_config() -> BeaconConfig {
        BeaconConfig::new(Uuid::new_v4())
    }

    fn create_test_position() -> GeodeticPosition {
        GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        }
    }

    #[tokio::test]
    async fn test_virtual_beacon_creation() {
        let beacon_id = Uuid::new_v4();
        let config = create_test_beacon_config();
        let position = create_test_position();
        let channel = VirtualChannel::new("test_channel");

        let beacon = VirtualBeacon::new(beacon_id, config, position, channel).unwrap();

        assert_eq!(beacon.id(), beacon_id);
        assert_eq!(beacon.position().latitude, position.latitude);
        assert_eq!(beacon.position().longitude, position.longitude);
        assert_eq!(beacon.position().depth, position.depth);
        assert!(!beacon.is_running());
    }

    #[tokio::test]
    async fn test_virtual_beacon_status() {
        let beacon_id = Uuid::new_v4();
        let config = create_test_beacon_config();
        let position = create_test_position();
        let channel = VirtualChannel::new("test_channel");

        let beacon = VirtualBeacon::new(beacon_id, config, position, channel).unwrap();
        let status = beacon.get_status();

        assert_eq!(status.id, beacon_id);
        assert_eq!(status.position.latitude, position.latitude);
        assert!(!status.is_running);
        assert!(matches!(status.movement_pattern, MovementPattern::Stationary));
        assert_eq!(status.stats.messages_sent, 0);
        assert!(status.stats.last_transmission.is_none());
    }

    #[tokio::test]
    async fn test_virtual_beacon_position_update() {
        let beacon_id = Uuid::new_v4();
        let config = create_test_beacon_config();
        let initial_position = create_test_position();
        let channel = VirtualChannel::new("test_channel");

        let mut beacon = VirtualBeacon::new(beacon_id, config, initial_position, channel).unwrap();

        let new_position = GeodeticPosition {
            latitude: 33.456,
            longitude: 46.789,
            depth: 15.0,
        };

        beacon.update_position(new_position).unwrap();
        assert_eq!(beacon.position().latitude, new_position.latitude);
        assert_eq!(beacon.position().longitude, new_position.longitude);
        assert_eq!(beacon.position().depth, new_position.depth);
    }

    #[tokio::test]
    async fn test_virtual_beacon_movement_pattern_update() {
        let beacon_id = Uuid::new_v4();
        let config = create_test_beacon_config();
        let position = create_test_position();
        let channel = VirtualChannel::new("test_channel");

        let mut beacon = VirtualBeacon::new(beacon_id, config, position, channel).unwrap();

        let linear_pattern = MovementPattern::Linear {
            speed_m_per_s: 1.5,
            bearing_deg: 45.0,
        };

        beacon.set_movement_pattern(linear_pattern.clone()).unwrap();
        let status = beacon.get_status();
        
        match status.movement_pattern {
            MovementPattern::Linear { speed_m_per_s, bearing_deg } => {
                assert_eq!(speed_m_per_s, 1.5);
                assert_eq!(bearing_deg, 45.0);
            }
            _ => panic!("Expected Linear movement pattern"),
        }
    }

    #[tokio::test]
    async fn test_virtual_beacon_start_stop() {
        let beacon_id = Uuid::new_v4();
        let config = create_test_beacon_config();
        let position = create_test_position();
        let channel = VirtualChannel::new("test_channel");

        let mut beacon = VirtualBeacon::new(beacon_id, config, position, channel).unwrap();

        // Test starting beacon
        assert!(!beacon.is_running());
        let task_handle = beacon.start().await.unwrap();
        assert!(beacon.is_running());

        // Let it run for a short time
        sleep(TokioDuration::from_millis(100)).await;

        // Test stopping beacon
        beacon.stop().unwrap();
        assert!(!beacon.is_running());

        // Clean up the task
        task_handle.abort();
    }

    #[tokio::test]
    async fn test_virtual_beacon_message_transmission() {
        let beacon_id = Uuid::new_v4();
        let config = create_test_beacon_config();
        let position = create_test_position();
        let channel = VirtualChannel::new("test_channel");
        let mut receiver = channel.subscribe();

        let mut beacon = VirtualBeacon::new(beacon_id, config, position, channel.clone()).unwrap();

        // Start the beacon
        let task_handle = beacon.start().await.unwrap();

        // Wait for at least one message to be transmitted
        sleep(TokioDuration::from_millis(200)).await;

        // Check if we received a message
        let received_message = tokio::time::timeout(
            TokioDuration::from_millis(100),
            receiver.recv()
        ).await;

        assert!(received_message.is_ok(), "Should have received a message");
        
        let message = received_message.unwrap().unwrap();
        assert_eq!(message.beacon_id, beacon_id);
        assert_eq!(message.position.latitude, position.latitude);
        assert_eq!(message.signal_quality, 255);
        assert!(!message.message_data.is_empty());

        // Stop the beacon
        beacon.stop().unwrap();
        task_handle.abort();

        // Verify message was logged in channel
        let message_count = channel.get_message_count().await;
        assert!(message_count > 0, "Channel should have logged messages");
    }

    #[tokio::test]
    async fn test_virtual_beacon_statistics_tracking() {
        let beacon_id = Uuid::new_v4();
        let config = create_test_beacon_config();
        let position = create_test_position();
        let channel = VirtualChannel::new("test_channel");

        let mut beacon = VirtualBeacon::new(beacon_id, config, position, channel).unwrap();

        // Start the beacon
        let task_handle = beacon.start().await.unwrap();

        // Let it run for a short time to send some messages
        sleep(TokioDuration::from_millis(200)).await;

        // Stop the beacon
        beacon.stop().unwrap();

        // Check statistics
        let status = beacon.get_status();
        assert!(status.stats.uptime.as_millis() > 0, "Uptime should be greater than 0");

        task_handle.abort();
    }

    #[tokio::test]
    async fn test_virtual_beacon_config_update() {
        let beacon_id = Uuid::new_v4();
        let config = create_test_beacon_config();
        let position = create_test_position();
        let channel = VirtualChannel::new("test_channel");

        let mut beacon = VirtualBeacon::new(beacon_id, config, position, channel).unwrap();

        let new_config = create_test_beacon_config();
        beacon.update_config(new_config.clone()).unwrap();

        let status = beacon.get_status();
        assert_eq!(status.config.beacon_id, new_config.beacon_id);
    }

    #[tokio::test]
    async fn test_movement_pattern_calculations() {
        // Test stationary movement
        let current_pos = GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        };
        
        let updated_pos = VirtualBeacon::update_position_from_movement(
            current_pos,
            current_pos,
            &MovementPattern::Stationary,
            std::time::SystemTime::now(),
            1000, // 1 second interval
            0,
        );
        
        assert_eq!(updated_pos.latitude, current_pos.latitude);
        assert_eq!(updated_pos.longitude, current_pos.longitude);
        assert_eq!(updated_pos.depth, current_pos.depth);

        // Test linear movement (basic test - just verify it changes position)
        let linear_pattern = MovementPattern::Linear {
            speed_m_per_s: 1.0,
            bearing_deg: 0.0, // North
        };
        
        let updated_pos = VirtualBeacon::update_position_from_movement(
            current_pos,
            current_pos,
            &linear_pattern,
            std::time::SystemTime::now(),
            1000, // 1 second interval
            0,
        );
        
        // Should move north (increase latitude)
        assert!(updated_pos.latitude > current_pos.latitude);
        assert_eq!(updated_pos.longitude, current_pos.longitude);
        assert_eq!(updated_pos.depth, current_pos.depth);
    }
}