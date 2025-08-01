#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Duration;

    #[test]
    fn test_transmission_status_default() {
        let status = TransmissionStatus::default();
        assert!(!status.is_transmitting);
        assert_eq!(status.transmission_count, 0);
        assert_eq!(status.transmission_failures, 0);
        assert_eq!(status.current_power_level, 128);
        assert_eq!(status.transmission_stats.successful_transmissions, 0);
    }

    #[test]
    fn test_transmission_stats_record_success() {
        let mut stats = TransmissionStats::default();
        
        stats.record_successful_transmission(100, 50, 200);
        
        assert_eq!(stats.successful_transmissions, 1);
        assert_eq!(stats.total_bytes_transmitted, 100);
        assert_eq!(stats.average_transmission_time_ms, 50.0);
        assert_eq!(stats.power_level_history.len(), 1);
        assert_eq!(stats.power_level_history[0], 200);
    }

    #[test]
    fn test_transmission_stats_record_failure() {
        let mut stats = TransmissionStats::default();
        let error = CommError::Timeout { timeout_ms: 1000 };
        
        stats.record_failed_transmission(error.clone());
        
        assert_eq!(stats.failed_transmissions, 1);
        assert_eq!(stats.last_error, Some(error));
        assert_eq!(stats.error_count_by_type.get("Timeout"), Some(&1));
    }

    #[test]
    fn test_transmission_stats_success_rate() {
        let mut stats = TransmissionStats::default();
        
        // Record some successes and failures
        stats.record_successful_transmission(100, 50, 200);
        stats.record_successful_transmission(100, 50, 200);
        stats.record_failed_transmission(CommError::Timeout { timeout_ms: 1000 });
        
        let success_rate = stats.get_success_rate();
        assert!((success_rate - 0.6667).abs() < 0.001); // 2/3 ≈ 0.6667
    }

    #[test]
    fn test_environmental_conditions_default() {
        let conditions = EnvironmentalConditions::default();
        assert_eq!(conditions.salinity_ppt, Some(35.0));
        assert_eq!(conditions.water_temperature_c, None);
    }

    #[test]
    fn test_mock_transceiver_transmission() {
        let mut transceiver = MockTransceiver::new(1);
        let config = TransceiverConfig::default();
        transceiver.configure(config).unwrap();
        
        let test_data = b"test message";
        let result = transceiver.transmit_message(test_data);
        
        assert!(result.is_ok());
        
        let status = transceiver.get_transmission_status();
        assert_eq!(status.transmission_count, 1);
        assert_eq!(status.transmission_stats.successful_transmissions, 1);
        assert_eq!(status.transmission_stats.total_bytes_transmitted, test_data.len() as u64);
    }

    #[test]
    fn test_mock_transceiver_transmission_with_errors() {
        let mut transceiver = MockTransceiver::new(1);
        let config = TransceiverConfig::default();
        transceiver.configure(config).unwrap();
        transceiver.enable_error_simulation(1.0); // 100% error rate
        
        let test_data = b"test message";
        let result = transceiver.transmit_message(test_data);
        
        assert!(result.is_err());
        
        let status = transceiver.get_transmission_status();
        assert_eq!(status.transmission_failures, 1);
        assert_eq!(status.transmission_stats.failed_transmissions, 1);
        assert!(status.transmission_stats.last_error.is_some());
    }

    #[test]
    fn test_transmission_power_control() {
        let mut transceiver = MockTransceiver::new(1);
        let config = TransceiverConfig::default();
        transceiver.configure(config).unwrap();
        
        let result = transceiver.set_transmission_power(200);
        assert!(result.is_ok());
        
        let status = transceiver.get_transmission_status();
        assert_eq!(status.current_power_level, 200);
    }

    #[test]
    fn test_adaptive_transmission_power_depth() {
        let mut transceiver = MockTransceiver::new(1);
        let config = TransceiverConfig::default();
        transceiver.configure(config).unwrap();
        
        let mut conditions = EnvironmentalConditions::default();
        conditions.depth_m = Some(50.0); // 50 meter depth
        
        let power_level = transceiver.adapt_transmission_power(&conditions).unwrap();
        
        // Power should be increased for deeper water
        assert!(power_level > 128); // Should be higher than default mid-range
        
        let status = transceiver.get_transmission_status();
        assert_eq!(status.current_power_level, power_level);
    }

    #[test]
    fn test_adaptive_transmission_power_temperature() {
        let mut transceiver = MockTransceiver::new(1);
        let config = TransceiverConfig::default();
        transceiver.configure(config).unwrap();
        
        // Test cold water (better transmission)
        let mut conditions = EnvironmentalConditions::default();
        conditions.water_temperature_c = Some(5.0);
        
        let cold_power = transceiver.adapt_transmission_power(&conditions).unwrap();
        
        // Test warm water (worse transmission)
        conditions.water_temperature_c = Some(30.0);
        let warm_power = transceiver.adapt_transmission_power(&conditions).unwrap();
        
        // Warm water should require more power than cold water
        assert!(warm_power > cold_power);
    }

    #[test]
    fn test_adaptive_transmission_power_noise() {
        let mut transceiver = MockTransceiver::new(1);
        let config = TransceiverConfig::default();
        transceiver.configure(config).unwrap();
        
        // Test high noise environment
        let mut conditions = EnvironmentalConditions::default();
        conditions.noise_level_db = Some(90.0);
        
        let high_noise_power = transceiver.adapt_transmission_power(&conditions).unwrap();
        
        // Test low noise environment
        conditions.noise_level_db = Some(30.0);
        let low_noise_power = transceiver.adapt_transmission_power(&conditions).unwrap();
        
        // High noise should require more power than low noise
        assert!(high_noise_power > low_noise_power);
    }

    #[test]
    fn test_adaptive_transmission_power_bounds() {
        let mut transceiver = MockTransceiver::new(1);
        let config = TransceiverConfig::default();
        transceiver.configure(config).unwrap();
        
        // Test extreme conditions that would push power beyond limits
        let mut conditions = EnvironmentalConditions::default();
        conditions.depth_m = Some(1000.0); // Very deep
        conditions.noise_level_db = Some(120.0); // Very noisy
        conditions.signal_attenuation_factor = Some(2.0); // High attenuation
        
        let power_level = transceiver.adapt_transmission_power(&conditions).unwrap();
        
        // Power should be clamped to maximum (255)
        assert_eq!(power_level, 255);
    }

    #[test]
    fn test_transmission_timed() {
        let mut transceiver = MockTransceiver::new(1);
        let config = TransceiverConfig::default();
        transceiver.configure(config).unwrap();
        
        let test_data = b"test message";
        let duration = transceiver.transmit_message_timed(test_data).unwrap();
        
        // Should have some measurable duration
        assert!(duration.as_millis() > 0);
        
        let status = transceiver.get_transmission_status();
        assert_eq!(status.transmission_count, 1);
        assert!(status.transmission_stats.average_transmission_time_ms > 0.0);
    }

    #[test]
    fn test_transmission_interval_tracking() {
        let mut stats = TransmissionStats::default();
        
        stats.record_transmission_interval(1000);
        stats.record_transmission_interval(1500);
        stats.record_transmission_interval(800);
        
        let avg_interval = stats.get_average_interval_ms().unwrap();
        assert!((avg_interval - 1100.0).abs() < 0.1); // (1000+1500+800)/3 = 1100
    }

    #[test]
    fn test_serial_transceiver_transmission() {
        let mut transceiver = SerialTransceiver::new(2, "/dev/ttyUSB0".to_string());
        transceiver.open().unwrap();
        
        let test_data = b"test message";
        let result = transceiver.transmit_message(test_data);
        
        assert!(result.is_ok());
        
        let status = transceiver.get_transmission_status();
        assert_eq!(status.transmission_count, 1);
        assert_eq!(status.transmission_stats.successful_transmissions, 1);
    }

    #[test]
    fn test_power_level_history_limit() {
        let mut stats = TransmissionStats::default();
        
        // Add more than 100 power level entries
        for i in 0..150 {
            stats.record_successful_transmission(100, 50, (i % 256) as u8);
        }
        
        // History should be limited to 100 entries
        assert_eq!(stats.power_level_history.len(), 100);
        assert_eq!(stats.successful_transmissions, 150);
    }

    #[test]
    fn test_transmission_interval_history_limit() {
        let mut stats = TransmissionStats::default();
        
        // Add more than 50 interval entries
        for i in 0..80 {
            stats.record_transmission_interval(1000 + i);
        }
        
        // History should be limited to 50 entries
        assert_eq!(stats.transmission_intervals_ms.len(), 50);
    }

    #[test]
    fn test_error_count_by_type() {
        let mut stats = TransmissionStats::default();
        
        // Record different types of errors
        stats.record_failed_transmission(CommError::Timeout { timeout_ms: 1000 });
        stats.record_failed_transmission(CommError::Timeout { timeout_ms: 2000 });
        stats.record_failed_transmission(CommError::ConnectionFailed { details: "test".to_string() });
        stats.record_failed_transmission(CommError::HardwareError { error_code: 1, details: "test".to_string() });
        
        assert_eq!(stats.error_count_by_type.get("Timeout"), Some(&2));
        assert_eq!(stats.error_count_by_type.get("ConnectionFailed"), Some(&1));
        assert_eq!(stats.error_count_by_type.get("HardwareError"), Some(&1));
    }
}