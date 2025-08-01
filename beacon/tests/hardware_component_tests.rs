use shared_positioning::{HardwareComponent, HardwareFaultType, BeaconError};

#[test]
fn test_underwater_transceiver_variant() {
    // Test that UnderwaterTransceiver variant exists and can be used
    let component = HardwareComponent::UnderwaterTransceiver;
    
    // Test that it can be used in BeaconError
    let error = BeaconError::HardwareError {
        component,
        fault_type: HardwareFaultType::SignalProcessingFailure,
        diagnostic_data: vec![],
        recovery_possible: false,
    };
    
    // Verify the error can be formatted (tests Debug trait)
    let error_string = format!("{:?}", error);
    assert!(error_string.contains("UnderwaterTransceiver"));
    assert!(error_string.contains("SignalProcessingFailure"));
}

#[test]
fn test_signal_processing_failure_variant() {
    // Test that SignalProcessingFailure variant exists and can be used
    let fault_type = HardwareFaultType::SignalProcessingFailure;
    
    // Test that it can be used in BeaconError
    let error = BeaconError::HardwareError {
        component: HardwareComponent::Transceiver,
        fault_type,
        diagnostic_data: vec![],
        recovery_possible: true,
    };
    
    // Verify the error can be formatted (tests Debug trait)
    let error_string = format!("{:?}", error);
    assert!(error_string.contains("SignalProcessingFailure"));
}

#[test]
fn test_additional_fault_types() {
    // Test SensorFailure variant
    let sensor_fault = HardwareFaultType::SensorFailure;
    let error1 = BeaconError::HardwareError {
        component: HardwareComponent::TemperatureSensor,
        fault_type: sensor_fault,
        diagnostic_data: vec![],
        recovery_possible: true,
    };
    
    // Test ConnectionFailure variant
    let connection_fault = HardwareFaultType::ConnectionFailure;
    let error2 = BeaconError::HardwareError {
        component: HardwareComponent::CommunicationModule,
        fault_type: connection_fault,
        diagnostic_data: vec![],
        recovery_possible: false,
    };
    
    // Verify both errors can be formatted
    let error1_string = format!("{:?}", error1);
    let error2_string = format!("{:?}", error2);
    
    assert!(error1_string.contains("SensorFailure"));
    assert!(error2_string.contains("ConnectionFailure"));
}