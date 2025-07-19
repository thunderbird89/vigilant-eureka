//! C-compatible API for integration with C-based systems
//! 
//! This module provides a C-compatible interface that can be used from C/C++
//! applications and other languages that support C FFI.

use crate::api::types::{ApiError, PositionRequest, PositionResponse, CompactPosition, OutputFormat, GeometryQuality};
use crate::api::blocking::BlockingPositioningApi;
use crate::core::Position;
use crate::hardware::{TransceiverConfig};
use crate::hardware::transceiver::InterfaceType;
use std::ffi::{CStr, CString};
use std::os::raw::{c_char, c_int, c_uint, c_float, c_double};
use std::ptr;
use std::collections::HashMap;

/// C-compatible error codes
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CApiError {
    Success = 0,
    InsufficientAnchors = 1,
    DegenerateGeometry = 2,
    StaleData = 3,
    ComputationFailure = 4,
    HardwareError = 5,
    ConfigurationError = 6,
    NotInitialized = 7,
    Timeout = 8,
    InvalidRequest = 9,
    NullPointer = 10,
    InvalidHandle = 11,
}

impl From<ApiError> for CApiError {
    fn from(error: ApiError) -> Self {
        match error {
            ApiError::InsufficientAnchors { .. } => CApiError::InsufficientAnchors,
            ApiError::DegenerateGeometry { .. } => CApiError::DegenerateGeometry,
            ApiError::StaleData { .. } => CApiError::StaleData,
            ApiError::ComputationFailure { .. } => CApiError::ComputationFailure,
            ApiError::HardwareError { .. } => CApiError::HardwareError,
            ApiError::ConfigurationError { .. } => CApiError::ConfigurationError,
            ApiError::NotInitialized => CApiError::NotInitialized,
            ApiError::Timeout { .. } => CApiError::Timeout,
            ApiError::InvalidRequest { .. } => CApiError::InvalidRequest,
        }
    }
}

/// C-compatible position structure
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CPosition {
    pub latitude: c_double,
    pub longitude: c_double,
    pub depth: c_double,
}

impl From<Position> for CPosition {
    fn from(pos: Position) -> Self {
        Self {
            latitude: pos.lat,
            longitude: pos.lon,
            depth: pos.depth,
        }
    }
}

impl From<CPosition> for Position {
    fn from(pos: CPosition) -> Self {
        Self {
            lat: pos.latitude,
            lon: pos.longitude,
            depth: pos.depth,
        }
    }
}

/// C-compatible position response structure
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CPositionResponse {
    pub position: CPosition,
    pub accuracy_estimate: c_float,
    pub timestamp_ms: u64,
    pub anchor_count: u8,
    pub geometry_quality: u8,
    pub computation_time_us: u32,
    pub sequence_number: u32,
}

impl From<PositionResponse> for CPositionResponse {
    fn from(resp: PositionResponse) -> Self {
        Self {
            position: resp.position.into(),
            accuracy_estimate: resp.accuracy_estimate,
            timestamp_ms: resp.timestamp_ms,
            anchor_count: resp.anchor_count,
            geometry_quality: resp.geometry_quality as u8,
            computation_time_us: resp.computation_time_us,
            sequence_number: resp.sequence_number,
        }
    }
}

/// C-compatible transceiver configuration
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct CTransceiverConfig {
    pub id: u8,
    pub interface_type: u8, // 0=Serial, 1=I2C, 2=Mock
    pub baud_rate: u32,
    pub i2c_address: u8,
    pub read_timeout_ms: u32,
    pub write_timeout_ms: u32,
}

/// Opaque handle for positioning system instance
pub type CPositioningHandle = *mut std::ffi::c_void;

/// Global storage for positioning system instances
static mut POSITIONING_INSTANCES: Option<HashMap<usize, BlockingPositioningApi>> = None;
static mut NEXT_HANDLE_ID: usize = 1;

/// Initialize the C API (call once at startup)
#[no_mangle]
pub extern "C" fn ups_init() -> CApiError {
    unsafe {
        if POSITIONING_INSTANCES.is_none() {
            POSITIONING_INSTANCES = Some(HashMap::new());
        }
    }
    CApiError::Success
}

/// Create a new positioning system instance
#[no_mangle]
pub extern "C" fn ups_create() -> CPositioningHandle {
    unsafe {
        if let Some(ref mut instances) = POSITIONING_INSTANCES {
            let handle_id = NEXT_HANDLE_ID;
            NEXT_HANDLE_ID += 1;
            
            let api = BlockingPositioningApi::default();
            instances.insert(handle_id, api);
            
            handle_id as CPositioningHandle
        } else {
            ptr::null_mut()
        }
    }
}

/// Initialize a positioning system instance
#[no_mangle]
pub extern "C" fn ups_initialize(handle: CPositioningHandle) -> CApiError {
    if handle.is_null() {
        return CApiError::NullPointer;
    }
    
    unsafe {
        if let Some(ref mut instances) = POSITIONING_INSTANCES {
            let handle_id = handle as usize;
            if let Some(api) = instances.get_mut(&handle_id) {
                match api.initialize() {
                    Ok(_) => CApiError::Success,
                    Err(e) => e.into(),
                }
            } else {
                CApiError::InvalidHandle
            }
        } else {
            CApiError::NotInitialized
        }
    }
}

/// Add a transceiver to the positioning system
#[no_mangle]
pub extern "C" fn ups_add_transceiver(
    handle: CPositioningHandle,
    config: CTransceiverConfig,
) -> CApiError {
    if handle.is_null() {
        return CApiError::NullPointer;
    }
    
    unsafe {
        if let Some(ref mut instances) = POSITIONING_INSTANCES {
            let handle_id = handle as usize;
            if let Some(api) = instances.get_mut(&handle_id) {
                let interface_type = match config.interface_type {
                    0 => InterfaceType::Serial,
                    1 => InterfaceType::I2C,
                    2 => InterfaceType::Mock,
                    _ => return CApiError::ConfigurationError,
                };
                
                let transceiver_config = TransceiverConfig {
                    id: config.id,
                    interface: interface_type,
                    baud_rate: if config.baud_rate > 0 { Some(config.baud_rate) } else { None },
                    i2c_address: if config.i2c_address > 0 { Some(config.i2c_address) } else { None },
                    read_timeout_ms: config.read_timeout_ms,
                    write_timeout_ms: config.write_timeout_ms,
                    ..Default::default()
                };
                
                // Create appropriate transceiver based on type
                let transceiver: Box<dyn crate::hardware::TransceiverInterface> = match interface_type {
                    InterfaceType::Mock => {
                        Box::new(crate::hardware::mock::MockTransceiver::new(transceiver_config.id))
                    }
                    InterfaceType::Serial => {
                        // For C API, we'll use mock transceivers as placeholders
                        // Real implementations would create actual serial transceivers
                        Box::new(crate::hardware::mock::MockTransceiver::new(transceiver_config.id))
                    }
                    InterfaceType::I2C => {
                        // For C API, we'll use mock transceivers as placeholders
                        // Real implementations would create actual I2C transceivers
                        Box::new(crate::hardware::mock::MockTransceiver::new(transceiver_config.id))
                    }
                };
                
                match api.add_transceiver(transceiver) {
                    Ok(_) => CApiError::Success,
                    Err(e) => e.into(),
                }
            } else {
                CApiError::InvalidHandle
            }
        } else {
            CApiError::NotInitialized
        }
    }
}

/// Get current position (blocking)
#[no_mangle]
pub extern "C" fn ups_get_position(
    handle: CPositioningHandle,
    position: *mut CPositionResponse,
) -> CApiError {
    if handle.is_null() || position.is_null() {
        return CApiError::NullPointer;
    }
    
    unsafe {
        if let Some(ref mut instances) = POSITIONING_INSTANCES {
            let handle_id = handle as usize;
            if let Some(api) = instances.get_mut(&handle_id) {
                match api.get_position() {
                    Ok(resp) => {
                        *position = resp.into();
                        CApiError::Success
                    }
                    Err(e) => e.into(),
                }
            } else {
                CApiError::InvalidHandle
            }
        } else {
            CApiError::NotInitialized
        }
    }
}

/// Get position with timeout
#[no_mangle]
pub extern "C" fn ups_get_position_timeout(
    handle: CPositioningHandle,
    timeout_ms: u32,
    min_anchors: u8,
    position: *mut CPositionResponse,
) -> CApiError {
    if handle.is_null() || position.is_null() {
        return CApiError::NullPointer;
    }
    
    unsafe {
        if let Some(ref mut instances) = POSITIONING_INSTANCES {
            let handle_id = handle as usize;
            if let Some(api) = instances.get_mut(&handle_id) {
                let request = PositionRequest {
                    timeout_ms,
                    min_anchors,
                    ..Default::default()
                };
                
                match api.get_position_with_request(&request) {
                    Ok(resp) => {
                        *position = resp.into();
                        CApiError::Success
                    }
                    Err(e) => e.into(),
                }
            } else {
                CApiError::InvalidHandle
            }
        } else {
            CApiError::NotInitialized
        }
    }
}

/// Get position in compact binary format
#[no_mangle]
pub extern "C" fn ups_get_position_compact(
    handle: CPositioningHandle,
    compact_position: *mut CompactPosition,
) -> CApiError {
    if handle.is_null() || compact_position.is_null() {
        return CApiError::NullPointer;
    }
    
    unsafe {
        if let Some(ref mut instances) = POSITIONING_INSTANCES {
            let handle_id = handle as usize;
            if let Some(api) = instances.get_mut(&handle_id) {
                match api.get_position() {
                    Ok(resp) => {
                        *compact_position = CompactPosition::from_position(&resp);
                        CApiError::Success
                    }
                    Err(e) => e.into(),
                }
            } else {
                CApiError::InvalidHandle
            }
        } else {
            CApiError::NotInitialized
        }
    }
}

/// Get system statistics
#[no_mangle]
pub extern "C" fn ups_get_statistics(
    handle: CPositioningHandle,
    positions_calculated: *mut u32,
    error_count: *mut u32,
    avg_computation_time_us: *mut u32,
    uptime_ms: *mut u64,
) -> CApiError {
    if handle.is_null() {
        return CApiError::NullPointer;
    }
    
    unsafe {
        if let Some(ref mut instances) = POSITIONING_INSTANCES {
            let handle_id = handle as usize;
            if let Some(api) = instances.get_mut(&handle_id) {
                let state = api.get_system_state();
                
                if !positions_calculated.is_null() {
                    *positions_calculated = state.positions_calculated;
                }
                if !error_count.is_null() {
                    *error_count = state.error_count;
                }
                if !avg_computation_time_us.is_null() {
                    *avg_computation_time_us = state.avg_computation_time_us;
                }
                if !uptime_ms.is_null() {
                    *uptime_ms = state.uptime_ms;
                }
                
                CApiError::Success
            } else {
                CApiError::InvalidHandle
            }
        } else {
            CApiError::NotInitialized
        }
    }
}

/// Reset system statistics
#[no_mangle]
pub extern "C" fn ups_reset_statistics(handle: CPositioningHandle) -> CApiError {
    if handle.is_null() {
        return CApiError::NullPointer;
    }
    
    unsafe {
        if let Some(ref mut instances) = POSITIONING_INSTANCES {
            let handle_id = handle as usize;
            if let Some(api) = instances.get_mut(&handle_id) {
                api.reset_statistics();
                CApiError::Success
            } else {
                CApiError::InvalidHandle
            }
        } else {
            CApiError::NotInitialized
        }
    }
}

/// Destroy a positioning system instance
#[no_mangle]
pub extern "C" fn ups_destroy(handle: CPositioningHandle) -> CApiError {
    if handle.is_null() {
        return CApiError::NullPointer;
    }
    
    unsafe {
        if let Some(ref mut instances) = POSITIONING_INSTANCES {
            let handle_id = handle as usize;
            if instances.remove(&handle_id).is_some() {
                CApiError::Success
            } else {
                CApiError::InvalidHandle
            }
        } else {
            CApiError::NotInitialized
        }
    }
}

/// Cleanup the C API (call once at shutdown)
#[no_mangle]
pub extern "C" fn ups_cleanup() -> CApiError {
    unsafe {
        POSITIONING_INSTANCES = None;
        NEXT_HANDLE_ID = 1;
    }
    CApiError::Success
}

/// Get error message for error code
#[no_mangle]
pub extern "C" fn ups_get_error_message(error_code: CApiError) -> *const c_char {
    let message = match error_code {
        CApiError::Success => "Success",
        CApiError::InsufficientAnchors => "Insufficient anchors for positioning",
        CApiError::DegenerateGeometry => "Poor anchor geometry",
        CApiError::StaleData => "Stale anchor data",
        CApiError::ComputationFailure => "Computation failure",
        CApiError::HardwareError => "Hardware communication error",
        CApiError::ConfigurationError => "Configuration error",
        CApiError::NotInitialized => "System not initialized",
        CApiError::Timeout => "Operation timeout",
        CApiError::InvalidRequest => "Invalid request",
        CApiError::NullPointer => "Null pointer provided",
        CApiError::InvalidHandle => "Invalid handle",
    };
    
    // Note: In a real implementation, you'd want to manage string lifetimes properly
    // This is a simplified version for demonstration
    message.as_ptr() as *const c_char
}

/// Get library version information
#[no_mangle]
pub extern "C" fn ups_get_version(
    major: *mut c_int,
    minor: *mut c_int,
    patch: *mut c_int,
) -> CApiError {
    unsafe {
        if !major.is_null() {
            *major = 1;
        }
        if !minor.is_null() {
            *minor = 0;
        }
        if !patch.is_null() {
            *patch = 0;
        }
    }
    CApiError::Success
}