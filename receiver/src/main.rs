use nalgebra::{Matrix3, Vector3};
use serde::Deserialize;
use std::f64::consts::PI;

// Import shared components from shared-positioning library
use shared_positioning::{
    MessageParser, TransceiverInterface, CoordinateSystemManager, 
    PositioningError, ErrorRecoveryManager, SystemConfig
};

pub mod accuracy_validation;
pub mod performance_monitor;
pub mod optimization_cache;
pub mod message_parser_demo;
pub mod transceiver_demo;
pub mod advanced_positioning;
pub mod graceful_degradation;
pub mod coordinate_system_demo;

const SPEED_OF_SOUND_WATER: f64 = 1500.0; // m/s

// Fixed-point arithmetic support for microcontrollers without FPU
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FixedPoint32 {
    value: i32,
    scale: u8, // Number of fractional bits
}

impl FixedPoint32 {
    pub fn new(value: f64, scale: u8) -> Self {
        let multiplier = 1i32 << scale;
        Self {
            value: (value * multiplier as f64) as i32,
            scale,
        }
    }
    
    pub fn to_f64(self) -> f64 {
        let divisor = 1i32 << self.scale;
        self.value as f64 / divisor as f64
    }
    
    pub fn add(self, other: Self) -> Self {
        assert_eq!(self.scale, other.scale);
        Self {
            value: self.value + other.value,
            scale: self.scale,
        }
    }
    
    pub fn sub(self, other: Self) -> Self {
        assert_eq!(self.scale, other.scale);
        Self {
            value: self.value - other.value,
            scale: self.scale,
        }
    }
    
    pub fn mul(self, other: Self) -> Self {
        assert_eq!(self.scale, other.scale);
        let result = (self.value as i64 * other.value as i64) >> self.scale;
        Self {
            value: result as i32,
            scale: self.scale,
        }
    }
}

// Memory-optimized position representation for embedded systems
#[derive(Debug, Clone, Copy)]
pub struct LocalPosition {
    pub east_mm: i32,   // millimeters east
    pub north_mm: i32,  // millimeters north  
    pub down_mm: i32,   // millimeters down
}

impl LocalPosition {
    pub fn new(east_m: f64, north_m: f64, down_m: f64) -> Self {
        Self {
            east_mm: (east_m * 1000.0) as i32,
            north_mm: (north_m * 1000.0) as i32,
            down_mm: (down_m * 1000.0) as i32,
        }
    }
    
    pub fn to_meters(&self) -> (f64, f64, f64) {
        (
            self.east_mm as f64 / 1000.0,
            self.north_mm as f64 / 1000.0,
            self.down_mm as f64 / 1000.0,
        )
    }
    
    pub fn to_vector3(&self) -> Vector3<f64> {
        let (east, north, down) = self.to_meters();
        Vector3::new(east, north, down)
    }
}

// Packed structure for minimal memory footprint
#[repr(C, packed)]
#[derive(Debug, Clone, Copy)]
pub struct CompactAnchorMessage {
    pub anchor_id: u16,
    pub timestamp_ms: u32,      // Relative timestamp (saves 4 bytes vs u64)
    pub lat_micro_deg: i32,     // Latitude * 1e6 (saves 4 bytes vs f64)
    pub lon_micro_deg: i32,     // Longitude * 1e6 (saves 4 bytes vs f64)
    pub depth_mm: u16,          // Depth in millimeters (saves 6 bytes vs f64)
    pub quality: u8,            // Signal quality indicator
}

// Global base timestamp for relative timestamp calculations
static mut BASE_TIMESTAMP_MS: u64 = 0;

impl CompactAnchorMessage {
    pub fn set_base_timestamp(base_ms: u64) {
        unsafe {
            BASE_TIMESTAMP_MS = base_ms;
        }
    }
    
    pub fn new(anchor_id: u16, timestamp_ms: u64, lat: f64, lon: f64, depth: f64, quality: u8) -> Self {
        let relative_timestamp = unsafe {
            if BASE_TIMESTAMP_MS == 0 {
                BASE_TIMESTAMP_MS = timestamp_ms;
            }
            (timestamp_ms.saturating_sub(BASE_TIMESTAMP_MS)) as u32
        };
        
        Self {
            anchor_id,
            timestamp_ms: relative_timestamp,
            lat_micro_deg: (lat * 1_000_000.0) as i32,
            lon_micro_deg: (lon * 1_000_000.0) as i32,
            depth_mm: (depth * 1000.0) as u16,
            quality,
        }
    }
    
    pub fn get_position(&self) -> Position {
        Position {
            lat: self.lat_micro_deg as f64 / 1_000_000.0,
            lon: self.lon_micro_deg as f64 / 1_000_000.0,
            depth: self.depth_mm as f64 / 1000.0,
        }
    }
    
    pub fn get_timestamp(&self) -> u64 {
        unsafe {
            BASE_TIMESTAMP_MS + self.timestamp_ms as u64
        }
    }
}

// Circular buffer for anchor message storage with fixed size
#[derive(Debug)]
pub struct AnchorBuffer<const N: usize> {
    messages: [Option<CompactAnchorMessage>; N],
    head: usize,
    count: usize,
}

impl<const N: usize> AnchorBuffer<N> {
    pub fn new() -> Self {
        Self {
            messages: [None; N],
            head: 0,
            count: 0,
        }
    }
    
    pub fn push(&mut self, message: CompactAnchorMessage) {
        self.messages[self.head] = Some(message);
        self.head = (self.head + 1) % N;
        if self.count < N {
            self.count += 1;
        }
    }
    
    pub fn get_recent_messages(&self, max_age_ms: u32, current_time_ms: u64) -> [Option<CompactAnchorMessage>; N] {
        let mut result = [None; N];
        let mut result_idx = 0;
        
        for i in 0..self.count {
            let idx = if self.head >= i + 1 {
                self.head - i - 1
            } else {
                N + self.head - i - 1
            };
            
            if let Some(msg) = self.messages[idx] {
                let age_ms = current_time_ms.saturating_sub(msg.get_timestamp());
                if age_ms <= max_age_ms as u64 && result_idx < N {
                    result[result_idx] = Some(msg);
                    result_idx += 1;
                }
            }
        }
        
        result
    }
    
    pub fn len(&self) -> usize {
        self.count
    }
    
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }
    
    pub fn clear(&mut self) {
        self.messages = [None; N];
        self.head = 0;
        self.count = 0;
    }
}

// Memory-optimized anchor configuration
#[derive(Debug, Clone)]
pub struct EmbeddedAnchorConfig {
    pub id: u16,
    pub position: CompactAnchorMessage,
    pub max_range_mm: u32,  // Range in millimeters to save space
    pub enabled: bool,
}

impl EmbeddedAnchorConfig {
    pub fn new(id: u16, lat: f64, lon: f64, depth: f64, max_range_m: f32) -> Self {
        Self {
            id,
            position: CompactAnchorMessage::new(id, 0, lat, lon, depth, 255),
            max_range_mm: (max_range_m * 1000.0) as u32,
            enabled: true,
        }
    }
    
    pub fn get_max_range_m(&self) -> f32 {
        self.max_range_mm as f32 / 1000.0
    }
}

// Fixed-size array for anchor configurations (replaces Vec)
pub type AnchorConfigArray<const N: usize> = [Option<EmbeddedAnchorConfig>; N];

// System configuration optimized for embedded systems
#[derive(Debug, Clone)]
pub struct EmbeddedSystemConfig {
    pub sound_speed_mm_per_ms: u16,     // Sound speed in mm/ms (saves space vs f32)
    pub max_anchor_age_ms: u32,
    pub min_anchors: u8,
    pub position_timeout_ms: u32,
    pub accuracy_threshold_mm: u16,     // Accuracy threshold in millimeters
}

impl EmbeddedSystemConfig {
    pub fn new() -> Self {
        Self {
            sound_speed_mm_per_ms: 1500,  // 1500 m/s = 1.5 mm/ms
            max_anchor_age_ms: 5000,
            min_anchors: 3,
            position_timeout_ms: 200,
            accuracy_threshold_mm: 2000,  // 2 meters
        }
    }
    
    pub fn get_sound_speed_m_per_s(&self) -> f64 {
        self.sound_speed_mm_per_ms as f64 / 1000.0 * 1000.0
    }
    
    pub fn get_accuracy_threshold_m(&self) -> f32 {
        self.accuracy_threshold_mm as f32 / 1000.0
    }
}

// Legacy structures for compatibility with existing JSON parsing
#[derive(Debug, Deserialize)]
struct AnchorsJson {
    anchors: Vec<Anchor>,
}

#[derive(Debug, Deserialize, Clone)]
pub struct Anchor {
    id: String,
    timestamp: u64, // milliseconds
    position: Position,
}

#[derive(Debug, Deserialize, Clone, Default, PartialEq)]
pub struct Position {
    lat: f64,  // degrees
    #[serde(rename = "long")]
    lon: f64,  // degrees
    depth: f64, // meters (positive down)
}

fn deg_to_rad(deg: f64) -> f64 {
    deg * PI / 180.0
}

/// Convert geographic coordinate to local tangent plane (END - East, North, Down) in metres relative to reference.
/// Returns (x_east, y_north, z_down) where z_down is positive downward.
fn geodetic_to_local(pos: &Position, reference: &Position) -> Vector3<f64> {
    // Approx meters per degree at reference latitude (valid for very small areas)
    let lat0_rad = deg_to_rad(reference.lat);
    let meters_per_deg_lat = 111_132.0; // roughly constant
    let meters_per_deg_lon = 111_320.0 * lat0_rad.cos();

    let dlat = pos.lat - reference.lat;
    let dlon = pos.lon - reference.lon;

    let north = dlat * meters_per_deg_lat;
    let east = dlon * meters_per_deg_lon;
    let down = pos.depth;

    Vector3::new(east, north, down)
}

/// Convert local tangent plane (END) coordinates back to geodetic position.
fn local_to_geodetic(local_pos: &Vector3<f64>, reference: &Position) -> Position {
    let lat0_rad = deg_to_rad(reference.lat);
    let meters_per_deg_lat = 111_132.0;
    let meters_per_deg_lon = 111_320.0 * lat0_rad.cos();

    let east = local_pos.x;
    let north = local_pos.y;
    let down = local_pos.z;

    let lat_est = reference.lat + north / meters_per_deg_lat;
    let lon_est = reference.lon + east / meters_per_deg_lon;
    let depth_est = down;

    Position {
        lat: lat_est,
        lon: lon_est,
        depth: depth_est,
    }
}

/// Performance-monitored trilateration with timing and memory tracking
pub fn trilaterate_with_monitoring(
    anchors: &[Anchor],
    receiver_time_ms: u64,
    monitor: Option<&mut performance_monitor::PerformanceMonitor>,
) -> Result<(Position, Vector3<f64>), String> {
    use performance_monitor::{PerformanceMetrics, MemoryTracker, TimingProfiler};
    use std::time::Instant;

    let start_time = Instant::now();
    let mut memory_tracker = MemoryTracker::new();
    let mut profiler = TimingProfiler::new();
    
    profiler.start();
    
    // Track memory usage for input data
    let input_memory = std::mem::size_of_val(anchors) + std::mem::size_of::<u64>();
    memory_tracker.track_allocation("input_data", input_memory);
    
    let result = trilaterate_internal(anchors, receiver_time_ms, &mut profiler, &mut memory_tracker);
    
    let total_time = start_time.elapsed();
    profiler.checkpoint("total_computation");
    
    // Record performance metrics
    if let Some(monitor) = monitor {
        let metrics = PerformanceMetrics {
            computation_time_ms: total_time.as_secs_f64() * 1000.0,
            memory_usage_bytes: memory_tracker.get_peak_usage(),
            operation_count: 1,
            timestamp: start_time,
        };
        monitor.record_metrics(metrics);
    }
    
    result
}

/// Original trilateration function for backward compatibility
pub fn trilaterate(
    anchors: &[Anchor],
    receiver_time_ms: u64,
) -> Result<(Position, Vector3<f64>), String> {
    trilaterate_with_monitoring(anchors, receiver_time_ms, None)
}

/// Internal trilateration implementation with detailed profiling
fn trilaterate_internal(
    anchors: &[Anchor],
    receiver_time_ms: u64,
    profiler: &mut performance_monitor::TimingProfiler,
    memory_tracker: &mut performance_monitor::MemoryTracker,
) -> Result<(Position, Vector3<f64>), String> {
    profiler.checkpoint("validation_start");
    
    if anchors.len() < 3 || anchors.len() > 4 {
        return Err("Between 3 and 4 anchors required".to_string());
    }

    // Track memory for intermediate calculations
    let positions_memory = std::mem::size_of::<Vector3<f64>>() * anchors.len();
    let distances_memory = std::mem::size_of::<f64>() * anchors.len();
    memory_tracker.track_allocation("positions_vec", positions_memory);
    memory_tracker.track_allocation("distances_vec", distances_memory);

    // Reference position (first anchor) for local tangent plane
    let reference_pos = &anchors[0].position;
    profiler.checkpoint("coordinate_conversion_start");

    // Convert anchor positions to local coordinates and compute distances
    let mut positions: Vec<Vector3<f64>> = Vec::new();
    let mut distances: Vec<f64> = Vec::new();

    for anchor in anchors {
        let local = geodetic_to_local(&anchor.position, reference_pos);
        positions.push(local);

        let dt_ms = receiver_time_ms as i64 - anchor.timestamp as i64;
        if dt_ms < 0 {
            return Err(format!(
                "Receiver time earlier than anchor time for anchor {}",
                anchor.id
            ));
        }
        let dt_sec = dt_ms as f64 / 1000.0;
        distances.push(SPEED_OF_SOUND_WATER * dt_sec);
    }

    profiler.checkpoint("coordinate_conversion_complete");

    // Handle 3-anchor case: 2D solution only
    if anchors.len() == 3 {
        profiler.checkpoint("3_anchor_processing_start");
        eprintln!(
            "WARNING: Only 3 anchors available. Providing 2D position without depth information. \
            Depth will be estimated as average anchor depth."
        );
        
        let p1 = positions[0];
        let d1_sq = distances[0] * distances[0];
        
        // Use 2x2 system for x,y coordinates only
        let mut a_data_2d = [[0.0; 2]; 2];
        let mut b_data = [0.0; 2];
        
        for i in 1..3 {
            let pi = positions[i];
            let di_sq = distances[i] * distances[i];
            let row = i - 1;
            a_data_2d[row][0] = 2.0 * (pi.x - p1.x);
            a_data_2d[row][1] = 2.0 * (pi.y - p1.y);

            // For 3 anchors, we assume the receiver is at approximately the same depth as anchors
            // This simplifies the equation by removing the z component
            b_data[row] = d1_sq - di_sq
                + pi.x.powi(2) - p1.x.powi(2)
                + pi.y.powi(2) - p1.y.powi(2)
                + pi.z.powi(2) - p1.z.powi(2);
        }
        
        let a_mat = nalgebra::Matrix2::from_rows(&[
            nalgebra::RowVector2::new(a_data_2d[0][0], a_data_2d[0][1]),
            nalgebra::RowVector2::new(a_data_2d[1][0], a_data_2d[1][1]),
        ]);
        let b_vec = nalgebra::Vector2::new(b_data[0], b_data[1]);
        
        // Check if anchors are collinear in 2D
        let det = a_mat.determinant();
        if det.abs() < 1e-10 {
            return Err("Anchors are collinear - cannot determine position".to_string());
        }
        
        // Solve for x,y coordinates
        let xy_solution = a_mat.try_inverse()
            .ok_or("Cannot solve: anchors are nearly collinear")?
            * b_vec;
        
        // Estimate depth as weighted average of anchor depths based on distances
        let total_weight: f64 = distances.iter().map(|d| 1.0 / (d + 1.0)).sum();
        let estimated_depth: f64 = positions.iter()
            .zip(distances.iter())
            .map(|(p, d)| p.z * (1.0 / (d + 1.0)) / total_weight)
            .sum();
        
        let position_local = Vector3::new(xy_solution.x, xy_solution.y, estimated_depth);
        let geodetic_pos = local_to_geodetic(&position_local, reference_pos);
        
        return Ok((geodetic_pos, position_local));
    }

    // For 4 anchors, continue with existing logic...

    // Check for geometric quality - detect nearly collinear anchors
    // For life-supporting systems, we need good geometry
    let p1 = positions[0];
    let p2 = positions[1];
    let p3 = positions[2];
    let p4 = positions[3];
    
    // Compute the volume of the tetrahedron formed by the 4 anchors
    // If this volume is too small, the anchors are nearly coplanar or collinear
    let v1 = p2 - p1;
    let v2 = p3 - p1;
    let v3 = p4 - p1;
    let volume = (v1.cross(&v2)).dot(&v3).abs() / 6.0;
    
    // Compute the maximum distance between anchors for scale
    let mut max_dist: f64 = 0.0;
    for i in 0..4 {
        for j in i+1..4 {
            let dist = (positions[i] - positions[j]).norm();
            max_dist = max_dist.max(dist);
        }
    }
    
    // The volume should be significant relative to the scale
    // For a regular tetrahedron with edge length L, volume = L³/(6√2) ≈ 0.118 L³
    // For coplanar anchors, volume will be 0, but we can still do 2D trilateration
    // We'll check for truly degenerate cases (collinear or nearly collinear)
    let min_volume = 0.0001 * max_dist.powi(3);  // Much smaller threshold
    
    // Also check if anchors are coplanar (all at same depth)
    let depths: Vec<f64> = positions.iter().map(|p| p.z).collect();
    let all_same_depth = depths.iter().all(|&d| (d - depths[0]).abs() < 1e-6);
    
    // For non-coplanar anchors, check 3D volume
    if !all_same_depth && volume < min_volume {
        eprintln!(
            "WARNING: Anchor geometry is nearly degenerate (collinear or coplanar). \
            Volume = {:.6} m³, recommended = {:.6} m³. \
            Position accuracy may be reduced to 5-10 meters.",
            volume, min_volume
        );
    }
    
    // For coplanar anchors, check 2D configuration
    if all_same_depth {
        // Check if anchors form a good 2D configuration
        // Compute area of triangles formed by first 3 anchors
        let area_123 = ((p2.x - p1.x) * (p3.y - p1.y) - (p3.x - p1.x) * (p2.y - p1.y)).abs() / 2.0;
        let area_124 = ((p2.x - p1.x) * (p4.y - p1.y) - (p4.x - p1.x) * (p2.y - p1.y)).abs() / 2.0;
        let area_134 = ((p3.x - p1.x) * (p4.y - p1.y) - (p4.x - p1.x) * (p3.y - p1.y)).abs() / 2.0;
        let area_234 = ((p3.x - p2.x) * (p4.y - p2.y) - (p4.x - p2.x) * (p3.y - p2.y)).abs() / 2.0;
        
        let min_area = area_123.min(area_124).min(area_134).min(area_234);
        let min_required_area = 0.01 * max_dist.powi(2);  // 1% of max_dist squared
        
        if min_area < min_required_area {
            eprintln!(
                "WARNING: Anchor geometry is nearly collinear in 2D. \
                Minimum triangle area = {:.3} m², recommended = {:.3} m². \
                Position accuracy may be reduced to 5-10 meters.",
                min_area, min_required_area
            );
        }
    }

    // Trilateration using linearized form
    let d1_sq = distances[0] * distances[0];

    // For coplanar anchors at same depth, we need to handle this specially
    if all_same_depth {
        // Use only x,y components for 2D trilateration
        let mut a_data_2d = [[0.0; 2]; 3];
        let mut b_data = [0.0; 3];
        
        for i in 1..4 {
            let pi = positions[i];
            let di_sq = distances[i] * distances[i];
            let row = i - 1;
            a_data_2d[row][0] = 2.0 * (pi.x - p1.x);
            a_data_2d[row][1] = 2.0 * (pi.y - p1.y);

            b_data[row] = d1_sq - di_sq
                + pi.x.powi(2) - p1.x.powi(2)
                + pi.y.powi(2) - p1.y.powi(2);
        }
        
        // Create overdetermined 3x2 matrix for least squares
        let a_mat = nalgebra::Matrix3x2::from_rows(&[
            nalgebra::RowVector2::new(a_data_2d[0][0], a_data_2d[0][1]),
            nalgebra::RowVector2::new(a_data_2d[1][0], a_data_2d[1][1]),
            nalgebra::RowVector2::new(a_data_2d[2][0], a_data_2d[2][1]),
        ]);
        let b_vec = Vector3::new(b_data[0], b_data[1], b_data[2]);
        
        // Solve using least squares
        let svd = a_mat.svd(true, true);
        let xy_solution = svd.solve(&b_vec, 1.0e-9).map_err(|e| e.to_string())?;
        
        // Construct full 3D position with the common depth
        let position_local = Vector3::new(xy_solution.x, xy_solution.y, depths[0]);
        
        // Convert back to lat/lon/depth for reporting
        let geodetic_pos = local_to_geodetic(&position_local, reference_pos);
        
        return Ok((geodetic_pos, position_local));
    }

    // For non-coplanar anchors, use full 3D trilateration
    let mut a_data = [[0.0; 3]; 3];
    let mut b_data = [0.0; 3];
    for i in 1..4 {
        let pi = positions[i];
        let di_sq = distances[i] * distances[i];
        let row = i - 1;
        a_data[row][0] = 2.0 * (pi.x - p1.x);
        a_data[row][1] = 2.0 * (pi.y - p1.y);
        a_data[row][2] = 2.0 * (pi.z - p1.z);

        b_data[row] = d1_sq - di_sq
            + pi.x.powi(2) - p1.x.powi(2)
            + pi.y.powi(2) - p1.y.powi(2)
            + pi.z.powi(2) - p1.z.powi(2);
    }

    let a_mat = Matrix3::from_rows(&[
        nalgebra::RowVector3::new(a_data[0][0], a_data[0][1], a_data[0][2]),
        nalgebra::RowVector3::new(a_data[1][0], a_data[1][1], a_data[1][2]),
        nalgebra::RowVector3::new(a_data[2][0], a_data[2][1], a_data[2][2]),
    ]);
    let b_vec = Vector3::new(b_data[0], b_data[1], b_data[2]);

    // Check condition number of the matrix
    let svd = a_mat.svd(true, true);
    let singular_values = svd.singular_values;
    let condition_number = if singular_values[2].abs() > 1e-10 {
        singular_values[0] / singular_values[2]
    } else {
        f64::INFINITY
    };
    
    let position_local = if condition_number > 1000.0 {
        eprintln!(
            "WARNING: Anchor configuration is ill-conditioned (condition number = {:.1}). \
            Position accuracy may be reduced to 5-10 meters.",
            condition_number
        );
        
        // Use regularized least squares (Tikhonov regularization)
        // Add small values to diagonal to stabilize the solution
        let lambda = 1e-6 * singular_values[0]; // Regularization parameter
        let mut a_regularized = a_mat;
        for i in 0..3 {
            a_regularized[(i, i)] += lambda;
        }
        
        let svd_reg = a_regularized.svd(true, true);
        svd_reg.solve(&b_vec, 1.0e-9).map_err(|e| e.to_string())?
    } else {
        svd.solve(&b_vec, 1.0e-9).map_err(|e| e.to_string())?
    };

    // Convert back to lat/lon/depth for reporting
    let geodetic_pos = local_to_geodetic(&position_local, reference_pos);

    Ok((geodetic_pos, position_local))
}

// Demonstration function showing embedded-optimized data structures usage
pub fn embedded_positioning_demo() {
    println!("=== Embedded Positioning System Demo ===");
    
    // Initialize system configuration
    let config = EmbeddedSystemConfig::new();
    println!("System config: sound speed = {:.1} m/s, max anchor age = {}ms", 
             config.get_sound_speed_m_per_s(), config.max_anchor_age_ms);
    
    // Create anchor buffer with fixed size (no heap allocation)
    let mut anchor_buffer: AnchorBuffer<8> = AnchorBuffer::new();
    
    // Set up base timestamp for compact messages
    let base_time = 1723111199000u64;
    CompactAnchorMessage::set_base_timestamp(base_time);
    
    // Create compact anchor messages (much smaller memory footprint)
    let anchors = [
        CompactAnchorMessage::new(1, base_time + 986, 32.12345, 45.47675, 0.0, 255),
        CompactAnchorMessage::new(2, base_time + 988, 32.12365, 45.47695, 0.0, 254),
        CompactAnchorMessage::new(3, base_time + 988, 32.12365, 45.47655, 0.0, 253),
        CompactAnchorMessage::new(4, base_time + 986, 32.12385, 45.47675, 0.0, 252),
    ];
    
    // Add to buffer
    for anchor in anchors.iter() {
        anchor_buffer.push(*anchor);
    }
    
    println!("Added {} anchor messages to buffer", anchor_buffer.len());
    
    // Get recent messages (within 5 seconds)
    let current_time = base_time + 1000;
    let recent_messages = anchor_buffer.get_recent_messages(5000, current_time);
    
    let mut valid_anchors = Vec::new();
    for msg_opt in recent_messages.iter() {
        if let Some(msg) = msg_opt {
            // Copy values to avoid unaligned access to packed struct
            let anchor_id = msg.anchor_id;
            // Convert compact message back to legacy format for trilateration
            let anchor = Anchor {
                id: anchor_id.to_string(),
                timestamp: msg.get_timestamp(),
                position: msg.get_position(),
            };
            valid_anchors.push(anchor);
        }
    }
    
    println!("Found {} recent anchor messages", valid_anchors.len());
    
    // Perform trilateration
    if let Ok((geodetic_pos, local_pos)) = trilaterate(&valid_anchors, current_time) {
        // Convert to memory-optimized local position
        let embedded_pos = LocalPosition::new(local_pos.x, local_pos.y, local_pos.z);
        let (east, north, down) = embedded_pos.to_meters();
        
        println!("Position calculated successfully:");
        println!("  Embedded local position: east={:.2}m, north={:.2}m, down={:.2}m", east, north, down);
        println!("  Geodetic: lat={:.6}, lon={:.6}, depth={:.2}m", 
                 geodetic_pos.lat, geodetic_pos.lon, geodetic_pos.depth);
        
        // Demonstrate fixed-point arithmetic for microcontrollers without FPU
        let fixed_east = FixedPoint32::new(east, 16);
        let fixed_north = FixedPoint32::new(north, 16);
        println!("  Fixed-point (16-bit fractional): east={:.3}, north={:.3}", 
                 fixed_east.to_f64(), fixed_north.to_f64());
    } else {
        println!("Trilateration failed");
    }
    
    // Show memory usage
    use std::mem;
    println!("\n=== Memory Footprint Analysis ===");
    println!("CompactAnchorMessage: {} bytes", mem::size_of::<CompactAnchorMessage>());
    println!("Legacy Anchor: {} bytes", mem::size_of::<Anchor>());
    println!("LocalPosition: {} bytes", mem::size_of::<LocalPosition>());
    println!("FixedPoint32: {} bytes", mem::size_of::<FixedPoint32>());
    println!("AnchorBuffer<8>: {} bytes", mem::size_of::<AnchorBuffer<8>>());
    println!("EmbeddedSystemConfig: {} bytes", mem::size_of::<EmbeddedSystemConfig>());
}

/// Demonstration of message parsing and validation system
pub fn message_parsing_system_demo() {
    message_parser_demo::message_parsing_demo();
    message_parser_demo::timestamp_precision_demo();
}

/// Demonstration of transceiver interface and communication system
pub fn transceiver_communication_demo() {
    transceiver_demo::transceiver_interface_demo();
    transceiver_demo::transceiver_performance_test();
}

/// Demonstration of coordinate system management with optimized transformations
pub fn coordinate_system_management_demo() {
    coordinate_system_demo::coordinate_system_demo();
}

/// Demonstration of comprehensive error handling and graceful degradation
pub fn error_handling_and_degradation_demo() {
    use shared_positioning::{ErrorLogger, ErrorRecoveryManager, PositioningError, ErrorSeverity, ErrorContext, ConsoleLogHandler};
    use graceful_degradation::{GracefulDegradationManager, OperatingMode};
    
    println!("=== ERROR HANDLING AND GRACEFUL DEGRADATION DEMO ===\n");
    
    // Initialize error handling system
    let mut error_logger = ErrorLogger::new(100, ErrorSeverity::Warning);
    error_logger.add_output_handler(Box::new(ConsoleLogHandler::new(ErrorSeverity::Error)));
    
    let mut recovery_manager = ErrorRecoveryManager::new();
    let mut degradation_manager = GracefulDegradationManager::new();
    
    println!("1. NORMAL OPERATION (4 anchors)");
    let base_time = 1723111199000u64;
    let good_anchors = vec![
        Anchor {
            id: "001".to_string(),
            timestamp: base_time + 986,
            position: Position { lat: 32.12345, lon: 45.47675, depth: 0.0 },
        },
        Anchor {
            id: "002".to_string(),
            timestamp: base_time + 988,
            position: Position { lat: 32.12365, lon: 45.47695, depth: 5.0 },
        },
        Anchor {
            id: "003".to_string(),
            timestamp: base_time + 988,
            position: Position { lat: 32.12365, lon: 45.47655, depth: 10.0 },
        },
        Anchor {
            id: "004".to_string(),
            timestamp: base_time + 986,
            position: Position { lat: 32.12385, lon: 45.47675, depth: 15.0 },
        },
    ];
    
    match degradation_manager.attempt_positioning(&good_anchors, base_time + 990) {
        Ok(result) => {
            println!("   Position: lat={:.6}, lon={:.6}, depth={:.2}m", 
                     result.position.lat, result.position.lon, result.position.depth);
            println!("   Mode: {:?}", result.operating_mode);
            println!("   Accuracy: {:.2}m", result.accuracy_estimate.overall_accuracy_m);
            if !result.warnings.is_empty() {
                println!("   Warnings: {:?}", result.warnings);
            }
        }
        Err(e) => println!("   Error: {}", e),
    }
    
    println!("\n2. DEGRADED OPERATION (3 anchors - 2D mode)");
    let degraded_anchors = vec![
        good_anchors[0].clone(),
        good_anchors[1].clone(),
        good_anchors[2].clone(),
    ];
    
    match degradation_manager.attempt_positioning(&degraded_anchors, base_time + 990) {
        Ok(result) => {
            println!("   Position: lat={:.6}, lon={:.6}, depth={:.2}m", 
                     result.position.lat, result.position.lon, result.position.depth);
            println!("   Mode: {:?}", result.operating_mode);
            println!("   Accuracy: {:.2}m", result.accuracy_estimate.overall_accuracy_m);
            if !result.warnings.is_empty() {
                println!("   Warnings: {:?}", result.warnings);
            }
        }
        Err(e) => println!("   Error: {}", e),
    }
    
    println!("\n3. SEVERELY DEGRADED OPERATION (2 anchors - range/bearing mode)");
    let minimal_anchors = vec![
        good_anchors[0].clone(),
        good_anchors[1].clone(),
    ];
    
    match degradation_manager.attempt_positioning(&minimal_anchors, base_time + 990) {
        Ok(result) => {
            println!("   Position: lat={:.6}, lon={:.6}, depth={:.2}m", 
                     result.position.lat, result.position.lon, result.position.depth);
            println!("   Mode: {:?}", result.operating_mode);
            println!("   Accuracy: {:.2}m", result.accuracy_estimate.overall_accuracy_m);
            if !result.warnings.is_empty() {
                println!("   Warnings: {:?}", result.warnings);
            }
        }
        Err(e) => println!("   Error: {}", e),
    }
    
    println!("\n4. ERROR HANDLING DEMONSTRATION");
    
    // Simulate various errors
    let errors = vec![
        PositioningError::InsufficientAnchors {
            available: 1,
            required: 3,
            anchor_ids: vec![1],
        },
        PositioningError::DegenerateGeometry {
            condition_number: 15000.0,
            volume: 0.001,
            geometry_type: shared_positioning::GeometryIssue::Collinear,
            anchor_positions: vec![(1, 32.0, 45.0, 0.0), (2, 32.001, 45.0, 0.0)],
        },
        PositioningError::StaleData {
            oldest_anchor_age_ms: 10000,
            max_allowed_age_ms: 5000,
            stale_anchor_ids: vec![1, 2, 3],
        },
    ];
    
    let context = ErrorContext::default();
    
    for (i, error) in errors.iter().enumerate() {
        println!("   Error {}: {}", i + 1, error);
        
        if let Some(strategy) = recovery_manager.handle_error(error, &context) {
            println!("   Recovery strategy: {:?}", strategy);
        }
        
        error_logger.log_error(error.clone(), context.clone(), None);
    }
    
    println!("\n5. SYSTEM HEALTH MONITORING");
    let health = degradation_manager.get_system_health();
    println!("   Overall status: {:?}", health.overall_status);
    println!("   Performance metrics:");
    println!("     Success rate: {:.1}%", health.performance_metrics.positioning_success_rate * 100.0);
    println!("     Avg computation time: {:.1} ms", health.performance_metrics.average_computation_time_ms);
    println!("     Avg accuracy: {:.2} m", health.performance_metrics.average_accuracy_m);
    
    println!("   Resource usage:");
    println!("     Memory: {:.1}%", health.resource_usage.memory_usage_percent);
    println!("     CPU: {:.1}%", health.resource_usage.cpu_usage_percent);
    if let Some(battery) = health.resource_usage.battery_level_percent {
        println!("     Battery: {:.1}%", battery);
    }
    
    println!("\n6. PERFORMANCE STATISTICS");
    let perf_stats = degradation_manager.get_performance_statistics();
    println!("   Total attempts: {}", perf_stats.total_attempts);
    println!("   Success rate: {:.1}%", perf_stats.success_rate * 100.0);
    println!("   Average accuracy: {:.2} m", perf_stats.average_accuracy_m);
    println!("   Average computation time: {:.1} ms", perf_stats.average_computation_time_ms);
    
    println!("\n7. ERROR STATISTICS");
    let error_stats = error_logger.get_error_statistics();
    println!("   Total errors: {}", error_stats.total_errors);
    println!("   Resolved errors: {}", error_stats.resolved_errors);
    
    println!("\n8. RECOVERY STATISTICS");
    let recovery_stats = recovery_manager.get_recovery_statistics();
    println!("   Recovery attempts: {}", recovery_stats.total_attempts);
    println!("   Success rate: {:.1}%", recovery_stats.success_rate * 100.0);
    println!("   Average recovery time: {:.1} ms", recovery_stats.average_recovery_time);
}

/// Demonstration of advanced positioning accuracy improvements
pub fn advanced_positioning_demo() {
    use advanced_positioning::{AdvancedPositioningEngine, enhanced_trilaterate};
    
    println!("=== ADVANCED POSITIONING ALGORITHMS DEMO ===\n");
    
    // Create test anchor data with some noise and 3D geometry
    let base_time = 1723111199000u64;
    let test_anchors = vec![
        Anchor {
            id: "001".to_string(),
            timestamp: base_time + 986,
            position: Position { lat: 32.12345, lon: 45.47675, depth: 0.0 },
        },
        Anchor {
            id: "002".to_string(),
            timestamp: base_time + 988,
            position: Position { lat: 32.12365, lon: 45.47695, depth: 5.0 }, // Add depth variation
        },
        Anchor {
            id: "003".to_string(),
            timestamp: base_time + 988,
            position: Position { lat: 32.12365, lon: 45.47655, depth: 10.0 }, // Add depth variation
        },
        Anchor {
            id: "004".to_string(),
            timestamp: base_time + 986,
            position: Position { lat: 32.12385, lon: 45.47675, depth: 15.0 }, // Add depth variation
        },
        // Add a potential outlier anchor
        Anchor {
            id: "005".to_string(),
            timestamp: base_time + 1200, // Much later timestamp (potential outlier)
            position: Position { lat: 32.12400, lon: 45.47700, depth: 20.0 },
        },
    ];
    
    let receiver_time = base_time + 1000;
    let reference_pos = &test_anchors[0].position;
    
    println!("1. STANDARD TRILATERATION (for comparison)");
    match trilaterate(&test_anchors[0..4], receiver_time) {
        Ok((pos, local_pos)) => {
            println!("   Standard result: lat={:.6}, lon={:.6}, depth={:.2}m", 
                     pos.lat, pos.lon, pos.depth);
            println!("   Local position: east={:.2}m, north={:.2}m, down={:.2}m", 
                     local_pos.x, local_pos.y, local_pos.z);
        }
        Err(e) => println!("   Standard trilateration failed: {}", e),
    }
    
    println!("\n2. ENHANCED TRILATERATION WITH ADVANCED ALGORITHMS");
    match enhanced_trilaterate(&test_anchors[0..4], receiver_time, reference_pos) {
        Ok((pos, local_pos, uncertainty)) => {
            println!("   Enhanced result: lat={:.6}, lon={:.6}, depth={:.2}m", 
                     pos.lat, pos.lon, pos.depth);
            println!("   Local position: east={:.2}m, north={:.2}m, down={:.2}m", 
                     local_pos.x, local_pos.y, local_pos.z);
            println!("   Uncertainty estimate: {:.3}m", uncertainty);
        }
        Err(e) => println!("   Enhanced trilateration failed: {}", e),
    }
    
    println!("\n3. INDIVIDUAL ALGORITHM TESTING");
    let mut engine = AdvancedPositioningEngine::new();
    
    // Test Maximum Likelihood Estimator
    println!("   a) Maximum Likelihood Estimator:");
    match engine.trilaterate_mle(&test_anchors[0..4], receiver_time, 1.0) {
        Ok(result) => {
            println!("      Position: east={:.2}m, north={:.2}m, down={:.2}m", 
                     result.position.x, result.position.y, result.position.z);
            println!("      Uncertainty: {:.3}m, Log-likelihood: {:.2}", 
                     result.uncertainty, result.log_likelihood);
            println!("      Iterations: {}", result.iterations);
        }
        Err(e) => println!("      MLE failed: {}", e),
    }
    
    // Test Levenberg-Marquardt optimization
    println!("   b) Levenberg-Marquardt Optimization:");
    match engine.trilaterate_levenberg_marquardt(&test_anchors[0..4], receiver_time) {
        Ok(result) => {
            println!("      Position: east={:.2}m, north={:.2}m, down={:.2}m", 
                     result.position.x, result.position.y, result.position.z);
            println!("      Residual norm: {:.3}, Iterations: {}, Converged: {}", 
                     result.residual_norm, result.iterations, result.converged);
        }
        Err(e) => println!("      LM optimization failed: {}", e),
    }
    
    // Test Weighted Least Squares
    println!("   c) Weighted Least Squares:");
    match engine.trilaterate_weighted_least_squares(&test_anchors[0..4], receiver_time) {
        Ok((position, uncertainty)) => {
            println!("      Position: east={:.2}m, north={:.2}m, down={:.2}m", 
                     position.x, position.y, position.z);
            println!("      Uncertainty: {:.3}m", uncertainty);
        }
        Err(e) => println!("      Weighted LS failed: {}", e),
    }
    
    // Test Robust Estimation (with outlier)
    println!("   d) Robust Estimation (with potential outlier):");
    match engine.trilaterate_robust(&test_anchors[0..4], receiver_time) {
        Ok((position, uncertainty)) => {
            println!("      Position: east={:.2}m, north={:.2}m, down={:.2}m", 
                     position.x, position.y, position.z);
            println!("      Uncertainty: {:.3}m", uncertainty);
        }
        Err(e) => println!("      Robust estimation failed: {}", e),
    }
    
    println!("\n4. GDOP OPTIMIZATION AND ANALYSIS");
    use advanced_positioning::{GDOPAnalyzer, GeometryQuality};
    
    let mut gdop_analyzer = GDOPAnalyzer::new();
    
    // Convert anchors to local coordinates for GDOP analysis
    let anchor_positions: Vec<Vector3<f64>> = test_anchors[0..4].iter()
        .map(|anchor| geodetic_to_local(&anchor.position, reference_pos))
        .collect();
    
    // Estimate position for GDOP calculation (use centroid of anchors as initial estimate)
    let estimated_position = {
        let centroid = anchor_positions.iter().fold(Vector3::zeros(), |acc, pos| acc + pos) / anchor_positions.len() as f64;
        Vector3::new(centroid.x + 10.0, centroid.y + 10.0, centroid.z + 5.0) // Offset from centroid
    };
    
    // Calculate GDOP metrics
    match gdop_analyzer.calculate_gdop(&anchor_positions, &estimated_position) {
        Ok(gdop_result) => {
            println!("   GDOP Analysis Results:");
            println!("      GDOP: {:.2}", gdop_result.gdop);
            println!("      PDOP: {:.2}", gdop_result.pdop);
            println!("      HDOP: {:.2}", gdop_result.hdop);
            println!("      VDOP: {:.2}", gdop_result.vdop);
            println!("      Geometry Quality: {:?}", gdop_result.geometry_quality);
            
            // Test geometry quality assessment
            match gdop_analyzer.assess_geometry_quality(&anchor_positions, &estimated_position) {
                Ok(quality_score) => {
                    println!("      Geometry Quality Score: {:.3} (0-1, higher is better)", quality_score);
                }
                Err(e) => println!("      Geometry assessment failed: {}", e),
            }
            
            // Test position uncertainty estimation
            match gdop_analyzer.estimate_position_uncertainty(&anchor_positions, &estimated_position, 0.5) {
                Ok((overall_uncertainty, directional_uncertainty)) => {
                    println!("      Position Uncertainty:");
                    println!("         Overall: {:.2}m", overall_uncertainty);
                    println!("         East: {:.2}m, North: {:.2}m, Down: {:.2}m", 
                             directional_uncertainty.x, directional_uncertainty.y, directional_uncertainty.z);
                }
                Err(e) => println!("      Uncertainty estimation failed: {}", e),
            }
            
            // Test algorithm selection
            let recommended_algorithm = gdop_analyzer.select_algorithm_for_gdop(gdop_result.gdop);
            println!("      Recommended Algorithm: {}", recommended_algorithm);
        }
        Err(e) => println!("   GDOP calculation failed: {}", e),
    }
    
    // Test anchor selection optimization
    println!("   Anchor Selection Optimization:");
    match gdop_analyzer.select_optimal_anchors(&test_anchors[0..4], receiver_time, 3) {
        Ok(selection_result) => {
            println!("      Selected anchors: {:?}", selection_result.selected_anchors);
            println!("      Expected GDOP: {:.2}", selection_result.gdop);
            println!("      Expected accuracy: {:.2}m", selection_result.expected_accuracy);
            println!("      Geometry quality: {:?}", selection_result.geometry_quality);
        }
        Err(e) => println!("      Anchor selection failed: {}", e),
    }
    
    // Test GDOP-optimized trilateration
    println!("   GDOP-Optimized Trilateration:");
    match engine.trilaterate_with_gdop_optimization(&test_anchors[0..4], receiver_time) {
        Ok((position, uncertainty, gdop_result)) => {
            println!("      Position: east={:.2}m, north={:.2}m, down={:.2}m", 
                     position.x, position.y, position.z);
            println!("      Uncertainty: {:.3}m", uncertainty);
            println!("      GDOP: {:.2}, Quality: {:?}", gdop_result.gdop, gdop_result.geometry_quality);
        }
        Err(e) => println!("      GDOP-optimized trilateration failed: {}", e),
    }

    println!("\n5. ADVANCED NOISE FILTERING AND SIGNAL PROCESSING");
    use advanced_positioning::{SignalProcessor, EnvironmentalParams, WeightedMeasurement};
    
    let mut signal_processor = SignalProcessor::new();
    
    // Create test measurements with noise and multipath effects
    let test_measurements = vec![
        WeightedMeasurement {
            anchor_position: Vector3::new(0.0, 0.0, 0.0),
            measured_distance: 100.0,
            weight: 1.0,
            quality: 0.9,
        },
        WeightedMeasurement {
            anchor_position: Vector3::new(100.0, 0.0, 5.0),
            measured_distance: 105.2, // With some noise
            weight: 1.0,
            quality: 0.7,
        },
        WeightedMeasurement {
            anchor_position: Vector3::new(0.0, 100.0, 10.0),
            measured_distance: 98.5, // With some noise
            weight: 1.0,
            quality: 0.8,
        },
        WeightedMeasurement {
            anchor_position: Vector3::new(100.0, 100.0, 15.0),
            measured_distance: 142.8, // With multipath effect
            weight: 1.0,
            quality: 0.4, // Lower quality due to multipath
        },
    ];
    
    let signal_qualities = vec![0.9, 0.7, 0.8, 0.4];
    
    // Test adaptive noise filtering
    println!("   Adaptive Noise Filtering:");
    let filtered_measurements = signal_processor.adaptive_noise_filter(&test_measurements, &signal_qualities);
    for (i, filtered) in filtered_measurements.iter().enumerate() {
        println!("      Anchor {}: Original={:.1}m, Filtered={:.1}m, Confidence={:.2}, Multipath={}",
                 i, test_measurements[i].measured_distance, filtered.filtered_range, 
                 filtered.confidence, filtered.multipath_detected);
    }
    
    // Test multipath detection and mitigation
    println!("   Multipath Detection and Mitigation:");
    let range_measurements = vec![100.0, 101.0, 99.5, 120.0, 102.0, 100.5]; // Spike at index 3
    let timestamps = vec![1000, 1100, 1200, 1300, 1400, 1500];
    let corrected_ranges = signal_processor.detect_and_mitigate_multipath(1, &range_measurements, &timestamps);
    
    for (i, (&original, &corrected)) in range_measurements.iter().zip(corrected_ranges.iter()).enumerate() {
        println!("      Time {}: Original={:.1}m, Corrected={:.1}m, Difference={:.1}m",
                 timestamps[i], original, corrected, original - corrected);
    }
    
    // Test systematic error compensation
    println!("   Systematic Error Compensation:");
    let env_params = EnvironmentalParams {
        sound_speed: 1520.0, // Slightly different from default
        temperature: 20.0,
        salinity: 32.0,
        pressure: 2.0,
        current_velocity: Vector3::new(0.1, 0.0, 0.0),
    };
    
    let raw_measurements = vec![100.0, 150.0, 200.0, 250.0];
    let anchor_positions = vec![
        Vector3::new(0.0, 0.0, 0.0),
        Vector3::new(100.0, 0.0, 5.0),
        Vector3::new(0.0, 100.0, 10.0),
        Vector3::new(100.0, 100.0, 15.0),
    ];
    
    let compensated_measurements = signal_processor.compensate_systematic_errors(
        &raw_measurements, &anchor_positions, &env_params
    );
    
    for (i, (&raw, &compensated)) in raw_measurements.iter().zip(compensated_measurements.iter()).enumerate() {
        println!("      Anchor {}: Raw={:.2}m, Compensated={:.2}m, Correction={:.3}m",
                 i, raw, compensated, compensated - raw);
    }
    
    // Test temporal jitter filtering
    println!("   Temporal Jitter Filtering:");
    let noisy_measurements = vec![100.0, 102.5, 99.8, 101.2, 100.3, 99.9, 100.7];
    let mut filtered_values = Vec::new();
    
    for (i, &measurement) in noisy_measurements.iter().enumerate() {
        let timestamp = 1000 + (i as u64 * 100);
        let filtered = signal_processor.temporal_jitter_filter(0, measurement, timestamp);
        filtered_values.push(filtered);
        println!("      Step {}: Noisy={:.1}m, Filtered={:.1}m", i + 1, measurement, filtered);
    }
    
    // Test signal quality weighting
    println!("   Signal Quality Weighting:");
    let test_ranges = vec![100.0, 105.0, 98.0, 142.0];
    let test_qualities = vec![0.9, 0.7, 0.8, 0.3];
    let weights = signal_processor.compute_signal_quality_weights(&test_ranges, &test_qualities);
    
    for (i, (&quality, &weight)) in test_qualities.iter().zip(weights.iter()).enumerate() {
        println!("      Anchor {}: Quality={:.1}, Weight={:.3}", i, quality, weight);
    }

    println!("\n6. SUB-METER ACCURACY OPTIMIZATION");
    use advanced_positioning::{SubMeterAccuracyOptimizer, AnchorCalibration, ClockCalibration};
    
    let mut sub_meter_optimizer = SubMeterAccuracyOptimizer::new(reference_pos.clone());
    
    // Add calibration data for improved accuracy
    sub_meter_optimizer.add_anchor_calibration(0, AnchorCalibration {
        position_offset: Vector3::new(0.05, -0.03, 0.02), // Small position corrections
        range_bias: 0.1,  // 10cm range bias correction
        uncertainty: 0.05, // 5cm uncertainty
    });
    
    sub_meter_optimizer.add_anchor_calibration(1, AnchorCalibration {
        position_offset: Vector3::new(-0.02, 0.04, -0.01),
        range_bias: -0.08,
        uncertainty: 0.04,
    });
    
    sub_meter_optimizer.add_clock_calibration(0, ClockCalibration {
        time_offset: 0.001,  // 1ms clock offset
        drift_rate: 1e-6,    // 1 microsecond per second drift
        uncertainty: 0.0005, // 0.5ms uncertainty
    });
    
    // Test sub-meter optimization
    println!("   High-Precision Positioning with Sub-Meter Optimization:");
    
    // Create high-quality test measurements
    let high_precision_measurements = vec![
        WeightedMeasurement {
            anchor_position: Vector3::new(0.0, 0.0, 0.0),
            measured_distance: 100.15,  // More precise measurements
            weight: 1.0,
            quality: 0.95,
        },
        WeightedMeasurement {
            anchor_position: Vector3::new(100.0, 0.0, 5.0),
            measured_distance: 105.23,
            weight: 1.0,
            quality: 0.92,
        },
        WeightedMeasurement {
            anchor_position: Vector3::new(0.0, 100.0, 10.0),
            measured_distance: 98.47,
            weight: 1.0,
            quality: 0.88,
        },
        WeightedMeasurement {
            anchor_position: Vector3::new(100.0, 100.0, 15.0),
            measured_distance: 142.13,
            weight: 1.0,
            quality: 0.85,
        },
    ];
    
    let high_precision_env = EnvironmentalParams {
        sound_speed: 1518.5,  // More precise sound speed
        temperature: 18.2,
        salinity: 34.7,
        pressure: 1.8,
        current_velocity: Vector3::new(0.05, -0.02, 0.01),
    };
    
    match sub_meter_optimizer.optimize_for_sub_meter_accuracy(
        &high_precision_measurements, 
        &high_precision_env, 
        receiver_time
    ) {
        Ok(result) => {
            println!("      Optimized Position:");
            println!("         Local: east={:.3}m, north={:.3}m, down={:.3}m", 
                     result.position.x, result.position.y, result.position.z);
            println!("         Geodetic: lat={:.7}, lon={:.7}, depth={:.3}m", 
                     result.geodetic_position.lat, result.geodetic_position.lon, result.geodetic_position.depth);
            
            println!("      Accuracy Estimate:");
            println!("         Horizontal: {:.3}m (95% confidence)", result.accuracy_estimate.horizontal_accuracy);
            println!("         Vertical: {:.3}m (95% confidence)", result.accuracy_estimate.vertical_accuracy);
            println!("         Overall: {:.3}m (95% confidence)", result.accuracy_estimate.overall_accuracy);
            
            println!("      Accuracy Components:");
            println!("         Measurement noise: {:.3}m", result.accuracy_estimate.accuracy_components.measurement_noise);
            println!("         Geometric dilution: {:.3}m", result.accuracy_estimate.accuracy_components.geometric_dilution);
            println!("         Systematic errors: {:.3}m", result.accuracy_estimate.accuracy_components.systematic_errors);
            println!("         Environmental effects: {:.3}m", result.accuracy_estimate.accuracy_components.environmental_effects);
            println!("         Calibration uncertainty: {:.3}m", result.accuracy_estimate.accuracy_components.calibration_uncertainty);
            
            println!("      Confidence Intervals (95%):");
            println!("         East: [{:.3}, {:.3}]m", result.confidence_interval.east_95.0, result.confidence_interval.east_95.1);
            println!("         North: [{:.3}, {:.3}]m", result.confidence_interval.north_95.0, result.confidence_interval.north_95.1);
            println!("         Down: [{:.3}, {:.3}]m", result.confidence_interval.down_95.0, result.confidence_interval.down_95.1);
            
            println!("      Validation Status: {:?}", result.validation_status);
            
            println!("      Applied Corrections:");
            println!("         Sound speed: {:.3}m", result.correction_summary.sound_speed_correction);
            println!("         Temperature: {:.3}m", result.correction_summary.temperature_correction);
            println!("         Pressure: {:.3}m", result.correction_summary.pressure_correction);
            println!("         Clock bias: {:.3}m", result.correction_summary.clock_bias_correction);
            println!("         Systematic bias: {:.3}m", result.correction_summary.systematic_bias_correction);
            println!("         Total correction magnitude: {:.3}m", result.correction_summary.total_correction_magnitude);
            
            // Check if sub-meter accuracy achieved
            if result.accuracy_estimate.overall_accuracy < 1.0 {
                println!("      ✓ SUB-METER ACCURACY ACHIEVED!");
            } else {
                println!("      ⚠ Sub-meter accuracy not achieved (accuracy: {:.3}m)", result.accuracy_estimate.overall_accuracy);
            }
        }
        Err(e) => println!("      Sub-meter optimization failed: {}", e),
    }

    println!("\n7. KALMAN FILTERING DEMONSTRATION");
    let mut filter_engine = AdvancedPositioningEngine::new();
    
    // Simulate a sequence of position measurements
    let measurements = vec![
        (Vector3::new(10.0, 20.0, 5.0), 1.0, base_time + 1000),
        (Vector3::new(10.2, 20.1, 5.1), 1.2, base_time + 1100),
        (Vector3::new(10.1, 19.9, 4.9), 0.8, base_time + 1200),
        (Vector3::new(10.3, 20.2, 5.2), 1.5, base_time + 1300),
        (Vector3::new(9.9, 19.8, 4.8), 0.9, base_time + 1400),
    ];
    
    println!("   Kalman filter smoothing results:");
    for (i, (measurement, uncertainty, timestamp)) in measurements.iter().enumerate() {
        let filtered = filter_engine.update_kalman_filter(*measurement, *uncertainty, *timestamp);
        filter_engine.add_position_measurement(*measurement, *uncertainty, *timestamp, 4, 0.8);
        
        println!("      Step {}: Raw=({:.1}, {:.1}, {:.1}), Filtered=({:.1}, {:.1}, {:.1})", 
                 i + 1,
                 measurement.x, measurement.y, measurement.z,
                 filtered.x, filtered.y, filtered.z);
    }
    
    // Test temporal filtering
    if let Some(temporal_filtered) = filter_engine.get_filtered_position(base_time + 1500) {
        println!("   Temporal filtered position: ({:.1}, {:.1}, {:.1})", 
                 temporal_filtered.x, temporal_filtered.y, temporal_filtered.z);
    }
    
    println!("\n=== Advanced positioning algorithms demonstration complete ===");
}

/// Demonstration of performance monitoring and optimization features
pub fn performance_optimization_demo() {
    use performance_monitor::{PerformanceMonitor, PerformanceConstraints};
    use optimization_cache::OptimizationManager;
    
    println!("=== PERFORMANCE OPTIMIZATION DEMO ===\n");
    
    // Initialize performance monitoring for embedded system
    let constraints = PerformanceConstraints::embedded_system();
    let mut monitor = PerformanceMonitor::new(constraints);
    
    // Initialize optimization manager with caching
    let mut optimizer = OptimizationManager::new();
    
    println!("1. PERFORMANCE MONITORING SETUP");
    println!("   - Max computation time: {:.1} ms", monitor.constraints.max_computation_time_ms);
    println!("   - Max memory usage: {} KB", monitor.constraints.max_memory_usage_bytes / 1024);
    println!("   - Min update rate: {:.1} Hz\n", monitor.constraints.min_update_rate_hz);
    
    // Create test anchor data
    let base_time = 1723111199000u64;
    CompactAnchorMessage::set_base_timestamp(base_time);
    
    let test_anchors = vec![
        Anchor {
            id: "001".to_string(),
            timestamp: base_time + 986,
            position: Position { lat: 32.12345, lon: 45.47675, depth: 0.0 },
        },
        Anchor {
            id: "002".to_string(),
            timestamp: base_time + 988,
            position: Position { lat: 32.12365, lon: 45.47695, depth: 0.0 },
        },
        Anchor {
            id: "003".to_string(),
            timestamp: base_time + 988,
            position: Position { lat: 32.12365, lon: 45.47655, depth: 0.0 },
        },
        Anchor {
            id: "004".to_string(),
            timestamp: base_time + 986,
            position: Position { lat: 32.12385, lon: 45.47675, depth: 0.0 },
        },
    ];
    
    println!("2. RUNNING PERFORMANCE TESTS");
    
    // Test multiple positioning calculations with monitoring
    for i in 0..10 {
        let receiver_time = base_time + 1000 + (i * 100);
        
        // Add some anchor data to cache
        for (j, anchor) in test_anchors.iter().enumerate() {
            let compact_msg = CompactAnchorMessage::new(
                (j + 1) as u16, 
                anchor.timestamp + (i * 10), 
                anchor.position.lat, 
                anchor.position.lon, 
                anchor.position.depth, 
                255 - (i as u8)
            );
            optimizer.anchor_cache.insert((j + 1) as u16, compact_msg);
        }
        
        // Perform trilateration with monitoring
        monitor.start_operation("trilateration");
        let result = trilaterate_with_monitoring(&test_anchors, receiver_time, Some(&mut monitor));
        let duration = monitor.end_operation("trilateration");
        
        if let Ok((pos, _)) = result {
            println!("   Test {}: Position calculated in {:.2} ms - lat={:.6}, lon={:.6}", 
                     i + 1, 
                     duration.map_or(0.0, |d| d.as_secs_f64() * 1000.0),
                     pos.lat, pos.lon);
        }
        
        // Simulate some processing delay
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
    
    println!("\n3. PERFORMANCE STATISTICS");
    let stats = monitor.get_statistics();
    println!("   Total operations: {}", stats.total_operations);
    println!("   Mean computation time: {:.2} ms", stats.mean_computation_time_ms);
    println!("   Max computation time: {:.2} ms", stats.max_computation_time_ms);
    println!("   95th percentile time: {:.2} ms", stats.p95_computation_time_ms);
    println!("   Violation count: {}", stats.violation_count);
    println!("   Violation rate: {:.1}%", stats.violation_rate * 100.0);
    
    // Validate constraints
    let validation = monitor.validate_real_time_constraints();
    println!("\n4. CONSTRAINT VALIDATION");
    if validation.is_valid {
        println!("   ✓ All real-time constraints satisfied");
    } else {
        println!("   ⚠ Constraint violations detected:");
        for violation in &validation.violations {
            println!("     - {}", violation);
        }
    }
    
    println!("\n5. CACHE OPTIMIZATION STATISTICS");
    let cache_stats = optimizer.anchor_cache.get_stats();
    println!("   Anchor cache entries: {}/{}", cache_stats.total_entries, cache_stats.max_entries);
    println!("   Total cache accesses: {}", cache_stats.total_accesses);
    println!("   Average data age: {} ms", cache_stats.avg_age_ms);
    
    // Test calculation caching
    optimizer.calculation_cache.cache_coordinate_transform(
        "test_transform".to_string(), 
        nalgebra::Vector3::new(1.0, 2.0, 3.0)
    );
    optimizer.calculation_cache.cache_distance("test_distance".to_string(), 150.0);
    
    let calc_stats = optimizer.calculation_cache.get_stats();
    println!("   Calculation cache entries: {}", calc_stats.total_entries);
    println!("   Coordinate transforms cached: {}", calc_stats.coordinate_transforms);
    println!("   Distance calculations cached: {}", calc_stats.distance_calculations);
    
    // Memory pool utilization
    let vector_util = optimizer.vector_pool.get_utilization();
    let position_util = optimizer.position_pool.get_utilization();
    println!("\n6. MEMORY POOL UTILIZATION");
    println!("   Vector3 pool: {}/{} ({:.1}% utilized)", 
             vector_util.allocated_count, vector_util.total_capacity, vector_util.utilization_percent);
    println!("   Position pool: {}/{} ({:.1}% utilized)", 
             position_util.allocated_count, position_util.total_capacity, position_util.utilization_percent);
    
    // Generate comprehensive reports
    println!("\n7. DETAILED REPORTS");
    println!("\n--- PERFORMANCE MONITOR REPORT ---");
    let perf_report = monitor.generate_embedded_report();
    println!("{}", perf_report);
    
    println!("\n--- OPTIMIZATION SYSTEM REPORT ---");
    let opt_report = optimizer.generate_report();
    println!("{}", opt_report);
    
    // Test lazy evaluation
    println!("\n8. LAZY EVALUATION DEMO");
    use optimization_cache::LazyValue;
    let mut lazy_calculation = LazyValue::new(|| {
        println!("   Performing expensive calculation...");
        std::thread::sleep(std::time::Duration::from_millis(50));
        42.0
    });
    
    println!("   Lazy value initialized: {}", lazy_calculation.is_initialized());
    println!("   First access: {}", lazy_calculation.get());
    println!("   Lazy value initialized: {}", lazy_calculation.is_initialized());
    println!("   Second access (cached): {}", lazy_calculation.get());
    
    println!("\n=== PERFORMANCE OPTIMIZATION DEMO COMPLETE ===");
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    
    // Check for embedded demo mode
    if args.len() == 2 && args[1] == "--embedded-demo" {
        embedded_positioning_demo();
        return Ok(());
    }
    
    // Check for performance monitoring demo mode
    if args.len() == 2 && args[1] == "--performance-demo" {
        performance_optimization_demo();
        return Ok(());
    }
    
    // Check for accuracy validation mode
    if args.len() == 2 && args[1] == "--accuracy-validation" {
        println!("Running comprehensive accuracy validation tests...");
        let validator = accuracy_validation::AccuracyValidator::new().with_trials(200);
        let results = validator.run_validation();
        let report = validator.generate_report(&results);
        println!("{}", report);
        
        // Check if minimum requirements are met
        let passing_configs = results.iter().filter(|r| r.meets_requirement).count();
        if passing_configs == 0 {
            eprintln!("ERROR: No configurations meet the 1.0m accuracy requirement!");
            return Err("Accuracy validation failed".into());
        }
        
        println!("Accuracy validation completed successfully!");
        return Ok(());
    }
    
    // Check for message parsing demo mode
    if args.len() == 2 && args[1] == "--message-parsing-demo" {
        message_parsing_system_demo();
        return Ok(());
    }
    
    // Check for transceiver communication demo mode
    if args.len() == 2 && args[1] == "--transceiver-demo" {
        transceiver_communication_demo();
        return Ok(());
    }
    
    // Check for advanced positioning demo mode
    if args.len() == 2 && args[1] == "--advanced-positioning-demo" {
        advanced_positioning_demo();
        return Ok(());
    }
    
    // Check for error handling and graceful degradation demo mode
    if args.len() == 2 && args[1] == "--error-handling-demo" {
        error_handling_and_degradation_demo();
        return Ok(());
    }
    
    if args.len() != 3 {
        eprintln!(
            "Usage: {} <json_file> <receiver_timestamp_ms>",
            args.get(0).map_or("trilateration", |s| s.as_str())
        );
        eprintln!("   or: {} --embedded-demo", args.get(0).map_or("trilateration", |s| s.as_str()));
        eprintln!("   or: {} --performance-demo", args.get(0).map_or("trilateration", |s| s.as_str()));
        eprintln!("   or: {} --accuracy-validation", args.get(0).map_or("trilateration", |s| s.as_str()));
        eprintln!("   or: {} --message-parsing-demo", args.get(0).map_or("trilateration", |s| s.as_str()));
        eprintln!("   or: {} --transceiver-demo", args.get(0).map_or("trilateration", |s| s.as_str()));
        eprintln!("   or: {} --advanced-positioning-demo", args.get(0).map_or("trilateration", |s| s.as_str()));
        eprintln!("   or: {} --error-handling-demo", args.get(0).map_or("trilateration", |s| s.as_str()));
        return Err("Invalid arguments".into());
    }

    let json_path = &args[1];
    let receiver_time_ms = args[2].parse::<u64>()?;

    let json_data = std::fs::read_to_string(json_path)?;
    let anchors_json: AnchorsJson = serde_json::from_str(&json_data)?;

    match trilaterate(&anchors_json.anchors, receiver_time_ms) {
        Ok((geodetic_pos, local_pos)) => {
            println!(
                "Estimated receiver position (local END): x={:.2} m east, y={:.2} m north, z={:.2} m down",
                local_pos.x, local_pos.y, local_pos.z
            );
            println!(
                "Estimated receiver position (lat/lon/depth): lat={:.6}, lon={:.6}, depth={:.2} m",
                geodetic_pos.lat, geodetic_pos.lon, geodetic_pos.depth
            );
        }
        Err(e) => {
            eprintln!("Error during trilateration: {}", e);
            return Err(e.into());
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_trilateration_with_hardcoded_data() {
        let json_data = r#"
        {
          "anchors": [
            {
              "id": "001",
              "timestamp": 1723111199986,
              "position": {
                "lat": 32.12345,
                "long": 45.47675,
                "depth": 0.0
              }
            },
            {
              "id": "002",
              "timestamp": 1723111199988,
              "position": {
                "lat": 32.12365,
                "long": 45.47695,
                "depth": 0.0
              }
            },
            {
              "id": "003",
              "timestamp": 1723111199988,
              "position": {
                "lat": 32.12365,
                "long": 45.47655,
                "depth": 0.0
              }
            },
            {
              "id": "004",
              "timestamp": 1723111199986,
              "position": {
                "lat": 32.12385,
                "long": 45.47675,
                "depth": 0.0
              }
            }
          ]
        }
        "#;

        let anchors_json: AnchorsJson = serde_json::from_str(json_data).unwrap();
        let receiver_time_ms: u64 = 1723111200000;

        let result = trilaterate(&anchors_json.anchors, receiver_time_ms);
        assert!(result.is_ok());

        let (geodetic_pos, local_pos) = result.unwrap();

        // Debug output
        println!(
            "Test 1 - Local position: x={:.2} m east, y={:.2} m north, z={:.2} m down",
            local_pos.x, local_pos.y, local_pos.z
        );
        println!(
            "Test 1 - Geodetic position: lat={:.6}, lon={:.6}, depth={:.2} m",
            geodetic_pos.lat, geodetic_pos.lon, geodetic_pos.depth
        );

        // Check local position
        assert!((local_pos.x - 0.00).abs() < 1e-2);
        assert!((local_pos.y - 22.07).abs() < 1e-2);
        assert!((local_pos.z - 0.00).abs() < 1e-2);

        // Check geodetic position
        assert!((geodetic_pos.lat - 32.123649).abs() < 1e-6);
        assert!((geodetic_pos.lon - 45.476750).abs() < 1e-6);
        assert!((geodetic_pos.depth - 0.00).abs() < 1e-2);
    }

    #[test]
    fn test_trilateration_2() {
        let json_data = r#"
        {
          "anchors": [
            {
              "id": "001",
              "timestamp": 1723111199986,
              "position": {
                "lat": 32.12345,
                "long": 45.47675,
                "depth": 6.5
              }
            },
            {
              "id": "002",
              "timestamp": 1723111199988,
              "position": {
                "lat": 32.12365,
                "long": 45.47695,
                "depth": 15.0
              }
            },
            {
              "id": "003",
              "timestamp": 1723111199988,
              "position": {
                "lat": 32.12365,
                "long": 45.47655,
                "depth": 25.0
              }
            },
            {
              "id": "004",
              "timestamp": 1723111199986,
              "position": {
                "lat": 32.12385,
                "long": 45.47675,
                "depth": 12.6
              }
            }
          ]
        }
        "#;

        let anchors_json: AnchorsJson = serde_json::from_str(json_data).unwrap();
        let receiver_time_ms: u64 = 1723111200000;

        let result = trilaterate(&anchors_json.anchors, receiver_time_ms);
        assert!(result.is_ok());

        let (geodetic_pos, local_pos) = result.unwrap();

        println!(
            "Estimated receiver position (local END): x={:.2} m east, y={:.2} m north, z={:.2} m down",
            local_pos.x, local_pos.y, local_pos.z
        );
        println!(
            "Estimated receiver position (lat/lon/depth): lat={:.6}, lon={:.6}, depth={:.2} m",
            geodetic_pos.lat, geodetic_pos.lon, geodetic_pos.depth
        );

        // Check local position
        assert!((local_pos.x - -1.46).abs() < 1e-2);
        assert!((local_pos.y - 21.55).abs() < 1e-2);
        assert!((local_pos.z - 14.50).abs() < 1e-2);

        // Check geodetic position
        assert!((geodetic_pos.lat - 32.123644).abs() < 1e-6);
        assert!((geodetic_pos.lon - 45.476735).abs() < 1e-6);
        assert!((geodetic_pos.depth - 14.50).abs() < 1e-2);
    }

    #[test]
    fn test_trilateration_nearly_collinear_anchors() {
        // Test case with anchors that are nearly collinear (along a line)
        // Should still produce a result but with reduced accuracy
        let json_data = r#"
        {
          "anchors": [
            {
              "id": "001",
              "timestamp": 1723111199986,
              "position": {
                "lat": 32.12345,
                "long": 45.47675,
                "depth": 0.0
              }
            },
            {
              "id": "002",
              "timestamp": 1723111199988,
              "position": {
                "lat": 32.12346,
                "long": 45.47676,
                "depth": 0.0
              }
            },
            {
              "id": "003",
              "timestamp": 1723111199988,
              "position": {
                "lat": 32.12347,
                "long": 45.47677,
                "depth": 0.0
              }
            },
            {
              "id": "004",
              "timestamp": 1723111199986,
              "position": {
                "lat": 32.12348,
                "long": 45.47678,
                "depth": 0.0
              }
            }
          ]
        }
        "#;

        let anchors_json: AnchorsJson = serde_json::from_str(json_data).unwrap();
        let receiver_time_ms: u64 = 1723111200000;

        let result = trilaterate(&anchors_json.anchors, receiver_time_ms);
        
        // Should produce a result even with nearly collinear anchors
        assert!(result.is_ok());
        
        let (geodetic_pos, local_pos) = result.unwrap();
        
        println!(
            "Nearly collinear anchors result: lat={:.6}, lon={:.6}, depth={:.2} m",
            geodetic_pos.lat, geodetic_pos.lon, geodetic_pos.depth
        );
        println!(
            "Local coordinates: x={:.2} m east, y={:.2} m north, z={:.2} m down",
            local_pos.x, local_pos.y, local_pos.z
        );
        
        // For nearly collinear anchors, we expect reduced accuracy (5-10 meters)
        // Check that the result is within reasonable bounds of the anchor area
        let anchor_center_lat = 32.123465; // Average of anchor latitudes
        let anchor_center_lon = 45.476765; // Average of anchor longitudes
        
        // Allow up to ~10 meters error (roughly 0.0001 degrees)
        assert!((geodetic_pos.lat - anchor_center_lat).abs() < 0.0001);
        assert!((geodetic_pos.lon - anchor_center_lon).abs() < 0.0001);
        assert!(geodetic_pos.depth.abs() < 10.0);
    }

    #[test]
    fn test_trilateration_three_anchors_only() {
        // Test case with only 3 anchors (one signal blocked by terrain)
        // Should produce 2D solution with estimated depth
        let json_data = r#"
        {
          "anchors": [
            {
              "id": "001",
              "timestamp": 1723111199986,
              "position": {
                "lat": 32.12345,
                "long": 45.47675,
                "depth": 5.0
              }
            },
            {
              "id": "002",
              "timestamp": 1723111199988,
              "position": {
                "lat": 32.12365,
                "long": 45.47695,
                "depth": 10.0
              }
            },
            {
              "id": "003",
              "timestamp": 1723111199988,
              "position": {
                "lat": 32.12365,
                "long": 45.47655,
                "depth": 15.0
              }
            }
          ]
        }
        "#;

        let anchors_json: AnchorsJson = serde_json::from_str(json_data).unwrap();
        let receiver_time_ms: u64 = 1723111200000;

        let result = trilaterate(&anchors_json.anchors, receiver_time_ms);
        
        // Should produce a result with 3 anchors
        assert!(result.is_ok());
        
        let (geodetic_pos, local_pos) = result.unwrap();
        
        println!(
            "3-anchor result: lat={:.6}, lon={:.6}, depth={:.2} m",
            geodetic_pos.lat, geodetic_pos.lon, geodetic_pos.depth
        );
        println!(
            "Local coordinates: x={:.2} m east, y={:.2} m north, z={:.2} m down",
            local_pos.x, local_pos.y, local_pos.z
        );
        
        // Check that lat/lon are reasonable (similar to 4-anchor test but may be less accurate)
        // The depth should be approximately the weighted average of anchor depths
        assert!((local_pos.x - 0.00).abs() < 5.0); // Allow more error for 3-anchor case
        assert!((local_pos.y - 22.0).abs() < 5.0);
        
        // Depth should be somewhere between min and max anchor depths
        assert!(geodetic_pos.depth >= 5.0 && geodetic_pos.depth <= 15.0);
        
        // Check geodetic position is in reasonable range
        assert!((geodetic_pos.lat - 32.1236).abs() < 0.0001);
        assert!((geodetic_pos.lon - 45.4768).abs() < 0.0001);
    }

    #[test]
    fn test_fixed_point_arithmetic() {
        // Test basic fixed-point operations
        let a = FixedPoint32::new(3.5, 16);
        let b = FixedPoint32::new(2.25, 16);
        
        // Test conversion back to f64
        assert!((a.to_f64() - 3.5).abs() < 1e-4);
        assert!((b.to_f64() - 2.25).abs() < 1e-4);
        
        // Test addition
        let sum = a.add(b);
        assert!((sum.to_f64() - 5.75).abs() < 1e-4);
        
        // Test subtraction
        let diff = a.sub(b);
        assert!((diff.to_f64() - 1.25).abs() < 1e-4);
        
        // Test multiplication
        let prod = a.mul(b);
        assert!((prod.to_f64() - 7.875).abs() < 1e-3); // Allow slightly more error for multiplication
    }

    #[test]
    fn test_local_position() {
        let pos = LocalPosition::new(123.456, -78.9, 45.67);
        
        // Test conversion to meters
        let (east, north, down) = pos.to_meters();
        assert!((east - 123.456).abs() < 1e-3);
        assert!((north - (-78.9)).abs() < 1e-3);
        assert!((down - 45.67).abs() < 1e-3);
        
        // Test conversion to Vector3
        let vec = pos.to_vector3();
        assert!((vec.x - 123.456).abs() < 1e-3);
        assert!((vec.y - (-78.9)).abs() < 1e-3);
        assert!((vec.z - 45.67).abs() < 1e-3);
    }

    #[test]
    fn test_compact_anchor_message() {
        // Reset base timestamp for this test
        CompactAnchorMessage::set_base_timestamp(0);
        
        let test_timestamp = 1723111199986u64;
        let msg = CompactAnchorMessage::new(
            42,
            test_timestamp,
            32.123456,
            45.476789,
            12.345,
            200
        );
        
        // Test that data is preserved with expected precision
        // Copy values to avoid unaligned access to packed struct
        let anchor_id = msg.anchor_id;
        let quality = msg.quality;
        assert_eq!(anchor_id, 42);
        assert_eq!(msg.get_timestamp(), test_timestamp);
        assert_eq!(quality, 200);
        
        let pos = msg.get_position();
        assert!((pos.lat - 32.123456).abs() < 1e-6);
        assert!((pos.lon - 45.476789).abs() < 1e-6);
        assert!((pos.depth - 12.345).abs() < 1e-3); // mm precision
        
        // Test relative timestamp functionality
        let msg2 = CompactAnchorMessage::new(
            43,
            test_timestamp + 14, // 14ms later
            32.0,
            45.0,
            10.0,
            255
        );
        
        let timestamp2 = msg2.get_timestamp();
        assert_eq!(timestamp2, test_timestamp + 14);
        
        // Test that relative timestamps work correctly
        let relative_ts = msg2.timestamp_ms;
        assert_eq!(relative_ts, 14); // Should be 14ms relative to base
    }

    #[test]
    fn test_anchor_buffer() {
        // Reset base timestamp for this test
        CompactAnchorMessage::set_base_timestamp(0);
        
        let mut buffer: AnchorBuffer<4> = AnchorBuffer::new();
        
        // Test empty buffer
        assert!(buffer.is_empty());
        assert_eq!(buffer.len(), 0);
        
        // Add messages with timestamps that work with the base timestamp system
        let base_time = 2000000000u64; // Use a different base time to avoid conflicts
        CompactAnchorMessage::set_base_timestamp(base_time);
        
        let msg1 = CompactAnchorMessage::new(1, base_time + 1000, 32.0, 45.0, 10.0, 255);
        let msg2 = CompactAnchorMessage::new(2, base_time + 2000, 32.1, 45.1, 11.0, 254);
        let msg3 = CompactAnchorMessage::new(3, base_time + 3000, 32.2, 45.2, 12.0, 253);
        
        buffer.push(msg1);
        buffer.push(msg2);
        buffer.push(msg3);
        
        assert_eq!(buffer.len(), 3);
        assert!(!buffer.is_empty());
        
        // Test getting recent messages
        let current_time = base_time + 4000;
        let recent = buffer.get_recent_messages(5000, current_time);
        let mut count = 0;
        for msg in recent.iter() {
            if msg.is_some() {
                count += 1;
            }
        }
        assert_eq!(count, 3); // All messages should be recent
        
        // Test age filtering - messages older than 1500ms from current_time should be filtered
        // msg1 is 3000ms old, msg2 is 2000ms old, msg3 is 1000ms old
        let recent_strict = buffer.get_recent_messages(1500, current_time);
        let mut count_strict = 0;
        for msg in recent_strict.iter() {
            if msg.is_some() {
                count_strict += 1;
            }
        }
        // Only msg3 (1000ms old) should be within 1500ms threshold
        assert_eq!(count_strict, 1);
        
        // Test circular buffer overflow
        let msg4 = CompactAnchorMessage::new(4, base_time + 4000, 32.3, 45.3, 13.0, 252);
        let msg5 = CompactAnchorMessage::new(5, base_time + 5000, 32.4, 45.4, 14.0, 251);
        buffer.push(msg4);
        buffer.push(msg5);
        
        assert_eq!(buffer.len(), 4); // Should be at capacity
        
        // Clear buffer
        buffer.clear();
        assert!(buffer.is_empty());
        assert_eq!(buffer.len(), 0);
    }

    #[test]
    fn test_embedded_anchor_config() {
        let config = EmbeddedAnchorConfig::new(123, 32.123, 45.456, 15.5, 1000.0);
        
        assert_eq!(config.id, 123);
        assert!(config.enabled);
        assert!((config.get_max_range_m() - 1000.0).abs() < 1e-3);
        
        let pos = config.position.get_position();
        assert!((pos.lat - 32.123).abs() < 1e-6);
        assert!((pos.lon - 45.456).abs() < 1e-6);
        assert!((pos.depth - 15.5).abs() < 1e-3);
    }

    #[test]
    fn test_embedded_system_config() {
        let config = EmbeddedSystemConfig::new();
        
        // Test default values
        assert_eq!(config.min_anchors, 3);
        assert_eq!(config.position_timeout_ms, 200);
        assert_eq!(config.max_anchor_age_ms, 5000);
        
        // Test conversions
        assert!((config.get_sound_speed_m_per_s() - 1500.0).abs() < 1e-3);
        assert!((config.get_accuracy_threshold_m() - 2.0).abs() < 1e-3);
    }

    #[test]
    fn test_memory_footprint() {
        use std::mem;
        
        // Test that compact structures are indeed smaller
        let compact_size = mem::size_of::<CompactAnchorMessage>();
        let legacy_anchor_size = mem::size_of::<Anchor>();
        let legacy_position_size = mem::size_of::<Position>();
        
        println!("CompactAnchorMessage size: {} bytes", compact_size);
        println!("Legacy Anchor size: {} bytes", legacy_anchor_size);
        println!("Legacy Position size: {} bytes", legacy_position_size);
        
        // CompactAnchorMessage should be significantly smaller than Anchor + Position
        // CompactAnchorMessage: 2+4+4+4+2+1 = 17 bytes (plus padding)
        // Anchor contains String + u64 + Position (3*f64) = String(24) + 8 + 24 = 56+ bytes
        assert!(compact_size < 32); // Should be much smaller than legacy structures
        
        // Test LocalPosition size
        let local_pos_size = mem::size_of::<LocalPosition>();
        println!("LocalPosition size: {} bytes", local_pos_size);
        assert_eq!(local_pos_size, 12); // 3 * i32 = 12 bytes
        
        // Test FixedPoint32 size
        let fixed_point_size = mem::size_of::<FixedPoint32>();
        println!("FixedPoint32 size: {} bytes", fixed_point_size);
        assert_eq!(fixed_point_size, 8); // i32 + u8 + padding = 8 bytes
    }
} 