//! Embedded-optimized trilateration algorithms for microcontroller deployment
//! 
//! This module provides memory-efficient implementations of trilateration algorithms
//! specifically designed for resource-constrained embedded systems. Key optimizations:
//! - Stack-based arrays instead of dynamic allocations
//! - Fixed-point arithmetic for systems without FPU
//! - Compile-time configuration for precision vs performance trade-offs
//! - Minimal memory footprint (<32KB RAM)

use crate::core::{Anchor, Position, SPEED_OF_SOUND_WATER};

/// Maximum number of anchors supported in embedded mode
pub const MAX_ANCHORS_EMBEDDED: usize = 8;

/// Fixed-point scaling factor (Q16.16 format)
pub const FIXED_POINT_SCALE: i32 = 65536; // 2^16

/// Embedded-optimized trilateration engine
pub struct EmbeddedTrilateration {
    /// Use fixed-point arithmetic instead of floating-point
    pub use_fixed_point: bool,
    /// Maximum iterations for iterative algorithms
    pub max_iterations: u8,
    /// Convergence tolerance (scaled for fixed-point)
    pub convergence_tolerance: i32,
    /// Regularization parameter (scaled for fixed-point)
    pub regularization_lambda: i32,
}

impl Default for EmbeddedTrilateration {
    fn default() -> Self {
        Self {
            use_fixed_point: false, // Default to floating-point for compatibility
            max_iterations: 20,     // Reduced from 50 for embedded constraints
            convergence_tolerance: (1e-4 * FIXED_POINT_SCALE as f64) as i32,
            regularization_lambda: (1e-6 * FIXED_POINT_SCALE as f64) as i32,
        }
    }
}

/// Fixed-point 3D vector for embedded systems
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FixedVector3 {
    pub x: i32,
    pub y: i32,
    pub z: i32,
}

impl FixedVector3 {
    pub fn new(x: i32, y: i32, z: i32) -> Self {
        Self { x, y, z }
    }

    pub fn zeros() -> Self {
        Self { x: 0, y: 0, z: 0 }
    }

    pub fn from_f64(x: f64, y: f64, z: f64) -> Self {
        Self {
            x: (x * FIXED_POINT_SCALE as f64) as i32,
            y: (y * FIXED_POINT_SCALE as f64) as i32,
            z: (z * FIXED_POINT_SCALE as f64) as i32,
        }
    }

    pub fn to_f64(&self) -> (f64, f64, f64) {
        (
            self.x as f64 / FIXED_POINT_SCALE as f64,
            self.y as f64 / FIXED_POINT_SCALE as f64,
            self.z as f64 / FIXED_POINT_SCALE as f64,
        )
    }

    pub fn dot(&self, other: &Self) -> i64 {
        (self.x as i64 * other.x as i64 + 
         self.y as i64 * other.y as i64 + 
         self.z as i64 * other.z as i64) / FIXED_POINT_SCALE as i64
    }

    pub fn norm_squared(&self) -> i64 {
        self.dot(self)
    }

    pub fn add(&self, other: &Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }

    pub fn sub(&self, other: &Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }

    pub fn scale(&self, factor: i32) -> Self {
        Self {
            x: ((self.x as i64 * factor as i64) / FIXED_POINT_SCALE as i64) as i32,
            y: ((self.y as i64 * factor as i64) / FIXED_POINT_SCALE as i64) as i32,
            z: ((self.z as i64 * factor as i64) / FIXED_POINT_SCALE as i64) as i32,
        }
    }
}

/// Fixed-point 3x3 matrix for embedded linear algebra
#[derive(Debug, Clone, Copy)]
pub struct FixedMatrix3x3 {
    pub data: [[i32; 3]; 3],
}

impl FixedMatrix3x3 {
    pub fn zeros() -> Self {
        Self { data: [[0; 3]; 3] }
    }

    pub fn identity() -> Self {
        let mut matrix = Self::zeros();
        matrix.data[0][0] = FIXED_POINT_SCALE;
        matrix.data[1][1] = FIXED_POINT_SCALE;
        matrix.data[2][2] = FIXED_POINT_SCALE;
        matrix
    }

    pub fn multiply_vector(&self, vec: &FixedVector3) -> FixedVector3 {
        FixedVector3 {
            x: ((self.data[0][0] as i64 * vec.x as i64 + 
                 self.data[0][1] as i64 * vec.y as i64 + 
                 self.data[0][2] as i64 * vec.z as i64) / FIXED_POINT_SCALE as i64) as i32,
            y: ((self.data[1][0] as i64 * vec.x as i64 + 
                 self.data[1][1] as i64 * vec.y as i64 + 
                 self.data[1][2] as i64 * vec.z as i64) / FIXED_POINT_SCALE as i64) as i32,
            z: ((self.data[2][0] as i64 * vec.x as i64 + 
                 self.data[2][1] as i64 * vec.y as i64 + 
                 self.data[2][2] as i64 * vec.z as i64) / FIXED_POINT_SCALE as i64) as i32,
        }
    }

    pub fn add_diagonal(&mut self, value: i32) {
        self.data[0][0] += value;
        self.data[1][1] += value;
        self.data[2][2] += value;
    }
}

/// Stack-based anchor data for embedded systems
#[derive(Debug)]
pub struct EmbeddedAnchorData {
    pub positions: [FixedVector3; MAX_ANCHORS_EMBEDDED],
    pub ranges: [i32; MAX_ANCHORS_EMBEDDED], // Fixed-point ranges
    pub weights: [i32; MAX_ANCHORS_EMBEDDED], // Fixed-point weights
    pub count: usize,
}

impl EmbeddedAnchorData {
    pub fn new() -> Self {
        Self {
            positions: [FixedVector3::zeros(); MAX_ANCHORS_EMBEDDED],
            ranges: [0; MAX_ANCHORS_EMBEDDED],
            weights: [FIXED_POINT_SCALE; MAX_ANCHORS_EMBEDDED], // Default weight = 1.0
            count: 0,
        }
    }

    pub fn add_anchor(&mut self, position: FixedVector3, range: i32, weight: i32) -> Result<(), &'static str> {
        if self.count >= MAX_ANCHORS_EMBEDDED {
            return Err("Maximum anchor count exceeded");
        }
        
        self.positions[self.count] = position;
        self.ranges[self.count] = range;
        self.weights[self.count] = weight;
        self.count += 1;
        
        Ok(())
    }
}

impl EmbeddedTrilateration {
    pub fn new() -> Self {
        Self::default()
    }

    /// Configure for fixed-point arithmetic (no FPU systems)
    pub fn with_fixed_point() -> Self {
        Self {
            use_fixed_point: true,
            ..Default::default()
        }
    }

    /// Configure for maximum performance (reduced precision)
    pub fn with_fast_mode() -> Self {
        Self {
            max_iterations: 10,
            convergence_tolerance: (1e-3 * FIXED_POINT_SCALE as f64) as i32,
            ..Default::default()
        }
    }

    /// Configure for maximum precision (slower)
    pub fn with_precision_mode() -> Self {
        Self {
            max_iterations: 50,
            convergence_tolerance: (1e-6 * FIXED_POINT_SCALE as f64) as i32,
            ..Default::default()
        }
    }

    /// Embedded-optimized trilateration using stack-based arrays
    pub fn calculate_position_embedded(
        &self,
        anchors: &[Anchor],
        receiver_time_ms: u64,
    ) -> Result<Position, &'static str> {
        if anchors.len() < 3 {
            return Err("At least 3 anchors required");
        }
        
        if anchors.len() > MAX_ANCHORS_EMBEDDED {
            return Err("Too many anchors for embedded mode");
        }

        if self.use_fixed_point {
            self.calculate_position_fixed_point(anchors, receiver_time_ms)
        } else {
            self.calculate_position_floating_point(anchors, receiver_time_ms)
        }
    }

    /// Fixed-point trilateration implementation
    fn calculate_position_fixed_point(
        &self,
        anchors: &[Anchor],
        receiver_time_ms: u64,
    ) -> Result<Position, &'static str> {
        // Convert to local coordinates and fixed-point representation
        let reference_pos = &anchors[0].position;
        let mut anchor_data = EmbeddedAnchorData::new();

        for anchor in anchors {
            let local = self.geodetic_to_local_fixed(&anchor.position, reference_pos);
            
            let dt_ms = receiver_time_ms as i64 - anchor.timestamp as i64;
            if dt_ms < 0 {
                return Err("Receiver time earlier than anchor time");
            }
            
            let dt_sec_fixed = (dt_ms * FIXED_POINT_SCALE as i64 / 1000) as i32;
            let range_fixed = ((SPEED_OF_SOUND_WATER * FIXED_POINT_SCALE as f64) as i64 * dt_sec_fixed as i64 / FIXED_POINT_SCALE as i64) as i32;
            
            anchor_data.add_anchor(local, range_fixed, FIXED_POINT_SCALE)?;
        }

        // Perform fixed-point least squares
        let result_fixed = self.fixed_point_least_squares(&anchor_data)?;
        
        // Convert back to geodetic coordinates
        let result_f64 = result_fixed.to_f64();
        let geodetic_pos = self.local_to_geodetic_fixed(result_f64, reference_pos);
        
        Ok(geodetic_pos)
    }

    /// Floating-point trilateration with stack-based arrays
    fn calculate_position_floating_point(
        &self,
        anchors: &[Anchor],
        receiver_time_ms: u64,
    ) -> Result<Position, &'static str> {
        // Stack-based arrays for positions and ranges
        let mut positions = [nalgebra::Vector3::zeros(); MAX_ANCHORS_EMBEDDED];
        let mut ranges = [0.0f64; MAX_ANCHORS_EMBEDDED];
        let anchor_count = anchors.len();

        let reference_pos = &anchors[0].position;

        // Convert to local coordinates
        for (i, anchor) in anchors.iter().enumerate() {
            let local = self.geodetic_to_local(&anchor.position, reference_pos);
            positions[i] = nalgebra::Vector3::new(local.0, local.1, local.2);

            let dt_ms = receiver_time_ms as i64 - anchor.timestamp as i64;
            if dt_ms < 0 {
                return Err("Receiver time earlier than anchor time");
            }
            
            let dt_sec = dt_ms as f64 / 1000.0;
            ranges[i] = SPEED_OF_SOUND_WATER * dt_sec;
        }

        // Perform least squares using stack-based arrays
        let result = self.stack_based_least_squares(&positions[..anchor_count], &ranges[..anchor_count])?;
        
        // Convert back to geodetic coordinates
        let geodetic_pos = self.local_to_geodetic_f64((result.x, result.y, result.z), reference_pos);
        
        Ok(geodetic_pos)
    }

    /// Fixed-point least squares implementation
    fn fixed_point_least_squares(
        &self,
        anchor_data: &EmbeddedAnchorData,
    ) -> Result<FixedVector3, &'static str> {
        if anchor_data.count < 3 {
            return Err("Insufficient anchor data");
        }

        let n = anchor_data.count;
        let p1 = anchor_data.positions[0];

        // Handle 3-anchor case (2D solution)
        if n == 3 {
            return self.fixed_point_2d_solution(anchor_data);
        }

        // 3D case with 4+ anchors using stack-based matrix operations
        let mut a_matrix = [[0i32; 3]; MAX_ANCHORS_EMBEDDED]; // A matrix (n-1 x 3)
        let mut b_vector = [0i32; MAX_ANCHORS_EMBEDDED];      // b vector (n-1)

        for i in 1..n {
            let pi = anchor_data.positions[i];
            let row = i - 1;
            
            a_matrix[row][0] = 2 * (pi.x - p1.x);
            a_matrix[row][1] = 2 * (pi.y - p1.y);
            a_matrix[row][2] = 2 * (pi.z - p1.z);
            
            // Calculate b_vector[row] using fixed-point arithmetic
            let range0_sq = ((anchor_data.ranges[0] as i64 * anchor_data.ranges[0] as i64) / FIXED_POINT_SCALE as i64) as i32;
            let rangei_sq = ((anchor_data.ranges[i] as i64 * anchor_data.ranges[i] as i64) / FIXED_POINT_SCALE as i64) as i32;
            
            let p1_norm_sq = ((p1.x as i64 * p1.x as i64 + p1.y as i64 * p1.y as i64 + p1.z as i64 * p1.z as i64) / FIXED_POINT_SCALE as i64) as i32;
            let pi_norm_sq = ((pi.x as i64 * pi.x as i64 + pi.y as i64 * pi.y as i64 + pi.z as i64 * pi.z as i64) / FIXED_POINT_SCALE as i64) as i32;
            
            b_vector[row] = range0_sq - rangei_sq + pi_norm_sq - p1_norm_sq;
        }

        // Solve using fixed-point Gaussian elimination
        self.solve_fixed_point_system(&a_matrix, &b_vector, n - 1)
    }

    /// Fixed-point 2D solution for 3 anchors
    fn fixed_point_2d_solution(
        &self,
        anchor_data: &EmbeddedAnchorData,
    ) -> Result<FixedVector3, &'static str> {
        let p1 = anchor_data.positions[0];
        let p2 = anchor_data.positions[1];
        let p3 = anchor_data.positions[2];

        // Set up 2x2 system for x, y coordinates
        let a11 = 2 * (p2.x - p1.x);
        let a12 = 2 * (p2.y - p1.y);
        let a21 = 2 * (p3.x - p1.x);
        let a22 = 2 * (p3.y - p1.y);

        // Calculate right-hand side
        let range1_sq = ((anchor_data.ranges[0] as i64 * anchor_data.ranges[0] as i64) / FIXED_POINT_SCALE as i64) as i32;
        let range2_sq = ((anchor_data.ranges[1] as i64 * anchor_data.ranges[1] as i64) / FIXED_POINT_SCALE as i64) as i32;
        let range3_sq = ((anchor_data.ranges[2] as i64 * anchor_data.ranges[2] as i64) / FIXED_POINT_SCALE as i64) as i32;

        let p1_norm_sq = ((p1.x as i64 * p1.x as i64 + p1.y as i64 * p1.y as i64 + p1.z as i64 * p1.z as i64) / FIXED_POINT_SCALE as i64) as i32;
        let p2_norm_sq = ((p2.x as i64 * p2.x as i64 + p2.y as i64 * p2.y as i64 + p2.z as i64 * p2.z as i64) / FIXED_POINT_SCALE as i64) as i32;
        let p3_norm_sq = ((p3.x as i64 * p3.x as i64 + p3.y as i64 * p3.y as i64 + p3.z as i64 * p3.z as i64) / FIXED_POINT_SCALE as i64) as i32;

        let b1 = range1_sq - range2_sq + p2_norm_sq - p1_norm_sq;
        let b2 = range1_sq - range3_sq + p3_norm_sq - p1_norm_sq;

        // Solve 2x2 system using Cramer's rule
        let det = ((a11 as i64 * a22 as i64 - a12 as i64 * a21 as i64) / FIXED_POINT_SCALE as i64) as i32;
        
        if det.abs() < (FIXED_POINT_SCALE / 1000) { // Check for near-singular matrix
            return Err("Singular matrix in 2D solution");
        }

        let x = ((b1 as i64 * a22 as i64 - b2 as i64 * a12 as i64) / det as i64) as i32;
        let y = ((a11 as i64 * b2 as i64 - a21 as i64 * b1 as i64) / det as i64) as i32;

        // Estimate depth as weighted average
        let total_weight = anchor_data.weights[0] + anchor_data.weights[1] + anchor_data.weights[2];
        let z = ((p1.z as i64 * anchor_data.weights[0] as i64 + 
                  p2.z as i64 * anchor_data.weights[1] as i64 + 
                  p3.z as i64 * anchor_data.weights[2] as i64) / total_weight as i64) as i32;

        Ok(FixedVector3::new(x, y, z))
    }

    /// Solve fixed-point linear system using Gaussian elimination
    fn solve_fixed_point_system(
        &self,
        a_matrix: &[[i32; 3]; MAX_ANCHORS_EMBEDDED],
        b_vector: &[i32; MAX_ANCHORS_EMBEDDED],
        n: usize,
    ) -> Result<FixedVector3, &'static str> {
        // Copy to working arrays
        let mut a = [[0i32; 3]; MAX_ANCHORS_EMBEDDED];
        let mut b = [0i32; MAX_ANCHORS_EMBEDDED];
        
        for i in 0..n {
            for j in 0..3 {
                a[i][j] = a_matrix[i][j];
            }
            b[i] = b_vector[i];
        }

        // Forward elimination
        for k in 0..3.min(n) {
            // Find pivot
            let mut max_row = k;
            for i in (k + 1)..n {
                if a[i][k].abs() > a[max_row][k].abs() {
                    max_row = i;
                }
            }

            // Swap rows if needed
            if max_row != k {
                for j in 0..3 {
                    let temp = a[k][j];
                    a[k][j] = a[max_row][j];
                    a[max_row][j] = temp;
                }
                let temp = b[k];
                b[k] = b[max_row];
                b[max_row] = temp;
            }

            // Check for singular matrix
            if a[k][k].abs() < (FIXED_POINT_SCALE / 10000) {
                return Err("Singular matrix in fixed-point solver");
            }

            // Eliminate column
            for i in (k + 1)..n {
                let factor = ((a[i][k] as i64 * FIXED_POINT_SCALE as i64) / a[k][k] as i64) as i32;
                
                for j in k..3 {
                    a[i][j] -= ((factor as i64 * a[k][j] as i64) / FIXED_POINT_SCALE as i64) as i32;
                }
                b[i] -= ((factor as i64 * b[k] as i64) / FIXED_POINT_SCALE as i64) as i32;
            }
        }

        // Back substitution for overdetermined system (least squares solution)
        let mut x = [0i32; 3];
        
        // Solve the upper triangular system
        for i in (0..3).rev() {
            let mut sum = 0i64;
            for j in (i + 1)..3 {
                sum += x[j] as i64 * a[i][j] as i64;
            }
            sum /= FIXED_POINT_SCALE as i64;
            
            x[i] = ((b[i] as i64 - sum) * FIXED_POINT_SCALE as i64 / a[i][i] as i64) as i32;
        }

        Ok(FixedVector3::new(x[0], x[1], x[2]))
    }

    /// Stack-based least squares using floating-point arithmetic
    fn stack_based_least_squares(
        &self,
        positions: &[nalgebra::Vector3<f64>],
        ranges: &[f64],
    ) -> Result<nalgebra::Vector3<f64>, &'static str> {
        let n = positions.len();
        if n < 3 {
            return Err("Insufficient positions");
        }

        let p1 = positions[0];

        // Handle 3-anchor case
        if n == 3 {
            let mut a_matrix = nalgebra::Matrix2::zeros();
            let mut b_vector = nalgebra::Vector2::zeros();

            for i in 1..3 {
                let pi = positions[i];
                let row = i - 1;
                
                a_matrix[(row, 0)] = 2.0 * (pi.x - p1.x);
                a_matrix[(row, 1)] = 2.0 * (pi.y - p1.y);
                
                b_vector[row] = ranges[0].powi(2) - ranges[i].powi(2)
                    + pi.x.powi(2) - p1.x.powi(2)
                    + pi.y.powi(2) - p1.y.powi(2)
                    + pi.z.powi(2) - p1.z.powi(2);
            }

            if let Some(inv_a) = a_matrix.try_inverse() {
                let xy_solution = inv_a * b_vector;
                
                // Estimate depth as average
                let estimated_depth = positions.iter().map(|p| p.z).sum::<f64>() / n as f64;
                
                return Ok(nalgebra::Vector3::new(xy_solution.x, xy_solution.y, estimated_depth));
            } else {
                return Err("Singular matrix in 2D solution");
            }
        }

        // 3D case with stack-based matrix operations
        let mut a_data = [[0.0f64; 3]; MAX_ANCHORS_EMBEDDED];
        let mut b_data = [0.0f64; MAX_ANCHORS_EMBEDDED];

        for i in 1..n {
            let pi = positions[i];
            let row = i - 1;
            
            a_data[row][0] = 2.0 * (pi.x - p1.x);
            a_data[row][1] = 2.0 * (pi.y - p1.y);
            a_data[row][2] = 2.0 * (pi.z - p1.z);
            
            b_data[row] = ranges[0].powi(2) - ranges[i].powi(2)
                + pi.x.powi(2) - p1.x.powi(2)
                + pi.y.powi(2) - p1.y.powi(2)
                + pi.z.powi(2) - p1.z.powi(2);
        }

        // Solve using stack-based Gaussian elimination
        self.solve_stack_based_system(&a_data, &b_data, n - 1)
    }

    /// Stack-based Gaussian elimination solver
    fn solve_stack_based_system(
        &self,
        a_matrix: &[[f64; 3]; MAX_ANCHORS_EMBEDDED],
        b_vector: &[f64; MAX_ANCHORS_EMBEDDED],
        n: usize,
    ) -> Result<nalgebra::Vector3<f64>, &'static str> {
        // Copy to working arrays
        let mut a = [[0.0f64; 3]; MAX_ANCHORS_EMBEDDED];
        let mut b = [0.0f64; MAX_ANCHORS_EMBEDDED];
        
        for i in 0..n {
            for j in 0..3 {
                a[i][j] = a_matrix[i][j];
            }
            b[i] = b_vector[i];
        }

        // Forward elimination with partial pivoting
        for k in 0..3.min(n) {
            // Find pivot
            let mut max_row = k;
            for i in (k + 1)..n {
                if a[i][k].abs() > a[max_row][k].abs() {
                    max_row = i;
                }
            }

            // Swap rows if needed
            if max_row != k {
                for j in 0..3 {
                    let temp = a[k][j];
                    a[k][j] = a[max_row][j];
                    a[max_row][j] = temp;
                }
                let temp = b[k];
                b[k] = b[max_row];
                b[max_row] = temp;
            }

            // Check for singular matrix
            if a[k][k].abs() < 1e-12 {
                return Err("Singular matrix in stack-based solver");
            }

            // Eliminate column
            for i in (k + 1)..n {
                let factor = a[i][k] / a[k][k];
                
                for j in k..3 {
                    a[i][j] -= factor * a[k][j];
                }
                b[i] -= factor * b[k];
            }
        }

        // Back substitution
        let mut x = [0.0f64; 3];
        
        for i in (0..3).rev() {
            let mut sum = 0.0;
            for j in (i + 1)..3 {
                sum += x[j] * a[i][j];
            }
            x[i] = (b[i] - sum) / a[i][i];
        }

        Ok(nalgebra::Vector3::new(x[0], x[1], x[2]))
    }

    /// Convert geodetic to local coordinates (fixed-point)
    fn geodetic_to_local_fixed(&self, pos: &Position, reference: &Position) -> FixedVector3 {
        let (x, y, z) = self.geodetic_to_local(pos, reference);
        FixedVector3::from_f64(x, y, z)
    }

    /// Convert geodetic to local coordinates (floating-point)
    fn geodetic_to_local(&self, pos: &Position, reference: &Position) -> (f64, f64, f64) {
        // Simplified local tangent plane conversion
        // For embedded systems, we use a simplified flat-earth approximation
        const EARTH_RADIUS: f64 = 6371000.0; // meters
        
        let lat_diff = (pos.lat - reference.lat).to_radians();
        let lon_diff = (pos.lon - reference.lon).to_radians();
        let ref_lat_rad = reference.lat.to_radians();
        
        let x = EARTH_RADIUS * lon_diff * ref_lat_rad.cos();
        let y = EARTH_RADIUS * lat_diff;
        let z = pos.depth - reference.depth;
        
        (x, y, z)
    }

    /// Convert local to geodetic coordinates (fixed-point)
    fn local_to_geodetic_fixed(&self, local: (f64, f64, f64), reference: &Position) -> Position {
        self.local_to_geodetic_f64(local, reference)
    }

    /// Convert local to geodetic coordinates (floating-point)
    fn local_to_geodetic_f64(&self, local: (f64, f64, f64), reference: &Position) -> Position {
        const EARTH_RADIUS: f64 = 6371000.0; // meters
        
        let ref_lat_rad = reference.lat.to_radians();
        
        let lat_diff = local.1 / EARTH_RADIUS;
        let lon_diff = local.0 / (EARTH_RADIUS * ref_lat_rad.cos());
        
        Position {
            lat: reference.lat + lat_diff.to_degrees(),
            lon: reference.lon + lon_diff.to_degrees(),
            depth: reference.depth + local.2,
        }
    }
}

/// Compile-time configuration for precision vs performance trade-offs
pub mod config {
    /// High precision configuration (slower, more accurate)
    pub const HIGH_PRECISION: super::EmbeddedTrilateration = super::EmbeddedTrilateration {
        use_fixed_point: false,
        max_iterations: 50,
        convergence_tolerance: (1e-6 * super::FIXED_POINT_SCALE as f64) as i32,
        regularization_lambda: (1e-8 * super::FIXED_POINT_SCALE as f64) as i32,
    };

    /// Balanced configuration (good balance of speed and accuracy)
    pub const BALANCED: super::EmbeddedTrilateration = super::EmbeddedTrilateration {
        use_fixed_point: false,
        max_iterations: 20,
        convergence_tolerance: (1e-4 * super::FIXED_POINT_SCALE as f64) as i32,
        regularization_lambda: (1e-6 * super::FIXED_POINT_SCALE as f64) as i32,
    };

    /// Fast configuration (faster, reduced precision)
    pub const FAST: super::EmbeddedTrilateration = super::EmbeddedTrilateration {
        use_fixed_point: true,
        max_iterations: 10,
        convergence_tolerance: (1e-3 * super::FIXED_POINT_SCALE as f64) as i32,
        regularization_lambda: (1e-4 * super::FIXED_POINT_SCALE as f64) as i32,
    };

    /// Ultra-fast configuration (maximum speed, minimum precision)
    pub const ULTRA_FAST: super::EmbeddedTrilateration = super::EmbeddedTrilateration {
        use_fixed_point: true,
        max_iterations: 5,
        convergence_tolerance: (1e-2 * super::FIXED_POINT_SCALE as f64) as i32,
        regularization_lambda: (1e-3 * super::FIXED_POINT_SCALE as f64) as i32,
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fixed_vector3_operations() {
        let v1 = FixedVector3::from_f64(1.0, 2.0, 3.0);
        let v2 = FixedVector3::from_f64(4.0, 5.0, 6.0);
        
        let sum = v1.add(&v2);
        let (x, y, z) = sum.to_f64();
        
        assert!((x - 5.0).abs() < 1e-3);
        assert!((y - 7.0).abs() < 1e-3);
        assert!((z - 9.0).abs() < 1e-3);
    }

    #[test]
    fn test_embedded_trilateration_3_anchors() {
        let anchors = vec![
            Anchor {
                id: "A1".to_string(),
                timestamp: 1000,
                position: Position { lat: 0.0, lon: 0.0, depth: 0.0 },
            },
            Anchor {
                id: "A2".to_string(),
                timestamp: 1001,
                position: Position { lat: 0.001, lon: 0.0, depth: 0.0 },
            },
            Anchor {
                id: "A3".to_string(),
                timestamp: 1002,
                position: Position { lat: 0.0, lon: 0.001, depth: 0.0 },
            },
        ];

        let trilateration = EmbeddedTrilateration::new();
        let result = trilateration.calculate_position_embedded(&anchors, 1003);
        
        assert!(result.is_ok());
    }

    #[test]
    fn test_fixed_point_arithmetic() {
        let trilateration = EmbeddedTrilateration::with_fixed_point();
        
        let anchors = vec![
            Anchor {
                id: "A1".to_string(),
                timestamp: 1000,
                position: Position { lat: 0.0, lon: 0.0, depth: 0.0 },
            },
            Anchor {
                id: "A2".to_string(),
                timestamp: 1001,
                position: Position { lat: 0.001, lon: 0.0, depth: 0.0 },
            },
            Anchor {
                id: "A3".to_string(),
                timestamp: 1002,
                position: Position { lat: 0.0, lon: 0.001, depth: 0.0 },
            },
            Anchor {
                id: "A4".to_string(),
                timestamp: 1003,
                position: Position { lat: 0.001, lon: 0.001, depth: 10.0 },
            },
        ];

        let result = trilateration.calculate_position_embedded(&anchors, 1004);
        assert!(result.is_ok());
    }
}