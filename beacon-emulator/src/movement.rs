
use serde::{Serialize, Deserialize};
use std::str::FromStr;
use shared_positioning::{GeodeticPosition, CoordinateSystemManager, Position};
use nalgebra::Vector3;
use crate::EmulatorError;

/// Movement patterns for virtual beacons
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum MovementPattern {
    /// Beacon remains stationary
    Stationary,
    /// Linear movement with specified speed and bearing
    Linear {
        /// Speed in meters per second
        speed_m_per_s: f64,
        /// Bearing in degrees (0-360, where 0 is north)
        bearing_deg: f64,
    },
    /// Circular movement around initial position
    Circular {
        /// Radius of circular movement in meters
        radius_m: f64,
        /// Period of one complete circle in seconds
        period_s: f64,
    },
    /// Random walk movement
    Random {
        /// Maximum speed in meters per second
        max_speed_m_per_s: f64,
    },
}

impl Default for MovementPattern {
    fn default() -> Self {
        Self::Stationary
    }
}

impl FromStr for MovementPattern {
    type Err = EmulatorError;
    
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        let parts: Vec<&str> = s.split(':').collect();
        
        match parts[0].to_lowercase().as_str() {
            "stationary" => {
                if parts.len() != 1 {
                    return Err(EmulatorError::MovementError(
                        "Stationary movement pattern takes no parameters".to_string()
                    ));
                }
                Ok(Self::Stationary)
            },
            "linear" => {
                if parts.len() != 3 {
                    return Err(EmulatorError::MovementError(
                        "Linear movement pattern requires speed and bearing: linear:speed:bearing".to_string()
                    ));
                }
                let speed_m_per_s = parts[1].parse::<f64>()
                    .map_err(|_| EmulatorError::MovementError(
                        format!("Invalid speed value: {}", parts[1])
                    ))?;
                let bearing_deg = parts[2].parse::<f64>()
                    .map_err(|_| EmulatorError::MovementError(
                        format!("Invalid bearing value: {}", parts[2])
                    ))?;
                
                if speed_m_per_s < 0.0 {
                    return Err(EmulatorError::MovementError(
                        "Speed must be non-negative".to_string()
                    ));
                }
                if bearing_deg < 0.0 || bearing_deg >= 360.0 {
                    return Err(EmulatorError::MovementError(
                        "Bearing must be between 0 and 360 degrees".to_string()
                    ));
                }
                
                Ok(Self::Linear { speed_m_per_s, bearing_deg })
            },
            "circular" => {
                if parts.len() != 3 {
                    return Err(EmulatorError::MovementError(
                        "Circular movement pattern requires radius and period: circular:radius:period".to_string()
                    ));
                }
                let radius_m = parts[1].parse::<f64>()
                    .map_err(|_| EmulatorError::MovementError(
                        format!("Invalid radius value: {}", parts[1])
                    ))?;
                let period_s = parts[2].parse::<f64>()
                    .map_err(|_| EmulatorError::MovementError(
                        format!("Invalid period value: {}", parts[2])
                    ))?;
                
                if radius_m <= 0.0 {
                    return Err(EmulatorError::MovementError(
                        "Radius must be positive".to_string()
                    ));
                }
                if period_s <= 0.0 {
                    return Err(EmulatorError::MovementError(
                        "Period must be positive".to_string()
                    ));
                }
                
                Ok(Self::Circular { radius_m, period_s })
            },
            "random" => {
                if parts.len() != 2 {
                    return Err(EmulatorError::MovementError(
                        "Random movement pattern requires max speed: random:max_speed".to_string()
                    ));
                }
                let max_speed_m_per_s = parts[1].parse::<f64>()
                    .map_err(|_| EmulatorError::MovementError(
                        format!("Invalid max speed value: {}", parts[1])
                    ))?;
                
                if max_speed_m_per_s <= 0.0 {
                    return Err(EmulatorError::MovementError(
                        "Max speed must be positive".to_string()
                    ));
                }
                
                Ok(Self::Random { max_speed_m_per_s })
            },
            _ => Err(EmulatorError::MovementError(
                format!("Unknown movement pattern: {}. Valid patterns: stationary, linear:speed:bearing, circular:radius:period, random:max_speed", parts[0])
            )),
        }
    }
}

impl std::fmt::Display for MovementPattern {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Stationary => write!(f, "stationary"),
            Self::Linear { speed_m_per_s, bearing_deg } => {
                write!(f, "linear:{}:{}", speed_m_per_s, bearing_deg)
            },
            Self::Circular { radius_m, period_s } => {
                write!(f, "circular:{}:{}", radius_m, period_s)
            },
            Self::Random { max_speed_m_per_s } => {
                write!(f, "random:{}", max_speed_m_per_s)
            },
        }
    }
}

/// Coordinate transformation utilities for movement pattern calculations
pub struct MovementCoordinateTransformer {
    coordinate_manager: CoordinateSystemManager,
}

impl MovementCoordinateTransformer {
    pub fn new() -> Self {
        Self {
            coordinate_manager: CoordinateSystemManager::new(),
        }
    }
    
    /// Convert GeodeticPosition to Position for coordinate transformations
    fn geodetic_to_position(geodetic: GeodeticPosition) -> Position {
        Position {
            lat: geodetic.latitude,
            lon: geodetic.longitude,
            depth: geodetic.depth,
        }
    }
    
    /// Convert Position back to GeodeticPosition
    fn position_to_geodetic(position: Position) -> GeodeticPosition {
        GeodeticPosition {
            latitude: position.lat,
            longitude: position.lon,
            depth: position.depth,
        }
    }
    
    /// Apply linear movement using high-precision coordinate transformations
    pub fn apply_linear_movement(
        &mut self,
        current_position: GeodeticPosition,
        speed_m_per_s: f64,
        bearing_deg: f64,
        time_delta_s: f64,
    ) -> Result<GeodeticPosition, EmulatorError> {
        // Validate inputs
        if speed_m_per_s < 0.0 {
            return Err(EmulatorError::MovementError("Speed cannot be negative".to_string()));
        }
        if bearing_deg < 0.0 || bearing_deg >= 360.0 {
            return Err(EmulatorError::MovementError("Bearing must be between 0 and 360 degrees".to_string()));
        }
        if time_delta_s < 0.0 {
            return Err(EmulatorError::MovementError("Time delta cannot be negative".to_string()));
        }
        
        let distance = speed_m_per_s * time_delta_s;
        
        // Convert bearing to radians (0 degrees = north, clockwise)
        let bearing_rad = (90.0 - bearing_deg).to_radians();
        
        // Calculate movement vector in local coordinates
        let east_offset = distance * bearing_rad.cos();
        let north_offset = distance * bearing_rad.sin();
        let movement_vector = Vector3::new(east_offset, north_offset, 0.0);
        
        // Convert current position to reference
        let current_pos = Self::geodetic_to_position(current_position);
        
        // Convert current position to local coordinates (should be origin)
        let current_local = self.coordinate_manager.geodetic_to_local_optimized(&current_pos, &current_pos);
        
        // Apply movement vector
        let new_local_pos = current_local + movement_vector;
        
        // Convert back to geodetic coordinates
        let new_position = self.coordinate_manager.local_to_geodetic_optimized(&new_local_pos, &current_pos);
        
        Ok(Self::position_to_geodetic(new_position))
    }
    
    /// Apply circular movement using high-precision coordinate transformations
    pub fn apply_circular_movement(
        &mut self,
        initial_position: GeodeticPosition,
        radius_m: f64,
        period_s: f64,
        elapsed_time_s: f64,
    ) -> Result<GeodeticPosition, EmulatorError> {
        // Validate inputs
        if radius_m <= 0.0 {
            return Err(EmulatorError::MovementError("Radius must be positive".to_string()));
        }
        if period_s <= 0.0 {
            return Err(EmulatorError::MovementError("Period must be positive".to_string()));
        }
        if elapsed_time_s < 0.0 {
            return Err(EmulatorError::MovementError("Elapsed time cannot be negative".to_string()));
        }
        
        // Calculate angle based on elapsed time
        let angle = 2.0 * std::f64::consts::PI * elapsed_time_s / period_s;
        
        // Calculate offset from center in local coordinates
        let east_offset = radius_m * angle.sin();
        let north_offset = radius_m * angle.cos();
        let offset_vector = Vector3::new(east_offset, north_offset, 0.0);
        
        // Convert initial position to reference
        let initial_pos = Self::geodetic_to_position(initial_position);
        
        // Apply circular offset
        let new_position = self.coordinate_manager.local_to_geodetic_optimized(&offset_vector, &initial_pos);
        
        Ok(Self::position_to_geodetic(new_position))
    }
    
    /// Apply random movement using high-precision coordinate transformations
    pub fn apply_random_movement(
        &mut self,
        current_position: GeodeticPosition,
        max_speed_m_per_s: f64,
        time_delta_s: f64,
    ) -> Result<GeodeticPosition, EmulatorError> {
        // Validate inputs
        if max_speed_m_per_s <= 0.0 {
            return Err(EmulatorError::MovementError("Max speed must be positive".to_string()));
        }
        if time_delta_s < 0.0 {
            return Err(EmulatorError::MovementError("Time delta cannot be negative".to_string()));
        }
        
        use rand::Rng;
        let mut rng = rand::thread_rng();
        
        let max_distance = max_speed_m_per_s * time_delta_s;
        
        // Generate random movement vector
        let distance = rng.gen::<f64>() * max_distance;
        let bearing = rng.gen::<f64>() * 2.0 * std::f64::consts::PI;
        
        let east_offset = distance * bearing.cos();
        let north_offset = distance * bearing.sin();
        let movement_vector = Vector3::new(east_offset, north_offset, 0.0);
        
        // Convert current position to reference
        let current_pos = Self::geodetic_to_position(current_position);
        
        // Convert current position to local coordinates (should be origin)
        let current_local = self.coordinate_manager.geodetic_to_local_optimized(&current_pos, &current_pos);
        
        // Apply movement vector
        let new_local_pos = current_local + movement_vector;
        
        // Convert back to geodetic coordinates
        let new_position = self.coordinate_manager.local_to_geodetic_optimized(&new_local_pos, &current_pos);
        
        Ok(Self::position_to_geodetic(new_position))
    }
}

impl Default for MovementCoordinateTransformer {
    fn default() -> Self {
        Self::new()
    }
}

/// Movement pattern validation utilities
pub struct MovementPatternValidator;

impl MovementPatternValidator {
    /// Validate movement pattern parameters
    pub fn validate_pattern(pattern: &MovementPattern) -> Result<(), EmulatorError> {
        match pattern {
            MovementPattern::Stationary => Ok(()),
            
            MovementPattern::Linear { speed_m_per_s, bearing_deg } => {
                Self::validate_speed(*speed_m_per_s)?;
                Self::validate_bearing(*bearing_deg)?;
                Ok(())
            },
            
            MovementPattern::Circular { radius_m, period_s } => {
                Self::validate_radius(*radius_m)?;
                Self::validate_period(*period_s)?;
                Ok(())
            },
            
            MovementPattern::Random { max_speed_m_per_s } => {
                Self::validate_positive_speed(*max_speed_m_per_s)?;
                Ok(())
            },
        }
    }
    
    /// Validate speed parameter (can be zero or positive)
    fn validate_speed(speed: f64) -> Result<(), EmulatorError> {
        if speed < 0.0 {
            return Err(EmulatorError::MovementError("Speed cannot be negative".to_string()));
        }
        if speed > 100.0 { // Reasonable maximum speed for underwater beacons
            return Err(EmulatorError::MovementError("Speed exceeds reasonable maximum (100 m/s)".to_string()));
        }
        if !speed.is_finite() {
            return Err(EmulatorError::MovementError("Speed must be a finite number".to_string()));
        }
        Ok(())
    }
    
    /// Validate positive speed parameter (must be positive)
    fn validate_positive_speed(speed: f64) -> Result<(), EmulatorError> {
        if speed <= 0.0 {
            return Err(EmulatorError::MovementError("Speed must be positive".to_string()));
        }
        if speed > 100.0 { // Reasonable maximum speed for underwater beacons
            return Err(EmulatorError::MovementError("Speed exceeds reasonable maximum (100 m/s)".to_string()));
        }
        if !speed.is_finite() {
            return Err(EmulatorError::MovementError("Speed must be a finite number".to_string()));
        }
        Ok(())
    }
    
    /// Validate bearing parameter
    fn validate_bearing(bearing: f64) -> Result<(), EmulatorError> {
        if bearing < 0.0 || bearing >= 360.0 {
            return Err(EmulatorError::MovementError("Bearing must be between 0 and 360 degrees".to_string()));
        }
        if !bearing.is_finite() {
            return Err(EmulatorError::MovementError("Bearing must be a finite number".to_string()));
        }
        Ok(())
    }
    
    /// Validate radius parameter
    fn validate_radius(radius: f64) -> Result<(), EmulatorError> {
        if radius <= 0.0 {
            return Err(EmulatorError::MovementError("Radius must be positive".to_string()));
        }
        if radius > 10000.0 { // Reasonable maximum radius (10 km)
            return Err(EmulatorError::MovementError("Radius exceeds reasonable maximum (10000 m)".to_string()));
        }
        if !radius.is_finite() {
            return Err(EmulatorError::MovementError("Radius must be a finite number".to_string()));
        }
        Ok(())
    }
    
    /// Validate period parameter
    fn validate_period(period: f64) -> Result<(), EmulatorError> {
        if period <= 0.0 {
            return Err(EmulatorError::MovementError("Period must be positive".to_string()));
        }
        if period > 86400.0 { // Maximum 24 hours
            return Err(EmulatorError::MovementError("Period exceeds reasonable maximum (86400 s)".to_string()));
        }
        if !period.is_finite() {
            return Err(EmulatorError::MovementError("Period must be a finite number".to_string()));
        }
        Ok(())
    }
    
    /// Validate position for reasonable geographic bounds
    pub fn validate_position(position: GeodeticPosition) -> Result<(), EmulatorError> {
        if position.latitude < -90.0 || position.latitude > 90.0 {
            return Err(EmulatorError::MovementError("Latitude must be between -90 and 90 degrees".to_string()));
        }
        if position.longitude < -180.0 || position.longitude > 180.0 {
            return Err(EmulatorError::MovementError("Longitude must be between -180 and 180 degrees".to_string()));
        }
        if position.depth < 0.0 || position.depth > 11000.0 { // Mariana Trench depth
            return Err(EmulatorError::MovementError("Depth must be between 0 and 11000 meters".to_string()));
        }
        if !position.latitude.is_finite() || !position.longitude.is_finite() || !position.depth.is_finite() {
            return Err(EmulatorError::MovementError("Position coordinates must be finite numbers".to_string()));
        }
        Ok(())
    }
    
    /// Validate time delta for movement calculations
    pub fn validate_time_delta(time_delta_s: f64) -> Result<(), EmulatorError> {
        if time_delta_s < 0.0 {
            return Err(EmulatorError::MovementError("Time delta cannot be negative".to_string()));
        }
        if time_delta_s > 3600.0 { // Maximum 1 hour per step
            return Err(EmulatorError::MovementError("Time delta exceeds reasonable maximum (3600 s)".to_string()));
        }
        if !time_delta_s.is_finite() {
            return Err(EmulatorError::MovementError("Time delta must be a finite number".to_string()));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_movement_pattern_validation() {
        // Test valid patterns
        assert!(MovementPatternValidator::validate_pattern(&MovementPattern::Stationary).is_ok());
        assert!(MovementPatternValidator::validate_pattern(&MovementPattern::Linear {
            speed_m_per_s: 1.5,
            bearing_deg: 45.0,
        }).is_ok());
        assert!(MovementPatternValidator::validate_pattern(&MovementPattern::Circular {
            radius_m: 100.0,
            period_s: 60.0,
        }).is_ok());
        assert!(MovementPatternValidator::validate_pattern(&MovementPattern::Random {
            max_speed_m_per_s: 2.0,
        }).is_ok());
        
        // Test invalid patterns
        assert!(MovementPatternValidator::validate_pattern(&MovementPattern::Linear {
            speed_m_per_s: -1.0,
            bearing_deg: 45.0,
        }).is_err());
        assert!(MovementPatternValidator::validate_pattern(&MovementPattern::Linear {
            speed_m_per_s: 1.0,
            bearing_deg: 400.0,
        }).is_err());
        assert!(MovementPatternValidator::validate_pattern(&MovementPattern::Circular {
            radius_m: -100.0,
            period_s: 60.0,
        }).is_err());
        assert!(MovementPatternValidator::validate_pattern(&MovementPattern::Random {
            max_speed_m_per_s: 0.0,
        }).is_err());
    }
    
    #[test]
    fn test_position_validation() {
        // Test valid positions
        let valid_pos = GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        };
        assert!(MovementPatternValidator::validate_position(valid_pos).is_ok());
        
        // Test invalid positions
        let invalid_lat = GeodeticPosition {
            latitude: 95.0,
            longitude: 45.476,
            depth: 10.0,
        };
        assert!(MovementPatternValidator::validate_position(invalid_lat).is_err());
        
        let invalid_lon = GeodeticPosition {
            latitude: 32.123,
            longitude: 200.0,
            depth: 10.0,
        };
        assert!(MovementPatternValidator::validate_position(invalid_lon).is_err());
        
        let invalid_depth = GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: -5.0,
        };
        assert!(MovementPatternValidator::validate_position(invalid_depth).is_err());
    }
    
    #[test]
    fn test_coordinate_transformer_linear_movement() {
        let mut transformer = MovementCoordinateTransformer::new();
        
        let start_pos = GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        };
        
        // Test northward movement
        let result = transformer.apply_linear_movement(start_pos, 1.0, 0.0, 1.0);
        assert!(result.is_ok());
        let new_pos = result.unwrap();
        assert!(new_pos.latitude > start_pos.latitude); // Should move north
        assert_eq!(new_pos.longitude, start_pos.longitude); // Longitude should not change
        assert_eq!(new_pos.depth, start_pos.depth); // Depth should not change
    }
    
    #[test]
    fn test_coordinate_transformer_circular_movement() {
        let mut transformer = MovementCoordinateTransformer::new();
        
        let center_pos = GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        };
        
        // Test circular movement at different times
        let result1 = transformer.apply_circular_movement(center_pos, 100.0, 60.0, 0.0);
        assert!(result1.is_ok());
        
        let result2 = transformer.apply_circular_movement(center_pos, 100.0, 60.0, 15.0);
        assert!(result2.is_ok());
        
        let pos1 = result1.unwrap();
        let pos2 = result2.unwrap();
        
        // Positions should be different
        assert_ne!(pos1.latitude, pos2.latitude);
        assert_ne!(pos1.longitude, pos2.longitude);
        assert_eq!(pos1.depth, pos2.depth); // Depth should remain the same
    }
    
    #[test]
    fn test_coordinate_transformer_random_movement() {
        let mut transformer = MovementCoordinateTransformer::new();
        
        let start_pos = GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        };
        
        let result = transformer.apply_random_movement(start_pos, 1.0, 1.0);
        assert!(result.is_ok());
        
        // Random movement should produce a valid position
        let new_pos = result.unwrap();
        assert!(MovementPatternValidator::validate_position(new_pos).is_ok());
    }
    
    #[test]
    fn test_time_delta_validation() {
        assert!(MovementPatternValidator::validate_time_delta(1.0).is_ok());
        assert!(MovementPatternValidator::validate_time_delta(0.0).is_ok());
        assert!(MovementPatternValidator::validate_time_delta(-1.0).is_err());
        assert!(MovementPatternValidator::validate_time_delta(5000.0).is_err());
    }
}