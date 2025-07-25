
use serde::{Serialize, Deserialize};
use std::str::FromStr;
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