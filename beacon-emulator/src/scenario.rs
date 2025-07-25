use clap::ValueEnum;
use serde::{Serialize, Deserialize};
use std::str::FromStr;
use shared_positioning::GeodeticPosition;
use crate::EmulatorError;

/// Predefined test scenario types for beacon arrangements
#[derive(Debug, Clone, Serialize, Deserialize, ValueEnum)]
#[serde(rename_all = "lowercase")]
pub enum ScenarioType {
    /// Triangular arrangement of beacons
    Triangle,
    /// Square arrangement of beacons
    Square,
    /// Linear arrangement of beacons
    Line,
    /// Grid arrangement of beacons
    Grid,
}

impl FromStr for ScenarioType {
    type Err = EmulatorError;
    
    fn from_str(s: &str) -> Result<Self, Self::Err> {
        match s.to_lowercase().as_str() {
            "triangle" => Ok(Self::Triangle),
            "square" => Ok(Self::Square),
            "line" => Ok(Self::Line),
            "grid" => Ok(Self::Grid),
            _ => Err(EmulatorError::InvalidScenario(
                format!("Unknown scenario type: {}", s)
            )),
        }
    }
}

impl std::fmt::Display for ScenarioType {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Triangle => write!(f, "triangle"),
            Self::Square => write!(f, "square"),
            Self::Line => write!(f, "line"),
            Self::Grid => write!(f, "grid"),
        }
    }
}

/// Configuration template for scenario generation
#[derive(Debug, Clone)]
pub struct ScenarioTemplate {
    pub scenario_type: ScenarioType,
    pub beacon_count: usize,
    pub spacing_meters: f64,
    pub center_position: GeodeticPosition,
    pub transmission_interval_ms: u32,
}

impl ScenarioTemplate {
    /// Create a new scenario template with validation
    pub fn new(
        scenario_type: ScenarioType,
        beacon_count: usize,
        spacing_meters: f64,
        center_position: GeodeticPosition,
        transmission_interval_ms: u32,
    ) -> Result<Self, EmulatorError> {
        // Validate scenario parameters
        validate_scenario_parameters(&scenario_type, beacon_count, spacing_meters)?;
        
        Ok(Self {
            scenario_type,
            beacon_count,
            spacing_meters,
            center_position,
            transmission_interval_ms,
        })
    }
    
    /// Generate beacon positions for this scenario
    pub fn generate_positions(&self) -> Result<Vec<GeodeticPosition>, EmulatorError> {
        generate_scenario_positions(
            &self.scenario_type,
            self.beacon_count,
            self.spacing_meters,
            &self.center_position,
        )
    }
}

/// Generate beacon positions for a given scenario type
pub fn generate_scenario_positions(
    scenario_type: &ScenarioType,
    count: usize,
    spacing: f64,
    center: &GeodeticPosition,
) -> Result<Vec<GeodeticPosition>, EmulatorError> {
    // Validate parameters first
    validate_scenario_parameters(scenario_type, count, spacing)?;
    
    match scenario_type {
        ScenarioType::Triangle => generate_triangle_positions(count, spacing, center),
        ScenarioType::Square => generate_square_positions(count, spacing, center),
        ScenarioType::Line => generate_line_positions(count, spacing, center),
        ScenarioType::Grid => generate_grid_positions(count, spacing, center),
    }
}

/// Validate scenario parameters for correctness
pub fn validate_scenario_parameters(
    scenario_type: &ScenarioType,
    count: usize,
    spacing: f64,
) -> Result<(), EmulatorError> {
    // General validations
    if count == 0 {
        return Err(EmulatorError::InvalidScenario(
            "Beacon count must be greater than 0".to_string()
        ));
    }
    
    if count > 100 {
        return Err(EmulatorError::InvalidScenario(
            "Beacon count cannot exceed 100".to_string()
        ));
    }
    
    if spacing <= 0.0 {
        return Err(EmulatorError::InvalidScenario(
            "Spacing must be greater than 0".to_string()
        ));
    }
    
    if spacing > 10000.0 {
        return Err(EmulatorError::InvalidScenario(
            "Spacing cannot exceed 10000 meters".to_string()
        ));
    }
    
    // Scenario-specific validations
    match scenario_type {
        ScenarioType::Triangle => {
            if count != 3 {
                return Err(EmulatorError::InvalidScenario(
                    "Triangle scenario requires exactly 3 beacons".to_string()
                ));
            }
        }
        ScenarioType::Square => {
            if count != 4 {
                return Err(EmulatorError::InvalidScenario(
                    "Square scenario requires exactly 4 beacons".to_string()
                ));
            }
        }
        ScenarioType::Line => {
            if count < 2 {
                return Err(EmulatorError::InvalidScenario(
                    "Line scenario requires at least 2 beacons".to_string()
                ));
            }
        }
        ScenarioType::Grid => {
            if count < 4 {
                return Err(EmulatorError::InvalidScenario(
                    "Grid scenario requires at least 4 beacons".to_string()
                ));
            }
            
            // Check if count is a perfect square or can form a reasonable rectangle
            let sqrt_count = (count as f64).sqrt();
            if sqrt_count.fract() != 0.0 {
                // Not a perfect square, check if it can form a reasonable rectangle
                let mut found_factors = false;
                for rows in 2..=(count / 2) {
                    if count % rows == 0 {
                        let cols = count / rows;
                        if (rows as f64 / cols as f64) >= 0.5 && (rows as f64 / cols as f64) <= 2.0 {
                            found_factors = true;
                            break;
                        }
                    }
                }
                if !found_factors {
                    return Err(EmulatorError::InvalidScenario(
                        format!("Grid scenario with {} beacons cannot form a reasonable rectangular arrangement", count)
                    ));
                }
            }
        }
    }
    
    Ok(())
}

/// Generate positions for triangular arrangement
fn generate_triangle_positions(
    count: usize,
    spacing: f64,
    center: &GeodeticPosition,
) -> Result<Vec<GeodeticPosition>, EmulatorError> {
    if count != 3 {
        return Err(EmulatorError::InvalidScenario(
            "Triangle arrangement requires exactly 3 beacons".to_string()
        ));
    }
    
    let mut positions = Vec::new();
    
    // Calculate positions for equilateral triangle
    // Vertices are at 120-degree intervals around the center
    for i in 0..3 {
        let angle_deg = (i as f64) * 120.0;
        let angle_rad = angle_deg.to_radians();
        
        // Calculate offset from center
        let lat_offset = spacing * angle_rad.cos() / METERS_PER_DEGREE_LAT;
        let lon_offset = spacing * angle_rad.sin() / 
            meters_per_degree_lon_at_latitude(center.latitude);
        
        positions.push(GeodeticPosition {
            latitude: center.latitude + lat_offset,
            longitude: center.longitude + lon_offset,
            depth: center.depth,
        });
    }
    
    Ok(positions)
}

/// Generate positions for square arrangement
fn generate_square_positions(
    count: usize,
    spacing: f64,
    center: &GeodeticPosition,
) -> Result<Vec<GeodeticPosition>, EmulatorError> {
    if count != 4 {
        return Err(EmulatorError::InvalidScenario(
            "Square arrangement requires exactly 4 beacons".to_string()
        ));
    }
    
    let half_spacing = spacing / 2.0;
    let lat_offset = half_spacing / METERS_PER_DEGREE_LAT;
    let lon_offset = half_spacing / meters_per_degree_lon_at_latitude(center.latitude);
    
    let positions = vec![
        // Top-right
        GeodeticPosition {
            latitude: center.latitude + lat_offset,
            longitude: center.longitude + lon_offset,
            depth: center.depth,
        },
        // Top-left
        GeodeticPosition {
            latitude: center.latitude + lat_offset,
            longitude: center.longitude - lon_offset,
            depth: center.depth,
        },
        // Bottom-left
        GeodeticPosition {
            latitude: center.latitude - lat_offset,
            longitude: center.longitude - lon_offset,
            depth: center.depth,
        },
        // Bottom-right
        GeodeticPosition {
            latitude: center.latitude - lat_offset,
            longitude: center.longitude + lon_offset,
            depth: center.depth,
        },
    ];
    
    Ok(positions)
}

/// Generate positions for linear arrangement
fn generate_line_positions(
    count: usize,
    spacing: f64,
    center: &GeodeticPosition,
) -> Result<Vec<GeodeticPosition>, EmulatorError> {
    if count < 2 {
        return Err(EmulatorError::InvalidScenario(
            "Line arrangement requires at least 2 beacons".to_string()
        ));
    }
    
    let mut positions = Vec::new();
    
    // Calculate starting position (leftmost beacon)
    let total_length = (count - 1) as f64 * spacing;
    let start_offset = -total_length / 2.0;
    
    // Place beacons along a north-south line (varying latitude)
    for i in 0..count {
        let offset = start_offset + (i as f64) * spacing;
        let lat_offset = offset / METERS_PER_DEGREE_LAT;
        
        positions.push(GeodeticPosition {
            latitude: center.latitude + lat_offset,
            longitude: center.longitude,
            depth: center.depth,
        });
    }
    
    Ok(positions)
}

/// Generate positions for grid arrangement
fn generate_grid_positions(
    count: usize,
    spacing: f64,
    center: &GeodeticPosition,
) -> Result<Vec<GeodeticPosition>, EmulatorError> {
    if count < 4 {
        return Err(EmulatorError::InvalidScenario(
            "Grid arrangement requires at least 4 beacons".to_string()
        ));
    }
    
    // Determine grid dimensions
    let (rows, cols) = find_best_grid_dimensions(count)?;
    
    let mut positions = Vec::new();
    
    // Calculate starting positions (top-left corner)
    let row_start = -((rows - 1) as f64) * spacing / 2.0;
    let col_start = -((cols - 1) as f64) * spacing / 2.0;
    
    // Generate positions row by row
    for row in 0..rows {
        for col in 0..cols {
            if positions.len() >= count {
                break;
            }
            
            let lat_offset = (row_start + (row as f64) * spacing) / METERS_PER_DEGREE_LAT;
            let lon_offset = (col_start + (col as f64) * spacing) / 
                meters_per_degree_lon_at_latitude(center.latitude);
            
            positions.push(GeodeticPosition {
                latitude: center.latitude + lat_offset,
                longitude: center.longitude + lon_offset,
                depth: center.depth,
            });
        }
    }
    
    Ok(positions)
}

/// Find the best grid dimensions for a given beacon count
fn find_best_grid_dimensions(count: usize) -> Result<(usize, usize), EmulatorError> {
    // First try perfect square
    let sqrt_count = (count as f64).sqrt();
    if sqrt_count.fract() == 0.0 {
        let dim = sqrt_count as usize;
        return Ok((dim, dim));
    }
    
    // Find factors that create a reasonable rectangle
    let mut best_rows = 1;
    let mut best_cols = count;
    let mut best_ratio = count as f64;
    
    for rows in 2..=(count / 2) {
        if count % rows == 0 {
            let cols = count / rows;
            let ratio = if rows > cols { rows as f64 / cols as f64 } else { cols as f64 / rows as f64 };
            
            // Prefer ratios closer to 1 (more square-like)
            if ratio < best_ratio {
                best_rows = rows;
                best_cols = cols;
                best_ratio = ratio;
            }
        }
    }
    
    // If we couldn't find good factors, create a rectangle that fits all beacons
    if best_ratio > 3.0 {
        // Create a roughly square grid, allowing for incomplete last row
        let cols = (count as f64).sqrt().ceil() as usize;
        let rows = (count + cols - 1) / cols; // Ceiling division
        return Ok((rows, cols));
    }
    
    Ok((best_rows, best_cols))
}

// Constants for coordinate calculations
const METERS_PER_DEGREE_LAT: f64 = 111_132.0; // Approximate meters per degree latitude

/// Calculate meters per degree longitude at a given latitude
fn meters_per_degree_lon_at_latitude(latitude_deg: f64) -> f64 {
    111_320.0 * latitude_deg.to_radians().cos()
}

#[cfg(test)]
mod tests {
    use super::*;
    
    fn test_center() -> GeodeticPosition {
        GeodeticPosition {
            latitude: 32.123,
            longitude: 45.476,
            depth: 10.0,
        }
    }
    
    #[test]
    fn test_triangle_scenario() {
        let positions = generate_triangle_positions(3, 100.0, &test_center()).unwrap();
        assert_eq!(positions.len(), 3);
        
        // All positions should be at the same depth
        for pos in &positions {
            assert_eq!(pos.depth, 10.0);
        }
        
        // Positions should be roughly equidistant from center
        let center = test_center();
        for pos in &positions {
            let lat_diff = pos.latitude - center.latitude;
            let lon_diff = pos.longitude - center.longitude;
            let distance_approx = (lat_diff * lat_diff + lon_diff * lon_diff).sqrt();
            assert!(distance_approx > 0.0);
        }
    }
    
    #[test]
    fn test_square_scenario() {
        let positions = generate_square_positions(4, 100.0, &test_center()).unwrap();
        assert_eq!(positions.len(), 4);
        
        // All positions should be at the same depth
        for pos in &positions {
            assert_eq!(pos.depth, 10.0);
        }
    }
    
    #[test]
    fn test_line_scenario() {
        let positions = generate_line_positions(5, 50.0, &test_center()).unwrap();
        assert_eq!(positions.len(), 5);
        
        // All positions should have the same longitude and depth
        let center = test_center();
        for pos in &positions {
            assert_eq!(pos.longitude, center.longitude);
            assert_eq!(pos.depth, center.depth);
        }
        
        // Positions should be ordered by latitude
        for i in 1..positions.len() {
            assert!(positions[i].latitude > positions[i-1].latitude);
        }
    }
    
    #[test]
    fn test_grid_scenario() {
        let positions = generate_grid_positions(9, 75.0, &test_center()).unwrap();
        assert_eq!(positions.len(), 9);
        
        // All positions should be at the same depth
        for pos in &positions {
            assert_eq!(pos.depth, 10.0);
        }
    }
    
    #[test]
    fn test_scenario_validation() {
        // Valid scenarios
        assert!(validate_scenario_parameters(&ScenarioType::Triangle, 3, 100.0).is_ok());
        assert!(validate_scenario_parameters(&ScenarioType::Square, 4, 100.0).is_ok());
        assert!(validate_scenario_parameters(&ScenarioType::Line, 5, 100.0).is_ok());
        assert!(validate_scenario_parameters(&ScenarioType::Grid, 9, 100.0).is_ok());
        
        // Invalid scenarios
        assert!(validate_scenario_parameters(&ScenarioType::Triangle, 4, 100.0).is_err());
        assert!(validate_scenario_parameters(&ScenarioType::Square, 3, 100.0).is_err());
        assert!(validate_scenario_parameters(&ScenarioType::Line, 1, 100.0).is_err());
        assert!(validate_scenario_parameters(&ScenarioType::Grid, 3, 100.0).is_err());
        
        // Invalid parameters
        assert!(validate_scenario_parameters(&ScenarioType::Triangle, 0, 100.0).is_err());
        assert!(validate_scenario_parameters(&ScenarioType::Triangle, 3, 0.0).is_err());
        assert!(validate_scenario_parameters(&ScenarioType::Triangle, 3, -10.0).is_err());
        assert!(validate_scenario_parameters(&ScenarioType::Triangle, 101, 100.0).is_err());
        assert!(validate_scenario_parameters(&ScenarioType::Triangle, 3, 15000.0).is_err());
    }
    
    #[test]
    fn test_grid_dimensions() {
        // Perfect squares
        assert_eq!(find_best_grid_dimensions(9).unwrap(), (3, 3));
        assert_eq!(find_best_grid_dimensions(16).unwrap(), (4, 4));
        
        // Rectangles
        let (rows, cols) = find_best_grid_dimensions(6).unwrap();
        assert_eq!(rows * cols, 6);
        assert!(rows >= 2 && cols >= 2);
        
        let (rows, cols) = find_best_grid_dimensions(12).unwrap();
        assert_eq!(rows * cols, 12);
        assert!(rows >= 2 && cols >= 2);
    }
    
    #[test]
    fn test_scenario_template() {
        let template = ScenarioTemplate::new(
            ScenarioType::Triangle,
            3,
            100.0,
            test_center(),
            5000,
        ).unwrap();
        
        let positions = template.generate_positions().unwrap();
        assert_eq!(positions.len(), 3);
        
        // Invalid template should fail
        assert!(ScenarioTemplate::new(
            ScenarioType::Triangle,
            4, // Wrong count for triangle
            100.0,
            test_center(),
            5000,
        ).is_err());
    }
}