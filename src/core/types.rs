//! Core data types for the positioning system

/// 3D position in geodetic coordinates
#[derive(Debug, Clone, PartialEq)]
pub struct Position {
    pub lat: f64,
    pub lon: f64,
    pub depth: f64,
}

/// Acoustic anchor with position and timing information
#[derive(Debug, Clone)]
pub struct Anchor {
    pub id: String,
    pub timestamp: u64,
    pub position: Position,
}