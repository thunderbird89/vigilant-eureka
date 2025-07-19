//! Position output formatting and serialization
//! 
//! This module provides various output formats for position data,
//! including human-readable, compact binary, and diagnostic formats.

use crate::api::types::{PositionResponse, LocalPosition, GeometryQuality, OutputFormat, CompactPosition};
use crate::core::Position;
use serde::{Deserialize, Serialize};
use std::fmt;

/// Formatted position output with multiple format options
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FormattedPosition {
    /// Position data in requested format
    pub position_data: PositionData,
    /// Quality indicators
    pub quality: QualityIndicators,
    /// Timing information
    pub timing: TimingInfo,
    /// Diagnostic information (optional)
    pub diagnostics: Option<DiagnosticInfo>,
}

/// Position data in various formats
#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "format")]
pub enum PositionData {
    /// Geodetic coordinates (latitude, longitude, depth)
    Geodetic {
        latitude_deg: f64,
        longitude_deg: f64,
        depth_m: f64,
    },
    /// Local tangent plane coordinates (East-North-Down)
    Local {
        east_m: f64,
        north_m: f64,
        down_m: f64,
    },
    /// Both geodetic and local coordinates
    Both {
        geodetic: GeodeticCoordinates,
        local: LocalCoordinates,
    },
    /// Compact binary format (for embedded systems)
    Compact {
        data: CompactPosition,
    },
}

/// Geodetic coordinates structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GeodeticCoordinates {
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub depth_m: f64,
}

/// Local coordinates structure
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LocalCoordinates {
    pub east_m: f64,
    pub north_m: f64,
    pub down_m: f64,
}

/// Quality indicators for position accuracy
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct QualityIndicators {
    /// Estimated accuracy in meters
    pub accuracy_estimate_m: f32,
    /// Geometry quality assessment
    pub geometry_quality: GeometryQuality,
    /// Number of anchors used
    pub anchor_count: u8,
    /// Dilution of Precision (if available)
    pub dilution_of_precision: Option<f32>,
    /// Signal quality score (0-100)
    pub signal_quality_score: Option<u8>,
    /// Position confidence level (0-100)
    pub confidence_level: u8,
}

/// Timing information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimingInfo {
    /// Position timestamp (milliseconds since epoch)
    pub timestamp_ms: u64,
    /// Sequence number for tracking
    pub sequence_number: u32,
    /// Computation time in microseconds
    pub computation_time_us: u32,
    /// Age of oldest anchor data (milliseconds)
    pub oldest_anchor_age_ms: Option<u32>,
}

/// Diagnostic information for debugging
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DiagnosticInfo {
    /// Anchor information used in calculation
    pub anchors_used: Vec<AnchorDiagnostic>,
    /// Algorithm details
    pub algorithm_info: AlgorithmDiagnostic,
    /// Error information (if any)
    pub errors: Vec<String>,
    /// Warnings (if any)
    pub warnings: Vec<String>,
    /// Performance metrics
    pub performance: PerformanceDiagnostic,
}

/// Diagnostic information for individual anchors
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AnchorDiagnostic {
    /// Anchor identifier
    pub id: String,
    /// Distance to anchor (meters)
    pub distance_m: f32,
    /// Signal quality (0-255)
    pub signal_quality: u8,
    /// Data age (milliseconds)
    pub age_ms: u32,
    /// Anchor position
    pub position: GeodeticCoordinates,
    /// Whether anchor was used in calculation
    pub used_in_calculation: bool,
}

/// Algorithm diagnostic information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AlgorithmDiagnostic {
    /// Algorithm name used
    pub algorithm_name: String,
    /// Number of iterations (if iterative)
    pub iterations: Option<u32>,
    /// Convergence achieved (if iterative)
    pub converged: Option<bool>,
    /// Residual error
    pub residual_error: Option<f32>,
    /// Condition number of geometry matrix
    pub condition_number: Option<f32>,
}

/// Performance diagnostic information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PerformanceDiagnostic {
    /// Total computation time (microseconds)
    pub total_time_us: u32,
    /// Time breakdown by component
    pub time_breakdown: TimeBreakdown,
    /// Memory usage (bytes)
    pub memory_usage_bytes: Option<u32>,
    /// CPU usage percentage
    pub cpu_usage_percent: Option<f32>,
}

/// Time breakdown by computation component
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimeBreakdown {
    /// Time spent parsing messages (microseconds)
    pub parsing_us: u32,
    /// Time spent validating data (microseconds)
    pub validation_us: u32,
    /// Time spent in trilateration algorithm (microseconds)
    pub trilateration_us: u32,
    /// Time spent in coordinate conversion (microseconds)
    pub coordinate_conversion_us: u32,
    /// Time spent formatting output (microseconds)
    pub formatting_us: u32,
}

/// Position formatter for converting PositionResponse to various formats
pub struct PositionFormatter {
    /// Include diagnostic information
    pub include_diagnostics: bool,
    /// Default output format
    pub default_format: OutputFormat,
    /// Precision for floating-point values
    pub precision: u8,
}

impl Default for PositionFormatter {
    fn default() -> Self {
        Self {
            include_diagnostics: false,
            default_format: OutputFormat::Geodetic,
            precision: 6,
        }
    }
}

impl PositionFormatter {
    /// Create a new position formatter
    pub fn new() -> Self {
        Self::default()
    }
    
    /// Create formatter with diagnostic information enabled
    pub fn with_diagnostics() -> Self {
        Self {
            include_diagnostics: true,
            ..Default::default()
        }
    }
    
    /// Set the default output format
    pub fn with_format(mut self, format: OutputFormat) -> Self {
        self.default_format = format;
        self
    }
    
    /// Set the precision for floating-point values
    pub fn with_precision(mut self, precision: u8) -> Self {
        self.precision = precision;
        self
    }
    
    /// Format a position response using the default format
    pub fn format(&self, response: &PositionResponse) -> FormattedPosition {
        self.format_with_options(response, self.default_format, self.include_diagnostics)
    }
    
    /// Format a position response with specific options
    pub fn format_with_options(
        &self,
        response: &PositionResponse,
        format: OutputFormat,
        include_diagnostics: bool,
    ) -> FormattedPosition {
        let position_data = self.format_position_data(&response.position, response.local_position.as_ref(), format);
        let quality = self.format_quality_indicators(response);
        let timing = self.format_timing_info(response);
        let diagnostics = if include_diagnostics {
            Some(self.format_diagnostic_info(response))
        } else {
            None
        };
        
        FormattedPosition {
            position_data,
            quality,
            timing,
            diagnostics,
        }
    }
    
    /// Format position data according to the specified format
    fn format_position_data(
        &self,
        position: &Position,
        local_position: Option<&LocalPosition>,
        format: OutputFormat,
    ) -> PositionData {
        match format {
            OutputFormat::Geodetic => PositionData::Geodetic {
                latitude_deg: self.round_to_precision(position.lat),
                longitude_deg: self.round_to_precision(position.lon),
                depth_m: self.round_to_precision(position.depth),
            },
            OutputFormat::Local => {
                if let Some(local) = local_position {
                    PositionData::Local {
                        east_m: self.round_to_precision(local.east_m as f64),
                        north_m: self.round_to_precision(local.north_m as f64),
                        down_m: self.round_to_precision(local.down_m as f64),
                    }
                } else {
                    // Fallback to geodetic if local not available
                    PositionData::Geodetic {
                        latitude_deg: self.round_to_precision(position.lat),
                        longitude_deg: self.round_to_precision(position.lon),
                        depth_m: self.round_to_precision(position.depth),
                    }
                }
            }
            OutputFormat::Both => {
                let geodetic = GeodeticCoordinates {
                    latitude_deg: self.round_to_precision(position.lat),
                    longitude_deg: self.round_to_precision(position.lon),
                    depth_m: self.round_to_precision(position.depth),
                };
                
                let local = if let Some(local_pos) = local_position {
                    LocalCoordinates {
                        east_m: self.round_to_precision(local_pos.east_m as f64),
                        north_m: self.round_to_precision(local_pos.north_m as f64),
                        down_m: self.round_to_precision(local_pos.down_m as f64),
                    }
                } else {
                    LocalCoordinates {
                        east_m: 0.0,
                        north_m: 0.0,
                        down_m: 0.0,
                    }
                };
                
                PositionData::Both { geodetic, local }
            }
            OutputFormat::Compact => {
                // Create a temporary PositionResponse for compact conversion
                let temp_response = PositionResponse {
                    position: position.clone(),
                    local_position: local_position.cloned(),
                    accuracy_estimate: 1.0, // Default values for compact format
                    timestamp_ms: 0,
                    anchor_count: 0,
                    geometry_quality: GeometryQuality::Good,
                    computation_time_us: 0,
                    sequence_number: 0,
                };
                
                PositionData::Compact {
                    data: CompactPosition::from_position(&temp_response),
                }
            }
        }
    }
    
    /// Format quality indicators
    fn format_quality_indicators(&self, response: &PositionResponse) -> QualityIndicators {
        QualityIndicators {
            accuracy_estimate_m: response.accuracy_estimate,
            geometry_quality: response.geometry_quality,
            anchor_count: response.anchor_count,
            dilution_of_precision: None, // TODO: Calculate actual DOP
            signal_quality_score: None,  // TODO: Calculate signal quality score
            confidence_level: self.calculate_confidence_level(response),
        }
    }
    
    /// Format timing information
    fn format_timing_info(&self, response: &PositionResponse) -> TimingInfo {
        TimingInfo {
            timestamp_ms: response.timestamp_ms,
            sequence_number: response.sequence_number,
            computation_time_us: response.computation_time_us,
            oldest_anchor_age_ms: None, // TODO: Calculate oldest anchor age
        }
    }
    
    /// Format diagnostic information
    fn format_diagnostic_info(&self, response: &PositionResponse) -> DiagnosticInfo {
        DiagnosticInfo {
            anchors_used: Vec::new(), // TODO: Add actual anchor diagnostics
            algorithm_info: AlgorithmDiagnostic {
                algorithm_name: "EmbeddedTrilateration".to_string(),
                iterations: None,
                converged: None,
                residual_error: None,
                condition_number: None,
            },
            errors: Vec::new(),
            warnings: Vec::new(),
            performance: PerformanceDiagnostic {
                total_time_us: response.computation_time_us,
                time_breakdown: TimeBreakdown {
                    parsing_us: 0,
                    validation_us: 0,
                    trilateration_us: response.computation_time_us,
                    coordinate_conversion_us: 0,
                    formatting_us: 0,
                },
                memory_usage_bytes: None,
                cpu_usage_percent: None,
            },
        }
    }
    
    /// Calculate confidence level based on geometry quality and accuracy
    fn calculate_confidence_level(&self, response: &PositionResponse) -> u8 {
        let geometry_score = match response.geometry_quality {
            GeometryQuality::Excellent => 100,
            GeometryQuality::Good => 80,
            GeometryQuality::Acceptable => 60,
            GeometryQuality::Poor => 40,
            GeometryQuality::Degenerate => 20,
        };
        
        let accuracy_score = if response.accuracy_estimate <= 1.0 {
            100
        } else if response.accuracy_estimate <= 2.0 {
            80
        } else if response.accuracy_estimate <= 5.0 {
            60
        } else if response.accuracy_estimate <= 10.0 {
            40
        } else {
            20
        };
        
        let anchor_score = match response.anchor_count {
            0..=2 => 20,
            3 => 60,
            4 => 80,
            5..=u8::MAX => 100,
        };
        
        // Weighted average
        ((geometry_score * 50 + accuracy_score * 30 + anchor_score * 20) / 100) as u8
    }
    
    /// Round floating-point value to specified precision
    fn round_to_precision(&self, value: f64) -> f64 {
        let multiplier = 10_f64.powi(self.precision as i32);
        (value * multiplier).round() / multiplier
    }
}

/// Human-readable text formatter
pub struct TextFormatter {
    /// Include diagnostic information
    pub include_diagnostics: bool,
    /// Use compact format
    pub compact: bool,
}

impl Default for TextFormatter {
    fn default() -> Self {
        Self {
            include_diagnostics: false,
            compact: false,
        }
    }
}

impl TextFormatter {
    /// Create a new text formatter
    pub fn new() -> Self {
        Self::default()
    }
    
    /// Format position as human-readable text
    pub fn format_text(&self, formatted_pos: &FormattedPosition) -> String {
        let mut output = String::new();
        
        // Position data
        match &formatted_pos.position_data {
            PositionData::Geodetic { latitude_deg, longitude_deg, depth_m } => {
                if self.compact {
                    output.push_str(&format!("Pos: {:.6}°N, {:.6}°E, {:.1}m", 
                        latitude_deg, longitude_deg, depth_m));
                } else {
                    output.push_str(&format!("Position (Geodetic):\n"));
                    output.push_str(&format!("  Latitude:  {:.6}°\n", latitude_deg));
                    output.push_str(&format!("  Longitude: {:.6}°\n", longitude_deg));
                    output.push_str(&format!("  Depth:     {:.1} m\n", depth_m));
                }
            }
            PositionData::Local { east_m, north_m, down_m } => {
                if self.compact {
                    output.push_str(&format!("Pos: E{:.1}m, N{:.1}m, D{:.1}m", 
                        east_m, north_m, down_m));
                } else {
                    output.push_str(&format!("Position (Local ENU):\n"));
                    output.push_str(&format!("  East:  {:.1} m\n", east_m));
                    output.push_str(&format!("  North: {:.1} m\n", north_m));
                    output.push_str(&format!("  Down:  {:.1} m\n", down_m));
                }
            }
            PositionData::Both { geodetic, local } => {
                if self.compact {
                    output.push_str(&format!("Geo: {:.6}°N,{:.6}°E,{:.1}m | Local: E{:.1}m,N{:.1}m,D{:.1}m", 
                        geodetic.latitude_deg, geodetic.longitude_deg, geodetic.depth_m,
                        local.east_m, local.north_m, local.down_m));
                } else {
                    output.push_str("Position (Geodetic):\n");
                    output.push_str(&format!("  Latitude:  {:.6}°\n", geodetic.latitude_deg));
                    output.push_str(&format!("  Longitude: {:.6}°\n", geodetic.longitude_deg));
                    output.push_str(&format!("  Depth:     {:.1} m\n", geodetic.depth_m));
                    output.push_str("Position (Local ENU):\n");
                    output.push_str(&format!("  East:  {:.1} m\n", local.east_m));
                    output.push_str(&format!("  North: {:.1} m\n", local.north_m));
                    output.push_str(&format!("  Down:  {:.1} m\n", local.down_m));
                }
            }
            PositionData::Compact { data } => {
                let pos = data.to_position();
                output.push_str(&format!("Compact: {:.6}°N, {:.6}°E, {:.1}m", 
                    pos.lat, pos.lon, pos.depth));
            }
        }
        
        // Quality information
        if !self.compact {
            output.push_str("\nQuality:\n");
            output.push_str(&format!("  Accuracy:   {:.1} m\n", formatted_pos.quality.accuracy_estimate_m));
            output.push_str(&format!("  Geometry:   {:?}\n", formatted_pos.quality.geometry_quality));
            output.push_str(&format!("  Anchors:    {}\n", formatted_pos.quality.anchor_count));
            output.push_str(&format!("  Confidence: {}%\n", formatted_pos.quality.confidence_level));
        } else {
            output.push_str(&format!(" | Acc:{:.1}m, Anc:{}, Conf:{}%", 
                formatted_pos.quality.accuracy_estimate_m,
                formatted_pos.quality.anchor_count,
                formatted_pos.quality.confidence_level));
        }
        
        // Timing information
        if !self.compact {
            output.push_str("\nTiming:\n");
            output.push_str(&format!("  Timestamp:  {} ms\n", formatted_pos.timing.timestamp_ms));
            output.push_str(&format!("  Sequence:   #{}\n", formatted_pos.timing.sequence_number));
            output.push_str(&format!("  Comp Time:  {} μs\n", formatted_pos.timing.computation_time_us));
        }
        
        // Diagnostic information
        if self.include_diagnostics && formatted_pos.diagnostics.is_some() {
            let diag = formatted_pos.diagnostics.as_ref().unwrap();
            output.push_str("\nDiagnostics:\n");
            output.push_str(&format!("  Algorithm:  {}\n", diag.algorithm_info.algorithm_name));
            output.push_str(&format!("  Total Time: {} μs\n", diag.performance.total_time_us));
            
            if !diag.errors.is_empty() {
                output.push_str("  Errors:\n");
                for error in &diag.errors {
                    output.push_str(&format!("    - {}\n", error));
                }
            }
            
            if !diag.warnings.is_empty() {
                output.push_str("  Warnings:\n");
                for warning in &diag.warnings {
                    output.push_str(&format!("    - {}\n", warning));
                }
            }
        }
        
        output
    }
}

/// JSON formatter for structured output
pub struct JsonFormatter {
    /// Pretty print JSON
    pub pretty: bool,
}

impl Default for JsonFormatter {
    fn default() -> Self {
        Self { pretty: false }
    }
}

impl JsonFormatter {
    /// Create a new JSON formatter
    pub fn new() -> Self {
        Self::default()
    }
    
    /// Create a pretty-printing JSON formatter
    pub fn pretty() -> Self {
        Self { pretty: true }
    }
    
    /// Format position as JSON string
    pub fn format_json(&self, formatted_pos: &FormattedPosition) -> Result<String, serde_json::Error> {
        if self.pretty {
            serde_json::to_string_pretty(formatted_pos)
        } else {
            serde_json::to_string(formatted_pos)
        }
    }
}

/// CSV formatter for data logging
pub struct CsvFormatter {
    /// Include header row
    pub include_header: bool,
}

impl Default for CsvFormatter {
    fn default() -> Self {
        Self { include_header: true }
    }
}

impl CsvFormatter {
    /// Create a new CSV formatter
    pub fn new() -> Self {
        Self::default()
    }
    
    /// Get CSV header
    pub fn header(&self) -> String {
        "timestamp_ms,sequence,latitude,longitude,depth,east,north,down,accuracy,geometry_quality,anchor_count,confidence,computation_time_us".to_string()
    }
    
    /// Format position as CSV row
    pub fn format_csv(&self, formatted_pos: &FormattedPosition) -> String {
        let (lat, lon, depth, east, north, down) = match &formatted_pos.position_data {
            PositionData::Geodetic { latitude_deg, longitude_deg, depth_m } => {
                (*latitude_deg, *longitude_deg, *depth_m, 0.0, 0.0, 0.0)
            }
            PositionData::Local { east_m, north_m, down_m } => {
                (0.0, 0.0, 0.0, *east_m, *north_m, *down_m)
            }
            PositionData::Both { geodetic, local } => {
                (geodetic.latitude_deg, geodetic.longitude_deg, geodetic.depth_m,
                 local.east_m, local.north_m, local.down_m)
            }
            PositionData::Compact { data } => {
                let pos = data.to_position();
                (pos.lat, pos.lon, pos.depth, 0.0, 0.0, 0.0)
            }
        };
        
        format!("{},{},{:.6},{:.6},{:.1},{:.1},{:.1},{:.1},{:.1},{:?},{},{},{}",
            formatted_pos.timing.timestamp_ms,
            formatted_pos.timing.sequence_number,
            lat, lon, depth,
            east, north, down,
            formatted_pos.quality.accuracy_estimate_m,
            formatted_pos.quality.geometry_quality,
            formatted_pos.quality.anchor_count,
            formatted_pos.quality.confidence_level,
            formatted_pos.timing.computation_time_us
        )
    }
}