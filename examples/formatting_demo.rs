//! Demonstration of position output formatting capabilities

use trilateration::{
    Position, PositionResponse, GeometryQuality, LocalPosition,
    PositionFormatter, TextFormatter, JsonFormatter, CsvFormatter,
    OutputFormat
};

fn main() {
    println!("=== Underwater Positioning System - Formatting Demo ===\n");

    // Create a sample position response
    let position_response = PositionResponse {
        position: Position {
            lat: 32.715736,
            lon: -117.161087,
            depth: 15.5,
        },
        local_position: Some(LocalPosition {
            east_m: 125.3,
            north_m: -87.2,
            down_m: 15.5,
        }),
        accuracy_estimate: 1.2,
        timestamp_ms: 1642780800000, // Example timestamp
        anchor_count: 4,
        geometry_quality: GeometryQuality::Good,
        computation_time_us: 1250,
        sequence_number: 42,
    };

    // Demonstrate different output formats
    demonstrate_geodetic_format(&position_response);
    demonstrate_local_format(&position_response);
    demonstrate_both_format(&position_response);
    demonstrate_compact_format(&position_response);
    
    // Demonstrate text formatting
    demonstrate_text_formatting(&position_response);
    
    // Demonstrate JSON formatting
    demonstrate_json_formatting(&position_response);
    
    // Demonstrate CSV formatting
    demonstrate_csv_formatting(&position_response);
}

fn demonstrate_geodetic_format(response: &PositionResponse) {
    println!("1. Geodetic Format:");
    let formatter = PositionFormatter::new().with_format(OutputFormat::Geodetic);
    let formatted = formatter.format(response);
    
    match &formatted.position_data {
        trilateration::api::formatting::PositionData::Geodetic { latitude_deg, longitude_deg, depth_m } => {
            println!("   Latitude:  {:.6}°", latitude_deg);
            println!("   Longitude: {:.6}°", longitude_deg);
            println!("   Depth:     {:.1} m", depth_m);
        }
        _ => println!("   Unexpected format"),
    }
    
    println!("   Accuracy:  {:.1} m", formatted.quality.accuracy_estimate_m);
    println!("   Quality:   {:?}", formatted.quality.geometry_quality);
    println!("   Anchors:   {}", formatted.quality.anchor_count);
    println!("   Confidence: {}%", formatted.quality.confidence_level);
    println!();
}

fn demonstrate_local_format(response: &PositionResponse) {
    println!("2. Local Format (East-North-Down):");
    let formatter = PositionFormatter::new().with_format(OutputFormat::Local);
    let formatted = formatter.format(response);
    
    match &formatted.position_data {
        trilateration::api::formatting::PositionData::Local { east_m, north_m, down_m } => {
            println!("   East:  {:.1} m", east_m);
            println!("   North: {:.1} m", north_m);
            println!("   Down:  {:.1} m", down_m);
        }
        _ => println!("   Unexpected format"),
    }
    println!();
}

fn demonstrate_both_format(response: &PositionResponse) {
    println!("3. Both Formats:");
    let formatter = PositionFormatter::new().with_format(OutputFormat::Both);
    let formatted = formatter.format(response);
    
    match &formatted.position_data {
        trilateration::api::formatting::PositionData::Both { geodetic, local } => {
            println!("   Geodetic: {:.6}°N, {:.6}°E, {:.1}m depth", 
                geodetic.latitude_deg, geodetic.longitude_deg, geodetic.depth_m);
            println!("   Local:    E{:.1}m, N{:.1}m, D{:.1}m", 
                local.east_m, local.north_m, local.down_m);
        }
        _ => println!("   Unexpected format"),
    }
    println!();
}

fn demonstrate_compact_format(response: &PositionResponse) {
    println!("4. Compact Binary Format:");
    let formatter = PositionFormatter::new().with_format(OutputFormat::Compact);
    let formatted = formatter.format(response);
    
    match &formatted.position_data {
        trilateration::api::formatting::PositionData::Compact { data } => {
            // Copy values to avoid packed struct alignment issues
            let lat = data.lat_micro_deg;
            let lon = data.lon_micro_deg;
            let depth = data.depth_mm;
            let accuracy = data.accuracy_cm;
            
            println!("   Lat (μ°):  {}", lat);
            println!("   Lon (μ°):  {}", lon);
            println!("   Depth (mm): {}", depth);
            println!("   Accuracy (cm): {}", accuracy);
            println!("   Anchors: {}", data.anchor_count());
            println!("   Quality: {:?}", data.geometry_quality());
        }
        _ => println!("   Unexpected format"),
    }
    println!();
}

fn demonstrate_text_formatting(response: &PositionResponse) {
    println!("5. Human-Readable Text Formatting:");
    
    // Verbose format
    println!("   Verbose:");
    let formatter = PositionFormatter::new();
    let formatted = formatter.format(response);
    let text_formatter = TextFormatter { include_diagnostics: false, compact: false };
    let text = text_formatter.format_text(&formatted);
    for line in text.lines() {
        println!("     {}", line);
    }
    
    // Compact format
    println!("\n   Compact:");
    let text_formatter_compact = TextFormatter { include_diagnostics: false, compact: true };
    let compact_text = text_formatter_compact.format_text(&formatted);
    println!("     {}", compact_text);
    println!();
}

fn demonstrate_json_formatting(response: &PositionResponse) {
    println!("6. JSON Formatting:");
    
    let formatter = PositionFormatter::new();
    let formatted = formatter.format(response);
    
    // Pretty JSON
    println!("   Pretty JSON:");
    let json_formatter = JsonFormatter::pretty();
    match json_formatter.format_json(&formatted) {
        Ok(json) => {
            for line in json.lines().take(10) { // Show first 10 lines
                println!("     {}", line);
            }
            if json.lines().count() > 10 {
                println!("     ... (truncated)");
            }
        }
        Err(e) => println!("     Error: {}", e),
    }
    
    // Compact JSON
    println!("\n   Compact JSON:");
    let json_formatter_compact = JsonFormatter::new();
    match json_formatter_compact.format_json(&formatted) {
        Ok(json) => {
            let truncated = if json.len() > 100 {
                format!("{}...", &json[..100])
            } else {
                json
            };
            println!("     {}", truncated);
        }
        Err(e) => println!("     Error: {}", e),
    }
    println!();
}

fn demonstrate_csv_formatting(response: &PositionResponse) {
    println!("7. CSV Formatting:");
    
    let formatter = PositionFormatter::new().with_format(OutputFormat::Both);
    let formatted = formatter.format(response);
    let csv_formatter = CsvFormatter::new();
    
    println!("   Header:");
    println!("     {}", csv_formatter.header());
    
    println!("   Data:");
    let csv_row = csv_formatter.format_csv(&formatted);
    println!("     {}", csv_row);
    println!();
}