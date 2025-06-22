use nalgebra::{Matrix3, Vector3};
use serde::Deserialize;
use std::f64::consts::PI;

const SPEED_OF_SOUND_WATER: f64 = 1500.0; // m/s

#[derive(Debug, Deserialize)]
struct AnchorsJson {
    anchors: Vec<Anchor>,
}

#[derive(Debug, Deserialize, Clone)]
struct Anchor {
    id: String,
    timestamp: u64, // milliseconds
    position: Position,
}

#[derive(Debug, Deserialize, Clone)]
struct Position {
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

pub fn trilaterate(
    anchors: &[Anchor],
    receiver_time_ms: u64,
) -> Result<(Position, Vector3<f64>), String> {
    if anchors.len() != 4 {
        return Err("Exactly four anchors required".to_string());
    }

    // Reference position (first anchor) for local tangent plane
    let reference_pos = &anchors[0].position;

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

    // Trilateration using linearized form: subtract equation of first anchor
    // (x - xi)^2 + (y - yi)^2 + (z - zi)^2 = di^2
    // Leads to A * p = b where A (3x3) and b (3x1)
    let p1 = positions[0];
    let d1_sq = distances[0] * distances[0];

    let mut a_data = [[0.0; 3]; 3];
    let mut b_data = [0.0; 3];
    for i in 1..4 {
        let pi = positions[i];
        let di_sq = distances[i] * distances[i];
        // Row index is i - 1
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

    // Solve using least-squares (handles singular or near-singular A)
    let svd = a_mat.svd(true, true);
    let position_local = svd
        .solve(&b_vec, 1.0e-9)
        .map_err(|e| e.to_string())?;

    // Convert back to lat/lon/depth for reporting
    let geodetic_pos = local_to_geodetic(&position_local, reference_pos);

    Ok((geodetic_pos, position_local))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().collect();
    if args.len() != 3 {
        eprintln!(
            "Usage: {} <json_file> <receiver_timestamp_ms>",
            args.get(0).map_or("trilateration", |s| s.as_str())
        );
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

        // println!(
        //     "Estimated receiver position (local END): x={:.2} m east, y={:.2} m north, z={:.2} m down",
        //     local_pos.x, local_pos.y, local_pos.z
        // );
        // println!(
        //     "Estimated receiver position (lat/lon/depth): lat={:.6}, lon={:.6}, depth={:.2} m",
        //     geodetic_pos.lat, geodetic_pos.lon, geodetic_pos.depth
        // );

        // Check local position
        assert!((local_pos.x - -1.46).abs() < 1e-2);
        assert!((local_pos.y - 21.55).abs() < 1e-2);
        assert!((local_pos.z - 14.50).abs() < 1e-2);

        // Check geodetic position
        assert!((geodetic_pos.lat - 32.123644).abs() < 1e-6);
        assert!((geodetic_pos.lon - 45.476735).abs() < 1e-6);
        assert!((geodetic_pos.depth - 14.50).abs() < 1e-2);
    }
} 