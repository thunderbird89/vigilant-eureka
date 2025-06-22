use nalgebra::{Matrix3, Vector3};
use serde::Deserialize;
use std::f64::consts::PI;

const SPEED_OF_SOUND_WATER: f64 = 1500.0; // m/s

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

#[derive(Debug, Deserialize, Clone)]
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

pub fn trilaterate(
    anchors: &[Anchor],
    receiver_time_ms: u64,
) -> Result<(Position, Vector3<f64>), String> {
    if anchors.len() < 3 || anchors.len() > 4 {
        return Err("Between 3 and 4 anchors required".to_string());
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

    // Handle 3-anchor case: 2D solution only
    if anchors.len() == 3 {
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
} 