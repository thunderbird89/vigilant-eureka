use criterion::{black_box, criterion_group, criterion_main, Criterion, BenchmarkId};
use std::f64::consts::PI;

use shared_positioning::{
    CoordinateSystem, GeodeticPosition, CartesianPosition, LocalPosition,
    CoordinateTransformError, Datum, ProjectionType
};

fn benchmark_geodetic_to_cartesian(c: &mut Criterion) {
    let mut group = c.benchmark_group("geodetic_to_cartesian");
    
    let coord_system = CoordinateSystem::new(Datum::WGS84);
    
    let positions = vec![
        GeodeticPosition { latitude: 0.0, longitude: 0.0, altitude: 0.0 }, // Equator
        GeodeticPosition { latitude: 37.7749, longitude: -122.4194, altitude: 10.0 }, // San Francisco
        GeodeticPosition { latitude: 90.0, longitude: 0.0, altitude: 0.0 }, // North Pole
        GeodeticPosition { latitude: -90.0, longitude: 0.0, altitude: 0.0 }, // South Pole
        GeodeticPosition { latitude: 45.0, longitude: 180.0, altitude: 1000.0 }, // High altitude
    ];
    
    for (i, position) in positions.iter().enumerate() {
        group.bench_with_input(
            BenchmarkId::new("transform", i),
            position,
            |b, position| {
                b.iter(|| {
                    black_box(coord_system.geodetic_to_cartesian(position.clone()).unwrap());
                });
            },
        );
    }
    
    group.finish();
}

fn benchmark_cartesian_to_geodetic(c: &mut Criterion) {
    let mut group = c.benchmark_group("cartesian_to_geodetic");
    
    let coord_system = CoordinateSystem::new(Datum::WGS84);
    
    // Pre-compute cartesian positions from known geodetic positions
    let geodetic_positions = vec![
        GeodeticPosition { latitude: 0.0, longitude: 0.0, altitude: 0.0 },
        GeodeticPosition { latitude: 37.7749, longitude: -122.4194, altitude: 10.0 },
        GeodeticPosition { latitude: 90.0, longitude: 0.0, altitude: 0.0 },
        GeodeticPosition { latitude: -90.0, longitude: 0.0, altitude: 0.0 },
        GeodeticPosition { latitude: 45.0, longitude: 180.0, altitude: 1000.0 },
    ];
    
    let cartesian_positions: Vec<CartesianPosition> = geodetic_positions
        .iter()
        .map(|pos| coord_system.geodetic_to_cartesian(pos.clone()).unwrap())
        .collect();
    
    for (i, position) in cartesian_positions.iter().enumerate() {
        group.bench_with_input(
            BenchmarkId::new("transform", i),
            position,
            |b, position| {
                b.iter(|| {
                    black_box(coord_system.cartesian_to_geodetic(position.clone()).unwrap());
                });
            },
        );
    }
    
    group.finish();
}

fn benchmark_geodetic_to_local(c: &mut Criterion) {
    let mut group = c.benchmark_group("geodetic_to_local");
    
    let coord_system = CoordinateSystem::new(Datum::WGS84);
    
    // Reference position (San Francisco)
    let reference = GeodeticPosition {
        latitude: 37.7749,
        longitude: -122.4194,
        altitude: 10.0,
    };
    
    // Positions at various distances from reference
    let positions = vec![
        GeodeticPosition { latitude: 37.7749, longitude: -122.4194, altitude: 10.0 }, // Same position
        GeodeticPosition { latitude: 37.7750, longitude: -122.4194, altitude: 10.0 }, // 1 arc-minute north
        GeodeticPosition { latitude: 37.7749, longitude: -122.4190, altitude: 10.0 }, // 4 arc-minutes east
        GeodeticPosition { latitude: 37.7849, longitude: -122.4294, altitude: 100.0 }, // ~1km northeast
        GeodeticPosition { latitude: 37.8749, longitude: -122.5194, altitude: 500.0 }, // ~10km northwest
    ];
    
    for (i, position) in positions.iter().enumerate() {
        group.bench_with_input(
            BenchmarkId::new("transform", i),
            position,
            |b, position| {
                b.iter(|| {
                    black_box(coord_system.geodetic_to_local(position.clone(), &reference).unwrap());
                });
            },
        );
    }
    
    group.finish();
}

fn benchmark_local_to_geodetic(c: &mut Criterion) {
    let mut group = c.benchmark_group("local_to_geodetic");
    
    let coord_system = CoordinateSystem::new(Datum::WGS84);
    
    // Reference position (San Francisco)
    let reference = GeodeticPosition {
        latitude: 37.7749,
        longitude: -122.4194,
        altitude: 10.0,
    };
    
    // Local positions at various distances
    let local_positions = vec![
        LocalPosition { north_m: 0.0, east_m: 0.0, up_m: 0.0 }, // Same position
        LocalPosition { north_m: 100.0, east_m: 0.0, up_m: 0.0 }, // 100m north
        LocalPosition { north_m: 0.0, east_m: 100.0, up_m: 0.0 }, // 100m east
        LocalPosition { north_m: 1000.0, east_m: 1000.0, up_m: 100.0 }, // 1km northeast, 100m up
        LocalPosition { north_m: -5000.0, east_m: 3000.0, up_m: -50.0 }, // 5km south, 3km east, 50m down
    ];
    
    for (i, position) in local_positions.iter().enumerate() {
        group.bench_with_input(
            BenchmarkId::new("transform", i),
            position,
            |b, position| {
                b.iter(|| {
                    black_box(coord_system.local_to_geodetic(position.clone(), &reference).unwrap());
                });
            },
        );
    }
    
    group.finish();
}

fn benchmark_distance_calculations(c: &mut Criterion) {
    let mut group = c.benchmark_group("distance_calculations");
    
    let coord_system = CoordinateSystem::new(Datum::WGS84);
    
    let pos1 = GeodeticPosition {
        latitude: 37.7749,
        longitude: -122.4194,
        altitude: 10.0,
    };
    
    let positions = vec![
        GeodeticPosition { latitude: 37.7749, longitude: -122.4194, altitude: 10.0 }, // Same position
        GeodeticPosition { latitude: 37.7750, longitude: -122.4194, altitude: 10.0 }, // Close
        GeodeticPosition { latitude: 37.8749, longitude: -122.5194, altitude: 100.0 }, // Medium distance
        GeodeticPosition { latitude: 40.7128, longitude: -74.0060, altitude: 50.0 }, // New York (long distance)
        GeodeticPosition { latitude: -33.8688, longitude: 151.2093, altitude: 0.0 }, // Sydney (very long distance)
    ];
    
    for (i, pos2) in positions.iter().enumerate() {
        group.bench_with_input(
            BenchmarkId::new("haversine_distance", i),
            pos2,
            |b, pos2| {
                b.iter(|| {
                    black_box(coord_system.haversine_distance(&pos1, pos2));
                });
            },
        );
    }
    
    for (i, pos2) in positions.iter().enumerate() {
        group.bench_with_input(
            BenchmarkId::new("vincenty_distance", i),
            pos2,
            |b, pos2| {
                b.iter(|| {
                    black_box(coord_system.vincenty_distance(&pos1, pos2).unwrap());
                });
            },
        );
    }
    
    group.finish();
}

fn benchmark_bearing_calculations(c: &mut Criterion) {
    let mut group = c.benchmark_group("bearing_calculations");
    
    let coord_system = CoordinateSystem::new(Datum::WGS84);
    
    let pos1 = GeodeticPosition {
        latitude: 37.7749,
        longitude: -122.4194,
        altitude: 10.0,
    };
    
    let positions = vec![
        GeodeticPosition { latitude: 37.7750, longitude: -122.4194, altitude: 10.0 }, // North
        GeodeticPosition { latitude: 37.7749, longitude: -122.4190, altitude: 10.0 }, // East
        GeodeticPosition { latitude: 37.7748, longitude: -122.4194, altitude: 10.0 }, // South
        GeodeticPosition { latitude: 37.7749, longitude: -122.4198, altitude: 10.0 }, // West
        GeodeticPosition { latitude: 40.7128, longitude: -74.0060, altitude: 50.0 }, // New York
    ];
    
    for (i, pos2) in positions.iter().enumerate() {
        group.bench_with_input(
            BenchmarkId::new("initial_bearing", i),
            pos2,
            |b, pos2| {
                b.iter(|| {
                    black_box(coord_system.initial_bearing(&pos1, pos2));
                });
            },
        );
    }
    
    for (i, pos2) in positions.iter().enumerate() {
        group.bench_with_input(
            BenchmarkId::new("final_bearing", i),
            pos2,
            |b, pos2| {
                b.iter(|| {
                    black_box(coord_system.final_bearing(&pos1, pos2));
                });
            },
        );
    }
    
    group.finish();
}

fn benchmark_projection_systems(c: &mut Criterion) {
    let mut group = c.benchmark_group("projection_systems");
    
    let projections = vec![
        ProjectionType::UTM,
        ProjectionType::Mercator,
        ProjectionType::LambertConformalConic,
        ProjectionType::Stereographic,
    ];
    
    let position = GeodeticPosition {
        latitude: 37.7749,
        longitude: -122.4194,
        altitude: 10.0,
    };
    
    for projection in projections {
        let coord_system = CoordinateSystem::with_projection(Datum::WGS84, projection);
        
        group.bench_with_input(
            BenchmarkId::new("project_to_plane", format!("{:?}", projection)),
            &coord_system,
            |b, coord_system| {
                b.iter(|| {
                    black_box(coord_system.project_to_plane(&position).unwrap());
                });
            },
        );
    }
    
    group.finish();
}

fn benchmark_datum_transformations(c: &mut Criterion) {
    let mut group = c.benchmark_group("datum_transformations");
    
    let datums = vec![
        Datum::WGS84,
        Datum::NAD83,
        Datum::ED50,
        Datum::Tokyo,
    ];
    
    let position = GeodeticPosition {
        latitude: 37.7749,
        longitude: -122.4194,
        altitude: 10.0,
    };
    
    for from_datum in &datums {
        for to_datum in &datums {
            if from_datum != to_datum {
                group.bench_with_input(
                    BenchmarkId::new("transform_datum", format!("{:?}_to_{:?}", from_datum, to_datum)),
                    &(from_datum, to_datum),
                    |b, (from_datum, to_datum)| {
                        let from_system = CoordinateSystem::new(*from_datum);
                        let to_system = CoordinateSystem::new(*to_datum);
                        
                        b.iter(|| {
                            black_box(from_system.transform_datum(&position, &to_system).unwrap());
                        });
                    },
                );
            }
        }
    }
    
    group.finish();
}

fn benchmark_coordinate_validation(c: &mut Criterion) {
    let mut group = c.benchmark_group("coordinate_validation");
    
    let coord_system = CoordinateSystem::new(Datum::WGS84);
    
    let valid_positions = vec![
        GeodeticPosition { latitude: 0.0, longitude: 0.0, altitude: 0.0 },
        GeodeticPosition { latitude: 37.7749, longitude: -122.4194, altitude: 10.0 },
        GeodeticPosition { latitude: 90.0, longitude: 180.0, altitude: 8848.0 }, // Everest height
        GeodeticPosition { latitude: -90.0, longitude: -180.0, altitude: -11034.0 }, // Mariana Trench
    ];
    
    let invalid_positions = vec![
        GeodeticPosition { latitude: 91.0, longitude: 0.0, altitude: 0.0 }, // Invalid latitude
        GeodeticPosition { latitude: 0.0, longitude: 181.0, altitude: 0.0 }, // Invalid longitude
        GeodeticPosition { latitude: 37.7749, longitude: -122.4194, altitude: -20000.0 }, // Invalid altitude
    ];
    
    group.bench_function("validate_valid_positions", |b| {
        b.iter(|| {
            for position in &valid_positions {
                black_box(coord_system.validate_geodetic_position(position).unwrap());
            }
        });
    });
    
    group.bench_function("validate_invalid_positions", |b| {
        b.iter(|| {
            for position in &invalid_positions {
                let result = coord_system.validate_geodetic_position(position);
                black_box(result.is_err());
            }
        });
    });
    
    group.finish();
}

fn benchmark_batch_transformations(c: &mut Criterion) {
    let mut group = c.benchmark_group("batch_transformations");
    
    let coord_system = CoordinateSystem::new(Datum::WGS84);
    
    // Create batch of positions
    let mut positions = Vec::new();
    for i in 0..1000 {
        let lat = 37.0 + (i as f64 * 0.001);
        let lon = -122.0 + (i as f64 * 0.001);
        positions.push(GeodeticPosition {
            latitude: lat,
            longitude: lon,
            altitude: (i as f64 * 0.1),
        });
    }
    
    group.bench_function("batch_geodetic_to_cartesian", |b| {
        b.iter(|| {
            let mut cartesian_positions = Vec::new();
            for position in &positions {
                let cartesian = coord_system.geodetic_to_cartesian(position.clone()).unwrap();
                cartesian_positions.push(cartesian);
            }
            black_box(cartesian_positions);
        });
    });
    
    // Pre-compute cartesian positions for reverse transformation
    let cartesian_positions: Vec<CartesianPosition> = positions
        .iter()
        .map(|pos| coord_system.geodetic_to_cartesian(pos.clone()).unwrap())
        .collect();
    
    group.bench_function("batch_cartesian_to_geodetic", |b| {
        b.iter(|| {
            let mut geodetic_positions = Vec::new();
            for position in &cartesian_positions {
                let geodetic = coord_system.cartesian_to_geodetic(position.clone()).unwrap();
                geodetic_positions.push(geodetic);
            }
            black_box(geodetic_positions);
        });
    });
    
    group.finish();
}

fn benchmark_precision_calculations(c: &mut Criterion) {
    let mut group = c.benchmark_group("precision_calculations");
    
    let coord_system = CoordinateSystem::new(Datum::WGS84);
    
    let position = GeodeticPosition {
        latitude: 37.7749,
        longitude: -122.4194,
        altitude: 10.0,
    };
    
    // Test different precision requirements
    let precisions = vec![1.0, 0.1, 0.01, 0.001]; // meters
    
    for precision in precisions {
        group.bench_with_input(
            BenchmarkId::new("high_precision_transform", precision),
            &precision,
            |b, &precision| {
                b.iter(|| {
                    // Perform high-precision transformation
                    let cartesian = coord_system.geodetic_to_cartesian_precise(&position, precision).unwrap();
                    let back_to_geodetic = coord_system.cartesian_to_geodetic_precise(&cartesian, precision).unwrap();
                    black_box(back_to_geodetic);
                });
            },
        );
    }
    
    group.finish();
}

fn benchmark_spherical_calculations(c: &mut Criterion) {
    let mut group = c.benchmark_group("spherical_calculations");
    
    let coord_system = CoordinateSystem::new(Datum::WGS84);
    
    let center = GeodeticPosition {
        latitude: 37.7749,
        longitude: -122.4194,
        altitude: 10.0,
    };
    
    let radii = vec![100.0, 1000.0, 10000.0, 100000.0]; // meters
    let angles = vec![0.0, PI/4.0, PI/2.0, PI, 3.0*PI/2.0]; // radians
    
    for radius in radii {
        for angle in &angles {
            group.bench_with_input(
                BenchmarkId::new("point_at_distance_bearing", format!("{}m_{}rad", radius, angle)),
                &(radius, angle),
                |b, &(radius, angle)| {
                    b.iter(|| {
                        black_box(coord_system.point_at_distance_bearing(&center, radius, *angle).unwrap());
                    });
                },
            );
        }
    }
    
    group.finish();
}

criterion_group!(
    coordinate_benches,
    benchmark_geodetic_to_cartesian,
    benchmark_cartesian_to_geodetic,
    benchmark_geodetic_to_local,
    benchmark_local_to_geodetic,
    benchmark_distance_calculations,
    benchmark_bearing_calculations,
    benchmark_projection_systems,
    benchmark_datum_transformations,
    benchmark_coordinate_validation,
    benchmark_batch_transformations,
    benchmark_precision_calculations,
    benchmark_spherical_calculations
);

criterion_main!(coordinate_benches);