use shared_positioning::{CoordinateSystemManager, MultiCoordinateSystemManager, CoordinateSystemType, Position};

/// Demonstration of the coordinate system management functionality
pub fn coordinate_system_demo() {
    println!("=== COORDINATE SYSTEM MANAGEMENT DEMO ===\n");
    
    // Test basic coordinate transformations with caching
    basic_transformation_demo();
    
    // Test multi-coordinate system support
    multi_coordinate_system_demo();
    
    // Test performance with caching
    performance_demo();
}

fn basic_transformation_demo() {
    println!("1. BASIC COORDINATE TRANSFORMATIONS WITH CACHING");
    
    let mut manager = CoordinateSystemManager::new();
    
    // Reference position (underwater base station)
    let reference = Position { lat: 32.123456, lon: 45.456789, depth: 0.0 };
    
    // Test positions (underwater vehicles)
    let test_positions = vec![
        Position { lat: 32.124456, lon: 45.457789, depth: 10.0 },
        Position { lat: 32.125456, lon: 45.458789, depth: 15.0 },
        Position { lat: 32.126456, lon: 45.459789, depth: 20.0 },
    ];
    
    println!("   Reference: lat={:.6}, lon={:.6}, depth={:.1}m", 
             reference.lat, reference.lon, reference.depth);
    
    // Single transformations
    for (i, pos) in test_positions.iter().enumerate() {
        let local = manager.geodetic_to_local_optimized(pos, &reference);
        println!("   Vehicle {}: east={:.2}m, north={:.2}m, down={:.1}m", 
                 i+1, local.x, local.y, local.z);
        
        // Test reverse transformation
        let recovered = manager.local_to_geodetic_optimized(&local, &reference);
        let lat_error = (recovered.lat - pos.lat).abs() * 111000.0; // Convert to meters
        let lon_error = (recovered.lon - pos.lon).abs() * 111000.0 * reference.lat.to_radians().cos();
        println!("     Roundtrip error: lat={:.3}m, lon={:.3}m", lat_error, lon_error);
    }
    
    // Batch transformation
    let local_batch = manager.geodetic_to_local_batch(&test_positions, &reference);
    println!("   Batch transformation completed for {} positions", local_batch.len());
    
    // High-precision transformation
    let high_precision_local = manager.geodetic_to_local_high_precision(&test_positions[0], &reference);
    let standard_local = manager.geodetic_to_local_optimized(&test_positions[0], &reference);
    let precision_diff = (high_precision_local - standard_local).norm();
    println!("   High-precision vs standard difference: {:.6}m", precision_diff);
    
    // Performance stats
    let stats = manager.get_performance_stats();
    println!("   Performance: {} transformations, {:.1}% cache hit rate", 
             stats.total_transformations, stats.cache_hit_rate * 100.0);
    
    println!();
}

fn multi_coordinate_system_demo() {
    println!("2. MULTI-COORDINATE SYSTEM SUPPORT");
    
    let mut manager = MultiCoordinateSystemManager::new(CoordinateSystemType::WGS84);
    
    // Test position in Mediterranean Sea
    let wgs84_pos = Position { lat: 35.123456, lon: 25.456789, depth: 50.0 };
    
    println!("   Original WGS84: lat={:.6}, lon={:.6}, depth={:.1}m", 
             wgs84_pos.lat, wgs84_pos.lon, wgs84_pos.depth);
    
    // Convert to UTM
    match manager.wgs84_to_utm(&wgs84_pos) {
        Ok(utm) => {
            println!("   UTM Zone {}{}: easting={:.2}m, northing={:.2}m, elevation={:.1}m", 
                     utm.zone, if utm.northern { "N" } else { "S" }, 
                     utm.easting, utm.northing, utm.elevation);
            
            // UTM to WGS84 conversion would be available in a full implementation
            println!("   UTM to WGS84 conversion would verify roundtrip accuracy here");
        }
        Err(e) => println!("   WGS84 to UTM conversion failed: {}", e),
    }
    
    // Test coordinate validation
    let invalid_pos = Position { lat: 95.0, lon: 200.0, depth: -2000.0 };
    match manager.validate_position(&invalid_pos) {
        Ok(_) => println!("   Validation unexpectedly passed"),
        Err(e) => println!("   Validation correctly failed: {}", e),
    }
    
    // Test Earth curvature correction for large areas
    let mut large_area_positions = vec![
        Position { lat: 35.0, lon: 25.0, depth: 0.0 },
        Position { lat: 35.5, lon: 25.5, depth: 0.0 },
        Position { lat: 36.0, lon: 26.0, depth: 0.0 },
    ];
    
    let reference_large = Position { lat: 35.25, lon: 25.25, depth: 0.0 };
    manager.configure_validation(true, true, 100.0); // 100km operational area
    
    // Earth curvature correction would be applied here in a full implementation
    println!("   Earth curvature correction would be applied for large operational area");
    
    // System information
    let info = manager.get_system_info();
    println!("   System info: {:?} coordinate system, validation={}, curvature_correction={}", 
             info.primary_system, info.validation_enabled, info.earth_curvature_correction);
    
    println!();
}

fn performance_demo() {
    println!("3. PERFORMANCE AND CACHING DEMONSTRATION");
    
    let mut manager = CoordinateSystemManager::new();
    let reference = Position { lat: 32.123456, lon: 45.456789, depth: 0.0 };
    
    // Generate test positions in a grid pattern
    let mut test_positions = Vec::new();
    for i in 0..10 {
        for j in 0..10 {
            test_positions.push(Position {
                lat: reference.lat + (i as f64) * 0.001,
                lon: reference.lon + (j as f64) * 0.001,
                depth: (i + j) as f64 * 2.0,
            });
        }
    }
    
    println!("   Testing with {} positions", test_positions.len());
    
    // Time the transformations
    let start_time = std::time::Instant::now();
    
    // First pass - cache misses
    for pos in &test_positions {
        let _local = manager.geodetic_to_local_optimized(pos, &reference);
    }
    
    let first_pass_time = start_time.elapsed();
    let stats_after_first = manager.get_performance_stats();
    
    // Second pass - cache hits
    let second_start = std::time::Instant::now();
    for pos in &test_positions {
        let _local = manager.geodetic_to_local_optimized(pos, &reference);
    }
    
    let second_pass_time = second_start.elapsed();
    let stats_after_second = manager.get_performance_stats();
    
    println!("   First pass (cache misses): {:.2}ms for {} transformations", 
             first_pass_time.as_secs_f64() * 1000.0, test_positions.len());
    println!("   Second pass (cache hits): {:.2}ms for {} transformations", 
             second_pass_time.as_secs_f64() * 1000.0, test_positions.len());
    
    let speedup = first_pass_time.as_secs_f64() / second_pass_time.as_secs_f64();
    println!("   Cache speedup: {:.1}x faster", speedup);
    
    println!("   Final stats: {} total transformations, {:.1}% cache hit rate, {} cached references", 
             stats_after_second.total_transformations, 
             stats_after_second.cache_hit_rate * 100.0,
             stats_after_second.cached_references);
    
    // Test cache cleanup
    manager.cleanup_cache();
    println!("   Cache cleanup completed");
    
    println!();
}