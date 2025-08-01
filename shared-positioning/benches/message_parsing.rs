use criterion::{black_box, criterion_group, criterion_main, Criterion, BenchmarkId};
use std::time::SystemTime;
use uuid::Uuid;

use shared_positioning::{
    MessageBuilder, MessageParser, GeodeticPosition, GpsPosition, 
    TransmissionMessageVersion, MessageValidationError
};

fn benchmark_message_building(c: &mut Criterion) {
    let mut group = c.benchmark_group("message_building");
    
    let beacon_id = Uuid::new_v4();
    let position = GeodeticPosition {
        latitude: 37.7749,
        longitude: -122.4194,
        altitude: 10.0,
    };
    let message_builder = MessageBuilder::new();
    
    let versions = vec![
        ("V1", TransmissionMessageVersion::V1),
        ("V2", TransmissionMessageVersion::V2),
        ("V3", TransmissionMessageVersion::V3),
    ];
    
    for (name, version) in versions {
        group.bench_with_input(
            BenchmarkId::new("build_message", name),
            &version,
            |b, version| {
                b.iter(|| {
                    let result = match version {
                        TransmissionMessageVersion::V1 => {
                            message_builder.build_v1_message(beacon_id, position.clone(), 85, 123)
                        }
                        TransmissionMessageVersion::V2 => {
                            message_builder.build_v2_message(beacon_id, position.clone(), 87, 124)
                        }
                        TransmissionMessageVersion::V3 => {
                            message_builder.build_v3_message(beacon_id, position.clone(), 89, 125)
                        }
                    };
                    black_box(result.unwrap());
                });
            },
        );
    }
    
    group.finish();
}

fn benchmark_message_parsing(c: &mut Criterion) {
    let mut group = c.benchmark_group("message_parsing");
    
    let beacon_id = Uuid::new_v4();
    let position = GeodeticPosition {
        latitude: 37.7749,
        longitude: -122.4194,
        altitude: 10.0,
    };
    let message_builder = MessageBuilder::new();
    let message_parser = MessageParser::new();
    
    // Pre-build messages for parsing benchmarks
    let v1_message = message_builder.build_v1_message(beacon_id, position.clone(), 85, 123).unwrap();
    let v2_message = message_builder.build_v2_message(beacon_id, position.clone(), 87, 124).unwrap();
    let v3_message = message_builder.build_v3_message(beacon_id, position, 89, 125).unwrap();
    
    group.bench_function("parse_v1_message", |b| {
        b.iter(|| {
            black_box(message_parser.parse_message(&v1_message).unwrap());
        });
    });
    
    group.bench_function("parse_v2_message", |b| {
        b.iter(|| {
            black_box(message_parser.parse_message(&v2_message).unwrap());
        });
    });
    
    group.bench_function("parse_v3_message", |b| {
        b.iter(|| {
            black_box(message_parser.parse_message(&v3_message).unwrap());
        });
    });
    
    group.finish();
}

fn benchmark_message_validation(c: &mut Criterion) {
    let mut group = c.benchmark_group("message_validation");
    
    let beacon_id = Uuid::new_v4();
    let position = GeodeticPosition {
        latitude: 37.7749,
        longitude: -122.4194,
        altitude: 10.0,
    };
    let message_builder = MessageBuilder::new();
    
    // Pre-build messages for validation benchmarks
    let v1_message = message_builder.build_v1_message(beacon_id, position.clone(), 85, 123).unwrap();
    let v2_message = message_builder.build_v2_message(beacon_id, position.clone(), 87, 124).unwrap();
    let v3_message = message_builder.build_v3_message(beacon_id, position, 89, 125).unwrap();
    
    group.bench_function("validate_v1_message", |b| {
        b.iter(|| {
            black_box(message_builder.validate_message(&v1_message).unwrap());
        });
    });
    
    group.bench_function("validate_v2_message", |b| {
        b.iter(|| {
            black_box(message_builder.validate_message(&v2_message).unwrap());
        });
    });
    
    group.bench_function("validate_v3_message", |b| {
        b.iter(|| {
            black_box(message_builder.validate_message(&v3_message).unwrap());
        });
    });
    
    group.finish();
}

fn benchmark_checksum_calculation(c: &mut Criterion) {
    let mut group = c.benchmark_group("checksum_calculation");
    
    let message_builder = MessageBuilder::new();
    
    let data_sizes = vec![16, 32, 64, 128, 256];
    
    for size in data_sizes {
        group.bench_with_input(
            BenchmarkId::new("crc16_checksum", size),
            &size,
            |b, &size| {
                let data: Vec<u8> = (0..size).map(|i| (i % 256) as u8).collect();
                b.iter(|| {
                    black_box(message_builder.calculate_crc16(&data));
                });
            },
        );
    }
    
    group.finish();
}

fn benchmark_message_serialization(c: &mut Criterion) {
    let mut group = c.benchmark_group("message_serialization");
    
    let beacon_id = Uuid::new_v4();
    let position = GeodeticPosition {
        latitude: 37.7749,
        longitude: -122.4194,
        altitude: 10.0,
    };
    let message_builder = MessageBuilder::new();
    
    // Create parsed message for serialization
    let v3_message = message_builder.build_v3_message(beacon_id, position, 89, 125).unwrap();
    let message_parser = MessageParser::new();
    let parsed_message = message_parser.parse_message(&v3_message).unwrap();
    
    group.bench_function("serialize_to_json", |b| {
        b.iter(|| {
            black_box(serde_json::to_string(&parsed_message).unwrap());
        });
    });
    
    group.bench_function("serialize_to_binary", |b| {
        b.iter(|| {
            black_box(bincode::serialize(&parsed_message).unwrap());
        });
    });
    
    group.finish();
}

fn benchmark_message_deserialization(c: &mut Criterion) {
    let mut group = c.benchmark_group("message_deserialization");
    
    let beacon_id = Uuid::new_v4();
    let position = GeodeticPosition {
        latitude: 37.7749,
        longitude: -122.4194,
        altitude: 10.0,
    };
    let message_builder = MessageBuilder::new();
    
    // Create serialized data for deserialization
    let v3_message = message_builder.build_v3_message(beacon_id, position, 89, 125).unwrap();
    let message_parser = MessageParser::new();
    let parsed_message = message_parser.parse_message(&v3_message).unwrap();
    
    let json_data = serde_json::to_string(&parsed_message).unwrap();
    let binary_data = bincode::serialize(&parsed_message).unwrap();
    
    group.bench_function("deserialize_from_json", |b| {
        b.iter(|| {
            let _: shared_positioning::ParsedMessage = black_box(serde_json::from_str(&json_data).unwrap());
        });
    });
    
    group.bench_function("deserialize_from_binary", |b| {
        b.iter(|| {
            let _: shared_positioning::ParsedMessage = black_box(bincode::deserialize(&binary_data).unwrap());
        });
    });
    
    group.finish();
}

fn benchmark_batch_message_processing(c: &mut Criterion) {
    let mut group = c.benchmark_group("batch_processing");
    
    let beacon_id = Uuid::new_v4();
    let message_builder = MessageBuilder::new();
    let message_parser = MessageParser::new();
    
    // Create batch of messages
    let mut messages = Vec::new();
    for i in 0..100 {
        let position = GeodeticPosition {
            latitude: 37.7749 + (i as f64 * 0.0001),
            longitude: -122.4194 + (i as f64 * 0.0001),
            altitude: 10.0 + (i as f64 * 0.1),
        };
        let message = message_builder.build_v3_message(beacon_id, position, 85, i as u16).unwrap();
        messages.push(message);
    }
    
    group.bench_function("process_message_batch", |b| {
        b.iter(|| {
            let mut parsed_messages = Vec::new();
            for message in &messages {
                let parsed = message_parser.parse_message(message).unwrap();
                parsed_messages.push(parsed);
            }
            black_box(parsed_messages);
        });
    });
    
    group.finish();
}

fn benchmark_message_filtering(c: &mut Criterion) {
    let mut group = c.benchmark_group("message_filtering");
    
    let message_builder = MessageBuilder::new();
    let message_parser = MessageParser::new();
    
    // Create messages from different beacons
    let mut messages = Vec::new();
    for i in 0..100 {
        let beacon_id = if i % 3 == 0 { 
            Uuid::parse_str("550e8400-e29b-41d4-a716-446655440000").unwrap()
        } else {
            Uuid::new_v4()
        };
        
        let position = GeodeticPosition {
            latitude: 37.7749 + (i as f64 * 0.0001),
            longitude: -122.4194 + (i as f64 * 0.0001),
            altitude: 10.0,
        };
        
        let message = message_builder.build_v3_message(beacon_id, position, 85, i as u16).unwrap();
        let parsed = message_parser.parse_message(&message).unwrap();
        messages.push(parsed);
    }
    
    let target_beacon = Uuid::parse_str("550e8400-e29b-41d4-a716-446655440000").unwrap();
    
    group.bench_function("filter_by_beacon_id", |b| {
        b.iter(|| {
            let filtered: Vec<_> = messages.iter()
                .filter(|msg| msg.beacon_id == target_beacon)
                .collect();
            black_box(filtered);
        });
    });
    
    group.bench_function("filter_by_signal_quality", |b| {
        b.iter(|| {
            let filtered: Vec<_> = messages.iter()
                .filter(|msg| msg.signal_quality >= 80)
                .collect();
            black_box(filtered);
        });
    });
    
    group.finish();
}

fn benchmark_message_compression(c: &mut Criterion) {
    let mut group = c.benchmark_group("message_compression");
    
    let beacon_id = Uuid::new_v4();
    let message_builder = MessageBuilder::new();
    
    // Create large message batch for compression testing
    let mut message_batch = Vec::new();
    for i in 0..1000 {
        let position = GeodeticPosition {
            latitude: 37.7749 + (i as f64 * 0.0001),
            longitude: -122.4194 + (i as f64 * 0.0001),
            altitude: 10.0 + (i as f64 * 0.1),
        };
        let message = message_builder.build_v3_message(beacon_id, position, 85, i as u16).unwrap();
        message_batch.extend(message);
    }
    
    group.bench_function("compress_message_batch", |b| {
        b.iter(|| {
            use std::io::Write;
            let mut encoder = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::default());
            encoder.write_all(&message_batch).unwrap();
            let compressed = encoder.finish().unwrap();
            black_box(compressed);
        });
    });
    
    // Create compressed data for decompression benchmark
    let mut encoder = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::default());
    encoder.write_all(&message_batch).unwrap();
    let compressed_data = encoder.finish().unwrap();
    
    group.bench_function("decompress_message_batch", |b| {
        b.iter(|| {
            use std::io::Read;
            let mut decoder = flate2::read::GzDecoder::new(&compressed_data[..]);
            let mut decompressed = Vec::new();
            decoder.read_to_end(&mut decompressed).unwrap();
            black_box(decompressed);
        });
    });
    
    group.finish();
}

fn benchmark_concurrent_message_processing(c: &mut Criterion) {
    let mut group = c.benchmark_group("concurrent_processing");
    
    let beacon_id = Uuid::new_v4();
    let message_builder = MessageBuilder::new();
    
    // Create messages for concurrent processing
    let mut messages = Vec::new();
    for i in 0..1000 {
        let position = GeodeticPosition {
            latitude: 37.7749 + (i as f64 * 0.0001),
            longitude: -122.4194 + (i as f64 * 0.0001),
            altitude: 10.0,
        };
        let message = message_builder.build_v3_message(beacon_id, position, 85, i as u16).unwrap();
        messages.push(message);
    }
    
    group.bench_function("sequential_processing", |b| {
        b.iter(|| {
            let message_parser = MessageParser::new();
            let mut parsed_messages = Vec::new();
            for message in &messages {
                let parsed = message_parser.parse_message(message).unwrap();
                parsed_messages.push(parsed);
            }
            black_box(parsed_messages);
        });
    });
    
    group.bench_function("parallel_processing", |b| {
        b.iter(|| {
            use std::sync::Arc;
            use std::thread;
            
            let messages = Arc::new(messages.clone());
            let chunk_size = messages.len() / 4; // 4 threads
            let mut handles = Vec::new();
            
            for chunk_start in (0..messages.len()).step_by(chunk_size) {
                let messages_clone = Arc::clone(&messages);
                let handle = thread::spawn(move || {
                    let message_parser = MessageParser::new();
                    let mut parsed_messages = Vec::new();
                    let chunk_end = std::cmp::min(chunk_start + chunk_size, messages_clone.len());
                    
                    for i in chunk_start..chunk_end {
                        let parsed = message_parser.parse_message(&messages_clone[i]).unwrap();
                        parsed_messages.push(parsed);
                    }
                    parsed_messages
                });
                handles.push(handle);
            }
            
            let mut all_parsed = Vec::new();
            for handle in handles {
                let parsed_chunk = handle.join().unwrap();
                all_parsed.extend(parsed_chunk);
            }
            
            black_box(all_parsed);
        });
    });
    
    group.finish();
}

criterion_group!(
    message_benches,
    benchmark_message_building,
    benchmark_message_parsing,
    benchmark_message_validation,
    benchmark_checksum_calculation,
    benchmark_message_serialization,
    benchmark_message_deserialization,
    benchmark_batch_message_processing,
    benchmark_message_filtering,
    benchmark_message_compression,
    benchmark_concurrent_message_processing
);

criterion_main!(message_benches);