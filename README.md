# Underwater Positioning System

A high-precision underwater positioning system using acoustic trilateration with advanced algorithms for sub-meter accuracy.

## Project Structure

The codebase has been organized into logical modules using Rust's module system:

```
src/
├── lib.rs                    # Library root with public API
├── main.rs                   # Demo application
├── core/                     # Core types and constants
│   ├── mod.rs
│   ├── types.rs             # Position, Anchor structs
│   └── constants.rs         # Physical constants
├── algorithms/              # Core positioning algorithms
│   ├── mod.rs
│   ├── trilateration.rs     # Advanced trilateration algorithms
│   ├── gdop.rs             # GDOP optimization
│   └── precision.rs        # High-precision transformations
├── processing/             # Signal and data processing
│   ├── mod.rs
│   ├── kalman.rs           # Kalman filtering
│   ├── noise.rs            # Noise filtering
│   ├── parser.rs           # Message parsing
│   └── cache.rs            # Optimization cache
├── validation/             # Data validation and quality assurance
│   ├── mod.rs
│   ├── accuracy.rs         # Accuracy validation
│   ├── data.rs             # Data validation
│   ├── error.rs            # Error handling
│   └── degradation.rs      # Graceful degradation
└── utils/                  # Utility modules
    ├── mod.rs
    ├── config.rs           # Configuration management
    └── monitor.rs          # Performance monitoring
```

## Features

### Core Algorithms

- **Advanced Trilateration**: Multiple algorithms including MLE, robust estimation, and Levenberg-Marquardt
- **GDOP Optimization**: Geometric dilution of precision analysis and anchor selection
- **High-Precision Transformations**: Coordinate system conversions with sub-meter accuracy

### Signal Processing

- **Kalman Filtering**: Temporal position smoothing and prediction
- **Noise Filtering**: Adaptive signal processing for underwater conditions
- **Message Parsing**: Support for multiple message formats and versions
- **Optimization Cache**: Performance optimization for repeated calculations

### Quality Assurance

- **Accuracy Validation**: Real-time accuracy assessment and statistics
- **Data Validation**: Message validation with configurable thresholds
- **Error Handling**: Comprehensive error classification and recovery
- **Graceful Degradation**: Automatic fallback modes for degraded conditions

### System Management

- **Configuration Management**: Runtime parameter adjustment and persistence
- **Performance Monitoring**: Real-time system performance tracking
- **Health Monitoring**: System and anchor health assessment

## Usage

### As a Library

```rust
use trilateration::{
    core::{Position, Anchor},
    algorithms::AdvancedTrilateration,
    validation::AccuracyValidator,
};

// Create positioning system
let mut trilateration = AdvancedTrilateration::new();
let mut validator = AccuracyValidator::new();

// Define anchors
let anchors = vec![
    Anchor { id: "001".to_string(), timestamp: 1234567890, position: Position { lat: 32.123, lon: -117.456, depth: 0.0 } },
    // ... more anchors
];

// Perform positioning
let result = trilateration.mle_trilateration(&anchors, 1234567890, &[0.5, 0.5, 0.5], noise_model);
```

### Demo Application

Run the comprehensive demo:

```bash
cargo run --bin underwater-positioning-demo
```

The demo showcases:

1. Message parsing and validation
2. Sub-meter accuracy optimization
3. Error handling and graceful degradation
4. Configuration management

## Building

```bash
# Check compilation
cargo check

# Build library and demo
cargo build

# Run tests
cargo test

# Run demo
cargo run --bin underwater-positioning-demo
```

## Configuration

The system supports runtime configuration through the `ConfigurationManager`:

```rust
use trilateration::utils::ConfigurationManager;

let mut config = ConfigurationManager::new();
config.set_sound_speed(1520.0)?;
config.set_water_temperature(18.0)?;
```

## Performance

The system is optimized for real-time operation:

- Message parsing: ~180,000 messages/sec
- Validation: ~380,000 messages/sec
- Sub-meter accuracy achievable with proper anchor geometry and environmental corrections

## Best Practices

1. **Anchor Deployment**: Use tetrahedral configuration for optimal GDOP
2. **Environmental Corrections**: Apply sound speed corrections based on water conditions
3. **Quality Monitoring**: Continuously monitor system health and accuracy
4. **Graceful Degradation**: Implement fallback modes for degraded conditions
5. **Configuration Management**: Use snapshots and validation for configuration changes

## License

MIT OR Apache-2.0
