# Project Structure

## Root Directory

- **Cargo.toml**: Project configuration and dependencies
- **Cargo.lock**: Dependency lock file
- **anchors.json**: Sample anchor configuration data
- **demo_config.json**: Demo system configuration
- **src/**: Main source code directory

## Source Code Organization

### Core Modules (`src/core/`)
- **types.rs**: Fundamental data structures (Position, Anchor)
- **constants.rs**: System constants (sound speed, etc.)
- **mod.rs**: Module exports and re-exports

### Algorithms (`src/algorithms/`)
- **trilateration.rs**: Core trilateration algorithms and coordinate transformations
- **gdop.rs**: Geometric Dilution of Precision calculations and optimization
- **precision.rs**: High-precision coordinate transformations and environmental corrections
- **mod.rs**: Algorithm module exports

### Processing (`src/processing/`)
- **kalman.rs**: Kalman filtering for position smoothing
- **noise.rs**: Noise filtering and signal processing
- **parser.rs**: Message parsing for anchor data
- **cache.rs**: Optimization caching for performance
- **mod.rs**: Processing module exports

### Validation (`src/validation/`)
- **accuracy.rs**: Accuracy validation and statistical analysis
- **data.rs**: Data validation for anchor messages
- **error.rs**: Error handling and recovery strategies
- **degradation.rs**: Graceful degradation management
- **mod.rs**: Validation module exports

### Utilities (`src/utils/`)
- **config.rs**: Configuration management and parameter handling
- **monitor.rs**: Performance monitoring and profiling
- **mod.rs**: Utility module exports

### Main Files
- **lib.rs**: Library root with public API exports
- **main.rs**: Demo application and example usage

## Module Dependencies

```
main.rs
├── lib.rs (public API)
├── core/ (fundamental types)
├── algorithms/ (positioning math)
│   ├── depends on: core/
├── processing/ (signal processing)
│   ├── depends on: core/, algorithms/
├── validation/ (data validation)
│   ├── depends on: core/, processing/
└── utils/ (configuration & monitoring)
    ├── depends on: core/
```

## Naming Conventions

- **Modules**: snake_case (e.g., `trilateration.rs`)
- **Structs**: PascalCase (e.g., `Position`, `AdvancedTrilateration`)
- **Functions**: snake_case (e.g., `calculate_position`)
- **Constants**: SCREAMING_SNAKE_CASE (e.g., `SPEED_OF_SOUND_WATER`)
- **Files**: snake_case with descriptive names

## Configuration Files

- **demo_config.json**: Complete system configuration example
- **anchors.json**: Anchor position data format example
- **.kiro/**: Kiro-specific configuration and specifications