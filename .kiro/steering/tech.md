# Technology Stack

## Language & Build System

- **Language**: Rust (2021 edition)
- **Build system**: Cargo
- **Target platforms**: Embedded systems (microcontrollers) and desktop

## Core Dependencies

- **nalgebra**: Linear algebra operations for matrix calculations and coordinate transformations
- **serde**: Serialization/deserialization for configuration and data structures
- **serde_json**: JSON handling for configuration files
- **rand**: Random number generation for testing and simulation
- **rand_distr**: Statistical distributions for noise simulation

## Common Commands

```bash
# Build the project
cargo build

# Run with optimizations (for performance testing)
cargo build --release

# Run the demo application
cargo run --bin underwater-positioning-demo

# Run tests
cargo test

# Run tests with output
cargo test -- --nocapture

# Check code without building
cargo check

# Format code
cargo fmt

# Run clippy linter
cargo clippy
```

## Architecture Patterns

- **Modular design**: Clear separation between algorithms, processing, validation, and core types
- **Trait-based abstractions**: Use traits for hardware abstraction and algorithm interfaces
- **Error handling**: Comprehensive Result<T, E> usage with custom error types
- **Configuration-driven**: JSON-based configuration with runtime parameter adjustment
- **Performance-first**: Designed for embedded constraints with memory and timing optimization

## Embedded Considerations

- **Memory management**: Avoid dynamic allocations where possible
- **Stack-based arrays**: Prefer fixed-size arrays over Vec for embedded deployment
- **Compile-time configuration**: Feature flags for optional functionality
- **Real-time constraints**: All operations must complete within 100ms
- **Power efficiency**: Designed for battery-powered underwater systems