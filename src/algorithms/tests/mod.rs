//! Unit tests for core algorithms
//! 
//! This module contains comprehensive unit tests for all trilateration algorithms,
//! coordinate transformations, and GDOP calculations. Tests validate:
//! - Mathematical accuracy under various conditions
//! - Edge case handling with degenerate anchor configurations
//! - Coordinate transformation precision
//! - Performance benchmarks for timing validation

pub mod trilateration_tests;
pub mod gdop_tests;
pub mod precision_tests;
pub mod embedded_tests;
pub mod coordinate_tests;
pub mod performance_tests;

// Re-export test utilities
pub use trilateration_tests::*;
pub use gdop_tests::*;
pub use precision_tests::*;
pub use embedded_tests::*;
pub use coordinate_tests::*;
pub use performance_tests::*;