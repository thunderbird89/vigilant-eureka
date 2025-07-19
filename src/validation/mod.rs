//! Data validation and quality assurance

pub mod accuracy;
pub mod data;
pub mod error;
pub mod degradation;

pub use accuracy::AccuracyValidator;
pub use data::DataValidator;
pub use error::ErrorReporter;
pub use degradation::GracefulDegradationManager;