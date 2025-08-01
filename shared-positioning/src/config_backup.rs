// Configuration backup and restore system for beacon reliability
// Implements automatic configuration backup, validation, and emergency restore functionality

use std::time::{Duration, SystemTime};
use std::collections::HashMap;
use std::fs::{self, File};
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use serde::{Serialize, Deserialize};

use crate::{
    BeaconConfig, BeaconError, ConfigurationErrorType, ErrorSeverity
};

/// Configuration backup errors
#[derive(Debug, Clone, PartialEq)]
pub enum ConfigBackupError {
    BackupFailed { reason: String },
    RestoreFailed { reason: String },
    ValidationFailed { errors: Vec<String> },
    FileSystemError { operation: String, error: String },
    CorruptedBackup { backup_id: String, reason: String },
    NoBackupAvailable,
    BackupTooOld { age_hours: u64, max_age_hours: u64 },
}

impl std::fmt::Display for ConfigBackupError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ConfigBackupError::BackupFailed { reason } => {
                write!(f, "Configuration backup failed: {}", reason)
            }
            ConfigBackupError::RestoreFailed { reason } => {
                write!(f, "Configuration restore failed: {}", reason)
            }
            ConfigBackupError::ValidationFailed { errors } => {
                write!(f, "Configuration validation failed: {}", errors.join(", "))
            }
            ConfigBackupError::FileSystemError { operation, error } => {
                write!(f, "File system error during {}: {}", operation, error)
            }
            ConfigBackupError::CorruptedBackup { backup_id, reason } => {
                write!(f, "Backup {} is corrupted: {}", backup_id, reason)
            }
            ConfigBackupError::NoBackupAvailable => {
                write!(f, "No backup configuration available")
            }
            ConfigBackupError::BackupTooOld { age_hours, max_age_hours } => {
                write!(f, "Backup is too old: {} hours (max: {} hours)", age_hours, max_age_hours)
            }
        }
    }
}

impl std::error::Error for ConfigBackupError {}

/// Configuration backup metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BackupMetadata {
    pub backup_id: String,
    pub creation_time: SystemTime,
    pub beacon_id: uuid::Uuid,
    pub config_version: String,
    pub backup_type: BackupType,
    pub checksum: String,
    pub file_size: u64,
    pub description: String,
    pub tags: Vec<String>,
}

/// Types of configuration backups
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum BackupType {
    Automatic,      // Scheduled automatic backup
    Manual,         // User-initiated backup
    PreUpdate,      // Before configuration update
    Emergency,      // Emergency backup before critical operation
    Factory,        // Factory default configuration
}

/// Configuration backup entry
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConfigBackup {
    pub metadata: BackupMetadata,
    pub configuration: BeaconConfig,
    pub validation_results: ValidationResults,
}

/// Configuration validation results
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ValidationResults {
    pub is_valid: bool,
    pub errors: Vec<String>,
    pub warnings: Vec<String>,
    pub validation_time: SystemTime,
    pub validator_version: String,
}

/// Backup manager configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BackupManagerConfig {
    pub backup_directory: PathBuf,
    pub max_backups: usize,
    pub max_backup_age_hours: u64,
    pub auto_backup_enabled: bool,
    pub auto_backup_interval_hours: u64,
    pub compress_backups: bool,
    pub encrypt_backups: bool,
    pub validation_required: bool,
}

impl Default for BackupManagerConfig {
    fn default() -> Self {
        Self {
            backup_directory: PathBuf::from("./config_backups"),
            max_backups: 50,
            max_backup_age_hours: 24 * 30,  // 30 days
            auto_backup_enabled: true,
            auto_backup_interval_hours: 24,  // Daily backups
            compress_backups: true,
            encrypt_backups: false,  // Simplified for now
            validation_required: true,
        }
    }
}

/// Configuration backup and restore manager
pub struct ConfigBackupManager {
    config: BackupManagerConfig,
    backups: HashMap<String, ConfigBackup>,
    last_auto_backup: Option<SystemTime>,
    backup_statistics: BackupStatistics,
}

/// Backup operation statistics
#[derive(Debug, Clone, Default)]
pub struct BackupStatistics {
    pub total_backups_created: u64,
    pub total_restores_performed: u64,
    pub successful_backups: u64,
    pub failed_backups: u64,
    pub successful_restores: u64,
    pub failed_restores: u64,
    pub corrupted_backups_detected: u32,
    pub automatic_cleanups: u32,
    pub last_backup_time: Option<SystemTime>,
    pub last_restore_time: Option<SystemTime>,
}

impl ConfigBackupManager {
    /// Create new configuration backup manager
    pub fn new(config: BackupManagerConfig) -> Result<Self, ConfigBackupError> {
        // Create backup directory if it doesn't exist
        if !config.backup_directory.exists() {
            fs::create_dir_all(&config.backup_directory)
                .map_err(|e| ConfigBackupError::FileSystemError {
                    operation: "create_backup_directory".to_string(),
                    error: e.to_string(),
                })?;
        }

        let mut manager = Self {
            config,
            backups: HashMap::new(),
            last_auto_backup: None,
            backup_statistics: BackupStatistics::default(),
        };

        // Load existing backups
        manager.load_existing_backups()?;

        Ok(manager)
    }

    /// Create a configuration backup
    pub fn create_backup(
        &mut self,
        configuration: &BeaconConfig,
        backup_type: BackupType,
        description: String,
    ) -> Result<String, ConfigBackupError> {
        self.backup_statistics.total_backups_created += 1;

        // Generate backup ID
        let backup_id = format!("backup_{}_{}", 
            configuration.beacon_id,
            SystemTime::now().duration_since(SystemTime::UNIX_EPOCH)
                .unwrap_or(Duration::from_secs(0)).as_secs()
        );

        // Validate configuration if required
        let validation_results = if self.config.validation_required {
            self.validate_configuration(configuration)?
        } else {
            ValidationResults {
                is_valid: true,
                errors: Vec::new(),
                warnings: Vec::new(),
                validation_time: SystemTime::now(),
                validator_version: "1.0.0".to_string(),
            }
        };

        if !validation_results.is_valid {
            self.backup_statistics.failed_backups += 1;
            return Err(ConfigBackupError::ValidationFailed {
                errors: validation_results.errors,
            });
        }

        // Calculate checksum
        let config_data = serde_json::to_string(configuration)
            .map_err(|e| ConfigBackupError::BackupFailed {
                reason: format!("Serialization failed: {}", e),
            })?;
        
        let checksum = self.calculate_checksum(&config_data);

        // Create backup metadata
        let metadata = BackupMetadata {
            backup_id: backup_id.clone(),
            creation_time: SystemTime::now(),
            beacon_id: configuration.beacon_id,
            config_version: "1.0.0".to_string(),  // Should come from config
            backup_type,
            checksum,
            file_size: config_data.len() as u64,
            description,
            tags: Vec::new(),
        };

        // Create backup entry
        let backup = ConfigBackup {
            metadata,
            configuration: configuration.clone(),
            validation_results,
        };

        // Save backup to file
        self.save_backup_to_file(&backup)?;

        // Store in memory
        self.backups.insert(backup_id.clone(), backup);

        // Update statistics
        self.backup_statistics.successful_backups += 1;
        self.backup_statistics.last_backup_time = Some(SystemTime::now());

        // Cleanup old backups if necessary
        self.cleanup_old_backups()?;

        Ok(backup_id)
    }

    /// Restore configuration from backup
    pub fn restore_backup(&mut self, backup_id: &str) -> Result<BeaconConfig, ConfigBackupError> {
        self.backup_statistics.total_restores_performed += 1;

        let backup = self.backups.get(backup_id)
            .ok_or(ConfigBackupError::NoBackupAvailable)?
            .clone();

        // Verify backup integrity
        self.verify_backup_integrity(&backup)?;

        // Check backup age
        let backup_age = SystemTime::now()
            .duration_since(backup.metadata.creation_time)
            .unwrap_or(Duration::from_secs(0));

        if backup_age.as_secs() / 3600 > self.config.max_backup_age_hours {
            return Err(ConfigBackupError::BackupTooOld {
                age_hours: backup_age.as_secs() / 3600,
                max_age_hours: self.config.max_backup_age_hours,
            });
        }

        // Validate restored configuration
        if self.config.validation_required {
            let validation_results = self.validate_configuration(&backup.configuration)?;
            if !validation_results.is_valid {
                self.backup_statistics.failed_restores += 1;
                return Err(ConfigBackupError::ValidationFailed {
                    errors: validation_results.errors,
                });
            }
        }

        // Update statistics
        self.backup_statistics.successful_restores += 1;
        self.backup_statistics.last_restore_time = Some(SystemTime::now());

        Ok(backup.configuration)
    }

    /// Get the most recent valid backup
    pub fn get_latest_backup(&self) -> Option<&ConfigBackup> {
        self.backups.values()
            .filter(|backup| backup.validation_results.is_valid)
            .max_by_key(|backup| backup.metadata.creation_time)
    }

    /// Get emergency restore configuration
    pub fn get_emergency_restore_config(&self) -> Result<BeaconConfig, ConfigBackupError> {
        // Try to find the most recent valid backup
        if let Some(backup) = self.get_latest_backup() {
            return Ok(backup.configuration.clone());
        }

        // Try to find a factory backup
        for backup in self.backups.values() {
            if matches!(backup.metadata.backup_type, BackupType::Factory) {
                return Ok(backup.configuration.clone());
            }
        }

        // No backup available
        Err(ConfigBackupError::NoBackupAvailable)
    }

    /// Create automatic backup if needed
    pub fn check_auto_backup(&mut self, current_config: &BeaconConfig) -> Result<Option<String>, ConfigBackupError> {
        if !self.config.auto_backup_enabled {
            return Ok(None);
        }

        let should_backup = match self.last_auto_backup {
            Some(last_backup) => {
                let elapsed = SystemTime::now()
                    .duration_since(last_backup)
                    .unwrap_or(Duration::from_secs(0));
                
                elapsed.as_secs() / 3600 >= self.config.auto_backup_interval_hours
            }
            None => true,
        };

        if should_backup {
            let backup_id = self.create_backup(
                current_config,
                BackupType::Automatic,
                "Automatic scheduled backup".to_string(),
            )?;
            
            self.last_auto_backup = Some(SystemTime::now());
            Ok(Some(backup_id))
        } else {
            Ok(None)
        }
    }

    /// List available backups
    pub fn list_backups(&self) -> Vec<&BackupMetadata> {
        let mut backups: Vec<&BackupMetadata> = self.backups.values()
            .map(|backup| &backup.metadata)
            .collect();
        
        backups.sort_by(|a, b| b.creation_time.cmp(&a.creation_time));
        backups
    }

    /// Delete a backup
    pub fn delete_backup(&mut self, backup_id: &str) -> Result<(), ConfigBackupError> {
        if let Some(backup) = self.backups.remove(backup_id) {
            // Delete backup file
            let backup_file = self.get_backup_file_path(&backup.metadata.backup_id);
            if backup_file.exists() {
                fs::remove_file(&backup_file)
                    .map_err(|e| ConfigBackupError::FileSystemError {
                        operation: "delete_backup_file".to_string(),
                        error: e.to_string(),
                    })?;
            }
        }

        Ok(())
    }

    /// Get backup statistics
    pub fn get_statistics(&self) -> &BackupStatistics {
        &self.backup_statistics
    }

    /// Validate configuration
    fn validate_configuration(&self, config: &BeaconConfig) -> Result<ValidationResults, ConfigBackupError> {
        let mut errors = Vec::new();
        let mut warnings = Vec::new();

        // Basic validation checks
        if config.transmission.interval_ms < 1000 || config.transmission.interval_ms > 60000 {
            errors.push("Transmission interval must be between 1000-60000ms".to_string());
        }

        if config.gps.acquisition_timeout_s < 10 || config.gps.acquisition_timeout_s > 300 {
            warnings.push("GPS acquisition timeout outside recommended range (10-300s)".to_string());
        }

        if config.power.low_battery_threshold_percent <= config.power.critical_battery_threshold_percent {
            errors.push("Low battery threshold must be higher than critical threshold".to_string());
        }

        // Additional validation logic would go here...

        Ok(ValidationResults {
            is_valid: errors.is_empty(),
            errors,
            warnings,
            validation_time: SystemTime::now(),
            validator_version: "1.0.0".to_string(),
        })
    }

    /// Calculate checksum for configuration data
    fn calculate_checksum(&self, data: &str) -> String {
        // Simple checksum implementation - in production would use proper hashing
        let char_sum = data.chars().map(|c| c as u32).sum::<u32>() as usize;
        format!("{:x}", data.len() * 31 + char_sum)
    }

    /// Verify backup integrity
    fn verify_backup_integrity(&self, backup: &ConfigBackup) -> Result<(), ConfigBackupError> {
        let config_data = serde_json::to_string(&backup.configuration)
            .map_err(|e| ConfigBackupError::CorruptedBackup {
                backup_id: backup.metadata.backup_id.clone(),
                reason: format!("Serialization failed: {}", e),
            })?;

        let calculated_checksum = self.calculate_checksum(&config_data);
        
        if calculated_checksum != backup.metadata.checksum {
            return Err(ConfigBackupError::CorruptedBackup {
                backup_id: backup.metadata.backup_id.clone(),
                reason: "Checksum mismatch".to_string(),
            });
        }

        Ok(())
    }

    /// Save backup to file
    fn save_backup_to_file(&self, backup: &ConfigBackup) -> Result<(), ConfigBackupError> {
        let backup_file = self.get_backup_file_path(&backup.metadata.backup_id);
        
        let backup_data = serde_json::to_string_pretty(backup)
            .map_err(|e| ConfigBackupError::BackupFailed {
                reason: format!("Serialization failed: {}", e),
            })?;

        let mut file = File::create(&backup_file)
            .map_err(|e| ConfigBackupError::FileSystemError {
                operation: "create_backup_file".to_string(),
                error: e.to_string(),
            })?;

        file.write_all(backup_data.as_bytes())
            .map_err(|e| ConfigBackupError::FileSystemError {
                operation: "write_backup_file".to_string(),
                error: e.to_string(),
            })?;

        Ok(())
    }

    /// Load existing backups from disk
    fn load_existing_backups(&mut self) -> Result<(), ConfigBackupError> {
        if !self.config.backup_directory.exists() {
            return Ok(());
        }

        let entries = fs::read_dir(&self.config.backup_directory)
            .map_err(|e| ConfigBackupError::FileSystemError {
                operation: "read_backup_directory".to_string(),
                error: e.to_string(),
            })?;

        for entry in entries {
            let entry = entry.map_err(|e| ConfigBackupError::FileSystemError {
                operation: "read_directory_entry".to_string(),
                error: e.to_string(),
            })?;

            let path = entry.path();
            if path.extension().and_then(|s| s.to_str()) == Some("json") {
                match self.load_backup_from_file(&path) {
                    Ok(backup) => {
                        self.backups.insert(backup.metadata.backup_id.clone(), backup);
                    }
                    Err(e) => {
                        eprintln!("Failed to load backup from {:?}: {}", path, e);
                        self.backup_statistics.corrupted_backups_detected += 1;
                    }
                }
            }
        }

        Ok(())
    }

    /// Load backup from file
    fn load_backup_from_file(&self, path: &Path) -> Result<ConfigBackup, ConfigBackupError> {
        let mut file = File::open(path)
            .map_err(|e| ConfigBackupError::FileSystemError {
                operation: "open_backup_file".to_string(),
                error: e.to_string(),
            })?;

        let mut contents = String::new();
        file.read_to_string(&mut contents)
            .map_err(|e| ConfigBackupError::FileSystemError {
                operation: "read_backup_file".to_string(),
                error: e.to_string(),
            })?;

        let backup: ConfigBackup = serde_json::from_str(&contents)
            .map_err(|e| ConfigBackupError::CorruptedBackup {
                backup_id: path.file_stem()
                    .and_then(|s| s.to_str())
                    .unwrap_or("unknown")
                    .to_string(),
                reason: format!("Deserialization failed: {}", e),
            })?;

        // Verify integrity
        self.verify_backup_integrity(&backup)?;

        Ok(backup)
    }

    /// Get backup file path
    fn get_backup_file_path(&self, backup_id: &str) -> PathBuf {
        self.config.backup_directory.join(format!("{}.json", backup_id))
    }

    /// Cleanup old backups
    fn cleanup_old_backups(&mut self) -> Result<(), ConfigBackupError> {
        let now = SystemTime::now();
        let max_age = Duration::from_secs(self.config.max_backup_age_hours * 3600);

        // Remove backups that are too old
        let mut to_remove = Vec::new();
        for (backup_id, backup) in &self.backups {
            let age = now.duration_since(backup.metadata.creation_time)
                .unwrap_or(Duration::from_secs(0));
            
            if age > max_age && !matches!(backup.metadata.backup_type, BackupType::Factory) {
                to_remove.push(backup_id.clone());
            }
        }

        for backup_id in to_remove {
            self.delete_backup(&backup_id)?;
            self.backup_statistics.automatic_cleanups += 1;
        }

        // Remove excess backups (keep only max_backups)
        if self.backups.len() > self.config.max_backups {
            let mut backups_by_age: Vec<_> = self.backups.iter()
                .filter(|(_, backup)| !matches!(backup.metadata.backup_type, BackupType::Factory))
                .map(|(id, backup)| (id.clone(), backup.metadata.creation_time))
                .collect();
            
            backups_by_age.sort_by(|a, b| a.1.cmp(&b.1));

            let excess_count = backups_by_age.len().saturating_sub(self.config.max_backups);
            for (backup_id, _) in backups_by_age.into_iter().take(excess_count) {
                self.delete_backup(&backup_id)?;
                self.backup_statistics.automatic_cleanups += 1;
            }
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    #[test]
    fn test_backup_manager_creation() {
        let temp_dir = TempDir::new().unwrap();
        let config = BackupManagerConfig {
            backup_directory: temp_dir.path().to_path_buf(),
            ..Default::default()
        };

        let manager = ConfigBackupManager::new(config).unwrap();
        assert_eq!(manager.backups.len(), 0);
    }

    #[test]
    fn test_create_backup() {
        use uuid::Uuid;
        let temp_dir = TempDir::new().unwrap();
        let config = BackupManagerConfig {
            backup_directory: temp_dir.path().to_path_buf(),
            validation_required: false,
            ..Default::default()
        };

        let mut manager = ConfigBackupManager::new(config).unwrap();
        let beacon_config = BeaconConfig::new(Uuid::new_v4());

        let backup_id = manager.create_backup(
            &beacon_config,
            BackupType::Manual,
            "Test backup".to_string(),
        ).unwrap();

        assert!(!backup_id.is_empty());
        assert_eq!(manager.backups.len(), 1);
    }
}