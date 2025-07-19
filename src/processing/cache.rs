use nalgebra::Vector3;
use crate::core::{Anchor, Position};
use std::collections::HashMap;
use std::time::{Duration, Instant};

/// Cache for optimization results to avoid redundant calculations
pub struct OptimizationCache {
    /// Cache of GDOP calculations for anchor configurations
    gdop_cache: HashMap<String, f64>,
    /// Cache of optimal anchor subsets
    anchor_subset_cache: HashMap<String, Vec<usize>>,
    /// Cache of environmental corrections
    environmental_correction_cache: HashMap<String, f64>,
    /// Cache expiration time
    cache_expiration: Duration,
    /// Last cache cleanup time
    last_cleanup: Instant,
    /// Cache hit count for statistics
    hit_count: usize,
    /// Cache miss count for statistics
    miss_count: usize,
}

impl Default for OptimizationCache {
    fn default() -> Self {
        Self {
            gdop_cache: HashMap::new(),
            anchor_subset_cache: HashMap::new(),
            environmental_correction_cache: HashMap::new(),
            cache_expiration: Duration::from_secs(60), // 1 minute default expiration
            last_cleanup: Instant::now(),
            hit_count: 0,
            miss_count: 0,
        }
    }
}

impl OptimizationCache {
    /// Create a new optimization cache with default settings
    pub fn new() -> Self {
        Self::default()
    }

    /// Create a new optimization cache with custom expiration time
    pub fn with_expiration(expiration_secs: u64) -> Self {
        Self {
            cache_expiration: Duration::from_secs(expiration_secs),
            ..Self::default()
        }
    }

    /// Generate a cache key for anchor positions
    fn generate_anchor_key(&self, positions: &[Vector3<f64>]) -> String {
        // Create a string representation of positions with reduced precision
        // to allow for small variations in position to still hit the cache
        positions.iter()
            .map(|p| format!("{:.2},{:.2},{:.2}", p.x, p.y, p.z))
            .collect::<Vec<String>>()
            .join("|")
    }

    /// Generate a cache key for environmental parameters
    fn generate_env_key(&self, temp: f64, salinity: f64, depth: f64, pressure: f64) -> String {
        format!("{:.1}|{:.1}|{:.1}|{:.1}", temp, salinity, depth, pressure)
    }

    /// Get cached GDOP value if available
    pub fn get_cached_gdop(&mut self, 
                          positions: &[Vector3<f64>], 
                          target: &Vector3<f64>) -> Option<f64> {
        // Generate cache key
        let key = format!("{}|{:.2},{:.2},{:.2}", 
                         self.generate_anchor_key(positions),
                         target.x, target.y, target.z);
        
        // Check cache
        if let Some(&gdop) = self.gdop_cache.get(&key) {
            self.hit_count += 1;
            Some(gdop)
        } else {
            self.miss_count += 1;
            None
        }
    }

    /// Store GDOP value in cache
    pub fn cache_gdop(&mut self, 
                     positions: &[Vector3<f64>], 
                     target: &Vector3<f64>,
                     gdop: f64) {
        // Generate cache key
        let key = format!("{}|{:.2},{:.2},{:.2}", 
                         self.generate_anchor_key(positions),
                         target.x, target.y, target.z);
        
        // Store in cache
        self.gdop_cache.insert(key, gdop);
        
        // Perform cleanup if needed
        self.cleanup_if_needed();
    }

    /// Get cached optimal anchor subset if available
    pub fn get_cached_anchor_subset(&mut self, 
                                   positions: &[Vector3<f64>],
                                   target: &Vector3<f64>) -> Option<Vec<usize>> {
        // Generate cache key
        let key = format!("{}|{:.2},{:.2},{:.2}", 
                         self.generate_anchor_key(positions),
                         target.x, target.y, target.z);
        
        // Check cache
        if let Some(subset) = self.anchor_subset_cache.get(&key) {
            self.hit_count += 1;
            Some(subset.clone())
        } else {
            self.miss_count += 1;
            None
        }
    }

    /// Store optimal anchor subset in cache
    pub fn cache_anchor_subset(&mut self, 
                              positions: &[Vector3<f64>], 
                              target: &Vector3<f64>,
                              subset: Vec<usize>) {
        // Generate cache key
        let key = format!("{}|{:.2},{:.2},{:.2}", 
                         self.generate_anchor_key(positions),
                         target.x, target.y, target.z);
        
        // Store in cache
        self.anchor_subset_cache.insert(key, subset);
        
        // Perform cleanup if needed
        self.cleanup_if_needed();
    }

    /// Get cached environmental correction if available
    pub fn get_cached_environmental_correction(&mut self, 
                                             temp: f64, 
                                             salinity: f64, 
                                             depth: f64, 
                                             pressure: f64) -> Option<f64> {
        // Generate cache key
        let key = self.generate_env_key(temp, salinity, depth, pressure);
        
        // Check cache
        if let Some(&correction) = self.environmental_correction_cache.get(&key) {
            self.hit_count += 1;
            Some(correction)
        } else {
            self.miss_count += 1;
            None
        }
    }

    /// Store environmental correction in cache
    pub fn cache_environmental_correction(&mut self, 
                                        temp: f64, 
                                        salinity: f64, 
                                        depth: f64, 
                                        pressure: f64,
                                        correction: f64) {
        // Generate cache key
        let key = self.generate_env_key(temp, salinity, depth, pressure);
        
        // Store in cache
        self.environmental_correction_cache.insert(key, correction);
        
        // Perform cleanup if needed
        self.cleanup_if_needed();
    }

    /// Clean up expired cache entries if needed
    fn cleanup_if_needed(&mut self) {
        let now = Instant::now();
        if now.duration_since(self.last_cleanup) > self.cache_expiration {
            // Time to clean up
            self.gdop_cache.clear();
            self.anchor_subset_cache.clear();
            self.environmental_correction_cache.clear();
            self.last_cleanup = now;
        }
    }

    /// Get cache statistics
    pub fn get_statistics(&self) -> (usize, usize, f64) {
        let total = self.hit_count + self.miss_count;
        let hit_rate = if total > 0 {
            self.hit_count as f64 / total as f64
        } else {
            0.0
        };
        
        (self.hit_count, self.miss_count, hit_rate)
    }

    /// Clear all caches
    pub fn clear(&mut self) {
        self.gdop_cache.clear();
        self.anchor_subset_cache.clear();
        self.environmental_correction_cache.clear();
        self.last_cleanup = Instant::now();
    }

    /// Set cache expiration time
    pub fn set_expiration(&mut self, expiration_secs: u64) {
        self.cache_expiration = Duration::from_secs(expiration_secs);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    
    #[test]
    fn test_gdop_cache() {
        let mut cache = OptimizationCache::new();
        
        let positions = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(10.0, 0.0, 0.0),
            Vector3::new(0.0, 10.0, 0.0),
            Vector3::new(0.0, 0.0, 10.0),
        ];
        
        let target = Vector3::new(5.0, 5.0, 5.0);
        
        // First access should be a miss
        assert!(cache.get_cached_gdop(&positions, &target).is_none());
        
        // Store a value
        cache.cache_gdop(&positions, &target, 2.5);
        
        // Second access should be a hit
        let cached_gdop = cache.get_cached_gdop(&positions, &target);
        assert!(cached_gdop.is_some());
        assert_eq!(cached_gdop.unwrap(), 2.5);
        
        // Check statistics
        let (hits, misses, hit_rate) = cache.get_statistics();
        assert_eq!(hits, 1);
        assert_eq!(misses, 1);
        assert!((hit_rate - 0.5).abs() < 1e-10);
    }
    
    #[test]
    fn test_anchor_subset_cache() {
        let mut cache = OptimizationCache::new();
        
        let positions = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(10.0, 0.0, 0.0),
            Vector3::new(0.0, 10.0, 0.0),
            Vector3::new(0.0, 0.0, 10.0),
            Vector3::new(10.0, 10.0, 10.0),
        ];
        
        let target = Vector3::new(5.0, 5.0, 5.0);
        let subset = vec![0, 1, 2, 3]; // Best 4 out of 5
        
        // First access should be a miss
        assert!(cache.get_cached_anchor_subset(&positions, &target).is_none());
        
        // Store a value
        cache.cache_anchor_subset(&positions, &target, subset.clone());
        
        // Second access should be a hit
        let cached_subset = cache.get_cached_anchor_subset(&positions, &target);
        assert!(cached_subset.is_some());
        assert_eq!(cached_subset.unwrap(), subset);
    }
    
    #[test]
    fn test_environmental_correction_cache() {
        let mut cache = OptimizationCache::new();
        
        let temp = 15.0;
        let salinity = 35.0;
        let depth = 50.0;
        let pressure = 5.0;
        
        // First access should be a miss
        assert!(cache.get_cached_environmental_correction(temp, salinity, depth, pressure).is_none());
        
        // Store a value
        cache.cache_environmental_correction(temp, salinity, depth, pressure, 1520.0);
        
        // Second access should be a hit
        let cached_correction = cache.get_cached_environmental_correction(temp, salinity, depth, pressure);
        assert!(cached_correction.is_some());
        assert_eq!(cached_correction.unwrap(), 1520.0);
    }
    
    #[test]
    fn test_cache_clear() {
        let mut cache = OptimizationCache::new();
        
        let positions = vec![
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(10.0, 0.0, 0.0),
        ];
        
        let target = Vector3::new(5.0, 0.0, 0.0);
        
        // Store a value
        cache.cache_gdop(&positions, &target, 2.0);
        
        // Clear cache
        cache.clear();
        
        // Access should be a miss after clearing
        assert!(cache.get_cached_gdop(&positions, &target).is_none());
    }
}