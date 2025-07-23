use std::collections::HashMap;
use std::time::{Duration, Instant};
use crate::{Position, CompactAnchorMessage};
use nalgebra::Vector3;

/// Cache entry with age-based expiration
#[derive(Debug, Clone)]
pub struct CacheEntry<T> {
    pub data: T,
    pub timestamp: Instant,
    pub access_count: u32,
}

impl<T> CacheEntry<T> {
    pub fn new(data: T) -> Self {
        Self {
            data,
            timestamp: Instant::now(),
            access_count: 0,
        }
    }

    pub fn age(&self) -> Duration {
        self.timestamp.elapsed()
    }

    pub fn access(&mut self) -> &T {
        self.access_count += 1;
        &self.data
    }

    pub fn is_expired(&self, max_age: Duration) -> bool {
        self.age() > max_age
    }
}

/// Anchor data cache with age-based expiration
pub struct AnchorDataCache {
    cache: HashMap<u16, CacheEntry<CompactAnchorMessage>>,
    max_age_ms: u64,
    max_entries: usize,
}

impl AnchorDataCache {
    pub fn new(max_age_ms: u64, max_entries: usize) -> Self {
        Self {
            cache: HashMap::new(),
            max_age_ms,
            max_entries,
        }
    }

    /// Insert or update anchor data in cache
    pub fn insert(&mut self, anchor_id: u16, message: CompactAnchorMessage) {
        // Remove expired entries before inserting
        self.cleanup_expired();

        // If cache is full, remove least recently used entry
        if self.cache.len() >= self.max_entries {
            self.evict_lru();
        }

        self.cache.insert(anchor_id, CacheEntry::new(message));
    }

    /// Get anchor data from cache if not expired
    pub fn get(&mut self, anchor_id: u16) -> Option<CompactAnchorMessage> {
        let max_age = Duration::from_millis(self.max_age_ms);
        
        if let Some(entry) = self.cache.get_mut(&anchor_id) {
            if !entry.is_expired(max_age) {
                entry.access_count += 1;
                return Some(entry.data);
            }
        }
        
        // Remove expired entry if it exists
        if self.cache.contains_key(&anchor_id) {
            self.cache.remove(&anchor_id);
        }
        
        None
    }

    /// Get all valid (non-expired) anchor data
    pub fn get_valid_anchors(&mut self) -> Vec<CompactAnchorMessage> {
        self.cleanup_expired();
        self.cache.values_mut()
            .map(|entry| {
                entry.access_count += 1;
                entry.data
            })
            .collect()
    }

    /// Remove expired entries from cache
    pub fn cleanup_expired(&mut self) {
        let max_age = Duration::from_millis(self.max_age_ms);
        self.cache.retain(|_, entry| !entry.is_expired(max_age));
    }

    /// Evict least recently used entry
    fn evict_lru(&mut self) {
        if let Some((&lru_id, _)) = self.cache.iter()
            .min_by_key(|(_, entry)| (entry.access_count, entry.timestamp)) {
            self.cache.remove(&lru_id);
        }
    }

    /// Get cache statistics
    pub fn get_stats(&self) -> CacheStats {
        let total_entries = self.cache.len();
        let total_accesses = self.cache.values().map(|e| e.access_count).sum();
        let avg_age_ms = if total_entries > 0 {
            self.cache.values()
                .map(|e| e.age().as_millis() as u64)
                .sum::<u64>() / total_entries as u64
        } else {
            0
        };

        CacheStats {
            total_entries,
            total_accesses,
            avg_age_ms,
            max_entries: self.max_entries,
            max_age_ms: self.max_age_ms,
        }
    }

    /// Clear all cache entries
    pub fn clear(&mut self) {
        self.cache.clear();
    }
}

/// Intermediate calculation cache for repeated operations
pub struct CalculationCache {
    coordinate_transforms: HashMap<String, CacheEntry<Vector3<f64>>>,
    distance_calculations: HashMap<String, CacheEntry<f64>>,
    geometry_assessments: HashMap<String, CacheEntry<GeometryQuality>>,
    max_age_ms: u64,
    max_entries_per_type: usize,
}

#[derive(Debug, Clone, PartialEq)]
pub enum GeometryQuality {
    Excellent,
    Good,
    Acceptable,
    Poor,
    Degenerate,
}

impl CalculationCache {
    pub fn new(max_age_ms: u64, max_entries_per_type: usize) -> Self {
        Self {
            coordinate_transforms: HashMap::new(),
            distance_calculations: HashMap::new(),
            geometry_assessments: HashMap::new(),
            max_age_ms,
            max_entries_per_type,
        }
    }

    /// Cache coordinate transformation result
    pub fn cache_coordinate_transform(&mut self, key: String, result: Vector3<f64>) {
        let max_age = Duration::from_millis(self.max_age_ms);
        self.coordinate_transforms.retain(|_, entry| !entry.is_expired(max_age));

        if self.coordinate_transforms.len() >= self.max_entries_per_type {
            if let Some(lru_key) = self.coordinate_transforms.iter()
                .min_by_key(|(_, entry)| (entry.access_count, entry.timestamp))
                .map(|(k, _)| k.clone()) {
                self.coordinate_transforms.remove(&lru_key);
            }
        }

        self.coordinate_transforms.insert(key, CacheEntry::new(result));
    }

    /// Get cached coordinate transformation
    pub fn get_coordinate_transform(&mut self, key: &str) -> Option<Vector3<f64>> {
        let max_age = Duration::from_millis(self.max_age_ms);
        
        if let Some(entry) = self.coordinate_transforms.get_mut(key) {
            if !entry.is_expired(max_age) {
                return Some(entry.access().clone());
            } else {
                self.coordinate_transforms.remove(key);
            }
        }
        None
    }

    /// Cache distance calculation result
    pub fn cache_distance(&mut self, key: String, distance: f64) {
        let max_age = Duration::from_millis(self.max_age_ms);
        self.distance_calculations.retain(|_, entry| !entry.is_expired(max_age));

        if self.distance_calculations.len() >= self.max_entries_per_type {
            if let Some(lru_key) = self.distance_calculations.iter()
                .min_by_key(|(_, entry)| (entry.access_count, entry.timestamp))
                .map(|(k, _)| k.clone()) {
                self.distance_calculations.remove(&lru_key);
            }
        }

        self.distance_calculations.insert(key, CacheEntry::new(distance));
    }

    /// Get cached distance calculation
    pub fn get_distance(&mut self, key: &str) -> Option<f64> {
        let max_age = Duration::from_millis(self.max_age_ms);
        
        if let Some(entry) = self.distance_calculations.get_mut(key) {
            if !entry.is_expired(max_age) {
                return Some(entry.access().clone());
            } else {
                self.distance_calculations.remove(key);
            }
        }
        None
    }

    /// Cache geometry quality assessment
    pub fn cache_geometry_quality(&mut self, key: String, quality: GeometryQuality) {
        let max_age = Duration::from_millis(self.max_age_ms);
        self.geometry_assessments.retain(|_, entry| !entry.is_expired(max_age));

        if self.geometry_assessments.len() >= self.max_entries_per_type {
            if let Some(lru_key) = self.geometry_assessments.iter()
                .min_by_key(|(_, entry)| (entry.access_count, entry.timestamp))
                .map(|(k, _)| k.clone()) {
                self.geometry_assessments.remove(&lru_key);
            }
        }

        self.geometry_assessments.insert(key, CacheEntry::new(quality));
    }

    /// Get cached geometry quality assessment
    pub fn get_geometry_quality(&mut self, key: &str) -> Option<GeometryQuality> {
        let max_age = Duration::from_millis(self.max_age_ms);
        
        if let Some(entry) = self.geometry_assessments.get_mut(key) {
            if !entry.is_expired(max_age) {
                return Some(entry.access().clone());
            } else {
                self.geometry_assessments.remove(key);
            }
        }
        None
    }



    /// Clear all caches
    pub fn clear(&mut self) {
        self.coordinate_transforms.clear();
        self.distance_calculations.clear();
        self.geometry_assessments.clear();
    }

    /// Get comprehensive cache statistics
    pub fn get_stats(&self) -> CalculationCacheStats {
        CalculationCacheStats {
            coordinate_transforms: self.coordinate_transforms.len(),
            distance_calculations: self.distance_calculations.len(),
            geometry_assessments: self.geometry_assessments.len(),
            total_entries: self.coordinate_transforms.len() + 
                          self.distance_calculations.len() + 
                          self.geometry_assessments.len(),
            max_entries_per_type: self.max_entries_per_type,
            max_age_ms: self.max_age_ms,
        }
    }
}

/// Memory pool for predictable allocation patterns
pub struct MemoryPool<T> {
    pool: Vec<Option<T>>,
    free_indices: Vec<usize>,
    allocated_count: usize,
}

impl<T: Default + Clone> MemoryPool<T> {
    pub fn new(capacity: usize) -> Self {
        let mut pool = Vec::with_capacity(capacity);
        let mut free_indices = Vec::with_capacity(capacity);
        
        for i in 0..capacity {
            pool.push(Some(T::default()));
            free_indices.push(i);
        }

        Self {
            pool,
            free_indices,
            allocated_count: 0,
        }
    }

    /// Allocate an object from the pool
    pub fn allocate(&mut self) -> Option<PoolHandle<T>> {
        if let Some(index) = self.free_indices.pop() {
            if let Some(obj) = self.pool[index].take() {
                self.allocated_count += 1;
                return Some(PoolHandle {
                    index,
                    object: obj,
                });
            }
        }
        None
    }

    /// Return an object to the pool
    pub fn deallocate(&mut self, handle: PoolHandle<T>) {
        if handle.index < self.pool.len() {
            self.pool[handle.index] = Some(handle.object);
            self.free_indices.push(handle.index);
            self.allocated_count = self.allocated_count.saturating_sub(1);
        }
    }

    /// Get pool utilization statistics
    pub fn get_utilization(&self) -> PoolUtilization {
        PoolUtilization {
            total_capacity: self.pool.len(),
            allocated_count: self.allocated_count,
            free_count: self.free_indices.len(),
            utilization_percent: (self.allocated_count as f64 / self.pool.len() as f64) * 100.0,
        }
    }

    /// Clear and reset the pool
    pub fn clear(&mut self) {
        self.free_indices.clear();
        for i in 0..self.pool.len() {
            self.pool[i] = Some(T::default());
            self.free_indices.push(i);
        }
        self.allocated_count = 0;
    }
}

/// Handle for objects allocated from memory pool
pub struct PoolHandle<T> {
    index: usize,
    object: T,
}

impl<T> PoolHandle<T> {
    pub fn get(&self) -> &T {
        &self.object
    }

    pub fn get_mut(&mut self) -> &mut T {
        &mut self.object
    }
}

/// Lazy evaluation wrapper for non-critical calculations
pub struct LazyValue<T, F>
where
    F: FnOnce() -> T,
{
    value: Option<T>,
    initializer: Option<F>,
}

impl<T, F> LazyValue<T, F>
where
    F: FnOnce() -> T,
{
    pub fn new(initializer: F) -> Self {
        Self {
            value: None,
            initializer: Some(initializer),
        }
    }

    pub fn get(&mut self) -> &T {
        if self.value.is_none() {
            if let Some(init) = self.initializer.take() {
                self.value = Some(init());
            }
        }
        self.value.as_ref().unwrap()
    }

    pub fn is_initialized(&self) -> bool {
        self.value.is_some()
    }

    pub fn reset(&mut self, new_initializer: F) {
        self.value = None;
        self.initializer = Some(new_initializer);
    }
}

/// Statistics for anchor data cache
#[derive(Debug)]
pub struct CacheStats {
    pub total_entries: usize,
    pub total_accesses: u32,
    pub avg_age_ms: u64,
    pub max_entries: usize,
    pub max_age_ms: u64,
}

/// Statistics for calculation cache
#[derive(Debug)]
pub struct CalculationCacheStats {
    pub coordinate_transforms: usize,
    pub distance_calculations: usize,
    pub geometry_assessments: usize,
    pub total_entries: usize,
    pub max_entries_per_type: usize,
    pub max_age_ms: u64,
}

/// Memory pool utilization statistics
#[derive(Debug)]
pub struct PoolUtilization {
    pub total_capacity: usize,
    pub allocated_count: usize,
    pub free_count: usize,
    pub utilization_percent: f64,
}

/// Comprehensive optimization manager
pub struct OptimizationManager {
    pub anchor_cache: AnchorDataCache,
    pub calculation_cache: CalculationCache,
    pub vector_pool: MemoryPool<Vector3<f64>>,
    pub position_pool: MemoryPool<Position>,
}

impl OptimizationManager {
    pub fn new() -> Self {
        Self {
            anchor_cache: AnchorDataCache::new(5000, 16), // 5 second cache, max 16 anchors
            calculation_cache: CalculationCache::new(10000, 32), // 10 second cache, max 32 per type
            vector_pool: MemoryPool::new(64), // Pool of 64 Vector3 objects
            position_pool: MemoryPool::new(32), // Pool of 32 Position objects
        }
    }

    /// Generate comprehensive optimization report
    pub fn generate_report(&self) -> String {
        let mut report = String::new();
        report.push_str("=== OPTIMIZATION SYSTEM REPORT ===\n\n");

        // Anchor cache statistics
        let anchor_stats = self.anchor_cache.get_stats();
        report.push_str("ANCHOR DATA CACHE:\n");
        report.push_str(&format!("  Entries: {}/{}\n", anchor_stats.total_entries, anchor_stats.max_entries));
        report.push_str(&format!("  Total accesses: {}\n", anchor_stats.total_accesses));
        report.push_str(&format!("  Average age: {} ms\n", anchor_stats.avg_age_ms));
        report.push_str(&format!("  Max age limit: {} ms\n", anchor_stats.max_age_ms));

        // Calculation cache statistics
        let calc_stats = self.calculation_cache.get_stats();
        report.push_str("\nCALCULATION CACHE:\n");
        report.push_str(&format!("  Coordinate transforms: {}\n", calc_stats.coordinate_transforms));
        report.push_str(&format!("  Distance calculations: {}\n", calc_stats.distance_calculations));
        report.push_str(&format!("  Geometry assessments: {}\n", calc_stats.geometry_assessments));
        report.push_str(&format!("  Total entries: {}\n", calc_stats.total_entries));
        report.push_str(&format!("  Max per type: {}\n", calc_stats.max_entries_per_type));

        // Memory pool statistics
        let vector_util = self.vector_pool.get_utilization();
        let position_util = self.position_pool.get_utilization();
        report.push_str("\nMEMORY POOLS:\n");
        report.push_str(&format!("  Vector3 pool: {}/{} ({:.1}% utilized)\n", 
                                vector_util.allocated_count, vector_util.total_capacity, vector_util.utilization_percent));
        report.push_str(&format!("  Position pool: {}/{} ({:.1}% utilized)\n", 
                                position_util.allocated_count, position_util.total_capacity, position_util.utilization_percent));

        // Recommendations
        report.push_str("\nOPTIMIZATION RECOMMENDATIONS:\n");
        if anchor_stats.total_entries as f64 / anchor_stats.max_entries as f64 > 0.8 {
            report.push_str("  - Consider increasing anchor cache size\n");
        }
        if calc_stats.total_entries as f64 / (calc_stats.max_entries_per_type * 3) as f64 > 0.8 {
            report.push_str("  - Consider increasing calculation cache size\n");
        }
        if vector_util.utilization_percent > 80.0 {
            report.push_str("  - Consider increasing Vector3 pool size\n");
        }
        if position_util.utilization_percent > 80.0 {
            report.push_str("  - Consider increasing Position pool size\n");
        }
        if anchor_stats.total_accesses == 0 {
            report.push_str("  - Anchor cache not being utilized - check integration\n");
        }

        report
    }

    /// Clear all caches and reset pools
    pub fn reset(&mut self) {
        self.anchor_cache.clear();
        self.calculation_cache.clear();
        self.vector_pool.clear();
        self.position_pool.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::thread;

    #[test]
    fn test_anchor_cache_basic() {
        let mut cache = AnchorDataCache::new(1000, 5);
        let message = CompactAnchorMessage::new(1, 1000, 32.0, 45.0, 10.0, 255);
        
        cache.insert(1, message);
        assert!(cache.get(1).is_some());
        assert!(cache.get(2).is_none());
    }

    #[test]
    fn test_anchor_cache_expiration() {
        let mut cache = AnchorDataCache::new(50, 5); // 50ms expiration
        let message = CompactAnchorMessage::new(1, 1000, 32.0, 45.0, 10.0, 255);
        
        cache.insert(1, message);
        assert!(cache.get(1).is_some());
        
        thread::sleep(Duration::from_millis(60));
        assert!(cache.get(1).is_none()); // Should be expired
    }

    #[test]
    fn test_calculation_cache() {
        let mut cache = CalculationCache::new(1000, 10);
        let vector = Vector3::new(1.0, 2.0, 3.0);
        
        cache.cache_coordinate_transform("test_key".to_string(), vector);
        let cached = cache.get_coordinate_transform("test_key");
        assert!(cached.is_some());
        assert_eq!(cached.unwrap(), vector);
    }

    #[test]
    fn test_memory_pool() {
        let mut pool: MemoryPool<Vector3<f64>> = MemoryPool::new(3);
        
        let handle1 = pool.allocate().unwrap();
        let handle2 = pool.allocate().unwrap();
        let handle3 = pool.allocate().unwrap();
        
        // Pool should be full
        assert!(pool.allocate().is_none());
        
        let util = pool.get_utilization();
        assert_eq!(util.allocated_count, 3);
        assert_eq!(util.free_count, 0);
        
        // Return one object
        pool.deallocate(handle1);
        assert!(pool.allocate().is_some());
    }

    #[test]
    fn test_lazy_value() {
        let mut lazy = LazyValue::new(|| {
            // Expensive computation simulation
            42
        });
        
        assert!(!lazy.is_initialized());
        assert_eq!(*lazy.get(), 42);
        assert!(lazy.is_initialized());
    }

    #[test]
    fn test_optimization_manager() {
        let manager = OptimizationManager::new();
        let report = manager.generate_report();
        
        assert!(report.contains("ANCHOR DATA CACHE"));
        assert!(report.contains("CALCULATION CACHE"));
        assert!(report.contains("MEMORY POOLS"));
    }

    #[test]
    fn test_cache_lru_eviction() {
        let mut cache = AnchorDataCache::new(10000, 2); // Max 2 entries
        
        let msg1 = CompactAnchorMessage::new(1, 1000, 32.0, 45.0, 10.0, 255);
        let msg2 = CompactAnchorMessage::new(2, 1001, 32.1, 45.1, 10.1, 254);
        let msg3 = CompactAnchorMessage::new(3, 1002, 32.2, 45.2, 10.2, 253);
        
        cache.insert(1, msg1);
        cache.insert(2, msg2);
        
        // Access first entry to make it more recently used
        cache.get(1);
        
        // Insert third entry - should evict entry 2 (least recently used)
        cache.insert(3, msg3);
        
        assert!(cache.get(1).is_some()); // Should still be there
        assert!(cache.get(2).is_none());  // Should be evicted
        assert!(cache.get(3).is_some()); // Should be there
    }
}