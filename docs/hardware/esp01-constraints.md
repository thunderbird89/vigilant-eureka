# ESP01-Class Device Constraints and Optimization

This document provides comprehensive guidance for deploying the underwater positioning beacon system on ESP01-class microcomputers, addressing hardware constraints and optimization strategies specific to these resource-limited devices.

## Table of Contents

- [Hardware Overview](#hardware-overview)
- [Memory Constraints](#memory-constraints)
- [Power Management](#power-management)
- [GPIO Pin Limitations](#gpio-pin-limitations)
- [Processing Constraints](#processing-constraints)
- [Storage Limitations](#storage-limitations)
- [Communication Constraints](#communication-constraints)
- [Optimization Strategies](#optimization-strategies)
- [Configuration Guidelines](#configuration-guidelines)
- [Performance Monitoring](#performance-monitoring)
- [Troubleshooting](#troubleshooting)

## Hardware Overview

### ESP01 Specifications

The ESP01 and similar microcomputers have significant resource constraints:

| Resource | ESP01 | ESP01S | ESP32-C3 | Optimization Target |
|----------|-------|--------|----------|-------------------|
| RAM | 32KB | 80KB | 400KB | <70% usage |
| Flash | 512KB | 1MB | 4MB | <80% usage |
| CPU | 80MHz | 80MHz | 160MHz | Efficient algorithms |
| GPIO Pins | 4 usable | 4 usable | 22 | Pin multiplexing |
| Power | 3.3V | 3.3V | 3.3V | <20% radio duty cycle |

### System Architecture Constraints

```
┌─────────────────────────────────────────┐
│ ESP01 System Architecture               │
├─────────────────────────────────────────┤
│ Application Code        │ ~20KB RAM     │
│ System Buffers         │ ~8KB RAM      │
│ Network Stack          │ ~4KB RAM      │
├─────────────────────────────────────────┤
│ Firmware/Bootloader    │ ~64KB Flash   │
│ Application Code       │ ~300KB Flash  │
│ Configuration/Data     │ ~100KB Flash  │
│ OTA Reserve           │ ~48KB Flash   │
└─────────────────────────────────────────┘
```

## Memory Constraints

### RAM Usage Optimization

#### Memory Layout Strategy

```rust
// Optimized memory allocation patterns
use heapless::{Vec, String, pool::{Pool, Node}};

// Use stack-allocated collections where possible
type MessageBuffer = Vec<u8, 256>;  // Stack-allocated, max 256 bytes
type BeaconId = String<64>;         // Stack-allocated string

// Pre-allocated memory pools for frequent allocations
static mut MEMORY: [Node<[u8; 64]>; 16] = [Node::new(); 16];
static POOL: Pool<[u8; 64]> = Pool::new();

pub struct OptimizedBeaconController {
    message_pool: &'static Pool<[u8; 64]>,
    position_buffer: MessageBuffer,
    status_buffer: MessageBuffer,
}

impl OptimizedBeaconController {
    pub fn new() -> Self {
        // Initialize memory pool
        unsafe { POOL.grow(&mut MEMORY) };
        
        Self {
            message_pool: &POOL,
            position_buffer: Vec::new(),
            status_buffer: Vec::new(),
        }
    }
    
    pub fn process_message(&mut self) -> Result<(), BeaconError> {
        // Use pool allocation for temporary data
        if let Some(mut buffer) = self.message_pool.alloc() {
            // Process message using pooled memory
            // Memory automatically returned to pool when dropped
            Ok(())
        } else {
            Err(BeaconError::SystemError(
                SystemErrorType::ResourceExhausted,
                "Memory pool exhausted".into()
            ))
        }
    }
}
```

#### Memory Usage Guidelines

1. **Stack Allocation Priority**
   ```rust
   // Preferred: Stack-allocated arrays
   let mut buffer: [u8; 256] = [0; 256];
   
   // Avoid: Heap allocation
   let mut buffer = Vec::with_capacity(256);
   ```

2. **Streaming Algorithms**
   ```rust
   // Process data in chunks to avoid large buffers
   pub fn process_gps_data_streaming(
       &mut self, 
       data_stream: impl Iterator<Item = u8>
   ) -> Result<GpsPosition, GpsError> {
       let mut parser_state = GpsParserState::new();
       
       for byte in data_stream {
           if let Some(position) = parser_state.process_byte(byte)? {
               return Ok(position);
           }
       }
       
       Err(GpsError::IncompleteData)
   }
   ```

3. **Memory Pool Usage**
   ```rust
   // Use object pools for frequently allocated/deallocated objects
   pub struct MessagePool {
       pool: Pool<MessageBuffer>,
   }
   
   impl MessagePool {
       pub fn get_buffer(&self) -> Option<PooledBuffer> {
           self.pool.alloc()
       }
   }
   ```

### Memory Monitoring

```rust
pub struct MemoryMonitor {
    peak_usage: usize,
    current_usage: usize,
    allocation_count: u32,
}

impl MemoryMonitor {
    pub fn check_memory_usage(&mut self) -> MemoryStatus {
        let free_heap = esp_get_free_heap_size();
        let total_heap = esp_get_heap_size();
        let usage_percent = ((total_heap - free_heap) * 100) / total_heap;
        
        MemoryStatus {
            free_bytes: free_heap,
            used_bytes: total_heap - free_heap,
            usage_percent,
            fragmentation_level: self.calculate_fragmentation(),
        }
    }
    
    pub fn is_memory_critical(&self) -> bool {
        let status = self.check_memory_usage();
        status.usage_percent > 70 || status.fragmentation_level > 0.3
    }
}
```

## Power Management

### Radio Duty Cycling

The radio is the largest power consumer and must be aggressively managed:

```rust
pub struct RadioDutyCycler {
    active_time_ms: u32,
    sleep_time_ms: u32,
    current_state: RadioState,
    last_transition: Instant,
}

impl RadioDutyCycler {
    pub fn new(duty_cycle_percent: u8) -> Self {
        let cycle_time_ms = 10000; // 10 second cycle
        let active_time_ms = (cycle_time_ms * duty_cycle_percent as u32) / 100;
        let sleep_time_ms = cycle_time_ms - active_time_ms;
        
        Self {
            active_time_ms,
            sleep_time_ms,
            current_state: RadioState::Sleep,
            last_transition: Instant::now(),
        }
    }
    
    pub fn should_wake_radio(&mut self) -> bool {
        match self.current_state {
            RadioState::Sleep => {
                if self.last_transition.elapsed().as_millis() as u32 >= self.sleep_time_ms {
                    self.current_state = RadioState::Active;
                    self.last_transition = Instant::now();
                    true
                } else {
                    false
                }
            },
            RadioState::Active => {
                if self.last_transition.elapsed().as_millis() as u32 >= self.active_time_ms {
                    self.current_state = RadioState::Sleep;
                    self.last_transition = Instant::now();
                }
                false
            }
        }
    }
}
```

### CPU Frequency Scaling

```rust
pub struct CpuFrequencyManager {
    current_frequency: CpuFrequency,
    workload_monitor: WorkloadMonitor,
}

#[derive(Clone, Copy)]
pub enum CpuFrequency {
    Low = 80_000_000,    // 80 MHz
    Medium = 160_000_000, // 160 MHz
    High = 240_000_000,   // 240 MHz (if supported)
}

impl CpuFrequencyManager {
    pub fn adjust_frequency(&mut self) -> Result<(), SystemError> {
        let workload = self.workload_monitor.get_current_workload();
        
        let target_frequency = match workload {
            WorkloadLevel::Idle => CpuFrequency::Low,
            WorkloadLevel::Light => CpuFrequency::Low,
            WorkloadLevel::Medium => CpuFrequency::Medium,
            WorkloadLevel::Heavy => CpuFrequency::High,
        };
        
        if target_frequency != self.current_frequency {
            self.set_cpu_frequency(target_frequency)?;
            self.current_frequency = target_frequency;
        }
        
        Ok(())
    }
    
    fn set_cpu_frequency(&self, frequency: CpuFrequency) -> Result<(), SystemError> {
        // Platform-specific frequency scaling implementation
        unsafe {
            esp_pm_configure(&esp_pm_config_esp32_t {
                max_freq_mhz: frequency as u32 / 1_000_000,
                min_freq_mhz: 10, // Minimum frequency
                light_sleep_enable: true,
            })
        }
        .map_err(|_| SystemError::HardwareFault("CPU frequency scaling failed".into()))
    }
}
```

### Peripheral Power Gating

```rust
pub struct PeripheralPowerManager {
    active_peripherals: HashSet<Peripheral>,
    power_gates: HashMap<Peripheral, PowerGate>,
}

#[derive(Hash, Eq, PartialEq)]
pub enum Peripheral {
    Gps,
    Transceiver,
    PowerMonitor,
    StatusLed,
    CommunicationModule,
}

impl PeripheralPowerManager {
    pub fn enable_peripheral(&mut self, peripheral: Peripheral) -> Result<(), SystemError> {
        if !self.active_peripherals.contains(&peripheral) {
            if let Some(gate) = self.power_gates.get(&peripheral) {
                gate.enable()?;
                self.active_peripherals.insert(peripheral);
            }
        }
        Ok(())
    }
    
    pub fn disable_peripheral(&mut self, peripheral: Peripheral) -> Result<(), SystemError> {
        if self.active_peripherals.contains(&peripheral) {
            if let Some(gate) = self.power_gates.get(&peripheral) {
                gate.disable()?;
                self.active_peripherals.remove(&peripheral);
            }
        }
        Ok(())
    }
    
    pub fn power_save_mode(&mut self) -> Result<(), SystemError> {
        // Disable non-essential peripherals
        self.disable_peripheral(Peripheral::StatusLed)?;
        self.disable_peripheral(Peripheral::CommunicationModule)?;
        
        // Reduce GPS update frequency
        self.configure_gps_power_save()?;
        
        Ok(())
    }
}
```

## GPIO Pin Limitations

### Pin Multiplexing Strategy

ESP01 has only 4 usable GPIO pins, requiring careful multiplexing:

```rust
pub struct GpioPinMultiplexer {
    pin_assignments: HashMap<GpioFunction, GpioPin>,
    current_mode: HashMap<GpioPin, GpioMode>,
    multiplexed_pins: HashMap<GpioPin, Vec<GpioFunction>>,
}

#[derive(Hash, Eq, PartialEq)]
pub enum GpioFunction {
    GpsEnable,
    PowerMonitor,
    StatusLed,
    EmergencyButton,
    SpiChipSelect,
    I2cSda,
    I2cScl,
}

impl GpioPinMultiplexer {
    pub fn new() -> Self {
        let mut multiplexer = Self {
            pin_assignments: HashMap::new(),
            current_mode: HashMap::new(),
            multiplexed_pins: HashMap::new(),
        };
        
        // Configure pin multiplexing for ESP01
        multiplexer.setup_esp01_multiplexing();
        multiplexer
    }
    
    fn setup_esp01_multiplexing(&mut self) {
        // GPIO0: Boot mode / Emergency button (input with pullup)
        self.multiplexed_pins.insert(GpioPin::Gpio0, vec![
            GpioFunction::EmergencyButton,
        ]);
        
        // GPIO2: GPS Enable / Status LED (output)
        self.multiplexed_pins.insert(GpioPin::Gpio2, vec![
            GpioFunction::GpsEnable,
            GpioFunction::StatusLed,
        ]);
        
        // GPIO1 (TX): UART TX / SPI MOSI (when not using UART)
        // GPIO3 (RX): UART RX / SPI MISO (when not using UART)
    }
    
    pub fn configure_pin(&mut self, function: GpioFunction, mode: GpioMode) -> Result<(), SystemError> {
        if let Some(pin) = self.pin_assignments.get(&function) {
            // Check if pin is available for this function
            if self.is_pin_available(*pin, function) {
                self.set_pin_mode(*pin, mode)?;
                self.current_mode.insert(*pin, mode);
                Ok(())
            } else {
                Err(SystemError::ResourceExhausted(
                    format!("GPIO pin {:?} not available for {:?}", pin, function)
                ))
            }
        } else {
            Err(SystemError::HardwareFault(
                format!("No pin assigned for function {:?}", function)
            ))
        }
    }
    
    pub fn time_multiplex_pin(&mut self, pin: GpioPin, functions: &[(GpioFunction, Duration)]) -> Result<(), SystemError> {
        // Time-division multiplexing for shared pins
        for (function, duration) in functions {
            self.configure_pin(*function, GpioMode::Output)?;
            
            // Perform function-specific operation
            match function {
                GpioFunction::StatusLed => self.blink_status_led()?,
                GpioFunction::GpsEnable => self.toggle_gps_power()?,
                _ => {}
            }
            
            // Wait for specified duration
            esp_delay_ms(duration.as_millis() as u32);
        }
        
        Ok(())
    }
}
```

### I2C Bus Sharing

```rust
pub struct I2cBusManager {
    bus: I2cDriver,
    devices: HashMap<I2cAddress, I2cDevice>,
    current_device: Option<I2cAddress>,
}

impl I2cBusManager {
    pub fn new() -> Result<Self, SystemError> {
        let config = I2cConfig::new()
            .baudrate(100_000.Hz())
            .sda(GpioPin::Gpio2)  // Shared with status LED
            .scl(GpioPin::Gpio0); // Shared with emergency button
            
        let bus = I2cDriver::new(config)?;
        
        Ok(Self {
            bus,
            devices: HashMap::new(),
            current_device: None,
        })
    }
    
    pub fn communicate_with_device<T>(&mut self, address: I2cAddress, operation: impl FnOnce(&mut I2cDriver) -> Result<T, I2cError>) -> Result<T, SystemError> {
        // Switch to I2C mode if needed
        if self.current_device != Some(address) {
            self.configure_pins_for_i2c()?;
            self.current_device = Some(address);
        }
        
        // Perform I2C operation
        operation(&mut self.bus)
            .map_err(|e| SystemError::HardwareFault(format!("I2C error: {:?}", e)))
    }
    
    fn configure_pins_for_i2c(&mut self) -> Result<(), SystemError> {
        // Configure GPIO pins for I2C operation
        // This may conflict with other functions using the same pins
        Ok(())
    }
}
```

## Processing Constraints

### Cooperative Multitasking

ESP01 is single-core, requiring cooperative multitasking:

```rust
pub struct TaskScheduler {
    tasks: Vec<Task>,
    current_task: usize,
    max_task_time_ms: u32,
}

pub struct Task {
    name: &'static str,
    handler: Box<dyn FnMut() -> TaskResult>,
    priority: TaskPriority,
    last_run: Instant,
    run_interval: Duration,
}

pub enum TaskResult {
    Completed,
    Yield,
    Error(String),
}

impl TaskScheduler {
    pub fn new() -> Self {
        Self {
            tasks: Vec::new(),
            current_task: 0,
            max_task_time_ms: 50, // Maximum 50ms per task
        }
    }
    
    pub fn add_task(&mut self, task: Task) {
        self.tasks.push(task);
        // Sort by priority
        self.tasks.sort_by_key(|t| t.priority as u8);
    }
    
    pub fn run_scheduler(&mut self) -> Result<(), SystemError> {
        let start_time = Instant::now();
        
        while start_time.elapsed().as_millis() < self.max_task_time_ms as u128 {
            if self.tasks.is_empty() {
                break;
            }
            
            let task = &mut self.tasks[self.current_task];
            
            // Check if task should run
            if task.last_run.elapsed() >= task.run_interval {
                let task_start = Instant::now();
                
                match (task.handler)() {
                    TaskResult::Completed => {
                        task.last_run = Instant::now();
                    },
                    TaskResult::Yield => {
                        // Task yielded, continue next time
                        break;
                    },
                    TaskResult::Error(msg) => {
                        return Err(SystemError::ThreadPanic(msg));
                    }
                }
                
                // Prevent task from running too long
                if task_start.elapsed().as_millis() > 20 {
                    break; // Yield to other tasks
                }
            }
            
            // Move to next task
            self.current_task = (self.current_task + 1) % self.tasks.len();
        }
        
        Ok(())
    }
}
```

### Efficient Algorithms

```rust
// Use efficient algorithms optimized for limited CPU
pub struct OptimizedCoordinateCalculator {
    // Pre-computed lookup tables to avoid expensive calculations
    sin_table: [f32; 360],
    cos_table: [f32; 360],
}

impl OptimizedCoordinateCalculator {
    pub fn new() -> Self {
        let mut calc = Self {
            sin_table: [0.0; 360],
            cos_table: [0.0; 360],
        };
        
        // Pre-compute trigonometric values
        for i in 0..360 {
            let radians = (i as f32) * std::f32::consts::PI / 180.0;
            calc.sin_table[i] = radians.sin();
            calc.cos_table[i] = radians.cos();
        }
        
        calc
    }
    
    pub fn fast_distance_calculation(&self, pos1: &GeodeticPosition, pos2: &GeodeticPosition) -> f32 {
        // Use Haversine formula with lookup tables
        let lat1_deg = pos1.latitude as i32;
        let lat2_deg = pos2.latitude as i32;
        let lon_diff_deg = (pos2.longitude - pos1.longitude) as i32;
        
        // Use lookup tables instead of expensive sin/cos calculations
        let sin_lat1 = self.sin_table[lat1_deg.abs() as usize % 360];
        let sin_lat2 = self.sin_table[lat2_deg.abs() as usize % 360];
        let cos_lat1 = self.cos_table[lat1_deg.abs() as usize % 360];
        let cos_lat2 = self.cos_table[lat2_deg.abs() as usize % 360];
        let cos_lon_diff = self.cos_table[lon_diff_deg.abs() as usize % 360];
        
        // Simplified Haversine calculation
        let a = (sin_lat2 - sin_lat1).powi(2) + cos_lat1 * cos_lat2 * (1.0 - cos_lon_diff);
        let c = 2.0 * a.sqrt().atan2((1.0 - a).sqrt());
        
        6371000.0 * c // Earth radius in meters
    }
}
```

## Storage Limitations

### Flash Memory Management

```rust
pub struct FlashStorageManager {
    config_partition: FlashPartition,
    log_partition: FlashPartition,
    ota_partition: FlashPartition,
    wear_leveling: WearLevelingManager,
}

impl FlashStorageManager {
    pub fn new() -> Result<Self, SystemError> {
        Ok(Self {
            config_partition: FlashPartition::new("config", 32 * 1024)?, // 32KB
            log_partition: FlashPartition::new("logs", 64 * 1024)?,     // 64KB
            ota_partition: FlashPartition::new("ota", 256 * 1024)?,     // 256KB
            wear_leveling: WearLevelingManager::new(),
        })
    }
    
    pub fn store_configuration(&mut self, config: &BeaconConfig) -> Result<(), SystemError> {
        // Compress configuration before storage
        let compressed_config = self.compress_config(config)?;
        
        // Use wear leveling to distribute writes
        self.wear_leveling.write_with_leveling(
            &mut self.config_partition,
            &compressed_config
        )?;
        
        Ok(())
    }
    
    pub fn circular_log_write(&mut self, log_entry: &LogEntry) -> Result<(), SystemError> {
        // Implement circular logging to manage limited flash space
        let serialized_entry = self.serialize_log_entry(log_entry)?;
        
        if self.log_partition.remaining_space() < serialized_entry.len() {
            // Wrap around and overwrite oldest entries
            self.log_partition.reset_write_position()?;
        }
        
        self.log_partition.write(&serialized_entry)?;
        Ok(())
    }
    
    fn compress_config(&self, config: &BeaconConfig) -> Result<Vec<u8>, SystemError> {
        // Use simple compression to reduce storage requirements
        let serialized = bincode::serialize(config)
            .map_err(|_| SystemError::ResourceExhausted("Serialization failed".into()))?;
        
        // Simple run-length encoding for configuration data
        self.rle_compress(&serialized)
    }
    
    fn rle_compress(&self, data: &[u8]) -> Result<Vec<u8>, SystemError> {
        let mut compressed = Vec::new();
        let mut i = 0;
        
        while i < data.len() {
            let current_byte = data[i];
            let mut count = 1;
            
            // Count consecutive identical bytes
            while i + count < data.len() && data[i + count] == current_byte && count < 255 {
                count += 1;
            }
            
            compressed.push(count as u8);
            compressed.push(current_byte);
            i += count;
        }
        
        Ok(compressed)
    }
}
```

## Communication Constraints

### Bandwidth Optimization

```rust
pub struct BandwidthOptimizer {
    compression_enabled: bool,
    message_queue: VecDeque<CompressedMessage>,
    batch_size: usize,
}

impl BandwidthOptimizer {
    pub fn optimize_message(&self, message: &[u8]) -> Result<Vec<u8>, SystemError> {
        let mut optimized = message.to_vec();
        
        // Apply compression if enabled
        if self.compression_enabled {
            optimized = self.compress_message(&optimized)?;
        }
        
        // Apply delta encoding for position updates
        optimized = self.delta_encode_position(&optimized)?;
        
        // Add error correction for underwater transmission
        optimized = self.add_error_correction(&optimized)?;
        
        Ok(optimized)
    }
    
    fn compress_message(&self, message: &[u8]) -> Result<Vec<u8>, SystemError> {
        // Simple dictionary compression for repeated patterns
        let mut compressed = Vec::new();
        let dictionary = self.build_message_dictionary();
        
        let mut i = 0;
        while i < message.len() {
            let mut matched = false;
            
            // Try to match against dictionary entries
            for (pattern, code) in &dictionary {
                if i + pattern.len() <= message.len() && &message[i..i + pattern.len()] == pattern {
                    compressed.push(0xFF); // Escape byte
                    compressed.push(*code);
                    i += pattern.len();
                    matched = true;
                    break;
                }
            }
            
            if !matched {
                compressed.push(message[i]);
                i += 1;
            }
        }
        
        Ok(compressed)
    }
    
    fn delta_encode_position(&self, message: &[u8]) -> Result<Vec<u8>, SystemError> {
        // Encode position as delta from previous position to reduce data size
        // Implementation depends on message format
        Ok(message.to_vec()) // Placeholder
    }
}
```

## Optimization Strategies

### Configuration Optimization

```toml
# ESP01-optimized configuration
[hardware]
memory_optimization_enabled = true
cpu_frequency_scaling_enabled = true
peripheral_power_gating_enabled = true
watchdog_timeout_s = 30

[hardware.memory]
heap_size_kb = 20
stack_size_kb = 4
message_buffer_size = 256
log_buffer_size = 512
max_concurrent_allocations = 8

[hardware.power]
radio_duty_cycle_percent = 15
cpu_idle_frequency_mhz = 80
peripheral_timeout_s = 60
deep_sleep_enabled = true

[hardware.gpio_optimization]
pin_multiplexing_enabled = true
i2c_bus_sharing_enabled = true
spi_bus_sharing_enabled = true

[transmission]
# Optimized for low bandwidth
interval_ms = 15000  # Longer intervals
power_level = 60     # Reduced power
message_compression_enabled = true
batch_transmission_enabled = true

[gps]
# Optimized for power saving
update_interval_s = 60
power_save_enabled = true
acquisition_timeout_s = 45
cold_start_timeout_s = 180

[power]
# Aggressive power management
power_save_mode_threshold_percent = 40.0
cpu_frequency_scaling_enabled = true
peripheral_power_gating_enabled = true
```

### Code Optimization Patterns

```rust
// Use const generics to avoid runtime allocations
pub struct OptimizedMessageBuffer<const N: usize> {
    buffer: [u8; N],
    length: usize,
}

impl<const N: usize> OptimizedMessageBuffer<N> {
    pub const fn new() -> Self {
        Self {
            buffer: [0; N],
            length: 0,
        }
    }
    
    pub fn push(&mut self, byte: u8) -> Result<(), BufferError> {
        if self.length < N {
            self.buffer[self.length] = byte;
            self.length += 1;
            Ok(())
        } else {
            Err(BufferError::Full)
        }
    }
}

// Use bit fields to pack data efficiently
#[repr(packed)]
pub struct CompactBeaconStatus {
    // Pack multiple boolean flags into a single byte
    flags: u8, // bit 0: GPS locked, bit 1: charging, bit 2: emergency, etc.
    battery_percent: u8,
    signal_quality: u8,
    satellite_count: u8,
}

impl CompactBeaconStatus {
    pub fn set_gps_locked(&mut self, locked: bool) {
        if locked {
            self.flags |= 0x01;
        } else {
            self.flags &= !0x01;
        }
    }
    
    pub fn is_gps_locked(&self) -> bool {
        (self.flags & 0x01) != 0
    }
}
```

## Configuration Guidelines

### Memory Configuration

```toml
[hardware.memory]
# Keep heap usage below 70% of available RAM
max_heap_usage_percent = 65
stack_size_kb = 4
message_queue_size = 8
log_buffer_entries = 32

# Use memory pools for frequent allocations
memory_pool_enabled = true
pool_block_size = 64
pool_block_count = 16
```

### Power Configuration

```toml
[hardware.power]
# Target <20% radio duty cycle
radio_duty_cycle_percent = 15
radio_sleep_time_ms = 8500
radio_active_time_ms = 1500

# CPU frequency scaling
min_cpu_frequency_mhz = 80
max_cpu_frequency_mhz = 160
frequency_scaling_enabled = true

# Peripheral power management
peripheral_timeout_s = 30
deep_sleep_enabled = true
light_sleep_enabled = true
```

### GPIO Configuration

```toml
[hardware.gpio_pin_mapping]
# ESP01 pin assignments with multiplexing
gpio0 = "emergency_button"  # Also boot mode pin
gpio2 = "gps_enable"        # Also status LED (time multiplexed)
gpio1_tx = "uart_tx"        # UART TX or SPI MOSI
gpio3_rx = "uart_rx"        # UART RX or SPI MISO

[hardware.pin_multiplexing]
enabled = true
status_led_duration_ms = 100
gps_enable_duration_ms = 1000
multiplexing_interval_ms = 5000
```

## Performance Monitoring

### Resource Usage Monitoring

```rust
pub struct ResourceMonitor {
    memory_monitor: MemoryMonitor,
    cpu_monitor: CpuMonitor,
    power_monitor: PowerMonitor,
    flash_monitor: FlashMonitor,
}

impl ResourceMonitor {
    pub fn get_system_metrics(&self) -> SystemMetrics {
        SystemMetrics {
            memory_usage: self.memory_monitor.get_usage(),
            cpu_usage: self.cpu_monitor.get_usage(),
            power_consumption: self.power_monitor.get_consumption(),
            flash_usage: self.flash_monitor.get_usage(),
            system_health_score: self.calculate_health_score(),
        }
    }
    
    pub fn check_resource_constraints(&self) -> Vec<ResourceWarning> {
        let mut warnings = Vec::new();
        
        // Check memory usage
        if self.memory_monitor.get_usage_percent() > 70.0 {
            warnings.push(ResourceWarning::HighMemoryUsage);
        }
        
        // Check flash wear
        if self.flash_monitor.get_wear_level() > 0.8 {
            warnings.push(ResourceWarning::FlashWearHigh);
        }
        
        // Check power consumption
        if self.power_monitor.get_average_consumption_mw() > 150.0 {
            warnings.push(ResourceWarning::HighPowerConsumption);
        }
        
        warnings
    }
}
```

### Performance Optimization Feedback

```rust
pub struct PerformanceOptimizer {
    metrics_history: VecDeque<SystemMetrics>,
    optimization_rules: Vec<OptimizationRule>,
}

impl PerformanceOptimizer {
    pub fn analyze_and_optimize(&mut self, current_metrics: SystemMetrics) -> Vec<OptimizationAction> {
        self.metrics_history.push_back(current_metrics);
        
        // Keep only recent history
        if self.metrics_history.len() > 10 {
            self.metrics_history.pop_front();
        }
        
        let mut actions = Vec::new();
        
        // Apply optimization rules
        for rule in &self.optimization_rules {
            if let Some(action) = rule.evaluate(&self.metrics_history) {
                actions.push(action);
            }
        }
        
        actions
    }
}

pub enum OptimizationAction {
    ReduceTransmissionFrequency,
    EnablePowerSaveMode,
    ReduceCpuFrequency,
    DisableNonEssentialFeatures,
    CompactFlashStorage,
    ClearLogBuffer,
}
```

## Troubleshooting

### Common Issues and Solutions

#### Memory Issues

**Problem**: Out of memory errors or system instability
```
Error: Memory allocation failed
System: Heap exhausted, 2KB remaining
```

**Solutions**:
1. Enable memory optimization:
   ```toml
   [hardware.memory]
   memory_optimization_enabled = true
   max_heap_usage_percent = 60
   ```

2. Reduce buffer sizes:
   ```toml
   message_buffer_size = 128
   log_buffer_entries = 16
   ```

3. Enable memory pooling:
   ```rust
   // Use pre-allocated memory pools
   static mut MEMORY_POOL: [u8; 4096] = [0; 4096];
   ```

#### Power Issues

**Problem**: Battery drains too quickly
```
Warning: Battery level 15% after 8 hours
Expected: >50% after 24 hours
```

**Solutions**:
1. Reduce radio duty cycle:
   ```toml
   [hardware.power]
   radio_duty_cycle_percent = 10
   ```

2. Enable aggressive power management:
   ```toml
   deep_sleep_enabled = true
   peripheral_power_gating_enabled = true
   ```

3. Optimize transmission intervals:
   ```toml
   [transmission]
   interval_ms = 30000  # 30 seconds instead of 5
   ```

#### GPIO Conflicts

**Problem**: GPIO pin conflicts between functions
```
Error: GPIO2 already in use by GPS enable
Cannot configure status LED
```

**Solutions**:
1. Enable pin multiplexing:
   ```toml
   [hardware.pin_multiplexing]
   enabled = true
   ```

2. Use time-division multiplexing:
   ```rust
   // Alternate between functions
   gpio_multiplexer.time_multiplex_pin(GpioPin::Gpio2, &[
       (GpioFunction::GpsEnable, Duration::from_secs(1)),
       (GpioFunction::StatusLed, Duration::from_millis(100)),
   ])?;
   ```

#### Flash Storage Issues

**Problem**: Flash storage full or wearing out
```
Warning: Flash partition 95% full
Error: Flash write failed - wear leveling exhausted
```

**Solutions**:
1. Enable circular logging:
   ```toml
   [hardware.storage]
   circular_logging_enabled = true
   max_log_entries = 100
   ```

2. Compress stored data:
   ```toml
   config_compression_enabled = true
   log_compression_enabled = true
   ```

3. Implement wear leveling:
   ```rust
   flash_manager.enable_wear_leveling(true);
   ```

### Diagnostic Commands

```bash
# Check resource usage
beacon diagnostic --memory --power --flash

# Monitor performance in real-time
beacon performance --real-time --duration 300

# Analyze optimization opportunities
beacon optimize --analyze --recommendations

# Test hardware constraints
beacon test-constraints --memory-stress --power-stress --gpio-test
```

### Performance Tuning Checklist

- [ ] Memory usage below 70% of available RAM
- [ ] Radio duty cycle below 20%
- [ ] CPU frequency scaling enabled
- [ ] Peripheral power gating configured
- [ ] GPIO pin multiplexing optimized
- [ ] Flash storage wear leveling enabled
- [ ] Message compression enabled
- [ ] Transmission intervals optimized for power
- [ ] Error recovery mechanisms tested
- [ ] Performance monitoring active

This comprehensive guide provides the foundation for successfully deploying the beacon system on ESP01-class devices while respecting their significant resource constraints and optimizing for reliable operation in marine environments.