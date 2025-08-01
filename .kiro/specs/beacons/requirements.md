# Requirements Document

## Introduction

The beacons feature provides a comprehensive system for underwater positioning beacons that can operate either floating on the surface or anchored at depth. These beacons serve as reference points for the underwater positioning system by broadcasting their precise positions to underwater receivers. The system includes GPS positioning capabilities, power management with monitoring, and cellular communication for remote monitoring and control.

The beacon system is designed to run on microcomputer hardware such as ESP01 or similar chips. This hardware constraint means the system has more computational resources than typical embedded systems but significantly less than personal computers, requiring efficient resource utilization and optimized code design.

**Hardware Constraints:**

- Limited RAM (typically 32-80KB available for application code)
- Flash storage constraints (typically 512KB-4MB total)
- Power consumption critical for battery operation
- GPIO pin limitations requiring efficient hardware interface design
- Single-core processing requiring cooperative multitasking
- WiFi/cellular radio power consumption significantly impacts battery life

## Requirements

### Requirement 1

**User Story:** As a marine researcher, I want beacons to transmit standardized positioning messages, so that underwater receivers can use them for trilateration calculations.

#### Acceptance Criteria

1. WHEN a beacon is operational THEN it SHALL transmit messages using the same format as specified in the receiver's message parser (Version 1 and Version 2 formats)
2. WHEN transmitting messages THEN the beacon SHALL include its current GPS position, timestamp, signal quality, and unique beacon ID
3. WHEN operating THEN the beacon SHALL maintain message transmission intervals of 1-10 seconds (configurable)
4. WHEN transmitting THEN the beacon SHALL calculate and include proper CRC16 checksums for message integrity
5. WHEN message sequence numbers reach maximum value THEN the beacon SHALL roll over sequence numbers gracefully

### Requirement 2

**User Story:** As a system operator, I want beacons to maintain accurate GPS positioning, so that the transmitted location data is reliable for underwater positioning calculations.

#### Acceptance Criteria

1. WHEN a beacon starts up THEN it SHALL acquire GPS lock within 60 seconds under clear sky conditions
2. WHEN GPS is available THEN the beacon SHALL update its position every 1-30 seconds (configurable)
3. WHEN GPS signal is lost THEN the beacon SHALL continue transmitting its last known position with degraded signal quality indicator
4. WHEN GPS accuracy is poor THEN the beacon SHALL indicate reduced signal quality in transmitted messages
5. IF GPS is unavailable for more than 5 minutes THEN the beacon SHALL enter degraded mode and reduce transmission frequency

### Requirement 3

**User Story:** As a field technician, I want to monitor beacon power status remotely, so that I can plan maintenance and prevent unexpected failures.

#### Acceptance Criteria

1. WHEN operating THEN the beacon SHALL continuously monitor battery voltage, current draw, and estimated remaining capacity
2. WHEN battery level drops below 20% THEN the beacon SHALL transmit low battery warnings in status messages
3. WHEN battery level drops below 10% THEN the beacon SHALL enter power conservation mode and reduce transmission frequency
4. WHEN battery level drops below 5% THEN the beacon SHALL transmit emergency shutdown warnings and prepare for safe shutdown
5. WHEN solar charging is available THEN the beacon SHALL monitor charging status and adjust power management accordingly

### Requirement 4

**User Story:** As a system administrator, I want beacons to communicate with shore-based systems via long-range communication link, so that I can monitor system health and update configurations remotely.

#### Acceptance Criteria

1. WHEN long-range communication is available THEN the beacon SHALL establish connection to cloud services or base station every 1-24 hours (configurable)
2. WHEN connected THEN the beacon SHALL upload status reports including position history, power status, and system health metrics
3. WHEN connected THEN the beacon SHALL check for and download configuration updates or firmware updates
4. WHEN communication link fails THEN the beacon SHALL retry connection with exponential backoff up to maximum interval of 6 hours
5. IF long-range communication is unavailable for more than 48 hours THEN the beacon SHALL continue autonomous operation with local logging

### Requirement 5

**User Story:** As a developer, I want to reuse existing receiver code components, so that the system maintains consistency and reduces development effort.

#### Acceptance Criteria

1. WHEN implementing beacon functionality THEN the system SHALL create a shared library containing common message parsing, transceiver interface, and coordinate system components
2. WHEN creating the shared library THEN it SHALL extract message_parser, transceiver_interface, coordinate_system, and error_handling modules from the receiver crate
3. WHEN both beacon and receiver crates are built THEN they SHALL use the shared library for common functionality
4. WHEN the shared library is updated THEN both beacon and receiver systems SHALL benefit from improvements and bug fixes
5. WHEN implementing beacon-specific features THEN they SHALL extend the shared interfaces rather than duplicating code

### Requirement 6

**User Story:** As a system operator, I want beacons to operate reliably in marine environments, so that the positioning system maintains continuous operation.

#### Acceptance Criteria

1. WHEN environmental conditions change THEN the beacon SHALL adapt transmission power and frequency based on conditions
2. WHEN hardware faults are detected THEN the beacon SHALL log errors and attempt automatic recovery procedures
3. WHEN critical system failures occur THEN the beacon SHALL transmit emergency status messages before safe shutdown
4. WHEN operating in harsh conditions THEN the beacon SHALL maintain operation in temperature range -20°C to +60°C
5. WHEN deployed THEN the beacon SHALL provide minimum 30 days of autonomous operation on battery power alone

### Requirement 7

**User Story:** As a field operator, I want to configure beacon parameters remotely, so that I can optimize system performance without physical access.

#### Acceptance Criteria

1. WHEN receiving configuration commands THEN the beacon SHALL validate parameters before applying changes
2. WHEN configuration is updated THEN the beacon SHALL acknowledge successful changes and report new settings
3. WHEN invalid configuration is received THEN the beacon SHALL reject changes and report specific validation errors
4. WHEN configuration changes affect safety THEN the beacon SHALL require confirmation before applying critical changes
5. WHEN factory reset is requested THEN the beacon SHALL restore default settings and confirm reset completion

### Requirement 8

**User Story:** As a developer, I want the beacon system to operate efficiently within microcomputer hardware constraints, so that the system can run reliably on ESP01-class devices with limited resources.

#### Acceptance Criteria

1. WHEN operating THEN the beacon SHALL maintain total RAM usage below 70% of available application memory to prevent system instability
2. WHEN managing power THEN the beacon SHALL implement aggressive power management including radio sleep modes, CPU frequency scaling, and peripheral power gating
3. WHEN interfacing with hardware THEN the beacon SHALL multiplex GPIO pins efficiently to support GPS, transceiver, power monitoring, and status indicators within pin limitations
4. WHEN processing data THEN the beacon SHALL use streaming algorithms and avoid large memory allocations to minimize heap fragmentation
5. WHEN radio is active THEN the beacon SHALL minimize transmission duration and implement duty cycling to reduce average power consumption by at least 80%
