# Underwater Positioning Beacon System Documentation

This directory contains comprehensive documentation for the underwater positioning beacon system, including API documentation, deployment guides, troubleshooting information, and performance optimization guides.

## Documentation Structure

### API Documentation
- [Shared Library API](api/shared-positioning.md) - Complete API reference for the shared positioning library
- [Beacon Controller API](api/beacon-controller.md) - Beacon controller and orchestration API
- [CLI Interface](api/cli-interface.md) - Command-line interface documentation

### Deployment and Configuration
- [Quick Start Guide](deployment/quick-start.md) - Get started with beacon deployment
- [Configuration Guide](deployment/configuration.md) - Comprehensive configuration documentation
- [Deployment Guide](deployment/deployment-guide.md) - Step-by-step deployment procedures
- [Fleet Management](deployment/fleet-management.md) - Managing multiple beacon deployments

### Hardware and Optimization
- [Hardware Constraints](hardware/esp01-constraints.md) - ESP01-class device constraints and optimization
- [Performance Tuning](hardware/performance-tuning.md) - Performance optimization strategies
- [Power Management](hardware/power-management.md) - Power optimization and battery management

### Integration and Compatibility
- [Receiver Integration](integration/receiver-integration.md) - Integration with receiver systems
- [Message Format Compatibility](integration/message-formats.md) - Message format specifications and compatibility
- [System Architecture](integration/system-architecture.md) - Overall system architecture and component interaction

### Troubleshooting and Maintenance
- [Troubleshooting Guide](troubleshooting/common-issues.md) - Common issues and solutions
- [Maintenance Procedures](troubleshooting/maintenance.md) - Regular maintenance and monitoring
- [Diagnostic Tools](troubleshooting/diagnostics.md) - Diagnostic and debugging tools

### User Manuals
- [CLI Tools Manual](manuals/cli-tools.md) - Complete CLI tools reference
- [Deployment Utilities Manual](manuals/deployment-utilities.md) - Deployment and management utilities
- [Configuration Templates](manuals/configuration-templates.md) - Configuration template reference

## Getting Started

For new users, start with the [Quick Start Guide](deployment/quick-start.md) to get your first beacon deployed quickly.

For comprehensive deployment planning, refer to the [Deployment Guide](deployment/deployment-guide.md).

For troubleshooting issues, check the [Troubleshooting Guide](troubleshooting/common-issues.md).

## Requirements Coverage

This documentation covers all requirements from the beacon system specification:

- **Requirement 1**: Message transmission and format compatibility
- **Requirement 2**: GPS positioning and accuracy management
- **Requirement 3**: Power monitoring and management
- **Requirement 4**: Long-range communication and remote monitoring
- **Requirement 5**: Code reuse and shared library architecture
- **Requirement 6**: Environmental reliability and adaptation
- **Requirement 7**: Remote configuration management
- **Requirement 8**: Hardware constraints and optimization for ESP01-class devices

## Contributing to Documentation

When updating documentation:

1. Keep examples practical and tested
2. Include error handling and troubleshooting information
3. Update cross-references when adding new sections
4. Validate all code examples and configuration snippets
5. Consider different deployment scenarios and use cases

## Support

For additional support:

1. Check the troubleshooting guides first
2. Review the API documentation for detailed interface information
3. Consult the hardware optimization guides for performance issues
4. Refer to the integration documentation for system compatibility questions