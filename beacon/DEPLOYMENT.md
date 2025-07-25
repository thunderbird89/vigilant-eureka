# Beacon Deployment and Operational Tools

This document describes the deployment and operational tools for the underwater positioning beacon system. These tools provide comprehensive support for beacon configuration generation, deployment validation, fleet management, and remote operations.

## Overview

The deployment tools consist of:

1. **Configuration Generation Tools** - Generate beacon configurations for deployment
2. **Status Monitoring Utilities** - Monitor beacon and fleet status
3. **Remote Configuration Update Tools** - Update beacon configurations remotely
4. **Deployment Validation Scripts** - Validate deployments before field deployment
5. **Fleet Management Utilities** - Manage beacon fleets and deployment sites

## Command Line Interface

### Basic Commands

```bash
# Generate default configuration
beacon generate-config --output beacon.toml

# Validate configuration
beacon validate-config --config beacon.toml

# Show beacon status
beacon status --config beacon.toml --detailed

# Run diagnostics
beacon diagnostic --config beacon.toml --all
```

### Deployment Commands

```bash
# Generate beacon configurations
beacon deploy generate-configs --count 5 --output deployment/

# Validate deployment
beacon deploy validate-deployment --deployment deployment/ --test-duration 30

# Monitor fleet status
beacon deploy monitor-fleet --fleet-config fleet.json --interval 60

# Update configurations
beacon deploy update-configs --fleet-config fleet.json --template template.json

# Fleet management
beacon deploy fleet create --id my_fleet --name "My Fleet"
beacon deploy fleet add-site --fleet-id my_fleet --site-config site.json
beacon deploy fleet summary --fleet-id my_fleet
```

## Configuration Generation

### Generate Single Configuration

```bash
beacon generate-config --output beacon.toml
```

### Generate Multiple Configurations

```bash
beacon deploy generate-configs \
  --count 10 \
  --output deployment/ \
  --site-config site.json \
  --template template.json
```

### Site Configuration Format

```json
{
  "site_id": "site_001",
  "name": "North Atlantic Site",
  "location": {
    "latitude": 40.7128,
    "longitude": -74.0060,
    "altitude": 0.0
  },
  "deployment_type": "Surface",
  "environmental_conditions": {
    "temperature_range_c": [5.0, 25.0],
    "wave_height_m": 2.0,
    "current_speed_ms": 0.8,
    "salinity_ppt": 35.0
  },
  "expected_beacons": 10
}
```

### Configuration Template Format

```json
{
  "transmission_interval_ms": 5000,
  "message_version": "V3",
  "gps_config": {
    "acquisition_timeout_s": 60,
    "update_interval_s": 30,
    "min_satellite_count": 4,
    "accuracy_threshold_m": 5.0,
    "cold_start_timeout_s": 300
  },
  "power_config": {
    "low_battery_threshold_percent": 20.0,
    "critical_battery_threshold_percent": 10.0,
    "emergency_battery_threshold_percent": 5.0,
    "power_save_mode_threshold_percent": 30.0,
    "charging_enabled": true,
    "solar_charging_enabled": false
  },
  "communication_config": {
    "connection_interval_hours": 24,
    "retry_attempts": 3,
    "retry_backoff_ms": 5000,
    "max_retry_interval_hours": 6,
    "connection_timeout_s": 60,
    "data_compression_enabled": true
  },
  "emergency_config": {
    "emergency_transmission_interval_ms": 5000,
    "emergency_power_threshold_percent": 5.0,
    "emergency_gps_timeout_s": 300,
    "emergency_communication_timeout_s": 1800,
    "auto_shutdown_enabled": true,
    "emergency_message_count": 10,
    "shutdown_delay_s": 30
  }
}
```

## Deployment Validation

### Validate Single Deployment

```bash
beacon deploy validate-deployment \
  --deployment deployment_manifest.json \
  --test-duration 60
```

### Validation Tests

The validation process includes:

1. **Configuration Validation** - Validates all configuration parameters
2. **GPS Functionality Test** - Tests GPS configuration and expected performance
3. **Power System Test** - Validates power management configuration
4. **Communication Test** - Tests communication configuration
5. **Transmission Test** - Validates transmission parameters
6. **Environmental Compatibility** - Checks environmental suitability

### Validation Report

```
=== DEPLOYMENT VALIDATION REPORT ===
Total Beacons: 5
Passed: 4
Failed: 1
Success Rate: 80.0%

--- Beacon 12345678-1234-5678-9abc-123456789abc ---
Overall Result: ✓ PASS
  ✓ Configuration Validation: Configuration is valid (0.01s)
  ✓ GPS Functionality: GPS configuration is suitable for deployment (0.50s)
  ✓ Power System: Power management configuration is valid (0.30s)
  ✓ Communication: Communication configuration is suitable (0.80s)
  ✓ Transmission: Transmission interval is within acceptable range (0.40s)
  ✓ Environmental Compatibility: Expected battery life: 28 days (0.20s)
  Recommendations:
    - All tests passed - deployment ready
```

## Fleet Management

### Create Fleet

```bash
beacon deploy fleet create \
  --id north_atlantic_fleet \
  --name "North Atlantic Deployment" \
  --data-dir fleet_data/
```

### Add Deployment Site

```bash
beacon deploy fleet add-site \
  --fleet-id north_atlantic_fleet \
  --site-config site_001.json \
  --data-dir fleet_data/
```

### Add Beacons to Fleet

```bash
beacon deploy fleet add-beacons \
  --fleet-id north_atlantic_fleet \
  --deployment deployment_manifest.json \
  --data-dir fleet_data/
```

### Fleet Summary

```bash
beacon deploy fleet summary \
  --fleet-id north_atlantic_fleet \
  --data-dir fleet_data/
```

### Generate Deployment Report

```bash
beacon deploy fleet report \
  --fleet-id north_atlantic_fleet \
  --output deployment_report.json \
  --data-dir fleet_data/
```

## Status Monitoring

### Single Status Check

```bash
beacon deploy monitor-fleet --fleet-config fleet.json
```

### Continuous Monitoring

```bash
beacon deploy monitor-fleet \
  --fleet-config fleet.json \
  --interval 300  # Check every 5 minutes
```

### Status Report Format

```
=== FLEET STATUS REPORT ===
Fleet: north_atlantic_fleet
Timestamp: 2024-01-15T10:30:00Z
Total Beacons: 10
Healthy: 8 (80.0%)
Warning: 1 (10.0%)
Critical: 1 (10.0%)

--- Beacon Details ---
Beacon 12345678-1234-5678-9abc-123456789abc: Healthy
  Battery: 85.2%
  GPS: Locked
  Position: 40.712800, -74.006000 (±2.1m)
  Last Contact: 2024-01-15T10:25:00Z

Beacon 87654321-4321-8765-cba9-987654321cba: Warning
  Battery: 18.5%
  GPS: Locked
  Position: 40.713000, -74.007000 (±3.2m)
  Last Contact: 2024-01-15T10:20:00Z
```

## Remote Configuration Updates

### Update Single Beacon

```bash
# This would be implemented in the actual beacon communication system
# The CLI provides the interface for configuration management
```

### Update Multiple Beacons

```bash
beacon deploy update-configs \
  --fleet-config fleet.json \
  --template new_template.json \
  --beacons "beacon1,beacon2,beacon3"
```

### Update All Beacons in Fleet

```bash
beacon deploy update-configs \
  --fleet-config fleet.json \
  --template new_template.json
```

## PowerShell Scripts

### Windows Deployment Scripts

The `scripts/` directory contains PowerShell scripts for Windows deployment:

#### Generate Configurations

```powershell
.\scripts\generate_configs.ps1 -Count 5 -OutputDir deployment
```

#### Validate Deployment

```powershell
.\scripts\validate_deployment.ps1 -DeploymentPath deployment -TestDuration 30
```

#### Manage Fleet

```powershell
# Create fleet
.\scripts\manage_fleet.ps1 -Action create -FleetId my_fleet -FleetName "My Fleet"

# Add site
.\scripts\manage_fleet.ps1 -Action add-site -FleetId my_fleet -SiteConfig site.json

# Monitor fleet
.\scripts\manage_fleet.ps1 -Action monitor -FleetId my_fleet -MonitorInterval 60
```

#### Comprehensive Testing

```powershell
.\scripts\test_deployment.ps1 -BeaconCount 3 -TestDir test_deployment
```

## Bash Scripts

### Unix Deployment Scripts

#### Validate Deployment

```bash
./scripts/validate_deployment.sh --deployment deployment/ --test-duration 30
```

## Best Practices

### Configuration Management

1. **Use Templates** - Create configuration templates for different deployment scenarios
2. **Validate Early** - Always validate configurations before deployment
3. **Version Control** - Keep configuration templates and site definitions in version control
4. **Document Changes** - Document any configuration changes and their rationale

### Deployment Process

1. **Site Survey** - Conduct site survey and create site configuration
2. **Generate Configurations** - Use site-specific templates to generate beacon configurations
3. **Validate Deployment** - Run comprehensive validation tests
4. **Create Fleet** - Set up fleet management for the deployment
5. **Deploy Beacons** - Deploy beacons with validated configurations
6. **Monitor Status** - Set up continuous monitoring for deployed beacons

### Fleet Management

1. **Organize by Site** - Group beacons by deployment site for easier management
2. **Regular Monitoring** - Set up automated monitoring with appropriate intervals
3. **Maintenance Scheduling** - Use fleet reports to plan maintenance activities
4. **Configuration Updates** - Use fleet management for coordinated configuration updates

### Troubleshooting

#### Common Issues

1. **Configuration Validation Failures**
   - Check parameter ranges and dependencies
   - Verify site-specific adjustments are appropriate
   - Review environmental compatibility

2. **Deployment Validation Failures**
   - Increase test duration for more thorough testing
   - Check system resources and network connectivity
   - Verify all required tools are installed

3. **Fleet Management Issues**
   - Ensure fleet data directory is accessible
   - Check JSON file formats for syntax errors
   - Verify beacon IDs are unique within the fleet

#### Debug Mode

Enable verbose output for detailed debugging:

```bash
beacon --verbose deploy generate-configs --count 5 --output deployment/
```

## Integration with Existing Systems

The deployment tools are designed to integrate with:

1. **Configuration Management Systems** - Export configurations in standard formats
2. **Monitoring Systems** - Provide status data in structured formats
3. **Asset Management** - Track beacon deployments and maintenance schedules
4. **Communication Systems** - Interface with remote configuration update mechanisms

## Security Considerations

1. **Configuration Security** - Protect configuration files containing sensitive information
2. **Fleet Data Protection** - Secure fleet management data and access controls
3. **Remote Updates** - Implement secure channels for remote configuration updates
4. **Access Control** - Restrict access to deployment and fleet management tools

## Performance Considerations

1. **Batch Operations** - Use batch operations for large fleet management tasks
2. **Monitoring Intervals** - Choose appropriate monitoring intervals to balance timeliness and resource usage
3. **Validation Duration** - Adjust validation test duration based on deployment requirements
4. **Data Storage** - Consider data retention policies for fleet management data

## Future Enhancements

1. **Web Interface** - Develop web-based fleet management interface
2. **Real-time Monitoring** - Implement real-time status monitoring with alerts
3. **Automated Deployment** - Add support for automated beacon deployment workflows
4. **Integration APIs** - Provide REST APIs for integration with external systems
5. **Mobile Support** - Develop mobile applications for field deployment support