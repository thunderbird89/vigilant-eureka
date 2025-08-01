#!/usr/bin/env pwsh
# Beacon configuration generation script
# Generates beacon configurations for deployment

param(
    [Parameter(Mandatory=$false)]
    [int]$Count = 1,
    
    [Parameter(Mandatory=$false)]
    [string]$OutputDir = "deployment",
    
    [Parameter(Mandatory=$false)]
    [string]$SiteConfig,
    
    [Parameter(Mandatory=$false)]
    [string]$Template,
    
    [Parameter(Mandatory=$false)]
    [switch]$Verbose
)

$ErrorActionPreference = "Stop"

Write-Host "=== Beacon Configuration Generation ===" -ForegroundColor Cyan
Write-Host "Count: $Count"
Write-Host "Output Directory: $OutputDir"
if ($SiteConfig) { Write-Host "Site Config: $SiteConfig" }
if ($Template) { Write-Host "Template: $Template" }
Write-Host ""

# Validate inputs
if ($Count -lt 1 -or $Count -gt 100) {
    Write-Error "Count must be between 1 and 100"
    exit 1
}

if ($SiteConfig -and -not (Test-Path $SiteConfig)) {
    Write-Error "Site config file does not exist: $SiteConfig"
    exit 1
}

if ($Template -and -not (Test-Path $Template)) {
    Write-Error "Template file does not exist: $Template"
    exit 1
}

# Build the beacon application if needed
$BeaconExe = ".\target\release\beacon.exe"
if (-not (Test-Path $BeaconExe)) {
    Write-Host "Building beacon application..." -ForegroundColor Yellow
    try {
        Push-Location (Split-Path $PSScriptRoot -Parent)
        cargo build --release
        if ($LASTEXITCODE -ne 0) {
            throw "Build failed"
        }
        Write-Host "✓ Build successful" -ForegroundColor Green
    } catch {
        Write-Error "Failed to build beacon application: $_"
        exit 1
    } finally {
        Pop-Location
    }
}

# Create default site config if none provided
if (-not $SiteConfig) {
    Write-Host "Creating default site configuration..." -ForegroundColor Yellow
    
    $DefaultSite = @{
        site_id = "default_site"
        name = "Default Deployment Site"
        location = @{
            latitude = 40.7128
            longitude = -74.0060
            altitude = 0.0
        }
        deployment_type = "Surface"
        environmental_conditions = @{
            temperature_range_c = @(5.0, 25.0)
            wave_height_m = 1.0
            current_speed_ms = 0.5
            salinity_ppt = 35.0
        }
        expected_beacons = $Count
    }
    
    $SiteConfig = Join-Path $env:TEMP "default_site.json"
    $DefaultSite | ConvertTo-Json -Depth 10 | Out-File -FilePath $SiteConfig -Encoding UTF8
    Write-Host "✓ Default site config created: $SiteConfig" -ForegroundColor Green
}

# Create default template if none provided
if (-not $Template) {
    Write-Host "Creating default configuration template..." -ForegroundColor Yellow
    
    $DefaultTemplate = @{
        transmission_interval_ms = 5000
        message_version = "V3"
        gps_config = @{
            acquisition_timeout_s = 60
            update_interval_s = 30
            min_satellite_count = 4
            accuracy_threshold_m = 5.0
            cold_start_timeout_s = 300
        }
        power_config = @{
            low_battery_threshold_percent = 20.0
            critical_battery_threshold_percent = 10.0
            emergency_battery_threshold_percent = 5.0
            power_save_mode_threshold_percent = 30.0
            charging_enabled = $true
            solar_charging_enabled = $false
        }
        communication_config = @{
            connection_interval_hours = 24
            retry_attempts = 3
            retry_backoff_ms = 5000
            max_retry_interval_hours = 6
            connection_timeout_s = 60
            data_compression_enabled = $true
        }
        emergency_config = @{
            emergency_transmission_interval_ms = 5000
            emergency_power_threshold_percent = 5.0
            emergency_gps_timeout_s = 300
            emergency_communication_timeout_s = 1800
            auto_shutdown_enabled = $true
            emergency_message_count = 10
            shutdown_delay_s = 30
        }
    }
    
    $Template = Join-Path $env:TEMP "default_template.json"
    $DefaultTemplate | ConvertTo-Json -Depth 10 | Out-File -FilePath $Template -Encoding UTF8
    Write-Host "✓ Default template created: $Template" -ForegroundColor Green
}

# Generate configurations
Write-Host "Generating $Count beacon configurations..." -ForegroundColor Yellow

try {
    Push-Location (Split-Path $PSScriptRoot -Parent)
    
    $args = @(
        "deploy", "generate-configs",
        "--count", $Count,
        "--output", $OutputDir,
        "--site-config", $SiteConfig,
        "--template", $Template
    )
    
    if ($Verbose) {
        $args += "--verbose"
    }
    
    & $BeaconExe @args
    
    if ($LASTEXITCODE -ne 0) {
        throw "Configuration generation failed"
    }
    
    Write-Host "✓ Configuration generation completed" -ForegroundColor Green
    
} catch {
    Write-Error "Configuration generation failed: $_"
    exit 1
} finally {
    Pop-Location
}

# Display summary
Write-Host ""
Write-Host "=== Generation Summary ===" -ForegroundColor Cyan

if (Test-Path $OutputDir) {
    $ConfigFiles = Get-ChildItem -Path $OutputDir -Filter "beacon_*.toml"
    Write-Host "Generated Files:"
    foreach ($file in $ConfigFiles) {
        Write-Host "  - $($file.Name)" -ForegroundColor Green
    }
    
    $ManifestFile = Join-Path $OutputDir "deployment_manifest.json"
    if (Test-Path $ManifestFile) {
        Write-Host "  - deployment_manifest.json" -ForegroundColor Green
    }
    
    Write-Host ""
    Write-Host "Next Steps:"
    Write-Host "1. Review generated configurations in: $OutputDir"
    Write-Host "2. Validate deployment: .\scripts\validate_deployment.ps1 -DeploymentPath $OutputDir"
    Write-Host "3. Create fleet and add beacons for management"
} else {
    Write-Warning "Output directory not found: $OutputDir"
}

Write-Host ""
Write-Host "=== Configuration Generation Complete ===" -ForegroundColor Cyan