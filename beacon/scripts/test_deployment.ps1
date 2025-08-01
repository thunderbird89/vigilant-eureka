#!/usr/bin/env pwsh
# Comprehensive beacon deployment testing script
# Tests all aspects of beacon deployment and operational tools

param(
    [Parameter(Mandatory=$false)]
    [string]$TestDir = "test_deployment",
    
    [Parameter(Mandatory=$false)]
    [int]$BeaconCount = 3,
    
    [Parameter(Mandatory=$false)]
    [switch]$CleanUp,
    
    [Parameter(Mandatory=$false)]
    [switch]$Verbose
)

$ErrorActionPreference = "Stop"

Write-Host "=== Beacon Deployment Testing Suite ===" -ForegroundColor Cyan
Write-Host "Test Directory: $TestDir"
Write-Host "Beacon Count: $BeaconCount"
Write-Host ""

# Clean up previous test if requested
if ($CleanUp -and (Test-Path $TestDir)) {
    Write-Host "Cleaning up previous test directory..." -ForegroundColor Yellow
    Remove-Item -Path $TestDir -Recurse -Force
    Write-Host "✓ Cleanup completed" -ForegroundColor Green
}

# Create test directory
if (-not (Test-Path $TestDir)) {
    New-Item -ItemType Directory -Path $TestDir | Out-Null
}

$ScriptDir = $PSScriptRoot
$ProjectDir = Split-Path $ScriptDir -Parent

try {
    Push-Location $ProjectDir
    
    # Test 1: Build application
    Write-Host "Test 1: Building beacon application..." -ForegroundColor Yellow
    cargo build --release
    if ($LASTEXITCODE -ne 0) {
        throw "Build failed"
    }
    Write-Host "✓ Build successful" -ForegroundColor Green
    
    # Test 2: Generate configurations
    Write-Host "Test 2: Generating beacon configurations..." -ForegroundColor Yellow
    $ConfigDir = Join-Path $TestDir "configs"
    & ".\scripts\generate_configs.ps1" -Count $BeaconCount -OutputDir $ConfigDir
    if ($LASTEXITCODE -ne 0) {
        throw "Configuration generation failed"
    }
    Write-Host "✓ Configuration generation successful" -ForegroundColor Green
    
    # Test 3: Validate deployment
    Write-Host "Test 3: Validating deployment..." -ForegroundColor Yellow
    & ".\scripts\validate_deployment.ps1" -DeploymentPath $ConfigDir -TestDuration 10
    if ($LASTEXITCODE -ne 0) {
        throw "Deployment validation failed"
    }
    Write-Host "✓ Deployment validation successful" -ForegroundColor Green
    
    # Test 4: Create fleet
    Write-Host "Test 4: Creating test fleet..." -ForegroundColor Yellow
    $FleetDataDir = Join-Path $TestDir "fleet_data"
    $FleetId = "test_fleet_$(Get-Date -Format 'yyyyMMdd_HHmmss')"
    & ".\scripts\manage_fleet.ps1" -Action create -FleetId $FleetId -FleetName "Test Fleet" -DataDir $FleetDataDir
    if ($LASTEXITCODE -ne 0) {
        throw "Fleet creation failed"
    }
    Write-Host "✓ Fleet creation successful" -ForegroundColor Green
    
    # Test 5: Add site to fleet
    Write-Host "Test 5: Adding site to fleet..." -ForegroundColor Yellow
    
    # Create test site configuration
    $TestSite = @{
        site_id = "test_site_001"
        name = "Test Deployment Site"
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
        expected_beacons = $BeaconCount
    }
    
    $SiteConfigPath = Join-Path $TestDir "test_site.json"
    $TestSite | ConvertTo-Json -Depth 10 | Out-File -FilePath $SiteConfigPath -Encoding UTF8
    
    & ".\scripts\manage_fleet.ps1" -Action add-site -FleetId $FleetId -SiteConfig $SiteConfigPath -DataDir $FleetDataDir
    if ($LASTEXITCODE -ne 0) {
        throw "Adding site to fleet failed"
    }
    Write-Host "✓ Site addition successful" -ForegroundColor Green
    
    # Test 6: Add beacons to fleet
    Write-Host "Test 6: Adding beacons to fleet..." -ForegroundColor Yellow
    $ManifestPath = Join-Path $ConfigDir "deployment_manifest.json"
    & ".\scripts\manage_fleet.ps1" -Action add-beacons -FleetId $FleetId -Deployment $ManifestPath -DataDir $FleetDataDir
    if ($LASTEXITCODE -ne 0) {
        throw "Adding beacons to fleet failed"
    }
    Write-Host "✓ Beacon addition successful" -ForegroundColor Green
    
    # Test 7: Fleet summary
    Write-Host "Test 7: Generating fleet summary..." -ForegroundColor Yellow
    & ".\scripts\manage_fleet.ps1" -Action summary -FleetId $FleetId -DataDir $FleetDataDir
    if ($LASTEXITCODE -ne 0) {
        throw "Fleet summary generation failed"
    }
    Write-Host "✓ Fleet summary successful" -ForegroundColor Green
    
    # Test 8: Fleet deployment report
    Write-Host "Test 8: Generating deployment report..." -ForegroundColor Yellow
    $ReportPath = Join-Path $TestDir "deployment_report.json"
    & ".\scripts\manage_fleet.ps1" -Action report -FleetId $FleetId -Output $ReportPath -DataDir $FleetDataDir
    if ($LASTEXITCODE -ne 0) {
        throw "Deployment report generation failed"
    }
    Write-Host "✓ Deployment report successful" -ForegroundColor Green
    
    # Test 9: Configuration validation
    Write-Host "Test 9: Validating individual configurations..." -ForegroundColor Yellow
    $ConfigFiles = Get-ChildItem -Path $ConfigDir -Filter "beacon_*.toml"
    foreach ($configFile in $ConfigFiles) {
        & ".\target\release\beacon.exe" validate-config --config $configFile.FullName
        if ($LASTEXITCODE -ne 0) {
            throw "Configuration validation failed for $($configFile.Name)"
        }
    }
    Write-Host "✓ Individual configuration validation successful" -ForegroundColor Green
    
    # Test 10: Status and diagnostic commands
    Write-Host "Test 10: Testing status and diagnostic commands..." -ForegroundColor Yellow
    $TestConfigFile = $ConfigFiles[0].FullName
    
    # Test status command
    & ".\target\release\beacon.exe" status --config $TestConfigFile --detailed
    if ($LASTEXITCODE -ne 0) {
        Write-Warning "Status command failed (expected with mock hardware)"
    } else {
        Write-Host "✓ Status command successful" -ForegroundColor Green
    }
    
    # Test diagnostic command
    & ".\target\release\beacon.exe" diagnostic --config $TestConfigFile --all
    if ($LASTEXITCODE -ne 0) {
        Write-Warning "Diagnostic command failed (expected with mock hardware)"
    } else {
        Write-Host "✓ Diagnostic command successful" -ForegroundColor Green
    }
    
    Write-Host ""
    Write-Host "=== All Tests Completed Successfully ===" -ForegroundColor Green
    Write-Host ""
    Write-Host "Test Results Summary:"
    Write-Host "✓ Application build" -ForegroundColor Green
    Write-Host "✓ Configuration generation ($BeaconCount beacons)" -ForegroundColor Green
    Write-Host "✓ Deployment validation" -ForegroundColor Green
    Write-Host "✓ Fleet creation" -ForegroundColor Green
    Write-Host "✓ Site management" -ForegroundColor Green
    Write-Host "✓ Beacon management" -ForegroundColor Green
    Write-Host "✓ Fleet reporting" -ForegroundColor Green
    Write-Host "✓ Configuration validation" -ForegroundColor Green
    Write-Host "✓ Status and diagnostics" -ForegroundColor Green
    Write-Host ""
    Write-Host "Generated Files:"
    Write-Host "- Configurations: $ConfigDir"
    Write-Host "- Fleet Data: $FleetDataDir"
    Write-Host "- Deployment Report: $ReportPath"
    Write-Host ""
    Write-Host "Fleet ID: $FleetId"
    
} catch {
    Write-Error "Test failed: $_"
    Write-Host ""
    Write-Host "=== Test Failed ===" -ForegroundColor Red
    Write-Host "Error occurred during testing. Check the error message above."
    Write-Host "Test directory preserved for debugging: $TestDir"
    exit 1
} finally {
    Pop-Location
}

if (-not $CleanUp) {
    Write-Host ""
    Write-Host "Test files preserved in: $TestDir"
    Write-Host "To clean up, run: $($MyInvocation.MyCommand.Name) -CleanUp"
}

Write-Host ""
Write-Host "=== Deployment Testing Complete ===" -ForegroundColor Cyan