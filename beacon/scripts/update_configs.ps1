#!/usr/bin/env pwsh
# Beacon configuration update utility
# Updates beacon configurations remotely using templates

param(
    [Parameter(Mandatory=$true)]
    [string]$FleetConfig,
    
    [Parameter(Mandatory=$true)]
    [string]$Template,
    
    [Parameter(Mandatory=$false)]
    [string]$Beacons,
    
    [Parameter(Mandatory=$false)]
    [string]$OutputReport,
    
    [Parameter(Mandatory=$false)]
    [switch]$DryRun,
    
    [Parameter(Mandatory=$false)]
    [switch]$Verbose
)

$ErrorActionPreference = "Stop"

Write-Host "=== Beacon Configuration Update ===" -ForegroundColor Cyan
Write-Host "Fleet Config: $FleetConfig"
Write-Host "Template: $Template"
if ($Beacons) { Write-Host "Target Beacons: $Beacons" }
if ($DryRun) { Write-Host "Mode: DRY RUN (no actual updates)" -ForegroundColor Yellow }
Write-Host ""

# Validate inputs
if (-not (Test-Path $FleetConfig)) {
    Write-Error "Fleet configuration file does not exist: $FleetConfig"
    exit 1
}

if (-not (Test-Path $Template)) {
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

# Validate template format
Write-Host "Validating template format..." -ForegroundColor Yellow
try {
    $TemplateContent = Get-Content -Path $Template -Raw | ConvertFrom-Json
    Write-Host "✓ Template format is valid" -ForegroundColor Green
} catch {
    Write-Error "Invalid template format: $_"
    exit 1
}

# Validate fleet configuration
Write-Host "Validating fleet configuration..." -ForegroundColor Yellow
try {
    $FleetContent = Get-Content -Path $FleetConfig -Raw | ConvertFrom-Json
    $BeaconCount = $FleetContent.beacons.PSObject.Properties.Count
    Write-Host "✓ Fleet configuration is valid ($BeaconCount beacons)" -ForegroundColor Green
} catch {
    Write-Error "Invalid fleet configuration: $_"
    exit 1
}

# Prepare beacon list
if ($Beacons) {
    Write-Host "Validating beacon IDs..." -ForegroundColor Yellow
    $BeaconList = $Beacons -split ',' | ForEach-Object { $_.Trim() }
    
    foreach ($BeaconId in $BeaconList) {
        try {
            [System.Guid]::Parse($BeaconId) | Out-Null
        } catch {
            Write-Error "Invalid beacon ID format: $BeaconId"
            exit 1
        }
    }
    
    Write-Host "✓ Beacon IDs are valid" -ForegroundColor Green
}

# Show configuration preview
Write-Host ""
Write-Host "=== Configuration Update Preview ===" -ForegroundColor Cyan
Write-Host "Template Configuration:"
Write-Host ($TemplateContent | ConvertTo-Json -Depth 5) -ForegroundColor Gray

if ($Beacons) {
    Write-Host ""
    Write-Host "Target Beacons:"
    foreach ($BeaconId in $BeaconList) {
        Write-Host "  - $BeaconId" -ForegroundColor Gray
    }
} else {
    Write-Host ""
    Write-Host "Target: All beacons in fleet ($BeaconCount beacons)" -ForegroundColor Gray
}

# Confirm update unless dry run
if (-not $DryRun) {
    Write-Host ""
    $Confirmation = Read-Host "Proceed with configuration update? (y/N)"
    if ($Confirmation -notmatch '^[Yy]') {
        Write-Host "Configuration update cancelled" -ForegroundColor Yellow
        exit 0
    }
}

# Execute configuration update
Write-Host ""
Write-Host "Executing configuration update..." -ForegroundColor Yellow

try {
    Push-Location (Split-Path $PSScriptRoot -Parent)
    
    $args = @(
        "deploy", "update-configs",
        "--fleet-config", $FleetConfig,
        "--template", $Template
    )
    
    if ($Beacons) {
        $args += @("--beacons", $Beacons)
    }
    
    if ($Verbose) {
        $args += "--verbose"
    }
    
    if ($DryRun) {
        Write-Host "DRY RUN: Would execute: $BeaconExe $($args -join ' ')" -ForegroundColor Yellow
        Write-Host "✓ Dry run completed successfully" -ForegroundColor Green
    } else {
        $UpdateOutput = & $BeaconExe @args 2>&1
        
        if ($LASTEXITCODE -eq 0) {
            Write-Host $UpdateOutput
            Write-Host "✓ Configuration update completed successfully" -ForegroundColor Green
            
            # Save output report if requested
            if ($OutputReport) {
                $ReportData = @{
                    timestamp = Get-Date
                    fleet_config = $FleetConfig
                    template = $Template
                    target_beacons = if ($Beacons) { $BeaconList } else { "all" }
                    status = "success"
                    output = $UpdateOutput
                }
                
                $ReportData | ConvertTo-Json -Depth 10 | Out-File -FilePath $OutputReport -Encoding UTF8
                Write-Host "✓ Update report saved to: $OutputReport" -ForegroundColor Green
            }
        } else {
            Write-Error "Configuration update failed with exit code $LASTEXITCODE"
            Write-Host $UpdateOutput -ForegroundColor Red
            
            if ($OutputReport) {
                $ReportData = @{
                    timestamp = Get-Date
                    fleet_config = $FleetConfig
                    template = $Template
                    target_beacons = if ($Beacons) { $BeaconList } else { "all" }
                    status = "failed"
                    error = $UpdateOutput
                    exit_code = $LASTEXITCODE
                }
                
                $ReportData | ConvertTo-Json -Depth 10 | Out-File -FilePath $OutputReport -Encoding UTF8
                Write-Host "Update report saved to: $OutputReport" -ForegroundColor Yellow
            }
            
            exit 1
        }
    }
    
} catch {
    Write-Error "Configuration update failed: $_"
    exit 1
} finally {
    Pop-Location
}

Write-Host ""
Write-Host "=== Configuration Update Complete ===" -ForegroundColor Cyan