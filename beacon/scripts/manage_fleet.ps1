#!/usr/bin/env pwsh
# Beacon fleet management script
# Provides utilities for managing beacon fleets

param(
    [Parameter(Mandatory=$true)]
    [ValidateSet("create", "add-site", "add-beacons", "summary", "report", "monitor")]
    [string]$Action,
    
    [Parameter(Mandatory=$false)]
    [string]$FleetId,
    
    [Parameter(Mandatory=$false)]
    [string]$FleetName,
    
    [Parameter(Mandatory=$false)]
    [string]$DataDir = "fleet_data",
    
    [Parameter(Mandatory=$false)]
    [string]$SiteConfig,
    
    [Parameter(Mandatory=$false)]
    [string]$Deployment,
    
    [Parameter(Mandatory=$false)]
    [string]$Output,
    
    [Parameter(Mandatory=$false)]
    [int]$MonitorInterval,
    
    [Parameter(Mandatory=$false)]
    [switch]$Verbose
)

$ErrorActionPreference = "Stop"

Write-Host "=== Beacon Fleet Management ===" -ForegroundColor Cyan
Write-Host "Action: $Action"
if ($FleetId) { Write-Host "Fleet ID: $FleetId" }
if ($FleetName) { Write-Host "Fleet Name: $FleetName" }
Write-Host "Data Directory: $DataDir"
Write-Host ""

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

# Execute fleet management command
try {
    Push-Location (Split-Path $PSScriptRoot -Parent)
    
    switch ($Action) {
        "create" {
            if (-not $FleetId -or -not $FleetName) {
                Write-Error "Fleet ID and Fleet Name are required for create action"
                exit 1
            }
            
            $args = @(
                "deploy", "fleet", "create",
                "--id", $FleetId,
                "--name", $FleetName,
                "--data-dir", $DataDir
            )
            
            & $BeaconExe @args
        }
        
        "add-site" {
            if (-not $FleetId -or -not $SiteConfig) {
                Write-Error "Fleet ID and Site Config are required for add-site action"
                exit 1
            }
            
            if (-not (Test-Path $SiteConfig)) {
                Write-Error "Site config file does not exist: $SiteConfig"
                exit 1
            }
            
            $args = @(
                "deploy", "fleet", "add-site",
                "--fleet-id", $FleetId,
                "--site-config", $SiteConfig,
                "--data-dir", $DataDir
            )
            
            & $BeaconExe @args
        }
        
        "add-beacons" {
            if (-not $FleetId -or -not $Deployment) {
                Write-Error "Fleet ID and Deployment manifest are required for add-beacons action"
                exit 1
            }
            
            if (-not (Test-Path $Deployment)) {
                Write-Error "Deployment manifest does not exist: $Deployment"
                exit 1
            }
            
            $args = @(
                "deploy", "fleet", "add-beacons",
                "--fleet-id", $FleetId,
                "--deployment", $Deployment,
                "--data-dir", $DataDir
            )
            
            & $BeaconExe @args
        }
        
        "summary" {
            if (-not $FleetId) {
                Write-Error "Fleet ID is required for summary action"
                exit 1
            }
            
            $args = @(
                "deploy", "fleet", "summary",
                "--fleet-id", $FleetId,
                "--data-dir", $DataDir
            )
            
            & $BeaconExe @args
        }
        
        "report" {
            if (-not $FleetId) {
                Write-Error "Fleet ID is required for report action"
                exit 1
            }
            
            $args = @(
                "deploy", "fleet", "report",
                "--fleet-id", $FleetId,
                "--data-dir", $DataDir
            )
            
            if ($Output) {
                $args += @("--output", $Output)
            }
            
            & $BeaconExe @args
        }
        
        "monitor" {
            if (-not $FleetId) {
                Write-Error "Fleet ID is required for monitor action"
                exit 1
            }
            
            # Create a temporary fleet config for monitoring
            $FleetConfigPath = Join-Path $DataDir "$FleetId.json"
            if (-not (Test-Path $FleetConfigPath)) {
                Write-Error "Fleet configuration not found: $FleetConfigPath"
                exit 1
            }
            
            $args = @(
                "deploy", "monitor-fleet",
                "--fleet-config", $FleetConfigPath
            )
            
            if ($MonitorInterval) {
                $args += @("--interval", $MonitorInterval)
            }
            
            & $BeaconExe @args
        }
        
        default {
            Write-Error "Unknown action: $Action"
            exit 1
        }
    }
    
    if ($LASTEXITCODE -ne 0) {
        throw "Command failed with exit code $LASTEXITCODE"
    }
    
    Write-Host "✓ Fleet management operation completed successfully" -ForegroundColor Green
    
} catch {
    Write-Error "Fleet management operation failed: $_"
    exit 1
} finally {
    Pop-Location
}

Write-Host ""
Write-Host "=== Fleet Management Complete ===" -ForegroundColor Cyan