#!/usr/bin/env pwsh
# Deployment status utility
# Provides comprehensive status reporting for beacon deployments

param(
    [Parameter(Mandatory=$true)]
    [ValidateSet("fleet", "site", "beacon", "summary")]
    [string]$Type,
    
    [Parameter(Mandatory=$false)]
    [string]$FleetId,
    
    [Parameter(Mandatory=$false)]
    [string]$SiteId,
    
    [Parameter(Mandatory=$false)]
    [string]$BeaconId,
    
    [Parameter(Mandatory=$false)]
    [string]$DataDir = "fleet_data",
    
    [Parameter(Mandatory=$false)]
    [string]$OutputFile,
    
    [Parameter(Mandatory=$false)]
    [ValidateSet("json", "csv", "html")]
    [string]$Format = "json",
    
    [Parameter(Mandatory=$false)]
    [switch]$Detailed,
    
    [Parameter(Mandatory=$false)]
    [switch]$Verbose
)

$ErrorActionPreference = "Stop"

Write-Host "=== Deployment Status Report ===" -ForegroundColor Cyan
Write-Host "Report Type: $Type"
if ($FleetId) { Write-Host "Fleet ID: $FleetId" }
if ($SiteId) { Write-Host "Site ID: $SiteId" }
if ($BeaconId) { Write-Host "Beacon ID: $BeaconId" }
Write-Host "Data Directory: $DataDir"
Write-Host "Output Format: $Format"
Write-Host ""

# Validate inputs based on report type
switch ($Type) {
    "fleet" {
        if (-not $FleetId) {
            Write-Error "Fleet ID is required for fleet status report"
            exit 1
        }
    }
    "site" {
        if (-not $FleetId -or -not $SiteId) {
            Write-Error "Fleet ID and Site ID are required for site status report"
            exit 1
        }
    }
    "beacon" {
        if (-not $FleetId -or -not $BeaconId) {
            Write-Error "Fleet ID and Beacon ID are required for beacon status report"
            exit 1
        }
    }
    "summary" {
        # Summary doesn't require specific IDs
    }
}

# Check if data directory exists
if (-not (Test-Path $DataDir)) {
    Write-Error "Data directory does not exist: $DataDir"
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

# Generate status report
Write-Host "Generating $Type status report..." -ForegroundColor Yellow

try {
    Push-Location (Split-Path $PSScriptRoot -Parent)
    
    switch ($Type) {
        "fleet" {
            $args = @(
                "deploy", "fleet", "summary",
                "--fleet-id", $FleetId,
                "--data-dir", $DataDir
            )
            
            $StatusOutput = & $BeaconExe @args 2>&1
            
            if ($LASTEXITCODE -eq 0) {
                Write-Host $StatusOutput
                
                # Also generate detailed report
                if ($Detailed) {
                    Write-Host ""
                    Write-Host "Generating detailed fleet report..." -ForegroundColor Yellow
                    
                    $ReportArgs = @(
                        "deploy", "fleet", "report",
                        "--fleet-id", $FleetId,
                        "--data-dir", $DataDir
                    )
                    
                    if ($OutputFile) {
                        $ReportArgs += @("--output", $OutputFile)
                    }
                    
                    $ReportOutput = & $BeaconExe @ReportArgs 2>&1
                    
                    if ($LASTEXITCODE -eq 0) {
                        Write-Host $ReportOutput
                        Write-Host "✓ Detailed fleet report generated" -ForegroundColor Green
                    } else {
                        Write-Warning "Failed to generate detailed report"
                        Write-Host $ReportOutput -ForegroundColor Yellow
                    }
                }
            } else {
                Write-Error "Fleet status command failed"
                Write-Host $StatusOutput -ForegroundColor Red
                exit 1
            }
        }
        
        "site" {
            # Load fleet data and extract site information
            $FleetFile = Join-Path $DataDir "$FleetId.json"
            if (-not (Test-Path $FleetFile)) {
                Write-Error "Fleet file not found: $FleetFile"
                exit 1
            }
            
            $FleetData = Get-Content -Path $FleetFile -Raw | ConvertFrom-Json
            
            if (-not $FleetData.sites.$SiteId) {
                Write-Error "Site '$SiteId' not found in fleet '$FleetId'"
                exit 1
            }
            
            $Site = $FleetData.sites.$SiteId
            $SiteBeacons = $FleetData.beacons.PSObject.Properties | Where-Object { $_.Value.site_id -eq $SiteId }
            
            Write-Host "=== SITE STATUS REPORT ===" -ForegroundColor Cyan
            Write-Host "Site ID: $($Site.site_id)"
            Write-Host "Name: $($Site.name)"
            Write-Host "Location: $($Site.location.latitude), $($Site.location.longitude)"
            Write-Host "Deployment Type: $($Site.deployment_type)"
            Write-Host "Expected Beacons: $($Site.expected_beacons)"
            Write-Host "Deployed Beacons: $($SiteBeacons.Count)"
            
            if ($SiteBeacons.Count -gt 0) {
                Write-Host ""
                Write-Host "Deployed Beacons:"
                foreach ($Beacon in $SiteBeacons) {
                    Write-Host "  - $($Beacon.Name): Deployed $($Beacon.Value.deployment_time)"
                }
            }
            
            # Save site report if output file specified
            if ($OutputFile) {
                $SiteReport = @{
                    site = $Site
                    deployed_beacons = $SiteBeacons.Count
                    beacon_details = $SiteBeacons
                    generated_at = Get-Date
                }
                
                switch ($Format) {
                    "json" {
                        $SiteReport | ConvertTo-Json -Depth 10 | Out-File -FilePath $OutputFile -Encoding UTF8
                    }
                    "csv" {
                        # Convert to CSV format
                        $CsvData = @()
                        foreach ($Beacon in $SiteBeacons) {
                            $CsvData += [PSCustomObject]@{
                                SiteId = $Site.site_id
                                SiteName = $Site.name
                                BeaconId = $Beacon.Name
                                DeploymentTime = $Beacon.Value.deployment_time
                                BatteryLifeDays = $Beacon.Value.expected_battery_life_days
                            }
                        }
                        $CsvData | Export-Csv -Path $OutputFile -NoTypeInformation
                    }
                }
                
                Write-Host "✓ Site report saved to: $OutputFile" -ForegroundColor Green
            }
        }
        
        "beacon" {
            # Load fleet data and extract beacon information
            $FleetFile = Join-Path $DataDir "$FleetId.json"
            if (-not (Test-Path $FleetFile)) {
                Write-Error "Fleet file not found: $FleetFile"
                exit 1
            }
            
            $FleetData = Get-Content -Path $FleetFile -Raw | ConvertFrom-Json
            
            if (-not $FleetData.beacons.$BeaconId) {
                Write-Error "Beacon '$BeaconId' not found in fleet '$FleetId'"
                exit 1
            }
            
            $Beacon = $FleetData.beacons.$BeaconId
            $Site = $FleetData.sites.($Beacon.site_id)
            
            Write-Host "=== BEACON STATUS REPORT ===" -ForegroundColor Cyan
            Write-Host "Beacon ID: $BeaconId"
            Write-Host "Site: $($Site.name) ($($Beacon.site_id))"
            Write-Host "Deployment Time: $($Beacon.deployment_time)"
            Write-Host "Expected Battery Life: $($Beacon.expected_battery_life_days) days"
            Write-Host "Maintenance Schedule: $($Beacon.maintenance_schedule_days) days"
            
            if ($Beacon.deployment_position) {
                Write-Host "Deployment Position: $($Beacon.deployment_position.latitude), $($Beacon.deployment_position.longitude)"
            } else {
                Write-Host "Deployment Position: Not set"
            }
            
            if ($Detailed) {
                Write-Host ""
                Write-Host "Configuration Details:"
                Write-Host "  Transmission Interval: $($Beacon.config.transmission_interval_ms)ms"
                Write-Host "  Message Version: $($Beacon.config.message_version)"
                Write-Host "  GPS Update Interval: $($Beacon.config.gps_config.update_interval_s)s"
                Write-Host "  Low Battery Threshold: $($Beacon.config.power_config.low_battery_threshold_percent)%"
            }
            
            # Save beacon report if output file specified
            if ($OutputFile) {
                $BeaconReport = @{
                    beacon = $Beacon
                    site = $Site
                    generated_at = Get-Date
                }
                
                $BeaconReport | ConvertTo-Json -Depth 10 | Out-File -FilePath $OutputFile -Encoding UTF8
                Write-Host "✓ Beacon report saved to: $OutputFile" -ForegroundColor Green
            }
        }
        
        "summary" {
            # Generate summary of all fleets
            $FleetFiles = Get-ChildItem -Path $DataDir -Filter "*.json"
            
            if ($FleetFiles.Count -eq 0) {
                Write-Host "No fleet data found in: $DataDir" -ForegroundColor Yellow
                exit 0
            }
            
            Write-Host "=== DEPLOYMENT SUMMARY ===" -ForegroundColor Cyan
            Write-Host "Data Directory: $DataDir"
            Write-Host "Total Fleets: $($FleetFiles.Count)"
            Write-Host ""
            
            $TotalBeacons = 0
            $TotalSites = 0
            
            foreach ($FleetFile in $FleetFiles) {
                try {
                    $FleetData = Get-Content -Path $FleetFile.FullName -Raw | ConvertFrom-Json
                    $BeaconCount = $FleetData.beacons.PSObject.Properties.Count
                    $SiteCount = $FleetData.sites.PSObject.Properties.Count
                    
                    Write-Host "Fleet: $($FleetData.name) ($($FleetData.fleet_id))"
                    Write-Host "  Beacons: $BeaconCount"
                    Write-Host "  Sites: $SiteCount"
                    Write-Host "  Created: $($FleetData.created_at)"
                    Write-Host "  Last Updated: $($FleetData.last_updated)"
                    Write-Host ""
                    
                    $TotalBeacons += $BeaconCount
                    $TotalSites += $SiteCount
                } catch {
                    Write-Warning "Failed to process fleet file: $($FleetFile.Name)"
                }
            }
            
            Write-Host "TOTALS:"
            Write-Host "  Total Beacons: $TotalBeacons"
            Write-Host "  Total Sites: $TotalSites"
            
            # Save summary report if output file specified
            if ($OutputFile) {
                $SummaryReport = @{
                    data_directory = $DataDir
                    total_fleets = $FleetFiles.Count
                    total_beacons = $TotalBeacons
                    total_sites = $TotalSites
                    generated_at = Get-Date
                }
                
                $SummaryReport | ConvertTo-Json -Depth 10 | Out-File -FilePath $OutputFile -Encoding UTF8
                Write-Host "✓ Summary report saved to: $OutputFile" -ForegroundColor Green
            }
        }
    }
    
    Write-Host "✓ Status report generated successfully" -ForegroundColor Green
    
} catch {
    Write-Error "Status report generation failed: $_"
    exit 1
} finally {
    Pop-Location
}

Write-Host ""
Write-Host "=== Status Report Complete ===" -ForegroundColor Cyan