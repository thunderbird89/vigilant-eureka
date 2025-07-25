#!/usr/bin/env pwsh
# Beacon status monitoring utility
# Provides real-time monitoring of beacon fleet status

param(
    [Parameter(Mandatory=$true)]
    [string]$FleetConfig,
    
    [Parameter(Mandatory=$false)]
    [int]$Interval = 30,
    
    [Parameter(Mandatory=$false)]
    [int]$Duration,
    
    [Parameter(Mandatory=$false)]
    [string]$OutputFile,
    
    [Parameter(Mandatory=$false)]
    [switch]$Continuous,
    
    [Parameter(Mandatory=$false)]
    [switch]$Verbose
)

$ErrorActionPreference = "Stop"

Write-Host "=== Beacon Fleet Status Monitor ===" -ForegroundColor Cyan
Write-Host "Fleet Config: $FleetConfig"
Write-Host "Monitoring Interval: $Interval seconds"
if ($Duration) { Write-Host "Duration: $Duration seconds" }
if ($OutputFile) { Write-Host "Output File: $OutputFile" }
Write-Host ""

# Validate inputs
if (-not (Test-Path $FleetConfig)) {
    Write-Error "Fleet configuration file does not exist: $FleetConfig"
    exit 1
}

if ($Interval -lt 5) {
    Write-Error "Monitoring interval must be at least 5 seconds"
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

# Initialize monitoring
$StartTime = Get-Date
$MonitoringData = @()
$IterationCount = 0

# Calculate end time if duration is specified
$EndTime = if ($Duration) { $StartTime.AddSeconds($Duration) } else { $null }

Write-Host "Starting fleet monitoring..." -ForegroundColor Yellow
if ($Continuous) {
    Write-Host "Press Ctrl+C to stop monitoring" -ForegroundColor Yellow
}
Write-Host ""

try {
    Push-Location (Split-Path $PSScriptRoot -Parent)
    
    do {
        $IterationCount++
        $CurrentTime = Get-Date
        
        Write-Host "=== Monitoring Iteration $IterationCount ($($CurrentTime.ToString('HH:mm:ss'))) ===" -ForegroundColor Cyan
        
        try {
            # Run fleet monitoring command
            $args = @(
                "deploy", "monitor-fleet",
                "--fleet-config", $FleetConfig
            )
            
            if ($Verbose) {
                $args += "--verbose"
            }
            
            $MonitorOutput = & $BeaconExe @args 2>&1
            
            if ($LASTEXITCODE -eq 0) {
                Write-Host $MonitorOutput
                
                # Store monitoring data if output file is specified
                if ($OutputFile) {
                    $MonitoringRecord = @{
                        timestamp = $CurrentTime
                        iteration = $IterationCount
                        status = "success"
                        output = $MonitorOutput
                    }
                    $MonitoringData += $MonitoringRecord
                }
                
                Write-Host "✓ Monitoring iteration completed successfully" -ForegroundColor Green
            } else {
                Write-Warning "Monitoring command failed with exit code $LASTEXITCODE"
                Write-Host $MonitorOutput -ForegroundColor Yellow
                
                if ($OutputFile) {
                    $MonitoringRecord = @{
                        timestamp = $CurrentTime
                        iteration = $IterationCount
                        status = "failed"
                        error = $MonitorOutput
                        exit_code = $LASTEXITCODE
                    }
                    $MonitoringData += $MonitoringRecord
                }
            }
            
        } catch {
            Write-Error "Error during monitoring iteration: $_"
            
            if ($OutputFile) {
                $MonitoringRecord = @{
                    timestamp = $CurrentTime
                    iteration = $IterationCount
                    status = "error"
                    error = $_.Exception.Message
                }
                $MonitoringData += $MonitoringRecord
            }
        }
        
        # Check if we should continue
        $ShouldContinue = $Continuous -or ($EndTime -and $CurrentTime -lt $EndTime)
        
        if ($ShouldContinue) {
            Write-Host ""
            Write-Host "Waiting $Interval seconds until next check..." -ForegroundColor Gray
            Start-Sleep -Seconds $Interval
        }
        
    } while ($ShouldContinue)
    
} catch {
    Write-Error "Monitoring failed: $_"
    exit 1
} finally {
    Pop-Location
}

# Save monitoring data if output file is specified
if ($OutputFile -and $MonitoringData.Count -gt 0) {
    Write-Host ""
    Write-Host "Saving monitoring data..." -ForegroundColor Yellow
    
    try {
        $MonitoringSummary = @{
            fleet_config = $FleetConfig
            monitoring_start = $StartTime
            monitoring_end = Get-Date
            total_iterations = $IterationCount
            interval_seconds = $Interval
            data = $MonitoringData
        }
        
        $MonitoringSummary | ConvertTo-Json -Depth 10 | Out-File -FilePath $OutputFile -Encoding UTF8
        Write-Host "✓ Monitoring data saved to: $OutputFile" -ForegroundColor Green
    } catch {
        Write-Warning "Failed to save monitoring data: $_"
    }
}

# Display summary
Write-Host ""
Write-Host "=== Monitoring Summary ===" -ForegroundColor Cyan
Write-Host "Start Time: $($StartTime.ToString('yyyy-MM-dd HH:mm:ss'))"
Write-Host "End Time: $((Get-Date).ToString('yyyy-MM-dd HH:mm:ss'))"
Write-Host "Total Duration: $((New-TimeSpan -Start $StartTime -End (Get-Date)).ToString('hh\:mm\:ss'))"
Write-Host "Total Iterations: $IterationCount"
Write-Host "Monitoring Interval: $Interval seconds"

if ($MonitoringData.Count -gt 0) {
    $SuccessfulIterations = ($MonitoringData | Where-Object { $_.status -eq "success" }).Count
    $FailedIterations = ($MonitoringData | Where-Object { $_.status -ne "success" }).Count
    
    Write-Host "Successful Iterations: $SuccessfulIterations"
    Write-Host "Failed Iterations: $FailedIterations"
    Write-Host "Success Rate: $([math]::Round(($SuccessfulIterations / $MonitoringData.Count) * 100, 1))%"
}

Write-Host ""
Write-Host "=== Fleet Monitoring Complete ===" -ForegroundColor Cyan