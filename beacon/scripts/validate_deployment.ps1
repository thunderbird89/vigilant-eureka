#!/usr/bin/env pwsh
# Beacon deployment validation script for Windows
# Validates beacon configurations and deployment readiness

param(
    [Parameter(Mandatory=$true)]
    [string]$DeploymentPath,
    
    [Parameter(Mandatory=$false)]
    [int]$TestDuration = 30,
    
    [Parameter(Mandatory=$false)]
    [switch]$Verbose
)

# Set error action preference
$ErrorActionPreference = "Stop"

Write-Host "=== Beacon Deployment Validation ===" -ForegroundColor Cyan
Write-Host "Deployment Path: $DeploymentPath"
Write-Host "Test Duration: $TestDuration seconds"
Write-Host ""

# Check if deployment path exists
if (-not (Test-Path $DeploymentPath)) {
    Write-Error "Deployment path does not exist: $DeploymentPath"
    exit 1
}

# Build the beacon application
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

# Run deployment validation
Write-Host "Running deployment validation..." -ForegroundColor Yellow
try {
    Push-Location (Split-Path $PSScriptRoot -Parent)
    
    $args = @(
        "deploy",
        "validate-deployment",
        "--deployment", $DeploymentPath,
        "--test-duration", $TestDuration
    )
    
    if ($Verbose) {
        $args += "--verbose"
    }
    
    & ".\target\release\beacon.exe" @args
    
    if ($LASTEXITCODE -ne 0) {
        throw "Validation failed"
    }
    
    Write-Host "✓ Deployment validation completed" -ForegroundColor Green
} catch {
    Write-Error "Deployment validation failed: $_"
    exit 1
} finally {
    Pop-Location
}

# Additional system checks
Write-Host "Running system compatibility checks..." -ForegroundColor Yellow

# Check available disk space
$drive = (Get-Location).Drive
$freeSpace = (Get-WmiObject -Class Win32_LogicalDisk -Filter "DeviceID='$($drive.Name)'").FreeSpace
$freeSpaceGB = [math]::Round($freeSpace / 1GB, 2)

if ($freeSpaceGB -lt 1) {
    Write-Warning "Low disk space: ${freeSpaceGB}GB available"
} else {
    Write-Host "✓ Sufficient disk space: ${freeSpaceGB}GB available" -ForegroundColor Green
}

# Check memory usage
$memory = Get-WmiObject -Class Win32_OperatingSystem
$totalMemory = [math]::Round($memory.TotalVisibleMemorySize / 1MB, 2)
$freeMemory = [math]::Round($memory.FreePhysicalMemory / 1MB, 2)
$memoryUsage = [math]::Round((($totalMemory - $freeMemory) / $totalMemory) * 100, 1)

if ($memoryUsage -gt 90) {
    Write-Warning "High memory usage: ${memoryUsage}%"
} else {
    Write-Host "✓ Memory usage acceptable: ${memoryUsage}%" -ForegroundColor Green
}

# Check network connectivity (basic)
try {
    Test-NetConnection -ComputerName "8.8.8.8" -Port 53 -InformationLevel Quiet | Out-Null
    Write-Host "✓ Network connectivity available" -ForegroundColor Green
} catch {
    Write-Warning "Network connectivity check failed"
}

Write-Host ""
Write-Host "=== Deployment Validation Complete ===" -ForegroundColor Cyan
Write-Host "Review the validation results above before proceeding with deployment."