# Comprehensive test suite runner for beacon system (Windows PowerShell)
# This script runs all unit tests, integration tests, and benchmarks

param(
    [switch]$SkipBenchmarks = $false,
    [switch]$Verbose = $false
)

# Colors for output
$Red = "Red"
$Green = "Green"
$Yellow = "Yellow"
$Blue = "Blue"

function Write-Status {
    param([string]$Message)
    Write-Host "[INFO] $Message" -ForegroundColor $Blue
}

function Write-Success {
    param([string]$Message)
    Write-Host "[SUCCESS] $Message" -ForegroundColor $Green
}

function Write-Warning {
    param([string]$Message)
    Write-Host "[WARNING] $Message" -ForegroundColor $Yellow
}

function Write-Error {
    param([string]$Message)
    Write-Host "[ERROR] $Message" -ForegroundColor $Red
}

Write-Host "🚀 Starting comprehensive beacon test suite..." -ForegroundColor $Blue

# Check if cargo is available
if (-not (Get-Command cargo -ErrorAction SilentlyContinue)) {
    Write-Error "Cargo is not installed or not in PATH"
    exit 1
}

# Initialize test results
$TotalTests = 0
$PassedTests = 0
$FailedTests = 0

# Function to run tests with error handling
function Invoke-Test {
    param(
        [string]$TestName,
        [string]$TestCommand,
        [string]$WorkingDirectory = "."
    )
    
    Write-Status "Running $TestName..."
    
    $originalLocation = Get-Location
    try {
        Set-Location $WorkingDirectory
        
        if ($Verbose) {
            Write-Host "Executing: $TestCommand" -ForegroundColor Gray
        }
        
        $result = Invoke-Expression $TestCommand
        $exitCode = $LASTEXITCODE
        
        if ($exitCode -eq 0) {
            Write-Success "$TestName completed successfully"
            return $true
        } else {
            Write-Error "$TestName failed with exit code $exitCode"
            return $false
        }
    }
    catch {
        Write-Error "$TestName failed with exception: $($_.Exception.Message)"
        return $false
    }
    finally {
        Set-Location $originalLocation
    }
}

# Function to update test results
function Update-Results {
    param([bool]$Success)
    
    $script:TotalTests++
    if ($Success) {
        $script:PassedTests++
    } else {
        $script:FailedTests++
    }
}

Write-Host "📋 Test Plan:"
Write-Host "  1. Unit tests for shared-positioning library"
Write-Host "  2. Unit tests for beacon application"
Write-Host "  3. Integration tests"
Write-Host "  4. Environmental condition tests"
Write-Host "  5. Failure mode tests"
if (-not $SkipBenchmarks) {
    Write-Host "  6. Performance benchmarks"
}
Write-Host ""

# 1. Unit tests for shared-positioning library
Write-Status "Phase 1: Shared-positioning unit tests"
$success = Invoke-Test "Shared-positioning unit tests" "cargo test --lib" "shared-positioning"
Update-Results $success

# 2. Unit tests for beacon application
Write-Status "Phase 2: Beacon application unit tests"
$success = Invoke-Test "Beacon unit tests" "cargo test --lib" "beacon"
Update-Results $success

# 3. Integration tests
Write-Status "Phase 3: Integration tests"

$integrationTests = @(
    @{ Name = "GPS integration tests"; Test = "test_gps_integration" },
    @{ Name = "Power integration tests"; Test = "test_power_integration" },
    @{ Name = "Communication integration tests"; Test = "test_communication_integration" },
    @{ Name = "End-to-end integration tests"; Test = "test_end_to_end_beacon_operation" },
    @{ Name = "Message transmission integration tests"; Test = "test_message_transmission_integration" }
)

foreach ($test in $integrationTests) {
    $success = Invoke-Test $test.Name "cargo test --test integration_tests $($test.Test)" "beacon"
    Update-Results $success
}

# 4. Environmental condition tests
Write-Status "Phase 4: Environmental condition tests"

$environmentalTests = @(
    "test_extreme_cold_conditions",
    "test_extreme_heat_conditions",
    "test_storm_conditions",
    "test_arctic_conditions",
    "test_tropical_cyclone_conditions"
)

foreach ($test in $environmentalTests) {
    $testName = $test -replace "test_", "" -replace "_", " "
    $success = Invoke-Test $testName "cargo test --test environmental_tests $test" "beacon"
    Update-Results $success
}

# 5. Failure mode tests
Write-Status "Phase 5: Failure mode tests"

$failureTests = @(
    "test_gps_hardware_failure",
    "test_power_system_failures",
    "test_communication_system_failures",
    "test_transmission_system_failures",
    "test_cascading_failures",
    "test_recovery_from_failures"
)

foreach ($test in $failureTests) {
    $testName = $test -replace "test_", "" -replace "_", " "
    $success = Invoke-Test $testName "cargo test --test failure_mode_tests $test" "beacon"
    Update-Results $success
}

# 6. Performance benchmarks (optional)
if (-not $SkipBenchmarks) {
    Write-Status "Phase 6: Performance benchmarks"
    
    # Check if cargo bench is available
    $cargoBenchAvailable = $false
    try {
        $cargoCommands = cargo --list 2>$null
        if ($cargoCommands -match "bench") {
            $cargoBenchAvailable = $true
        }
    }
    catch {
        Write-Warning "Could not check cargo commands"
    }
    
    if ($cargoBenchAvailable) {
        $benchmarks = @(
            @{ Name = "Power consumption benchmarks"; Command = "cargo bench --bench power_consumption"; Dir = "beacon" },
            @{ Name = "Transmission reliability benchmarks"; Command = "cargo bench --bench transmission_reliability"; Dir = "beacon" },
            @{ Name = "Message parsing benchmarks"; Command = "cargo bench --bench message_parsing"; Dir = "shared-positioning" },
            @{ Name = "Coordinate transformation benchmarks"; Command = "cargo bench --bench coordinate_transformations"; Dir = "shared-positioning" }
        )
        
        foreach ($benchmark in $benchmarks) {
            $success = Invoke-Test $benchmark.Name $benchmark.Command $benchmark.Dir
            Update-Results $success
        }
    } else {
        Write-Warning "Cargo bench not available, skipping performance benchmarks"
    }
}

# Test summary
Write-Host ""
Write-Host "📊 Test Results Summary:" -ForegroundColor $Blue
Write-Host "========================"
Write-Host "Total tests: $TotalTests"
Write-Host "Passed: $PassedTests" -ForegroundColor $Green
Write-Host "Failed: $FailedTests" -ForegroundColor $(if ($FailedTests -eq 0) { $Green } else { $Red })

if ($FailedTests -eq 0) {
    Write-Success "All tests passed! 🎉"
    Write-Host ""
    Write-Host "✅ Unit tests: All components tested" -ForegroundColor $Green
    Write-Host "✅ Integration tests: GPS, power, communication systems validated" -ForegroundColor $Green
    Write-Host "✅ Environmental tests: Extreme conditions handled" -ForegroundColor $Green
    Write-Host "✅ Failure mode tests: System resilience verified" -ForegroundColor $Green
    if (-not $SkipBenchmarks) {
        Write-Host "✅ Performance tests: Benchmarks completed" -ForegroundColor $Green
    }
    Write-Host ""
    Write-Host "The beacon system is ready for deployment! 🚀" -ForegroundColor $Green
    exit 0
} else {
    Write-Error "$FailedTests test(s) failed"
    Write-Host ""
    Write-Host "❌ Some tests failed. Please review the output above." -ForegroundColor $Red
    Write-Host "🔧 Fix the failing tests before deployment." -ForegroundColor $Yellow
    exit 1
}