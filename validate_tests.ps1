# Test validation script - checks if test files are properly structured

Write-Host "🔍 Validating test suite structure..." -ForegroundColor Blue

$errors = @()
$warnings = @()

# Check if required test files exist
$requiredFiles = @(
    "beacon/src/beacon_controller_tests.rs",
    "beacon/tests/integration_tests.rs",
    "beacon/tests/environmental_tests.rs", 
    "beacon/tests/failure_mode_tests.rs",
    "beacon/benches/power_consumption.rs",
    "beacon/benches/transmission_reliability.rs",
    "shared-positioning/benches/message_parsing.rs",
    "shared-positioning/benches/coordinate_transformations.rs"
)

foreach ($file in $requiredFiles) {
    if (-not (Test-Path $file)) {
        $errors += "Missing test file: $file"
    } else {
        Write-Host "✅ Found: $file" -ForegroundColor Green
    }
}

# Check Cargo.toml files for test dependencies
$cargoFiles = @("beacon/Cargo.toml", "shared-positioning/Cargo.toml")

foreach ($cargoFile in $cargoFiles) {
    if (Test-Path $cargoFile) {
        $content = Get-Content $cargoFile -Raw
        
        # Check for dev-dependencies section
        if ($content -notmatch '\[dev-dependencies\]') {
            $warnings += "$cargoFile: Missing [dev-dependencies] section"
        }
        
        # Check for required test dependencies
        $requiredDeps = @("tempfile", "tokio-test", "criterion", "proptest", "serial_test")
        foreach ($dep in $requiredDeps) {
            if ($content -notmatch $dep) {
                $warnings += "$cargoFile: Missing dependency: $dep"
            }
        }
        
        Write-Host "✅ Validated: $cargoFile" -ForegroundColor Green
    } else {
        $errors += "Missing Cargo.toml: $cargoFile"
    }
}

# Check for benchmark configurations
$benchmarkConfigs = @(
    @{ File = "beacon/Cargo.toml"; Bench = "power_consumption" },
    @{ File = "beacon/Cargo.toml"; Bench = "transmission_reliability" },
    @{ File = "shared-positioning/Cargo.toml"; Bench = "message_parsing" },
    @{ File = "shared-positioning/Cargo.toml"; Bench = "coordinate_transformations" }
)

foreach ($config in $benchmarkConfigs) {
    if (Test-Path $config.File) {
        $content = Get-Content $config.File -Raw
        if ($content -notmatch "name = `"$($config.Bench)`"") {
            $warnings += "$($config.File): Missing benchmark configuration for $($config.Bench)"
        }
    }
}

# Validate test file syntax (basic check)
$testFiles = Get-ChildItem -Path "beacon/tests/*.rs", "beacon/src/*_tests.rs", "shared-positioning/src/*_tests.rs" -ErrorAction SilentlyContinue

foreach ($testFile in $testFiles) {
    $content = Get-Content $testFile.FullName -Raw
    
    # Check for basic test structure
    if ($content -notmatch '#\[.*test.*\]') {
        $warnings += "$($testFile.Name): No test functions found"
    }
    
    # Check for proper imports
    if ($content -notmatch 'use.*test') {
        $warnings += "$($testFile.Name): Missing test-related imports"
    }
}

# Summary
Write-Host ""
Write-Host "📊 Validation Summary:" -ForegroundColor Blue
Write-Host "====================="

if ($errors.Count -eq 0) {
    Write-Host "✅ No critical errors found" -ForegroundColor Green
} else {
    Write-Host "❌ Critical errors found:" -ForegroundColor Red
    foreach ($error in $errors) {
        Write-Host "  - $error" -ForegroundColor Red
    }
}

if ($warnings.Count -eq 0) {
    Write-Host "✅ No warnings" -ForegroundColor Green
} else {
    Write-Host "⚠️  Warnings:" -ForegroundColor Yellow
    foreach ($warning in $warnings) {
        Write-Host "  - $warning" -ForegroundColor Yellow
    }
}

Write-Host ""
if ($errors.Count -eq 0) {
    Write-Host "🎉 Test suite structure is valid!" -ForegroundColor Green
    Write-Host "You can now run the tests using:" -ForegroundColor Cyan
    Write-Host "  .\run_tests.ps1" -ForegroundColor Cyan
    exit 0
} else {
    Write-Host "🔧 Please fix the critical errors before running tests" -ForegroundColor Red
    exit 1
}