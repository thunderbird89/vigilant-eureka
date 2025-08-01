# Local CI runner script for Windows PowerShell
# Runs the same checks as GitHub Actions locally

param(
    [switch]$SkipInteractive = $false
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

Write-Status "🚀 Running local CI checks..."

# Check if cargo is available
if (-not (Get-Command cargo -ErrorAction SilentlyContinue)) {
    Write-Error "Cargo is not installed or not in PATH"
    exit 1
}

# Initialize results
$TotalChecks = 0
$PassedChecks = 0
$FailedChecks = 0

# Function to run a check with error handling
function Invoke-Check {
    param(
        [string]$CheckName,
        [string]$CheckCommand
    )
    
    Write-Status "Running $CheckName..."
    
    try {
        $result = Invoke-Expression $CheckCommand
        $exitCode = $LASTEXITCODE
        
        if ($exitCode -eq 0) {
            Write-Success "$CheckName passed"
            return $true
        } else {
            Write-Error "$CheckName failed with exit code $exitCode"
            return $false
        }
    }
    catch {
        Write-Error "$CheckName failed with exception: $($_.Exception.Message)"
        return $false
    }
}

# Function to update results
function Update-Results {
    param([bool]$Success)
    
    $script:TotalChecks++
    if ($Success) {
        $script:PassedChecks++
    } else {
        $script:FailedChecks++
    }
}

Write-Host "📋 Local CI Check Plan:"
Write-Host "  1. Code formatting check"
Write-Host "  2. Clippy linting"
Write-Host "  3. Build all packages"
Write-Host "  4. Run unit tests"
Write-Host "  5. Run integration tests"
Write-Host "  6. Security audit (if cargo-audit is installed)"
Write-Host ""

# 1. Check formatting
$success = Invoke-Check "Code formatting" "cargo fmt --all -- --check"
Update-Results $success

# 2. Run clippy
$success = Invoke-Check "Clippy linting" "cargo clippy --all-targets --all-features -- -D warnings"
Update-Results $success

# 3. Build all packages
$success = Invoke-Check "Build all packages" "cargo build --verbose --all"
Update-Results $success

# 4. Run unit tests
$success = Invoke-Check "Unit tests" "cargo test --verbose --all --lib"
Update-Results $success

# 5. Run integration tests
$success = Invoke-Check "Integration tests" "cargo test --verbose --all --test '*'"
Update-Results $success

# 6. Security audit (optional)
if (Get-Command cargo-audit -ErrorAction SilentlyContinue) {
    $success = Invoke-Check "Security audit" "cargo audit"
    Update-Results $success
} else {
    Write-Warning "cargo-audit not installed, skipping security audit"
    Write-Warning "Install with: cargo install cargo-audit"
}

# Optional: Run comprehensive test suite if available
if (Test-Path "run_tests.ps1") {
    Write-Status "Comprehensive test suite available"
    if (-not $SkipInteractive) {
        $response = Read-Host "Run comprehensive test suite? (y/N)"
        if ($response -match "^[Yy]$") {
            $success = Invoke-Check "Comprehensive test suite" ".\run_tests.ps1"
            Update-Results $success
        }
    }
}

# Summary
Write-Host ""
Write-Host "📊 Local CI Results Summary:" -ForegroundColor $Blue
Write-Host "============================"
Write-Host "Total checks: $TotalChecks"
Write-Host "Passed: $PassedChecks" -ForegroundColor $Green
Write-Host "Failed: $FailedChecks" -ForegroundColor $(if ($FailedChecks -eq 0) { $Green } else { $Red })

if ($FailedChecks -eq 0) {
    Write-Success "All local CI checks passed! 🎉"
    Write-Host ""
    Write-Host "✅ Your code is ready to push to GitHub" -ForegroundColor $Green
    Write-Host "✅ GitHub Actions should pass successfully" -ForegroundColor $Green
    Write-Host ""
    Write-Host "Next steps:"
    Write-Host "  1. git add ."
    Write-Host "  2. git commit -m `"Your commit message`""
    Write-Host "  3. git push"
    exit 0
} else {
    Write-Error "$FailedChecks check(s) failed"
    Write-Host ""
    Write-Host "❌ Please fix the failing checks before pushing" -ForegroundColor $Red
    Write-Host "🔧 Review the output above for specific issues" -ForegroundColor $Yellow
    Write-Host ""
    Write-Host "Common fixes:"
    Write-Host "  - Run 'cargo fmt' to fix formatting"
    Write-Host "  - Fix clippy warnings and errors"
    Write-Host "  - Ensure all tests pass"
    exit 1
}