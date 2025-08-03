#!/bin/bash

# Local CI runner script - runs the same checks as GitHub Actions locally
# This helps developers catch issues before pushing to GitHub

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_status "🚀 Running local CI checks..."

# Check if cargo is available
if ! command -v cargo &> /dev/null; then
    print_error "Cargo is not installed or not in PATH"
    exit 1
fi

# Function to run a check with error handling
run_check() {
    local check_name="$1"
    local check_command="$2"
    
    print_status "Running $check_name..."
    
    if eval "$check_command"; then
        print_success "$check_name passed"
        return 0
    else
        print_error "$check_name failed"
        return 1
    fi
}

# Initialize results
TOTAL_CHECKS=0
PASSED_CHECKS=0
FAILED_CHECKS=0

# Function to update results
update_results() {
    TOTAL_CHECKS=$((TOTAL_CHECKS + 1))
    if [ $1 -eq 0 ]; then
        PASSED_CHECKS=$((PASSED_CHECKS + 1))
    else
        FAILED_CHECKS=$((FAILED_CHECKS + 1))
    fi
}

echo "📋 Local CI Check Plan:"
echo "  1. Code formatting check"
echo "  2. Clippy linting"
echo "  3. Build all packages"
echo "  4. Run unit tests"
echo "  5. Run integration tests"
echo "  6. Security audit (if cargo-audit is installed)"
echo ""

# 1. Check formatting
run_check "Code formatting" "cargo fmt --all -- --check"
update_results $?

# 2. Run clippy
run_check "Clippy linting" "cargo clippy --all-targets --all-features -- -D warnings"
update_results $?

# 3. Build all packages
run_check "Build all packages" "cargo build --verbose --all"
update_results $?

# 4. Run unit tests
run_check "Unit tests" "cargo test --verbose --all --lib"
update_results $?

# 5. Run integration tests
run_check "Integration tests" "cargo test --verbose --all --test '*'"
update_results $?

# 6. Security audit (optional)
if command -v cargo-audit &> /dev/null; then
    run_check "Security audit" "cargo audit"
    update_results $?
else
    print_warning "cargo-audit not installed, skipping security audit"
    print_warning "Install with: cargo install cargo-audit"
fi

# Optional: Run comprehensive test suite if available
if [ -f "run_tests.sh" ]; then
    print_status "Comprehensive test suite available"
    read -p "Run comprehensive test suite? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        run_check "Comprehensive test suite" "./run_tests.sh"
        update_results $?
    fi
fi

# Summary
echo ""
echo "📊 Local CI Results Summary:"
echo "============================"
echo "Total checks: $TOTAL_CHECKS"
echo "Passed: $PASSED_CHECKS"
echo "Failed: $FAILED_CHECKS"

if [ $FAILED_CHECKS -eq 0 ]; then
    print_success "All local CI checks passed! 🎉"
    echo ""
    echo "✅ Your code is ready to push to GitHub"
    echo "✅ GitHub Actions should pass successfully"
    echo ""
    echo "Next steps:"
    echo "  1. git add ."
    echo "  2. git commit -m \"Your commit message\""
    echo "  3. git push"
    exit 0
else
    print_error "$FAILED_CHECKS check(s) failed"
    echo ""
    echo "❌ Please fix the failing checks before pushing"
    echo "🔧 Review the output above for specific issues"
    echo ""
    echo "Common fixes:"
    echo "  - Run 'cargo fmt' to fix formatting"
    echo "  - Fix clippy warnings and errors"
    echo "  - Ensure all tests pass"
    exit 1
fi