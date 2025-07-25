#!/bin/bash

# Comprehensive test suite runner for beacon system
# This script runs all unit tests, integration tests, and benchmarks

set -e

echo "🚀 Starting comprehensive beacon test suite..."

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
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

# Check if cargo is available
if ! command -v cargo &> /dev/null; then
    print_error "Cargo is not installed or not in PATH"
    exit 1
fi

# Function to run tests with error handling
run_test() {
    local test_name="$1"
    local test_command="$2"
    
    print_status "Running $test_name..."
    
    if eval "$test_command"; then
        print_success "$test_name completed successfully"
        return 0
    else
        print_error "$test_name failed"
        return 1
    fi
}

# Initialize test results
TOTAL_TESTS=0
PASSED_TESTS=0
FAILED_TESTS=0

# Function to update test results
update_results() {
    TOTAL_TESTS=$((TOTAL_TESTS + 1))
    if [ $1 -eq 0 ]; then
        PASSED_TESTS=$((PASSED_TESTS + 1))
    else
        FAILED_TESTS=$((FAILED_TESTS + 1))
    fi
}

echo "📋 Test Plan:"
echo "  1. Unit tests for shared-positioning library"
echo "  2. Unit tests for beacon application"
echo "  3. Integration tests"
echo "  4. Environmental condition tests"
echo "  5. Failure mode tests"
echo "  6. Performance benchmarks"
echo ""

# 1. Unit tests for shared-positioning library
print_status "Phase 1: Shared-positioning unit tests"
run_test "Shared-positioning unit tests" "cd shared-positioning && cargo test --lib"
update_results $?

# 2. Unit tests for beacon application
print_status "Phase 2: Beacon application unit tests"
run_test "Beacon unit tests" "cd beacon && cargo test --lib"
update_results $?

# 3. Integration tests
print_status "Phase 3: Integration tests"
run_test "GPS integration tests" "cd beacon && cargo test --test integration_tests test_gps_integration"
update_results $?

run_test "Power integration tests" "cd beacon && cargo test --test integration_tests test_power_integration"
update_results $?

run_test "Communication integration tests" "cd beacon && cargo test --test integration_tests test_communication_integration"
update_results $?

run_test "End-to-end integration tests" "cd beacon && cargo test --test integration_tests test_end_to_end_beacon_operation"
update_results $?

run_test "Message transmission integration tests" "cd beacon && cargo test --test integration_tests test_message_transmission_integration"
update_results $?

# 4. Environmental condition tests
print_status "Phase 4: Environmental condition tests"
run_test "Extreme cold conditions" "cd beacon && cargo test --test environmental_tests test_extreme_cold_conditions"
update_results $?

run_test "Extreme heat conditions" "cd beacon && cargo test --test environmental_tests test_extreme_heat_conditions"
update_results $?

run_test "Storm conditions" "cd beacon && cargo test --test environmental_tests test_storm_conditions"
update_results $?

run_test "Arctic conditions" "cd beacon && cargo test --test environmental_tests test_arctic_conditions"
update_results $?

run_test "Tropical cyclone conditions" "cd beacon && cargo test --test environmental_tests test_tropical_cyclone_conditions"
update_results $?

# 5. Failure mode tests
print_status "Phase 5: Failure mode tests"
run_test "GPS hardware failure" "cd beacon && cargo test --test failure_mode_tests test_gps_hardware_failure"
update_results $?

run_test "Power system failures" "cd beacon && cargo test --test failure_mode_tests test_power_system_failures"
update_results $?

run_test "Communication failures" "cd beacon && cargo test --test failure_mode_tests test_communication_system_failures"
update_results $?

run_test "Transmission failures" "cd beacon && cargo test --test failure_mode_tests test_transmission_system_failures"
update_results $?

run_test "Cascading failures" "cd beacon && cargo test --test failure_mode_tests test_cascading_failures"
update_results $?

run_test "Recovery from failures" "cd beacon && cargo test --test failure_mode_tests test_recovery_from_failures"
update_results $?

# 6. Performance benchmarks (optional, only if criterion is available)
print_status "Phase 6: Performance benchmarks"

if cargo --list | grep -q "bench"; then
    print_status "Running power consumption benchmarks..."
    if cd beacon && cargo bench --bench power_consumption; then
        print_success "Power consumption benchmarks completed"
        update_results 0
    else
        print_warning "Power consumption benchmarks failed or skipped"
        update_results 1
    fi
    
    print_status "Running transmission reliability benchmarks..."
    if cd beacon && cargo bench --bench transmission_reliability; then
        print_success "Transmission reliability benchmarks completed"
        update_results 0
    else
        print_warning "Transmission reliability benchmarks failed or skipped"
        update_results 1
    fi
    
    print_status "Running message parsing benchmarks..."
    if cd shared-positioning && cargo bench --bench message_parsing; then
        print_success "Message parsing benchmarks completed"
        update_results 0
    else
        print_warning "Message parsing benchmarks failed or skipped"
        update_results 1
    fi
    
    print_status "Running coordinate transformation benchmarks..."
    if cd shared-positioning && cargo bench --bench coordinate_transformations; then
        print_success "Coordinate transformation benchmarks completed"
        update_results 0
    else
        print_warning "Coordinate transformation benchmarks failed or skipped"
        update_results 1
    fi
else
    print_warning "Cargo bench not available, skipping performance benchmarks"
fi

# Test summary
echo ""
echo "📊 Test Results Summary:"
echo "========================"
echo "Total tests: $TOTAL_TESTS"
echo "Passed: $PASSED_TESTS"
echo "Failed: $FAILED_TESTS"

if [ $FAILED_TESTS -eq 0 ]; then
    print_success "All tests passed! 🎉"
    echo ""
    echo "✅ Unit tests: All components tested"
    echo "✅ Integration tests: GPS, power, communication systems validated"
    echo "✅ Environmental tests: Extreme conditions handled"
    echo "✅ Failure mode tests: System resilience verified"
    echo "✅ Performance tests: Benchmarks completed"
    echo ""
    echo "The beacon system is ready for deployment! 🚀"
    exit 0
else
    print_error "$FAILED_TESTS test(s) failed"
    echo ""
    echo "❌ Some tests failed. Please review the output above."
    echo "🔧 Fix the failing tests before deployment."
    exit 1
fi