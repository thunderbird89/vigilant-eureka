#!/bin/bash
# Beacon deployment validation script for Unix systems
# Validates beacon configurations and deployment readiness

set -e

# Default values
DEPLOYMENT_PATH=""
TEST_DURATION=30
VERBOSE=false

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -d|--deployment)
            DEPLOYMENT_PATH="$2"
            shift 2
            ;;
        -t|--test-duration)
            TEST_DURATION="$2"
            shift 2
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 -d|--deployment <path> [-t|--test-duration <seconds>] [-v|--verbose]"
            echo ""
            echo "Options:"
            echo "  -d, --deployment      Path to deployment manifest or directory"
            echo "  -t, --test-duration   Test duration in seconds (default: 30)"
            echo "  -v, --verbose         Enable verbose output"
            echo "  -h, --help           Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Check required parameters
if [[ -z "$DEPLOYMENT_PATH" ]]; then
    echo "Error: Deployment path is required"
    echo "Use -h or --help for usage information"
    exit 1
fi

echo "=== Beacon Deployment Validation ==="
echo "Deployment Path: $DEPLOYMENT_PATH"
echo "Test Duration: $TEST_DURATION seconds"
echo ""

# Check if deployment path exists
if [[ ! -e "$DEPLOYMENT_PATH" ]]; then
    echo "Error: Deployment path does not exist: $DEPLOYMENT_PATH"
    exit 1
fi

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Build the beacon application
echo "Building beacon application..."
cd "$PROJECT_DIR"
if cargo build --release; then
    echo "✓ Build successful"
else
    echo "✗ Build failed"
    exit 1
fi

# Run deployment validation
echo "Running deployment validation..."
ARGS=(
    "deploy"
    "validate-deployment"
    "--deployment" "$DEPLOYMENT_PATH"
    "--test-duration" "$TEST_DURATION"
)

if [[ "$VERBOSE" == "true" ]]; then
    ARGS+=("--verbose")
fi

if ./target/release/beacon "${ARGS[@]}"; then
    echo "✓ Deployment validation completed"
else
    echo "✗ Deployment validation failed"
    exit 1
fi

# Additional system checks
echo "Running system compatibility checks..."

# Check available disk space
AVAILABLE_SPACE=$(df -h . | awk 'NR==2 {print $4}')
echo "✓ Available disk space: $AVAILABLE_SPACE"

# Check memory usage
if command -v free >/dev/null 2>&1; then
    MEMORY_INFO=$(free -h | awk 'NR==2{printf "Memory Usage: %s/%s (%.2f%%)", $3,$2,$3*100/$2}')
    echo "✓ $MEMORY_INFO"
elif command -v vm_stat >/dev/null 2>&1; then
    # macOS
    echo "✓ Memory information available via vm_stat"
else
    echo "? Memory usage information not available"
fi

# Check network connectivity
if ping -c 1 8.8.8.8 >/dev/null 2>&1; then
    echo "✓ Network connectivity available"
else
    echo "⚠ Network connectivity check failed"
fi

# Check for required tools
REQUIRED_TOOLS=("cargo" "rustc")
for tool in "${REQUIRED_TOOLS[@]}"; do
    if command -v "$tool" >/dev/null 2>&1; then
        VERSION=$($tool --version | head -n1)
        echo "✓ $tool: $VERSION"
    else
        echo "✗ Required tool not found: $tool"
        exit 1
    fi
done

echo ""
echo "=== Deployment Validation Complete ==="
echo "Review the validation results above before proceeding with deployment."