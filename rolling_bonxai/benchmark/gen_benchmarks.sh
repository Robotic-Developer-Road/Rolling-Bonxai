#!/bin/bash

################################################################################
# gen_benchmarks.sh
# 
# Automated benchmark script for Rolling Bonxai chunk storage.
# 
# This script:
# 1. Navigates to workspace root
# 2. Builds the project with benchmarks enabled
# 3. Sources the workspace
# 4. Runs the benchmarks
# 5. Saves results to timestamped directory
# 6. Generates CSV and PNG visualizations
#
# Usage:
#   cd ~/ros2_ws/src/Rolling-Bonxai/rolling_bonxai/benchmark
#   ./gen_benchmarks.sh
#
################################################################################

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Print colored message
print_step() {
    echo -e "${BLUE}==>${NC} ${1}"
}

print_success() {
    echo -e "${GREEN}✓${NC} ${1}"
}

print_error() {
    echo -e "${RED}✗${NC} ${1}"
}

print_warning() {
    echo -e "${YELLOW}⚠${NC} ${1}"
}

################################################################################
# Step 1: Determine paths
################################################################################

print_step "Determining workspace structure..."

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Script directory: ${SCRIPT_DIR}"

# Navigate up to find workspace root (where src/ is)
# Current structure: workspace/src/Rolling-Bonxai/rolling_bonxai/benchmark
PACKAGE_DIR="$(dirname "${SCRIPT_DIR}")"  # rolling_bonxai
REPO_DIR="$(dirname "${PACKAGE_DIR}")"    # Rolling-Bonxai
SRC_DIR="$(dirname "${REPO_DIR}")"        # src
WORKSPACE_ROOT="$(dirname "${SRC_DIR}")"  # workspace root

echo "Package directory: ${PACKAGE_DIR}"
echo "Repository directory: ${REPO_DIR}"
echo "Source directory: ${SRC_DIR}"
echo "Workspace root: ${WORKSPACE_ROOT}"

# Verify workspace structure
if [[ ! -d "${SRC_DIR}" ]]; then
    print_error "Cannot find src/ directory at ${SRC_DIR}"
    print_error "Are you running this from the correct location?"
    exit 1
fi

print_success "Workspace structure verified"

################################################################################
# Step 2: Create results directory with timestamp
################################################################################

print_step "Creating results directory..."

# Generate timestamp (format: YYYY-MM-DD_HH-MM-SS)
TIMESTAMP=$(date +"%Y-%m-%d_%H-%M-%S")
RESULTS_DIR="${SCRIPT_DIR}/results/${TIMESTAMP}"

mkdir -p "${RESULTS_DIR}"
print_success "Results directory created: ${RESULTS_DIR}"

################################################################################
# Step 3: Build the project
################################################################################

print_step "Building Rolling Bonxai with benchmarks..."

cd "${WORKSPACE_ROOT}"

# Clean build (optional - comment out if you want faster incremental builds)
# print_warning "Performing clean build..."
# colcon build --packages-select rolling_bonxai --cmake-clean-cache --cmake-args -DBUILD_BENCHMARKS=ON

# Regular build
colcon build --packages-select rolling_bonxai --cmake-args -DBUILD_BENCHMARKS=ON

if [[ $? -ne 0 ]]; then
    print_error "Build failed!"
    exit 1
fi

print_success "Build completed successfully"

################################################################################
# Step 4: Source the workspace
################################################################################

print_step "Sourcing workspace..."

if [[ -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
    source "${WORKSPACE_ROOT}/install/setup.bash"
    print_success "Workspace sourced"
else
    print_error "Cannot find ${WORKSPACE_ROOT}/install/setup.bash"
    exit 1
fi

################################################################################
# Step 5: Run benchmarks
################################################################################

print_step "Running benchmarks (this will take 2-5 minutes)..."
echo ""
echo "=========================================="
echo "  Starting Benchmark Execution"
echo "=========================================="
echo ""

BENCHMARK_OUTPUT="${RESULTS_DIR}/benchmark_raw.txt"

# Run benchmark and capture output
ros2 run rolling_bonxai chunk_storage_benchmark | tee "${BENCHMARK_OUTPUT}"

if [[ $? -ne 0 ]]; then
    print_error "Benchmark execution failed!"
    exit 1
fi

print_success "Benchmarks completed"
print_success "Raw results saved to: ${BENCHMARK_OUTPUT}"
