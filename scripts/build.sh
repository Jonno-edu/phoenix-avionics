#!/bin/bash
# ============================================================================
# Phoenix Avionics - Build Script
# Builds the project for Raspberry Pi Pico 2
# ============================================================================

set -e

# Configuration
BUILD_DIR="${BUILD_DIR:-build/Release-rp2350}"
PICO_BOARD="${PICO_BOARD:-pico2_w}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
JOBS="${JOBS:-$(nproc)}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=============================================="
echo "Phoenix Avionics - Build"
echo -e "==============================================${NC}"
echo ""
echo "Configuration:"
echo "  Build Directory: ${BUILD_DIR}"
echo "  Pico Board:      ${PICO_BOARD}"
echo "  Build Type:      ${BUILD_TYPE}"
echo "  Parallel Jobs:   ${JOBS}"
echo ""

# Determine PICO_SDK_PATH
if [ -f "lib/pico-sdk/pico_sdk_init.cmake" ]; then
    export PICO_SDK_PATH="$(pwd)/lib/pico-sdk"
    echo -e "${GREEN}Using local pico-sdk${NC}"
elif [ -d "/opt/pico-sdk" ]; then
    export PICO_SDK_PATH="/opt/pico-sdk"
    echo -e "${YELLOW}Using system pico-sdk at /opt/pico-sdk${NC}"
else
    echo -e "${RED}Error: pico-sdk not found!${NC}"
    echo "Please run: git submodule update --init --recursive"
    exit 1
fi

# Create build directory
mkdir -p "${BUILD_DIR}"

# Configure with CMake
echo ""
echo -e "${GREEN}Configuring CMake...${NC}"
cmake -B "${BUILD_DIR}" \
    -G Ninja \
    -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
    -DPICO_BOARD="${PICO_BOARD}" \
    -DPICO_SDK_PATH="${PICO_SDK_PATH}"

# Build
echo ""
echo -e "${GREEN}Building...${NC}"
cmake --build "${BUILD_DIR}" -j "${JOBS}"

# Check for output
UF2_FILE="${BUILD_DIR}/phoenix-avionics.uf2"
if [ -f "${UF2_FILE}" ]; then
    echo ""
    echo -e "${GREEN}=============================================="
    echo "Build Successful!"
    echo -e "==============================================${NC}"
    echo ""
    echo "Output files:"
    ls -lh "${BUILD_DIR}"/phoenix-avionics.{uf2,elf,bin,hex} 2>/dev/null || true
    echo ""
    echo "To flash to Pico via Rock Pi:"
    echo "  ./scripts/flash.sh"
else
    echo ""
    echo -e "${RED}Build may have failed - UF2 file not found${NC}"
    exit 1
fi
