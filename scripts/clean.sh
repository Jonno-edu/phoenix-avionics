#!/bin/bash
# ============================================================================
# Phoenix Avionics - Clean Build Script
# Removes all build artifacts and reconfigures
# ============================================================================

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}=============================================="
echo "Phoenix Avionics - Clean"
echo -e "==============================================${NC}"
echo ""

# Remove build directories
if [ -d "build" ]; then
    echo "Removing build directory..."
    rm -rf build
    echo -e "${GREEN}✓ Build directory removed${NC}"
else
    echo "No build directory found"
fi

# Remove CMake cache files
if [ -f "CMakeCache.txt" ]; then
    rm -f CMakeCache.txt
    echo -e "${GREEN}✓ CMake cache removed${NC}"
fi

if [ -d "CMakeFiles" ]; then
    rm -rf CMakeFiles
    echo -e "${GREEN}✓ CMakeFiles directory removed${NC}"
fi

echo ""
echo -e "${GREEN}Clean complete!${NC}"
echo ""
echo "To rebuild, run:"
echo "  ./scripts/build.sh"
