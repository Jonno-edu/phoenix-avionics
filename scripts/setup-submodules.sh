#!/bin/bash
# ============================================================================
# Phoenix Avionics - Git Submodules Setup Script
# This script initializes and updates all git submodules required for the build
# ============================================================================

set -e

echo "=============================================="
echo "Phoenix Avionics - Submodule Setup"
echo "=============================================="

# Check if we're in a git repository
if [ ! -d ".git" ]; then
    echo "Warning: Not a git repository. Submodules may need manual setup."
    echo "If you cloned without --recursive, run:"
    echo "  git submodule update --init --recursive"
fi

# Initialize and update submodules
echo ""
echo "Initializing git submodules..."

if git submodule update --init --recursive 2>/dev/null; then
    echo "✓ Submodules initialized successfully"
else
    echo "Warning: Could not initialize submodules via git."
    echo "Checking for required dependencies..."
fi

# Verify critical directories exist
echo ""
echo "Verifying dependencies..."

# Check pico-sdk
if [ -f "lib/pico-sdk/pico_sdk_init.cmake" ]; then
    echo "✓ pico-sdk: OK"
else
    echo "✗ pico-sdk: Missing or incomplete"
    echo "  Will use system pico-sdk at /opt/pico-sdk"
fi

# Check FreeRTOS-Kernel
if [ -f "lib/FreeRTOS-Kernel/tasks.c" ]; then
    echo "✓ FreeRTOS-Kernel: OK"
else
    echo "✗ FreeRTOS-Kernel: Missing (will be downloaded by CMake)"
fi

# Check esl-sensors
if [ -d "lib/esl-sensors/src" ]; then
    echo "✓ esl-sensors: OK"
else
    echo "✗ esl-sensors: Missing or incomplete"
fi

# Check esl-math
if [ -d "lib/esl-math/src" ]; then
    echo "✓ esl-math: OK"
else
    echo "✗ esl-math: Missing or incomplete"
fi

# Check esl-comms
if [ -d "lib/esl-comms/src" ]; then
    echo "✓ esl-comms: OK"
else
    echo "✗ esl-comms: Missing or incomplete"
fi

echo ""
echo "=============================================="
echo "Setup complete!"
echo "=============================================="
echo ""
echo "To build the project:"
echo "  ./scripts/build.sh"
echo ""
echo "To build and flash to Pico via Rock Pi:"
echo "  ./scripts/flash.sh"
echo ""
