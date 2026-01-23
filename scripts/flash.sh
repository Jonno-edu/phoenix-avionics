#!/bin/bash
# ============================================================================
# Phoenix Avionics - Remote Flash Script
# Builds the project, transfers to Rock Pi via SSH, and flashes using picotool
# ============================================================================

set -e

# Configuration - can be overridden via environment variables
ROCK_PI_HOST="${ROCK_PI_HOST:-100.115.224.114}"
ROCK_PI_USER="${ROCK_PI_USER:-rock}"
ROCK_PI_PORT="${ROCK_PI_PORT:-22}"
BUILD_DIR="${BUILD_DIR:-build/Release-rp2350}"
FIRMWARE_NAME="phoenix-avionics.uf2"
REMOTE_FIRMWARE_PATH="~/firmware.uf2"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Parse command line arguments
SKIP_BUILD=false
MONITOR_AFTER=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --skip-build|-s)
            SKIP_BUILD=true
            shift
            ;;
        --monitor|-m)
            MONITOR_AFTER=true
            shift
            ;;
        --host|-h)
            ROCK_PI_HOST="$2"
            shift 2
            ;;
        --user|-u)
            ROCK_PI_USER="$2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  -s, --skip-build    Skip building, use existing UF2"
            echo "  -m, --monitor       Open serial monitor after flashing"
            echo "  -h, --host HOST     Rock Pi hostname/IP (default: ${ROCK_PI_HOST})"
            echo "  -u, --user USER     Rock Pi username (default: ${ROCK_PI_USER})"
            echo "      --help          Show this help message"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

echo -e "${GREEN}=============================================="
echo "Phoenix Avionics - Remote Flash"
echo -e "==============================================${NC}"
echo ""
echo "Configuration:"
echo "  Rock Pi Host: ${ROCK_PI_USER}@${ROCK_PI_HOST}:${ROCK_PI_PORT}"
echo "  Build Dir:    ${BUILD_DIR}"
echo "  Firmware:     ${FIRMWARE_NAME}"
echo ""

# Step 1: Build (unless skipped)
if [ "$SKIP_BUILD" = false ]; then
    echo -e "${CYAN}[1/3] Building firmware...${NC}"
    ./scripts/build.sh
else
    echo -e "${YELLOW}[1/3] Skipping build (using existing firmware)${NC}"
fi

# Check if UF2 file exists
UF2_FILE="${BUILD_DIR}/${FIRMWARE_NAME}"
if [ ! -f "${UF2_FILE}" ]; then
    echo -e "${RED}Error: Firmware file not found: ${UF2_FILE}${NC}"
    exit 1
fi

echo ""
echo -e "${CYAN}[2/3] Transferring firmware to Rock Pi...${NC}"
scp -P "${ROCK_PI_PORT}" "${UF2_FILE}" "${ROCK_PI_USER}@${ROCK_PI_HOST}:${REMOTE_FIRMWARE_PATH}"
echo -e "${GREEN}✓ Firmware transferred${NC}"

echo ""
echo -e "${CYAN}[3/3] Flashing Pico via picotool...${NC}"

# SSH command to flash the Pico
# This handles:
# 1. Finding the Pico serial port
# 2. Killing any processes using the port
# 3. Flashing with picotool (with force reboot)
ssh -p "${ROCK_PI_PORT}" -t "${ROCK_PI_USER}@${ROCK_PI_HOST}" << 'REMOTE_SCRIPT'
set -e

echo "Looking for Raspberry Pi Pico..."

# Find Pico serial port
PICO_PORT=$(ls /dev/serial/by-id/usb-Raspberry_Pi_Pico* 2>/dev/null | head -n 1)

if [ -n "$PICO_PORT" ]; then
    echo "Found Pico at: $PICO_PORT"
    REAL_PORT=$(readlink -f "$PICO_PORT")
    echo "Real device: $REAL_PORT"
    
    # Kill any processes using the serial port
    echo "Clearing sessions on $REAL_PORT..."
    sudo fuser -k "$REAL_PORT" >/dev/null 2>&1 || true
    sleep 0.5
fi

# Flash using picotool with force flag
echo "Flashing firmware..."
sudo picotool load -f ~/firmware.uf2

echo ""
echo "✓ Flash complete!"
REMOTE_SCRIPT

echo ""
echo -e "${GREEN}=============================================="
echo "Flash Successful!"
echo -e "==============================================${NC}"

# Optionally start serial monitor
if [ "$MONITOR_AFTER" = true ]; then
    echo ""
    echo -e "${CYAN}Starting serial monitor...${NC}"
    ./scripts/monitor.sh
fi
