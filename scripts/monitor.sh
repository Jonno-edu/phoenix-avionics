#!/bin/bash
# ============================================================================
# Phoenix Avionics - Remote Serial Monitor Script
# Connects to the Pico's serial output via the Rock Pi
# ============================================================================

# Configuration - can be overridden via environment variables
ROCK_PI_HOST="${ROCK_PI_HOST:-100.115.224.114}"
ROCK_PI_USER="${ROCK_PI_USER:-rock}"
ROCK_PI_PORT="${ROCK_PI_PORT:-22}"
BAUD_RATE="${BAUD_RATE:-115200}"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${GREEN}=============================================="
echo "Phoenix Avionics - Serial Monitor"
echo -e "==============================================${NC}"
echo ""
echo "Connecting to: ${ROCK_PI_USER}@${ROCK_PI_HOST}"
echo "Baud Rate:     ${BAUD_RATE}"
echo ""
echo -e "${YELLOW}Press Ctrl+A then X to exit minicom${NC}"
echo ""

# SSH into Rock Pi and connect to Pico serial port
ssh -p "${ROCK_PI_PORT}" -t "${ROCK_PI_USER}@${ROCK_PI_HOST}" << REMOTE_SCRIPT
echo "Waiting for Pico serial port..."

# Try to find the Pico for up to 10 seconds
for i in {1..10}; do
    PICO_PORT=\$(ls /dev/serial/by-id/usb-Raspberry_Pi_Pico* 2>/dev/null | head -n 1)
    
    if [ -n "\$PICO_PORT" ]; then
        echo "Found Pico at: \$PICO_PORT"
        REAL_PORT=\$(readlink -f "\$PICO_PORT")
        echo "Real device: \$REAL_PORT"
        
        # Kill any processes using the serial port
        echo "Clearing any existing sessions..."
        sudo fuser -k "\$REAL_PORT" >/dev/null 2>&1 || true
        sleep 0.5
        
        # Start minicom
        echo "Starting minicom..."
        minicom -D "\$PICO_PORT" -b ${BAUD_RATE}
        exit 0
    fi
    
    echo "Waiting... (\$i/10)"
    sleep 1
done

echo "Error: Pico not found after 10 seconds!"
exit 1
REMOTE_SCRIPT
