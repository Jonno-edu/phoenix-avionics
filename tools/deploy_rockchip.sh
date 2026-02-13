#!/bin/bash

# --- ROBUST PATH HANDLING ---
# Get the absolute path to the folder where this script lives (workspace/tools)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Get the workspace root (one level up)
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# --- CONFIGURATION ---
ROCKCHIP_USER="rock"
ROCKCHIP_IP="100.115.224.114"
REMOTE_DIR="/home/rock/phoenix_gsu"
LOCAL_PY_DIR="$WORKSPACE_DIR/phoenix-gsu"

# 1. GENERATE CONFIGS
echo "--------------------------------------"
echo "1. Generating Sensor Configurations..."

# Use SCRIPT_DIR to find the python script correctly
python3 "$SCRIPT_DIR/gen_sensors.py"
python3 "$SCRIPT_DIR/gen_telemetry.py"

if [ $? -ne 0 ]; then
    echo "Error: Generator scripts failed."
    exit 1
fi
echo "   - sensor_config.h/py (Updated)"
echo "   - telemetry_defs.py (Updated)"

# 2. DEPLOY TO ROCKCHIP
echo "--------------------------------------"
echo "2. Deploying to Rockchip ($ROCKCHIP_IP)..."

# Use rsync to sync the entire folder
rsync -avz --progress --delete \
    --exclude "__pycache__" \
    "$LOCAL_PY_DIR/" \
    $ROCKCHIP_USER@$ROCKCHIP_IP:$REMOTE_DIR/

if [ $? -eq 0 ]; then
    echo "--------------------------------------"
    echo "SUCCESS: Deployment Complete."
    # echo "Run this on Rockchip: python3 $REMOTE_DIR/rockchip_streamer.py"
else
    echo "--------------------------------------"
    echo "ERROR: Deployment failed. Check IP and SSH keys."
fi