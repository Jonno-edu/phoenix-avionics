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
LOCAL_PY_DIR="$WORKSPACE_DIR/rockchip"

# 1. GENERATE CONFIGS
echo "--------------------------------------"
echo "1. Generating Sensor Configurations..."

# Use SCRIPT_DIR to find the python script correctly
python3 "$SCRIPT_DIR/gen_sensors.py"

if [ $? -ne 0 ]; then
    echo "Error: Sensor generation failed."
    exit 1
fi
echo "   - sensor_config.h (Updated)"
echo "   - sensor_config.py (Updated)"

# 2. DEPLOY TO ROCKCHIP
echo "--------------------------------------"
echo "2. Deploying to Rockchip ($ROCKCHIP_IP)..."

# Use rsync with the robust paths
rsync -avz --progress \
    "$LOCAL_PY_DIR/rockchip_streamer.py" \
    "$LOCAL_PY_DIR/sensor_config.py" \
    "$LOCAL_PY_DIR/phoenix_sensor_stream.csv" \
    "$LOCAL_PY_DIR/run_hil.sh" \
    "$LOCAL_PY_DIR//hil_node.uf2" \
    $ROCKCHIP_USER@$ROCKCHIP_IP:$REMOTE_DIR/

if [ $? -eq 0 ]; then
    echo "--------------------------------------"
    echo "SUCCESS: Deployment Complete."
    echo "Run this on Rockchip: python3 $REMOTE_DIR/rockchip_streamer.py"
else
    echo "--------------------------------------"
    echo "ERROR: Deployment failed. Check IP and SSH keys."
fi