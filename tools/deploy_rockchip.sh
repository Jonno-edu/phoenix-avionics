#!/bin/bash

# --- ROBUST PATH HANDLING ---
# Get the absolute path to the folder where this script lives (workspace/tools)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
# Get the workspace root (one level up)
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# --- CONFIGURATION ---
ROCKCHIP_USER="rock"
ROCKCHIP_IP="100.115.224.114"
REMOTE_DIR="/home/rock/GSU"
LOCAL_PY_DIR="$WORKSPACE_DIR/GSU"

# 1. GENERATE CONFIGS
echo "--------------------------------------"
echo "1. Generating Sensor Configurations..."

# Use SCRIPT_DIR to find the python script correctly
# python3 "$SCRIPT_DIR/gen_sensors.py"
# python3 "$SCRIPT_DIR/gen_telemetry.py"

# if [ $? -ne 0 ]; then
#     echo "Error: Generator scripts failed."
#     exit 1
# fi
# echo "   - sensor_config.h/py (Updated)"
# echo "   - telemetry_defs.py (Updated)"

# 2. DEPLOY TO ROCKCHIP (Using scp for maximum compatibility)
echo "--------------------------------------"
echo "2. Deploying to Rockchip ($ROCKCHIP_IP)..."

# Ensure the remote directory exists first, then copy
ssh $ROCKCHIP_USER@$ROCKCHIP_IP "mkdir -p $REMOTE_DIR"
scp -r "$LOCAL_PY_DIR/"* $ROCKCHIP_USER@$ROCKCHIP_IP:$REMOTE_DIR/

if [ $? -eq 0 ]; then
    echo "--------------------------------------"
    echo "SUCCESS: Deployment Complete."
    
    echo "Restarting obc_monitor.service..."
    ssh -t $ROCKCHIP_USER@$ROCKCHIP_IP "sudo systemctl restart obc_monitor.service"
    
    if [ $? -eq 0 ]; then
        echo "Service restarted successfully."
    else
        echo "Warning: Failed to restart the service. You may need to check sudo permissions."
    fi
else
    echo "--------------------------------------"
    echo "ERROR: Deployment failed. Check IP and SSH keys."
fi