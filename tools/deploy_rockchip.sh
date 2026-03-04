#!/bin/bash

# --- ROBUST PATH HANDLING ---
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# --- CONFIGURATION ---
ROCKCHIP_USER="rock"
ROCKCHIP_IP="100.115.224.114"
REMOTE_DIR="/home/rock/GSU"
LOCAL_PY_DIR="$WORKSPACE_DIR/GSU"

# --- OS DETECTION ---
OS_TYPE="Unknown"
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    OS_TYPE="Linux"
elif [[ "$OSTYPE" == "darwin"* ]]; then
    OS_TYPE="macOS"
elif [[ "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" ]]; then
    OS_TYPE="Windows (Git Bash/Cygwin)"
else
    OS_TYPE="Other"
fi

echo "Running on: $OS_TYPE"

# --- DEPLOYMENT LOGIC ---
echo "--------------------------------------"
echo "Deploying to Rockchip ($ROCKCHIP_IP)..."

# Check if rsync is available (the most efficient way)
if command -v rsync >/dev/null 2>&1; then
    echo "Using RSYNC (Delta-transfer mode)..."
    # Note: On Windows, we add --chmod to ensure Linux-friendly permissions
    rsync -avz --delete \
          --exclude='venv/' \
          --exclude='__pycache__/' \
          --exclude='*.pyc' \
          "$LOCAL_PY_DIR/" $ROCKCHIP_USER@$ROCKCHIP_IP:$REMOTE_DIR/
else
    echo "RSYNC not found. Falling back to SCP (Full copy mode)..."
    ssh $ROCKCHIP_USER@$ROCKCHIP_IP "mkdir -p $REMOTE_DIR"
    scp -r "$LOCAL_PY_DIR/"* $ROCKCHIP_USER@$ROCKCHIP_IP:$REMOTE_DIR/
fi

# --- POST-DEPLOYMENT ---
if [ $? -eq 0 ]; then
    echo "--------------------------------------"
    echo "SUCCESS: Deployment Complete."
    
    echo "Updating remote dependencies..."
    ssh $ROCKCHIP_USER@$ROCKCHIP_IP "cd $REMOTE_DIR && ./venv/bin/pip install -r requirements.txt"
    
    echo "Restarting obc_monitor.service..."
    # -t is used for sudo password prompts if needed
    ssh -t $ROCKCHIP_USER@$ROCKCHIP_IP "sudo systemctl restart obc_monitor.service"
else
    echo "--------------------------------------"
    echo "ERROR: Deployment failed."
fi