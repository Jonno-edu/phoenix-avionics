#!/bin/bash
# rockchip/run_hil.sh

# 1. Kill old instances (Ignore "not found" errors)
# We exclude 'sshd' to ensure we never kill the connection
pgrep -f "python3 .*rockchip_streamer.py" | grep -v sshd | xargs -r kill

# 2. Wait a moment for ports to free
sleep 0.5

# 3. Run the streamer
echo "--- HIL STREAM STARTED ---"
python3 /home/rock/phoenix_gsu/rockchip_streamer.py