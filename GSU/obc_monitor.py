# APP

import struct
from mission_control import MissionControlApp

# 1. Initialize the App
# Port is set to 8080 to match your existing GSU configuration
mc = MissionControlApp(
    title="Phoenix OBC Dashboard",
    host_addr=0xF0,      # ADDR_GSE from phoenix_icd.h
    broadcast_addr=0x00, 
    target_addr=1,       # Default destination address
    port="/dev/ttyOBC",  # Default COM port
    baud=115200          # Default baud rate
)

# ============================================================================
# nORB TOPIC STREAMING
# Topics are now auto-populated from OBC/build/norb_generated/gsu_norb_dict.py
# which is regenerated every time the OBC firmware is built.
#
# To stream a new sensor topic:
#   1. Drop a .msg file into OBC/msg/
#   2. Build the OBC firmware (CMake regenerates gsu_norb_dict.py)
#   3. Refresh the GSU webpage — the new topic card appears automatically.
#
# No add_norb_topic() calls are needed here anymore.
# ============================================================================

# ============================================================================
# EXECUTION
# ============================================================================

if __name__ == '__main__':
    # host='0.0.0.0' allows Tailscale to route to this app
    # port=8080 matches your previous setup
    mc.run(host='0.0.0.0', port=8080, debug=False)