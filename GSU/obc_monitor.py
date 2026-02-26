# APP

import struct
import re
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

# 1. Write a parser function that takes raw bytes and returns a dictionary
def parse_ident(raw_bytes):
    if len(raw_bytes) < 8: return {}
    # Changed from > (Big Endian) to < (Little Endian) for ARM/RP2040
    node_type, if_ver, fw_maj, fw_min, up_sec, up_ms = struct.unpack("<BBBBHH", raw_bytes[:8])
    return {
        "node_type": node_type,
        "interface_version": if_ver,
        "firmware": f"{fw_maj}.{fw_min}",
        "uptime": up_sec + (up_ms / 1000.0)
    }

# 2. Register the telemetry packet with the framework
mc.add_telemetry(
    tm_id=0, 
    name="Identification", 
    parser=parse_ident,
    
    # Automatically generate a Display Card showing exact values
    displays=[
        {"key": "node_type", "label": "Node Type"},
        {"key": "firmware", "label": "Firmware Version"},
        {"key": "uptime", "label": "Uptime (s)"}
    ],
)

# ============================================================================
# EXECUTION
# ============================================================================

if __name__ == '__main__':
    # host='0.0.0.0' allows Tailscale to route to this app
    # port=8080 matches your previous setup
    mc.run(host='0.0.0.0', port=8080, debug=False)