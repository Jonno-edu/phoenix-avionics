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
# nORB TOPIC STREAMING
# Register topics that the GSU can subscribe to via the nORB Listener page.
# topic_id values match the topic_id_t enum in build/norb_generated/norb/topics.h
# struct_fmt is Python struct format (little-endian '<', matching OBC layout).
# ============================================================================

# TOPIC_OBC_IDENT = 4  —  TlmIdentificationPayload_t: uint8×4 + uint16×2 = 8 B
mc.add_norb_topic(
    topic_id=4,
    name="obc_ident",
    struct_fmt='<BBBBHH',
    struct_fields=['node_type', 'interface_version', 'firmware_major', 'firmware_minor', 'uptime_seconds', 'uptime_ms'],
)

# TOPIC_BAROMETER = 0  —  barometer_t: uint64 + float×3 = 20 B
mc.add_norb_topic(
    topic_id=0,
    name="barometer",
    struct_fmt='<Qfff',
    struct_fields=['timestamp_us', 'pressure_pa', 'temperature_c', 'altitude_m'],
)

# TOPIC_SENSOR_IMU = 6  —  sensor_imu_t: uint64 + float×7 = 36 B
mc.add_norb_topic(
    topic_id=6,
    name="sensor_imu",
    struct_fmt='<Qfffffff',
    struct_fields=['timestamp_us', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z', 'temperature_c'],
)

# TOPIC_VEHICLE_STATE = 8  —  vehicle_state_t: uint64 + float×17 = 76 B
mc.add_norb_topic(
    topic_id=8,
    name="vehicle_state",
    struct_fmt='<Qfffffffffffffffff',
    struct_fields=[
        'timestamp_us',
        'qw', 'qx', 'qy', 'qz',
        'vel_n', 'vel_e', 'vel_d',
        'pos_n', 'pos_e', 'pos_d',
        'acc_bias_x', 'acc_bias_y', 'acc_bias_z',
        'gyro_bias_x', 'gyro_bias_y', 'gyro_bias_z',
        'altitude_m',
    ],
)

# TOPIC_EPS_MEASUREMENTS = 2  —  EpsMeasurements_t: uint16×5 = 10 B
mc.add_norb_topic(
    topic_id=2,
    name="eps_measurements",
    struct_fmt='<HHHHH',
    struct_fields=['batt_voltage_mv', 'batt_current_ma', 'current_3v3_1_ma', 'current_5v_1_ma', 'current_12v_ma'],
)

# ============================================================================
# EXECUTION
# ============================================================================

if __name__ == '__main__':
    # host='0.0.0.0' allows Tailscale to route to this app
    # port=8080 matches your previous setup
    mc.run(host='0.0.0.0', port=8080, debug=False)