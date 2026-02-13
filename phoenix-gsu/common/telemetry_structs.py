import ctypes
try:
    from . import telemetry_defs as tlm
except ImportError:
    import telemetry_defs as tlm

class SensorData(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("time", ctypes.c_uint32),
        ("accel_x", ctypes.c_int16),
        ("accel_y", ctypes.c_int16),
        ("accel_z", ctypes.c_int16),
    ]

class SystemStatus(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("time", ctypes.c_uint32),
        ("mode", ctypes.c_uint8),
        ("vbat_mv", ctypes.c_uint16),
        ("cpu_load", ctypes.c_uint8),
    ]

class SystemIdent(ctypes.Structure):
    _pack_ = 1
    _fields_ = [
        ("node_type", ctypes.c_uint8),
        ("interface_version", ctypes.c_uint8),
        ("firmware_major", ctypes.c_uint8),
        ("firmware_minor", ctypes.c_uint8),
        ("uptime_seconds", ctypes.c_uint16),
        ("uptime_milliseconds", ctypes.c_uint16),
    ]

# Helper to map IDs to Classes
TELEMETRY_MAP = {
    tlm.TLM_COMMON_IDENT: SystemIdent,
    tlm.TLM_OBC_SENSOR_DATA: SensorData,
    # Add others as needed
}
