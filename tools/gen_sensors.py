import json
import os
import sys

# --- CONFIGURATION ---
# We determine paths relative to *this script's location* so it works 
# regardless of where you run the command from.
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_ROOT = os.path.dirname(SCRIPT_DIR) # Go up one level from 'tools/'

INPUT_FILE = os.path.join(WORKSPACE_ROOT, 'common', 'sensors.json')
OUTPUT_H   = os.path.join(WORKSPACE_ROOT, 'common', 'include', 'sensor_config.h')
OUTPUT_PY  = os.path.join(WORKSPACE_ROOT, 'rockchip', 'sensor_config.py')

def calculate_sensitivity(sensor_data):
    """
    Calculates the 'Sensitivity' (Value per LSB) based on the JSON data.
    """
    # CASE 1: Explicit Sensitivity (Manual Override in JSON)
    if 'sensitivity' in sensor_data and sensor_data['sensitivity'] is not None:
        return sensor_data['sensitivity']

    # CASE 2: Range & Bits (Standard ADC logic)
    # Sensitivity = Range / (2^(Bits-1))
    if 'range_g' in sensor_data:
        val_range = sensor_data['range_g']
        bits = sensor_data['resolution_bits']
        max_int = 2**(bits - 1)
        return val_range / max_int

    if 'range_dps' in sensor_data:
        val_range = sensor_data['range_dps']
        bits = sensor_data['resolution_bits']
        max_int = 2**(bits - 1)
        return val_range / max_int
        
    if 'range_ut' in sensor_data:
        val_range = sensor_data['range_ut']
        bits = sensor_data['resolution_bits']
        max_int = 2**(bits - 1)
        return val_range / max_int

    # CASE 3: Scale Factor (Inverse Sensitivity)
    # Sensitivity = 1.0 / Scale_Factor
    if 'scale_factor' in sensor_data:
        return 1.0 / sensor_data['scale_factor']

    # CASE 4: Lat/Lon Scaling
    if 'lat_lon_scale' in sensor_data:
        return 1.0 / sensor_data['lat_lon_scale']

    # Default fallback
    return 1.0 

def generate():
    # 1. READ JSON
    if not os.path.exists(INPUT_FILE):
        print(f"Error: Could not find {INPUT_FILE}")
        sys.exit(1)

    print(f"Reading: {INPUT_FILE}")
    with open(INPUT_FILE, 'r') as f:
        data = json.load(f)

    # 2. PREPARE CONTENT BUFFERS
    c_lines = []
    py_lines = []

    # Headers
    c_lines.append("#ifndef SENSOR_CONFIG_H")
    c_lines.append("#define SENSOR_CONFIG_H")
    c_lines.append("// AUTOMATICALLY GENERATED - DO NOT EDIT")
    c_lines.append("// Source: common/sensors.json")
    c_lines.append("")

    py_lines.append("# AUTOMATICALLY GENERATED - DO NOT EDIT")
    py_lines.append("# Source: common/sensors.json")
    py_lines.append("")

    # 3. PROCESS SENSORS
    for sensor_name, params in data.items():
        name_upper = sensor_name.upper()
        
        c_lines.append(f"// --- {name_upper} ---")
        py_lines.append(f"# --- {name_upper} ---")

        # Check for complex sensors (IMU with sub-sensors)
        sub_sensors = []
        if 'accel' in params: sub_sensors.append('accel')
        if 'gyro' in params: sub_sensors.append('gyro')
        if 'mag' in params: sub_sensors.append('mag')
        
        if sub_sensors:
            # Complex Sensor (e.g., IMU)
            for sub in sub_sensors:
                s_data = params[sub]
                sens = calculate_sensitivity(s_data)
                
                # C Define
                def_name = f"SENSOR_{name_upper}_{sub.upper()}_SENSITIVITY"
                c_lines.append(f"#define {def_name:<45} {sens:.9f}f")
                
                # Python Var
                py_name = f"{name_upper}_{sub.upper()}_SENSITIVITY"
                py_lines.append(f"{py_name:<45} = {sens}")
        else:
            # Simple Sensor (Baro, GPS, Temp)
            sens = calculate_sensitivity(params)
            
            # C Define
            def_name = f"SENSOR_{name_upper}_SENSITIVITY"
            c_lines.append(f"#define {def_name:<45} {sens:.9f}f")
            
            # Python Var
            py_name = f"{name_upper}_SENSITIVITY"
            py_lines.append(f"{py_name:<45} = {sens}")

        c_lines.append("")
        py_lines.append("")

    # Footer
    c_lines.append("#endif // SENSOR_CONFIG_H")

    # 4. WRITE C HEADER
    os.makedirs(os.path.dirname(OUTPUT_H), exist_ok=True)
    with open(OUTPUT_H, 'w') as f:
        f.write("\n".join(c_lines))
    print(f"Generated C Header:      {OUTPUT_H}")

    # 5. WRITE PYTHON CONFIG
    os.makedirs(os.path.dirname(OUTPUT_PY), exist_ok=True)
    with open(OUTPUT_PY, 'w') as f:
        f.write("\n".join(py_lines))
    print(f"Generated Python Config: {OUTPUT_PY}")

if __name__ == "__main__":
    generate()