import json
import os
import sys

# --- CONFIGURATION ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_ROOT = os.path.dirname(SCRIPT_DIR) # Go up one level from 'tools/'

INPUT_FILE = os.path.join(WORKSPACE_ROOT, 'common', 'sensors.json')
OUTPUT_H   = os.path.join(WORKSPACE_ROOT, 'common', 'include', 'sensor_config.h')
OUTPUT_PY  = os.path.join(WORKSPACE_ROOT, 'rockchip', 'sensor_config.py')

def calculate_sensitivity_value(sensor_data):
    """
    Helper: returns the float sensitivity given a dictionary of params.
    Returns None if no sensitivity data is found.
    """
    # CASE 1: Explicit Sensitivity
    if 'sensitivity' in sensor_data and sensor_data['sensitivity'] is not None:
        return float(sensor_data['sensitivity'])

    # CASE 2: Range & Bits (Standard ADC logic)
    bits = sensor_data.get('resolution_bits')
    if bits:
        max_int = 2**(bits - 1)
        if 'range_g' in sensor_data:
            return sensor_data['range_g'] / max_int
        if 'range_dps' in sensor_data:
            return sensor_data['range_dps'] / max_int
        if 'range_ut' in sensor_data:
            return sensor_data['range_ut'] / max_int

    # CASE 3: Scale Factor (Inverse)
    if 'scale_factor' in sensor_data:
        return 1.0 / float(sensor_data['scale_factor'])

    # CASE 4: Lat/Lon Scaling (Legacy support)
    if 'lat_lon_scale' in sensor_data:
        return 1.0 / float(sensor_data['lat_lon_scale'])

    return None

def append_definitions(c_list, py_list, name_base, val):
    """
    Helper to append lines to the C and Python lists
    """
    # C Define
    def_name = f"SENSOR_{name_base}_SENSITIVITY"
    c_list.append(f"#define {def_name:<45} {val:.9f}f")
    
    # Python Var
    py_name = f"{name_base}_SENSITIVITY"
    py_list.append(f"{py_name:<45} = {val}")

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

        # --- A. Check Root Level (e.g. Pressure for BMP581, or simple sensors) ---
        # If the root object itself has sensitivity/units, generate a macro for it.
        root_sens = calculate_sensitivity_value(params)
        if root_sens is not None:
            # If it's a simple sensor, use NAME_SENSITIVITY
            # If it's a hybrid (like BMP which has pressure at root), this is the base sensitivity
            append_definitions(c_lines, py_lines, name_upper, root_sens)

        # --- B. Check Standard Sub-Sensors (accel, gyro, mag, temp) ---
        sub_keys = ['accel', 'gyro', 'mag', 'temp']
        for sub in sub_keys:
            if sub in params:
                sub_sens = calculate_sensitivity_value(params[sub])
                if sub_sens is not None:
                    # e.g. SENSOR_IMU_ACCEL_SENSITIVITY or SENSOR_BMP581_TEMP_SENSITIVITY
                    append_definitions(c_lines, py_lines, f"{name_upper}_{sub.upper()}", sub_sens)

        # --- C. Check Explicit Scaling Map (e.g. GPS: lat_lon, altitude) ---
        if 'scaling' in params and isinstance(params['scaling'], dict):
            for scale_key, scale_val in params['scaling'].items():
                # We assume values in 'scaling' are Divisors (Scale Factors)
                # Sensitivity = 1.0 / scale_val
                sens = 1.0 / float(scale_val)
                append_definitions(c_lines, py_lines, f"{name_upper}_{scale_key.upper()}", sens)

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