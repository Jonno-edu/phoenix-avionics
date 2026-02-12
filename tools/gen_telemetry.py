import os
import re

# --- CONFIGURATION ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_ROOT = os.path.dirname(SCRIPT_DIR)

INPUT_H = os.path.join(WORKSPACE_ROOT, 'common', 'include', 'telemetry_defs.h')
OUTPUT_PY = os.path.join(WORKSPACE_ROOT, 'rockchip', 'common', 'telemetry_defs.py')

def generate():
    if not os.path.exists(INPUT_H):
        print(f"Error: {INPUT_H} not found")
        return

    print(f"Reading: {INPUT_H}")
    with open(INPUT_H, 'r') as f:
        content = f.read()

    # Match #define NAME VALUE
    # This handles both hex (0x...) and decimal values
    pattern = re.compile(r'#define\s+(\w+)\s+(0x[\da-fA-F]+|\d+)')
    matches = pattern.findall(content)

    # Ensure output directory exists
    os.makedirs(os.path.dirname(OUTPUT_PY), exist_ok=True)

    with open(OUTPUT_PY, 'w') as f:
        f.write("# AUTOMATICALLY GENERATED FROM telemetry_defs.h - DO NOT EDIT\n")
        f.write("# Source: common/include/telemetry_defs.h\n\n")
        for name, val in matches:
            f.write(f"{name} = {val}\n")
    
    print(f"Generated {OUTPUT_PY}")

if __name__ == "__main__":
    generate()
