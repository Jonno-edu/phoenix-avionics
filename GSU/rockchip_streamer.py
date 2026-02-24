import struct
import serial
import csv
import sys
import glob
import time
import os

# --- CONFIGURATION ---
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CSV_FILE = os.path.join(SCRIPT_DIR, 'phoenix_sensor_stream.csv')
BAUD_RATE = 115200

# Packet Structures
FMT_FAST = '<BB fff fff fff'
FMT_SLOW = '<BB dd f fff f f f'

def find_pico_port():
    if sys.platform.startswith('darwin'): ports = glob.glob('/dev/tty.usbmodem*')
    elif sys.platform.startswith('win'): ports = ['COM%s' % (i + 1) for i in range(256)]
    else: ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    valid = []
    for p in ports:
        try:
            s = serial.Serial(p); s.close(); valid.append(p)
        except: pass
    if not valid: sys.exit(1)
    return valid[0]

def run():
    print("Loading CSV into RAM...")
    fast_packets = []
    slow_packets = []
    pk_fast = struct.Struct(FMT_FAST).pack
    pk_slow = struct.Struct(FMT_SLOW).pack

    if os.path.exists(CSV_FILE):
        with open(CSV_FILE, 'r') as f:
            reader = csv.DictReader(f)
            reader.fieldnames = [n.strip() for n in reader.fieldnames]
            idx = 0
            for row in reader:
                try:
                    ax, ay, az = float(row.get('accel_x',0)), float(row.get('accel_y',0)), float(row.get('accel_z',0))
                    gx, gy, gz = float(row.get('gyro_x',0)), float(row.get('gyro_y',0)), float(row.get('gyro_z',0))
                    mx, my, mz = float(row.get('mag_x',0)), float(row.get('mag_y',0)), float(row.get('mag_z',0))
                    fast_packets.append(pk_fast(0xAA, 0x11, ax, ay, az, gx, gy, gz, mx, my, mz))
                    
                    if idx % 20 == 0:
                        lat, lon = float(row.get('lat',0)), float(row.get('lon',0))
                        alt = float(row.get('alt',0))
                        vn, ve, vd = float(row.get('vel_n',0)), float(row.get('vel_e',0)), float(row.get('vel_d',0))
                        prs = float(row.get('pressure_pa',101325))
                        tb = float(row.get('temp_c',25))
                        ts = float(row.get('temp_stack',25))
                        slow_packets.append(pk_slow(0xAA, 0x22, lat, lon, alt, vn, ve, vd, prs, tb, ts))
                    else:
                        slow_packets.append(None)
                    idx += 1
                except ValueError: pass
    else: sys.exit(1)
    
    total_steps = len(fast_packets)
    print(f"Loaded {total_steps} steps. Streaming HIL...")

    ser = serial.Serial(find_pico_port(), BAUD_RATE, timeout=0)
    
    # --- FEED-FORWARD CONTROL ---
    # Start with a reasonable guess for 10ms batch (~5ms processing = 5ms sleep)
    # 0.0045 allows slight initial acceleration to fill buffer
    current_sleep = 0.0045 
    
    sent_count = 0
    last_log = time.time()
    last_pico_stats = "Waiting for Pico..."
    idx = 0
    
    try:
        while True:
            # --- BATCH SENDING (10 steps = 10ms of physics) ---
            chunk = bytearray()
            for _ in range(10): 
                chunk.extend(fast_packets[idx])
                if slow_packets[idx]: chunk.extend(slow_packets[idx])
                idx = (idx + 1) % total_steps
                sent_count += 1
            ser.write(chunk)
            
            # --- ADAPTIVE PACING (Feedback Loop) ---
            if ser.in_waiting:
                try:
                    data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
                    lines = data.split('\n')
                    for line in lines:
                        if "[HIL]" in line and "Buf:" in line:
                            last_pico_stats = line.strip()
                            try:
                                part = line.split("Buf:")[1].split("|")[0].strip()
                                curr, cap = map(int, part.split("/"))
                                ratio = curr / cap
                                
                                # CONTROL LOGIC
                                # EMERGENCY RECOVERY (Buffer Emptying Fast)
                                if ratio < 0.1:     
                                    current_sleep = max(0, current_sleep - 0.0005) # Cut 0.5ms immediately!
                                elif ratio < 0.2:   
                                    current_sleep = max(0, current_sleep - 0.0002) # Cut 0.2ms
                                    
                                # STANDARD TUNING
                                elif ratio < 0.4:   
                                    current_sleep -= 0.00002 # Gentle nudge up
                                elif ratio > 0.8:   
                                    current_sleep += 0.0002  # Brake hard
                                elif ratio > 0.6:   
                                    current_sleep += 0.00002 # Gentle brake
                                    
                                # Clamp (0.1ms to 9.5ms) - never sleep longer than a batch!
                                current_sleep = max(0.0001, min(0.0095, current_sleep))
                            except: pass
                except: pass
            
            time.sleep(current_sleep)

            # --- STATS ---
            if time.time() - last_log > 1.0:
                print(f"HOST: {sent_count} pkts/s | Sleep: {current_sleep*1000:.3f}ms | PICO: {last_pico_stats}")
                sent_count = 0
                last_log = time.time()

    except KeyboardInterrupt:
        ser.close()

if __name__ == "__main__": run()
