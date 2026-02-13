#!/usr/bin/env python3
import serial
import time
import struct
import argparse
import sys
import os

# Ensure we can find the common modules
# sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from common.esl_framer import ESLFramer
from common import telemetry_defs as tlm
from common.telemetry_structs import TELEMETRY_MAP

def main():
    parser = argparse.ArgumentParser(description="Monitor Logs from OBC")
    parser.add_argument("--port", default="/dev/ttyOBC", help="Serial port (default: /dev/ttyOBC)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    args = parser.parse_args()

    print(f"Opening {args.port} at {args.baud}...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"Error opening port: {e}")
        return

    framer = ESLFramer()
    
    print("Listening for logs... (Press Ctrl+C to stop)")
    print("-" * 60)

    try:
        while True:
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                for byte in data:
                    pkt = framer.process_byte(byte)
                    if pkt:
                        # We have a valid packet!
                        header, payload = framer.parse_packet(pkt)
                        if not header:
                            # print("CRC Error", end=".", flush=True)
                            continue
                            
                        # Handle Logs
                        if header['id'] == tlm.TLM_COMMON_LOG:
                            if len(payload) > 0:
                                level = payload[0]
                                text = payload[1:].decode('utf-8', errors='replace')
                                
                                if level == tlm.LOG_LEVEL_ERROR:
                                    print(f"\033[91m[ERROR]\033[0m {text}")
                                elif level == tlm.LOG_LEVEL_WARN:
                                    print(f"\033[93m[WARN] \033[0m {text}")
                                elif level == tlm.LOG_LEVEL_INFO:
                                    print(f"\033[32m[INFO] \033[0m {text}")
                                elif level == tlm.LOG_LEVEL_DEBUG:
                                    print(f"\033[90m[DEBUG]\033[0m {text}")
                                else:
                                    print(f"[LOG?]  {text}")
                            else:
                                print(f"[LOG] (Empty)")
                        
                        # Just print a dot or summary for other traffic
                        # else:
                        #    print(f"[Packet ID={header['id']}]")

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
