import serial
import time
import argparse
import struct
from common.esl_framer import ESLFramer
from common import telemetry_defs as tlm

def main():
    parser = argparse.ArgumentParser(description="Set OBC Log Level")
    parser.add_argument("--port", default="/dev/ttyOBC", help="Serial port (default: /dev/ttyOBC)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    parser.add_argument("level", choices=["none", "error", "info", "debug"], help="Log Level")
    args = parser.parse_args()

    # Map string to integer
    level_map = {
        "none": tlm.LOG_LEVEL_NONE,
        "error": tlm.LOG_LEVEL_ERROR,
        "info": tlm.LOG_LEVEL_INFO,
        "debug": tlm.LOG_LEVEL_DEBUG
    }
    
    target_level = level_map[args.level]

    print(f"Opening {args.port} at {args.baud}...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"Error opening port: {e}")
        return

    framer = ESLFramer()
    
    print(f"Setting Log Level to {args.level.upper()} ({target_level})...")
    
    # Construct Payload (1 byte: Level)
    payload = struct.pack("B", target_level)
    
    # Frame Packet
    # Type: MSG_TYPE_TELECOMMAND (2)
    # ID: TC_OBC_LOG_LEVEL (0x06)
    packet = framer.frame_packet(
        dest=tlm.ADDR_OBC,
        src=tlm.ADDR_GSE,
        msg_type=ESLFramer.MSG_TYPE_TELECOMMAND, 
        msg_id=tlm.TC_OBC_LOG_LEVEL,
        payload=payload
    )
    
    ser.write(packet)
    print("Command Sent.")
    
    # Since we don't strictly expect an ACK for this specific simple command unless configured,
    # we just exit. Ideally, we should listen for an ACK (Type 3), but let's keep it simple first.
    
if __name__ == "__main__":
    main()
