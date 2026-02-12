import serial
import time
import struct
import argparse
from common.esl_framer import ESLFramer
from common import telemetry_defs as tlm
from common.telemetry_structs import TELEMETRY_MAP, SystemIdent

def main():
    parser = argparse.ArgumentParser(description="Send Identification Request to OBC")
    parser.add_argument("--port", default="/dev/ttyOBC", help="Serial port (default: /dev/ttyOBC)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate")
    args = parser.parse_args()

    print(f"Opening {args.port} at {args.baud}...")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1.0)
    except serial.SerialException as e:
        print(f"Error opening port: {e}")
        return

    framer = ESLFramer()
    
    # 1. Construct Request
    # Requesting IDENT from OBC
    # Type: MSG_TYPE_TLM_REQ (4)
    # ID: TLM_COMMON_IDENT (0)
    # Payload: Empty for a simple Get
    
    print("Sending Identification Request...")
    req_packet = framer.frame_packet(
        dest=tlm.ADDR_OBC, 
        src=tlm.ADDR_GSE,  # We act as GSE/Ground
        msg_type=ESLFramer.MSG_TYPE_TLM_REQ, 
        msg_id=tlm.TLM_COMMON_IDENT
    )
    
    ser.write(req_packet)
    
    # 2. Listen for Response
    start_time = time.time()
    print("Waiting for response...")
    
    while time.time() - start_time < 2.0:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            for byte in data:
                pkt = framer.process_byte(byte)
                if pkt:
                    # We have a valid packet!
                    header, payload = framer.parse_packet(pkt)
                    if not header:
                        print("CRC Error or Invalid Packet")
                        continue
                        
                    print(f"\n[Response Received] Src: 0x{header['src']:02X}, ID: {header['id']}")
                    
                    # Check if it's the response we want
                    if header['id'] == tlm.TLM_COMMON_IDENT and header['src'] == tlm.ADDR_OBC:
                        if header['id'] in TELEMETRY_MAP:
                            # Use ctypes to parse
                            StructType = TELEMETRY_MAP[header['id']]
                            if len(payload) >= ctypes.sizeof(StructType):
                                data = StructType.from_buffer_copy(payload)
                                
                                print("-" * 40)
                                print("       SYSTEM IDENTIFICATION")
                                print("-" * 40)
                                print(f"Node Type:        {data.node_type}")
                                print(f"Interface Ver:    {data.interface_version}")
                                print(f"Firmware Ver:     {data.firmware_major}.{data.firmware_minor}")
                                print(f"Uptime:           {data.uptime_seconds}.{data.uptime_milliseconds} s")
                                print("-" * 40)
                                return
                            else:
                                print(f"Payload too short for struct. Got {len(payload)}, expected {ctypes.sizeof(StructType)}")
                        else:
                            print(f"Unknown ID {header['id']}")
        
        time.sleep(0.01)

    print("Timeout: No response received.")

import ctypes # Imported late for sizeof check above

if __name__ == "__main__":
    main()
