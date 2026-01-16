#!/usr/bin/env python3
import argparse
import time
import serial

# Match rs485_protocol.h / rs485_protocol.c
RS485_ESC = 0x1F
RS485_SOM = 0x7F
RS485_EOM = 0xFF

ADDR_OBC = 0x01
ADDR_GSE = 0xF0

# Message Types (bits 0-2)
MSG_TYPE_EVENT       = 0b001
MSG_TYPE_TELECOMMAND = 0b010
MSG_TYPE_TC_ACK      = 0b011
MSG_TYPE_TLM_REQ     = 0b100
MSG_TYPE_TLM_RESP    = 0b101

# IDs (bits 3-7) from rs485_protocol.h
ID_CMD_RESET = 0b00001
ID_TLM_IDENTIFICATION = 0b00001

def build_msg_desc(msg_type, msg_id):
    # Spec: desc = (ID << 3) | (Type & 0x07)
    return (((msg_id & 0x1F) << 3) | (msg_type & 0x07))

def crc16_ccitt(data: bytes) -> int:
    # Poly 0x1021, init 0xFFFF (matches firmware)
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def escape_bytes(raw: bytes) -> bytes:
    # Firmware escapes ONLY the ESC byte by duplicating it
    out = bytearray()
    for b in raw:
        if b == RS485_ESC:
            out.append(RS485_ESC)
            out.append(RS485_ESC)
        else:
            out.append(b)
    return bytes(out)

def frame_packet(payload_len, dest, src, msg_desc, payload: bytes) -> bytes:
    header = bytes([payload_len & 0xFF, dest & 0xFF, src & 0xFF, msg_desc & 0xFF])
    body = header + payload
    crc = crc16_ccitt(body)
    raw = body + bytes([(crc >> 8) & 0xFF, crc & 0xFF])

    framed = bytearray()
    framed += bytes([RS485_ESC, RS485_SOM])
    framed += escape_bytes(raw)
    framed += bytes([RS485_ESC, RS485_EOM])
    return bytes(framed)

def read_one_packet(ser: serial.Serial, timeout_s=1.0):
    start = time.time()
    rx = bytearray()
    in_frame = False
    escape_flag = False

    while time.time() - start < timeout_s:
        b = ser.read(1)
        if not b:
            continue
        byte = b[0]

        if escape_flag:
            escape_flag = False
            if byte == RS485_SOM:
                # START OF MESSAGE
                rx.clear()
                in_frame = True
            elif byte == RS485_EOM:
                # END OF MESSAGE
                if not in_frame:
                    continue
                # Validate minimum size: len+dest+src+desc + crc(2) = 6
                if len(rx) < 6:
                    return None

                received_crc = (rx[-2] << 8) | rx[-1]
                calc_crc = crc16_ccitt(bytes(rx[:-2]))
                if received_crc != calc_crc:
                    return None

                payload_len = rx[0]
                dest = rx[1]
                src = rx[2]
                msg_desc = rx[3]
                payload = bytes(rx[4:-2])

                # payload_len is authoritative; but be tolerant if stream differs slightly
                if len(payload) != payload_len:
                    payload = payload[:payload_len]

                msg_type = msg_desc & 0x07
                msg_id = (msg_desc >> 3) & 0x1F

                return {
                    "len": payload_len,
                    "dest": dest,
                    "src": src,
                    "msg_desc": msg_desc,
                    "msg_type": msg_type,
                    "msg_id": msg_id,
                    "payload": payload,
                    "crc": received_crc,
                }
            elif byte == RS485_ESC:
                # Escaped ESC byte literal
                if in_frame:
                    rx.append(RS485_ESC)
            else:
                # Invalid escape sequence -> reset
                in_frame = False
                rx.clear()
        else:
            if byte == RS485_ESC:
                escape_flag = True
            else:
                if in_frame:
                    rx.append(byte)

    return None

def send_and_wait(ser, pkt: bytes, wait_s=1.0):
    ser.reset_input_buffer()
    ser.write(pkt)
    ser.flush()
    return read_one_packet(ser, timeout_s=wait_s)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    print(f"Connecting to {args.port} @ {args.baud}")
    ser = serial.Serial(args.port, args.baud, timeout=0.05)

    # 1) Telemetry request: identification
    tlm_req_desc = build_msg_desc(MSG_TYPE_TLM_REQ, ID_TLM_IDENTIFICATION)
    pkt = frame_packet(0, ADDR_OBC, ADDR_GSE, tlm_req_desc, b"")
    resp = send_and_wait(ser, pkt, wait_s=1.0)

    if resp:
        print("Got response:")
        print(resp)
        if resp["msg_type"] == MSG_TYPE_TLM_RESP and resp["msg_id"] == ID_TLM_IDENTIFICATION:
            p = resp["payload"]
            if len(p) >= 8:
                node_type = p[0]
                iface_ver = p[1]
                fw_major = p[2]
                fw_minor = p[3]
                runtime_sec = (p[4] << 8) | p[5]
                runtime_ms  = (p[6] << 8) | p[7]
                print(f"Identification: node={node_type}, iface={iface_ver}, fw={fw_major}.{fw_minor}, uptime={runtime_sec}.{runtime_ms:03d}s")
    else:
        print("No / invalid telemetry response received")

    # 2) Telecommand: reset
    tc_desc = build_msg_desc(MSG_TYPE_TELECOMMAND, ID_CMD_RESET)
    pkt = frame_packet(0, ADDR_OBC, ADDR_GSE, tc_desc, b"")
    resp = send_and_wait(ser, pkt, wait_s=1.0)

    if resp and resp["msg_type"] == MSG_TYPE_TC_ACK and resp["msg_id"] == ID_CMD_RESET:
        print("Received RESET ACK")
    else:
        print("No / invalid RESET ACK received")

    ser.close()

if __name__ == "__main__":
    main()
