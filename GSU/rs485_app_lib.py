"""
rs485_lib.py — Reusable RS‑485 comms library.
"""

from __future__ import annotations
import struct
import threading
import time
from typing import Callable, Dict, Optional, Tuple
try:
    import serial  # type: ignore
except Exception as e:
    serial = None

# ===== Framing & Protocol Constants =====
ESC_CHAR = 0x1F
SOM_CHAR = 0x7F
EOM_CHAR = 0xFF

HEADER_SIZE = 4
CRC_SIZE = 2

# msg_type in desc[7:5]
MSG_EVT     = 0b001  # Event
MSG_TC      = 0b010  # Telecommand
MSG_TC_ACK  = 0b011  # Telecommand Acknowledge (Restored)
MSG_TM_REQ  = 0b100  # Telemetry Request
MSG_TM_RPT  = 0b101  # Telemetry Report

def make_desc(msg_type: int, msg_id: int) -> int:
    return ((msg_type & 0x07) << 5) | (msg_id & 0x1F)

def split_desc(desc: int) -> Tuple[int, int]:
    return ((desc >> 5) & 0x07, desc & 0x1F)

# ===== CRC16-CCITT =====
def crc16_ccitt(data: bytes, poly: int = 0x1021, init_val: int = 0xFFFF) -> int:
    crc = init_val
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) & 0xFFFF) ^ poly
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

# ===== Packet =====
class Packet:
    @staticmethod
    def escape_data(data: bytes) -> bytes:
        out = bytearray()
        for b in data:
            if b == ESC_CHAR:
                out += bytes([ESC_CHAR, ESC_CHAR])
            else:
                out.append(b)
        return bytes(out)

    @staticmethod
    def unescape_data(data: bytes) -> bytes:
        out = bytearray()
        i = 0
        while i < len(data):
            if data[i] == ESC_CHAR:
                if i + 1 < len(data) and data[i + 1] == ESC_CHAR:
                    out.append(ESC_CHAR)
                    i += 2
                else:
                    i += 1
            else:
                out.append(data[i])
                i += 1
        return bytes(out)

    @staticmethod
    def calc_crc(data: bytes) -> bytes:
        crc_val = crc16_ccitt(data)
        return crc_val.to_bytes(2, byteorder="big")

    @staticmethod
    def build(dest: int, src: int, desc: int, data: bytes) -> bytes:
        size = len(data)
        header_and_payload = struct.pack("BBBB", size, dest, src, desc) + data
        crc = Packet.calc_crc(header_and_payload)
        body_escaped = Packet.escape_data(header_and_payload)
        crc_escaped = Packet.escape_data(crc)
        return bytes([ESC_CHAR, SOM_CHAR]) + body_escaped + crc_escaped + bytes([ESC_CHAR, EOM_CHAR])

    @staticmethod
    def parse(raw: bytes) -> Optional[dict]:
        if len(raw) < 2 or raw[0] != ESC_CHAR or raw[1] != SOM_CHAR: return None
        try: end_idx = raw.index(bytes([ESC_CHAR, EOM_CHAR]))
        except ValueError: return None
        
        escaped = raw[2:end_idx]
        unescaped = Packet.unescape_data(escaped)
        if len(unescaped) < HEADER_SIZE + CRC_SIZE: return None
        
        body, rx_crc = unescaped[:-2], unescaped[-2:]
        if Packet.calc_crc(body) != rx_crc: return None
        
        size, dest, src, desc = struct.unpack("BBBB", body[:4])
        data = body[4:4 + size]
        return {"size": size, "dest": dest, "src": src, "desc": desc, "data": data, "crc": rx_crc}

# ===== Serial Transport =====
class RS485Interface:
    def __init__(self, port: str = "COM4", baudrate: int = 115200, log_cb: Optional[Callable[[str, str], None]] = None):
        self.port = port
        self.baudrate = baudrate
        self.log_cb = log_cb
        self.ser = None
        self.connected = False

    def connect(self) -> bool:
        try:
            if serial is None: raise RuntimeError("pyserial not available")
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.connected = True
            return True
        except Exception as e:
            if self.log_cb: self.log_cb("ERROR", f"Connect failed: {e}")
            return False

    def disconnect(self) -> None:
        try:
            if self.ser and self.ser.is_open: self.ser.close()
        finally:
            self.connected = False

    def send(self, packet: bytes) -> bool:
        if not self.connected: return False
        try:
            if self.log_cb: self.log_cb("TX", f"Raw: {packet.hex(' ')}")
            self.ser.write(packet)
            return True
        except: return False

    def receive_once(self) -> Optional[bytes]:
        if not self.connected: return None
        buf = bytearray()
        try:
            # 1. Hunt for the start sequence (1F 7F)
            while True:
                b = self.ser.read(1)
                if not b: return None # Timeout
                if b[0] == ESC_CHAR:
                    next_b = self.ser.read(1)
                    if next_b and next_b[0] == SOM_CHAR:
                        buf.extend([ESC_CHAR, SOM_CHAR])
                        break
            
            # 2. Now collect until the end sequence (1F FF)
            while True:
                b = self.ser.read(1)
                if not b: break
                buf += b
                if len(buf) >= 2 and buf[-2:] == bytes([ESC_CHAR, EOM_CHAR]):
                    break
                    
            return bytes(buf) if buf else None
        except:
            return None

# ===== Dispatcher & Link =====
class Dispatcher:
    def __init__(self, log_cb=None):
        self._handlers: Dict[Tuple[int, int], Callable[[dict], None]] = {}
        self.log_cb = log_cb

    def register(self, msg_type: int, msg_id: int, handler: Callable[[dict], None]) -> None:
        self._handlers[(msg_type & 0x07, msg_id & 0x1F)] = handler

    def dispatch(self, pkt: dict) -> None:
        msg_type, msg_id = split_desc(pkt["desc"])
        handler = self._handlers.get((msg_type, msg_id))
        if handler: handler(pkt)
        elif self.log_cb: self.log_cb("WARN", f"No handler for Type={msg_type} ID={msg_id}")

class OBCLink:
    def __init__(self, interface: RS485Interface, host_addr: int = 240, broadcast_addr: int = 0, log_cb=None):
        self.iface = interface
        self.dispatcher = Dispatcher(log_cb)
        self.host_addr = host_addr
        self.broadcast_addr = broadcast_addr  # Store it
        self._listening = False
        self._thread = None

    def send_telecommand(self, dest: int, cmd_id: int, payload: bytes = b"") -> bool:
        return self.iface.send(Packet.build(dest, self.host_addr, make_desc(MSG_TC, cmd_id), payload))

    def request_telemetry(self, dest: int, tm_id: int) -> bool:
        return self.iface.send(Packet.build(dest, self.host_addr, make_desc(MSG_TM_REQ, tm_id), b""))

    def send_event(self, dest: int, evt_id: int, payload: bytes = b"") -> bool:
        return self.iface.send(Packet.build(dest, self.host_addr, make_desc(MSG_EVT, evt_id), payload))

    # Handlers
    def on_tm_report(self, tm_id: int, handler: Callable[[dict], None]):
        self.dispatcher.register(MSG_TM_RPT, tm_id, handler)

    def on_event(self, evt_id: int, handler: Callable[[dict], None]):
        self.dispatcher.register(MSG_EVT, evt_id, handler)

    def on_tc_ack(self, cmd_id: int, handler: Callable[[dict], None]):
        self.dispatcher.register(MSG_TC_ACK, cmd_id, handler)

    # Loop
    def start_listening(self):
        if self._listening: return
        self._listening = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop_listening(self):
        self._listening = False
        if self._thread: self._thread.join(1.0)

    def _loop(self):
        while self._listening:
            raw = self.iface.receive_once()
            if not raw:
                time.sleep(0.01)
                continue
            pkt = Packet.parse(raw)
            # CHECK against both host and broadcast addresses
            if pkt and pkt["dest"] in (self.host_addr, self.broadcast_addr):
                self.dispatcher.dispatch(pkt)