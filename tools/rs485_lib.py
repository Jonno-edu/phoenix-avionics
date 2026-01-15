"""
rs485_lib.py — Reusable RS‑485 comms library (packetization + high‑level helpers).

This module contains:
- Byte‑stuffed framed packets with ESC/SOM/EOM
- Simple CRC (sum & 0xFFFF, big‑endian)
- Message typing (Telecommand, Telemetry Request, Telemetry Report, Event)
- A transport (serial) and a dispatcher with handler registration
- A high‑level OBCLink that exposes: send_telecommand, request_telemetry, send_event
"""

from __future__ import annotations
import struct
import threading
import time
from typing import Callable, Dict, Optional, Tuple
try:
    import serial  # type: ignore
except Exception as e:  # pragma: no cover
    serial = None  # Allow import on systems without pyserial


# ===== Framing & Protocol Constants =====
ESC_CHAR = 0x1F
SOM_CHAR = 0x7F
EOM_CHAR = 0xFF

HEADER_SIZE = 4  # size (len(data)), dest, src, desc
CRC_SIZE = 2

# msg_type in desc[0:2] (lowest 3 bits)
MSG_EVT     = 0b001
MSG_TC      = 0b010
MSG_TC_ACK  = 0b011
MSG_TM_REQ  = 0b100
MSG_TM_RPT  = 0b101
MSG_BULK    = 0b111


def make_desc(msg_type: int, msg_id: int) -> int:
    return ((msg_id & 0x1F) << 3) | (msg_type & 0x07)


def split_desc(desc: int) -> Tuple[int, int]:
    return (desc & 0x07, (desc >> 3) & 0x1F)

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
                    # Skip stray ESC (could be protocol error)
                    i += 1
            else:
                out.append(data[i])
                i += 1
        return bytes(out)

    @staticmethod
    def calc_crc(data: bytes) -> bytes:
        crc_val = crc16_ccitt(data)
        return crc_val.to_bytes(2, byteorder="big")  # MSB first


    @staticmethod
    def build(dest: int, src: int, desc: int, data: bytes) -> bytes:
        size = len(data)
        msg = struct.pack("BBBB", size, dest, src, desc) + data
        crc_val = Packet.calc_crc(msg)
        # Escape everything including CRC
        full_msg = msg + crc_val
        msg_escaped = Packet.escape_data(full_msg)
        return bytes([ESC_CHAR, SOM_CHAR]) + msg_escaped + bytes([ESC_CHAR, EOM_CHAR])

    @staticmethod
    def parse(raw: bytes) -> Optional[dict]:
        try:
            start_idx = raw.index(bytes([ESC_CHAR, SOM_CHAR]))
            end_idx = raw.index(bytes([ESC_CHAR, EOM_CHAR]), start_idx + 2)
        except ValueError:
            return None
        
        payload = raw[start_idx + 2 : end_idx]
        payload = Packet.unescape_data(payload)
        if len(payload) < HEADER_SIZE + CRC_SIZE:
            return None
            
        # Extract CRC (last 2 bytes of payload)
        body = payload[:-2]
        rx_crc = payload[-2:]
        if Packet.calc_crc(body) != rx_crc:
            return None  # CRC fail
            
        size, dest, src, desc = struct.unpack("BBBB", body[:4])
        data = body[4 : 4 + size]  # use declared size for safety
        return {"size": size, "dest": dest, "src": src, "desc": desc, "data": data, "crc": rx_crc}


# ===== Serial Transport =====
class RS485Interface:
    """
    Thin wrapper over pyserial to send/receive framed packets.
    Provides optional callback for raw logging (tx/rx/errors).
    """
    def __init__(self, port: str = "COM4", baudrate: int = 115200, log_cb: Optional[Callable[[str, str], None]] = None):
        self.port = port
        self.baudrate = baudrate
        self.log_cb = log_cb
        self.ser = None
        self.connected = False

    def connect(self) -> bool:
        try:
            if serial is None:
                raise RuntimeError("pyserial not available")
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1)
            self.connected = True
            return True
        except Exception as e:
            if self.log_cb:
                self.log_cb("ERROR", f"Failed to connect to {self.port}: {e}")
            self.connected = False
            self.ser = None
            return False

    def disconnect(self) -> None:
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        finally:
            self.connected = False
            self.ser = None

    def send(self, packet: bytes) -> bool:
        if not self.connected or not self.ser:
            if self.log_cb:
                self.log_cb("ERROR", "Not connected to serial port")
            return False
        try:
            if self.log_cb:
                self.log_cb("TX", f"Raw: {packet.hex(' ')}")
            self.ser.write(packet)
            return True
        except Exception as e:
            if self.log_cb:
                self.log_cb("ERROR", f"Send error: {e}")
            return False

    def receive_once(self) -> Optional[bytes]:
        if not self.connected or not self.ser:
            return None
        buf = bytearray()
        try:
            while True:
                b = self.ser.read(1)
                if b:
                    buf += b
                    if len(buf) >= 2 and buf[-2:] == bytes([ESC_CHAR, EOM_CHAR]):
                        break
                else:
                    break
            if buf:
                if self.log_cb:
                    self.log_cb("RX", f"Raw: {bytes(buf).hex(' ')}")
                return bytes(buf)
        except Exception as e:
            if self.log_cb:
                self.log_cb("ERROR", f"Receive error: {e}")
            return None
        return None


# ===== Dispatcher =====
class Dispatcher:
    """
    Maintains handler maps per (msg_type, msg_id). Calls matching handlers for incoming packets.
    """
    def __init__(self, log_cb: Optional[Callable[[str, str], None]] = None):
        self._handlers: Dict[Tuple[int, int], Callable[[dict], None]] = {}
        self.log_cb = log_cb

    def register(self, msg_type: int, msg_id: int, handler: Callable[[dict], None]) -> None:
        self._handlers[(msg_type & 0x07, msg_id & 0x1F)] = handler

    def dispatch(self, pkt: dict) -> None:
        desc = pkt["desc"]
        msg_type, msg_id = split_desc(desc)
        key = (msg_type, msg_id)
        fn = self._handlers.get(key)
        if fn:
            fn(pkt)
        else:
            if self.log_cb:
                self.log_cb("WARN", f"No handler for msg_type={msg_type:03b}, id={msg_id}. Raw data: {pkt['data'].hex(' ')}")


# ===== OBC Link (High‑level convenience) =====
class OBCLink:
    """
    High‑level link that couples an RS485Interface with a Dispatcher and provides:
      - send_telecommand(dest, cmd_id, payload)
      - request_telemetry(dest, tm_id)
      - send_event(dest, evt_id, payload)
    It also has an async listener thread that parses and dispatches packets addressed to 'host_addr'.
    """
    def __init__(self, interface: RS485Interface, host_addr: int = 240, log_cb: Optional[Callable[[str, str], None]] = None):
        self.iface = interface
        self.dispatcher = Dispatcher(log_cb=log_cb)
        self.log_cb = log_cb
        self.host_addr = host_addr
        self._listening = False
        self._thread: Optional[threading.Thread] = None

    # --- High‑level senders ---
    def send_telecommand(self, dest: int, cmd_id: int, payload: bytes = b"") -> bool:
        desc = make_desc(MSG_TC, cmd_id)
        pkt = Packet.build(dest=dest, src=self.host_addr, desc=desc, data=payload)
        return self.iface.send(pkt)

    def request_telemetry(self, dest: int, tm_id: int) -> bool:
        desc = make_desc(MSG_TM_REQ, tm_id)
        pkt = Packet.build(dest=dest, src=self.host_addr, desc=desc, data=b"")
        return self.iface.send(pkt)

    def send_event(self, dest: int, evt_id: int, payload: bytes = b"") -> bool:
        desc = make_desc(MSG_EVT, evt_id)
        pkt = Packet.build(dest=dest, src=self.host_addr, desc=desc, data=payload)
        return self.iface.send(pkt)

    # --- Handler registration shortcuts ---
    def on_tm_report(self, tm_id: int, handler: Callable[[dict], None]) -> None:
        self.dispatcher.register(MSG_TM_RPT, tm_id, handler)

    def on_event(self, evt_id: int, handler: Callable[[dict], None]) -> None:
        self.dispatcher.register(MSG_EVT, evt_id, handler)

    def on_tc_response(self, cmd_id: int, handler: Callable[[dict], None]) -> None:
        # If your device sends responses tagged as TC with same ID
        self.dispatcher.register(MSG_TC, cmd_id, handler)

    # --- Listener ---
    def start_listening(self) -> None:
        if self._listening:
            return
        self._listening = True
        self._thread = threading.Thread(target=self._listen_loop, daemon=True)
        self._thread.start()

    def stop_listening(self) -> None:
        self._listening = False
        if self._thread:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _listen_loop(self) -> None:
        while self._listening:
            raw = self.iface.receive_once()
            if not raw:
                time.sleep(0.05)
                continue
            pkt = Packet.parse(raw)
            if not pkt:
                if self.log_cb:
                    self.log_cb("ERROR", "Packet parse failed / CRC mismatch / bad framing")
                continue
            # Only handle packets addressed to host_addr
            if pkt["dest"] != self.host_addr:
                continue
            self.dispatcher.dispatch(pkt)
            time.sleep(0.01)  # Yield a bit
