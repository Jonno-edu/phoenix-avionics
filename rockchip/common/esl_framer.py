import struct

class ESLFramer:
    ESC = 0x1F
    SOM = 0x7F
    EOM = 0xFF

    # Message Types
    MSG_TYPE_EVENT       = 1
    MSG_TYPE_TELECOMMAND = 2
    MSG_TYPE_TC_ACK      = 3
    MSG_TYPE_TLM_REQ     = 4
    MSG_TYPE_TLM_RESP    = 5
    MSG_TYPE_BULK       = 7

    def __init__(self):
        self.rx_buffer = bytearray()
        self.in_escape = False
        self.in_message = False

    def process_byte(self, rx_byte):
        """
        Processes a single byte. 
        Returns the raw packet bytes if a complete message is found, else None.
        The returned packet DOES NOT include the framing (ESC SOM/EOM) but DOES include CRC.
        """
        if self.in_escape:
            self.in_escape = False
            if rx_byte == self.SOM:
                self.rx_buffer = bytearray()
                self.in_message = True
                return None
            elif rx_byte == self.EOM:
                if self.in_message:
                    self.in_message = False
                    return bytes(self.rx_buffer)
                return None
            elif rx_byte == self.ESC:
                if self.in_message:
                    self.rx_buffer.append(self.ESC)
                return None
            else:
                # Invalid escape sequence
                self.in_message = False
                return None

        if rx_byte == self.ESC:
            self.in_escape = True
            return None
        
        if self.in_message:
            self.rx_buffer.append(rx_byte)
        
        return None

    @staticmethod
    def calculate_crc(data: bytes) -> int:
        """CRC16-CCITT (Poly 0x1021, Init 0xFFFF)"""
        crc = 0xFFFF
        for b in data:
            crc ^= (b << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc <<= 1
                crc &= 0xFFFF
        return crc

    def parse_packet(self, raw_pkt: bytes):
        """
        Parses a raw packet (Header + Payload + CRC).
        Returns (header_info, payload) if valid, else (None, None).
        """
        if len(raw_pkt) < 6: # Len(1)+Dest(1)+Src(1)+Desc(1)+CRC(2)
            return None, None

        # 1. Verify CRC
        received_crc = struct.unpack(">H", raw_pkt[-2:])[0]
        calc_crc = self.calculate_crc(raw_pkt[:-2])

        if received_crc != calc_crc:
            return None, None

        # 2. Extract Header
        length = raw_pkt[0]
        dest_addr = raw_pkt[1]
        src_addr = raw_pkt[2]
        msg_desc_raw = raw_pkt[3]
        
        # msg_desc breakdown: 0-4 ID, 5-7 Type
        msg_id = msg_desc_raw & 0x1F
        msg_type = (msg_desc_raw >> 5) & 0x07

        header = {
            'length': length,
            'dest': dest_addr,
            'src': src_addr,
            'id': msg_id,
            'type': msg_type
        }

        payload = raw_pkt[4:-2]
        return header, payload

    def frame_packet(self, dest: int, src: int, msg_type: int, msg_id: int, payload: bytes = b'') -> bytes:
        """
        Encodes a packet into the wire format with escaping and framing.
        """
        # 1. Construct Raw Header + Payload
        # Header: Len(1) + Dest(1) + Src(1) + Desc(1)
        length = len(payload)
        
        # msg_desc: Bits 0-4=ID, 5-7=Type
        msg_desc = (msg_id & 0x1F) | ((msg_type & 0x07) << 5)
        
        raw_pkt = bytearray()
        raw_pkt.append(length)
        raw_pkt.append(dest)
        raw_pkt.append(src)
        raw_pkt.append(msg_desc)
        raw_pkt.extend(payload)
        
        # 2. Calculate CRC
        crc = self.calculate_crc(raw_pkt)
        raw_pkt.append((crc >> 8) & 0xFF)
        raw_pkt.append(crc & 0xFF)
        
        # 3. Apply Framing & Escaping
        wire_pkt = bytearray()
        wire_pkt.append(self.ESC)
        wire_pkt.append(self.SOM)
        
        for byte in raw_pkt:
            if byte == self.ESC:
                wire_pkt.append(self.ESC) # Escape the Escape char
            wire_pkt.append(byte)
            
        wire_pkt.append(self.ESC)
        wire_pkt.append(self.EOM)
        
        return bytes(wire_pkt)
