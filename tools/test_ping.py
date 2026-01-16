import time
import sys
import serial.tools.list_ports
from rs485_lib import RS485Interface, OBCLink, MSG_TC_ACK, Packet, make_desc, MSG_TC

def select_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("❌ No serial ports found! Check your connection.")
        sys.exit(1)
    
    # Auto-select the first port that mentions "Pico"
    for p in ports:
        if "pico" in p.description.lower() or "pico" in p.device.lower():
            print(f"✅ Auto-selected Pico: {p.device} ({p.description})")
            return p.device

    print("\n--- Available Serial Ports ---")
    for i, p in enumerate(ports):
        print(f"[{i}] {p.device} - {p.description}")
    
    if len(ports) == 1:
        selection = input(f"\nOnly one port found. Use {ports[0].device}? [Y/n]: ").strip().lower()
        if selection != 'n':
            return ports[0].device
        sys.exit(0)
            
    while True:
        try:
            selection = input("\nSelect port index: ")
            idx = int(selection)
            if 0 <= idx < len(ports):
                return ports[idx].device
            print("Invalid index.")
        except ValueError:
            print("Please enter a number.")

# 1. Setup Connection
PORT = select_port()
print(f"Connecting to {PORT}...")

link = OBCLink(RS485Interface(port=PORT, baudrate=115200, log_cb=print), host_addr=240)
# Note: connect() might fail if the port is wrong.
if not link.iface.connect():
    print(f"Failed to connect to {PORT}. Please check the port name.")
    exit(1)

link.start_listening()

# 2. Define Callback for ACK
def on_ack(pkt):
    print(f"✅ SUCCESS! Received ACK from Device {pkt['src']}")

# 3. Register Callback
CMD_ID = 1 # RESET
link.dispatcher.register(MSG_TC_ACK, CMD_ID, on_ack)

# --- TEST SCENARIOS ---

def test_bad_crc():
    print(f"\n🔍 [Negative Test] Sending Command with BAD CRC...")
    # Build a valid packet
    desc = make_desc(MSG_TC, CMD_ID)
    pkt = bytearray(Packet.build(dest=1, src=240, desc=desc, data=b""))
    
    # Corrupt the last data byte (before ESC EOM)
    # Packet ends with: ... [CRC_LO] [ESC] [EOM]
    # We target index -3 (CRC_LO) or -4 (CRC_HI) depending on escaping.
    # Simplest way: just flip the 3rd to last byte. 
    # Even if it's an escaped byte, it will break CRC or Framing, which is what we want.
    pkt[-3] = (pkt[-3] + 1) % 256
    
    link.iface.send(bytes(pkt))
    # Give it time to potentially wrongly respond
    time.sleep(1.0)
    print("   (Should verify no ACK above)")

def test_truncated():
    print(f"\n🔍 [Negative Test] Sending Truncated Packet (No EOM)...")
    desc = make_desc(MSG_TC, CMD_ID)
    pkt = Packet.build(dest=1, src=240, desc=desc, data=b"")
    
    # Send all except last 2 bytes (ESC EOM)
    short_pkt = pkt[:-2]
    
    link.iface.send(short_pkt)
    time.sleep(1.0)
    print("   (Should verify no ACK above)")

def test_garbage():
    print(f"\n🔍 [Negative Test] Sending Random Garbage...")
    garbage = b"\xDE\xAD\xBE\xEF\x00\x1F\x00" # Has an ESC but no valid frame
    link.iface.send(garbage)
    time.sleep(1.0)
    print("   (Should verify no ACK above)")

def test_valid():
    print(f"\n🚀 [Positive Test] Sending VALID Command ID {CMD_ID}...")
    link.send_telecommand(dest=1, cmd_id=CMD_ID)
    
    # Wait for callback
    for i in range(10):
        time.sleep(0.1)

# 4. Run Tests
try:
    test_bad_crc()
    test_truncated()
    test_garbage()
    test_valid()
except KeyboardInterrupt:
    pass
finally:
    print("\nTests Complete. Closing connection.")
    link.stop_listening()
    link.iface.disconnect()
