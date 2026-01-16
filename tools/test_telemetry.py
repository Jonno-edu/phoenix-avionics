import time
import sys
import struct
import serial.tools.list_ports
from rs485_lib import RS485Interface, OBCLink, MSG_TM_RPT, split_desc

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
if not link.iface.connect():
    print(f"Failed to connect to {PORT}. Please check the port name.")
    exit(1)

link.start_listening()

# 2. Define Callback for Telemetry Response
# Note: In rs485_lib.py the telemetry response type is named MSG_TM_RPT (= 0b101).
TLM_ID_IDENTIFICATION = 1
TLM_ID_UNKNOWN = 2

# Track reception
received_ids = []

def on_telemetry(pkt):
    _, msg_id = split_desc(pkt['desc'])
    received_ids.append(msg_id)

    data = pkt['data']
    if msg_id == TLM_ID_IDENTIFICATION:
        if len(data) != 8:
            print(f"⚠️ Received telemetry but length is wrong: {len(data)} bytes")
            return

        node_type, iface_ver, fw_maj, fw_min, rt_sec, rt_ms = struct.unpack(">BBBBHH", data)
        
        print("\n✅ SUCCESS! Received Identification Telemetry:")
        print(f"   - Node Type:       {node_type}")
        print(f"   - Interface Ver:   {iface_ver}")
        print(f"   - Firmware:        v{fw_maj}.{fw_min}")
        print(f"   - Uptime:          {rt_sec}.{rt_ms:03d} seconds")
    else:
        print(f"ℹ️ Received telemetry for unknown ID: {msg_id}")

# 3. Register Callback
# We register for both IDs we'll try
link.dispatcher.register(MSG_TM_RPT, TLM_ID_IDENTIFICATION, on_telemetry)
link.dispatcher.register(MSG_TM_RPT, TLM_ID_UNKNOWN, on_telemetry)

# 4. Test Case 1: Send Unknown Telemetry ID
print(f"\n--- Test 1: Requesting Unknown Telemetry ID {TLM_ID_UNKNOWN} ---")
print(f"Requesting Telemetry ID {TLM_ID_UNKNOWN} from OBC...")
link.request_telemetry(dest=1, tm_id=TLM_ID_UNKNOWN)

# Wait for potential response (expecting none)
received_ids.clear()
time.sleep(1.0)
if len(received_ids) == 0:
    print(f"✅ SUCCESS: OBC correctly ignored unknown ID {TLM_ID_UNKNOWN}")
else:
    print(f"❌ FAILURE: OBC responded to unknown ID {TLM_ID_UNKNOWN}!")

# 5. Test Case 2: Send ID 1 (to confirm OBC is still responsive)
print(f"\n--- Test 2: Requesting Valid Telemetry ID {TLM_ID_IDENTIFICATION} ---")
received_ids.clear()
print(f"Requesting Telemetry ID {TLM_ID_IDENTIFICATION} from OBC...")
link.request_telemetry(dest=1, tm_id=TLM_ID_IDENTIFICATION)

# Wait for response
for i in range(20):
    if len(received_ids) > 0:
        break
    time.sleep(0.1)

if TLM_ID_IDENTIFICATION in received_ids:
    print(f"✅ SUCCESS: OBC responded correctly to ID {TLM_ID_IDENTIFICATION} after failure test")
else:
    print(f"❌ FAILURE: OBC stopped responding or missed ID {TLM_ID_IDENTIFICATION}!")

# 6. Wait for response
try:
    pass
except KeyboardInterrupt:
    pass
finally:
    print("\nTest Complete. Closing connection.")
    link.stop_listening()
    link.iface.disconnect()
