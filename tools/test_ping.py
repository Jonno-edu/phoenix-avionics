import time
import sys
import serial.tools.list_ports
from rs485_lib import RS485Interface, OBCLink, MSG_TC_ACK

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
    print(f"\n✅ SUCCESS! Received ACK from Device {pkt['src']}")

# 3. Register Callback
# We expect an ACK for Command ID 1 (Reset) 
# Note: You might want to use a safer ID like SLEEP/WAKE for testing if RESET actually reboots!
CMD_ID = 1 # RESET
link.dispatcher.register(MSG_TC_ACK, CMD_ID, on_ack)

# 4. Send Command
print(f"Sending Command ID {CMD_ID} to device 1 (OBC)...")
link.send_telecommand(dest=1, cmd_id=CMD_ID)

# 5. Wait
try:
    for i in range(20):
        time.sleep(0.1)
except KeyboardInterrupt:
    pass

link.stop_listening()
link.iface.disconnect()
