import socket
import sys

# Configuration - Listen on all interfaces
UDP_IP = "0.0.0.0"
UDP_PORT = 5000

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind((UDP_IP, UDP_PORT))
    except OSError as e:
        print(f"Error binding to port {UDP_PORT}: {e}")
        print("Check if another process (like socat) is already using this port.")
        sys.exit(1)

    print(f"Listening for UDP packets on port {UDP_PORT}...")
    print("Press Ctrl+C to stop.")

    packet_count = 0
    
    try:
        while True:
            data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
            packet_count += 1
            
            # Simple hexdump of first 16 bytes to verify content
            # Expecting to see START_FLAG (7E) at the beginning
            hex_preview = data[:16].hex(' ').upper()
            
            # Print status line (overwrite same line to reduce spam)
            # Format: Count | Source | Length | First 16 Bytes
            print(f"RX #{packet_count:<6} from {addr[0]}:{addr[1]} | Len: {len(data):<3} | Data: {hex_preview}...", end='\r')
            
            # Flush stdout to ensure real-time updates over SSH
            sys.stdout.flush()

    except KeyboardInterrupt:
        print("\nStopped.")

if __name__ == "__main__":
    main()
