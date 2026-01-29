#!/usr/bin/env python3
import serial
import time

def connect_to_pico(port='/dev/ttyACM0', baudrate=115200):
    """Establish serial connection to Pico"""
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
        time.sleep(0.1)  # Allow connection to stabilize
        print(f"Connected to {port}")
        return ser
    except serial.SerialException as e:
        print(f"Error opening {port}: {e}")
        return None

def send_command_and_read(ser, command_byte):
    """Send hex command and read response"""
    try:
        # Send the command as a single byte
        ser.write(bytes([command_byte]))
        print(f"Sent: 0x{command_byte:02X}")
        
        # Wait briefly for response
        time.sleep(0.1)
        
        # Read available data
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print(f"Received ({len(response)} bytes): {response}")
            print(f"Hex: {response.hex()}")
            
            # Try to decode as text if possible
            try:
                text = response.decode('utf-8')
                print(f"Text: {text}")
            except UnicodeDecodeError:
                print("(Response is not valid UTF-8)")
        else:
            print("No response received")
            
    except Exception as e:
        print(f"Error during communication: {e}")

def main():
    # Connect to first Pico on /dev/ttyACM0
    # Change to /dev/ttyACM1 if needed
    ser = connect_to_pico('/dev/ttyACM0')
    
    if ser is None:
        return
    
    try:
        # Send 0x80 command
        send_command_and_read(ser, 0x20)
        
    finally:
        ser.close()
        print("\nConnection closed")

if __name__ == "__main__":
    main()
