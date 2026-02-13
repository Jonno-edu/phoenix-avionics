import serial
import serial.tools.list_ports
import time
import base64
from threading import Lock
from flask import Flask, render_template
from flask_socketio import SocketIO, emit

app = Flask(__name__)
app.config['SECRET_KEY'] = 'phoenix_launch_key'
socketio = SocketIO(app, async_mode='eventlet', cors_allowed_origins='*')

# --- GLOBAL STATE ---
thread = None
thread_lock = Lock()
ser = None
target_port = None
connection_state = "DISCONNECTED"
# --------------------

def bytes_to_formats(data):
    """Convert raw bytes to multiple display formats."""
    return {
        'hex': data.hex().upper(),
        'binary': ' '.join(format(b, '08b') for b in data),
        'raw_ascii': ''.join(chr(b) if 32 <= b < 127 else '.' for b in data),
        'formatted_ascii': data.decode('utf-8', errors='replace')
    }

def background_serial_reader():
    """Continuously reads from selected serial port."""
    global ser, connection_state, target_port
    print("[SYSTEM] Background Reader Started")
    
    while True:
        try:
            if target_port and (ser is None or not ser.is_open):
                connection_state = "CONNECTING"
                socketio.emit('connection_state', {'state': 'CONNECTING', 'port': target_port})
                
                try:
                    print(f"[SYSTEM] Attempting connect to {target_port}...")
                    ser = serial.Serial(target_port, 115200, timeout=0.1)
                    connection_state = "CONNECTED"
                    print(f"[SYSTEM] Success: {target_port}")
                    socketio.emit('connection_state', {'state': 'CONNECTED', 'port': target_port})
                    socketio.emit('status', {'msg': f'Link Established: {target_port}'})
                except serial.SerialException as e:
                    print(f"[ERROR] Connect failed: {e}")
                    connection_state = "ERROR"
                    socketio.emit('connection_state', {'state': 'ERROR', 'msg': str(e)})
                    time.sleep(2)

            if ser and ser.is_open:
                if ser.in_waiting > 0:
                    try:
                        # Read raw bytes
                        data = ser.read(ser.in_waiting)
                        
                        # Convert to all formats
                        formats = bytes_to_formats(data)
                        formats['raw_bytes'] = base64.b64encode(data).decode('ascii')
                        formats['length'] = len(data)
                        
                        # Send to frontend
                        socketio.emit('telemetry', formats)
                        
                    except Exception as read_error:
                        print(f"[ERROR] Read error: {read_error}")
                
            socketio.sleep(0.01)
            
        except Exception as e:
            print(f"[CRITICAL] Loop Exception: {e}")
            time.sleep(1)

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('connect')
def handle_connect():
    global thread
    with thread_lock:
        if thread is None:
            thread = socketio.start_background_task(background_serial_reader)
    emit('status', {'msg': 'Phoenix GSU Connected'})
    emit('connection_state', {'state': connection_state, 'port': target_port})

@socketio.on('get_ports')
def handle_get_ports():
    ports = serial.tools.list_ports.comports()
    port_list = [{'device': p.device, 'desc': f"{p.device} - {p.description}"} for p in ports]
    emit('port_list', {'ports': port_list})

@socketio.on('connect_device')
def handle_connect_device(data):
    global target_port, ser, connection_state
    
    new_port = data.get('port')
    action = data.get('action')

    if action == 'disconnect':
        target_port = None
        if ser:
            ser.close()
        ser = None
        connection_state = "DISCONNECTED"
        emit('connection_state', {'state': 'DISCONNECTED', 'port': None})
        emit('status', {'msg': 'Link Terminated by User'})
        
    elif action == 'connect':
        if ser and ser.is_open:
            ser.close()
        target_port = new_port
        emit('status', {'msg': f'Targeting {new_port}...'})

@socketio.on('send_command')
def handle_command(json):
    """Handle both text and hex commands."""
    global ser
    cmd = json.get('data')
    cmd_type = json.get('type', 'text')  # 'text' or 'hex'
    
    if not ser or not ser.is_open:
        emit('status', {'msg': 'Error: Uplink Offline'})
        return
    
    try:
        if cmd_type == 'hex':
            # Parse hex string like "0x80" or "80 A0 FF"
            hex_str = cmd.replace('0x', '').replace(' ', '')
            cmd_bytes = bytes.fromhex(hex_str)
            ser.write(cmd_bytes)
            emit('status', {'msg': f'TX HEX: {cmd_bytes.hex().upper()}'})
        else:
            # Text mode - send as UTF-8 with newline
            ser.write((cmd + '\n').encode('utf-8'))
            emit('status', {'msg': f'TX TEXT: {cmd}'})
            
    except ValueError as e:
        emit('status', {'msg': f'Invalid Hex Format: {e}'})
    except Exception as e:
        emit('status', {'msg': f'Tx Error: {e}'})

if __name__ == '__main__':
    print("--- PHOENIX GROUND STATION INITIALIZED ---")
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)
