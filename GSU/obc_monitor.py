import serial
import time
import threading
import re
from flask import Flask, render_template
from flask_socketio import SocketIO
from ansi2html import Ansi2HTMLConverter

app = Flask(__name__)
# Force threading mode to ensure background thread works smoothly with Flask
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

SERIAL_PORT = '/dev/ttyOBC'
BAUD_RATE = 115200

# ANSI Converter (inline=True puts styles directly in the HTML tags)
conv = Ansi2HTMLConverter(inline=True)

# Regex to strip ANSI codes just for the regex parser (clean data extraction)
ANSI_ESCAPE = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')

# Regex to parse EPS measurements
# Matches: "Measurements stored: Vbat=0mV, Ibat=0mA, I3V3=54mA, I5V=0mA, I12V=0mA"
EPS_REGEX = re.compile(r'Vbat=(\d+)mV,\s*Ibat=(\d+)mA,\s*I3V3=(\d+)mA,\s*I5V=(\d+)mA,\s*I12V=(\d+)mA')

current_telemetry = {
    'vbat': 0, 'ibat': 0, 'i3v3': 0, 'i5v': 0, 'i12v': 0
}

def read_serial():
    ser = None
    print("--- Background Thread Started ---")
    while True:
        try:
            if ser is None:
                print(f"Attempting to open {SERIAL_PORT}...")
                ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
                ser.dtr = False
                time.sleep(0.1)
                ser.dtr = True
                print(f"SUCCESS: Connected to {SERIAL_PORT}")

            line = ser.readline()
            if line:
                # 1. Decode bytes to string
                raw_text = line.decode('utf-8', errors='replace').rstrip('\r\n')
                
                if not raw_text:
                    continue

                # 2. Convert raw text (with ANSI codes) to HTML for the console
                html_line = conv.convert(raw_text, full=False)
                socketio.emit('log', {'data': html_line})

                # 3. Create a clean version (no ANSI) for regex parsing
                clean_text = ANSI_ESCAPE.sub('', raw_text)

                # 4. Check for measurements
                if "Measurements stored:" in clean_text:
                    match = EPS_REGEX.search(clean_text)
                    if match:
                        current_telemetry['vbat'] = match.group(1)
                        current_telemetry['ibat'] = match.group(2)
                        current_telemetry['i3v3'] = match.group(3)
                        current_telemetry['i5v']  = match.group(4)
                        current_telemetry['i12v'] = match.group(5)
                        
                        socketio.emit('telemetry', current_telemetry)
            else:
                # No data, sleep briefly to save CPU
                time.sleep(0.01)

        except Exception as e:
            print(f"Thread Error: {e}")
            if ser:
                try: ser.close()
                except: pass
            ser = None
            time.sleep(2)

# Start background thread
thread = threading.Thread(target=read_serial)
thread.daemon = True
thread.start()

@app.route('/')
def index():
    return """
    <!DOCTYPE html>
    <html>
    <head>
        <title>Phoenix OBC Dashboard</title>
        <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
        <style>
            body { background: #121212; color: #e0e0e0; font-family: 'Segoe UI', sans-serif; margin: 0; display: flex; height: 100vh; }
            
            /* Sidebar for Live Data */
            #sidebar { width: 300px; background: #1e1e1e; padding: 20px; border-right: 1px solid #333; box-sizing: border-box; }
            .metric { margin-bottom: 20px; background: #2d2d2d; padding: 15px; border-radius: 8px; }
            .metric label { display: block; color: #888; font-size: 0.9em; margin-bottom: 5px; }
            .metric .value { font-size: 1.8em; font-weight: bold; color: #4CAF50; }
            .unit { font-size: 0.5em; color: #aaa; }

            /* Main Console Area */
            #console-container { flex: 1; padding: 20px; display: flex; flex-direction: column; box-sizing: border-box; }
            h2 { margin-top: 0; font-size: 1.2em; color: #fff; }
            
            #log { 
                flex: 1; 
                background: #000; 
                font-family: 'Courier New', monospace; 
                padding: 15px; 
                overflow-y: auto; 
                border: 1px solid #333; 
                border-radius: 4px;
                white-space: pre-wrap;
                font-size: 0.9em;
                line-height: 1.2;
            }

            /* Make sure ansi2html colors show up against dark background */
            .ansi_fore_black   { color: #000000; }
            .ansi_fore_red     { color: #ff5555; }
            .ansi_fore_green   { color: #55ff55; }
            .ansi_fore_yellow  { color: #ffff55; }
            .ansi_fore_blue    { color: #5555ff; }
            .ansi_fore_magenta { color: #ff55ff; }
            .ansi_fore_cyan    { color: #55ffff; }
            .ansi_fore_white   { color: #ffffff; }
        </style>
    </head>
    <body>
        <!-- Live Telemetry Sidebar -->
        <div id="sidebar">
            <h2>Live Data</h2>
            
            <div class="metric">
                <label>Battery Voltage</label>
                <span class="value" id="vbat">0</span> <span class="unit">mV</span>
            </div>
            
            <div class="metric">
                <label>Battery Current</label>
                <span class="value" id="ibat">0</span> <span class="unit">mA</span>
            </div>

            <div class="metric">
                <label>3.3V Line</label>
                <span class="value" id="i3v3">0</span> <span class="unit">mA</span>
            </div>

            <div class="metric">
                <label>5V Line</label>
                <span class="value" id="i5v">0</span> <span class="unit">mA</span>
            </div>
            
             <div class="metric">
                <label>12V Line</label>
                <span class="value" id="i12v">0</span> <span class="unit">mA</span>
            </div>
        </div>

        <!-- Scrolling Logs -->
        <div id="console-container">
            <h2>System Logs</h2>
            <div id="log"></div>
        </div>

        <script>
            // Force robust connection with polling/websocket
            var socket = io({transports: ['websocket', 'polling']});
            var logDiv = document.getElementById('log');

            // Update Dashboard Numbers
            socket.on('telemetry', function(data) {
                document.getElementById('vbat').innerText = data.vbat;
                document.getElementById('ibat').innerText = data.ibat;
                document.getElementById('i3v3').innerText = data.i3v3;
                document.getElementById('i5v').innerText  = data.i5v;
                document.getElementById('i12v').innerText = data.i12v;
            });

            // Append Log Lines
            socket.on('log', function(msg) {
                var p = document.createElement('div');
                p.innerHTML = msg.data; // Using innerHTML to render the ANSI colors
                logDiv.appendChild(p);
                
                // Auto-scroll logic: only if near bottom
                if (logDiv.scrollTop + logDiv.clientHeight >= logDiv.scrollHeight - 100) {
                     logDiv.scrollTop = logDiv.scrollHeight;
                }
                
                // Keep buffer small (prevents browser lag)
                if (logDiv.childElementCount > 2000) {
                    logDiv.removeChild(logDiv.firstChild);
                }
            });
        </script>
    </body>
    </html>
    """

if __name__ == '__main__':
    print("Starting Phoenix OBC Dashboard...")
    socketio.run(app, host='0.0.0.0', port=8080, debug=False, allow_unsafe_werkzeug=True)
