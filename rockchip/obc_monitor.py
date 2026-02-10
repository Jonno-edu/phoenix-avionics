#!/usr/bin/env python3
"""
Phoenix OBC Serial Monitor - Flask Version
Single-file web interface for serial monitoring with ANSI color parsing.
"""

import sys
import time
import serial
import threading
import re
import json
from collections import deque
from datetime import datetime
from flask import Flask, render_template_string, request, jsonify

# ==========================================
# CONFIGURATION
# ==========================================
SERIAL_PORT = "/dev/ttyOBC"  # Change to your port (e.g., COM3, /dev/ttyUSB0)
BAUD_RATE = 115200
MAX_LOG_LINES = 1000
HOST = "0.0.0.0"
PORT = 5000

# ==========================================
# ANSI COLOR PARSER
# ==========================================
def ansi_to_html(text):
    """
    Parses ANSI escape codes from the C header and converts them to HTML spans.
    Specific mappings based on the provided LOGGING_H:
    RED(31), YELLOW(33), GREEN(32), CYAN(36), BLUE(34)
    """
    # Map ANSI codes to CSS colors matching the dark theme
    replacements = {
        r'\x1B\[31m': '<span style="color: #CF6679;">',  # Red (Error)
        r'\x1B\[33m': '<span style="color: #FFB74D;">',  # Yellow (Warning)
        r'\x1B\[32m': '<span style="color: #03DAC6;">',  # Green (Info)
        r'\x1B\[36m': '<span style="color: #4DD0E1;">',  # Cyan (Debug)
        r'\x1B\[34m': '<span style="color: #BB86FC;">',  # Blue (Verbose)
        r'\x1B\[0m': '</span>',                           # Reset
    }
    
    # 1. Handle HTML escaping first (to prevent injection)
    text = text.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")
    
    # 2. Apply ANSI replacements
    for pattern, replacement in replacements.items():
        text = re.sub(pattern, replacement, text)
    
    # 3. Clean up any remaining unknown ANSI codes
    text = re.sub(r'\x1B\[[0-9;]*[a-zA-Z]', '', text)
    
    return text

# ==========================================
# GLOBAL STATE & SERIAL THREAD
# ==========================================
app = Flask(__name__)
serial_lock = threading.Lock()
serial_conn = None
log_buffer = deque(maxlen=MAX_LOG_LINES)
stats = {"rx": 0, "tx": 0, "start_time": time.time(), "connected": False}

def serial_monitor_task():
    """Background thread to read from serial port continuously."""
    global serial_conn
    
    print(f"[*] Starting Serial Monitor on {SERIAL_PORT} @ {BAUD_RATE}...")
    
    while True:
        with serial_lock:
            # parsing connection state
            if serial_conn is None or not serial_conn.is_open:
                try:
                    serial_conn = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
                    stats["connected"] = True
                    log_system_message(f"Connected to {SERIAL_PORT}")
                except Exception:
                    stats["connected"] = False
                    # Wait before retrying
                    pass 
            
            # Reading data
            if serial_conn and serial_conn.is_open:
                try:
                    if serial_conn.in_waiting > 0:
                        # Read line-by-line to preserve log integrity
                        raw_line = serial_conn.readline()
                        try:
                            decoded_line = raw_line.decode('utf-8', errors='replace').strip()
                        except:
                            decoded_line = str(raw_line)
                        
                        if decoded_line:
                            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                            # Parse colors immediately
                            formatted_html = ansi_to_html(decoded_line)
                            
                            entry = {
                                "type": "RX",
                                "time": timestamp,
                                "raw": decoded_line,
                                "html": formatted_html
                            }
                            log_buffer.append(entry)
                            stats["rx"] += 1
                except Exception as e:
                    stats["connected"] = False
                    log_system_message(f"Connection Lost: {e}")
                    if serial_conn:
                        serial_conn.close()
        
        # Sleep slightly to prevent CPU hogging
        time.sleep(0.01)

def log_system_message(msg):
    """Injects a system message into the log buffer."""
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    entry = {
        "type": "SYS",
        "time": timestamp,
        "raw": msg,
        "html": f'<span style="color: #666; font-style: italic;">[SYSTEM] {msg}</span>'
    }
    log_buffer.append(entry)

def send_serial_command(cmd):
    """Sends a command to the serial port."""
    global serial_conn
    with serial_lock:
        if serial_conn and serial_conn.is_open:
            try:
                full_cmd = cmd + "\n"
                serial_conn.write(full_cmd.encode('utf-8'))
                stats["tx"] += 1
                
                # Log the TX
                timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                entry = {
                    "type": "TX",
                    "time": timestamp,
                    "raw": cmd,
                    "html": f'<span style="color: #BB86FC;">TX &gt; {cmd}</span>'
                }
                log_buffer.append(entry)
                return True, "Sent"
            except Exception as e:
                return False, str(e)
        return False, "Not Connected"

# Start the background thread
t = threading.Thread(target=serial_monitor_task, daemon=True)
t.start()

# ==========================================
# FLASK ROUTES
# ==========================================

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/api/status')
def get_status():
    uptime = int(time.time() - stats["start_time"])
    return jsonify({
        "connected": stats["connected"],
        "rx": stats["rx"],
        "tx": stats["tx"],
        "uptime": uptime,
        "port": SERIAL_PORT
    })

@app.route('/api/logs')
def get_logs():
    # Return all logs currently in buffer
    # In a production app, you might use an 'after_index' param to only fetch new logs
    # but for local LAN, sending the last 500 lines JSON is fast enough.
    return jsonify(list(log_buffer))

@app.route('/api/send', methods=['POST'])
def send_cmd():
    data = request.json
    cmd = data.get('cmd', '')
    if not cmd:
        return jsonify({"success": False, "error": "Empty command"})
    
    success, msg = send_serial_command(cmd)
    return jsonify({"success": success, "message": msg})

@app.route('/api/clear', methods=['POST'])
def clear_logs():
    log_buffer.clear()
    stats["rx"] = 0
    stats["tx"] = 0
    return jsonify({"success": True})

# ==========================================
# FRONTEND TEMPLATE
# ==========================================
HTML_TEMPLATE = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Phoenix OBC Monitor</title>
    <link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&family=JetBrains+Mono:wght@400;500&display=swap" rel="stylesheet">
    <style>
        :root {
            --bg: #121212;
            --card: #1E1E1E;
            --border: #333;
            --text: #E0E0E0;
            --primary: #03DAC6;
            --secondary: #BB86FC;
            --danger: #CF6679;
            --input-bg: #2C2C2C;
            --success: #00E676;
        }

        body {
            background-color: var(--bg);
            color: var(--text);
            font-family: 'Inter', sans-serif;
            margin: 0;
            padding: 20px;
            height: 100vh;
            box-sizing: border-box;
            display: flex;
            flex-direction: column;
        }

        /* Header */
        header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding-bottom: 20px;
            border-bottom: 1px solid var(--border);
            margin-bottom: 20px;
        }

        h1 {
            margin: 0;
            font-size: 1.5rem;
            letter-spacing: 1px;
            color: var(--primary);
        }
        
        .status-badge {
            font-size: 0.85rem;
            padding: 4px 12px;
            border-radius: 12px;
            background: var(--input-bg);
            border: 1px solid var(--border);
            display: flex;
            align-items: center;
            gap: 8px;
        }
        
        .led {
            width: 8px;
            height: 8px;
            border-radius: 50%;
            background-color: #555;
        }
        .led.on { background-color: var(--success); box-shadow: 0 0 8px var(--success); }
        .led.off { background-color: var(--danger); }

        /* Main Layout */
        .container {
            display: grid;
            grid-template-columns: 300px 1fr;
            gap: 20px;
            height: 100%;
            overflow: hidden; /* Lock window size */
        }

        /* Sidebar */
        .sidebar {
            display: flex;
            flex-direction: column;
            gap: 20px;
        }

        .card {
            background: var(--card);
            border: 1px solid var(--border);
            border-radius: 8px;
            padding: 15px;
        }

        .card-title {
            font-size: 0.8rem;
            text-transform: uppercase;
            color: #888;
            margin-bottom: 10px;
            font-weight: 600;
        }

        .stat-row {
            display: flex;
            justify-content: space-between;
            margin-bottom: 8px;
            font-size: 0.9rem;
        }
        
        /* Buttons */
        button {
            background: var(--input-bg);
            color: var(--text);
            border: 1px solid var(--border);
            padding: 8px 12px;
            border-radius: 6px;
            cursor: pointer;
            font-weight: 500;
            transition: all 0.2s;
        }
        button:hover { background: var(--primary); color: #000; }
        button:active { transform: translateY(1px); }
        
        .btn-grid {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 8px;
        }

        .btn-sm {
            padding: 4px 8px;
            font-size: 0.8rem;
        }

        /* Terminal Window */
        .terminal-wrapper {
            display: flex;
            flex-direction: column;
            background: #000;
            border: 1px solid var(--border);
            border-radius: 8px;
            overflow: hidden;
        }

        #terminal {
            flex-grow: 1;
            padding: 15px;
            font-family: 'JetBrains Mono', monospace;
            font-size: 0.9rem;
            overflow-y: auto;
            white-space: pre-wrap;
            color: #ccc;
        }

        .log-entry { margin-bottom: 2px; line-height: 1.4; }
        .ts { color: #555; margin-right: 10px; user-select: none; }

        /* Command Input */
        .cmd-bar {
            display: flex;
            padding: 10px;
            background: var(--card);
            border-top: 1px solid var(--border);
        }

        input[type="text"] {
            flex-grow: 1;
            background: var(--input-bg);
            border: 1px solid var(--border);
            color: var(--text);
            padding: 10px;
            border-radius: 6px;
            font-family: 'JetBrains Mono', monospace;
            margin-right: 10px;
        }
        
        input:focus { outline: none; border-color: var(--primary); }

        /* Scrollbar */
        ::-webkit-scrollbar { width: 8px; }
        ::-webkit-scrollbar-track { background: var(--bg); }
        ::-webkit-scrollbar-thumb { background: #444; border-radius: 4px; }
        ::-webkit-scrollbar-thumb:hover { background: #555; }
    </style>
</head>
<body>

<header>
    <h1>PHOENIX <span style="font-size: 0.6em; opacity: 0.6;">OBC MONITOR</span></h1>
    <div class="status-badge">
        <div id="led" class="led"></div>
        <span id="connection-text">Connecting...</span>
        <span style="margin-left:10px; opacity:0.5;" id="uptime">0s</span>
    </div>
</header>

<div class="container">
    <div class="sidebar">
        <div class="card">
            <div class="card-title">Telemetry</div>
            <div class="stat-row">
                <span>RX Packets</span>
                <span id="rx-count" style="color: var(--primary)">0</span>
            </div>
            <div class="stat-row">
                <span>TX Packets</span>
                <span id="tx-count" style="color: var(--secondary)">0</span>
            </div>
        </div>

        <div class="card">
            <div class="card-title">Logging Control</div>
            <div style="display:flex; flex-direction:column; gap:8px;">
                <div style="display:flex; align-items:center; justify-content:space-between;">
                    <span style="font-size:0.85rem;">Heartbeat</span>
                    <div>
                        <button class="btn-sm" onclick="sendCmd('log heartbeat on')">ON</button>
                        <button class="btn-sm" onclick="sendCmd('log heartbeat off')">OFF</button>
                    </div>
                </div>
                <div style="display:flex; align-items:center; justify-content:space-between;">
                    <span style="font-size:0.85rem;">Sensors</span>
                    <div>
                        <button class="btn-sm" onclick="sendCmd('log sensors on')">ON</button>
                        <button class="btn-sm" onclick="sendCmd('log sensors off')">OFF</button>
                    </div>
                </div>
                <div style="display:flex; align-items:center; justify-content:space-between;">
                    <span style="font-size:0.85rem;">RS485</span>
                    <div>
                        <button class="btn-sm" onclick="sendCmd('log rs485 on')">ON</button>
                        <button class="btn-sm" onclick="sendCmd('log rs485 off')">OFF</button>
                    </div>
                </div>
            </div>
        </div>

        <div class="card">
            <div class="card-title">Quick Commands</div>
            <div class="btn-grid">
                <button onclick="sendCmd('STATUS')">STATUS</button>
                <button onclick="sendCmd('PING')">PING</button>
                <button onclick="sendCmd('HELP')">HELP</button>
                <button onclick="sendCmd('RESET')" style="border-color: var(--danger); color: var(--danger);">RESET</button>
            </div>
        </div>
        
        <div style="margin-top:auto;">
            <button style="width:100%;" onclick="clearLogs()">CLEAR TERMINAL</button>
        </div>
    </div>

    <div class="terminal-wrapper">
        <div id="terminal"></div>
        <div class="cmd-bar">
            <input type="text" id="cmd-input" placeholder="Type command... (Enter to send)" autocomplete="off">
            <button onclick="submitCmd()">SEND</button>
        </div>
    </div>
</div>

<script>
    const terminal = document.getElementById('terminal');
    const rxCount = document.getElementById('rx-count');
    const txCount = document.getElementById('tx-count');
    const led = document.getElementById('led');
    const connText = document.getElementById('connection-text');
    const uptimeEl = document.getElementById('uptime');
    const cmdInput = document.getElementById('cmd-input');
    
    let lastLogLength = 0;
    let autoScroll = true;

    // Detect if user scrolled up
    terminal.addEventListener('scroll', () => {
        const isAtBottom = terminal.scrollHeight - terminal.scrollTop <= terminal.clientHeight + 50;
        autoScroll = isAtBottom;
    });

    function formatTime(s) {
        const h = Math.floor(s / 3600);
        const m = Math.floor((s % 3600) / 60);
        const sec = s % 60;
        return `${h}h ${m}m ${sec}s`;
    }

    async function updateStatus() {
        try {
            const res = await fetch('/api/status');
            const data = await res.json();
            
            rxCount.innerText = data.rx;
            txCount.innerText = data.tx;
            uptimeEl.innerText = formatTime(data.uptime);
            
            if (data.connected) {
                led.className = "led on";
                connText.innerText = "Connected";
                connText.style.color = "var(--text)";
            } else {
                led.className = "led off";
                connText.innerText = "Disconnected";
                connText.style.color = "var(--danger)";
            }
        } catch (e) { console.error(e); }
    }

    async function fetchLogs() {
        try {
            const res = await fetch('/api/logs');
            const logs = await res.json();
            
            // Only update DOM if new logs exist
            // (Naive check: length comparison. Robust check would use IDs)
            if (logs.length !== lastLogLength) {
                terminal.innerHTML = logs.map(l => 
                    `<div class="log-entry"><span class="ts">[${l.time}]</span>${l.html}</div>`
                ).join('');
                
                lastLogLength = logs.length;
                
                if (autoScroll) {
                    terminal.scrollTop = terminal.scrollHeight;
                }
            }
        } catch (e) { console.error(e); }
    }

    async function sendCmd(cmd) {
        if (!cmd) return;
        try {
            await fetch('/api/send', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({cmd: cmd})
            });
            autoScroll = true; // Snap back to bottom on send
            fetchLogs(); // Instant refresh
        } catch (e) { console.error(e); }
    }

    function submitCmd() {
        const val = cmdInput.value;
        if (val) {
            sendCmd(val);
            cmdInput.value = '';
        }
    }
    
    async function clearLogs() {
        await fetch('/api/clear', {method: 'POST'});
        lastLogLength = 0;
        terminal.innerHTML = '';
    }

    // Enter key support
    cmdInput.addEventListener('keypress', function (e) {
        if (e.key === 'Enter') submitCmd();
    });

    // Polling loops
    setInterval(updateStatus, 1000); // 1s status
    setInterval(fetchLogs, 100);     // 100ms logs (fast feel)

</script>
</body>
</html>
"""

if __name__ == '__main__':
    try:
        app.run(host=HOST, port=PORT, debug=False, threaded=True)
    except KeyboardInterrupt:
        print("Shutting down...")