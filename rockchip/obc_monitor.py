#!/usr/bin/env python3
"""
OBC Serial Monitor - Streamlit Web Interface
Monitors /dev/ttyOBC and allows sending messages over serial.
Styled to match Phoenix EPS Control aesthetic.
"""

import streamlit as st
import serial
import time
import re
from collections import deque
from datetime import datetime

# Configuration
SERIAL_PORT = "/dev/ttyOBC"
BAUD_RATE = 115200
MAX_LOG_LINES = 500

# Page configuration - MUST be first Streamlit command
st.set_page_config(
    page_title="OBC Monitor",
    page_icon="📡",
    layout="wide",
    initial_sidebar_state="collapsed"
)

# Custom CSS matching ESP32 dashboard with swapped colors
st.markdown("""
<style>
    /* Import for consistent font */
    @import url('https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&display=swap');
    
    /* Root variables - SWAPPED: Teal is now primary, Purple is secondary */
    :root {
        --bg: #121212;
        --card: #1E1E1E;
        --text: #E0E0E0;
        --primary: #03DAC6;
        --secondary: #BB86FC;
        --success: #03DAC6;
        --danger: #CF6679;
        --active: #00FF00;
        --inactive: #333333;
        --input-bg: #2C2C2C;
    }
    
    /* Main app background */
    .stApp {
        background-color: var(--bg);
        color: var(--text);
        font-family: 'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
    }
    
    /* Hide Streamlit branding */
    #MainMenu {visibility: hidden;}
    footer {visibility: hidden;}
    header {visibility: hidden;}
    
    /* Main container */
    .main .block-container {
        padding-top: 2rem;
        padding-bottom: 2rem;
        max-width: 1200px;
    }
    
    /* Title styling */
    .main-title {
        text-align: center;
        margin-bottom: 1.5rem;
    }
    .main-title h1 {
        color: var(--primary);
        font-size: 2.2rem;
        font-weight: 700;
        letter-spacing: 2px;
        margin: 0;
    }
    .main-title .subtitle {
        color: var(--text);
        opacity: 0.5;
        font-size: 0.9rem;
        letter-spacing: 1px;
    }
    
    /* Card styling */
    .card {
        background: var(--card);
        padding: 20px;
        border-radius: 16px;
        box-shadow: 0 4px 20px rgba(0,0,0,0.4);
        margin-bottom: 1rem;
    }
    .card-header {
        font-size: 0.95rem;
        font-weight: 600;
        text-transform: uppercase;
        letter-spacing: 1px;
        color: var(--text);
        opacity: 0.7;
        border-bottom: 1px solid #333;
        padding-bottom: 10px;
        margin-bottom: 15px;
    }
    
    /* Stat items */
    .stat-grid {
        display: grid;
        grid-template-columns: 1fr 1fr;
        gap: 12px;
    }
    .stat-item {
        background: var(--input-bg);
        padding: 15px;
        border-radius: 8px;
        text-align: center;
    }
    .stat-label {
        font-size: 0.75rem;
        color: var(--text);
        opacity: 0.6;
        text-transform: uppercase;
        letter-spacing: 0.5px;
        margin-bottom: 5px;
    }
    .stat-value {
        font-size: 1.6rem;
        font-weight: 700;
        color: var(--text);
        font-variant-numeric: tabular-nums;
    }
    .stat-unit {
        font-size: 0.85rem;
        opacity: 0.6;
        margin-left: 2px;
    }
    
    /* Status indicators */
    .status-connected {
        color: var(--success);
        display: flex;
        align-items: center;
        gap: 8px;
    }
    .status-disconnected {
        color: var(--danger);
        display: flex;
        align-items: center;
        gap: 8px;
    }
    .led {
        width: 10px;
        height: 10px;
        border-radius: 50%;
        display: inline-block;
    }
    .led-on {
        background-color: var(--active);
        box-shadow: 0 0 8px var(--active);
    }
    .led-off {
        background-color: var(--inactive);
    }
    
    /* Log display */
    .log-container {
        background: #0a0a0a;
        border: 1px solid #333;
        border-radius: 8px;
        padding: 12px;
        font-family: 'Consolas', 'Monaco', monospace;
        font-size: 0.85rem;
        line-height: 1.5;
        height: 450px; /* Fixed height for consistency */
        overflow-y: auto;
        color: var(--text);
        display: flex;
        flex-direction: column;
    }
    .log-rx { color: var(--primary); }
    .log-tx { color: var(--secondary); }
    .log-timestamp { color: #666; margin-right: 8px; }
    .log-error { color: var(--danger); }
    
    /* Button styling */
    .stButton > button {
        background-color: var(--primary) !important;
        color: #000 !important;
        border: none !important;
        border-radius: 8px !important;
        padding: 12px 20px !important;
        font-weight: 600 !important;
        text-transform: uppercase !important;
        letter-spacing: 0.5px !important;
        transition: all 0.2s !important;
        width: 100%;
    }
    .stButton > button:hover {
        opacity: 0.9 !important;
        transform: scale(0.98);
    }
    .stButton > button:active {
        transform: scale(0.96) !important;
    }
    
    /* Secondary button */
    .secondary-btn > button {
        background-color: var(--input-bg) !important;
        color: var(--text) !important;
        border: 1px solid #444 !important;
    }
    
    /* Danger button */
    .danger-btn > button {
        background-color: var(--danger) !important;
        color: white !important;
    }
    
    /* Compact grid buttons for Logging Control */
    .compact-grid .stButton > button {
        padding: 4px 8px !important;
        font-size: 0.75rem !important;
        min-height: 0px !important;
        height: auto !important;
        margin-top: 0px !important;
        background-color: var(--input-bg) !important;
        color: var(--primary) !important;
        border: 1px solid var(--primary) !important;
    }
    .compact-grid .stButton > button:hover {
        background-color: var(--primary) !important;
        color: black !important;
    }
    
    /* Input styling */
    .stTextInput > div > div > input {
        background-color: var(--input-bg) !important;
        color: var(--text) !important;
        border: 1px solid #444 !important;
        border-radius: 8px !important;
        padding: 12px !important;
    }
    .stTextInput > div > div > input:focus {
        border-color: var(--primary) !important;
        box-shadow: 0 0 0 1px var(--primary) !important;
    }
    
    /* Checkbox styling */
    .stCheckbox > label {
        color: var(--text) !important;
    }
    
    /* Slider styling */
    .stSlider > div > div > div > div {
        background-color: var(--primary) !important;
    }
    
    /* Expander styling */
    .streamlit-expanderHeader {
        background-color: var(--card) !important;
        color: var(--text) !important;
        border-radius: 8px !important;
    }
    
    /* Divider */
    hr {
        border-color: #333 !important;
        margin: 1rem 0 !important;
    }
    
    /* Quick command buttons grid */
    .cmd-grid {
        display: grid;
        grid-template-columns: 1fr 1fr;
        gap: 8px;
    }
    
    /* Scrollbar styling */
    ::-webkit-scrollbar {
        width: 8px;
        height: 8px;
    }
    ::-webkit-scrollbar-track {
        background: var(--bg);
    }
    ::-webkit-scrollbar-thumb {
        background: #444;
        border-radius: 4px;
    }
    ::-webkit-scrollbar-thumb:hover {
        background: #555;
    }
</style>
""", unsafe_allow_html=True)


# Initialize session state
if "serial_log" not in st.session_state:
    st.session_state.serial_log = deque(maxlen=MAX_LOG_LINES)
if "serial_connection" not in st.session_state:
    st.session_state.serial_connection = None
if "connected" not in st.session_state:
    st.session_state.connected = False
if "rx_count" not in st.session_state:
    st.session_state.rx_count = 0
if "tx_count" not in st.session_state:
    st.session_state.tx_count = 0
if "start_time" not in st.session_state:
    st.session_state.start_time = time.time()


def get_serial_connection():
    """Get or create serial connection."""
    try:
        if st.session_state.serial_connection is None or not st.session_state.serial_connection.is_open:
            ser = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUD_RATE,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            st.session_state.serial_connection = ser
            st.session_state.connected = True
        return st.session_state.serial_connection
    except serial.SerialException:
        st.session_state.connected = False
        st.session_state.serial_connection = None
        return None

def clean_text(text):
    """
    Strips ANSI escape codes and other control characters that cause
    logs to look like corrupted ASCII or garbage.
    """
    # Regex to remove ANSI escape sequences (colors, cursor movements)
    ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
    cleaned = ansi_escape.sub('', text)
    # Remove non-printable characters except newlines/tabs
    cleaned = "".join(ch for ch in cleaned if ch.isprintable() or ch in '\n\t')
    return cleaned

def read_serial_data(ser):
    """Read available data from serial port."""
    data_lines = []
    try:
        # Check if there is data waiting
        if ser.in_waiting > 0:
            # Read all lines currently available in the buffer
            # This prevents buffering lag where we only read 1 line per refresh
            raw_lines = ser.readlines()
            
            for line in raw_lines:
                try:
                    # Decode with replacement to handle actual byte corruption
                    decoded = line.decode("utf-8", errors="replace").strip()
                    # Clean ANSI codes
                    decoded = clean_text(decoded)
                except Exception:
                    decoded = "<binary data>"
                    
                if decoded:
                    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    data_lines.append(("RX", timestamp, decoded))
                    st.session_state.rx_count += 1
    except serial.SerialException:
        pass
    return data_lines


def send_serial_data(ser, message):
    """Send data over serial port."""
    try:
        if not message.endswith("\n"):
            message += "\n"
        ser.write(message.encode("utf-8"))
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        st.session_state.tx_count += 1
        return ("TX", timestamp, message.strip())
    except serial.SerialException as e:
        return ("ERROR", datetime.now().strftime("%H:%M:%S.%f")[:-3], f"Failed to send: {e}")


def format_log_entry(entry):
    """Format a log entry as HTML."""
    direction, timestamp, data = entry
    if direction == "RX":
        return f'<div><span class="log-timestamp">[{timestamp}]</span><span class="log-rx">RX:</span> {data}</div>'
    elif direction == "TX":
        return f'<div><span class="log-timestamp">[{timestamp}]</span><span class="log-tx">TX:</span> {data}</div>'
    else:
        return f'<div><span class="log-timestamp">[{timestamp}]</span><span class="log-error">{direction}:</span> {data}</div>'


def get_uptime():
    """Get formatted uptime."""
    elapsed = int(time.time() - st.session_state.start_time)
    hours, remainder = divmod(elapsed, 3600)
    minutes, seconds = divmod(remainder, 60)
    if hours > 0:
        return f"{hours}h {minutes}m {seconds}s"
    elif minutes > 0:
        return f"{minutes}m {seconds}s"
    else:
        return f"{seconds}s"


# Title
st.markdown("""
<div class="main-title">
    <h1>PHOENIX <span style="opacity: 0.5; font-size: 0.5em;">OBC MONITOR</span></h1>
    <div class="subtitle">Serial Terminal Interface</div>
</div>
""", unsafe_allow_html=True)

# Get serial connection
ser = get_serial_connection()

# Layout
col_main, col_side = st.columns([3, 1])

with col_side:
    # Connection Status Card
    st.markdown('<div class="card">', unsafe_allow_html=True)
    st.markdown('<div class="card-header">Connection</div>', unsafe_allow_html=True)
    
    if ser and st.session_state.connected:
        st.markdown(f'''
        <div class="status-connected">
            <span class="led led-on"></span>
            <span>Connected</span>
        </div>
        <div style="font-size: 0.8rem; color: #888; margin-top: 5px;">
            {SERIAL_PORT} @ {BAUD_RATE}
        </div>
        ''', unsafe_allow_html=True)
    else:
        st.markdown(f'''
        <div class="status-disconnected">
            <span class="led led-off"></span>
            <span>Disconnected</span>
        </div>
        <div style="font-size: 0.8rem; color: #888; margin-top: 5px;">
            {SERIAL_PORT} unavailable
        </div>
        ''', unsafe_allow_html=True)
    st.markdown('</div>', unsafe_allow_html=True)
    
    # Telemetry Card
    st.markdown(f'''
    <div class="card">
        <div class="card-header">Telemetry</div>
        <div class="stat-grid">
            <div class="stat-item">
                <div class="stat-label">RX Packets</div>
                <div class="stat-value">{st.session_state.rx_count}</div>
            </div>
            <div class="stat-item">
                <div class="stat-label">TX Packets</div>
                <div class="stat-value">{st.session_state.tx_count}</div>
            </div>
        </div>
        <div class="stat-item" style="margin-top: 12px;">
            <div class="stat-label">Uptime</div>
            <div class="stat-value" style="font-size: 1.2rem;">{get_uptime()}</div>
        </div>
    </div>
    ''', unsafe_allow_html=True)
    
    # --- LOGGING CONTROL (NEW BOX) ---
    st.markdown('<div class="card">', unsafe_allow_html=True)
    st.markdown('<div class="card-header">Logging Control</div>', unsafe_allow_html=True)
    # We use a custom CSS class .compact-grid to make these buttons smaller
    st.markdown('<div class="compact-grid">', unsafe_allow_html=True)

    # Dictionary of Display Name -> Command ID
    tasks_to_log = [
        ("Heartbeat", "heartbeat"),
        ("HIL Sensors", "hil_sensor"),
        ("RS485", "rs485"),
        ("Sensors", "sensors"),
        ("Telemetry", "telemetry")
    ]

    for label, task_id in tasks_to_log:
        # Create 3 columns: Label | ON | OFF
        lc1, lc2, lc3 = st.columns([2, 1, 1])
        with lc1:
            st.markdown(f'<div style="padding-top: 6px; font-size: 0.85rem; opacity: 0.9;">{label}</div>', unsafe_allow_html=True)
        with lc2:
            if st.button("ON", key=f"log_on_{task_id}"):
                if ser:
                    # Assumed Command: "log <taskname> on"
                    msg = f"log {task_id} on"
                    tx_log = send_serial_data(ser, msg)
                    st.session_state.serial_log.append(tx_log)
        with lc3:
            if st.button("OFF", key=f"log_off_{task_id}"):
                if ser:
                    # Assumed Command: "log <taskname> off"
                    msg = f"log {task_id} off"
                    tx_log = send_serial_data(ser, msg)
                    st.session_state.serial_log.append(tx_log)

    st.markdown('</div>', unsafe_allow_html=True) # End compact-grid
    st.markdown('</div>', unsafe_allow_html=True) # End card
    
    # Send Command Card
    st.markdown('<div class="card">', unsafe_allow_html=True)
    st.markdown('<div class="card-header">Send Command</div>', unsafe_allow_html=True)
    
    message_to_send = st.text_input(
        "Command",
        placeholder="Type message...",
        label_visibility="collapsed",
        key="cmd_input"
    )
    
    if st.button("SEND", key="send_btn"):
        if message_to_send and ser:
            tx_log = send_serial_data(ser, message_to_send)
            st.session_state.serial_log.append(tx_log)
    
    st.markdown('</div>', unsafe_allow_html=True)
    
    # Quick Commands Card
    st.markdown('<div class="card">', unsafe_allow_html=True)
    st.markdown('<div class="card-header">Quick Commands</div>', unsafe_allow_html=True)
    
    # Example quick command buttons - customize these later
    cmd_col1, cmd_col2 = st.columns(2)
    with cmd_col1:
        if st.button("STATUS", key="cmd_status"):
            if ser:
                tx_log = send_serial_data(ser, "STATUS")
                st.session_state.serial_log.append(tx_log)
    with cmd_col2:
        if st.button("PING", key="cmd_ping"):
            if ser:
                tx_log = send_serial_data(ser, "PING")
                st.session_state.serial_log.append(tx_log)
    
    cmd_col3, cmd_col4 = st.columns(2)
    with cmd_col3:
        if st.button("RESET", key="cmd_reset"):
            if ser:
                tx_log = send_serial_data(ser, "RESET")
                st.session_state.serial_log.append(tx_log)
    with cmd_col4:
        if st.button("HELP", key="cmd_help"):
            if ser:
                tx_log = send_serial_data(ser, "HELP")
                st.session_state.serial_log.append(tx_log)
    
    st.markdown('</div>', unsafe_allow_html=True)
    
    # Controls Card
    st.markdown('<div class="card">', unsafe_allow_html=True)
    st.markdown('<div class="card-header">Controls</div>', unsafe_allow_html=True)
    
    st.markdown('<div class="secondary-btn">', unsafe_allow_html=True)
    if st.button("CLEAR LOG", key="clear_btn"):
        st.session_state.serial_log.clear()
        st.session_state.rx_count = 0
        st.session_state.tx_count = 0
    st.markdown('</div>', unsafe_allow_html=True)
    
    auto_refresh = st.checkbox("Auto-refresh", value=True, key="auto_refresh")
    refresh_rate = st.slider("Refresh (s)", 0.5, 5.0, 1.0, 0.5, key="refresh_rate")
    
    st.markdown('</div>', unsafe_allow_html=True)


with col_main:
    # Serial Log Card
    st.markdown('<div class="card">', unsafe_allow_html=True)
    st.markdown('<div class="card-header">Serial Log</div>', unsafe_allow_html=True)
    
    # Read new data if connected
    if ser and st.session_state.connected:
        new_lines = read_serial_data(ser)
        for line in new_lines:
            st.session_state.serial_log.append(line)
    
    # Display log with Auto-Scroll (To Top) ID and Script
    if st.session_state.serial_log:
        # 1. Reverse the log list so Newest is First
        reversed_logs = reversed(st.session_state.serial_log)
        
        # 2. Join into HTML
        log_html = "".join([format_log_entry(entry) for entry in reversed_logs])
        
        # 3. Render with Script that snaps scrollTop to 0 (TOP)
        st.markdown(f'''
        <div id="serial-log-container" class="log-container">
            {log_html}
        </div>
        <script>
            var logDiv = document.getElementById("serial-log-container");
            if (logDiv) {{
                logDiv.scrollTop = 0;
            }}
        </script>
        ''', unsafe_allow_html=True)
    else:
        st.markdown('''
        <div class="log-container" style="text-align: center; color: #666; padding: 40px;">
            Waiting for data...
        </div>
        ''', unsafe_allow_html=True)
    
    st.markdown('</div>', unsafe_allow_html=True)


# Auto-refresh
if auto_refresh:
    time.sleep(refresh_rate)
    st.rerun()