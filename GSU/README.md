# MissionControlApp - Dynamic RS-485 Dashboard Framework

MissionControlApp is a configuration-driven, Flask-based framework designed for rapidly developing web dashboards for embedded systems communicating over RS-485.

Instead of hardcoding frontend components, developers write a single Python script (`main.py`) to define telemetry parsers, telecommand packers, and UI elements. The framework automatically provisions the required REST APIs, state management, and a Bootstrap/Chart.js Single Page Application (SPA).

## 📂 Project Structure

To use the framework, your project directory should look like this:

```plaintext
project_root/
├── rs485_app_lib.py    # The core RS-485 serial communication library
├── mission_control.py  # The dynamic Flask wrapper (this framework)
├── templates/
│   └── index.html      # The generic Single Page Application (SPA)
└── main.py             # Your vehicle-specific implementation (You write this!)
```

## 🚀 Quick Start

### 1. Initialization

In your `main.py`, import the library and initialize the app with your default connection properties.

```python
from mission_control import MissionControlApp

# Initialize the framework and set network addresses
mc = MissionControlApp(
    title="Phoenix OBC Dashboard",
    host_addr=0xF0,      # ADDR_GSE from phoenix_icd.h
    broadcast_addr=0x00, # The network broadcast address
    target_addr=1,       # Default destination address
    port="/dev/ttyOBC",  # Default COM port
    baud=115200          # Default baud rate
)
```

### 2. Defining Telemetry (TM)

To receive and display data, you must define a Parser and register the telemetry packet. The framework will automatically generate the UI cards and live charts.

```python
import struct

# 1. Write a parser function that takes raw bytes and returns a dictionary
def parse_power_system(raw_bytes):
    if len(raw_bytes) < 4: return {}
    v_batt, i_batt = struct.unpack(">HH", raw_bytes[:4])
    return {
        "voltage": v_batt / 1000.0, 
        "current": i_batt / 1000.0
    }

# 2. Register the telemetry packet with the framework
mc.add_telemetry(
    tm_id=5, 
    name="EPS Status", 
    parser=parse_power_system,
    
    # Automatically generate a Display Card showing exact values
    displays=[
        {"key": "voltage", "label": "Battery Voltage (V)"},
        {"key": "current", "label": "Draw Current (A)"}
    ],
    
    # Automatically generate a real-time Chart.js plot
    plots=[
        {
            "id": "eps_plot", 
            "title": "Power Draw", 
            "keys": ["voltage", "current"], 
            "colors": ["#0dcaf0", "#ef476f"]
        }
    ]
)
```

### 3. Defining Telecommands (TC)

To send commands to the vehicle, define a Packer function and register the telecommand. The framework will generate the input forms (dropdowns, checkboxes, text fields) and handle the data transmission.

```python
# 1. Write a packer function that takes a dictionary of UI inputs and returns bytes
def pack_gnc_mode(kwargs):
    # kwargs maps directly to the "name" keys in your fields list below
    mode_val = int(kwargs['mode_select'])
    return bytes([mode_val])

# 2. Register the telecommand with the framework
mc.add_telecommand(
    tc_id=10,
    name="Set ADCS Mode",
    group="Flight Control", # Groups commands together in the UI
    
    # Define the UI inputs you want the operator to see
    fields=[
        {
            "name": "mode_select", 
            "label": "Operating Mode", 
            "type": "select", 
            "options": {0: "Idle", 1: "BDot Detumble", 2: "Y-Spin"}
        }
    ],
    packer=pack_gnc_mode
)
```

### 4. Running the Application

At the bottom of `main.py`, run the server.

```python
if __name__ == '__main__':
    mc.run(host='0.0.0.0', port=5000, debug=True)
```

Navigate to `http://localhost:5000` in your browser. Ensure your serial port and baud rate in the left-hand connection pane are correct, and click **Connect**.

---

## 📖 API Reference

### `MissionControlApp`
Creates the core application instance.
* `title` (str): The name of your dashboard, displayed in the sidebar.
* `host_addr` (int): The RS-485 address of the ground station / host PC.
* `broadcast_addr` (int): The RS-485 network broadcast address.
* `target_addr` (int): The default RS-485 destination address for the UI.
* `port` (str): The default serial port (e.g., `/dev/ttyACM0` or `COM4`).
* `baud` (int): The default serial baud rate.

### `add_telemetry(tm_id, name, parser, displays=None, plots=None)`
Registers an incoming data packet.
* `tm_id` (int): The unique ID of the telemetry packet (matches the embedded firmware).
* `name` (str): Human-readable name for the UI.
* `parser` (callable): A function `func(bytes) -> dict`. It must handle raw payload bytes and return a dictionary of scaled, ready-to-display values.
* `displays` (list of dicts, optional): Creates a text-based readout card.
    * *Format:* `[{"key": "dict_key", "label": "UI Label"}]`
* `plots` (list of dicts, optional): Creates a scrolling line chart.
    * *Format:* `[{"id": "unique_id", "title": "Chart Title", "keys": ["dict_key1", "dict_key2"], "colors": ["#hex1", "#hex2"]}]`

### `add_telecommand(tc_id, name, group="General", fields=None, packer=None)`
Registers an outgoing command.
* `tc_id` (int): The unique command ID.
* `name` (str): Human-readable name for the UI button.
* `group` (str): Commands with the same group name are clustered together in the same UI card.
* `packer` (callable): A function `func(kwargs) -> bytes`. It receives the dictionary of user inputs (keyed by the field names) and must return the raw byte payload to transmit.
* `fields` (list of dicts, optional): Defines the UI inputs. The framework supports the following interface `"type"` values:
    * **`number`**: Generates a numeric input field. 
        * *Format:* `{"name": "var", "label": "Label", "type": "number"}`
    * **`text`**: Generates a standard text input field for strings. 
        * *Format:* `{"name": "var", "label": "Label", "type": "text"}`
    * **`bool`**: Generates a checkbox. Passes `True` or `False` to your packer.
        * *Format:* `{"name": "var", "label": "Label", "type": "bool"}`
    * **`select`**: Generates a drop-down menu. Requires an `"options"` dictionary mapping the returned values to display strings.
        * *Format:* `{"name": "var", "label": "Label", "type": "select", "options": {"0": "Opt 1", "1": "Opt 2"}}`

---

## ⚙️ Advanced UI Features

### 1. Command Without Payload (Simple Click Buttons)

If you have a telecommand that requires no payload (e.g., a simple "Deploy Boom" or "Reboot" trigger), you can omit the `fields` and `packer` entirely:

```python
mc.add_telecommand(
    tc_id=99,
    name="Reboot OBC",
    group="System"
)
```

The UI will generate a simple standalone button that transmits a packet with an empty payload.

### 2. Grouping Multiple Checkboxes

Because the framework groups all `fields` for a single telecommand into one card, you can easily create configuration panels (like sensor or actuator toggles) by combining multiple `bool` fields:

```python
mc.add_telecommand(
    tc_id=5,
    name="Apply Sensors",
    group="SENSORS",
    fields=[
        {"name": "en_mag", "label": "Magnetometer", "type": "bool"},
        {"name": "en_gyro", "label": "Gyroscope", "type": "bool"},
        {"name": "en_css", "label": "Sun Sensors", "type": "bool"}
    ],
    # Pack the 3 booleans into a single byte using bit-shifting
    packer=lambda kwargs: bytes([
        (1 if kwargs["en_mag"] else 0) | 
        ((1 if kwargs["en_gyro"] else 0) << 1) | 
        ((1 if kwargs["en_css"] else 0) << 2)
    ])
)
```
This will render a clean card with three stacked checkboxes and a single "Apply Sensors" button at the bottom.