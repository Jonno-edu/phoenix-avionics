import struct
import threading
import json
import io
import csv
import sys
from datetime import datetime
from collections import deque
from pathlib import Path
from queue import Queue, Empty
from flask import Flask, render_template, jsonify, request, Response
import logging
logging.getLogger('werkzeug').setLevel(logging.ERROR)
import rs485_app_lib as lib

# --- Auto-import generated nORB dictionary (written here by CMake build) ---
# gsu_norb_dict.py is regenerated into this directory every OBC firmware build
# via the --gsu-outdir flag passed to tools/generate_norb_topics.py.
_GSU_DIR = Path(__file__).parent
sys.path.insert(0, str(_GSU_DIR))
try:
    import gsu_norb_dict
except ImportError:
    print(f"[WARN] Could not import gsu_norb_dict from {_GSU_DIR}. Build OBC firmware first.")
    gsu_norb_dict = None

# ICD constants mirrored from phoenix_icd.h
_TC_OBC_NORB_SUBSCRIBE = 0x0C
_EVT_OBC_NORB_STREAM   = lib.EVENT_OBC_NORB_STREAM   # 0x02

class MissionControlApp:
    def __init__(self, title="Mission Control Dashboard", host_addr=240, broadcast_addr=0, target_addr=2, port="/dev/ttyACM0", baud=115200):
        self.title = title
        self.host_addr = host_addr             
        self.broadcast_addr = broadcast_addr   
        self.app = Flask(__name__)
        self.interface = None
        self.link = None
        self.connected = False
        self.lock = threading.Lock()
        
        # State & History
        self.latest_data = {}
        self.raw_log = deque(maxlen=1000)
        self.history = {} 
        
        # Definitions
        self.tm_defs = {}   
        self.tc_defs = {}   

        # nORB streaming — keyed by numeric topic_id
        self.norb_defs = {}           # topic_id -> {name, [fmt, fields]}
        self.norb_stream_clients = {} # topic_id -> list[Queue]

        self.ui_config = {
            "title": self.title,
            "target_addr": target_addr,
            "port": port,
            "baud": baud,
            "display_cards": [],
            "command_cards": [],
            "plots": [],
            "norb_topics": [],         # auto-populated from gsu_norb_dict
        }

        # Auto-populate nORB definitions from the generated dictionary
        self._load_norb_from_dict()

        self._setup_routes()

    # --- INTERNAL ENGINE ---

    def _load_norb_from_dict(self) -> None:
        """Auto-populate norb_defs and UI config from the auto-generated gsu_norb_dict.
        Called once at startup. Topics added here do NOT require add_norb_topic() calls."""
        if not gsu_norb_dict:
            return
        for topic_id, topic_name in gsu_norb_dict.TOPIC_NAMES.items():
            if topic_id not in self.norb_defs:
                self.norb_defs[topic_id] = {"name": topic_name}
                self.norb_stream_clients[topic_id] = []
                self.ui_config["norb_topics"].append({"id": topic_id, "name": topic_name})
        print(f"[nORB] Auto-loaded {len(gsu_norb_dict.TOPIC_NAMES)} topics from gsu_norb_dict.")

    def _auto_subscribe_norb(self, target_addr: int, rate_ms: int = 50) -> None:
        """Send TC_OBC_NORB_SUBSCRIBE for every known topic immediately after OBC connects."""
        if not gsu_norb_dict or not self.connected:
            return
        print("[nORB] Auto-subscribing to all topics...")
        for topic_id, topic_name in gsu_norb_dict.TOPIC_NAMES.items():
            payload = struct.pack('<BH', topic_id, rate_ms)
            self.link.send_telecommand(target_addr, _TC_OBC_NORB_SUBSCRIBE, payload)
            print(f"  -> Subscribed to {topic_name} (ID: {topic_id}) at {rate_ms} ms")

    def add_norb_topic(self, topic_id, name, struct_fmt, struct_fields):
        """
        Manually register a nORB topic (legacy / override path).
        Prefer dropping a .msg file and rebuilding OBC — gsu_norb_dict is
        imported automatically and add_norb_topic() calls are no longer needed.
        """
        self.norb_defs[topic_id] = {
            "name":   name,
            "fmt":    struct_fmt,
            "fields": struct_fields,
        }
        self.norb_stream_clients.setdefault(topic_id, [])
        # Only append to UI list if not already present from the dict
        if not any(t["id"] == topic_id for t in self.ui_config["norb_topics"]):
            self.ui_config["norb_topics"].append({"id": topic_id, "name": name})

    def add_telecommand(self, tc_id, name, group="General", fields=None, packer=None):
        """
        Register a Telecommand.
        fields: UI inputs e.g., [{"name": "mode", "label": "Mode", "type": "select", "options": {0: "Idle", 1: "BDot"}}]
        packer: A function that takes kwargs and returns bytes.
        """
        self.tc_defs[tc_id] = {"name": name, "packer": packer}
        # Find or create command card group
        group_card = next((g for g in self.ui_config["command_cards"] if g["group"] == group), None)
        if not group_card:
            group_card = {"group": group, "commands": []}
            self.ui_config["command_cards"].append(group_card)
        
        group_card["commands"].append({"id": tc_id, "name": name, "fields": fields or []})

    def run(self, host='0.0.0.0', port=5000, debug=True):
        # threaded=True is required for Server-Sent Events (SSE) to work;
        # each SSE client holds a long-lived response in its own thread.
        self.app.run(host=host, port=port, debug=debug, use_reloader=False, threaded=True)

    # --- INTERNAL ENGINE ---

    def _update_norb_stream(self, pkt):
        """Called when an EVENT_OBC_NORB_STREAM packet arrives from the OBC."""
        raw = pkt["data"]
        ts_rcv = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        if len(raw) < 1:
            return
        topic_id = raw[0]
        raw_data = raw[1:]
        data = None

        # Preferred path: use auto-generated unpack function from gsu_norb_dict
        if gsu_norb_dict and topic_id in gsu_norb_dict.UNPACK_FNS:
            try:
                unpacked_data = gsu_norb_dict.UNPACK_FNS[topic_id](raw_data)
                topic_name = gsu_norb_dict.TOPIC_NAMES.get(topic_id, f"topic_{topic_id}")
                
                # Update global state for System Overview
                with self.lock:
                    self.latest_data[topic_id] = unpacked_data
                    
                    # --- DELETED the self.raw_log.appendleft() call here! ---
                    # We no longer send nORB packets to the raw terminal log.
                
                # Send to SSE clients in expected format
                sse_message = {
                    "topic_id": topic_id,
                    "topic_name": topic_name,
                    "payload": unpacked_data
                }
                
                clients = self.norb_stream_clients.get(topic_id, [])
                for q in list(clients):
                    try:
                        q.put_nowait(sse_message)
                    except Exception:
                        pass  # drop if client queue is full
                return
                
            except Exception as e:
                print(f"nORB dict parse error topic {topic_id}: {e}")
                return
        # Fallback path: use a manually registered struct definition
        elif topic_id in self.norb_defs and "fmt" in self.norb_defs[topic_id]:
            defn = self.norb_defs[topic_id]
            try:
                values = struct.unpack(defn["fmt"], raw_data)
                unpacked_data = dict(zip(defn["fields"], values))
                topic_name = defn["name"]
                
                # Update global state for System Overview
                with self.lock:
                    self.latest_data[topic_id] = unpacked_data
                    
                    # --- DELETED the self.raw_log.appendleft() call here! ---
                    # We no longer send nORB packets to the raw terminal log.
                
                # Send to SSE clients in expected format
                sse_message = {
                    "topic_id": topic_id,
                    "topic_name": topic_name,
                    "payload": unpacked_data
                }
                
                clients = self.norb_stream_clients.get(topic_id, [])
                for q in list(clients):
                    try:
                        q.put_nowait(sse_message)
                    except Exception:
                        pass  # drop if client queue is full
                return
                
            except Exception as e:
                print(f"nORB manual parse error topic {topic_id}: {e}")
                return
        else:
            print(f"[nORB] Warning: Received unknown topic ID {topic_id}")
            return

    def _setup_routes(self):
        @self.app.route('/')
        def index(): return render_template('index.html')

        @self.app.route('/api/config', methods=['GET'])
        def get_config(): return jsonify(self.ui_config)

        @self.app.route('/norb_listener')
        def norb_listener(): return render_template('norb_listener.html')

        @self.app.route('/api/norb/subscribe', methods=['POST'])
        def norb_subscribe():
            if not self.connected:
                return jsonify(success=False, error="Not connected")
            data = request.json
            topic_id = int(data.get('topic_id'))
            rate_ms  = int(data.get('rate_ms', 100))
            target   = int(data.get('target', self.ui_config['target_addr']))
            if topic_id not in self.norb_defs:
                return jsonify(success=False, error="Unknown topic")
            # NorbSubscribePayload_t: uint8 topic_id, uint16 rate_ms (little-endian)
            payload = struct.pack('<BH', topic_id, rate_ms)
            self.link.send_telecommand(target, _TC_OBC_NORB_SUBSCRIBE, payload)
            return jsonify(success=True)

        @self.app.route('/api/norb/stream')
        def norb_stream():
            topic_arg = request.args.get('topic_id', '-1')
            if topic_arg == 'all':
                topic_id = None
            else:
                topic_id = int(topic_arg)
            if topic_id is not None and topic_id not in self.norb_defs:
                return jsonify(error="Unknown topic"), 400

            client_queue = Queue(maxsize=200)
            with self.lock:
                if topic_id is None:
                    for tid in self.norb_defs:
                        self.norb_stream_clients[tid].append(client_queue)
                else:
                    self.norb_stream_clients[topic_id].append(client_queue)

            def generate():
                try:
                    while True:
                        try:
                            item = client_queue.get(timeout=1.0)
                            yield f"data: {json.dumps(item)}\n\n"
                        except Empty:
                            yield ": keepalive\n\n"  # prevent proxy timeouts
                except GeneratorExit:
                    pass
                finally:
                    with self.lock:
                        if topic_id is None:
                            for tid in self.norb_defs:
                                try:
                                    self.norb_stream_clients[tid].remove(client_queue)
                                except ValueError:
                                    pass
                        else:
                            try:
                                self.norb_stream_clients[topic_id].remove(client_queue)
                            except ValueError:
                                pass

            return Response(
                generate(),
                mimetype='text/event-stream',
                headers={
                    'Cache-Control':    'no-cache',
                    'X-Accel-Buffering': 'no',
                }
            )

        @self.app.route('/api/status', methods=['GET'])
        def get_status():
            with self.lock: return jsonify({"connected": self.connected, "data": self.latest_data, "logs": list(self.raw_log)[:50]})

        @self.app.route('/api/history', methods=['GET'])
        def get_history():
            # A generic filter that checks query params against plot IDs
            res = {}
            with self.lock:
                for plot_id, dataset in self.history.items():
                    cursor = request.args.get(f'since_{plot_id}')
                    if not dataset: 
                        res[plot_id] = []
                        continue
                    if not cursor: 
                        res[plot_id] = list(dataset)
                        continue
                    try:
                        # Numeric vs String time check
                        try:
                            float(dataset[-1]["Time"])
                            res[plot_id] = [x for x in dataset if float(x["Time"]) > float(cursor)]
                        except ValueError:
                            res[plot_id] = [x for x in dataset if x["Time"] > cursor]
                    except: res[plot_id] = list(dataset)
            return jsonify(res)

        @self.app.route('/api/command', methods=['POST'])
        def handle_command():
            if not self.connected: return jsonify(success=False, error="Not connected")
            data = request.json
            target, cmd_id = int(data.get('target', 2)), int(data.get('id'))
            is_req = data.get('is_request', False)
            
            if is_req:
                self.link.request_telemetry(target, cmd_id)
                self.raw_log.appendleft({"Time": datetime.now().strftime("%H:%M:%S.%f")[:-3], "Dir": "TX", "Type": "REQ", "ID": cmd_id, "Hex": ""})
            else:
                payload = b""
                if 'payload_raw' in data:
                    payload = bytes(data['payload_raw'])
                elif 'kwargs' in data and cmd_id in self.tc_defs:
                    # Dynamically pack arguments using the developer's registered lambda
                    packer = self.tc_defs[cmd_id]["packer"]
                    if packer: payload = packer(data['kwargs'])
                
                self.link.send_telecommand(target, cmd_id, payload)
                self.raw_log.appendleft({"Time": datetime.now().strftime("%H:%M:%S.%f")[:-3], "Dir": "TX", "Type": "TCM", "ID": cmd_id, "Hex": payload.hex(" ").upper()})
            return jsonify(success=True)

        @self.app.route('/api/connect', methods=['POST'])
        def connect():
            args = request.json
            try:
                # Use the port and baud submitted by the UI, fallback to defaults
                port = args.get('port') or self.ui_config.get('port', '/dev/ttyACM0')
                baud = int(args.get('baud')) if args.get('baud') else self.ui_config.get('baud', 115200)
                
                self.interface = lib.RS485Interface(port=port, baudrate=baud)
                if self.interface.connect():
                    # USE self.host_addr and self.broadcast_addr here!
                    self.link = lib.OBCLink(self.interface, host_addr=self.host_addr, broadcast_addr=self.broadcast_addr)
                    
                    # Register nORB stream handler (EVENT_OBC_NORB_STREAM = 0x02)
                    self.link.on_event(_EVT_OBC_NORB_STREAM, lambda p: self._update_norb_stream(p))

                    self.link.on_event(lib.EVENT_COMMON_LOG, lambda p: self.raw_log.appendleft({"Time": datetime.now().strftime("%H:%M:%S.%f")[:-3], "Dir": "RX", "Type": "EVENT", "ID": lib.EVENT_COMMON_LOG, "Hex": f"[ASCII] {p['data'].decode('ascii','replace')}"}))
                    for i in range(1, 32):
                        self.link.on_tc_ack(i, lambda p: self.raw_log.appendleft({"Time": datetime.now().strftime("%H:%M:%S.%f")[:-3], "Dir": "RX", "Type": "ACK", "ID": p['desc']&0x1F, "Hex": "OK"}))

                    self.link.start_listening()
                    self.connected = True

                    # Auto-subscribe to all nORB topics from the generated dictionary
                    target = int(args.get('target', self.ui_config['target_addr']))
                    self._auto_subscribe_norb(target)

                    return jsonify(success=True)
            except Exception as e: 
                print(f"Connect route error: {e}")
                return jsonify(success=False, error=str(e))
            return jsonify(success=False, error="Serial port could not be opened")

        @self.app.route('/api/disconnect', methods=['POST'])
        def disconnect():
            if self.link: self.link.stop_listening()
            if self.interface: self.interface.disconnect()
            self.connected = False
            return jsonify(success=True)
            
        @self.app.route('/api/clear_log', methods=['POST'])
        def clear_log():
            with self.lock:
                self.raw_log.clear()
                for k in self.history: self.history[k].clear()
                self.latest_data.clear()
            return jsonify(success=True)
        
        @self.app.route('/api/download_log', methods=['GET'])
        def download_log():
            with self.lock:
                txt = "TIMESTAMP       DIR   TYPE     ID    DATA\n" + "-"*60 + "\n"
                # raw_log is a deque where left is newest. We reverse it for chronological order.
                for l in reversed(self.raw_log):
                    txt += f"{l.get('Time',''):<15} {l.get('Dir',''):<5} {l.get('Type',''):<8} {l.get('ID',''):<5} {l.get('Hex','')}\n"
            return Response(txt, mimetype="text/plain", headers={"Content-disposition": "attachment; filename=mission_log.txt"})

        @self.app.route('/api/download_data', methods=['GET'])
        def download_data():
            with self.lock:
                si = io.StringIO()
                cw = csv.writer(si)
                
                # 1. Dynamically discover all unique data keys across all registered plots
                all_columns = ["Plot_ID", "Time"]
                for plot_id, dataset in self.history.items():
                    for entry in dataset:
                        for key in entry.keys():
                            if key not in all_columns:
                                all_columns.append(key)
                
                # 2. Write the header row
                cw.writerow(all_columns)
                
                # 3. Write the data, filling missing columns with empty strings
                for plot_id, dataset in self.history.items():
                    for entry in dataset:
                        row = [plot_id, entry.get("Time", "")]
                        for col in all_columns[2:]:
                            row.append(entry.get(col, ""))
                        cw.writerow(row)
                        
                output = si.getvalue()
            return Response(output, mimetype="text/csv", headers={"Content-disposition": "attachment; filename=telemetry_data.csv"})