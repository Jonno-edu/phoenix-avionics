import struct
import threading
import io
import csv
from datetime import datetime
from collections import deque
from flask import Flask, render_template, jsonify, request, Response
import rs485_app_lib as lib

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
        self.ui_config = {
            "title": self.title,
            "target_addr": target_addr, # Added
            "port": port,               # Added
            "baud": baud,               # Added
            "tm_requests": [],
            "display_cards": [],
            "command_cards": [],
            "plots": []
        }

        self._setup_routes()

    # --- API FOR DEVELOPERS ---

    def add_telemetry(self, tm_id, name, parser, displays=None, plots=None):
        """
        Register a Telemetry packet.
        displays: list of dicts e.g., [{"key": "Mag_X", "label": "Mag X", "type": "text"}]
        plots: list of dicts e.g., [{"id": "plot_mag", "title": "Magnetometer", "keys": ["Mag_X"], "colors": ["#f00"]}]
        """
        self.tm_defs[tm_id] = {"name": name, "parser": parser}
        self.ui_config["tm_requests"].append({"id": tm_id, "name": name})
        
        if displays:
            self.ui_config["display_cards"].append({"tm_id": tm_id, "name": name, "fields": displays})
            
        if plots:
            for plot in plots:
                self.ui_config["plots"].append(plot)
                self.history[plot["id"]] = deque(maxlen=5000)
                self.tm_defs[tm_id].setdefault("plots", []).append(plot["id"])

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
        self.app.run(host=host, port=port, debug=debug, use_reloader=False)

    # --- INTERNAL ENGINE ---

    def _update(self, tm_id, pkt):
        ts_rcv = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        parser = self.tm_defs[tm_id]["parser"]
        try:
            data = parser(pkt["data"])
            data["_updated"] = ts_rcv
            with self.lock:
                self.latest_data[tm_id] = data
                
                # Update history for plots associated with this TM
                if "plots" in self.tm_defs[tm_id]:
                    entry = {k: v for k, v in data.items() if isinstance(v, (int, float, str))}
                    entry["Time"] = data.get("Sat_Time", ts_rcv)
                    for plot_id in self.tm_defs[tm_id]["plots"]:
                        self.history[plot_id].append(entry)
                        
                self.raw_log.appendleft({"Time": ts_rcv, "Dir": "RX", "Type": "TM", "ID": tm_id, "Hex": pkt["data"].hex(" ").upper()})
        except Exception as e: print(f"Parse Error TM {tm_id}: {e}")

    def _setup_routes(self):
        @self.app.route('/')
        def index(): return render_template('index.html')

        @self.app.route('/api/config', methods=['GET'])
        def get_config(): return jsonify(self.ui_config)

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
                self.interface = lib.RS485Interface(port=args.get('port', '/dev/ttyACM0'), baudrate=int(args.get('baud', 9600)))
                if self.interface.connect():
                    # USE self.host_addr and self.broadcast_addr here!
                    self.link = lib.OBCLink(self.interface, host_addr=self.host_addr, broadcast_addr=self.broadcast_addr)
                    
                    for tm_id in self.tm_defs:
                        self.link.on_tm_report(tm_id, lambda p, id=tm_id: self._update(id, p))
                    
                    self.link.on_event(1, lambda p: self.raw_log.appendleft({"Time": datetime.now().strftime("%H:%M:%S.%f")[:-3], "Dir": "RX", "Type": "EVENT", "ID": 1, "Hex": f"[ASCII] {p['data'].decode('ascii','replace')}"}))
                    for i in range(1, 32): 
                        self.link.on_tc_ack(i, lambda p: self.raw_log.appendleft({"Time": datetime.now().strftime("%H:%M:%S.%f")[:-3], "Dir": "RX", "Type": "ACK", "ID": p['desc']&0x1F, "Hex": "OK"}))
                    
                    self.link.start_listening()
                    self.connected = True
                    return jsonify(success=True)
            except Exception as e: print(e)
            return jsonify(success=False)

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