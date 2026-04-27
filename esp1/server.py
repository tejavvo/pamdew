from http.server import HTTPServer, BaseHTTPRequestHandler
import csv
from datetime import datetime, timezone
import json
import os

LOG_DIR = os.environ.get("SENSOR_LOG_DIR", "logs")
JSONL_PATH = os.path.join(LOG_DIR, "sensor_data.jsonl")
CSV_PATH = os.path.join(LOG_DIR, "sensor_data.csv")
VALID_PATHS = {"/api/sensor_data", "/data"}

CSV_FIELDS = [
    "timestamp",
    "bpm",
    "mean_rr_ms",
    "sdnn_ms",
    "rmssd_ms",
    "r_peaks",
    "ppg_valid",
    "ppg_bpm",
    "ppg_spo2",
    "ir_ac",
    "spo2_red",
    "spo2_ir",
    "accel_x",
    "accel_y",
    "accel_z",
    "ecg_leads",
    "gsr",
    "temp",
]


def append_logs(data):
    os.makedirs(LOG_DIR, exist_ok=True)
    timestamp = datetime.now(timezone.utc).isoformat()

    with open(JSONL_PATH, "a", encoding="utf-8") as f:
        f.write(json.dumps({"timestamp": timestamp, "data": data}) + "\n")

    accel = data.get("accel") or [None, None, None]
    row = {
        "timestamp": timestamp,
        "bpm": data.get("bpm"),
        "mean_rr_ms": data.get("mean_rr_ms"),
        "sdnn_ms": data.get("sdnn_ms"),
        "rmssd_ms": data.get("rmssd_ms"),
        "r_peaks": data.get("r_peaks"),
        "ppg_valid": data.get("ppg_valid"),
        "ppg_bpm": data.get("ppg_bpm"),
        "ppg_spo2": data.get("ppg_spo2"),
        "ir_ac": data.get("ir_ac"),
        "spo2_red": data.get("spo2_red"),
        "spo2_ir": data.get("spo2_ir"),
        "accel_x": accel[0] if len(accel) > 0 else None,
        "accel_y": accel[1] if len(accel) > 1 else None,
        "accel_z": accel[2] if len(accel) > 2 else None,
        "ecg_leads": data.get("ecg_leads"),
        "gsr": data.get("gsr"),
        "temp": data.get("temp"),
    }
    write_header = not os.path.exists(CSV_PATH)
    with open(CSV_PATH, "a", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        if write_header:
            writer.writeheader()
        writer.writerow(row)

class SimpleRESTHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        # Accept both the original dashboard path and the firmware default path.
        if self.path in VALID_PATHS:
            # Get the length of the data
            content_length = int(self.headers.get('Content-Length', 0))
            # Read the data
            post_data = self.rfile.read(content_length)
            
            try:
                # Parse and print the JSON
                data = json.loads(post_data.decode('utf-8'))
                print("\n=== Received Sensor Data ===")
                print(json.dumps(data, indent=2))
                print("============================\n")
                append_logs(data)
                
                # Send a 200 OK response back to the ESP32
                self.send_response(200)
                self.send_header('Content-type', 'application/json')
                self.end_headers()
                self.wfile.write(b'{"status": "success"}')
                
            except Exception as e:
                print(f"Error parsing JSON: {e}")
                self.send_response(400)
                self.end_headers()
        else:
            self.send_response(404)
            self.end_headers()

    # Mute the default logging so our JSON stands out
    def log_message(self, format, *args):
        pass

if __name__ == '__main__':
    port = int(os.environ.get("SENSOR_SERVER_PORT", "8080"))
    server = HTTPServer(('0.0.0.0', port), SimpleRESTHandler)
    print(f"Listening for ESP32 POST requests on port {port}...")
    print(f"Accepting paths: {', '.join(sorted(VALID_PATHS))}")
    print(f"Logging JSONL to {JSONL_PATH}")
    print(f"Logging CSV to {CSV_PATH}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down server.")
        server.server_close()
