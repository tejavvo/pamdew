from http.server import HTTPServer, BaseHTTPRequestHandler
import json

class SimpleRESTHandler(BaseHTTPRequestHandler):
    def do_POST(self):
        # We only care about /api/sensor_data
        if self.path == '/api/sensor_data':
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
    port = 8080
    server = HTTPServer(('0.0.0.0', port), SimpleRESTHandler)
    print(f"Listening for ESP32 POST requests on port {port}...")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nShutting down server.")
        server.server_close()
