#!/bin/bash
# scripts/start_monitor.sh - Simple system monitor

echo "📊 Starting Pi Monitor..."

python3 -c "
import http.server
import socketserver
import json
import subprocess

class MonitorHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        self.send_response(200)
        self.send_header('Content-type', 'application/json')
        self.end_headers()
        
        # Simple system status
        try:
            temp = int(subprocess.check_output('cat /host/sys/class/thermal/thermal_zone0/temp 2>/dev/null || echo 0', shell=True)) / 1000.0
            status = {'temperature': temp, 'status': 'running'}
            self.wfile.write(json.dumps(status).encode())
        except:
            self.wfile.write(b'{\"error\": \"monitoring unavailable\"}')

with socketserver.TCPServer(('', 8000), MonitorHandler) as httpd:
    print('Monitor running on port 8000')
    httpd.serve_forever()
"