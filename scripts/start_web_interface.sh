#!/bin/bash
# start_web_interface.sh - Web UI launcher

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

# Get script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
WEB_DIR="$PROJECT_DIR/web_interface"

print_status "Starting Gazebo ROS Web Interface..."
print_info "Project directory: $PROJECT_DIR"

# Check if web interface directory exists
if [ ! -d "$WEB_DIR" ]; then
    print_warning "Web interface directory not found. Creating..."
    mkdir -p "$WEB_DIR"
fi

# Create web interface files if they don't exist
if [ ! -f "$WEB_DIR/app.py" ]; then
    print_status "Creating web interface application..."
    
    cat > "$WEB_DIR/app.py" << 'EOF'
#!/usr/bin/env python3
"""
Simple web interface for Gazebo ROS simulation
"""

from flask import Flask, render_template, request, jsonify
from flask_socketio import SocketIO, emit
import json
import subprocess
import threading
import time
import os

app = Flask(__name__)
app.config['SECRET_KEY'] = 'gazebo-ros-secret-key'
socketio = SocketIO(app, cors_allowed_origins="*")

class SimulationManager:
    def __init__(self):
        self.is_running = False
        self.process = None
        
    def start_simulation(self):
        if not self.is_running:
            try:
                # Start headless Gazebo simulation
                cmd = ['docker-compose', '-f', '../docker-compose.pi.yml', 'up', '-d']
                self.process = subprocess.Popen(cmd, cwd='/app')
                self.is_running = True
                return True
            except Exception as e:
                print(f"Error starting simulation: {e}")
                return False
        return True
    
    def stop_simulation(self):
        if self.is_running and self.process:
            try:
                subprocess.run(['docker-compose', '-f', '../docker-compose.pi.yml', 'down'], cwd='/app')
                self.is_running = False
                return True
            except Exception as e:
                print(f"Error stopping simulation: {e}")
                return False
        return True
    
    def get_status(self):
        try:
            result = subprocess.run(['docker', 'ps', '--format', 'table {{.Names}}\t{{.Status}}'], 
                                  capture_output=True, text=True)
            return {
                'running': self.is_running,
                'containers': result.stdout
            }
        except:
            return {'running': False, 'containers': 'Error getting status'}

sim_manager = SimulationManager()

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/api/simulation/start', methods=['POST'])
def start_simulation():
    success = sim_manager.start_simulation()
    return jsonify({'success': success, 'message': 'Simulation started' if success else 'Failed to start'})

@app.route('/api/simulation/stop', methods=['POST'])
def stop_simulation():
    success = sim_manager.stop_simulation()
    return jsonify({'success': success, 'message': 'Simulation stopped' if success else 'Failed to stop'})

@app.route('/api/simulation/status')
def get_status():
    return jsonify(sim_manager.get_status())

@app.route('/api/system/stats')
def get_system_stats():
    try:
        # Get temperature
        temp_result = subprocess.run(['vcgencmd', 'measure_temp'], capture_output=True, text=True)
        temperature = temp_result.stdout.strip() if temp_result.returncode == 0 else 'N/A'
        
        # Get memory usage
        mem_result = subprocess.run(['free', '-h'], capture_output=True, text=True)
        memory_lines = mem_result.stdout.split('\n')[1].split()
        memory_used = memory_lines[2]
        memory_total = memory_lines[1]
        
        # Get CPU usage
        cpu_result = subprocess.run(['top', '-bn1'], capture_output=True, text=True)
        cpu_line = [line for line in cpu_result.stdout.split('\n') if 'Cpu(s)' in line][0]
        cpu_usage = cpu_line.split(',')[0].split(':')[1].strip()
        
        return jsonify({
            'temperature': temperature,
            'memory': f"{memory_used}/{memory_total}",
            'cpu': cpu_usage
        })
    except Exception as e:
        return jsonify({'error': str(e)})

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    emit('status', {'message': 'Connected to Gazebo ROS Interface'})

@socketio.on('robot_command')
def handle_robot_command(data):
    # Handle robot movement commands
    cmd = data.get('command', '')
    print(f"Received robot command: {cmd}")
    
    # Here you would send commands to ROS
    # For now, just echo back
    emit('robot_response', {'command': cmd, 'status': 'received'})

if __name__ == '__main__':
    print("Starting Gazebo ROS Web Interface...")
    print("Access at: http://localhost:5000")
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)
EOF

    # Create templates directory and HTML template
    mkdir -p "$WEB_DIR/templates"
    cat > "$WEB_DIR/templates/index.html" << 'EOF'
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Gazebo ROS Simulation Interface</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f0f0f0;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
            background: white;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        .header {
            text-align: center;
            color: #333;
            margin-bottom: 30px;
        }
        .control-panel {
            display: grid;
            grid-template-columns: 1fr 1fr;
            gap: 20px;
            margin-bottom: 20px;
        }
        .panel {
            background: #f8f9fa;
            padding: 20px;
            border-radius: 8px;
            border: 1px solid #dee2e6;
        }
        .panel h3 {
            margin-top: 0;
            color: #495057;
        }
        .button {
            background-color: #007bff;
            color: white;
            border: none;
            padding: 10px 20px;
            border-radius: 5px;
            cursor: pointer;
            margin: 5px;
            font-size: 14px;
        }
        .button:hover {
            background-color: #0056b3;
        }
        .button.danger {
            background-color: #dc3545;
        }
        .button.danger:hover {
            background-color: #c82333;
        }
        .status {
            background: #d4edda;
            color: #155724;
            padding: 10px;
            border-radius: 5px;
            margin: 10px 0;
            border: 1px solid #c3e6cb;
        }
        .robot-controls {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 10px;
            max-width: 300px;
            margin: 20px auto;
        }
        .robot-controls .button {
            padding: 15px;
            font-size: 16px;
        }
        .stats {
            display: grid;
            grid-template-columns: repeat(3, 1fr);
            gap: 15px;
            margin-top: 20px;
        }
        .stat-card {
            background: white;
            padding: 15px;
            border-radius: 8px;
            border: 1px solid #dee2e6;
            text-align: center;
        }
        .stat-value {
            font-size: 24px;
            font-weight: bold;
            color: #007bff;
        }
        .stat-label {
            color: #6c757d;
            font-size: 14px;
        }
        #log {
            background: #212529;
            color: #fff;
            padding: 15px;
            border-radius: 5px;
            height: 200px;
            overflow-y: auto;
            font-family: monospace;
            font-size: 12px;
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🤖 Gazebo ROS Simulation Interface</h1>
            <p>Raspberry Pi 4B Mobile Robot Simulation Environment</p>
        </div>

        <div class="control-panel">
            <div class="panel">
                <h3>Simulation Control</h3>
                <button class="button" onclick="startSimulation()">▶️ Start Simulation</button>
                <button class="button danger" onclick="stopSimulation()">⏹️ Stop Simulation</button>
                <button class="button" onclick="getStatus()">🔄 Refresh Status</button>
                <div id="sim-status" class="status">Status: Unknown</div>
            </div>

            <div class="panel">
                <h3>System Stats</h3>
                <div class="stats">
                    <div class="stat-card">
                        <div class="stat-value" id="temperature">--</div>
                        <div class="stat-label">Temperature</div>
                    </div>
                    <div class="stat-card">
                        <div class="stat-value" id="memory">--</div>
                        <div class="stat-label">Memory</div>
                    </div>
                    <div class="stat-card">
                        <div class="stat-value" id="cpu">--</div>
                        <div class="stat-label">CPU Usage</div>
                    </div>
                </div>
            </div>
        </div>

        <div class="panel">
            <h3>Robot Control</h3>
            <div class="robot-controls">
                <div></div>
                <button class="button" onclick="sendCommand('forward')">⬆️ Forward</button>
                <div></div>
                <button class="button" onclick="sendCommand('left')">⬅️ Left</button>
                <button class="button" onclick="sendCommand('stop')">⏹️ Stop</button>
                <button class="button" onclick="sendCommand('right')">➡️ Right</button>
                <div></div>
                <button class="button" onclick="sendCommand('backward')">⬇️ Backward</button>
                <div></div>
            </div>
        </div>

        <div class="panel">
            <h3>System Log</h3>
            <div id="log"></div>
        </div>
    </div>

    <script>
        const socket = io();
        
        socket.on('connect', function() {
            addLog('Connected to server');
            getStatus();
            updateSystemStats();
        });

        socket.on('status', function(data) {
            addLog('Status: ' + data.message);
        });

        socket.on('robot_response', function(data) {
            addLog('Robot command: ' + data.command + ' - Status: ' + data.status);
        });

        function addLog(message) {
            const log = document.getElementById('log');
            const timestamp = new Date().toLocaleTimeString();
            log.innerHTML += `[${timestamp}] ${message}\n`;
            log.scrollTop = log.scrollHeight;
        }

        function startSimulation() {
            fetch('/api/simulation/start', {method: 'POST'})
                .then(response => response.json())
                .then(data => {
                    addLog(data.message);
                    getStatus();
                });
        }

        function stopSimulation() {
            fetch('/api/simulation/stop', {method: 'POST'})
                .then(response => response.json())
                .then(data => {
                    addLog(data.message);
                    getStatus();
                });
        }

        function getStatus() {
            fetch('/api/simulation/status')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('sim-status').innerHTML = 
                        `Status: ${data.running ? 'Running' : 'Stopped'}`;
                });
        }

        function updateSystemStats() {
            fetch('/api/system/stats')
                .then(response => response.json())
                .then(data => {
                    if (data.error) {
                        addLog('Error getting system stats: ' + data.error);
                    } else {
                        document.getElementById('temperature').textContent = data.temperature || '--';
                        document.getElementById('memory').textContent = data.memory || '--';
                        document.getElementById('cpu').textContent = data.cpu || '--';
                    }
                });
        }

        function sendCommand(command) {
            socket.emit('robot_command', {command: command});
            addLog('Sent command: ' + command);
        }

        // Update stats every 5 seconds
        setInterval(updateSystemStats, 5000);
        setInterval(getStatus, 10000);

        // Keyboard controls
        document.addEventListener('keydown', function(event) {
            switch(event.key.toLowerCase()) {
                case 'w': sendCommand('forward'); break;
                case 's': sendCommand('backward'); break;
                case 'a': sendCommand('left'); break;
                case 'd': sendCommand('right'); break;
                case ' ': sendCommand('stop'); event.preventDefault(); break;
            }
        });
    </script>
</body>
</html>
EOF

    chmod +x "$WEB_DIR/app.py"
fi

# Check dependencies
print_status "Checking Python dependencies..."
if ! python3 -c "import flask" 2>/dev/null; then
    print_warning "Flask not found. Installing..."
    pip3 install flask flask-socketio eventlet
fi

# Get Pi IP address
PI_IP=$(hostname -I | awk '{print $1}')

# Check if Docker is running
if ! docker info >/dev/null 2>&1; then
    print_error "Docker is not running. Please start Docker first:"
    echo "sudo systemctl start docker"
    exit 1
fi

# Start the web interface
print_status "Starting web interface..."
print_info "Web interface will be available at:"
echo "  Local:    http://localhost:5000"
echo "  Network:  http://$PI_IP:5000"
echo ""
print_info "Use Ctrl+C to stop the web interface"

cd "$WEB_DIR"
export FLASK_ENV=development
python3 app.py