from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit
import json
import threading
import time

app = Flask(__name__)
app.config['SECRET_KEY'] = 'your-secret-key'
socketio = SocketIO(app, cors_allowed_origins="*")

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/health')
def health():
    return jsonify({"status": "healthy", "timestamp": time.time()})

@app.route('/api/robot/status')
def robot_status():
    # This would connect to ROS and get actual status
    return jsonify({
        "connected": True,
        "position": {"x": 0, "y": 0, "z": 0},
        "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
        "linear_velocity": {"x": 0, "y": 0, "z": 0},
        "angular_velocity": {"x": 0, "y": 0, "z": 0}
    })

@socketio.on('connect')
def handle_connect():
    print('Client connected')
    emit('status', {'connected': True})

@socketio.on('disconnect')
def handle_disconnect():
    print('Client disconnected')

@socketio.on('robot_command')
def handle_robot_command(data):
    # Forward commands to ROS (implement based on your needs)
    print(f"Received robot command: {data}")
    emit('command_response', {'status': 'received', 'command': data})

if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)