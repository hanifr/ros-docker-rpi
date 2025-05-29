from flask import Flask, render_template, jsonify, request, redirect, url_for
from flask_socketio import SocketIO, emit
import json
import threading
import time
import os

app = Flask(__name__)
app.config['SECRET_KEY'] = 'your-secret-key'
socketio = SocketIO(app, cors_allowed_origins="*")

# =============================================================================
# MAIN ROUTES - Interface Navigation
# =============================================================================

@app.route('/')
def dashboard():
    """Main dashboard - system overview and interface selection"""
    return render_template('dashboard.html')

@app.route('/control')
@app.route('/robot_physics_controller.html')
def robot_control():
    """Advanced robot control with 3D visualization"""
    return render_template('robot_physics_controller.html')

@app.route('/physics')
@app.route('/physics_tuner.html')
def physics_tuner():
    """Physics parameter tuning interface"""
    return render_template('physics_tuner.html')

@app.route('/data')
@app.route('/physics_data_logger.html')
def data_logger():
    """Physics simulation data logger"""
    return render_template('physics_data_logger.html')

@app.route('/quality')
@app.route('/quality_control.html')
def quality_control():
    """Quality control station interface"""
    return render_template('quality_control.html')

@app.route('/material')
@app.route('/advanced_material_handling.html')
def material_handling():
    """Material handling system interface"""
    return render_template('material_handling.html')

@app.route('/excel')
@app.route('/excel_integration.html')
def excel_integration():
    """Excel integration and data export"""
    return render_template('excel_integration.html')

@app.route('/advanced')
def advanced_control():
    """Your original sophisticated interface"""
    return render_template('index.html')

# =============================================================================
# API ENDPOINTS - Enhanced for Multiple Interfaces
# =============================================================================

@app.route('/health')
def health():
    """System health check"""
    return jsonify({
        "status": "healthy", 
        "timestamp": time.time(),
        "interfaces": {
            "dashboard": True,
            "control": True,
            "physics": True,
            "data": True,
            "quality": True,
            "material": True,
            "excel": True,
            "advanced": True
        }
    })

@app.route('/api/robot/status')
def robot_status():
    """Robot status - enhanced with more details"""
    return jsonify({
        "connected": True,
        "position": {"x": 0, "y": 0, "z": 0},
        "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
        "linear_velocity": {"x": 0, "y": 0, "z": 0},
        "angular_velocity": {"x": 0, "y": 0, "z": 0},
        "battery": 100,
        "uptime": time.time(),
        "physics_engine": "Gazebo Classic",
        "simulation_time": time.time()
    })

@app.route('/api/interfaces')
def list_interfaces():
    """List all available interfaces"""
    interfaces = [
        {"id": "dashboard", "name": "Dashboard", "url": "/", "icon": "🏠", "description": "System overview and quick controls"},
        {"id": "control", "name": "Robot Control", "url": "/control", "icon": "🎮", "description": "Advanced robot control with 3D visualization"},
        {"id": "physics", "name": "Physics Tuner", "url": "/physics", "icon": "⚙️", "description": "Physics parameter experimentation"},
        {"id": "data", "name": "Data Logger", "url": "/data", "icon": "📊", "description": "Data collection and analysis"},
        {"id": "quality", "name": "Quality Control", "url": "/quality", "icon": "🔬", "description": "Automated quality inspection"},
        {"id": "material", "name": "Material Handling", "url": "/material", "icon": "📦", "description": "AGV and logistics simulation"},
        {"id": "excel", "name": "Excel Integration", "url": "/excel", "icon": "📈", "description": "Data export and analysis tools"},
        {"id": "advanced", "name": "Advanced Control", "url": "/advanced", "icon": "🚀", "description": "Full-featured control interface"}
    ]
    return jsonify(interfaces)

@app.route('/api/system/info')
def system_info():
    """Extended system information"""
    return jsonify({
        "platform": "Raspberry Pi 4B",
        "ros_version": "ROS 2 Humble",
        "physics_engine": "Gazebo Classic",
        "web_framework": "Flask + SocketIO",
        "interfaces_available": 8,
        "docker_containers": ["gazebo-sim", "tf2-web-republisher", "web-interface", "monitor"],
        "ports": {
            "web_interface": 5000,
            "ros_bridge": 9090,
            "tf2_web": 9091,
            "gazebo_master": 11345
        }
    })

# =============================================================================
# DATA EXPORT ENDPOINTS - For Excel/MATLAB Integration
# =============================================================================

@app.route('/api/export/robot_data')
def export_robot_data():
    """Export robot data in various formats"""
    format_type = request.args.get('format', 'json')
    
    # Sample data - replace with actual robot data collection
    data = {
        "timestamp": time.time(),
        "robot_data": [
            {"time": 0, "x": 0, "y": 0, "velocity": 0},
            {"time": 1, "x": 0.1, "y": 0, "velocity": 0.1},
            # Add more sample data
        ]
    }
    
    if format_type == 'csv':
        # Return CSV formatted data
        return jsonify({"format": "csv", "data": data})
    elif format_type == 'matlab':
        # Return MATLAB-friendly format
        return jsonify({"format": "matlab", "data": data})
    else:
        return jsonify(data)

# =============================================================================
# SOCKETIO HANDLERS - Enhanced for Multiple Interfaces
# =============================================================================

@socketio.on('connect')
def handle_connect():
    """Handle client connection"""
    print(f'Client connected from {request.sid}')
    emit('status', {
        'connected': True,
        'timestamp': time.time(),
        'available_interfaces': 8
    })

@socketio.on('disconnect')
def handle_disconnect():
    """Handle client disconnection"""
    print(f'Client disconnected: {request.sid}')

@socketio.on('robot_command')
def handle_robot_command(data):
    """Handle robot commands from any interface"""
    print(f"Received robot command: {data}")
    
    # Add command validation
    valid_commands = ['forward', 'backward', 'left', 'right', 'stop', 'reset']
    if data.get('command') in valid_commands:
        emit('command_response', {
            'status': 'received', 
            'command': data,
            'timestamp': time.time()
        })
    else:
        emit('command_response', {
            'status': 'error', 
            'message': 'Invalid command',
            'timestamp': time.time()
        })

@socketio.on('request_robot_status')
def handle_status_request():
    """Handle real-time status requests"""
    emit('robot_status_update', {
        'position': {'x': 0, 'y': 0, 'z': 0},
        'velocity': {'linear': 0, 'angular': 0},
        'battery': 100,
        'timestamp': time.time()
    })

@socketio.on('physics_parameter_update')
def handle_physics_update(data):
    """Handle physics parameter updates from tuner interface"""
    print(f"Physics parameter update: {data}")
    emit('physics_update_response', {
        'status': 'updated',
        'parameters': data,
        'timestamp': time.time()
    })

@socketio.on('data_export_request')
def handle_data_export(data):
    """Handle data export requests"""
    export_format = data.get('format', 'json')
    print(f"Data export requested: {export_format}")
    
    # Generate sample export data
    export_data = {
        'format': export_format,
        'filename': f'robot_data_{int(time.time())}.{export_format}',
        'ready': True,
        'download_url': f'/api/export/robot_data?format={export_format}'
    }
    
    emit('data_export_ready', export_data)

# =============================================================================
# ERROR HANDLERS
# =============================================================================

@app.errorhandler(404)
def not_found(error):
    """Handle 404 errors by redirecting to dashboard"""
    return redirect(url_for('dashboard'))

@app.errorhandler(500)
def internal_error(error):
    """Handle 500 errors"""
    return jsonify({
        'error': 'Internal server error',
        'message': 'Please check system status',
        'redirect': '/'
    }), 500

# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

def check_template_files():
    """Check if all required template files exist"""
    required_templates = [
        'dashboard.html',
        'index.html',
        'robot_physics_controller.html',
        'physics_tuner.html',
        'physics_data_logger.html',
        'quality_control.html',
        'material_handling.html',
        'excel_integration.html'
    ]
    
    templates_dir = os.path.join(app.root_path, 'templates')
    missing_templates = []
    
    for template in required_templates:
        template_path = os.path.join(templates_dir, template)
        if not os.path.exists(template_path):
            missing_templates.append(template)
    
    if missing_templates:
        print("WARNING: Missing template files:")
        for template in missing_templates:
            print(f"  - {template}")
    else:
        print("✅ All template files found")
    
    return len(missing_templates) == 0

# =============================================================================
# MAIN APPLICATION STARTUP
# =============================================================================

if __name__ == '__main__':
    print("🚀 Starting ROS 2 Engineering Platform...")
    print("📍 Platform: Raspberry Pi 4B")
    print("🔧 Framework: Flask + SocketIO")
    print("🤖 ROS Version: ROS 2 Humble")
    print("⚙️ Physics Engine: Gazebo Classic")
    
    # Check template files
    if check_template_files():
        print("✅ Template verification complete")
    
    print("🌐 Available interfaces:")
    print("   🏠 Dashboard: http://localhost:5000/")
    print("   🎮 Robot Control: http://localhost:5000/control")
    print("   ⚙️ Physics Tuner: http://localhost:5000/physics")
    print("   📊 Data Logger: http://localhost:5000/data")
    print("   🔬 Quality Control: http://localhost:5000/quality")
    print("   📦 Material Handling: http://localhost:5000/material")
    print("   📈 Excel Integration: http://localhost:5000/excel")
    print("   🚀 Advanced Control: http://localhost:5000/advanced")
    
    print("\n🔄 Starting server...")
    socketio.run(app, host='0.0.0.0', port=5000, debug=False)