#!/usr/bin/env python3
"""
Pi Resource Monitor - Web interface for monitoring Raspberry Pi resources
and Docker containers in real-time.
"""

from flask import Flask, jsonify, render_template_string
import psutil
import json
import time
import os
import subprocess
from datetime import datetime
import logging
import colorlog

# Setup colored logging
handler = colorlog.StreamHandler()
handler.setFormatter(colorlog.ColoredFormatter(
    '%(log_color)s%(levelname)s:%(name)s:%(message)s'
))
logger = colorlog.getLogger(__name__)
logger.addHandler(handler)
logger.setLevel(logging.INFO)

app = Flask(__name__)

class PiMonitor:
    """Raspberry Pi system monitor with Docker integration"""
    
    def __init__(self):
        self.docker_available = self._check_docker()
        logger.info(f"Docker available: {self.docker_available}")
    
    def _check_docker(self):
        """Check if Docker is available"""
        try:
            import docker
            client = docker.from_env()
            client.ping()
            return True
        except Exception as e:
            logger.warning(f"Docker not available: {e}")
            return False
    
    def get_system_info(self):
        """Get comprehensive system information"""
        try:
            # CPU Information
            cpu_percent = psutil.cpu_percent(interval=1)
            cpu_count = psutil.cpu_count()
            cpu_freq = psutil.cpu_freq()
            load_avg = os.getloadavg() if hasattr(os, 'getloadavg') else None
            
            # Memory Information
            memory = psutil.virtual_memory()
            swap = psutil.swap_memory()
            
            # Disk Information
            disk = psutil.disk_usage('/')
            
            # Network Information
            network = psutil.net_io_counters() if psutil.net_io_counters() else {}
            
            # Pi Temperature
            temperature = self._get_pi_temperature()
            
            # Uptime
            boot_time = psutil.boot_time()
            uptime = time.time() - boot_time
            
            return {
                'timestamp': datetime.now().isoformat(),
                'cpu': {
                    'percent': round(cpu_percent, 1),
                    'count': cpu_count,
                    'frequency': cpu_freq._asdict() if cpu_freq else None,
                    'load_avg': load_avg
                },
                'memory': {
                    'used': memory.used,
                    'total': memory.total,
                    'percent': round(memory.percent, 1),
                    'available': memory.available
                },
                'swap': {
                    'used': swap.used,
                    'total': swap.total,
                    'percent': round(swap.percent, 1) if swap.total > 0 else 0
                },
                'disk': {
                    'used': disk.used,
                    'total': disk.total,
                    'percent': round((disk.used / disk.total) * 100, 1),
                    'free': disk.free
                },
                'network': {
                    'bytes_sent': network.bytes_sent if hasattr(network, 'bytes_sent') else 0,
                    'bytes_recv': network.bytes_recv if hasattr(network, 'bytes_recv') else 0,
                    'packets_sent': network.packets_sent if hasattr(network, 'packets_sent') else 0,
                    'packets_recv': network.packets_recv if hasattr(network, 'packets_recv') else 0
                },
                'temperature': temperature,
                'uptime': uptime
            }
        except Exception as e:
            logger.error(f"Error getting system info: {e}")
            return {'error': str(e)}
    
    def _get_pi_temperature(self):
        """Get Raspberry Pi temperature"""
        try:
            # Try thermal zone method first
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp = float(f.read().strip()) / 1000.0
                return round(temp, 1)
        except:
            try:
                # Try vcgencmd method
                result = subprocess.run(['vcgencmd', 'measure_temp'], 
                                      capture_output=True, text=True, timeout=5)
                if result.returncode == 0:
                    temp_str = result.stdout.strip()
                    # Extract temperature from "temp=45.1'C"
                    temp = float(temp_str.split('=')[1].split("'")[0])
                    return round(temp, 1)
            except:
                pass
        return None
    
    def get_docker_info(self):
        """Get Docker container information"""
        if not self.docker_available:
            return {'error': 'Docker not available'}
        
        try:
            import docker
            client = docker.from_env()
            
            containers = []
            for container in client.containers.list(all=True):
                try:
                    # Get container stats if running
                    stats = None
                    cpu_percent = 0
                    memory_usage = 0
                    memory_limit = 0
                    memory_percent = 0
                    
                    if container.status == 'running':
                        try:
                            stats = container.stats(stream=False)
                            
                            # Calculate CPU percentage
                            if 'cpu_stats' in stats and 'precpu_stats' in stats:
                                cpu_stats = stats['cpu_stats']
                                precpu_stats = stats['precpu_stats']
                                
                                cpu_delta = cpu_stats['cpu_usage']['total_usage'] - \
                                           precpu_stats['cpu_usage']['total_usage']
                                system_delta = cpu_stats['system_cpu_usage'] - \
                                              precpu_stats['system_cpu_usage']
                                
                                if system_delta > 0 and cpu_delta >= 0:
                                    cpu_count = len(cpu_stats['cpu_usage']['percpu_usage'])
                                    cpu_percent = (cpu_delta / system_delta) * cpu_count * 100.0
                            
                            # Calculate memory usage
                            if 'memory_stats' in stats:
                                memory_usage = stats['memory_stats'].get('usage', 0)
                                memory_limit = stats['memory_stats'].get('limit', 0)
                                if memory_limit > 0:
                                    memory_percent = (memory_usage / memory_limit) * 100
                        except Exception as e:
                            logger.warning(f"Error getting stats for {container.name}: {e}")
                    
                    containers.append({
                        'name': container.name,
                        'image': container.image.tags[0] if container.image.tags else container.image.id[:12],
                        'status': container.status,
                        'created': container.attrs['Created'],
                        'cpu_percent': round(cpu_percent, 2),
                        'memory_usage': memory_usage,
                        'memory_limit': memory_limit,
                        'memory_percent': round(memory_percent, 2),
                        'ports': container.ports
                    })
                except Exception as e:
                    logger.warning(f"Error processing container {container.name}: {e}")
            
            return {
                'containers': containers,
                'total_containers': len(containers),
                'running_containers': len([c for c in containers if c['status'] == 'running'])
            }
        except Exception as e:
            logger.error(f"Error getting Docker info: {e}")
            return {'error': str(e)}

# Initialize monitor
monitor = PiMonitor()

@app.route('/')
def dashboard():
    """Main dashboard with real-time updates"""
    html_template = """
    <!DOCTYPE html>
    <html>
    <head>
        <title>🍓 Pi Resource Monitor</title>
        <meta http-equiv="refresh" content="10">
        <meta name="viewport" content="width=device-width, initial-scale=1.0">
        <style>
            * { margin: 0; padding: 0; box-sizing: border-box; }
            body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: #1a1a1a; color: #fff; }
            .container { max-width: 1200px; margin: 0 auto; padding: 20px; }
            .header { text-align: center; margin-bottom: 30px; }
            .header h1 { color: #ff6b6b; margin-bottom: 10px; }
            .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px; }
            .card { background: #2d2d2d; border-radius: 12px; padding: 20px; border: 1px solid #404040; }
            .card h3 { color: #4ecdc4; margin-bottom: 15px; display: flex; align-items: center; }
            .card h3::before { content: '●'; margin-right: 8px; }
            .metric { display: flex; justify-content: space-between; margin: 12px 0; padding: 8px; background: #1a1a1a; border-radius: 6px; }
            .metric-name { font-weight: 500; }
            .metric-value { color: #ff6b6b; font-weight: bold; }
            .progress-bar { width: 100%; height: 8px; background: #404040; border-radius: 4px; overflow: hidden; margin: 8px 0; }
            .progress-fill { height: 100%; border-radius: 4px; transition: width 0.3s ease; }
            .progress-low { background: linear-gradient(90deg, #4ecdc4, #45b7aa); }
            .progress-med { background: linear-gradient(90deg, #ffe66d, #ff8c42); }
            .progress-high { background: linear-gradient(90deg, #ff6b6b, #c44569); }
            .temperature { color: #ff6b6b; font-weight: bold; }
            .status { padding: 4px 8px; border-radius: 4px; font-size: 12px; font-weight: bold; }
            .status-running { background: #4ecdc4; color: #000; }
            .status-stopped { background: #666; color: #fff; }
            .container-list { max-height: 300px; overflow-y: auto; }
            .container-item { background: #1a1a1a; margin: 8px 0; padding: 10px; border-radius: 6px; border-left: 3px solid #4ecdc4; }
            .refresh-info { text-align: center; color: #666; font-size: 14px; margin-top: 20px; }
            .auto-refresh { animation: pulse 2s infinite; }
            @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.5; } }
        </style>
    </head>
    <body>
        <div class="container">
            <div class="header">
                <h1>🍓 Raspberry Pi Monitor</h1>
                <p>Real-time system monitoring for ROS 2 Gazebo simulation</p>
            </div>
            
            <div class="grid" id="dashboard">
                <div class="card">
                    <h3>Loading system information...</h3>
                    <p>Please wait while we gather system metrics.</p>
                </div>
            </div>
            
            <div class="refresh-info">
                <span class="auto-refresh">● Auto-refresh every 10 seconds</span> | 
                <a href="javascript:location.reload()" style="color: #4ecdc4;">Manual Refresh</a> |
                <a href="/api/system" style="color: #4ecdc4;">JSON API</a>
            </div>
        </div>
        
        <script>
            function formatBytes(bytes) {
                if (bytes === 0) return '0 B';
                const k = 1024;
                const sizes = ['B', 'KB', 'MB', 'GB'];
                const i = Math.floor(Math.log(bytes) / Math.log(k));
                return parseFloat((bytes / Math.pow(k, i)).toFixed(1)) + ' ' + sizes[i];
            }
            
            function getProgressClass(percent) {
                if (percent < 50) return 'progress-low';
                if (percent < 80) return 'progress-med';
                return 'progress-high';
            }
            
            function formatUptime(seconds) {
                const days = Math.floor(seconds / 86400);
                const hours = Math.floor((seconds % 86400) / 3600);
                const minutes = Math.floor((seconds % 3600) / 60);
                return `${days}d ${hours}h ${minutes}m`;
            }
            
            function updateDashboard() {
                Promise.all([
                    fetch('/api/system').then(r => r.json()),
                    fetch('/api/docker').then(r => r.json())
                ]).then(([systemData, dockerData]) => {
                    const dashboard = document.getElementById('dashboard');
                    dashboard.innerHTML = generateDashboard(systemData, dockerData);
                }).catch(error => {
                    console.error('Error updating dashboard:', error);
                    document.getElementById('dashboard').innerHTML = '<div class="card"><h3>Error loading data</h3><p>' + error + '</p></div>';
                });
            }
            
            function generateDashboard(system, docker) {
                let html = `
                    <div class="card">
                        <h3>💻 System Overview</h3>
                        <div class="metric">
                            <span class="metric-name">CPU Usage</span>
                            <span class="metric-value">${system.cpu.percent}%</span>
                        </div>
                        <div class="progress-bar">
                            <div class="progress-fill ${getProgressClass(system.cpu.percent)}" style="width: ${system.cpu.percent}%"></div>
                        </div>
                        
                        <div class="metric">
                            <span class="metric-name">Memory</span>
                            <span class="metric-value">${formatBytes(system.memory.used)} / ${formatBytes(system.memory.total)}</span>
                        </div>
                        <div class="progress-bar">
                            <div class="progress-fill ${getProgressClass(system.memory.percent)}" style="width: ${system.memory.percent}%"></div>
                        </div>
                        
                        <div class="metric">
                            <span class="metric-name">Disk Usage</span>
                            <span class="metric-value">${formatBytes(system.disk.used)} / ${formatBytes(system.disk.total)}</span>
                        </div>
                        <div class="progress-bar">
                            <div class="progress-fill ${getProgressClass(system.disk.percent)}" style="width: ${system.disk.percent}%"></div>
                        </div>
                        
                        <div class="metric">
                            <span class="metric-name">Temperature</span>
                            <span class="metric-value temperature">${system.temperature ? system.temperature + '°C' : 'N/A'}</span>
                        </div>
                        
                        <div class="metric">
                            <span class="metric-name">Uptime</span>
                            <span class="metric-value">${formatUptime(system.uptime)}</span>
                        </div>
                    </div>
                `;
                
                if (docker.containers) {
                    html += `
                        <div class="card">
                            <h3>🐳 Docker Containers (${docker.running_containers}/${docker.total_containers})</h3>
                            <div class="container-list">
                    `;
                    
                    docker.containers.forEach(container => {
                        const statusClass = container.status === 'running' ? 'status-running' : 'status-stopped';
                        html += `
                            <div class="container-item">
                                <div style="display: flex; justify-content: space-between; align-items: center;">
                                    <strong>${container.name}</strong>
                                    <span class="status ${statusClass}">${container.status}</span>
                                </div>
                                <div style="font-size: 12px; color: #999; margin-top: 4px;">
                                    ${container.image}
                                </div>
                                ${container.status === 'running' ? `
                                <div style="font-size: 12px; margin-top: 4px;">
                                    CPU: ${container.cpu_percent}% | Memory: ${container.memory_percent}%
                                </div>
                                ` : ''}
                            </div>
                        `;
                    });
                    
                    html += '</div></div>';
                } else {
                    html += `
                        <div class="card">
                            <h3>🐳 Docker Status</h3>
                            <p>${docker.error || 'Docker information unavailable'}</p>
                        </div>
                    `;
                }
                
                return html;
            }
            
            // Initial load and periodic updates
            updateDashboard();
            setInterval(updateDashboard, 10000);
        </script>
    </body>
    </html>
    """
    return render_template_string(html_template)

@app.route('/api/system')
def api_system():
    """System information API endpoint"""
    return jsonify(monitor.get_system_info())

@app.route('/api/docker')
def api_docker():
    """Docker information API endpoint"""
    return jsonify(monitor.get_docker_info())

@app.route('/api/all')
def api_all():
    """Combined system and Docker information"""
    return jsonify({
        'system': monitor.get_system_info(),
        'docker': monitor.get_docker_info(),
        'timestamp': datetime.now().isoformat()
    })

@app.route('/health')
def health():
    """Health check endpoint for Docker"""
    try:
        # Basic health check - ensure we can get system info
        system_info = monitor.get_system_info()
        if 'error' in system_info:
            return jsonify({'status': 'unhealthy', 'error': system_info['error']}), 500
        
        return jsonify({
            'status': 'healthy',
            'timestamp': datetime.now().isoformat(),
            'uptime': system_info.get('uptime', 0)
        })
    except Exception as e:
        return jsonify({'status': 'unhealthy', 'error': str(e)}), 500

if __name__ == '__main__':
    logger.info('🔍 Starting Pi Resource Monitor...')
    logger.info('Dashboard available at: http://localhost:8000')
    logger.info('API endpoints: /api/system, /api/docker, /api/all')
    
    app.run(host='0.0.0.0', port=8000, debug=False, threaded=True)