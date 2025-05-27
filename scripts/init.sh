#!/bin/bash
# pi_setup_combined.sh - Complete Pi environment setup for Gazebo-ROS2 simulation
# Combines installation and configuration in one robust script

set -e  # Exit on any error

echo "🚀 Setting up Raspberry Pi 4B for Gazebo-ROS2 Simulation (Complete Setup)..."
echo "========================================================================"

# Define workspace directory early
WORKSPACE_DIR="$HOME/gazebo-ros-pi"

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

print_step() {
    echo -e "${BLUE}[STEP]${NC} $1"
}

# Create log file
LOG_FILE="/tmp/pi_setup_$(date +%Y%m%d_%H%M%S).log"
exec 1> >(tee -a "$LOG_FILE")
exec 2> >(tee -a "$LOG_FILE" >&2)

print_status "Setup log will be saved to: $LOG_FILE"

# Check if running on Pi
print_step "Checking system compatibility..."
if ! grep -q "Raspberry Pi" /proc/cpuinfo 2>/dev/null; then
    print_warning "This doesn't appear to be a Raspberry Pi. Continuing anyway..."
else
    print_status "Raspberry Pi detected"
fi

# Check system requirements
print_step "Checking system requirements..."
TOTAL_MEM=$(free -m | awk 'NR==2{print $2}')
if [ "$TOTAL_MEM" -lt 3500 ]; then
    print_warning "Less than 4GB RAM detected ($TOTAL_MEM MB). ROS simulation may be limited."
else
    print_status "Memory check passed: $TOTAL_MEM MB"
fi

# Update system first
print_step "Updating system packages..."
sudo apt update && sudo apt upgrade -y

# Install essential packages
print_step "Installing essential system packages..."
sudo apt install -y \
    curl \
    wget \
    git \
    vim \
    htop \
    tree \
    unzip \
    software-properties-common \
    apt-transport-https \
    ca-certificates \
    gnupg \
    lsb-release \
    python3-pip \
    python3-venv \
    python3-full \
    openssh-server

print_status "Essential packages installed"

# Docker installation and configuration
print_step "Setting up Docker..."

if command -v docker &> /dev/null; then
    print_status "Docker already installed: $(docker --version)"
    DOCKER_ALREADY_INSTALLED=true
else
    print_status "Installing Docker from official repository..."
    DOCKER_ALREADY_INSTALLED=false
    
    # Remove any old Docker installations
    sudo apt remove -y docker docker-engine docker.io containerd runc 2>/dev/null || true
    
    # Add Docker's official GPG key
    sudo mkdir -p /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
    
    # Set up Docker repository
    echo \
      "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
      $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
    
    # Install Docker
    sudo apt update
    sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
    
    # Add user to docker group
    sudo usermod -aG docker $USER
    print_status "Docker installed successfully"
fi

# Docker Compose installation
print_step "Setting up Docker Compose..."

if command -v docker-compose &> /dev/null; then
    print_status "Docker Compose already installed: $(docker-compose --version)"
else
    print_status "Installing Docker Compose..."
    
    # Get latest version
    DOCKER_COMPOSE_VERSION=$(curl -s https://api.github.com/repos/docker/compose/releases/latest | grep 'tag_name' | cut -d\" -f4)
    
    # Download for ARM64
    sudo curl -L "https://github.com/docker/compose/releases/download/${DOCKER_COMPOSE_VERSION}/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
    sudo chmod +x /usr/local/bin/docker-compose
    
    # Create symlink
    sudo ln -sf /usr/local/bin/docker-compose /usr/bin/docker-compose
    
    print_status "Docker Compose installed: $(docker-compose --version)"
fi

# Configure Docker daemon for Pi optimization (SAFE VERSION)
print_step "Configuring Docker daemon for Pi optimization..."

if [ ! -f /etc/docker/daemon.json ]; then
    print_status "Creating Docker daemon configuration..."
    sudo tee /etc/docker/daemon.json > /dev/null <<EOF
{
  "log-driver": "json-file",
  "log-opts": {
    "max-size": "10m",
    "max-file": "3"
  },
  "storage-driver": "overlay2"
}
EOF
    
    print_status "Restarting Docker daemon..."
    sudo systemctl restart docker
    
    # Verify Docker is still running
    sleep 3
    if ! systemctl is-active --quiet docker; then
        print_error "Docker failed to restart. Removing daemon.json..."
        sudo rm /etc/docker/daemon.json
        sudo systemctl restart docker
        print_warning "Docker daemon configuration reverted to defaults"
    else
        print_status "Docker daemon configured successfully"
    fi
else
    print_status "Docker daemon configuration already exists"
fi

# Create simulation workspace
print_step "Setting up simulation workspace..."

if [ ! -d "$WORKSPACE_DIR" ]; then
    mkdir -p "$WORKSPACE_DIR"/{scripts,ros2_ws/src,models,worlds,web_interface/{templates,static/{js,css}}}
    print_status "Workspace directory created: $WORKSPACE_DIR"
else
    print_status "Workspace directory already exists: $WORKSPACE_DIR"
fi

# Create Python virtual environment
print_step "Setting up Python virtual environment..."

if [ ! -d "$WORKSPACE_DIR/venv" ]; then
    python3 -m venv "$WORKSPACE_DIR/venv"
    source "$WORKSPACE_DIR/venv/bin/activate"
    pip install --upgrade pip
    pip install flask flask-socketio flask-cors eventlet roslibpy requests
    deactivate
    print_status "Python virtual environment created and configured"
else
    print_status "Python virtual environment already exists"
fi

# Configure system performance
print_step "Configuring system performance..."

# GPU memory split
CONFIG_FILE="/boot/firmware/config.txt"
if [ ! -f "$CONFIG_FILE" ]; then
    CONFIG_FILE="/boot/config.txt"  # Fallback for older systems
fi

if [ -f "$CONFIG_FILE" ]; then
    if ! grep -q "gpu_mem=128" "$CONFIG_FILE" 2>/dev/null; then
        echo "gpu_mem=128" | sudo tee -a "$CONFIG_FILE"
        print_status "GPU memory split configured (128MB)"
    else
        print_status "GPU memory split already configured"
    fi
    
    # CPU frequency (optional - commented out for safety)
    if ! grep -q "arm_freq=2000" "$CONFIG_FILE" 2>/dev/null; then
        echo "" | sudo tee -a "$CONFIG_FILE"
        echo "# Overclocking (uncomment if you have adequate cooling)" | sudo tee -a "$CONFIG_FILE"
        echo "#arm_freq=2000" | sudo tee -a "$CONFIG_FILE"
        echo "#over_voltage=6" | sudo tee -a "$CONFIG_FILE"
        print_status "CPU overclocking configuration added (commented out for safety)"
        print_warning "Uncomment overclocking lines in $CONFIG_FILE if you have adequate cooling"
    else
        print_status "CPU overclocking configuration already exists"
    fi
else
    print_warning "Config file not found at expected locations"
fi

# Configure swap
print_step "Configuring swap memory..."

if [ -f /etc/dphys-swapfile ]; then
    if ! grep -q "CONF_SWAPSIZE=2048" /etc/dphys-swapfile; then
        sudo sed -i 's/CONF_SWAPSIZE=100/CONF_SWAPSIZE=2048/' /etc/dphys-swapfile
        sudo systemctl restart dphys-swapfile
        print_status "Swap configured to 2GB"
    else
        print_status "Swap already configured to 2GB"
    fi
elif [ -f /swapfile ]; then
    SWAP_SIZE=$(ls -lh /swapfile | awk '{print $5}')
    print_status "Swap file detected: $SWAP_SIZE - keeping existing configuration"
else
    print_warning "No swap configuration found - consider adding swap for better performance"
fi

# Enable and start SSH
print_step "Configuring SSH access..."
if ! systemctl is-active --quiet ssh; then
    sudo systemctl enable ssh
    sudo systemctl start ssh
    print_status "SSH enabled and started"
else
    print_status "SSH already running"
fi

# Create system monitoring script
print_step "Creating system monitoring tools..."

cat > "$WORKSPACE_DIR/scripts/monitor_pi_resources.sh" << 'EOF'
#!/bin/bash
# monitor_pi_resources.sh - Monitor Pi system resources

echo "=== Raspberry Pi System Monitor ==="
echo "Press Ctrl+C to exit"
echo ""

while true; do
    clear
    echo "=== Raspberry Pi System Monitor ==="
    echo "$(date)"
    echo "======================================="
    
    # Temperature
    TEMP=$(vcgencmd measure_temp 2>/dev/null | cut -d= -f2 || echo 'N/A')
    echo "🌡️  Temperature: $TEMP"
    
    # CPU Usage
    CPU_USAGE=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | awk -F'%' '{print $1}')
    echo "💻 CPU Usage: ${CPU_USAGE}%"
    
    # Memory
    MEMORY=$(free -h | awk 'NR==2{printf "Used: %s/%s (%.1f%%)", $3,$2,$3*100/$2 }')
    echo "🧠 Memory: $MEMORY"
    
    # Disk Usage
    DISK=$(df -h / | awk 'NR==2{print "Used: " $3 "/" $2 " (" $5 ")"}')
    echo "💾 Disk: $DISK"
    
    # Load Average
    LOAD=$(uptime | awk -F': ' '{print $2}')
    echo "⚖️  Load Average: $LOAD"
    
    echo "======================================="
    
    # Docker containers
    if command -v docker &> /dev/null && [ "$(docker ps -q 2>/dev/null)" ]; then
        echo "🐳 Docker Containers:"
        docker stats --no-stream --format "table {{.Name}}\t{{.CPUPerc}}\t{{.MemUsage}}\t{{.NetIO}}" 2>/dev/null || echo "   No running containers"
    else
        echo "🐳 Docker: No running containers"
    fi
    
    echo "======================================="
    echo "Press Ctrl+C to exit"
    sleep 5
done
EOF
chmod +x "$WORKSPACE_DIR/scripts/monitor_pi_resources.sh"

# Create Docker test script
print_step "Creating Docker test script..."

cat > "$WORKSPACE_DIR/scripts/test_docker_setup.sh" << 'EOF'
#!/bin/bash
# test_docker_setup.sh - Test Docker installation and ROS compatibility

echo "🔧 Testing Docker Setup for ROS2 Simulation..."
echo "=============================================="

# Test basic Docker
echo "1. Testing Docker installation..."
if docker --version; then
    echo "   ✅ Docker installed"
else
    echo "   ❌ Docker not working"
    exit 1
fi

echo ""
echo "2. Testing Docker daemon..."
if docker info > /dev/null 2>&1; then
    echo "   ✅ Docker daemon running"
else
    echo "   ❌ Docker daemon not running"
    exit 1
fi

echo ""
echo "3. Testing Docker permissions..."
if docker run --rm hello-world > /dev/null 2>&1; then
    echo "   ✅ Docker permissions OK"
else
    echo "   ❌ Docker permissions issue"
    echo "   Try: sudo usermod -aG docker $USER && logout"
    exit 1
fi

echo ""
echo "4. Testing Docker Compose..."
if docker-compose --version; then
    echo "   ✅ Docker Compose installed"
else
    echo "   ❌ Docker Compose not working"
    exit 1
fi

echo ""
echo "5. Testing ARM64 ROS2 image availability..."
echo "   (This may take a few minutes on first run...)"
if docker pull ros:humble-ros-base > /dev/null 2>&1; then
    echo "   ✅ ROS2 Humble image available"
else
    echo "   ❌ Cannot pull ROS2 image - check internet connection"
fi

echo ""
echo "6. System resources check:"
MEMORY=$(free -h | awk 'NR==2{print $2}')
TEMP=$(vcgencmd measure_temp 2>/dev/null | cut -d= -f2 || echo 'N/A')
DISK=$(df -h / | awk 'NR==2{print $4}')
echo "   Memory: $MEMORY"
echo "   Temperature: $TEMP"
echo "   Available Disk: $DISK"

echo ""
echo "7. Testing Docker container resource limits..."
if docker run --rm --memory=1g --cpus=0.5 ubuntu:22.04 echo "Resource limits test passed" > /dev/null 2>&1; then
    echo "   ✅ Docker resource limits working"
else
    echo "   ⚠️  Docker resource limits may not work properly"
fi

echo ""
echo "🎉 Docker setup test completed successfully!"
echo "=============================================="
echo ""
echo "Next steps:"
echo "1. Create your docker-compose.pi.yml file"
echo "2. Build your ROS2 containers"
echo "3. Start your simulation with: docker-compose up -d"
EOF
chmod +x "$WORKSPACE_DIR/scripts/test_docker_setup.sh"

# Create quick start script
print_step "Creating quick start script..."

cat > "$WORKSPACE_DIR/scripts/quick_start.sh" << 'EOF'
#!/bin/bash
# quick_start.sh - Quick start guide for ROS2 simulation

echo "🚀 Gazebo-ROS2-Pi Quick Start Guide"
echo "=================================="
echo ""
echo "1. 📁 Your workspace is at: ~/gazebo-ros-pi"
echo ""
echo "2. 🔧 Essential commands:"
echo "   Monitor resources: ~/gazebo-ros-pi/scripts/monitor_pi_resources.sh"
echo "   Test Docker setup: ~/gazebo-ros-pi/scripts/test_docker_setup.sh"
echo ""
echo "3. 🐳 Docker management:"
echo "   Build containers: docker-compose -f docker-compose.pi.yml build"
echo "   Start simulation: docker-compose -f docker-compose.pi.yml up -d"
echo "   View logs:       docker-compose -f docker-compose.pi.yml logs -f"
echo "   Stop simulation: docker-compose -f docker-compose.pi.yml down"
echo ""
echo "4. 🌐 Access points (once running):"
echo "   Web Interface:   http://$(hostname -I | awk '{print $1}'):5000"
echo "   ROS Bridge:      ws://$(hostname -I | awk '{print $1}'):9090"
echo "   Monitoring:      http://$(hostname -I | awk '{print $1}'):8000"
echo ""
echo "5. 🔍 Troubleshooting:"
echo "   Check container status: docker ps"
echo "   View container logs:    docker logs <container_name>"
echo "   Check system resources: htop or ~/gazebo-ros-pi/scripts/monitor_pi_resources.sh"
echo ""
echo "6. 🐍 Python virtual environment:"
echo "   Activate:   source ~/gazebo-ros-pi/venv/bin/activate"
echo "   Deactivate: deactivate"
echo ""
echo "Happy simulating! 🤖"
EOF
chmod +x "$WORKSPACE_DIR/scripts/quick_start.sh"

# Final system check and summary
print_step "Running final system check..."

echo ""
echo "========================================"
echo "📊 FINAL SYSTEM SUMMARY"
echo "========================================"
echo "🖥️  OS: $(lsb_release -d | cut -f2)"
echo "🏗️  Kernel: $(uname -r)"
echo "🔧 Architecture: $(uname -m)"
echo "🧠 Memory: $(free -h | awk 'NR==2{print $2}')"
echo "💾 Disk Space: $(df -h / | awk 'NR==2{print $4 " available"}')"
echo "🌡️  Temperature: $(vcgencmd measure_temp 2>/dev/null | cut -d= -f2 || echo 'N/A')"
echo "🐳 Docker: $(docker --version 2>/dev/null || echo 'Not working')"
echo "📦 Docker Compose: $(docker-compose --version 2>/dev/null || echo 'Not working')"
echo "📁 Workspace: $WORKSPACE_DIR"
echo "📝 Log File: $LOG_FILE"
echo "========================================"

print_status "🎉 Pi setup completed successfully!"

# Check if reboot is needed
REBOOT_NEEDED=false
if [ "$DOCKER_ALREADY_INSTALLED" = false ]; then
    REBOOT_NEEDED=true
fi

if [ "$REBOOT_NEEDED" = true ]; then
    echo ""
    print_warning "⚠️  REBOOT REQUIRED"
    print_warning "Please reboot your Pi to ensure all changes take effect:"
    echo ""
    echo "    sudo reboot"
    echo ""
    print_status "After reboot, run the quick start guide:"
    echo "    ~/gazebo-ros-pi/scripts/quick_start.sh"
else
    echo ""
    print_status "✅ No reboot required. You can start using the system immediately!"
    echo ""
    print_status "Run the quick start guide:"
    echo "    ~/gazebo-ros-pi/scripts/quick_start.sh"
fi

echo ""
print_status "📚 Useful next steps:"
echo "1. 🧪 Test Docker setup: ~/gazebo-ros-pi/scripts/test_docker_setup.sh"
echo "2. 📊 Monitor resources: ~/gazebo-ros-pi/scripts/monitor_pi_resources.sh"
echo "3. 🔧 Create docker-compose.pi.yml and Dockerfile.arm64"
echo "4. 🚀 Start your ROS2 simulation!"

echo ""
print_status "🔗 Setup complete! Full log saved to: $LOG_FILE"