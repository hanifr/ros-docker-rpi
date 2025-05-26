#!/bin/bash
# pi_setup.sh - Pi environment setup (Docker ROS Only)

set -e  # Exit on any error

echo "🚀 Setting up Raspberry Pi 4B for Gazebo ROS Simulation (Docker Mode)..."
echo "=================================================="

# Define workspace directory early
WORKSPACE_DIR="$HOME/gazebo-ros-pi"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
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

# Check if running on Pi
if ! grep -q "Raspberry Pi" /proc/cpuinfo 2>/dev/null; then
    print_warning "This doesn't appear to be a Raspberry Pi. Continuing anyway..."
fi

# Update system
print_status "Updating system packages..."
sudo apt update && sudo apt upgrade -y

# Install essential packages
print_status "Installing essential packages..."
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
    lsb-release

# Install Docker
print_status "Installing Docker..."
if ! command -v docker &> /dev/null; then
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
    print_status "Docker installed. You may need to log out and back in for group changes to take effect."
else
    print_status "Docker already installed."
fi

# Install Docker Compose (standalone)
print_status "Installing Docker Compose..."
if ! command -v docker-compose &> /dev/null; then
    # Get latest version
    DOCKER_COMPOSE_VERSION=$(curl -s https://api.github.com/repos/docker/compose/releases/latest | grep 'tag_name' | cut -d\" -f4)
    
    # Download for ARM64
    sudo curl -L "https://github.com/docker/compose/releases/download/${DOCKER_COMPOSE_VERSION}/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
    sudo chmod +x /usr/local/bin/docker-compose
    
    # Create symlink
    sudo ln -sf /usr/local/bin/docker-compose /usr/bin/docker-compose
else
    print_status "Docker Compose already installed."
fi

# Create simulation workspace first
print_status "Setting up simulation workspace..."
if [ ! -d "$WORKSPACE_DIR" ]; then
    mkdir -p "$WORKSPACE_DIR"/{scripts,catkin_ws/src,models,worlds,web_interface}
    print_status "Workspace directory created: $WORKSPACE_DIR"
fi

# Install Python packages for web interface
print_status "Installing Python packages..."
sudo apt install -y python3-pip python3-venv python3-full

# Create virtual environment for the project
print_status "Creating Python virtual environment..."
if [ ! -d "$WORKSPACE_DIR/venv" ]; then
    python3 -m venv "$WORKSPACE_DIR/venv"
    source "$WORKSPACE_DIR/venv/bin/activate"
    pip install flask flask-socketio eventlet
    deactivate
    print_status "Virtual environment created: $WORKSPACE_DIR/venv"
else
    print_status "Virtual environment already exists"
fi

# Configure system performance
print_status "Configuring system performance..."

# GPU memory split
if ! grep -q "gpu_mem=128" /boot/firmware/config.txt 2>/dev/null; then
    echo "gpu_mem=128" | sudo tee -a /boot/firmware/config.txt
    print_status "GPU memory split configured"
fi

# CPU frequency (optional - may require more cooling)
if ! grep -q "arm_freq=2000" /boot/firmware/config.txt 2>/dev/null; then
    echo "# Overclocking (ensure adequate cooling)" | sudo tee -a /boot/firmware/config.txt
    echo "arm_freq=2000" | sudo tee -a /boot/firmware/config.txt
    echo "over_voltage=6" | sudo tee -a /boot/firmware/config.txt
    print_warning "CPU overclocking enabled - ensure adequate cooling!"
fi

# Configure swap
print_status "Configuring swap..."
if [ -f /etc/dphys-swapfile ]; then
    sudo sed -i 's/CONF_SWAPSIZE=100/CONF_SWAPSIZE=2048/' /etc/dphys-swapfile
    sudo systemctl restart dphys-swapfile
    print_status "Swap configured to 2GB"
elif [ -f /swapfile ]; then
    # Ubuntu might use different swap setup
    print_status "Swap file detected, keeping existing configuration"
else
    print_warning "No swap configuration found"
fi

# Enable SSH (if not already enabled)
print_status "Ensuring SSH is enabled..."
sudo apt install openssh-server -y
sudo systemctl enable ssh
sudo systemctl start ssh

# Configure Docker daemon for Pi optimization
print_status "Configuring Docker daemon..."
if [ ! -f /etc/docker/daemon.json ]; then
    sudo tee /etc/docker/daemon.json > /dev/null <<EOF
{
  "log-driver": "json-file",
  "log-opts": {
    "max-size": "10m",
    "max-file": "3"
  },
  "storage-driver": "overlay2",
  "experimental": true,
  "features": {
    "buildkit": true
  },
  "default-runtime": "runc",
  "runtimes": {
    "runc": {
      "path": "runc"
    }
  }
}
EOF
    print_status "Docker daemon configured for Pi optimization"
    sudo systemctl restart docker
fi

# Create system monitoring script
print_status "Creating system monitoring tools..."
cat > "$WORKSPACE_DIR/scripts/monitor_pi_resources.sh" << 'EOF'
#!/bin/bash
# monitor_pi_resources.sh - Monitor Pi system resources

echo "=== Raspberry Pi System Monitor ==="
while true; do
    clear
    echo "$(date)"
    echo "======================================"
    echo "Temperature: $(vcgencmd measure_temp 2>/dev/null || echo 'N/A')"
    echo "CPU Usage: $(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | awk -F'%' '{print $1}')%"
    echo "Memory: $(free -h | awk 'NR==2{printf "Used: %s/%s (%.1f%%)", $3,$2,$3*100/$2 }')"
    echo "Disk Usage: $(df -h / | awk 'NR==2{print "Used: " $3 "/" $2 " (" $5 ")"}')"
    echo "Load Average: $(uptime | awk -F': ' '{print $2}')"
    echo "======================================"
    
    if command -v docker &> /dev/null; then
        echo "Docker Containers:"
        if [ "$(docker ps -q)" ]; then
            docker stats --no-stream --format "table {{.Name}}\t{{.CPUPerc}}\t{{.MemUsage}}\t{{.NetIO}}"
        else
            echo "No running containers"
        fi
    fi
    
    echo "======================================"
    echo "Press Ctrl+C to exit"
    sleep 5
done
EOF
chmod +x "$WORKSPACE_DIR/scripts/monitor_pi_resources.sh"

# Create Docker test script
print_status "Creating Docker test script..."
cat > "$WORKSPACE_DIR/scripts/test_docker_setup.sh" << 'EOF'
#!/bin/bash
# test_docker_setup.sh - Test Docker installation

echo "🔧 Testing Docker Setup..."

# Test basic Docker
echo "1. Testing Docker installation..."
docker --version

echo "2. Testing Docker daemon..."
docker info > /dev/null && echo "✅ Docker daemon running" || echo "❌ Docker daemon not running"

echo "3. Testing Docker permissions..."
docker run --rm hello-world > /dev/null 2>&1 && echo "✅ Docker permissions OK" || echo "❌ Docker permissions issue - try: sudo usermod -aG docker $USER && logout"

echo "4. Testing Docker Compose..."
docker-compose --version

echo "5. Testing ARM64 ROS image pull..."
docker pull arm64v8/ros:humble-ros-base-jammy && echo "✅ ARM64 ROS image available" || echo "❌ Cannot pull ARM64 ROS image"

echo "6. System resources:"
echo "   Memory: $(free -h | awk 'NR==2{print $2}')"
echo "   Temperature: $(vcgencmd measure_temp 2>/dev/null || echo 'N/A')"
echo ""
echo "🎉 Docker setup test completed!"
EOF
chmod +x "$WORKSPACE_DIR/scripts/test_docker_setup.sh"

# Final system check
print_status "Running system check..."
echo "======================================"
echo "System Information:"
echo "OS: $(lsb_release -d | cut -f2)"
echo "Kernel: $(uname -r)"
echo "Architecture: $(uname -m)"
echo "Memory: $(free -h | awk 'NR==2{print $2}')"
echo "Disk Space: $(df -h / | awk 'NR==2{print $4 " available"}')"
echo "Docker: $(docker --version 2>/dev/null || echo 'Not installed')"
echo "Docker Compose: $(docker-compose --version 2>/dev/null || echo 'Not installed')"
echo "Temperature: $(vcgencmd measure_temp 2>/dev/null || echo 'N/A')"
echo "======================================"

print_status "Pi setup completed successfully! 🎉"
print_warning "Please reboot your Pi to ensure all changes take effect:"
echo "sudo reboot"

print_status "After reboot, you can:"
echo "1. Test Docker setup: $WORKSPACE_DIR/scripts/test_docker_setup.sh"
echo "2. Start monitoring: $WORKSPACE_DIR/scripts/monitor_pi_resources.sh"
echo "3. Create your Docker Compose configuration"
echo "4. Start headless Gazebo simulation"

echo ""
print_status "Next steps:"
print_status "1. Create docker-compose.pi.yml with ROS Humble container"
print_status "2. Create Dockerfile.arm64 for custom image"
print_status "3. Test with simple_robot.urdf in headless mode"
print_status "source $WORKSPACE_DIR/venv/bin/activate"
print_status "python your_script.py"
print_status "deactivate"

echo ""
print_status "Setup log saved to: /tmp/pi_setup.log"