#!/bin/bash
# pi_setup.sh - Pi environment setup

set -e  # Exit on any error

echo "🚀 Setting up Raspberry Pi 4B for Gazebo ROS Simulation..."
echo "=================================================="

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

# Install ROS 2 Humble (optional - can run in Docker instead)
print_status "Installing ROS 2 Humble..."
if ! command -v ros2 &> /dev/null; then
    # Add ROS 2 GPG key
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    
    # Add ROS 2 repository
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    sudo apt update
    sudo apt install -y ros-humble-desktop-full
    
    # Install additional ROS packages
    sudo apt install -y \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-ros-control \
        ros-humble-rosbridge-server \
        ros-humble-web-video-server \
        python3-rosdep \
        python3-colcon-common-extensions
    
    # Initialize rosdep
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
        sudo rosdep init
    fi
    rosdep update
    
    # Add ROS setup to bashrc
    if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
        echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    fi
else
    print_status "ROS 2 already installed."
fi

# Install Python packages for web interface
print_status "Installing Python packages..."
sudo apt install -y python3-pip python3-venv
pip3 install --user flask flask-socketio eventlet

# Configure system performance
print_status "Configuring system performance..."

# GPU memory split
if ! grep -q "gpu_mem=128" /boot/firmware/config.txt; then
    echo "gpu_mem=128" | sudo tee -a /boot/firmware/config.txt
fi

# CPU frequency (optional - may require more cooling)
if ! grep -q "arm_freq=2000" /boot/firmware/config.txt; then
    echo "# Overclocking (ensure adequate cooling)" | sudo tee -a /boot/firmware/config.txt
    echo "arm_freq=2000" | sudo tee -a /boot/firmware/config.txt
    echo "over_voltage=6" | sudo tee -a /boot/firmware/config.txt
fi

# Configure swap
print_status "Configuring swap..."
if [ -f /etc/dphys-swapfile ]; then
    sudo sed -i 's/CONF_SWAPSIZE=100/CONF_SWAPSIZE=2048/' /etc/dphys-swapfile
    sudo systemctl restart dphys-swapfile
fi

# Enable SSH (if not already enabled)
print_status "Ensuring SSH is enabled..."
sudo systemctl enable ssh
sudo systemctl start ssh

# Configure Docker daemon
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
  }
}
EOF
    sudo systemctl restart docker
fi

# Create simulation workspace
print_status "Setting up simulation workspace..."
WORKSPACE_DIR="$HOME/gazebo-ros-pi"
if [ ! -d "$WORKSPACE_DIR" ]; then
    mkdir -p "$WORKSPACE_DIR"/{scripts,catkin_ws/src,models,worlds,web_interface}
fi

# Set up catkin workspace
if [ ! -f "$WORKSPACE_DIR/catkin_ws/src/CMakeLists.txt" ]; then
    cd "$WORKSPACE_DIR/catkin_ws"
    if command -v ros2 &> /dev/null; then
        # ROS 2 workspace
        colcon build
    fi
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
echo "ROS 2: $(ros2 --version 2>/dev/null || echo 'Not installed')"
echo "Temperature: $(vcgencmd measure_temp 2>/dev/null || echo 'N/A')"
echo "======================================"

print_status "Pi setup completed successfully! 🎉"
print_warning "Please reboot your Pi to ensure all changes take effect:"
echo "sudo reboot"

print_status "After reboot, you can:"
echo "1. Start monitoring: ./scripts/monitor_pi_resources.sh"
echo "2. Set up your Gazebo simulation environment"
echo "3. Test Docker: docker run hello-world"

echo ""
print_status "Setup log saved to: /tmp/pi_setup.log"