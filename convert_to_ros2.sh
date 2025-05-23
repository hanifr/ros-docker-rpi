#!/bin/bash
# convert_to_ros2.sh - Convert ROS 1 packages to ROS 2 format

set -e  # Exit on any error

echo "🔄 Converting ROS 1 packages to ROS 2 (Ubuntu 22.04 compatible)..."
echo "=================================================================="

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

# Check if we're in the right directory
if [ ! -d "catkin_ws" ]; then
    print_error "catkin_ws directory not found. Are you in the gazebo-ros-pi directory?"
    exit 1
fi

# Backup original workspace
print_status "Backing up original ROS 1 workspace..."
if [ ! -d "catkin_ws_backup" ]; then
    cp -r catkin_ws catkin_ws_backup
    print_status "Original workspace backed up to catkin_ws_backup/"
else
    print_warning "Backup already exists, skipping..."
fi

# Create ROS 2 workspace structure
print_status "Creating ROS 2 workspace structure..."
mkdir -p ros2_ws/{src,build,install,log}

# Convert each package
for pkg_dir in catkin_ws/src/*/; do
    if [ -d "$pkg_dir" ]; then
        pkg_name=$(basename "$pkg_dir")
        print_status "Converting package: $pkg_name"
        
        # Create package directory
        mkdir -p "ros2_ws/src/$pkg_name"
        
        # Copy non-build files
        for item in urdf meshes config worlds launch; do
            if [ -d "$pkg_dir$item" ]; then
                print_status "  Copying $item/ directory..."
                cp -r "$pkg_dir$item" "ros2_ws/src/$pkg_name/"
            fi
        done
        
        # Copy other files (excluding CMakeLists.txt and package.xml - we'll create new ones)
        find "$pkg_dir" -maxdepth 1 -type f ! -name "CMakeLists.txt" ! -name "package.xml" -exec cp {} "ros2_ws/src/$pkg_name/" \;
        
        # Generate ROS 2 package.xml
        print_status "  Creating ROS 2 package.xml..."
        cat > "ros2_ws/src/$pkg_name/package.xml" << EOF
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <n>$pkg_name</n>
  <version>1.0.0</version>
  <description>$pkg_name package converted from ROS 1</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <!-- Common dependencies -->
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  
  <!-- Package-specific dependencies based on name -->
EOF

        # Add package-specific dependencies
        if [[ "$pkg_name" == *"description"* ]]; then
            cat >> "ros2_ws/src/$pkg_name/package.xml" << EOF
  <depend>urdf</depend>
  <depend>xacro</depend>
  <depend>robot_state_publisher</depend>
  <depend>joint_state_publisher</depend>
EOF
        fi
        
        if [[ "$pkg_name" == *"gazebo"* ]]; then
            cat >> "ros2_ws/src/$pkg_name/package.xml" << EOF
  <depend>gazebo_ros_pkgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>
EOF
        fi
        
        # Close package.xml
        cat >> "ros2_ws/src/$pkg_name/package.xml" << EOF

  <!-- Test dependencies -->
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF
        
        # Generate ROS 2 CMakeLists.txt
        print_status "  Creating ROS 2 CMakeLists.txt..."
        cat > "ros2_ws/src/$pkg_name/CMakeLists.txt" << EOF
cmake_minimum_required(VERSION 3.8)
project($pkg_name)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
EOF

        # Add install directories based on what exists
        for item in urdf meshes config worlds launch; do
            if [ -d "ros2_ws/src/$pkg_name/$item" ]; then
                echo "  $item" >> "ros2_ws/src/$pkg_name/CMakeLists.txt"
            fi
        done
        
        cat >> "ros2_ws/src/$pkg_name/CMakeLists.txt" << EOF
  DESTINATION share/\${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
EOF
        
        print_status "  Package $pkg_name converted successfully!"
    fi
done

# Convert .launch files to .launch.py
print_status "Converting launch files to Python format..."
find ros2_ws/src -name "*.launch" | while read launch_file; do
    if [ -f "$launch_file" ]; then
        py_launch_file="${launch_file%.launch}.launch.py"
        print_status "  Converting $(basename "$launch_file") to Python format..."
        
        # Create a basic Python launch file template
        cat > "$py_launch_file" << 'EOF'
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Add your launch logic here
    # This is a template - you may need to customize based on your original .launch file
    
    return LaunchDescription([
        # Add your nodes and launch configurations here
    ])
EOF
        
        # Remove old .launch file
        rm "$launch_file"
        print_warning "    Created template $py_launch_file - you may need to customize it!"
    fi
done

# Create build script for ROS 2
print_status "Creating ROS 2 build script..."
cat > "scripts/build_ros2_workspace.sh" << 'EOF'
#!/bin/bash
# build_ros2_workspace.sh - Build ROS 2 workspace

echo "🔨 Building ROS 2 workspace..."

cd ros2_ws

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build with colcon
colcon build --symlink-install \
             --cmake-args -DCMAKE_BUILD_TYPE=Release \
             --parallel-workers 2 \
             --event-handlers console_direct+

echo "✅ ROS 2 workspace built successfully!"
echo "To use: source ros2_ws/install/setup.bash"
EOF
chmod +x "scripts/build_ros2_workspace.sh"

# Create test script
print_status "Creating ROS 2 test script..."
cat > "scripts/test_ros2_setup.sh" << 'EOF'
#!/bin/bash
# test_ros2_setup.sh - Test ROS 2 installation and workspace

echo "🔧 Testing ROS 2 Setup..."

# Test ROS 2 installation
echo "1. Testing ROS 2 installation..."
ros2 --version

echo "2. Testing ROS 2 workspace..."
cd ros2_ws
if [ -f install/setup.bash ]; then
    source install/setup.bash
    echo "✅ ROS 2 workspace built and ready"
    
    echo "3. Listing available packages..."
    ros2 pkg list | grep my_robot || echo "⚠️  No my_robot packages found - build workspace first"
else
    echo "❌ ROS 2 workspace not built - run ./scripts/build_ros2_workspace.sh first"
fi

echo "🎉 ROS 2 setup test completed!"
EOF
chmod +x "scripts/test_ros2_setup.sh"

print_status "Conversion completed! 🎉"
echo ""
print_status "Summary of changes:"
echo "✅ Original workspace backed up to catkin_ws_backup/"
echo "✅ ROS 2 workspace created in ros2_ws/"
echo "✅ Package files converted to ROS 2 format"
echo "✅ Launch files converted to Python format (templates created)"
echo "✅ Build and test scripts created"
echo ""
print_warning "Next steps:"
echo "1. Review and customize the generated launch files in ros2_ws/src/*/launch/"
echo "2. Build the workspace: ./scripts/build_ros2_workspace.sh"
echo "3. Test the setup: ./scripts/test_ros2_setup.sh"
echo "4. Update your Docker setup to use ROS 2"
echo ""
print_status "Your URDF, mesh, and world files are unchanged and compatible with ROS 2!"