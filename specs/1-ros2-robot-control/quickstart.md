# Quickstart Guide: ROS 2 Robot Control Module

## Overview
This quickstart guide provides a brief introduction to the ROS 2 Robot Control Module, outlining the essential steps for setting up the environment and running the examples provided in the educational content.

## Prerequisites

### System Requirements
- Ubuntu 22.04 LTS
- At least 8GB RAM (16GB recommended)
- 20GB free disk space
- ROS 2 Humble Hawksbill installed
- Docusaurus prerequisites (Node.js v18+, npm)

### Software Dependencies
1. **ROS 2 Humble Hawksbill**
   - Installation via Debian packages recommended
   - Source installation if needing latest updates

2. **Python 3.8+**
   - Includes pip for package management
   - Virtual environment support recommended

3. **Docusaurus v3**
   - Node.js package for documentation site
   - Required for viewing the book locally

## Setup Steps

### 1. Install ROS 2 Humble Hawksbill
```bash
# Add ROS 2 repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/rosInstall.sh | bash -s -
sudo apt update

# Install ROS 2 packages
sudo apt install -y ros-humble-desktop ros-humble-rclpy ros-humble-ros2run
sudo apt install -y python3-rosdep2 python3-argcomplete
```

### 2. Install Docusaurus Prerequisites
```bash
# Install Node.js and npm
curl -fsSL https://deb.nodesource.com/setup_lts.x | sudo -E bash -
sudo apt-get install -y nodejs

# Verify installations
node --version
npm --version
```

### 3. Clone the Repository
```bash
git clone https://github.com/Areejshaikh/Physical_AI_-_Humanoid_Robotics.git
cd Physical_AI_-_Humanoid_Robotics
```

### 4. Set up Workspace
```bash
# Create ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Copy the book examples to your workspace
cp -r Physical_AI_-_Humanoid_Robotics/book-content/ros2-module/chapter-2-python-ros-integration/code-examples/* ~/ros2_ws/src/

# Build the workspace
colcon build
source install/setup.bash
```

### 5. Run Examples
```bash
# Navigate to workspace
cd ~/ros2_ws

# Source the environment
source install/setup.bash

# Run a simple publisher example
ros2 run <package_name> simple_publisher.py

# In another terminal, run a subscriber to see messages
ros2 run <package_name> simple_subscriber.py
```

## Running the Docusaurus Book

### 1. Install Book Dependencies
```bash
cd Physical_AI_-_Humanoid_Robotics
npm install
```

### 2. Start Local Server
```bash
npm start
```

### 3. Access the Book
Open your browser to `http://localhost:3000` to view the book locally.

## Basic Examples

### 1. Simple Publisher-Subscriber
- Locate: `book-content/ros2-module/chapter-1-intro-middleware/code-examples/simple-publisher.py`
- Run: `ros2 run <package_name> simple_publisher.py`

### 2. Python Agent Connecting to ROS
- Locate: `book-content/ros2-module/chapter-2-python-ros-integration/code-examples/python-ros-agent.py`
- Run: `python3 python-ros-agent.py`

### 3. URDF Robot Model
- Locate: `book-content/ros2-module/chapter-3-urdf-modeling/code-examples/sample-urdf-model.urdf`
- Load in Gazebo: `ros2 launch gazebo_ros gazebo.launch.py`

## Troubleshooting

### Common Issues
1. **ROS 2 commands not found**
   - Ensure ROS 2 environment is sourced: `source /opt/ros/humble/setup.bash`
   - Add to your `.bashrc` for persistence

2. **Permission errors with Gazebo**
   - Make sure all ROS 2 packages are installed with proper permissions
   - Check Gazebo model paths are set correctly

3. **Docusaurus fails to start**
   - Ensure Node.js and npm are properly installed
   - Run `npm install` again to reinstall dependencies

### Validation
To validate your setup:
- Confirm ROS 2 nodes can communicate: `ros2 node list`
- Verify Python can interface with ROS: `python3 -c "import rclpy"`
- Check Gazebo launches without errors: `gazebo --version`