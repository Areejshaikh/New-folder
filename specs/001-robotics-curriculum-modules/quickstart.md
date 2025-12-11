# Quickstart Guide for Robotics Curriculum Modules

## Prerequisites

Before starting with the Robotics Curriculum Modules implementation, ensure you have the following:

1. **System Requirements:**
   - 64-bit processor with 8+ cores
   - 16GB+ RAM (32GB recommended for full simulation)
   - NVIDIA GPU with CUDA support (for Isaac modules)
   - 100GB+ free disk space

2. **Software Requirements:**
   - Ubuntu 22.04 LTS or Windows 10/11 (WSL2 for Linux compatibility)
   - Git version 2.25+
   - Docker and Docker Compose
   - Node.js v18+ and npm v9+
   - Python 3.10+ with pip and venv
   - ROS 2 Humble Hawksbill (or newer LTS version)
   - Unity Hub 3.0+ with Unity 2022.3 LTS

## Initial Setup

### 1. Clone the Repository
```bash
git clone https://github.com/Areejshaikh/Physical_AI_-_Humanoid_Robotics.git
cd Physical_AI_-_Humanoid_Robotics
```

### 2. Install Core Dependencies
```bash
# Install Node.js dependencies for Docusaurus
cd docusaurus-book
npm install

# Install Python dependencies
pip3 install virtualenv
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 3. Set Up ROS 2 Environment
```bash
# Source ROS 2 installation
source /opt/ros/humble/setup.bash

# Create a workspace for the project
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

### 4. Install Simulation Frameworks
```bash
# Install Gazebo Harmonic
sudo apt install ros-humble-gazebo-*

# Install NVIDIA Isaac ROS (follow official installation guide)
# This requires NVIDIA GPU drivers and CUDA

# Set up Unity project (from the unity-scenes directory)
```

## Running the Curriculum

### 1. Start the Docusaurus Book
```bash
cd docusaurus-book
npm run start
```
The book will be available at http://localhost:3000

### 2. Start the RAG Chatbot
```bash
cd RAG-chatbot
npm install
npm run start
```

### 3. Run the First Module (ROS 2)
```bash
# Navigate to the ROS 2 examples
cd simulations/ros2-examples
source ~/ros2_ws/install/setup.bash

# Run the basic node communication example
python3 basic_nodes.py
```

### 4. Run a Simulation
```bash
# Launch Gazebo simulation
cd simulations/gazebo-worlds
source ~/ros2_ws/install/setup.bash

# Run a basic simulation environment
ros2 launch basic_simulation.launch.py
```

## Development Workflow

### 1. Creating New Content
1. Add new curriculum content to the `docusaurus-book/docs` directory
2. Update the sidebar configuration in `docusaurus-book/sidebars.js`
3. Link to any new simulation or code examples

### 2. Adding New Simulations
1. Create simulation files in the appropriate directory:
   - For Gazebo: `simulations/gazebo-worlds/`
   - For Unity: `simulations/unity-scenes/`
   - For Isaac: `simulations/isaac-sim/`
2. Update the simulation configuration files
3. Test the simulation independently before integration

### 3. Testing the Implementation
```bash
# Run Python unit tests
cd docusaurus-book
source venv/bin/activate
python -m pytest tests/

# Run Docusaurus validation
npm run build

# Validate ROS 2 nodes
cd ~/ros2_ws
colcon test
```

## Troubleshooting Common Issues

### ROS 2 Issues
- If ROS 2 commands are not found, ensure you've sourced the setup.bash file
- If Python modules aren't found, activate your Python virtual environment
- For permission errors, check that ROS 2 is installed correctly

### Simulation Issues
- For Gazebo performance issues, check graphics drivers
- For Unity build errors, ensure Unity Hub is properly configured
- For Isaac modules, verify CUDA and GPU compatibility

### Docusaurus Issues
- Clear browser cache if content doesn't update
- Check Node.js version compatibility
- Verify all dependencies are installed

## Next Steps

1. Complete the full curriculum setup by following the module-specific instructions
2. Explore the testing documentation in the `tests/` directory
3. Review the API documentation for integration points
4. Set up your preferred IDE with the appropriate language extensions