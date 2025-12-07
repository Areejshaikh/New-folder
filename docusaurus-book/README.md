# Physical AI & Humanoid Robotics Book - ROS 2 Module

This repository contains educational content for a comprehensive book on ROS 2 and humanoid robotics, implemented as a Docusaurus documentation site.

## About This Book

This book bridges the gap between digital artificial intelligence and physical embodiment, providing learners with the knowledge and tools necessary to create intelligent humanoid robots. Module 1 covers:

1. **ROS 2 Architecture**: Understanding Nodes, Topics, and Services
2. **Python Agent Integration**: Connecting Python agents to ROS 2 using rclpy
3. **Humanoid Robot Modeling**: Creating robot models with URDF

## Prerequisites

- Node.js (v18 or higher)
- npm (v8 or higher)
- ROS 2 Humble Hawksbill (for running code examples)
- Python 3.8+ (for Python code examples)

## Installation

1. Install dependencies:
   ```bash
   npm install
   ```

2. Run the development server:
   ```bash
   npm start
   ```

3. Open your browser to `http://localhost:3000` to view the book.

## Building for Production

To create a production-ready build of the site:

```bash
npm run build
```

## Contributing

We welcome contributions to improve the content. Please feel free to submit issues or pull requests with your suggestions.

## Content Structure

```
book-content/
├── ros2-module/
    ├── chapter-1-intro-middleware/
    ├── chapter-2-python-ros-integration/
    └── chapter-3-urdf-modeling/
```

Each chapter contains:
- `content.md`: Main content file
- `code-examples/`: Python code examples
- `diagrams/`: Diagrams and illustrations

## Running Code Examples

To run the ROS 2 code examples:

1. Source your ROS 2 environment:
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. Create and build a ROS 2 workspace:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   # Copy code examples to your workspace
   colcon build
   source install/setup.bash
   ```

3. Run examples:
   ```bash
   ros2 run <package_name> simple_publisher.py
   ```

## License

This content is available under the MIT License.