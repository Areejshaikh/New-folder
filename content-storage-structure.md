# Content Storage Structure for Curriculum Modules

The curriculum content will be organized in the following structure:

## File System Structure

```
book-content/
├── modules/
│   ├── 001-ros2/
│   │   ├── module.json          # Module metadata
│   │   ├── content/
│   │   │   ├── index.md         # Main content page
│   │   │   ├── architecture.md  # ROS 2 architecture
│   │   │   ├── rclpy-examples.md
│   │   │   ├── urdf-guide.md
│   │   │   └── quickstart.md
│   │   ├── assets/
│   │   │   ├── diagrams/
│   │   │   ├── code-examples/
│   │   │   └── videos/
│   │   └── assessments/
│   │       ├── quiz.json
│   │       └── hands-on.json
│   ├── 002-gazebo-unity/
│   │   ├── module.json
│   │   ├── content/
│   │   └── assets/
│   ├── 003-nvidia-isaac/
│   │   ├── module.json
│   │   ├── content/
│   │   └── assets/
│   └── 004-vla/
│       ├── module.json
│       ├── content/
│       └── assets/
├── shared/
│   ├── assets/
│   │   ├── images/
│   │   ├── icons/
│   │   └── models/
│   ├── components/
│   └── templates/
└── metadata/
    ├── curriculum.json      # Overall curriculum structure
    ├── learning-paths.json  # Different learning paths
    └── dependencies.json    # Module dependencies
```

## Module Metadata Structure (module.json)

Each module directory contains a module.json file with metadata:

```json
{
  "id": "001-ros2",
  "name": "ROS 2",
  "title": "The Robotic Nervous System (ROS 2)",
  "description": "Introduction to Robot Operating System 2 for humanoid robotics",
  "difficulty": "intermediate",
  "duration": 20,
  "learningObjectives": [
    "Understand ROS 2 architecture (nodes, topics, services)",
    "Create a simple robot controller that communicates with ROS 2 nodes",
    "Implement rclpy examples for Python agent bridging"
  ],
  "prerequisites": [
    "Basic Python programming",
    "Fundamentals of robotics"
  ],
  "technologyStack": ["ROS 2", "Python", "rclpy"],
  "contentFiles": [
    "index.md",
    "architecture.md",
    "rclpy-examples.md",
    "urdf-guide.md",
    "quickstart.md"
  ],
  "assets": [
    {
      "id": "ros2-architecture-diagram",
      "name": "ROS 2 Architecture Diagram",
      "type": "diagram",
      "path": "./assets/diagrams/ros2-architecture.png",
      "size": 125000
    }
  ],
  "assessments": [
    {
      "type": "quiz",
      "path": "./assessments/quiz.json",
      "weight": 0.3
    },
    {
      "type": "hands-on",
      "path": "./assessments/hands-on.json",
      "weight": 0.7
    }
  ],
  "createdAt": "2025-01-15T00:00:00Z",
  "updatedAt": "2025-01-15T00:00:00Z"
}
```

## Assessment Structure

Quiz assessments (quiz.json):
```json
{
  "id": "ros2-quiz-1",
  "moduleId": "001-ros2",
  "title": "ROS 2 Fundamentals Quiz",
  "questions": [
    {
      "id": "q1",
      "type": "multiple-choice",
      "question": "What is a ROS 2 node?",
      "options": [
        {"id": "a", "text": "A network protocol"},
        {"id": "b", "text": "A running process that performs computation"},
        {"id": "c", "text": "A data type"},
        {"id": "d", "text": "A message"}
      ],
      "correctAnswer": "b",
      "explanation": "A node is a process performing computation in the ROS computation graph."
    }
  ],
  "timeLimit": 1800,
  "passingScore": 70
}
```

Hands-on assessments (hands-on.json):
```json
{
  "id": "ros2-hands-on-1",
  "moduleId": "001-ros2",
  "title": "ROS 2 Node Communication Exercise",
  "description": "Implement a publisher and subscriber to exchange data",
  "requirements": [
    "Implement a publisher node",
    "Implement a subscriber node",
    "Exchange a custom message type"
  ],
  "evaluationCriteria": [
    "Correct implementation of publisher",
    "Correct implementation of subscriber",
    "Proper message type definition",
    "Successful data exchange"
  ],
  "weight": 1.0,
  "estimatedTime": 3600
}
```

## Overall Curriculum Metadata (curriculum.json)

```json
{
  "title": "Physical AI & Humanoid Robotics",
  "description": "A comprehensive curriculum covering modern robotics technologies",
  "modules": [
    "001-ros2",
    "002-gazebo-unity", 
    "003-nvidia-isaac",
    "004-vla"
  ],
  "totalDuration": 75,
  "learningPaths": {
    "beginner": ["001-ros2", "002-gazebo-unity"],
    "intermediate": ["001-ros2", "002-gazebo-unity", "003-nvidia-isaac"],
    "advanced": ["001-ros2", "002-gazebo-unity", "003-nvidia-isaac", "004-vla"]
  },
  "createdAt": "2025-01-15T00:00:00Z",
  "updatedAt": "2025-01-15T00:00:00Z"
}
```

This structure provides:
1. Clear organization of curriculum content
2. Separation of content, assets, and assessments
3. Standardized metadata for modules
4. Support for different learning paths
5. Scalability for adding new modules