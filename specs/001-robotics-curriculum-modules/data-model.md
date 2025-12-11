# Data Model for Robotics Curriculum Modules

## Entities

### 1. Curriculum Modules
- **ID**: Unique identifier for the module
- **Name**: The name of the module (e.g., "ROS 2", "Gazebo & Unity", "NVIDIA Isaac", "Vision-Language-Action")
- **Description**: Detailed description of the module focus and objectives
- **Learning Objectives**: List of specific learning goals for the module
- **Duration**: Estimated time to complete the module (in hours)
- **Difficulty Level**: Beginner, Intermediate, or Advanced classification
- **Prerequisites**: List of other modules or knowledge required before starting
- **Content Path**: Path to the curriculum content files
- **Assets**: Links to required assets (simulations, datasets, etc.)
- **Assessment Methods**: How learners' understanding is validated

### 2. Simulation Environments
- **ID**: Unique identifier for the simulation
- **Name**: Name of the simulation environment
- **Type**: Type of simulation (Gazebo, Unity, Isaac Sim, etc.)
- **Description**: Detailed description of what the simulation teaches
- **Components**: List of specific components (physics, sensors, etc.)
- **Scenarios**: Different scenarios available in the simulation
- **Assets**: Links to required assets for the simulation
- **Compatibility**: List of platforms and requirements for running the simulation

### 3. Robot Control Systems
- **ID**: Unique identifier for the control system
- **Name**: Name of the control system (e.g., "ROS 2", "Isaac ROS")
- **Type**: Type of middleware or control system
- **Description**: What the control system is used for
- **Components**: List of components (Nodes, Topics, Services, etc.)
- **API Endpoints**: Available APIs for interacting with the system
- **Examples**: Code examples demonstrating usage
- **Integration Points**: How it connects with other systems

### 4. AI Integration Tools
- **ID**: Unique identifier for the AI tool
- **Name**: Name of the AI tool or technology (e.g., "OpenAI Whisper", "VSLAM", etc.)
- **Type**: Type of AI technology (NLP, Computer Vision, etc.)
- **Description**: What the tool is used for in the curriculum
- **Capabilities**: List of specific capabilities taught
- **Examples**: Code examples demonstrating usage
- **Integration Points**: How it connects with other systems
- **Privacy Compliance**: Any privacy considerations for using the tool

### 5. Learner Profiles
- **ID**: Unique identifier for the learner
- **Skill Level**: Current proficiency in robotics (Beginner, Intermediate, Expert)
- **Progress**: Current progress through the curriculum
- **Preferences**: User preferences for learning style
- **Achievements**: Completed modules and earned certifications
- **Assessment Scores**: Scores from module assessments
- **Learning Path**: Customized path based on skill level and goals

## Relationships

1. **Curriculum Modules** contain multiple **Simulation Environments**
2. **Curriculum Modules** use multiple **Robot Control Systems**
3. **Curriculum Modules** incorporate multiple **AI Integration Tools**
4. **Learner Profiles** interact with **Curriculum Modules** to track progress
5. **Simulation Environments** may use multiple **Robot Control Systems**
6. **Simulation Environments** may incorporate **AI Integration Tools**

## State Transitions

### Module Progression
- **Not Started** → **In Progress** → **Completed** → **Assessed**
- Learner can return to **In Progress** from **Completed** to review content

### Simulation Status
- **Available** → **In Use** → **Completed** → **Results Reviewed**
- Can return to **In Use** from **Completed** for additional practice

## Validation Rules

1. A module must have at least one learning objective
2. A module's difficulty level must be appropriate for its prerequisites
3. A simulation environment must be compatible with the learner's system
4. A learner profile's skill level must be set before accessing advanced modules
5. A module can only be marked as completed after achieving the minimum assessment score