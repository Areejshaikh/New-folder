"""
Data Models for Robotics Curriculum Modules

This file defines the core data models for the curriculum platform:
- Curriculum Modules
- Simulation Environments
- Learner Profiles
"""

from datetime import datetime
from typing import List, Optional, Dict, Any
from enum import Enum
import uuid


class DifficultyLevel(Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class ModuleStatus(Enum):
    NOT_STARTED = "not_started"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    ASSESSED = "assessed"


class SimulationStatus(Enum):
    AVAILABLE = "available"
    IN_USE = "in_use"
    COMPLETED = "completed"
    RESULTS_REVIEWED = "results_reviewed"


class BaseModel:
    """Base model with common attributes"""
    def __init__(self):
        self.id = str(uuid.uuid4())
        self.created_at = datetime.now()
        self.updated_at = datetime.now()


class CurriculumModule(BaseModel):
    """
    Represents a curriculum module in the robotics education platform
    """
    def __init__(
        self,
        name: str,
        description: str,
        duration: int,  # in hours
        difficulty: DifficultyLevel,
        learning_objectives: List[str],
        prerequisites: Optional[List[str]] = None,
        technology_stack: Optional[List[str]] = None,
        content_url: Optional[str] = None,
        assets: Optional[List[Dict[str, Any]]] = None
    ):
        super().__init__()
        self.name = name
        self.description = description
        self.duration = duration  # Estimated time to complete in hours
        self.difficulty = difficulty
        self.learning_objectives = learning_objectives or []
        self.prerequisites = prerequisites or []
        self.technology_stack = technology_stack or []
        self.content_url = content_url
        self.assets = assets or []
        self.assessment_methods = []

    def __repr__(self):
        return f"CurriculumModule(id={self.id}, name='{self.name}', difficulty={self.difficulty})"


class SimulationEnvironment(BaseModel):
    """
    Represents a simulation environment for robotics curriculum
    """
    def __init__(
        self,
        name: str,
        description: str,
        sim_type: str,  # e.g., "Gazebo", "Unity", "Isaac Sim"
        components: Optional[List[str]] = None,
        scenarios: Optional[List[str]] = None,
        assets: Optional[List[str]] = None,
        compatibility: Optional[List[str]] = None
    ):
        super().__init__()
        self.name = name
        self.description = description
        self.type = sim_type
        self.components = components or []
        self.scenarios = scenarios or []
        self.assets = assets or []
        self.compatibility = compatibility or []

    def __repr__(self):
        return f"SimulationEnvironment(id={self.id}, name='{self.name}', type='{self.type}')"


class LearnerProfile(BaseModel):
    """
    Represents a learner's profile and progress in the robotics curriculum
    """
    def __init__(
        self,
        user_id: str,
        skill_level: DifficultyLevel,
        preferences: Optional[Dict[str, Any]] = None,
        achievements: Optional[List[str]] = None,
        assessment_scores: Optional[Dict[str, float]] = None
    ):
        super().__init__()
        self.user_id = user_id
        self.skill_level = skill_level
        self.progress = {}  # module_id: ModuleStatus
        self.preferences = preferences or {}
        self.achievements = achievements or []
        self.assessment_scores = assessment_scores or {}
        self.learning_path = []  # Ordered list of module IDs

    def add_progress(self, module_id: str, status: ModuleStatus):
        """Update progress for a specific module"""
        self.progress[module_id] = status
        self.updated_at = datetime.now()

    def add_assessment_score(self, module_id: str, score: float):
        """Add an assessment score for a module"""
        self.assessment_scores[module_id] = score
        self.updated_at = datetime.now()

    def __repr__(self):
        return f"LearnerProfile(user_id='{self.user_id}', skill_level={self.skill_level}, modules_completed={len([s for s in self.progress.values() if s == ModuleStatus.COMPLETED])})"


class UserProgress(BaseModel):
    """
    Tracks user progress through the curriculum
    """
    def __init__(
        self,
        user_id: str,
        modules_completed: Optional[List[str]] = None,
        current_module: Optional[str] = None,
        progress_percent: float = 0.0,
        achievements: Optional[List[Dict[str, Any]]] = None,
        assessment_scores: Optional[List[Dict[str, float]]] = None
    ):
        super().__init__()
        self.user_id = user_id
        self.modules_completed = modules_completed or []
        self.current_module = current_module
        self.progress_percent = progress_percent
        self.achievements = achievements or []
        self.assessment_scores = assessment_scores or []


# Example usage
if __name__ == "__main__":
    # Example of creating a curriculum module
    ros2_module = CurriculumModule(
        name="ROS 2",
        description="The Robotic Nervous System (ROS 2)",
        duration=20,
        difficulty=DifficultyLevel.INTERMEDIATE,
        learning_objectives=[
            "Understand ROS 2 architecture (nodes, topics, services)",
            "Create a simple robot controller",
            "Implement rclpy examples for Python agent bridging"
        ],
        technology_stack=["ROS 2", "Python", "rclpy"],
        prerequisites=["Basic Python programming"]
    )
    
    print(ros2_module)
    
    # Example of creating a simulation environment
    gazebo_env = SimulationEnvironment(
        name="Gazebo Physics Simulation",
        description="Physics simulation environment for robotics testing",
        sim_type="Gazebo",
        components=["Physics Engine", "Sensors", "Robot Models"],
        scenarios=["Navigation", "Manipulation", "SLAM"]
    )
    
    print(gazebo_env)
    
    # Example of creating a learner profile
    learner = LearnerProfile(
        user_id="user123",
        skill_level=DifficultyLevel.INTERMEDIATE
    )
    
    print(learner)
    
    # Update learner progress
    learner.add_progress(ros2_module.id, ModuleStatus.IN_PROGRESS)
    print(f"Updated progress: {learner.progress}")