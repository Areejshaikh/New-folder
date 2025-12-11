"""
API Endpoint for Serving Curriculum Modules

This module implements the /modules endpoint to serve curriculum modules
as part of the ROS 2 curriculum platform.
"""

from fastapi import FastAPI, HTTPException, Query
from typing import List, Optional, Dict, Any
from pydantic import BaseModel
from enum import Enum
import uuid
from datetime import datetime


class DifficultyLevel(str, Enum):
    """Enumeration of difficulty levels"""
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class ModuleStatus(str, Enum):
    """Enumeration of module statuses"""
    DRAFT = "draft"
    REVIEWED = "reviewed"
    PUBLISHED = "published"
    ARCHIVED = "archived"


class TechnologyStack(str, Enum):
    """Enumeration of technology stacks used in modules"""
    ROS2 = "ROS 2"
    GAZEBO = "Gazebo"
    UNITY = "Unity"
    NVIDIA_ISAAC = "NVIDIA Isaac"
    OPENAI_WHISPER = "OpenAI Whisper"


class CurriculumModule(BaseModel):
    """
    Pydantic model for curriculum modules
    """
    id: str
    title: str
    description: str
    content_type: str  # module, lesson, exercise
    difficulty: DifficultyLevel
    duration: int  # estimated duration in minutes
    learning_objectives: List[str]
    prerequisites: List[str]
    technology_stack: List[TechnologyStack]
    content_url: str
    assets: List[str]
    status: ModuleStatus
    created_at: datetime
    updated_at: datetime


class ModuleResponse(BaseModel):
    """
    Response model for the modules endpoint
    """
    modules: List[CurriculumModule]
    total_count: int
    page: int
    page_size: int


# Initialize FastAPI app
app = FastAPI(
    title="PAHR Book API",
    description="API for serving robotics curriculum modules",
    version="1.0.0"
)


# In-memory database for demonstration (would use actual DB in production)
modules_db = [
    CurriculumModule(
        id="mod-ros2-001",
        title="Introduction to ROS 2",
        description="The Robotic Nervous System (ROS 2)",
        content_type="module",
        difficulty=DifficultyLevel.INTERMEDIATE,
        duration=120,
        learning_objectives=[
            "Understand ROS 2 architecture (nodes, topics, services)",
            "Implement a simple robot controller using rclpy",
            "Connect Python agents to ROS controllers"
        ],
        prerequisites=["Basic Python programming", "Fundamentals of robotics"],
        technology_stack=[TechnologyStack.ROS2],
        content_url="/docs/modules/01-the-robotic-nervous-system-ros2",
        assets=["/assets/ros2-architecture-diagram.png", "/assets/simple-node-example.py"],
        status=ModuleStatus.PUBLISHED,
        created_at=datetime(2025, 1, 15),
        updated_at=datetime(2025, 1, 20)
    ),
    CurriculumModule(
        id="mod-sim-002",
        title="Simulation Environments",
        description="Digital Twin with Gazebo & Unity",
        content_type="module",
        difficulty=DifficultyLevel.INTERMEDIATE,
        duration=150,
        learning_objectives=[
            "Set up physics simulations in Gazebo",
            "Create high-fidelity environments in Unity",
            "Implement sensor simulation (LiDAR, Depth Cameras, IMUs)"
        ],
        prerequisites=["mod-ros2-001"],
        technology_stack=[TechnologyStack.GAZEBO, TechnologyStack.UNITY],
        content_url="/docs/modules/02-simulation-environments",
        assets=["/assets/gazebo-world-example.world", "/assets/unity-scene-example.unity"],
        status=ModuleStatus.PUBLISHED,
        created_at=datetime(2025, 1, 16),
        updated_at=datetime(2025, 1, 21)
    ),
    CurriculumModule(
        id="mod-ai-003",
        title="AI Integration",
        description="The AI-Robot Brain (NVIDIA Isaac)",
        content_type="module",
        difficulty=DifficultyLevel.ADVANCED,
        duration=180,
        learning_objectives=[
            "Implement synthetic data generation with Isaac Sim",
            "Create VSLAM examples using Isaac ROS",
            "Implement Nav2 path planning for humanoid movement"
        ],
        prerequisites=["mod-ros2-001", "mod-sim-002"],
        technology_stack=[TechnologyStack.NVIDIA_ISAAC],
        content_url="/docs/modules/03-ai-integration",
        assets=["/assets/isaac-sim-examples.sdf", "/assets/vslam-implementation.py"],
        status=ModuleStatus.DRAFT,
        created_at=datetime(2025, 1, 17),
        updated_at=datetime(2025, 1, 17)
    ),
    CurriculumModule(
        id="mod-vla-004",
        title="Voice Command Processing",
        description="Vision-Language-Action (VLA)",
        content_type="module",
        difficulty=DifficultyLevel.ADVANCED,
        duration=100,
        learning_objectives=[
            "Implement voice-to-action capabilities using OpenAI Whisper",
            "Connect natural language processing to robot actions",
            "Create cognitive planning for natural language to ROS 2 actions"
        ],
        prerequisites=["mod-ros2-001"],
        technology_stack=[TechnologyStack.OPENAI_WHISPER],
        content_url="/docs/modules/04-voice-command-processing",
        assets=["/assets/voice-command-flowchart.png", "/assets/nlp-ros-bridge.py"],
        status=ModuleStatus.DRAFT,
        created_at=datetime(2025, 1, 18),
        updated_at=datetime(2025, 1, 18)
    )
]


@app.get("/modules", response_model=ModuleResponse)
async def get_modules(
    difficulty: Optional[DifficultyLevel] = Query(None, description="Filter by difficulty level"),
    technology: Optional[TechnologyStack] = Query(None, description="Filter by technology stack"),
    status: Optional[ModuleStatus] = Query(None, description="Filter by module status"),
    page: int = Query(1, ge=1, description="Page number for pagination"),
    page_size: int = Query(10, ge=1, le=100, description="Number of modules per page")
):
    """
    Retrieve list of curriculum modules with optional filtering
    """
    # Apply filters if provided
    filtered_modules = []
    for module in modules_db:
        include_module = True

        # Apply difficulty filter
        if difficulty and module.difficulty != difficulty:
            include_module = False

        # Apply technology filter - correcting the typo
        if technology and technology.value not in [tech.value for tech in module.technology_stack]:
            include_module = False

        # Apply status filter
        if status and module.status != status:
            include_module = False

        if include_module:
            filtered_modules.append(module)

    # Apply pagination
    start_idx = (page - 1) * page_size
    end_idx = start_idx + page_size
    paginated_modules = filtered_modules[start_idx:end_idx]

    return ModuleResponse(
        modules=paginated_modules,
        total_count=len(filtered_modules),
        page=page,
        page_size=page_size
    )


@app.get("/modules/{module_id}", response_model=CurriculumModule)
async def get_module(module_id: str):
    """
    Retrieve details of a specific curriculum module by ID

    Args:
        module_id: The ID of the module to retrieve

    Returns:
        Curriculum module details

    Raises:
        HTTPException: If module with given ID is not found
    """
    for module in modules_db:
        if module.id == module_id:
            return module

    raise HTTPException(status_code=404, detail=f"Module with ID {module_id} not found")


class ModuleProgressUpdate(BaseModel):
    """
    Pydantic model for updating module progress
    """
    user_id: str
    module_id: str
    status: str  # not_started, in_progress, completed, assessed
    progress_percent: float = 0.0
    score: Optional[float] = None  # for scored modules


class UserProgressResponse(BaseModel):
    """
    Response model for user progress data
    """
    user_id: str
    modules_completed: List[str]
    current_module: Optional[str] = None
    progress_percent: float
    achievements: List[Dict[str, Any]]
    assessment_scores: List[Dict[str, Any]]
    last_updated: datetime


# In-memory storage for user progress (would use actual DB in production)
user_progress_db = {}


@app.post("/users/{user_id}/progress", response_model=Dict[str, str])
async def update_user_progress(user_id: str, progress_update: ModuleProgressUpdate):
    """
    Update user progress for a specific module

    Args:
        user_id: The ID of the user
        progress_update: Progress update data

    Returns:
        Confirmation message
    """
    # Initialize user progress data if it doesn't exist
    if user_id not in user_progress_db:
        user_progress_db[user_id] = {
            "user_id": user_id,
            "modules_progress": {},
            "achievements": [],
            "assessment_scores": [],
            "last_updated": datetime.now()
        }

    # Update progress for the specific module
    user_progress_db[user_id]["modules_progress"][progress_update.module_id] = {
        "module_id": progress_update.module_id,
        "status": progress_update.status,
        "progress_percent": progress_update.progress_percent,
        "score": progress_update.score,
        "updated_at": datetime.now()
    }

    # If the module is completed, add to achievements if not already there
    if progress_update.status == "completed":
        achievement_id = f"completed-{progress_update.module_id}"
        achievement_exists = any(a.get("id") == achievement_id for a in user_progress_db[user_id]["achievements"])
        if not achievement_exists:
            user_progress_db[user_id]["achievements"].append({
                "id": achievement_id,
                "name": f"Completed {progress_update.module_id}",
                "description": f"Successfully completed the {progress_update.module_id} module",
                "date_earned": datetime.now()
            })

    # If a score is provided, add to assessment scores
    if progress_update.score is not None:
        user_progress_db[user_id]["assessment_scores"].append({
            "module_id": progress_update.module_id,
            "score": progress_update.score,
            "date_taken": datetime.now()
        })

    user_progress_db[user_id]["last_updated"] = datetime.now()

    return {"message": f"Progress updated successfully for user {user_id} and module {progress_update.module_id}"}


@app.get("/users/{user_id}/progress", response_model=UserProgressResponse)
async def get_user_progress(user_id: str):
    """
    Get user progress information

    Args:
        user_id: The ID of the user to retrieve progress for

    Returns:
        User progress information
    """
    if user_id not in user_progress_db:
        # Return default empty progress
        return UserProgressResponse(
            user_id=user_id,
            modules_completed=[],
            current_module=None,
            progress_percent=0.0,
            achievements=[],
            assessment_scores=[],
            last_updated=datetime.now()
        )

    user_data = user_progress_db[user_id]

    # Calculate modules completed and progress percent
    modules_completed = []
    total_progress = 0
    modules_count = len(user_data["modules_progress"])

    for module_id, progress_data in user_data["modules_progress"].items():
        if progress_data["status"] == "completed":
            modules_completed.append(module_id)
        total_progress += progress_data["progress_percent"]

    overall_progress = total_progress / modules_count if modules_count > 0 else 0.0

    # Determine current module (the one in progress)
    current_module = None
    for module_id, progress_data in user_data["modules_progress"].items():
        if progress_data["status"] == "in_progress":
            current_module = module_id
            break

    return UserProgressResponse(
        user_id=user_id,
        modules_completed=modules_completed,
        current_module=current_module,
        progress_percent=overall_progress,
        achievements=user_data["achievements"],
        assessment_scores=user_data["assessment_scores"],
        last_updated=user_data["last_updated"]
    )


@app.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {
        "status": "healthy",
        "service": "curriculum-modules-api",
        "timestamp": datetime.now()
    }


# For testing purposes when running this file directly
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)