"""
Basic API Infrastructure for Robotics Curriculum Modules

This module sets up the basic API infrastructure with authentication
for the curriculum platform using FastAPI.
"""

from datetime import datetime, timedelta
from typing import Optional, List
from fastapi import FastAPI, HTTPException, Depends, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from fastapi.middleware.cors import CORSMiddleware
from jose import JWTError, jwt
from passlib.context import CryptContext
from pydantic import BaseModel
import uuid


# Initialize FastAPI app
app = FastAPI(
    title="Robotics Curriculum API",
    description="API for the Robotics Curriculum Modules platform",
    version="1.0.0"
)

# Add CORS middleware to allow cross-origin requests
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, specify allowed origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Security settings
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")
security = HTTPBearer()
SECRET_KEY = "your-secret-key-change-in-production"  # Should be loaded from environment
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30

# In-memory storage (replace with database in production)
users_db = {}
modules_db = {}
progress_db = {}


# Pydantic models
class UserCreate(BaseModel):
    username: str
    email: str
    password: str
    first_name: Optional[str] = None
    last_name: Optional[str] = None
    role: str = "student"  # student, instructor, admin


class UserLogin(BaseModel):
    username: str
    password: str


class Token(BaseModel):
    access_token: str
    token_type: str


class CurriculumModule(BaseModel):
    id: str
    title: str
    description: str
    content_type: str  # module, lesson, exercise, documentation, asset, assessment
    content: str
    author: str
    difficulty: str  # beginner, intermediate, advanced
    estimated_duration: int  # in minutes
    tags: List[str]
    prerequisites: List[str]
    status: str  # draft, published, archived
    created_at: datetime
    updated_at: datetime


class ModuleProgress(BaseModel):
    user_id: str
    module_id: str
    status: str  # not_started, in_progress, completed, assessed
    progress_percent: float = 0.0
    score: Optional[float] = None  # for scored modules


class User(BaseModel):
    id: str
    username: str
    email: str
    first_name: Optional[str] = None
    last_name: Optional[str] = None
    role: str = "student"
    created_at: datetime


# Helper functions
def verify_password(plain_password, hashed_password):
    return pwd_context.verify(plain_password, hashed_password)


def get_password_hash(password):
    return pwd_context.hash(password)


def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=15)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt


def decode_token(token: str):
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        username: str = payload.get("sub")
        if username is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Could not validate credentials"
            )
        return username
    except JWTError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials"
        )


async def get_current_user(credentials: HTTPAuthorizationCredentials = Depends(security)):
    try:
        token = credentials.credentials
        username = decode_token(token)
        user = users_db.get(username)
        if user is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="User not found"
            )
        return user
    except Exception:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials"
        )


# API Endpoints
@app.post("/auth/register", response_model=User)
async def register_user(user: UserCreate):
    """Register a new user"""
    if user.username in users_db:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Username already registered"
        )

    hashed_password = get_password_hash(user.password)
    user_id = str(uuid.uuid4())

    db_user = {
        "id": user_id,
        "username": user.username,
        "email": user.email,
        "hashed_password": hashed_password,
        "first_name": user.first_name,
        "last_name": user.last_name,
        "role": user.role,
        "created_at": datetime.utcnow()
    }
    users_db[user.username] = db_user

    return User(
        id=user_id,
        username=user.username,
        email=user.email,
        first_name=user.first_name,
        last_name=user.last_name,
        role=user.role,
        created_at=db_user["created_at"]
    )


@app.post("/auth/login", response_model=Token)
async def login_user(user: UserLogin):
    """Authenticate user and return access token"""
    db_user = users_db.get(user.username)
    if not db_user or not verify_password(user.password, db_user["hashed_password"]):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect username or password"
        )

    access_token_expires = timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    access_token = create_access_token(
        data={"sub": user.username}, expires_delta=access_token_expires
    )

    return {"access_token": access_token, "token_type": "bearer"}


@app.get("/modules")
async def get_modules(difficulty: Optional[str] = None, technology: Optional[str] = None):
    """List all curriculum modules with optional filtering"""
    # In a real implementation, this would query a database
    # For now, we'll return some example modules
    example_modules = [
        {
            "id": "mod-ros2-001",
            "title": "ROS 2",
            "description": "Introduction to Robot Operating System 2",
            "content_type": "module",
            "content": "# Introduction to ROS 2\n\nThis module introduces the fundamental concepts of ROS 2...",
            "author": "Curriculum Team",
            "difficulty": "intermediate",
            "estimated_duration": 20,
            "tags": ["ROS 2", "Python", "rclpy"],
            "prerequisites": [],
            "status": "published",
            "created_at": datetime.utcnow(),
            "updated_at": datetime.utcnow()
        },
        {
            "id": "mod-sim-002",
            "title": "Gazebo & Unity",
            "description": "Simulation environments for robotics",
            "content_type": "module",
            "content": "# Gazebo & Unity\n\nThis module covers...",
            "author": "Curriculum Team",
            "difficulty": "intermediate",
            "estimated_duration": 15,
            "tags": ["Gazebo", "Unity"],
            "prerequisites": ["mod-ros2-001"],
            "status": "published",
            "created_at": datetime.utcnow(),
            "updated_at": datetime.utcnow()
        }
    ]

    # Apply filters if provided
    if difficulty:
        example_modules = [m for m in example_modules if m["difficulty"] == difficulty]

    if technology:
        example_modules = [
            m for m in example_modules
            if technology.lower() in [tag.lower() for tag in m.get("tags", [])]
        ]

    return example_modules


@app.get("/modules/{module_id}")
async def get_module(module_id: str, current_user: dict = Depends(get_current_user)):
    """Get detailed information about a specific module"""
    # In a real implementation, this would query a database
    # For now, return example data based on the module ID
    if module_id == "mod-ros2-001":
        return {
            "id": "mod-ros2-001",
            "title": "ROS 2",
            "description": "Introduction to Robot Operating System 2",
            "content_type": "module",
            "content": "# Introduction to ROS 2\n\nThis module introduces the fundamental concepts of ROS 2...",
            "author": "Curriculum Team",
            "difficulty": "intermediate",
            "estimated_duration": 20,
            "tags": ["ROS 2", "Python", "rclpy"],
            "prerequisites": [],
            "status": "published",
            "created_at": datetime.utcnow(),
            "updated_at": datetime.utcnow()
        }
    elif module_id == "mod-sim-002":
        return {
            "id": "mod-sim-002",
            "title": "Gazebo & Unity",
            "description": "Simulation environments for robotics",
            "content_type": "module",
            "content": "# Gazebo & Unity\n\nThis module covers simulation environments in Gazebo and Unity for robotics applications...",
            "author": "Curriculum Team",
            "difficulty": "intermediate",
            "estimated_duration": 15,
            "tags": ["Gazebo", "Unity"],
            "prerequisites": ["mod-ros2-001"],
            "status": "published",
            "created_at": datetime.utcnow(),
            "updated_at": datetime.utcnow()
        }
    else:
        raise HTTPException(status_code=404, detail="Module not found")


@app.get("/users/{user_id}/progress")
async def get_user_progress(user_id: str, current_user: dict = Depends(get_current_user)):
    """Get the learning progress for a specific user"""
    # Check if user is requesting their own progress or is an admin
    if current_user["id"] != user_id and current_user["role"] != "admin":
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Not authorized to view this user's progress"
        )

    # In a real implementation, this would query a database
    # For now, return example data
    if user_id == "user-123":
        return {
            "userId": user_id,
            "modulesCompleted": ["mod-ros2-001"],
            "currentModule": "mod-sim-002",
            "progressPercent": 45.0,
            "achievements": [
                {
                    "id": "ach-001",
                    "name": "First Steps in ROS 2",
                    "description": "Complete the first ROS 2 module",
                    "dateEarned": "2025-01-15T10:30:00Z"
                }
            ],
            "assessmentScores": [
                {
                    "moduleId": "mod-ros2-001",
                    "score": 85.0,
                    "dateTaken": "2025-01-14T15:22:00Z"
                }
            ]
        }
    else:
        return {
            "userId": user_id,
            "modulesCompleted": [],
            "currentModule": None,
            "progressPercent": 0.0,
            "achievements": [],
            "assessmentScores": []
        }


@app.put("/users/{user_id}/progress")
async def update_user_progress(
    user_id: str,
    progress_update: ModuleProgress,
    current_user: dict = Depends(get_current_user)
):
    """Update the learning progress for a specific user"""
    # Check if user is updating their own progress or is an admin
    if current_user["id"] != user_id and current_user["role"] != "admin":
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Not authorized to update this user's progress"
        )

    # In a real implementation, this would update a database
    # For now, store in memory
    progress_key = f"{user_id}:{progress_update.module_id}"
    progress_db[progress_key] = {
        "user_id": progress_update.user_id,
        "module_id": progress_update.module_id,
        "status": progress_update.status,
        "progress_percent": progress_update.progress_percent,
        "score": progress_update.score,
        "updated_at": datetime.utcnow()
    }

    return {
        "message": "Progress updated successfully",
        "progress": progress_db[progress_key]
    }


@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {"status": "healthy", "timestamp": datetime.utcnow()}


# Run the application with uvicorn if this file is executed directly
if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)