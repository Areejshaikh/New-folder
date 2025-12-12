from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime


class BookContent(BaseModel):
    """
    Represents educational material including chapters, modules, lessons, text, images, diagrams, 
    code snippets, and practice questions with associated metadata
    """
    id: str = Field(..., description="Unique identifier for the content piece")
    title: str = Field(..., description="Title of the content piece")
    description: str = Field(..., description="Brief description of the content")
    keywords: List[str] = Field(..., description="Keywords associated with the content")
    tags: List[str] = Field(..., description="Tags for categorization")
    type: str = Field(..., description="Type of content (e.g., 'chapter', 'module', 'lesson', 'diagram', 'code')")
    content: str = Field(..., description="The actual content in markdown format")
    parent_id: Optional[str] = Field(default=None, description="ID of parent content (optional, null for root level)")
    order: int = Field(..., description="Order position in the hierarchy", ge=1)
    created_at: datetime = Field(default_factory=datetime.utcnow, description="Creation timestamp")
    updated_at: datetime = Field(default_factory=datetime.utcnow, description="Last update timestamp")
    version: str = Field(..., description="Content version")
    
    class Config:
        json_schema_extra = {
            "example": {
                "id": "content_1234567890",
                "title": "Chapter 5: Forward Kinematics",
                "description": "An introduction to forward kinematics in robotics",
                "keywords": ["kinematics", "robotics", "forward"],
                "tags": ["robotics", "kinematics", "theoretical"],
                "type": "chapter",
                "content": "# Forward Kinematics\n\nForward kinematics is the process of determining the position...",
                "parent_id": None,
                "order": 5,
                "version": "1.0.0"
            }
        }