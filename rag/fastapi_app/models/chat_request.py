from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime


class ChatRequest(BaseModel):
    """
    Request model for chat interactions, including optional selected text context
    """
    query: str = Field(..., description="The user's query text", min_length=1, max_length=1000)
    session_id: str = Field(..., description="ID to group related queries together")
    selected_text: Optional[str] = Field(default=None, description="Text selected on the page when the query was made", max_length=500)
    context_page: Optional[str] = Field(default=None, description="Page/section where the query originated")
    
    class Config:
        json_schema_extra = {
            "example": {
                "query": "Explain the concept of forward kinematics",
                "session_id": "sess_1234567890",
                "selected_text": "Forward kinematics is the process of determining the position and orientation of the end-effector...",
                "context_page": "/chapters/kinematics/forward-kinematics"
            }
        }