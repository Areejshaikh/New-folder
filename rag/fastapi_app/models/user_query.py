from pydantic import BaseModel, Field
from typing import Optional
from datetime import datetime


class UserQuery(BaseModel):
    """
    Represents questions or requests submitted by readers to the RAG chatbot system
    """
    id: str = Field(..., description="Unique identifier for the query")
    user_id: Optional[str] = Field(default=None, description="ID of the user who submitted the query (optional for anonymous)")
    query_text: str = Field(..., description="The text of the user's query", min_length=1, max_length=1000)
    selected_text: Optional[str] = Field(default=None, description="Text selected on the page when the query was made", max_length=500)
    context_page: Optional[str] = Field(default=None, description="Page/section where the query originated")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Time when the query was submitted")
    session_id: str = Field(..., description="ID to group related queries together")
    
    class Config:
        json_schema_extra = {
            "example": {
                "id": "query_0987654321",
                "user_id": "user_abcdef123456",
                "query_text": "Explain the concept of forward kinematics",
                "selected_text": "Forward kinematics is the process of determining the position and orientation of the end-effector...",
                "context_page": "/chapters/kinematics/forward-kinematics",
                "session_id": "sess_1234567890"
            }
        }