from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime


class Citation(BaseModel):
    """
    Citation object structure for referencing book content
    """
    content_id: str = Field(..., description="ID of the referenced book content")
    title: str = Field(..., description="Title of the referenced content")
    url: str = Field(..., description="URL to the referenced content")


class ChatbotResponse(BaseModel):
    """
    Represents answers and information provided by the AI system to user queries, 
    including citations and follow-up suggestions
    """
    id: str = Field(..., description="Unique identifier for the response")
    query_id: str = Field(..., description="ID of the corresponding user query")
    response_text: str = Field(..., description="The text of the chatbot's response", min_length=1)
    citations: Optional[List[Citation]] = Field(default=None, description="List of citations to book content")
    follow_up_questions: Optional[List[str]] = Field(default=None, description="Suggested follow-up questions")
    confidence_score: Optional[float] = Field(default=None, description="Confidence level of the response (0-1)", ge=0.0, le=1.0)
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Time when the response was generated")
    model_used: str = Field(..., description="AI model used to generate the response")
    
    class Config:
        json_schema_extra = {
            "example": {
                "id": "resp_1234567890",
                "query_id": "query_0987654321",
                "response_text": "Forward kinematics is the process of calculating the position and orientation of a robot's end-effector based on the known joint angles.",
                "citations": [
                    {
                        "content_id": "content_1234567890",
                        "title": "Forward Kinematics Explained",
                        "url": "/chapters/kinematics/forward-kinematics"
                    }
                ],
                "follow_up_questions": [
                    "What is inverse kinematics?",
                    "How is forward kinematics used in robotics?"
                ],
                "confidence_score": 0.92,
                "model_used": "gpt-4"
            }
        }