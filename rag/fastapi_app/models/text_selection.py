from pydantic import BaseModel, Field
from typing import Optional


class TextSelection(BaseModel):
    """
    Represents highlighted text on a page that serves as context for targeted queries to the chatbot
    """
    id: str = Field(..., description="Unique identifier for the selection")
    content_id: str = Field(..., description="ID of the content where text was selected")
    text: str = Field(..., description="The actual selected text", min_length=1)
    start_position: int = Field(..., description="Character position where selection starts", ge=0)
    end_position: int = Field(..., description="Character position where selection ends", gt=0)
    html_element: Optional[str] = Field(default=None, description="HTML element type where text was selected")
    paragraph_number: Optional[int] = Field(default=None, description="Paragraph number within the content", ge=1)
    
    class Config:
        json_schema_extra = {
            "example": {
                "id": "selection_1234567890",
                "content_id": "content_1234567890",
                "text": "Forward kinematics is the process of determining the position and orientation of the end-effector based on the joint angles.",
                "start_position": 100,
                "end_position": 200,
                "html_element": "p",
                "paragraph_number": 3
            }
        }