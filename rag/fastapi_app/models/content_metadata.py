from pydantic import BaseModel, Field
from typing import List, Optional
from datetime import datetime


class ContentMetadata(BaseModel):
    """
    Represents structured information about book content including id, title, description, keywords, and tags
    """
    content_id: str = Field(..., description="Reference to the book content")
    version: str = Field(..., description="Version of the content these metadata apply to")
    authors: Optional[List[str]] = Field(default=None, description="Authors of the content")
    reviewers: Optional[List[str]] = Field(default=None, description="Reviewers who have approved the content")
    status: str = Field(..., description="Status of the content ('draft', 'review', 'published')")
    language: Optional[str] = Field(default="en", description="Language of the content")
    estimated_reading_time: Optional[int] = Field(default=None, description="Estimated reading time in minutes", ge=1)
    related_content_ids: Optional[List[str]] = Field(default=None, description="IDs of related content pieces")
    created_at: Optional[datetime] = Field(default_factory=datetime.utcnow, description="When the metadata was created")
    updated_at: Optional[datetime] = Field(default_factory=datetime.utcnow, description="When the metadata was last updated")
    
    class Config:
        json_schema_extra = {
            "example": {
                "content_id": "content_1234567890",
                "version": "1.0.0",
                "authors": ["Author One", "Author Two"],
                "reviewers": ["Reviewer One"],
                "status": "published",
                "language": "en",
                "estimated_reading_time": 15,
                "related_content_ids": ["content_0987654321", "content_1122334455"]
            }
        }