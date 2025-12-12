from fastapi import APIRouter, HTTPException, Query
from typing import List
import uuid
from rag.db.qdrant_client import qdrant_config
from rag.fastapi_app.models.book_content import BookContent

router = APIRouter()


@router.post(
    "/content/search",
    summary="Search book content",
    description="Search the book content for relevant passages to a query using semantic search",
    responses={
        200: {
            "description": "List of relevant content passages",
            "content": {
                "application/json": {
                    "example": [
                        {
                            "id": "content_1234567890",
                            "title": "Forward Kinematics Explained",
                            "content": "Forward kinematics is the process of determining the position...",
                            "relevance_score": 0.85,
                            "url": "/chapters/kinematics/forward-kinematics"
                        }
                    ]
                }
            }
        },
        500: {"description": "Internal server error"}
    }
)
async def search_content(
    query: str = Query(
        ...,
        description="The search query",
        example="forward kinematics robotics",
        min_length=1,
        max_length=500
    ),
    limit: int = Query(
        5,
        description="Maximum number of results to return",
        ge=1,
        le=20,
        example=5
    )
):
    """
    Search the book content for relevant passages to a query
    """
    try:
        # Search for relevant content in the vector database
        search_results = qdrant_config.search_content(query, limit=limit)

        # Format results for response
        formatted_results = []
        for result in search_results:
            formatted_results.append({
                "id": result["id"],
                "title": result["metadata"].get("title", ""),
                "content": result["content"][:500] + "..." if len(result["content"]) > 500 else result["content"],  # Truncate long content
                "relevance_score": result["score"],
                "url": result["metadata"].get("url", "")
            })

        return formatted_results
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error searching content: {str(e)}")


@router.get(
    "/content/{content_id}",
    summary="Retrieve specific content",
    description="Get the full details of a specific book content item by ID",
    responses={
        200: {"description": "Content item details"},
        404: {"description": "Content not found"},
        500: {"description": "Internal server error"}
    }
)
async def get_content(
    content_id: str,
):
    """
    Retrieve specific content by ID from Qdrant
    """
    try:
        # For now, we'll return an error since Qdrant doesn't directly store full content
        # In a real implementation, you might want to maintain a mapping to retrieve content
        raise HTTPException(status_code=404, detail="Content retrieval by ID requires direct document storage, which is not implemented in this example")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving content: {str(e)}")