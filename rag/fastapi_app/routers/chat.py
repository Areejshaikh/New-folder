from fastapi import APIRouter, HTTPException, Depends, Query
from typing import Optional
import uuid
from datetime import datetime
from rag.fastapi_app.models.user_query import UserQuery
from rag.fastapi_app.models.chatbot_response import ChatbotResponse
from rag.api.chat_handler import process_query_with_rag

router = APIRouter()


@router.post(
    "/chat",
    response_model=ChatbotResponse,
    summary="Submit a query to the RAG chatbot",
    description=(
        "Process a user query against the book content and return a contextual response. "
        "The system will use RAG (Retrieval Augmented Generation) to find relevant content "
        "and generate an appropriate response with citations."
    ),
    responses={
        200: {
            "description": "Successful response from the chatbot",
            "content": {
                "application/json": {
                    "example": {
                        "id": "resp_1234567890",
                        "query_id": "query_0987654321",
                        "response_text": "Forward kinematics is the process of calculating the position...",
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
                        "timestamp": "2023-10-01T12:00:00Z",
                        "model_used": "gpt-4"
                    }
                }
            }
        },
        400: {"description": "Bad request due to invalid input"},
        500: {"description": "Internal server error"}
    }
)
async def chat_endpoint(
    query: str = Query(
        ...,
        description="The user's query text",
        example="Explain the concept of forward kinematics",
        min_length=1,
        max_length=1000
    ),
    session_id: str = Query(
        ...,
        description="Unique identifier for the chat session",
        example="sess_1234567890"
    ),
    selected_text: Optional[str] = Query(
        None,
        description="Optional text that was selected on the page when the query was made",
        example="Forward kinematics is the process of determining the position and orientation...",
        max_length=500
    ),
    context_page: Optional[str] = Query(
        None,
        description="Optional page/section where the query originated",
        example="/chapters/kinematics/forward-kinematics"
    )
):
    """
    Process a user query against the book content and return a contextual response
    """
    try:
        # Generate a unique ID for this query
        query_id = str(uuid.uuid4())

        # Create the query object
        user_query = UserQuery(
            id=query_id,
            query_text=query,
            selected_text=selected_text,
            context_page=context_page,
            timestamp=datetime.utcnow(),
            session_id=session_id
        )

        # Process the query using the RAG system
        response = await process_query_with_rag(user_query)

        return response

    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")


@router.get(
    "/chat/history",
    summary="Retrieve chat history for a session",
    description="Get the conversation history for a specific session",
    responses={
        200: {
            "description": "List of chat messages in the session",
            "content": {
                "application/json": {
                    "example": {
                        "session_id": "sess_1234567890",
                        "history": [
                            {
                                "query": "What is RAG?",
                                "query_timestamp": "2023-10-01T12:00:00Z",
                                "response": "RAG stands for Retrieval Augmented Generation...",
                                "response_timestamp": "2023-10-01T12:00:05Z"
                            }
                        ]
                    }
                }
            }
        },
        404: {"description": "Session not found"},
        500: {"description": "Internal server error"}
    }
)
async def get_chat_history(
    session_id: str = Query(
        ...,
        description="Unique identifier for the chat session",
        example="sess_1234567890"
    )
):
    """
    Retrieve chat history for a specific session
    """
    from rag.db.postgres_client import postgres_config

    try:
        history = await postgres_config.get_chat_history(session_id)
        return {"session_id": session_id, "history": history}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving chat history: {str(e)}")