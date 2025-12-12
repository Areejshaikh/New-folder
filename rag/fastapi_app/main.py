from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from rag.fastapi_app.routers import chat, content
from rag.db.postgres_client import postgres_config
from rag.fastapi_app.exceptions import add_exception_handlers
import asyncio
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Initialize FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="API for the RAG (Retrieval Augmented Generation) chatbot system integrated with the book content",
    version="1.0.0"
)

# Add exception handlers
add_exception_handlers(app)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(chat.router, prefix="/v1", tags=["chat"])
app.include_router(content.router, prefix="/v1", tags=["content"])

@app.on_event("startup")
async def startup_event():
    """Initialize database connection on startup"""
    postgres_config.connect()

@app.get("/")
def read_root():
    return {"Hello": "Welcome to the RAG Chatbot API"}

@app.get("/health")
def health_check():
    return {"status": "healthy"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)