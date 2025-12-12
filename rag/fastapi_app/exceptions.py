from fastapi import FastAPI, Request
from fastapi.responses import JSONResponse
from fastapi.exceptions import RequestValidationError
from starlette.exceptions import HTTPException as StarletteHTTPException


class RAGException(Exception):
    """Base exception class for RAG system"""
    def __init__(self, message: str, status_code: int = 500):
        self.message = message
        self.status_code = status_code
        super().__init__(self.message)


class ContentNotFound(RAGException):
    """Raised when requested content is not found"""
    def __init__(self, message: str = "Content not found"):
        super().__init__(message, 404)


class InvalidQuery(RAGException):
    """Raised when query is invalid or malformed"""
    def __init__(self, message: str = "Invalid query"):
        super().__init__(message, 400)


class DatabaseError(RAGException):
    """Raised when database operations fail"""
    def __init__(self, message: str = "Database error"):
        super().__init__(message, 500)


class VectorDBError(RAGException):
    """Raised when vector database operations fail"""
    def __init__(self, message: str = "Vector database error"):
        super().__init__(message, 500)


class AIClientError(RAGException):
    """Raised when AI client (e.g., OpenAI) operations fail"""
    def __init__(self, message: str = "AI client error"):
        super().__init__(message, 500)


def add_exception_handlers(app: FastAPI):
    """Add exception handlers to the FastAPI app"""
    
    @app.exception_handler(StarletteHTTPException)
    async def http_exception_handler(request: Request, exc: StarletteHTTPException):
        return JSONResponse(
            status_code=exc.status_code,
            content={"detail": exc.detail}
        )
    
    @app.exception_handler(RequestValidationError)
    async def validation_exception_handler(request: Request, exc: RequestValidationError):
        return JSONResponse(
            status_code=422,
            content={
                "detail": "Validation error",
                "errors": [
                    {
                        "loc": error["loc"],
                        "msg": error["msg"],
                        "type": error["type"]
                    }
                    for error in exc.errors()
                ]
            }
        )
    
    @app.exception_handler(RAGException)
    async def rag_exception_handler(request: Request, exc: RAGException):
        return JSONResponse(
            status_code=exc.status_code,
            content={"detail": exc.message}
        )
    
    @app.exception_handler(Exception)
    async def general_exception_handler(request: Request, exc: Exception):
        return JSONResponse(
            status_code=500,
            content={"detail": "Internal server error"}
        )