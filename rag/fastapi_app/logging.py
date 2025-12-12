import logging
import sys
from datetime import datetime
from enum import Enum
from typing import Any, Dict, Optional
import json


class LogLevel(Enum):
    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"


class Logger:
    def __init__(self, name: str = "RAGLogger", level: LogLevel = LogLevel.INFO):
        self.logger = logging.getLogger(name)
        self.logger.setLevel(getattr(logging, level.value))
        
        # Create formatter
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        
        # Create console handler
        if not self.logger.handlers:  # Avoid adding multiple handlers
            console_handler = logging.StreamHandler(sys.stdout)
            console_handler.setFormatter(formatter)
            self.logger.addHandler(console_handler)
    
    def _log(self, level: LogLevel, message: str, extra: Optional[Dict[str, Any]] = None):
        """Internal method to log messages with optional extra data"""
        if extra:
            message = f"{message} | Extra: {json.dumps(extra, default=str)}"
        
        getattr(self.logger, level.value.lower())(message)
    
    def debug(self, message: str, extra: Optional[Dict[str, Any]] = None):
        self._log(LogLevel.DEBUG, message, extra)
    
    def info(self, message: str, extra: Optional[Dict[str, Any]] = None):
        self._log(LogLevel.INFO, message, extra)
    
    def warning(self, message: str, extra: Optional[Dict[str, Any]] = None):
        self._log(LogLevel.WARNING, message, extra)
    
    def error(self, message: str, extra: Optional[Dict[str, Any]] = None):
        self._log(LogLevel.ERROR, message, extra)
    
    def critical(self, message: str, extra: Optional[Dict[str, Any]] = None):
        self._log(LogLevel.CRITICAL, message, extra)


# Global logger instance
rag_logger = Logger()


# Performance monitoring decorator
def monitor_performance(func):
    """Decorator to log execution time of functions"""
    async def wrapper(*args, **kwargs):
        start_time = datetime.now()
        try:
            result = await func(*args, **kwargs)
            execution_time = (datetime.now() - start_time).total_seconds()
            rag_logger.info(
                f"Performance: {func.__name__}",
                extra={
                    "execution_time": f"{execution_time:.4f}s",
                    "args_count": len(args),
                    "kwargs_count": len(kwargs)
                }
            )
            return result
        except Exception as e:
            execution_time = (datetime.now() - start_time).total_seconds()
            rag_logger.error(
                f"Performance Error: {func.__name__}",
                extra={
                    "execution_time": f"{execution_time:.4f}s",
                    "error": str(e)
                }
            )
            raise
    
    return wrapper


# Request logging middleware for FastAPI
async def log_request_middleware(request, call_next):
    """Middleware to log incoming requests"""
    start_time = datetime.now()
    
    # Log the incoming request
    rag_logger.info(
        f"Request: {request.method} {request.url.path}",
        extra={
            "url": str(request.url),
            "method": request.method,
            "client": request.client.host
        }
    )
    
    response = await call_next(request)
    
    execution_time = (datetime.now() - start_time).total_seconds()
    
    rag_logger.info(
        f"Response: {response.status_code}",
        extra={
            "status_code": response.status_code,
            "execution_time": f"{execution_time:.4f}s"
        }
    )
    
    return response


# Usage example functions
def log_user_query(query_id: str, user_id: Optional[str], query_text: str):
    """Log when a user submits a query"""
    rag_logger.info(
        f"User query received",
        extra={
            "query_id": query_id,
            "user_id": user_id,
            "query_text": query_text[:100]  # Limit length for logs
        }
    )


def log_content_search(query: str, results_count: int, search_time: float):
    """Log content search operations"""
    rag_logger.info(
        f"Content search performed",
        extra={
            "query": query[:100],
            "results_count": results_count,
            "search_time": f"{search_time:.4f}s"
        }
    )


def log_chat_response(response_id: str, query_id: str, response_length: int):
    """Log chat responses"""
    rag_logger.info(
        f"Chat response generated",
        extra={
            "response_id": response_id,
            "query_id": query_id,
            "response_length": response_length
        }
    )


def log_error(error_type: str, error_message: str, context: Optional[Dict] = None):
    """Log errors with context"""
    rag_logger.error(
        f"Error occurred: {error_type}",
        extra={
            "error_message": error_message,
            "context": context
        }
    )