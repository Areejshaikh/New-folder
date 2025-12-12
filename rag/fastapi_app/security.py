"""
Security best practices implementation for the RAG Chatbot API
"""
from datetime import timedelta
import secrets
from fastapi import Request, Response, HTTPException, status
from fastapi.security import HTTPBearer
import hashlib
import hmac
from typing import Optional
import re
from urllib.parse import urlparse


class SecurityConfig:
    """Security configuration and utilities"""
    
    # Define allowed origins for CORS
    ALLOWED_ORIGINS = [
        "http://localhost:3000",
        "http://localhost:3001", 
        "https://your-book-domain.github.io",
        # Add your production domains here
    ]
    
    # Rate limiting configuration
    RATE_LIMIT_REQUESTS = 100  # requests
    RATE_LIMIT_WINDOW = timedelta(minutes=15)  # per 15 minutes
    
    # Input validation patterns
    USERNAME_PATTERN = r'^[a-zA-Z0-9_.-]{3,30}$'
    EMAIL_PATTERN = r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$'
    
    @staticmethod
    def validate_input(input_str: str, max_length: int = 1000) -> bool:
        """Basic input validation"""
        if not input_str or len(input_str) > max_length:
            return False
        
        # Check for dangerous patterns
        dangerous_patterns = [
            r'<script',  # XSS attempts
            r'javascript:',  # JavaScript injection
            r'on\w+\s*=',  # Event handlers
            r'<iframe',  # Frame injection
        ]
        
        input_lower = input_str.lower()
        for pattern in dangerous_patterns:
            if re.search(pattern, input_lower):
                return False
        
        return True
    
    @staticmethod
    def sanitize_input(input_str: str) -> str:
        """Basic input sanitization"""
        # Remove potentially dangerous characters/sequences
        sanitized = input_str.replace('<script', '&lt;script')
        sanitized = sanitized.replace('javascript:', 'javascript_')
        sanitized = sanitized.replace('vbscript:', 'vbscript_')
        sanitized = sanitized.replace('<iframe', '&lt;iframe')
        sanitized = sanitized.replace('onerror', 'on_error')
        sanitized = sanitized.replace('onload', 'on_load')
        
        return sanitized


def add_security_headers(response: Response) -> Response:
    """Add security headers to response"""
    response.headers["X-Content-Type-Options"] = "nosniff"
    response.headers["X-Frame-Options"] = "DENY"  # Prevent clickjacking
    response.headers["X-XSS-Protection"] = "1; mode=block"
    response.headers["Referrer-Policy"] = "strict-origin-when-cross-origin"
    response.headers["Strict-Transport-Security"] = "max-age=31536000; includeSubDomains"
    response.headers["Content-Security-Policy"] = "default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' data: https:; font-src 'self' https:;"
    
    return response


async def security_middleware(request: Request, call_next):
    """Security middleware for request processing"""
    # Check for suspicious headers
    suspicious_headers = [
        'x-forwarded-for',
        'x-real-ip',
        'x-client-ip',
        'x-originating-ip'
    ]
    
    for header in suspicious_headers:
        if header in request.headers:
            # Log potential proxy/header manipulation attempts
            print(f"Warning: Suspicious header {header} found in request")
    
    # Add security headers to response
    response = await call_next(request)
    response = add_security_headers(response)
    
    return response


def rate_limit_check(identifier: str) -> tuple[bool, Optional[str]]:
    """
    Check if the identifier has exceeded rate limit.
    This is a simplified implementation; in production you'd use Redis or similar.
    """
    # In a real implementation, you would:
    # 1. Store request counts in Redis with expiration
    # 2. Check against configured limits
    # 3. Return whether the limit is exceeded
    return True, None  # Placeholder - always allow


def generate_csrf_token() -> str:
    """Generate a CSRF token"""
    return secrets.token_urlsafe(32)


def validate_csrf_token(token: str, expected: str) -> bool:
    """Validate a CSRF token"""
    return hmac.compare_digest(token, expected)


def hash_password(password: str, salt: Optional[str] = None) -> tuple[str, str]:
    """Hash a password with a salt"""
    if salt is None:
        salt = secrets.token_hex(32)
    
    # Use PBKDF2 with SHA-256
    pwdhash = hashlib.pbkdf2_hmac('sha256', 
                                  password.encode('utf-8'), 
                                  salt.encode('ascii'), 
                                  100000)
    pwdhash = pwdhash.hex()
    
    return pwdhash, salt


def verify_password(stored_password: str, stored_salt: str, provided_password: str) -> bool:
    """Verify a provided password against the stored hash"""
    pwdhash, _ = hash_password(provided_password, stored_salt)
    return hmac.compare_digest(pwdhash, stored_password)