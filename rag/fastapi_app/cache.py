import asyncio
import hashlib
from typing import Any, Optional
from datetime import datetime, timedelta
from functools import wraps

# Simple in-memory cache implementation
# In production, you'd use Redis or another caching solution
class SimpleCache:
    def __init__(self):
        self._cache = {}
        self._expirations = {}
    
    def set(self, key: str, value: Any, ttl: int = 300) -> None:
        """Set a value in cache with TTL in seconds"""
        self._cache[key] = value
        self._expirations[key] = datetime.now() + timedelta(seconds=ttl)
    
    def get(self, key: str) -> Optional[Any]:
        """Get a value from cache, return None if not found or expired"""
        # Check if key exists and hasn't expired
        if key in self._expirations:
            if datetime.now() < self._expirations[key]:
                return self._cache[key]
            else:
                # Remove expired key
                del self._cache[key]
                del self._expirations[key]
        return None
    
    def delete(self, key: str) -> bool:
        """Delete a key from cache"""
        if key in self._cache:
            del self._cache[key]
            if key in self._expirations:
                del self._expirations[key]
            return True
        return False
    
    def clear(self) -> None:
        """Clear all cache"""
        self._cache.clear()
        self._expirations.clear()

# Global cache instance
cache = SimpleCache()


def cached(ttl: int = 300):
    """Decorator to cache function results"""
    def decorator(func):
        @wraps(func)
        async def wrapper(*args, **kwargs):
            # Create a cache key based on function name and arguments
            key_parts = [func.__name__] + list(args)
            for k, v in sorted(kwargs.items()):
                key_parts.append(f"{k}:{v}")
            key = hashlib.md5(str(key_parts).encode()).hexdigest()
            
            # Try to get from cache first
            result = cache.get(key)
            if result is not None:
                return result
            
            # If not in cache, call function and cache result
            result = await func(*args, **kwargs)
            cache.set(key, result, ttl)
            return result
        
        return wrapper
    return decorator


def cache_key(prefix: str, *args, **kwargs) -> str:
    """Generate a cache key with a prefix"""
    key_parts = [prefix] + list(args)
    for k, v in sorted(kwargs.items()):
        key_parts.append(f"{k}:{v}")
    return hashlib.md5(str(key_parts).encode()).hexdigest()


# Example usage functions
async def get_content_from_cache(content_id: str) -> Optional[str]:
    """Example function to get content from cache"""
    # Generate cache key
    key = cache_key("content", content_id)
    return cache.get(key)


async def set_content_in_cache(content_id: str, content: str, ttl: int = 600) -> bool:
    """Example function to set content in cache"""
    # Generate cache key
    key = cache_key("content", content_id)
    cache.set(key, content, ttl)
    return True


async def get_search_results_from_cache(query: str, limit: int) -> Optional[list]:
    """Example function to get search results from cache"""
    # Generate cache key
    key = cache_key("search", query, limit)
    return cache.get(key)


async def set_search_results_in_cache(query: str, limit: int, results: list, ttl: int = 300) -> bool:
    """Example function to set search results in cache"""
    # Generate cache key
    key = cache_key("search", query, limit)
    cache.set(key, results, ttl)
    return True