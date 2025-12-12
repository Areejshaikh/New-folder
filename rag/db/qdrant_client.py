import os
from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Optional
import uuid


class QdrantConfig:
    def __init__(self):
        self.url = os.getenv("QDRANT_URL")
        self.api_key = os.getenv("QDRANT_API_KEY")
        self.collection_name = "book_content_collection"
        
        # Initialize the Qdrant client
        self.client = QdrantClient(
            url=self.url,
            api_key=self.api_key,
        )
        
        # Create collection if it doesn't exist
        self._create_collection()
    
    def _create_collection(self):
        """Create the collection for storing book content embeddings if it doesn't exist"""
        try:
            # Check if collection exists
            self.client.get_collection(self.collection_name)
        except:
            # Collection doesn't exist, create it
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),  # Assuming OpenAI embeddings
            )
    
    def store_content(self, content_id: str, content: str, metadata: Dict) -> bool:
        """
        Store content with its embedding in Qdrant
        """
        try:
            from openai import OpenAI
            client = OpenAI()
            
            # Generate embedding for the content
            response = client.embeddings.create(
                input=content,
                model="text-embedding-ada-002"
            )
            embedding = response.data[0].embedding
            
            # Store in Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=[
                    models.PointStruct(
                        id=content_id,
                        vector=embedding,
                        payload={
                            "content": content,
                            "metadata": metadata
                        }
                    )
                ]
            )
            return True
        except Exception as e:
            print(f"Error storing content in Qdrant: {e}")
            return False
    
    def search_content(self, query: str, limit: int = 5) -> List[Dict]:
        """
        Search for content based on query
        """
        try:
            from openai import OpenAI
            client = OpenAI()
            
            # Generate embedding for the query
            response = client.embeddings.create(
                input=query,
                model="text-embedding-ada-002"
            )
            query_embedding = response.data[0].embedding
            
            # Search in Qdrant
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit
            )
            
            results = []
            for result in search_results:
                results.append({
                    "id": result.id,
                    "content": result.payload.get("content", ""),
                    "metadata": result.payload.get("metadata", {}),
                    "score": result.score
                })
            
            return results
        except Exception as e:
            print(f"Error searching content in Qdrant: {e}")
            return []
    
    def delete_content(self, content_id: str) -> bool:
        """
        Delete content from Qdrant by ID
        """
        try:
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(
                    points=[content_id]
                )
            )
            return True
        except Exception as e:
            print(f"Error deleting content from Qdrant: {e}")
            return False

# Singleton instance
qdrant_config = QdrantConfig()