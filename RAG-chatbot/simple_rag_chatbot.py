"""
Simple RAG Chatbot for Robotics Curriculum

This is a lightweight implementation that demonstrates the core concepts
of retrieval-augmented generation for the robotics curriculum.
"""

import os
import json
import pickle
from typing import List, Dict, Any, Optional
from pathlib import Path
import re

# Use sentence transformers for embeddings (offline-capable)
from sentence_transformers import SentenceTransformer
import numpy as np
from numpy.linalg import norm


class SimpleRAGChatbot:
    """
    A simple RAG (Retrieval Augmented Generation) chatbot that integrates 
    with the book content to provide intelligent Q&A for learners.
    """
    
    def __init__(self, 
                 content_dir: str = "E:/PAHR-book/docusaurus-book/docs",
                 embedding_model: str = "all-MiniLM-L6-v2"):
        """
        Initialize the RAG chatbot
        
        Args:
            content_dir: Directory containing the book content
            embedding_model: Sentence transformer model for embeddings
        """
        self.content_dir = Path(content_dir)
        self.embedding_model_name = embedding_model
        self.model = SentenceTransformer(embedding_model)
        self.documents = []
        self.embeddings = []
        self.sources = []
        
    def load_documents(self, file_pattern: str = "*.md"):
        """
        Load documents from the content directory
        
        Args:
            file_pattern: Pattern to match files (e.g., "*.md" for markdown files)
        """
        if not self.content_dir.exists():
            print(f"Warning: Content directory {self.content_dir} does not exist. Using mock documents.")
            self._create_mock_documents()
            return
        
        md_files = list(self.content_dir.glob(file_pattern))
        if not md_files:
            # If no markdown files found in root, search recursively
            md_files = list(self.content_dir.rglob(file_pattern))
        
        for file_path in md_files:
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()
                    
                # Split content into chunks to handle large documents
                chunks = self._split_document(content, max_length=512)
                
                for i, chunk in enumerate(chunks):
                    self.documents.append(chunk)
                    self.sources.append(f"{file_path.name}#{i+1}")  # Include chunk number
                
                print(f"Loaded {len(chunks)} chunks from {file_path.name}")
            except Exception as e:
                print(f"Error loading {file_path}: {e}")
        
        if not self.documents:
            print("No documents loaded. Creating mock documents.")
            self._create_mock_documents()
    
    def _create_mock_documents(self):
        """Create mock documents for demonstration purposes"""
        self.documents = [
            "# Introduction to ROS 2\n\nROS 2 (Robot Operating System 2) is the next generation of ROS, focusing on production environments. It addresses limitations in ROS 1 such as lack of support for real-time systems, security, and multi-robot systems.",
            "# Setting up ROS 2 Environment\n\nTo set up a ROS 2 environment, you'll need to install the ROS 2 distribution (e.g., Humble Hawksbill) and create a workspace. This involves sourcing the setup script and creating packages with specific structures.",
            "# Gazebo Simulation\n\nGazebo is a physics-based simulation environment used in robotics development. It provides realistic dynamics, sensors, and environments to test robots safely before deploying on real hardware.",
            "# Unity Robotics Simulation\n\nUnity provides high-fidelity rendering and human-robot interaction capabilities. It excels in visualization and can complement physics simulation environments like Gazebo for comprehensive testing.",
            "# NVIDIA Isaac Integration\n\nNVIDIA Isaac provides perception tools and simulation capabilities. It includes Isaac Sim for photorealistic simulation and Isaac ROS for hardware-accelerated perception and navigation.",
            "# AI and Perception Systems\n\nModern robotics relies heavily on AI, including computer vision, sensor fusion, and machine learning algorithms. These systems enable robots to perceive and interact with the world intelligently."
        ]
        self.sources = [f"mock_doc_{i}.md" for i in range(len(self.documents))]
        print("Created mock documents for demonstration")
    
    def _split_document(self, text: str, max_length: int = 512) -> List[str]:
        """
        Split a document into chunks of approximately max_length tokens
        
        Args:
            text: Document text to split
            max_length: Maximum length of each chunk
            
        Returns:
            List of text chunks
        """
        # Simple sentence-based splitting
        sentences = re.split(r'(?<=[.!?]) +', text)
        chunks = []
        current_chunk = ""
        
        for sentence in sentences:
            if len(current_chunk) + len(sentence) <= max_length:
                current_chunk += " " + sentence if current_chunk else sentence
            else:
                if current_chunk:
                    chunks.append(current_chunk)
                current_chunk = sentence
        
        if current_chunk:
            chunks.append(current_chunk)
        
        # Handle any unusually long sentences
        final_chunks = []
        for chunk in chunks:
            if len(chunk) > max_length:
                # Split into smaller pieces by words
                words = chunk.split()
                temp_chunk = ""
                for word in words:
                    if len(temp_chunk + word) <= max_length:
                        temp_chunk += " " + word if temp_chunk else word
                    else:
                        if temp_chunk:
                            final_chunks.append(temp_chunk)
                        temp_chunk = word
                if temp_chunk:
                    final_chunks.append(temp_chunk)
            else:
                final_chunks.append(chunk)
        
        return final_chunks
    
    def create_embeddings(self):
        """
        Create embeddings for all loaded documents
        """
        if not self.documents:
            raise ValueError("No documents loaded. Call load_documents() first.")
        
        print(f"Creating embeddings for {len(self.documents)} document chunks...")
        self.embeddings = self.model.encode(self.documents)
        print("Embeddings created successfully")
    
    def find_relevant_docs(self, query: str, top_k: int = 3) -> List[Dict[str, Any]]:
        """
        Find the most relevant documents for a query using cosine similarity
        
        Args:
            query: Query string
            top_k: Number of top documents to return
            
        Returns:
            List of dictionaries containing 'content', 'source', and 'score'
        """
        query_embedding = self.model.encode([query])[0]
        
        # Calculate cosine similarities
        similarities = []
        for doc_embedding in self.embeddings:
            # Cosine similarity calculation
            cos_sim = np.dot(query_embedding, doc_embedding) / (norm(query_embedding) * norm(doc_embedding))
            similarities.append(cos_sim)
        
        # Get top-k most similar documents
        top_indices = np.argsort(similarities)[-top_k:][::-1]
        
        results = []
        for idx in top_indices:
            results.append({
                'content': self.documents[idx],
                'source': self.sources[idx],
                'score': float(similarities[idx])
            })
        
        return results
    
    def generate_response(self, query: str, context_docs: List[Dict[str, str]]) -> str:
        """
        Generate a response based on the query and context documents
        
        Args:
            query: User's question
            context_docs: Relevant documents to use as context
            
        Returns:
            Generated response string
        """
        if not context_docs:
            return "I couldn't find any relevant information to answer your question. Please try rephrasing your question or consult the full documentation."
        
        # Combine context from relevant documents
        context = "\n\n".join([doc['content'] for doc in context_docs])
        
        # Truncate context if it's too long
        max_context_length = 2000
        if len(context) > max_context_length:
            context = context[:max_context_length] + "... [truncated]"
        
        # Create a simple prompt for response generation
        prompt = f"""
        Based on the following context, answer the question. If the context doesn't contain enough information to answer the question, say so clearly.
        
        Context:
        {context}
        
        Question: {query}
        
        Answer:"""
        
        # For this simple implementation, we'll just return the relevant information
        # combined with a note that in a full implementation, this would be processed
        # by an LLM to generate a coherent response
        response = f"Based on the documentation:\n\n"
        response += "\n\n".join([f"- From {doc['source']} (relevance: {doc['score']:.2}): {doc['content'][:200]}..." for doc in context_docs])
        response += f"\n\nThis is a simplified RAG response. In a full implementation, an LLM would synthesize this information into a coherent answer."
        
        return response
    
    def query(self, question: str) -> Dict[str, Any]:
        """
        Query the chatbot with a question
        
        Args:
            question: The question to ask
            
        Returns:
            Dictionary containing the response and sources
        """
        # Find relevant documents
        relevant_docs = self.find_relevant_docs(question, top_k=3)
        
        # Generate response
        answer = self.generate_response(question, relevant_docs)
        
        # Extract sources
        sources = [doc['source'] for doc in relevant_docs]
        
        return {
            "question": question,
            "answer": answer,
            "sources": sources,
            "relevance_scores": [float(doc['score']) for doc in relevant_docs]
        }
    
    def save_index(self, path: str):
        """
        Save the document index to disk
        
        Args:
            path: Path to save the index
        """
        index_data = {
            'documents': self.documents,
            'sources': self.sources,
            'embeddings': self.embeddings
        }
        
        with open(path, 'wb') as f:
            pickle.dump(index_data, f)
        
        print(f"Index saved to {path}")
    
    def load_index(self, path: str):
        """
        Load a document index from disk
        
        Args:
            path: Path to load the index from
        """
        with open(path, 'rb') as f:
            index_data = pickle.load(f)
        
        self.documents = index_data['documents']
        self.sources = index_data['sources']
        self.embeddings = index_data['embeddings']
        
        print(f"Index loaded from {path}")


# Example usage
if __name__ == "__main__":
    # Initialize the RAG chatbot
    print("Initializing RAG Chatbot...")
    chatbot = SimpleRAGChatbot(content_dir="E:/PAHR-book/docusaurus-book/docs")
    
    # Load documents
    print("Loading documents...")
    chatbot.load_documents("**/*.md")  # Look for MD files recursively
    
    # Create embeddings
    print("Creating embeddings...")
    chatbot.create_embeddings()
    
    # Example queries
    queries = [
        "What is ROS 2?",
        "How do I set up a simulation environment?",
        "What is NVIDIA Isaac?",
        "How does the curriculum structure work?"
    ]
    
    print("\n" + "="*50)
    print("RAG CHATBOT DEMO")
    print("="*50)
    
    for i, query in enumerate(queries, 1):
        print(f"\nQuestion {i}: {query}")
        response = chatbot.query(query)
        print(f"Answer: {response['answer']}")
        print(f"Sources: {response['sources']}")
        print("-" * 30)
    
    print("\nRAG Chatbot demo completed successfully!")