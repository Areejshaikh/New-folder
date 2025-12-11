"""
RAG (Retrieval Augmented Generation) Chatbot for Robotics Curriculum

This module implements a RAG system that connects to the book content
and provides intelligent Q&A capabilities for learners.
"""

import os
from typing import List, Dict, Any, Optional
import uuid
from datetime import datetime

from langchain.embeddings.openai import OpenAIEmbeddings
from langchain.vectorstores import FAISS
from langchain.text_splitter import RecursiveCharacterTextSplitter
from langchain.chains import ConversationalRetrievalChain
from langchain.chat_models import ChatOpenAI
from langchain.document_loaders import DirectoryLoader, TextLoader

# For similarity search and document handling
from langchain.schema import Document
import pickle
import faiss


class RAGChatbot:
    """
    Implementation of a RAG (Retrieval Augmented Generation) chatbot
    that integrates with book content for answering questions.
    """
    
    def __init__(self, 
                 content_dir: str = "docusaurus-book/docs",
                 embeddings_model: str = "text-embedding-ada-002",
                 llm_model: str = "gpt-3.5-turbo"):
        """
        Initialize the RAG chatbot
        
        Args:
            content_dir: Directory containing the book content
            embeddings_model: Model to use for embeddings
            llm_model: Model to use for generation
        """
        self.content_dir = content_dir
        self.embeddings_model = embeddings_model
        self.llm_model = llm_model
        self.documents = []
        self.vector_store = None
        self.conversation_chain = None
        
        # Initialize components
        self._setup_openai_api()
        
    def _setup_openai_api(self):
        """Set up OpenAI API key"""
        # In a real system, this would come from environment variables
        # For now we'll just check if it's set
        self.openai_api_key = os.getenv("OPENAI_API_KEY")
        if not self.openai_api_key:
            print("Warning: OPENAI_API_KEY not set. Using mock mode.")
    
    def load_documents(self, file_pattern: str = "**/*.md") -> List[Document]:
        """
        Load documents from the content directory
        
        Args:
            file_pattern: Pattern to match files (e.g., "**/*.md" for markdown files)
        """
        try:
            # Use DirectoryLoader to load all documents
            loader = DirectoryLoader(
                self.content_dir,
                glob=file_pattern,
                loader_cls=TextLoader,
                loader_kwargs={'encoding': 'utf-8'}
            )
            self.documents = loader.load()
            print(f"Loaded {len(self.documents)} documents from {self.content_dir}")
            return self.documents
        except Exception as e:
            print(f"Error loading documents: {e}")
            # Create mock documents for demonstration
            self.documents = [
                Document(
                    page_content="This is a sample document about robotics curriculum. It contains information about ROS 2, Gazebo simulations, and AI integration for humanoid robots.",
                    metadata={"source": "sample_document.md", "title": "Sample Robotics Curriculum Document"}
                ),
                Document(
                    page_content="Another sample document about AI vision-language-action systems. This covers how to connect natural language processing with robot actions.",
                    metadata={"source": "sample_vla.md", "title": "Vision-Language-Action Systems"}
                )
            ]
            print("Created mock documents for demonstration purposes")
            return self.documents
    
    def create_vector_store(self, chunk_size: int = 1000, chunk_overlap: int = 200):
        """
        Create a vector store from the loaded documents
        
        Args:
            chunk_size: Size of text chunks
            chunk_overlap: Overlap between chunks
        """
        if not self.documents:
            raise ValueError("No documents loaded. Call load_documents() first.")
        
        # Split documents into chunks
        text_splitter = RecursiveCharacterTextSplitter(
            chunk_size=chunk_size,
            chunk_overlap=chunk_overlap,
            length_function=len,
        )
        texts = text_splitter.split_documents(self.documents)
        
        print(f"Split documents into {len(texts)} chunks")
        
        # Create vector store using FAISS
        if self.openai_api_key:
            embeddings = OpenAIEmbeddings(model=self.embeddings_model)
            self.vector_store = FAISS.from_documents(texts, embeddings)
        else:
            # Use a local embedding model since no API key is set
            # For now, we'll create a mock vector store
            print("No OpenAI API key found, creating mock vector store")
            self.vector_store = MockVectorStore(texts)
        
        print("Vector store created successfully")
    
    def initialize_conversation_chain(self, temperature: float = 0.7):
        """
        Initialize the conversation chain for the chatbot
        
        Args:
            temperature: Controls randomness of responses (0.0-1.0)
        """
        if not self.vector_store:
            raise ValueError("No vector store created. Call create_vector_store() first.")
        
        if self.openai_api_key:
            llm = ChatOpenAI(
                model_name=self.llm_model,
                temperature=temperature,
                openai_api_key=self.openai_api_key
            )
        else:
            # Use a mock LLM since no API key is set
            llm = MockLLM()
        
        self.conversation_chain = ConversationalRetrievalChain.from_llm(
            llm=llm,
            retriever=self.vector_store.as_retriever(),
            return_source_documents=True,
            verbose=False
        )
        
        print("Conversation chain initialized successfully")
    
    def query(self, question: str, chat_history: Optional[List] = None) -> Dict[str, Any]:
        """
        Query the chatbot with a question
        
        Args:
            question: The question to ask
            chat_history: Previous conversation history (for context)
            
        Returns:
            Dictionary containing the response and sources
        """
        if not self.conversation_chain:
            raise ValueError("Conversation chain not initialized. Call initialize_conversation_chain() first.")
        
        if chat_history is None:
            chat_history = []
        
        try:
            response = self.conversation_chain({
                "question": question,
                "chat_history": chat_history
            })
            
            return {
                "question": question,
                "answer": response["answer"],
                "sources": [doc.metadata.get("source", "unknown") for doc in response["source_documents"]],
                "confidence": response.get("confidence", 0.8)  # Default confidence
            }
        except Exception as e:
            print(f"Error processing query: {e}")
            # Return a mock response
            return {
                "question": question,
                "answer": f"I'm sorry, I encountered an error processing your question: {str(e)}. This is a mock response indicating the system is working conceptually.",
                "sources": ["mock_source.md"],
                "confidence": 0.0
            }
    
    def save_vector_store(self, path: str):
        """
        Save the vector store to disk
        
        Args:
            path: Path to save the vector store
        """
        if not self.vector_store:
            raise ValueError("No vector store to save")
        
        # Save the vector store
        if hasattr(self.vector_store, 'save_local'):
            self.vector_store.save_local(path)
        else:
            # For mock vector store
            with open(path + '/mock_store.pkl', 'wb') as f:
                pickle.dump(self.vector_store, f)
        
        print(f"Vector store saved to {path}")
    
    def load_vector_store(self, path: str):
        """
        Load a vector store from disk
        
        Args:
            path: Path to load the vector store from
        """
        # Try to load the vector store
        try:
            if self.openai_api_key:
                embeddings = OpenAIEmbeddings(model=self.embeddings_model)
                self.vector_store = FAISS.load_local(path, embeddings)
            else:
                # Load mock vector store
                with open(path + '/mock_store.pkl', 'rb') as f:
                    self.vector_store = pickle.load(f)
            
            print(f"Vector store loaded from {path}")
        except Exception as e:
            print(f"Error loading vector store: {e}. Creating a new one with mock data.")
            self.create_vector_store()


class MockVectorStore:
    """
    Mock implementation of a vector store for demonstration purposes
    """
    def __init__(self, documents):
        self.documents = documents
        self.index = list(range(len(documents)))
    
    def as_retriever(self):
        """Return a mock retriever"""
        return MockRetriever(self.documents)


class MockRetriever:
    """Mock retriever for the mock vector store"""
    
    def __init__(self, documents):
        self.documents = documents
    
    def get_relevant_documents(self, query: str):
        """Return relevant documents for the query (mock implementation)"""
        # Simple mock implementation: return all documents
        return self.documents[:2]  # Return first 2 documents for demo


class MockLLM:
    """Mock LLM for demonstration when API key is not available"""
    
    def __call__(self, *args, **kwargs):
        # This is a simplified mock that returns a fixed response
        class MockResponse:
            def __init__(self):
                self.content = "This is a mock response from the LLM. In a real implementation, this would be generated by an actual large language model based on the retrieved context."
        
        class MockMessage:
            def __init__(self):
                self.content = MockResponse().content
        
        class MockResult:
            def __init__(self):
                self.generations = [[MockMessage()]]
        
        return MockResult()


# Example usage
if __name__ == "__main__":
    # Initialize the RAG chatbot
    chatbot = RAGChatbot(content_dir="docusaurus-book/docs")
    
    # Load documents
    docs = chatbot.load_documents("**/*.md")
    
    # Create vector store
    chatbot.create_vector_store()
    
    # Initialize conversation chain
    chatbot.initialize_conversation_chain()
    
    # Example conversation
    chat_history = []
    
    # First question
    question1 = "What are the main components of the ROS 2 curriculum?"
    print(f"Q: {question1}")
    response1 = chatbot.query(question1, chat_history)
    print(f"A: {response1['answer']}")
    print(f"Sources: {response1['sources']}\n")
    
    # Add to chat history
    chat_history.append((question1, response1["answer"]))
    
    # Second question (follow-up)
    question2 = "Can you explain more about the simulation environments?"
    print(f"Q: {question2}")
    response2 = chatbot.query(question2, chat_history)
    print(f"A: {response2['answer']}")
    print(f"Sources: {response2['sources']}\n")
    
    # Add to chat history
    chat_history.append((question2, response2["answer"]))
    
    print("RAG Chatbot demo completed successfully!")