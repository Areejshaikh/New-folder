"""
Integration Module: Connecting Curriculum Content with RAG System

This module provides functionality to integrate the curriculum content
with the RAG (Retrieval Augmented Generation) system to provide 
Q&A support for learners.
"""

from pathlib import Path
import json
from typing import List, Dict, Any, Optional
import logging
from datetime import datetime

from RAG-chatbot.rag_chatbot import RAGChatbot
from content_management_system import ContentManager, ContentType


class CurriculumRAGIntegration:
    """
    Integration class that connects curriculum content with the RAG system
    """
    
    def __init__(self, content_manager: ContentManager, rag_chatbot: RAGChatbot):
        """
        Initialize the integration between curriculum content and RAG system
        
        Args:
            content_manager: Instance of ContentManager to access curriculum content
            rag_chatbot: Instance of RAGChatbot to provide Q&A functionality
        """
        self.content_manager = content_manager
        self.rag_chatbot = rag_chatbot
        self.logger = logging.getLogger(__name__)
        
        # Set up logging
        logging.basicConfig(level=logging.INFO)
        
    def index_curriculum_content(self) -> bool:
        """
        Index all curriculum content for the RAG system
        
        Returns:
            True if indexing was successful, False otherwise
        """
        try:
            self.logger.info("Starting curriculum content indexing for RAG system")
            
            # Get all published curriculum items
            all_items = self.content_manager.list_items(status="published")
            
            if not all_items:
                self.logger.warning("No published curriculum items found to index")
                # Try to index all items regardless of status
                all_items = self.content_manager.list_items()
            
            if not all_items:
                self.logger.error("No curriculum items found to index")
                return False
                
            # Update the RAG chatbot's content directory to include curriculum content
            # The RAG system looks in docusaurus-book/docs by default, which is where our content is
            
            # Load documents using the RAG system
            self.rag_chatbot.load_documents("**/*.md")
            
            # Create vector store from curriculum content
            self.rag_chatbot.create_vector_store()
            
            # Initialize the conversation chain
            self.rag_chatbot.initialize_conversation_chain()
            
            self.logger.info(f"Successfully indexed {len(all_items)} curriculum items for RAG system")
            return True
            
        except Exception as e:
            self.logger.error(f"Error indexing curriculum content: {str(e)}")
            return False
    
    def update_index_for_item(self, item_id: str) -> bool:
        """
        Update the RAG index when a specific curriculum item is modified
        
        Args:
            item_id: ID of the curriculum item to update in the index
            
        Returns:
            True if update was successful, False otherwise
        """
        try:
            self.logger.info(f"Updating RAG index for item: {item_id}")
            
            # Get the specific item
            item = self.content_manager.get_item(item_id)
            if not item:
                self.logger.error(f"Item {item_id} not found in content manager")
                return False
            
            # For now, re-index all content since the RAG system doesn't have incremental updates
            # In a production system, you'd want to implement incremental indexing
            return self.index_curriculum_content()
            
        except Exception as e:
            self.logger.error(f"Error updating index for item {item_id}: {str(e)}")
            return False
    
    def answer_curriculum_question(self, question: str, context: Optional[List] = None) -> Dict[str, Any]:
        """
        Answer a question about curriculum content using the RAG system
        
        Args:
            question: The question to answer
            context: Optional conversation history
            
        Returns:
            Dictionary containing the answer and source information
        """
        try:
            self.logger.info(f"Answering curriculum question: {question}")
            
            # Use the RAG chatbot to generate a response
            result = self.rag_chatbot.query(question, context or [])
            
            # Enhance the response with additional metadata
            result['timestamp'] = datetime.now().isoformat()
            result['source_type'] = 'curriculum_content'
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error answering question: {str(e)}")
            return {
                "question": question,
                "answer": "I'm sorry, I encountered an error processing your question. Please try rephrasing it.",
                "sources": [],
                "confidence": 0.0,
                "timestamp": datetime.now().isoformat(),
                "source_type": 'error'
            }
    
    def get_relevant_curriculum_items(self, query: str, max_items: int = 5) -> List[Dict[str, Any]]:
        """
        Get curriculum items most relevant to a query
        
        Args:
            query: Query to match against curriculum content
            max_items: Maximum number of items to return
            
        Returns:
            List of dictionaries containing item information and relevance scores
        """
        try:
            # Use the content manager's search functionality
            search_results = self.content_manager.search_items(query)
            
            # Convert to the required format
            items = []
            for item in search_results[:max_items]:
                items.append({
                    'id': item.id,
                    'title': item.title,
                    'content_type': item.content_type.value,
                    'description': item.description,
                    'difficulty': item.difficulty,
                    'estimated_duration': item.estimated_duration,
                    'tags': item.tags,
                    'relevance_score': 0.8  # Placeholder - in a real system, this would come from vector similarity
                })
            
            return items
            
        except Exception as e:
            self.logger.error(f"Error getting relevant curriculum items: {str(e)}")
            return []
    
    def validate_integration(self) -> bool:
        """
        Validate that the curriculum content is properly integrated with the RAG system
        
        Returns:
            True if validation passes, False otherwise
        """
        try:
            # Perform basic validation checks
            if not self.rag_chatbot.vector_store:
                self.logger.error("No vector store found in RAG chatbot")
                return False
            
            # Try a simple test question
            test_result = self.answer_curriculum_question("What is ROS 2?")
            
            if not test_result.get('answer') or 'error' in test_result.get('source_type', ''):
                self.logger.error("Test query failed")
                return False
                
            self.logger.info("Curriculum-RAG integration validation passed")
            return True
            
        except Exception as e:
            self.logger.error(f"Error validating integration: {str(e)}")
            return False


def setup_curriculum_rag_integration() -> CurriculumRAGIntegration:
    """
    Set up the integration between curriculum content and the RAG system
    
    Returns:
        Instance of CurriculumRAGIntegration
    """
    # Initialize content manager
    content_manager = ContentManager()
    
    # Initialize RAG chatbot
    rag_chatbot = RAGChatbot(content_dir="docusaurus-book/docs")
    
    # Create integration instance
    integration = CurriculumRAGIntegration(content_manager, rag_chatbot)
    
    # Index curriculum content
    if integration.index_curriculum_content():
        print("Successfully integrated curriculum content with RAG system")
    else:
        print("Warning: Failed to integrate curriculum content with RAG system")
        
    return integration


# Example usage and testing
if __name__ == "__main__":
    print("Setting up curriculum-RAG integration...")
    
    # Set up the integration
    integration = setup_curriculum_rag_integration()
    
    # Validate the integration
    if integration.validate_integration():
        print("Integration validation passed!")
        
        # Test asking questions
        print("\nTesting Q&A functionality:")
        
        test_questions = [
            "What are the main concepts in ROS 2?",
            "How do I create a ROS 2 node?",
            "Can you explain the publish-subscribe pattern in ROS 2?",
            "What is turtlesim used for?",
            "How do I control a turtle in the turtlesim environment?"
        ]
        
        for question in test_questions:
            print(f"\nQ: {question}")
            result = integration.answer_curriculum_question(question)
            print(f"A: {result['answer']}")
            print(f"Sources: {result['sources']}")
            print(f"Confidence: {result['confidence']:.2f}")
    else:
        print("Integration validation failed!")
    
    print("\nCurriculum-RAG integration setup complete!")