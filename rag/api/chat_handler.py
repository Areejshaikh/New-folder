from typing import List, Dict, Optional
import uuid
from datetime import datetime
from rag.db.qdrant_client import qdrant_config
from rag.db.postgres_client import postgres_config
from rag.fastapi_app.models.user_query import UserQuery
from rag.fastapi_app.models.chatbot_response import ChatbotResponse, Citation
from rag.fastapi_app.models.book_content import BookContent
import os
from openai import OpenAI


async def process_query_with_rag(user_query: UserQuery) -> ChatbotResponse:
    """
    Process a user query using the RAG (Retrieval Augmented Generation) system
    """
    # Store the user query in the database
    query_data = {
        'id': user_query.id,
        'user_id': user_query.user_id,
        'query_text': user_query.query_text,
        'selected_text': user_query.selected_text,
        'context_page': user_query.context_page,
        'timestamp': user_query.timestamp,
        'session_id': user_query.session_id
    }
    
    await postgres_config.store_user_query(query_data)
    
    # Determine the search context based on whether text was selected
    search_query = user_query.query_text
    is_selected_text_context = bool(user_query.selected_text)

    if is_selected_text_context:
        # If text was selected, search primarily in that context
        search_query = f"Context: {user_query.selected_text}. Question: {user_query.query_text}"

    # Search for relevant content in the vector database
    search_results = qdrant_config.search_content(search_query, limit=5)

    # Prepare context for the LLM
    context_text = ""
    citations = []

    for result in search_results:
        context_text += f"\n\nSection: {result['metadata'].get('title', 'Unknown')}\n"
        context_text += f"Content: {result['content']}\n"

        citation = Citation(
            content_id=result['id'],
            title=result['metadata'].get('title', 'Unknown'),
            url=result['metadata'].get('url', '')
        )
        citations.append(citation)
    
    # If no content found, return a helpful message
    if not context_text.strip():
        response_text = (
            f"I couldn't find any relevant content in the book to answer your query: '{user_query.query_text}'. "
            f"Please make sure your question relates to the book content, or try rephrasing your question."
        )
        follow_up_questions = [
            "Is there a specific topic in the book you'd like to know more about?",
            "Could you try asking your question in different words?",
            "Would you like me to help you find a relevant section in the book?"
        ]
        confidence_score = 0.0
    else:
        # Generate response using OpenAI API
        client = OpenAI()
        
        # Prepare the prompt for the LLM
        prompt = f"""
        You are an AI assistant for an educational book. Your goal is to answer questions based ONLY on the provided book content.
        
        Context from the book:
        {context_text}
        
        User's question: {user_query.query_text}
        
        Instructions:
        1. Answer the user's question based only on the provided context
        2. If the context doesn't contain enough information, say so
        3. Provide specific citations to the book content when possible
        4. If the user's question is outside the scope of the provided content, politely explain that you can only answer questions based on the book content
        
        Response:
        """
        
        try:
            completion = client.chat.completions.create(
                model="gpt-4-turbo",  # Using GPT-4 Turbo for better understanding
                messages=[
                    {"role": "system", "content": "You are an AI assistant for an educational book. Answer questions based only on the provided book content, and provide citations when possible."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=1000,
                temperature=0.3  # Lower temperature for more consistent, fact-based responses
            )
            
            response_text = completion.choices[0].message.content.strip()
            
            # For now, we'll use a simple approach to determine confidence score
            # In a production system, this would be more sophisticated
            confidence_score = 0.85  # Assuming high confidence as we're using GPT-4
            
            # Generate follow-up questions
            follow_up_prompt = f"""
            Based on the following question and answer, generate 3 follow-up questions that would help the user learn more:
            
            Question: {user_query.query_text}
            Answer: {response_text}
            
            Return only the 3 follow-up questions as a JSON array:
            """
            
            follow_up_completion = client.chat.completions.create(
                model="gpt-4-turbo",
                messages=[
                    {"role": "user", "content": follow_up_prompt}
                ],
                max_tokens=200,
                temperature=0.5
            )
            
            import json
            try:
                # Try to parse the response as JSON
                follow_up_questions = json.loads(follow_up_completion.choices[0].message.content.strip())
            except:
                # If parsing fails, create a default set of follow-up questions
                follow_up_questions = [
                    "What other concepts in the book relate to this topic?",
                    "How might this concept be applied in practice?",
                    "Can you provide more details about a specific aspect of this topic?"
                ]
        
        except Exception as e:
            response_text = f"Sorry, I encountered an error generating a response: {str(e)}"
            follow_up_questions = []
            confidence_score = 0.0
    
    # Create the response object
    response_id = str(uuid.uuid4())
    
    chatbot_response = ChatbotResponse(
        id=response_id,
        query_id=user_query.id,
        response_text=response_text,
        citations=citations if citations else None,
        follow_up_questions=follow_up_questions if follow_up_questions else None,
        confidence_score=confidence_score,
        model_used="gpt-4-turbo"
    )
    
    # Store the response in the database
    response_data = {
        'id': chatbot_response.id,
        'query_id': chatbot_response.query_id,
        'response_text': chatbot_response.response_text,
        'citations': [citation.dict() for citation in citations] if citations else [],
        'follow_up_questions': follow_up_questions if follow_up_questions else [],
        'confidence_score': confidence_score,
        'timestamp': datetime.utcnow(),
        'model_used': chatbot_response.model_used
    }
    
    await postgres_config.store_chatbot_response(response_data)
    
    return chatbot_response