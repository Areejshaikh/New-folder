"""
Test script for the RAG Chatbot implementation
"""

from simple_rag_chatbot import SimpleRAGChatbot

def test_rag_chatbot():
    """Test the RAG Chatbot functionality"""
    print("Testing RAG Chatbot Implementation...")

    # Initialize the RAG chatbot
    chatbot = SimpleRAGChatbot(
        content_dir="E:/PAHR-book/docusaurus-book/docs",
        embedding_model="all-MiniLM-L6-v2"
    )

    # Load documents
    print("\n1. Loading documents...")
    chatbot.load_documents("**/*.md")  # Look for MD files recursively

    # Create embeddings
    print("\n2. Creating embeddings...")
    chatbot.create_embeddings()

    # Define test queries related to robotics curriculum
    test_queries = [
        "What is ROS 2?",
        "How do I set up a simulation environment?",
        "What is NVIDIA Isaac?",
        "Explain the curriculum structure"
    ]

    print("\n3. Running test queries...")
    for i, query in enumerate(test_queries, 1):
        print(f"\nQuery {i}: {query}")
        response = chatbot.query(query)
        print(f"Answer: {response['answer'][:500]}...")  # Show first 500 chars
        print(f"Sources: {response['sources']}")
        print(f"Relevance scores: {[round(score, 2) for score in response['relevance_scores']]}")
        print("-" * 50)

    # Test saving and loading
    print("\n4. Testing save/load functionality...")
    chatbot.save_index("test_rag_index.pkl")
    print("Index saved successfully")

    # Create a new chatbot instance and load the index
    new_chatbot = SimpleRAGChatbot()
    new_chatbot.load_index("test_rag_index.pkl")
    print("Index loaded successfully")

    # Test a query on the loaded index
    print("\n5. Testing query on loaded index...")
    test_response = new_chatbot.query("What is robotics?")
    print(f"Answer: {test_response['answer'][:300]}...")

    print("\n[SUCCESS] RAG Chatbot test completed successfully!")


if __name__ == "__main__":
    # First, let's install the required dependency
    import subprocess
    import sys

    try:
        import sentence_transformers
        print("Sentence transformers is already installed")
    except ImportError:
        print("Installing sentence transformers...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "sentence-transformers"])
        import sentence_transformers

    test_rag_chatbot()