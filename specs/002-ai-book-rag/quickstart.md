# Quickstart Guide: AI-Driven Book with Integrated RAG Chatbot

## Overview
This guide provides a quick overview of how to set up and run the AI-driven book with integrated RAG chatbot system. It covers the core components and how they work together.

## Prerequisites
- Python 3.11+
- Node.js 18+ and npm
- Access to OpenAI API key
- Access to Qdrant Cloud account
- Access to Neon Postgres account

## Setup Instructions

### 1. Environment Configuration
```bash
# Clone the repository
git clone [repository-url]
cd [repository-name]

# Create Python virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install Python dependencies
pip install -r requirements.txt

# Install Node.js dependencies
npm install
```

### 2. Environment Variables
Create a `.env` file in the project root with the following:
```env
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_DATABASE_URL=your_neon_postgres_connection_string
```

### 3. Running the Components

#### Docusaurus Book Server
```bash
cd book
npm start
```
The book will be available at `http://localhost:3000`

#### RAG Chatbot API Server
```bash
cd rag/fastapi_app
python main.py
```
The API will be available at `http://localhost:8000`

## Core Components

### 1. Book Content Structure
The book content follows Docusaurus conventions:
```
book/
├── docs/
│   ├── chapter-1/
│   │   ├── intro.md
│   │   └── concepts.md
│   └── chapter-2/
│       ├── theory.md
│       └── practice.md
├── src/
│   └── components/
│       └── chatbot-widget/
├── static/
├── sidebars.js
└── docusaurus.config.js
```

Each markdown file should include front-matter metadata:
```markdown
---
id: my-content-id
title: My Content Title
description: Brief description of the content
keywords: [keyword1, keyword2, keyword3]
tags: [tag1, tag2]
---
```

### 2. RAG System Architecture
```
┌─────────────┐    ┌──────────────┐    ┌──────────────┐
│   User      │───▶│  FastAPI     │───▶│  Qdrant      │
│   Query     │    │  Backend     │    │  Vector DB   │
└─────────────┘    └──────────────┘    └──────────────┘
                          │
                          ▼
                    ┌──────────────┐
                    │  Neon        │
                    │  Postgres    │
                    └──────────────┘
```

The system works as follows:
1. User submits a query via the chatbot widget
2. The frontend captures the query and any selected text
3. The request is sent to the FastAPI backend
4. The backend performs vector similarity search on Qdrant
5. Relevant content is retrieved and passed to OpenAI API
6. A contextual response is generated with citations
7. The response is sent back to the frontend

### 3. Chatbot Widget Integration
The chatbot widget is a React component integrated into the Docusaurus layout:
- Appears as a floating button on all book pages
- Expands to show the chat interface when clicked
- Can detect and include selected text in queries
- Displays responses with citations and follow-up suggestions

## Running Tests
```bash
# Python backend tests
cd rag/fastapi_app
python -m pytest tests/

# Frontend component tests
cd book
npm test
```

## Key Endpoints

### Chat API
`POST /chat` - Process a user query against book content
```json
{
  "query": "Explain forward kinematics",
  "session_id": "session_123",
  "selected_text": "Forward kinematics is the process...",
  "context_page": "/chapters/kinematics"
}
```

### Content Search API
`POST /content/search` - Search book content for relevant passages
```json
{
  "query": "kinematics robotics",
  "limit": 5
}
```

## Development Workflow

### Adding New Content
1. Create a new markdown file in the appropriate chapter directory in `book/docs/`
2. Include required front-matter metadata (id, title, description, keywords, tags)
3. Add the content to the appropriate sidebar in `book/sidebars.js`
4. Run `python rag/loader/load_markdown.py` to index the new content in Qdrant

### Extending the Chatbot
1. Modify the chat handler in `rag/api/chat_handler.py`
2. Update the API contract in `specs/contracts/rag-chatbot-api.yaml` if needed
3. Update the frontend widget in `book/src/components/chatbot-widget/`
4. Test the changes in the development environment

## Deployment
For production deployment to GitHub Pages:
```bash
# Build the Docusaurus site
cd book
npm run build

# The build output is in the build/ directory
# Configure GitHub Pages to serve from the build/ directory
```

The backend API must be deployed separately to a server that supports Python/FastAPI applications.