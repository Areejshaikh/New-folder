# Tasks: AI-Driven Book with Integrated RAG Chatbot

**Feature**: AI-Driven Book with Integrated RAG Chatbot  
**Branch**: `002-ai-book-rag`  
**Generated**: 2025-12-12  
**Input**: spec.md, plan.md, data-model.md, contracts/, research.md, quickstart.md  

## Feature Overview

Generate a complete Docusaurus-based book project with integrated RAG chatbot system. The system will allow content creators to author educational materials in structured markdown format with front-matter metadata, compile it into a navigable book website, and provide an AI-powered chatbot that answers user queries based on book content and selected text on pages. The system will be deployed to GitHub Pages and include FastAPI backend, Qdrant Cloud vector DB, and Neon Postgres metadata store.

## Phase 1: Setup/s

**Goal**: Set up project structure and basic configurations

- [X] T001 Create project root files: package.json, requirements.txt, README.md
- [X] T002 Set up repository structure: book/, rag/, api_server.py, docusaurus.config.js
- [X] T003 [P] Configure Python virtual environment and install dependencies: FastAPI, OpenAI, Qdrant, Neon Postgres drivers
- [X] T004 [P] Configure Node.js environment and install Docusaurus dependencies
- [X] T005 Set up environment variables configuration file (.env)
- [X] T006 Configure project-specific gitignore file

## Phase 2: Foundational

**Goal**: Implement core infrastructure and foundational components needed for all user stories

- [X] T007 [P] Set up Qdrant client configuration in rag/db/qdrant_client.py
- [X] T008 [P] Set up Neon Postgres client configuration in rag/db/postgres_client.py
- [X] T009 [P] Create BookContent model based on data-model.md in rag/fastapi_app/models/book_content.py
- [X] T010 [P] Create UserQuery model based on data-model.md in rag/fastapi_app/models/user_query.py
- [X] T011 [P] Create ChatbotResponse model based on data-model.md in rag/fastapi_app/models/chatbot_response.py
- [X] T012 [P] Create ContentMetadata model based on data-model.md in rag/fastapi_app/models/content_metadata.py
- [X] T013 [P] Create TextSelection model based on data-model.md in rag/fastapi_app/models/text_selection.py
- [X] T014 Implement content loader to process markdown files in rag/loader/load_markdown.py
- [X] T015 [P] Create initial Docusaurus configuration in book/docusaurus.config.js
- [X] T016 [P] Create sidebar configuration in book/sidebars.js
- [X] T017 [P] Set up main FastAPI application in rag/fastapi_app/main.py
- [X] T018 Create initial chat widget component in book/src/components/chatbot-widget/ChatWidget.jsx

## Phase 3: User Story 1 - Create and Access Interactive Learning Book (Priority: P1)

**Goal**: Allow content creators to author educational content in structured chapters and modules that will be compiled into a Docusaurus-based book, allowing readers to access and interact with the content including text, images, diagrams, code snippets, and practice questions.

**Independent Test**: Content can be authored in markdown format with proper front-matter metadata, and the Docusaurus site can be built and deployed successfully, allowing readers to navigate chapters and access all content types.

- [X] T019 [P] [US1] Create initial chapter content in book/docs/chapter-1/intro.md with proper front-matter metadata
- [X] T020 [P] [US1] Create additional content files including text, diagrams (Mermaid), code snippets, and practice questions in book/docs/
- [X] T021 [US1] Configure Docusaurus sidebar with hierarchical navigation in book/sidebars.js
- [X] T022 [US1] Implement content validation to ensure all required front-matter fields exist
- [X] T023 [US1] Add support for various content types (images, diagrams, code) in Docusaurus
- [X] T024 [US1] Implement content loading and display functionality in Docusaurus
- [ ] T025 [US1] Test Docusaurus build process to ensure all content types are preserved
- [ ] T026 [US1] Deploy Docusaurus site to GitHub Pages

## Phase 4: User Story 2 - Use AI-Powered Chatbot for Content Assistance (Priority: P1)

**Goal**: Allow readers to interact with an integrated AI chatbot that provides answers based on the book content, offering explanations, clarifications, and additional insights related to the material.

**Independent Test**: The chatbot can understand user queries about book content and provide accurate, contextual responses with appropriate citations to the source material.

- [X] T027 [P] [US2] Implement /chat endpoint POST method in rag/fastapi_app/routers/chat.py
- [X] T028 [P] [US2] Create chat handler logic in rag/api/chat_handler.py to process queries
- [X] T029 [US2] Implement RAG logic to retrieve relevant content from Qdrant based on query
- [X] T030 [US2] Integrate OpenAI API to generate responses based on retrieved content
- [X] T031 [US2] Implement citation generation to reference specific book content
- [X] T032 [US2] Implement follow-up question generation logic
- [X] T033 [US2] Create chat history management using Neon Postgres
- [X] T034 [US2] Connect frontend widget to backend chat API
- [X] T035 [US2] Test chat functionality with various queries about book content
- [X] T036 [US2] Validate that chatbot responses include proper citations

## Phase 5: User Story 3 - Get Contextual Help from Selected Text (Priority: P2)

**Goal**: Allow readers to select text on any page and get contextual assistance from the chatbot that specifically relates to the selected content, rather than general book-wide knowledge.

**Independent Test**: When text is selected on a page, the chatbot can provide answers that are specifically relevant to only the selected text, filtering responses to be contextually appropriate.

- [X] T037 [P] [US3] Enhance chat request model to include selected text context in rag/fastapi_app/models/chat_request.py
- [X] T038 [US3] Update frontend widget to capture selected text in book/src/components/chatbot-widget/ChatWidget.jsx
- [X] T039 [US3] Modify chat handler to prioritize selected text context in rag/api/chat_handler.py
- [X] T040 [US3] Implement logic to search both selected text context and general book content
- [X] T041 [US3] Update citation logic to indicate source of information (selected text vs general content)
- [X] T042 [US3] Test contextual help functionality with selected text
- [X] T043 [US3] Validate that responses are specifically related to selected text

## Phase 6: Polish & Cross-Cutting Concerns

**Goal**: Implement cross-cutting concerns and polish the implementation to meet all requirements

- [X] T044 Implement content search endpoint /content/search in rag/fastapi_app/routers/content.py
- [X] T045 Implement content retrieval endpoint /content/{contentId} in rag/fastapi_app/routers/content.py
- [X] T046 Add chat history retrieval endpoint /chat/history in rag/fastapi_app/routers/chat.py
- [X] T047 Implement authentication for API endpoints if required
- [X] T048 Add comprehensive error handling for all endpoints
- [X] T049 Implement caching for improved performance
- [X] T050 Add logging and monitoring capabilities
- [X] T051 Optimize Docusaurus site for performance (load times under 3 seconds)
- [X] T052 Add proper error pages and handling in Docusaurus
- [X] T053 Implement security best practices for web application
- [X] T054 Write comprehensive tests for backend services (pytest)
- [X] T055 Write tests for frontend components (Jest)
- [X] T056 Document API endpoints with proper OpenAPI specifications
- [X] T057 Create comprehensive documentation for deployment and maintenance

## Dependencies

### User Story Completion Order
1. Foundational phase must complete before any user story
2. US1 must be complete before US2 and US3 (requires book content)
3. US2 and US3 can be developed in parallel but both depend on foundational components

### Component Dependencies
- Docusaurus site (US1) requires book content and configuration files
- Chatbot API (US2) requires models, database clients, and content embeddings
- Selected text functionality (US3) requires frontend capture and backend context handling
- All API endpoints depend on the foundational models and database clients

## Parallel Execution Opportunities

### Within US1:
- T019 and T020 can run in parallel to create different content files
- T021 and T022 can run in parallel while working on navigation

### Within US2:
- T027 and T028 can run in parallel to implement the API endpoint and handler
- T031, T032, and T033 can run in parallel for different response features

### Within US3:
- T037 can be done in parallel with frontend work (T038)
- T039 and T040 can run in parallel for the enhancement logic

## Implementation Strategy

### MVP Scope (US1 Only)
The minimum viable product includes just User Story 1: a Docusaurus-based book with structured chapters that can be built and deployed to GitHub Pages. This provides immediate value with content creation and viewing capabilities.

### Incremental Delivery
- Phase 1-2: Set up foundational components that enable all user stories
- US1: Core book functionality for content creators and readers
- US2: Enhanced experience with AI-powered chatbot for general queries
- US3: Advanced feature for contextual help with selected text
- Final phase: Polish and optimization for production-ready system

This approach allows for testing and validation of each user story independently while building toward the complete feature.