---
id: 002
title: "ai-book-rag-specification"
stage: "spec"
date_iso: "2025-12-12"
surface: "agent"
model: "unknown"
feature: "ai-book-rag"
branch: "002-ai-book-rag"
user: "unknown"
command: "/sp.specify"
labels:
  - "specification"
  - "ai-book"
  - "rag-chatbot"
  - "docusaurus"
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - "specs/002-ai-book-rag/spec.md"
  - "specs/002-ai-book-rag/checklists/requirements.md"
tests_yaml: []
prompt_text: |
  $ARGUMENTS
  You are to create a complete AI/Spec-Driven Book Project using **Spec-Kit Plus** and **Claude Code**. ## 🎯 Project Goal Generate a full, production-ready **book project** built with **Docusaurus**, containing: - structured chapters, - module-level and lesson-level content, - assets folder, - sidebar navigation, - front-matter metadata, - and a build-ready GitHub Pages deployment configuration. Follow all requirements below. --- ## 📘 Core Deliverables (MANDATORY) ### **1. Book Creation (Spec-Kit + Claude Code)** Generate: - `/sp.spec` for the entire book - `/book/` folder containing: - Chapters (Markdown) - Sub-chapters - Code snippets - Diagrams (Mermaid) - Practice questions - `/sidebars.js` - `/docusaurus.config.js` - `/static/` assets - GitHub Pages config files All chapters must follow: - clear hierarchy - front-matter fields: `id`, `title`, `description`, `keywords`, `tags` - clean, multi-section content - examples - exercises --- ## 🤖 2. Integrated RAG Chatbot Development (MANDATORY) Create a full RAG chatbot system embedded inside the Docusaurus book. ### The chatbot must include: - FastAPI backend - Qdrant Cloud (Free Tier) vector DB - Neon Serverless Postgres (metadata store) - OpenAI Agents / ChatKit SDK - Retrieval on: - all book content - AND any text the user selects on the page ### Deliverables: - `/rag/fastapi_app/main.py` - `/rag/loader/load_markdown.py` - `/rag/db/qdrant_client.py` - `/rag/db/postgres_client.py` - `/rag/api/chat_handler.py` - `/rag/ui/widget.jsx` (frontend widget injected into Docusaurus layout) - `/rag/specs/rag.spec` (Spec-Kit Plus spec for chatbot) The chatbot must be able to: 1. Answer questions about any chapter. 2. Filter answers strictly to book content. 3. Answer questions based *only* on highlighted/selected text. 4. Provide citations. 5. Provide follow-up questions for learning. --- ## 🧱 Folder Structure (MANDATORY) Generate:
response_text: |
  Created feature specification for AI-Driven Book with Integrated RAG Chatbot. Generated complete spec at specs/002-ai-book-rag/spec.md with 3 prioritized user stories, 12 functional requirements, 7 success criteria, and key entities. Created quality checklist at specs/002-ai-book-rag/checklists/requirements.md. Feature branch 002-ai-book-rag created successfully.
outcome: |
  Successfully created feature specification for AI-driven book project with RAG chatbot. Specification is complete and ready for planning phase.
evaluation: |
  Feature specification successfully created according to requirements with all mandatory sections completed.
---

You are to create a complete AI/Spec-Driven Book Project using **Spec-Kit Plus** and **Claude Code**. ## 🎯 Project Goal Generate a full, production-ready **book project** built with **Docusaurus**, containing: - structured chapters, - module-level and lesson-level content, - assets folder, - sidebar navigation, - front-matter metadata, - and a build-ready GitHub Pages deployment configuration. Follow all requirements below. --- ## 📘 Core Deliverables (MANDATORY) ### **1. Book Creation (Spec-Kit + Claude Code)** Generate: - `/sp.spec` for the entire book - `/book/` folder containing: - Chapters (Markdown) - Sub-chapters - Code snippets - Diagrams (Mermaid) - Practice questions - `/sidebars.js` - `/docusaurus.config.js` - `/static/` assets - GitHub Pages config files All chapters must follow: - clear hierarchy - front-matter fields: `id`, `title`, `description`, `keywords`, `tags` - clean, multi-section content - examples - exercises --- ## 🤖 2. Integrated RAG Chatbot Development (MANDATORY) Create a full RAG chatbot system embedded inside the Docusaurus book. ### The chatbot must include: - FastAPI backend - Qdrant Cloud (Free Tier) vector DB - Neon Serverless Postgres (metadata store) - OpenAI Agents / ChatKit SDK - Retrieval on: - all book content - AND any text the user selects on the page ### Deliverables: - `/rag/fastapi_app/main.py` - `/rag/loader/load_markdown.py` - `/rag/db/qdrant_client.py` - `/rag/db/postgres_client.py` - `/rag/api/chat_handler.py` - `/rag/ui/widget.jsx` (frontend widget injected into Docusaurus layout) - `/rag/specs/rag.spec` (Spec-Kit Plus spec for chatbot) The chatbot must be able to: 1. Answer questions about any chapter. 2. Filter answers strictly to book content. 3. Answer questions based *only* on highlighted/selected text. 4. Provide citations. 5. Provide follow-up questions for learning. --- ## 🧱 Folder Structure (MANDATORY) Generate: