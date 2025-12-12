# Data Model: AI-Driven Book with Integrated RAG Chatbot

## Overview
This document defines the data models for the AI-driven book with integrated RAG chatbot system, including entities, relationships, and validation rules derived from the feature requirements.

## Entity: Book Content
**Description**: Represents educational material including chapters, modules, lessons, text, images, diagrams, code snippets, and practice questions with associated metadata
**Fields**:
- `id` (string): Unique identifier for the content piece (required)
- `title` (string): Title of the content piece (required)
- `description` (string): Brief description of the content (required)
- `keywords` (array of strings): Keywords associated with the content (required)
- `tags` (array of strings): Tags for categorization (required)
- `type` (string): Type of content (e.g., 'chapter', 'module', 'lesson', 'diagram', 'code') (required)
- `content` (string): The actual content in markdown format (required)
- `parent_id` (string): ID of parent content (optional, null for root level)
- `order` (integer): Order position in the hierarchy (required)
- `created_at` (timestamp): Creation timestamp (required)
- `updated_at` (timestamp): Last update timestamp (required)
- `version` (string): Content version (required)

**Validation Rules**:
- `id` must be unique across all content
- `title`, `description`, `keywords`, and `tags` must not be empty
- `type` must be one of the predefined types
- `order` must be a positive integer

## Entity: User Query
**Description**: Represents questions or requests submitted by readers to the RAG chatbot system
**Fields**:
- `id` (string): Unique identifier for the query (required)
- `user_id` (string): ID of the user who submitted the query (optional for anonymous)
- `query_text` (string): The text of the user's query (required)
- `selected_text` (string): Text selected on the page when the query was made (optional)
- `context_page` (string): Page/section where the query originated (optional)
- `timestamp` (timestamp): Time when the query was submitted (required)
- `session_id` (string): ID to group related queries together (required)

**Validation Rules**:
- `query_text` must not be empty
- `query_text` must be less than 1000 characters
- If `selected_text` is provided, it must not exceed 500 characters

## Entity: Chatbot Response
**Description**: Represents answers and information provided by the AI system to user queries, including citations and follow-up suggestions
**Fields**:
- `id` (string): Unique identifier for the response (required)
- `query_id` (string): ID of the corresponding user query (required)
- `response_text` (string): The text of the chatbot's response (required)
- `citations` (array of objects): List of citations to book content (optional)
- `follow_up_questions` (array of strings): Suggested follow-up questions (optional)
- `confidence_score` (number): Confidence level of the response (0-1) (optional)
- `timestamp` (timestamp): Time when the response was generated (required)
- `model_used` (string): AI model used to generate the response (required)

**Citations Object Structure**:
- `content_id` (string): ID of the referenced book content (required)
- `title` (string): Title of the referenced content (required)
- `url` (string): URL to the referenced content (required)

**Validation Rules**:
- `response_text` must not be empty
- `citations` must reference valid book content IDs
- `confidence_score` must be between 0 and 1 if provided

## Entity: Content Metadata
**Description**: Represents structured information about book content including id, title, description, keywords, and tags
**Fields**:
- `content_id` (string): Reference to the book content (required)
- `version` (string): Version of the content these metadata apply to (required)
- `authors` (array of strings): Authors of the content (optional)
- `reviewers` (array of strings): Reviewers who have approved the content (optional)
- `status` (string): Status of the content ('draft', 'review', 'published') (required)
- `language` (string): Language of the content (default: 'en') (optional)
- `estimated_reading_time` (integer): Estimated reading time in minutes (optional)
- `related_content_ids` (array of strings): IDs of related content pieces (optional)

**Validation Rules**:
- `content_id` must reference a valid book content
- `status` must be one of the predefined values
- `estimated_reading_time` must be a positive integer if provided

## Entity: Text Selection
**Description**: Represents highlighted text on a page that serves as context for targeted queries to the chatbot
**Fields**:
- `id` (string): Unique identifier for the selection (required)
- `content_id` (string): ID of the content where text was selected (required)
- `text` (string): The actual selected text (required)
- `start_position` (integer): Character position where selection starts (required)
- `end_position` (integer): Character position where selection ends (required)
- `html_element` (string): HTML element type where text was selected (optional)
- `paragraph_number` (integer): Paragraph number within the content (optional)

**Validation Rules**:
- `text` must not be empty
- `start_position` must be less than `end_position`
- `start_position` and `end_position` must be valid indices within the content

## Relationships

```
Book Content (1) ←→ (0..n) Content Metadata
Book Content (1) ←→ (0..n) Text Selection
User Query (1) ←→ (1) Chatbot Response
User Query (0..n) ←→ (0..n) Text Selection
```

## State Transitions

### Content Status Transitions
- `draft` → `review` (when submitted for review)
- `review` → `draft` (when changes requested) 
- `review` → `published` (when approved)
- `published` → `draft` (when changes needed)

## Indexes
- Book Content: id (primary), parent_id (foreign key), type (secondary)
- User Query: id (primary), session_id (secondary), user_id (secondary)
- Chatbot Response: id (primary), query_id (foreign key), timestamp (secondary)
- Content Metadata: content_id (foreign key), version (secondary), status (secondary)
- Text Selection: id (primary), content_id (foreign key)