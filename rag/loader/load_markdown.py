import os
import glob
import asyncio
from typing import List, Dict
import markdown
from pathlib import Path
import re
from datetime import datetime
from rag.db.qdrant_client import qdrant_config
from rag.fastapi_app.models.book_content import BookContent


def extract_frontmatter(content: str) -> tuple:
    """
    Extract frontmatter from markdown content.
    Expected format:
    ---
    id: content-id
    title: Content Title
    description: Content description
    keywords: [keyword1, keyword2]
    tags: [tag1, tag2]
    type: chapter
    order: 1
    version: 1.0.0
    ---
    """
    frontmatter_pattern = r'^---\n(.*?)\n---\n(.*)'
    match = re.match(frontmatter_pattern, content, re.DOTALL)
    
    if match:
        frontmatter = match.group(1)
        content = match.group(2)
        
        # Parse frontmatter
        parsed_frontmatter = {}
        for line in frontmatter.split('\n'):
            line = line.strip()
            if line and ':' in line:
                key, value = line.split(':', 1)
                key = key.strip()
                value = value.strip()
                
                # Handle array values
                if value.startswith('[') and value.endswith(']'):
                    value = [item.strip().strip('"\'') for item in value[1:-1].split(',')]
                
                # Handle numeric values
                if value.isdigit():
                    value = int(value)
                
                parsed_frontmatter[key] = value
        
        return parsed_frontmatter, content
    else:
        return {}, content


def process_markdown_file(file_path: str) -> BookContent:
    """
    Process a single markdown file and extract its content and metadata.
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    frontmatter, body = extract_frontmatter(content)
    
    # Get additional metadata from file path
    path_parts = Path(file_path).parts
    parent_dir = None
    if len(path_parts) >= 3:  # Format: book/docs/parent_dir/file.md
        parent_dir = path_parts[-2]
    
    # Create BookContent object
    book_content = BookContent(
        id=frontmatter.get('id', Path(file_path).stem),
        title=frontmatter.get('title', Path(file_path).stem.replace('-', ' ').title()),
        description=frontmatter.get('description', ''),
        keywords=frontmatter.get('keywords', []),
        tags=frontmatter.get('tags', []),
        type=frontmatter.get('type', 'lesson'),
        content=body,
        parent_id=frontmatter.get('parent_id', parent_dir),
        order=frontmatter.get('order', 1),
        version=frontmatter.get('version', '1.0.0')
    )
    
    return book_content


def load_markdown_files(directory: str) -> List[BookContent]:
    """
    Load all markdown files from the specified directory and its subdirectories.
    """
    book_contents = []
    
    # Find all markdown files
    md_files = glob.glob(os.path.join(directory, "**/*.md"), recursive=True)
    
    for file_path in md_files:
        try:
            book_content = process_markdown_file(file_path)
            book_contents.append(book_content)
        except Exception as e:
            print(f"Error processing file {file_path}: {e}")
    
    return book_contents


def index_content_to_qdrant(book_contents: List[BookContent]):
    """
    Index all book content into the Qdrant vector database.
    """
    for book_content in book_contents:
        # Prepare metadata for Qdrant
        metadata = {
            "title": book_content.title,
            "description": book_content.description,
            "keywords": book_content.keywords,
            "tags": book_content.tags,
            "type": book_content.type,
            "parent_id": book_content.parent_id,
            "order": book_content.order,
            "version": book_content.version
        }
        
        # Store content in Qdrant
        success = qdrant_config.store_content(
            content_id=book_content.id,
            content=book_content.content,
            metadata=metadata
        )
        
        if success:
            print(f"Successfully indexed {book_content.title} to Qdrant")
        else:
            print(f"Failed to index {book_content.title} to Qdrant")


def main():
    """
    Main function to load markdown files and index them to Qdrant.
    """
    # Directory containing markdown files
    markdown_dir = os.path.join(os.path.dirname(__file__), "..", "..", "book", "docs")
    
    print(f"Loading markdown files from {markdown_dir}")
    
    # Load markdown files
    book_contents = load_markdown_files(markdown_dir)
    
    print(f"Loaded {len(book_contents)} content items")
    
    # Index content to Qdrant
    index_content_to_qdrant(book_contents)
    
    print("Content loading and indexing complete")


if __name__ == "__main__":
    main()