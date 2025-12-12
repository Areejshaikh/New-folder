#!/usr/bin/env python3
"""
Content Validator
This script validates that all markdown files in the book content have the required front-matter fields.
"""
import os
import glob
import re
import sys
from pathlib import Path


def extract_frontmatter(content: str):
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
        return frontmatter
    else:
        return None


def parse_frontmatter(frontmatter: str) -> dict:
    """Parse frontmatter string into a dictionary"""
    parsed = {}
    for line in frontmatter.split('\n'):
        line = line.strip()
        if line and ':' in line:
            key, value = line.split(':', 1)
            key = key.strip()
            value = value.strip()
            
            # Handle array values
            if value.startswith('[') and value.endswith(']'):
                value = [item.strip().strip('"\'') for item in value[1:-1].split(',') if item.strip()]
            
            # Handle numeric values
            if value.isdigit():
                value = int(value)
            
            parsed[key] = value
    return parsed


def validate_content_file(file_path: str) -> tuple:
    """
    Validate a single content file for required front-matter fields.
    Returns (is_valid: bool, errors: list)
    """
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    frontmatter = extract_frontmatter(content)
    
    if not frontmatter:
        return False, ["No frontmatter found"]
    
    parsed_frontmatter = parse_frontmatter(frontmatter)
    
    # Required fields
    required_fields = {
        'id': str,
        'title': str,
        'description': str,
        'keywords': list,
        'tags': list,
        'type': str,
        'order': int,
        'version': str
    }
    
    errors = []
    
    for field, expected_type in required_fields.items():
        if field not in parsed_frontmatter:
            errors.append(f"Missing required field: {field}")
        else:
            value = parsed_frontmatter[field]
            # Type validation
            if expected_type == list and not isinstance(value, list):
                errors.append(f"Field {field} should be a list, got {type(value).__name__}")
            elif expected_type == str and not isinstance(value, str):
                errors.append(f"Field {field} should be a string, got {type(value).__name__}")
            elif expected_type == int and not isinstance(value, int):
                errors.append(f"Field {field} should be an integer, got {type(value).__name__}")
    
    # Additional validations
    if 'title' in parsed_frontmatter and not parsed_frontmatter['title'].strip():
        errors.append("Title field cannot be empty")
    
    if 'description' in parsed_frontmatter and not parsed_frontmatter['description'].strip():
        errors.append("Description field cannot be empty")
    
    if 'type' in parsed_frontmatter and parsed_frontmatter['type'] not in ['chapter', 'module', 'lesson', 'diagram', 'code']:
        errors.append(f"Invalid type '{parsed_frontmatter['type']}', must be one of: chapter, module, lesson, diagram, code")
    
    if 'order' in parsed_frontmatter and parsed_frontmatter['order'] < 1:
        errors.append("Order must be a positive integer")
    
    if 'keywords' in parsed_frontmatter and len(parsed_frontmatter['keywords']) == 0:
        errors.append("Keywords field cannot be empty")
    
    if 'tags' in parsed_frontmatter and len(parsed_frontmatter['tags']) == 0:
        errors.append("Tags field cannot be empty")
    
    return len(errors) == 0, errors


def validate_all_content_files(directory: str) -> dict:
    """
    Validate all markdown files in the specified directory and its subdirectories.
    Returns a dictionary with validation results.
    """
    results = {
        'total_files': 0,
        'valid_files': 0,
        'invalid_files': 0,
        'errors': {}
    }
    
    # Find all markdown files
    md_files = glob.glob(os.path.join(directory, "**/*.md"), recursive=True)
    
    for file_path in md_files:
        results['total_files'] += 1
        is_valid, errors = validate_content_file(file_path)
        
        if is_valid:
            results['valid_files'] += 1
        else:
            results['invalid_files'] += 1
            results['errors'][file_path] = errors
    
    return results


def main():
    # Directory containing markdown files
    markdown_dir = os.path.join(os.path.dirname(__file__), "..", "..", "book", "docs")
    
    print(f"Validating markdown files in {markdown_dir}")
    
    results = validate_all_content_files(markdown_dir)
    
    print(f"Total files: {results['total_files']}")
    print(f"Valid files: {results['valid_files']}")
    print(f"Invalid files: {results['invalid_files']}")
    
    if results['invalid_files'] > 0:
        print("\nValidation errors:")
        for file_path, errors in results['errors'].items():
            print(f"\n{file_path}:")
            for error in errors:
                print(f"  - {error}")
        
        return 1  # Return error code
    else:
        print("\nAll content files are valid!")
        return 0


if __name__ == "__main__":
    sys.exit(main())