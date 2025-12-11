"""
Content Management System for Robotics Curriculum Modules

This module provides functionality to manage curriculum content including:
- Module creation and editing
- Content validation
- Asset management
- Version control
- Publishing workflow
"""

import os
import json
import yaml
from typing import Dict, List, Optional, Any
from datetime import datetime
from pathlib import Path
import shutil
from dataclasses import dataclass, asdict
from enum import Enum


class ContentType(Enum):
    """Enumeration of content types in the curriculum"""
    MODULE = "module"
    LESSON = "lesson"
    EXERCISE = "exercise"
    DOCUMENTATION = "documentation"
    ASSET = "asset"
    ASSESSMENT = "assessment"


@dataclass
class CurriculumItem:
    """Data class representing a curriculum item"""
    id: str
    title: str
    content_type: ContentType
    description: str
    content: str
    author: str
    created_date: datetime
    updated_date: datetime
    version: str = "1.0"
    tags: List[str] = None
    prerequisites: List[str] = None
    estimated_duration: int = 0  # in minutes
    difficulty: str = "beginner"  # beginner, intermediate, advanced
    status: str = "draft"  # draft, reviewed, published, archived
    assets: List[str] = None
    related_items: List[str] = None

    def __post_init__(self):
        if self.tags is None:
            self.tags = []
        if self.prerequisites is None:
            self.prerequisites = []
        if self.assets is None:
            self.assets = []
        if self.related_items is None:
            self.related_items = []


class ContentManager:
    """
    Core content management system for curriculum materials
    """

    def __init__(self, content_base_path: str = "E:/PAHR-book/docusaurus-book/docs"):
        """
        Initialize the content manager

        Args:
            content_base_path: Base path for curriculum content storage
        """
        self.content_base_path = Path(content_base_path)
        self.content_base_path.mkdir(parents=True, exist_ok=True)

        # Create subdirectories for different content types
        self.directories = {
            "modules": self.content_base_path / "modules",
            "lessons": self.content_base_path / "lessons",
            "exercises": self.content_base_path / "exercises",
            "assets": self.content_base_path / "assets",
            "assessments": self.content_base_path / "assessments"
        }

        for dir_path in self.directories.values():
            dir_path.mkdir(exist_ok=True)

        # File to store content metadata
        self.metadata_file = self.content_base_path / "content_metadata.json"
        self.load_metadata()

    def load_metadata(self):
        """Load existing metadata from file"""
        if self.metadata_file.exists():
            with open(self.metadata_file, 'r', encoding='utf-8') as f:
                self.metadata = json.load(f)
        else:
            self.metadata = {"items": {}, "versions": {}}

    def save_metadata(self):
        """Save metadata to file"""
        with open(self.metadata_file, 'w', encoding='utf-8') as f:
            json.dump(self.metadata, f, indent=2, default=str)

    def create_item(self, item: CurriculumItem) -> bool:
        """
        Create a new curriculum item

        Args:
            item: CurriculumItem object to create

        Returns:
            True if successful, False otherwise
        """
        try:
            # Determine the appropriate directory based on content type
            content_dir_mapping = {
                ContentType.MODULE.value: 'modules',
                ContentType.LESSON.value: 'lessons',
                ContentType.EXERCISE.value: 'exercises',
                ContentType.DOCUMENTATION.value: 'lessons',  # Map to lessons
                ContentType.ASSET.value: 'assets',  # This will be handled separately
                ContentType.ASSESSMENT.value: 'assessments'
            }

            dir_key = content_dir_mapping.get(item.content_type.value)
            if not dir_key:
                print(f"Unknown content type: {item.content_type.value}")
                return False

            content_dir = self.directories[dir_key]

            # Create file path
            file_path = content_dir / f"{item.id}.md"

            # Create the content file
            with open(file_path, 'w', encoding='utf-8') as f:
                # Write content with frontmatter-like metadata
                f.write("---\n")
                f.write(f"title: {item.title}\n")
                f.write(f"type: {item.content_type.value}\n")
                f.write(f"author: {item.author}\n")
                f.write(f"version: {item.version}\n")
                f.write(f"difficulty: {item.difficulty}\n")
                f.write(f"estimated_duration: {item.estimated_duration} minutes\n")
                f.write(f"tags: {item.tags}\n")
                f.write(f"prerequisites: {item.prerequisites}\n")
                f.write(f"created_date: {item.created_date.isoformat()}\n")
                f.write(f"updated_date: {item.updated_date.isoformat()}\n")
                f.write(f"status: {item.status}\n")
                f.write("---\n\n")
                f.write(item.content)

            # Update metadata
            self.metadata["items"][item.id] = asdict(item)
            self.metadata["versions"][item.id] = item.version

            self.save_metadata()
            print(f"Created {item.content_type.value}: {item.title}")
            return True
        except Exception as e:
            print(f"Error creating item {item.id}: {str(e)}")
            return False

    def update_item(self, item: CurriculumItem) -> bool:
        """
        Update an existing curriculum item

        Args:
            item: CurriculumItem object with updated information

        Returns:
            True if successful, False otherwise
        """
        try:
            # Find the current file for the item
            file_path = None
            for dir_path in self.directories.values():
                potential_path = dir_path / f"{item.id}.md"
                if potential_path.exists():
                    file_path = potential_path
                    break

            if not file_path:
                print(f"Item {item.id} not found")
                return False

            # Update the content file
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write("---\n")
                f.write(f"title: {item.title}\n")
                f.write(f"type: {item.content_type.value}\n")
                f.write(f"author: {item.author}\n")
                f.write(f"version: {item.version}\n")
                f.write(f"difficulty: {item.difficulty}\n")
                f.write(f"estimated_duration: {item.estimated_duration} minutes\n")
                f.write(f"tags: {item.tags}\n")
                f.write(f"prerequisites: {item.prerequisites}\n")
                f.write(f"created_date: {item.created_date.isoformat()}\n")
                f.write(f"updated_date: {item.updated_date.isoformat()}\n")
                f.write(f"status: {item.status}\n")
                f.write("---\n\n")
                f.write(item.content)

            # Update metadata
            self.metadata["items"][item.id] = asdict(item)
            self.metadata["versions"][item.id] = item.version

            self.save_metadata()
            print(f"Updated {item.content_type.value}: {item.title}")
            return True
        except Exception as e:
            print(f"Error updating item {item.id}: {str(e)}")
            return False

    def get_item(self, item_id: str) -> Optional[CurriculumItem]:
        """
        Get a curriculum item by ID

        Args:
            item_id: ID of the item to retrieve

        Returns:
            CurriculumItem object or None if not found
        """
        item_data = self.metadata["items"].get(item_id)
        if not item_data:
            print(f"Item {item_id} not found in metadata")
            return None

        # Convert dict back to CurriculumItem
        item_data['content_type'] = ContentType(item_data['content_type'])
        if isinstance(item_data['created_date'], str):
            item_data['created_date'] = datetime.fromisoformat(item_data['created_date'])
        if isinstance(item_data['updated_date'], str):
            item_data['updated_date'] = datetime.fromisoformat(item_data['updated_date'])

        return CurriculumItem(**item_data)

    def list_items(self, content_type: Optional[ContentType] = None,
                   status: Optional[str] = None) -> List[CurriculumItem]:
        """
        List curriculum items with optional filters

        Args:
            content_type: Filter by content type
            status: Filter by publication status

        Returns:
            List of CurriculumItem objects matching the criteria
        """
        items = []
        for item_id, item_data in self.metadata["items"].items():
            # Apply filters
            if content_type and ContentType(item_data['content_type']) != content_type:
                continue
            if status and item_data['status'] != status:
                continue

            # Convert to CurriculumItem
            item_data_copy = item_data.copy()
            item_data_copy['content_type'] = ContentType(item_data_copy['content_type'])
            if isinstance(item_data_copy['created_date'], str):
                item_data_copy['created_date'] = datetime.fromisoformat(item_data_copy['created_date'])
            if isinstance(item_data_copy['updated_date'], str):
                item_data_copy['updated_date'] = datetime.fromisoformat(item_data_copy['updated_date'])

            items.append(CurriculumItem(**item_data_copy))

        return items

    def publish_item(self, item_id: str) -> bool:
        """
        Publish a curriculum item (change status to published)

        Args:
            item_id: ID of the item to publish

        Returns:
            True if successful, False otherwise
        """
        item = self.get_item(item_id)
        if not item:
            return False

        item.status = "published"
        item.updated_date = datetime.now()
        return self.update_item(item)

    def unpublish_item(self, item_id: str) -> bool:
        """
        Unpublish a curriculum item (change status to reviewed)

        Args:
            item_id: ID of the item to unpublish

        Returns:
            True if successful, False otherwise
        """
        item = self.get_item(item_id)
        if not item:
            return False

        if item.status == "published":
            item.status = "reviewed"
            item.updated_date = datetime.now()
            return self.update_item(item)
        else:
            return True  # Already unpublished

    def delete_item(self, item_id: str) -> bool:
        """
        Delete a curriculum item

        Args:
            item_id: ID of the item to delete

        Returns:
            True if successful, False otherwise
        """
        # Find and delete the content file
        file_path = None
        for dir_path in self.directories.values():
            potential_path = dir_path / f"{item_id}.md"
            if potential_path.exists():
                file_path = potential_path
                break

        if file_path:
            try:
                file_path.unlink()  # Remove the file
            except OSError as e:
                print(f"Error deleting file {file_path}: {str(e)}")
                return False

        # Remove from metadata
        if item_id in self.metadata["items"]:
            del self.metadata["items"][item_id]
        if item_id in self.metadata["versions"]:
            del self.metadata["versions"][item_id]

        self.save_metadata()
        print(f"Deleted item: {item_id}")
        return True

    def search_items(self, query: str) -> List[CurriculumItem]:
        """
        Search for items by content

        Args:
            query: Search term to look for

        Returns:
            List of CurriculumItem objects matching the query
        """
        results = []
        query_lower = query.lower()

        for item_id, item_data in self.metadata["items"].items():
            # Search in title, description, and tags
            searchable_text = (
                item_data['title'].lower() +
                item_data['description'].lower() +
                item_data['content'].lower() +
                ' '.join(item_data['tags']).lower() +
                ' '.join(item_data.get('prerequisites', [])).lower()
            )

            if query_lower in searchable_text:
                # Convert to CurriculumItem
                item_data_copy = item_data.copy()
                item_data_copy['content_type'] = ContentType(item_data_copy['content_type'])
                if isinstance(item_data_copy['created_date'], str):
                    item_data_copy['created_date'] = datetime.fromisoformat(item_data_copy['created_date'])
                if isinstance(item_data_copy['updated_date'], str):
                    item_data_copy['updated_date'] = datetime.fromisoformat(item_data_copy['updated_date'])

                results.append(CurriculumItem(**item_data_copy))

        return results

    def bulk_import_from_directory(self, source_dir: str) -> List[str]:
        """
        Import curriculum content from a source directory

        Args:
            source_dir: Path to directory containing content to import

        Returns:
            List of IDs of successfully imported items
        """
        source_path = Path(source_dir)
        if not source_path.exists():
            print(f"Source directory {source_dir} does not exist")
            return []

        imported_ids = []

        # Process markdown files in the source directory
        for file_path in source_path.rglob("*.md"):
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()

                # Attempt to extract metadata from the content (frontmatter)
                metadata = self._extract_metadata_from_content(content)

                # Create a new item ID based on filename
                item_id = file_path.stem.replace(" ", "_").replace("-", "_")

                # Create a CurriculumItem from the extracted data
                curriculum_item = CurriculumItem(
                    id=item_id,
                    title=metadata.get("title", file_path.stem),
                    content_type=ContentType(metadata.get("type", "documentation")),
                    description=metadata.get("description", content[:200]),
                    content=metadata.get("content", content),
                    author=metadata.get("author", "system"),
                    created_date=datetime.fromisoformat(metadata.get("created_date", datetime.now().isoformat())),
                    updated_date=datetime.fromisoformat(metadata.get("updated_date", datetime.now().isoformat())),
                    version=metadata.get("version", "1.0"),
                    tags=metadata.get("tags", []),
                    prerequisites=metadata.get("prerequisites", []),
                    estimated_duration=metadata.get("estimated_duration", 0),
                    difficulty=metadata.get("difficulty", "beginner"),
                    status=metadata.get("status", "draft")
                )

                # Create the item in the CMS
                if self.create_item(curriculum_item):
                    imported_ids.append(item_id)

            except Exception as e:
                print(f"Error importing {file_path}: {str(e)}")

        return imported_ids

    def _extract_metadata_from_content(self, content: str) -> Dict[str, Any]:
        """
        Extract metadata from content (frontmatter-like format)

        Args:
            content: Content string to parse

        Returns:
            Dictionary containing extracted metadata
        """
        # Try to find YAML frontmatter
        lines = content.split('\n')
        if len(lines) > 2 and lines[0] == '---' and '---' in lines[1:]:
            # Find end of frontmatter
            end_frontmatter_idx = 1
            for i, line in enumerate(lines[1:], 1):
                if line.strip() == '---':
                    end_frontmatter_idx = i
                    break

            # Parse the frontmatter
            frontmatter = '\n'.join(lines[1:end_frontmatter_idx])
            try:
                metadata = yaml.safe_load(frontmatter)
                if metadata is None:
                    metadata = {}
            except yaml.YAMLError:
                metadata = {}

            # Include the actual content
            actual_content = '\n'.join(lines[end_frontmatter_idx+1:])
            metadata['content'] = actual_content.strip()
            return metadata
        else:
            # No frontmatter found, return content in a simple structure
            return {
                'content': content,
                'title': 'Untitled',
                'author': 'unknown',
                'created_date': datetime.now().isoformat(),
                'updated_date': datetime.now().isoformat(),
                'status': 'draft'
            }


# Example usage and demonstration
if __name__ == "__main__":
    # Initialize the content manager
    cms = ContentManager()

    print("Content Management System initialized!")
    print(f"Content base path: {cms.content_base_path}")

    # Create sample content items
    sample_module = CurriculumItem(
        id="intro_to_ros2_module",
        title="Introduction to ROS 2",
        content_type=ContentType.MODULE,
        description="An introductory module to Robot Operating System 2",
        content="# Introduction to ROS 2\n\nThis module introduces the fundamental concepts of ROS 2...",
        author="Curriculum Team",
        created_date=datetime.now(),
        updated_date=datetime.now(),
        version="1.0",
        tags=["ROS2", "introduction", "middleware"],
        prerequisites=[],
        estimated_duration=120,
        difficulty="beginner",
        status="draft"
    )

    sample_lesson = CurriculumItem(
        id="ros2_nodes_lesson",
        title="ROS 2 Nodes",
        content_type=ContentType.LESSON,
        description="Understanding Nodes in ROS 2",
        content="# ROS 2 Nodes\n\nA node is a process that performs computation in a ROS program...",
        author="Curriculum Team",
        created_date=datetime.now(),
        updated_date=datetime.now(),
        version="1.0",
        tags=["ROS2", "nodes", "processes"],
        prerequisites=["intro_to_ros2_module"],
        estimated_duration=90,
        difficulty="intermediate",
        status="draft"
    )

    sample_exercise = CurriculumItem(
        id="create_simple_node_exercise",
        title="Create a Simple Node",
        content_type=ContentType.EXERCISE,
        description="Exercise: Create and run a simple ROS 2 node",
        content="# Exercise: Create a Simple Node\n\nIn this exercise you will create a basic ROS 2 node...",
        author="Curriculum Team",
        created_date=datetime.now(),
        updated_date=datetime.now(),
        version="1.0",
        tags=["ROS2", "nodes", "exercise", "python"],
        prerequisites=["ros2_nodes_lesson"],
        estimated_duration=180,
        difficulty="intermediate",
        status="draft"
    )

    # Create the items
    print("\nCreating sample content...")
    cms.create_item(sample_module)
    cms.create_item(sample_lesson)
    cms.create_item(sample_exercise)

    # List items
    print("\nListing all modules:")
    modules = cms.list_items(content_type=ContentType.MODULE)
    for mod in modules:
        print(f"  - {mod.title} ({mod.status})")

    # Get a specific item
    print(f"\nRetrieving module: {sample_module.id}")
    retrieved_module = cms.get_item(sample_module.id)
    if retrieved_module:
        print(f"  Title: {retrieved_module.title}")
        print(f"  Content type: {retrieved_module.content_type}")
        print(f"  Difficulty: {retrieved_module.difficulty}")

    # Publish the module
    print(f"\nPublishing {sample_module.id}")
    cms.publish_item(sample_module.id)

    # Search for items
    print("\nSearching for 'ROS 2'...")
    search_results = cms.search_items("ROS 2")
    for item in search_results:
        print(f"  - {item.title} ({item.content_type.value})")

    print("\nContent Management System demo completed successfully!")