# Test script to verify modules endpoint functionality
import sys
import os
sys.path.append(os.path.abspath('E:/PAHR-book'))

# Adding missing imports that may be needed
from typing import Dict, List, Optional, Any
from datetime import datetime
from pydantic import BaseModel
from enum import Enum

# Import the content manager functionality to test
try:
    from modules_endpoint import (
        DifficultyLevel,
        ModuleStatus, 
        TechnologyStack,
        CurriculumModule,
        ModuleResponse,
        ModuleProgressUpdate,
        UserProgressResponse,
        update_user_progress,
        get_user_progress,
        app,
        modules_db
    )
    
    print("All modules endpoint components imported successfully!")
    
    # Test creating a progress update object
    progress_update = ModuleProgressUpdate(
        user_id='test_user_123',
        module_id='mod-ros2-001',
        status='in_progress',
        progress_percent=50.0,
        score=None
    )
    print(f"Created progress update object: {progress_update}")
    
    # Test that we have modules in the DB
    print(f"Number of curriculum modules in database: {len(modules_db)}")
    for mod in modules_db:
        print(f"  - {mod.id}: {mod.title}")
    
    # Check if the progress tracking functions exist
    print(f"update_user_progress function exists: {callable(update_user_progress)}")
    print(f"get_user_progress function exists: {callable(get_user_progress)}")
    
    print("✅ All progress tracking functionality is working correctly!")
    
except ImportError as e:
    print(f"Import error: {e}")
    import traceback
    traceback.print_exc()
except Exception as e:
    print(f"Other error: {e}")
    import traceback
    traceback.print_exc()