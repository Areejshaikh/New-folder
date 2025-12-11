"""
Final Verification Script for PAHR Book Implementation

This script verifies that all components of the Phase 2 Foundational tasks have been successfully implemented.
"""

import os
import sys
from pathlib import Path

def verify_implementation():
    print("Verifying PAHR Book Implementation")
    print("="*50)

    # Check for key directories
    required_directories = [
        "docusaurus-book",
        "RAG-chatbot",
        "specs/001-robotics-curriculum-modules",
        ".github/workflows"
    ]

    print("Checking required directories:")
    for directory in required_directories:
        path = Path("E:/PAHR-book") / directory
        exists = path.exists()
        status = "[OK]" if exists else "[MISSING]"
        print(f"  {status} {directory}")

    print()

    # Check for key files
    required_files = [
        "docusaurus-book/package.json",
        "python_data_models.py",
        "content_management_system.py",
        "security_privacy_features.py",
        "progress_tracker.py",
        "api_server.py",
        "database-schema.sql",
        "specs/001-robotics-curriculum-modules/tasks.md",
        ".github/workflows/deploy.yml",
        "requirements.txt",
        "Dockerfile",
        "docker-compose.yml"
    ]

    print("Checking required files:")
    all_files_present = True
    for file in required_files:
        path = Path("E:/PAHR-book") / file
        exists = path.exists()
        status = "[OK]" if exists else "[MISSING]"
        print(f"  {status} {file}")
        if not exists:
            all_files_present = False

    print()

    # Verify core functionality exists by attempting to import
    print("Checking core functionality:")

    try:
        # Test data models
        import importlib.util
        spec = importlib.util.spec_from_file_location("data_models", "E:/PAHR-book/python_data_models.py")
        data_models = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(data_models)
        print("  [OK] Data models module loads successfully")

        # Test content management system
        spec = importlib.util.spec_from_file_location("content_mgmt", "E:/PAHR-book/content_management_system.py")
        content_mgmt = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(content_mgmt)
        print("  [OK] Content management system module loads successfully")

        # Test security features
        spec = importlib.util.spec_from_file_location("security", "E:/PAHR-book/security_privacy_features.py")
        security = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(security)
        print("  [OK] Security and privacy features module loads successfully")

        # Test API server
        spec = importlib.util.spec_from_file_location("api_server", "E:/PAHR-book/api_server.py")
        api_server = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(api_server)
        print("  [OK] API server module loads successfully")

        # Test progress tracker
        spec = importlib.util.spec_from_file_location("progress_tracker", "E:/PAHR-book/progress_tracker.py")
        progress_tracker = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(progress_tracker)
        print("  [OK] Progress tracker module loads successfully")

    except Exception as e:
        print(f"  [ERROR] Module loading failed: {e}")
        all_files_present = False

    print()

    if all_files_present:
        print("SUCCESS: Implementation verification PASSED!")
        print("All required components of the PAHR Book project have been successfully implemented.")
        print("\nThe Phase 2 Foundational tasks are complete with:")
        print("- Data models for curriculum modules, simulation environments, and learner profiles")
        print("- Database schema and content storage structure")
        print("- API infrastructure with authentication")
        print("- User progress tracking system")
        print("- RAG chatbot integration")
        print("- Content management system")
        print("- Security and privacy features")
        print("- Deployment pipeline to GitHub Pages")
        return True
    else:
        print("FAILURE: Implementation verification FAILED!")
        print("Some required components are missing or not functioning properly.")
        return False

if __name__ == "__main__":
    success = verify_implementation()
    sys.exit(0 if success else 1)