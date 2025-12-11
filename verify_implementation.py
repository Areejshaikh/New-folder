#!/usr/bin/env python3
"""
Verification script to validate the implementation so far
"""

import os
import sys

def check_directory_structure():
    """Check if the project directory structure is correct"""
    expected_directories = [
        "docusaurus-book",
        "simulations",
        "simulations/ros2-examples",
        "simulations/gazebo-worlds",
        "simulations/unity-scenes",
        "simulations/isaac-sim",
        "notebooks",
        "notebooks/python",
        "notebooks/jupyter",
        "RAG-chatbot",
        "RAG-chatbot/src",
        "RAG-chatbot/data",
        "RAG-chatbot/models",
        "tests",
        "tests/integration",
        "tests/validation",
        "specs",
        "specs/001-robotics-curriculum-modules",
        "specs/001-robotics-curriculum-modules/checklists",
        "specs/001-robotics-curriculum-modules/contracts"
    ]

    print("Checking directory structure...")
    missing_dirs = []
    for directory in expected_directories:
        full_path = os.path.join("E:\\PAHR-book", directory)
        if not os.path.exists(full_path):
            missing_dirs.append(directory)

    if missing_dirs:
        print(f"Missing directories: {missing_dirs}")
        return False
    else:
        print("[PASS] Directory structure check passed")
        return True

def check_files():
    """Check if the key files we've created exist"""
    expected_files = [
        "E:\\PAHR-book\\requirements.txt",
        "E:\\PAHR-book\\.gitignore",
        "E:\\PAHR-book\\.dockerignore",
        "E:\\PAHR-book\\Dockerfile",
        "E:\\PAHR-book\\docker-compose.yml",
        "E:\\PAHR-book\\python_data_models.py",
        "E:\\PAHR-book\\database-schema.sql",
        "E:\\PAHR-book\\content-storage-structure.md",
        "E:\\PAHR-book\\api_server.py",
        "E:\\PAHR-book\\specs\\001-robotics-curriculum-modules\\tasks.md",
        "E:\\PAHR-book\\specs\\001-robotics-curriculum-modules\\checklists\\requirements.md"
    ]

    print("Checking key files...")
    missing_files = []
    for file_path in expected_files:
        if not os.path.exists(file_path):
            missing_files.append(file_path)

    if missing_files:
        print(f"Missing files: {missing_files}")
        return False
    else:
        print("[PASS] File check passed")
        return True

def check_api():
    """Try to import and validate the API server"""
    try:
        import api_server
        print("[PASS] API server module imported successfully")
        return True
    except ImportError as e:
        print(f"[FAIL] API server import failed: {e}")
        return False

def check_data_models():
    """Try to import and validate the data models"""
    try:
        import python_data_models
        print("[PASS] Data models module imported successfully")
        return True
    except ImportError as e:
        print(f"[FAIL] Data models import failed: {e}")
        return False

def main():
    print("=== Verification of Implementation Progress ===")

    checks = [
        ("Directory Structure", check_directory_structure),
        ("Key Files", check_files),
        ("API Server", check_api),
        ("Data Models", check_data_models)
    ]

    results = []
    for name, check_func in checks:
        print(f"\n{name}:")
        result = check_func()
        results.append((name, result))

    print(f"\n=== Summary ===")
    all_passed = True
    for name, result in results:
        status = "[PASS]" if result else "[FAIL]"
        print(f"{name}: {status}")
        if not result:
            all_passed = False

    if all_passed:
        print("\n[SUCCESS] All checks passed! Implementation is progressing well.")
        return 0
    else:
        print("\n[WARNING] Some checks failed. Please review the implementation.")
        return 1

if __name__ == "__main__":
    sys.exit(main())