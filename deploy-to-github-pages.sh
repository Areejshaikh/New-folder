#!/bin/bash
# Deployment script for PAHR Book to GitHub Pages

set -e  # Exit immediately if a command exits with a non-zero status

echo "Starting deployment to GitHub Pages..."

# Configuration
REPO_URL="https://github.com/username/pahr-book.git"  # Replace with the actual repository
BRANCH="gh-pages"
BUILD_DIR="./build"

# Check if we're in the right directory
if [ ! -f "package.json" ]; then
    echo "Error: package.json not found. Are you in the docusaurus-book directory?"
    exit 1
fi

# Build the Docusaurus site
echo "Building Docusaurus site..."
npm run build

# Check if build was successful
if [ ! -d "$BUILD_DIR" ]; then
    echo "Error: Build directory not found. Build may have failed."
    exit 1
fi

# Store the current commit hash
CURRENT_COMMIT=$(git rev-parse HEAD)

# Create a temporary directory for deployment
TEMP_DEPLOY_DIR=$(mktemp -d)

echo "Preparing deployment directory..."
# Copy built files to temporary directory
cp -r $BUILD_DIR/* $TEMP_DEPLOY_DIR/

# Navigate to temporary directory
cd $TEMP_DEPLOY_DIR

# Initialize git repo
git init
git remote add origin $REPO_URL
git checkout -b $BRANCH

# Add all files
git add .

# Configure git identity
git config --local user.email "deployment-bot@example.com"
git config --local user.name "Deployment Bot"

# Commit changes
git commit -m "Deploy: PAHR Book site [ci skip] - $CURRENT_COMMIT"

# Force push to gh-pages branch
git push -f origin $BRANCH

# Return to original directory
cd - > /dev/null

# Cleanup
rm -rf $TEMP_DEPLOY_DIR

echo "Deployment completed successfully!"