/**
 * Deployment script for Docusaurus site to GitHub Pages
 * This script builds and deploys the site using the docusaurus build command
 */

const { execSync } = require('child_process');
const fs = require('fs');
const path = require('path');

// Configuration
const DEPLOYMENT_BRANCH = 'gh-pages';
const BUILD_DIR = 'build';

console.log('Starting GitHub Pages deployment...');

try {
  // Check if in correct directory
  if (!fs.existsSync('docusaurus.config.js')) {
    console.error('Error: docusaurus.config.js not found. Please run this script from the docusaurus directory.');
    process.exit(1);
  }

  // Build the site
  console.log('Building the site...');
  execSync('npm run build', { stdio: 'inherit' });

  // Verify build directory exists
  if (!fs.existsSync(BUILD_DIR)) {
    console.error(`Error: Build directory ${BUILD_DIR} does not exist after build.`);
    process.exit(1);
  }

  console.log(`Site built successfully in ${BUILD_DIR}/`);

  // Check if we're in a git repository
  let isGitRepo = false;
  try {
    execSync('git status', { stdio: 'pipe' });
    isGitRepo = true;
  } catch (error) {
    // Not in a git repo
    isGitRepo = false;
  }

  if (isGitRepo) {
    // Deploy using docusaurus command
    console.log('Deploying to GitHub Pages...');
    execSync('npx docusaurus deploy', { stdio: 'inherit' });
  } else {
    console.log('Not in a git repository, creating deployment directory...');
    // If not in git repo, just prepare the build directory for manual deployment
    console.log(`Build is ready in ${path.join(process.cwd(), BUILD_DIR)}`);
    console.log('You can now upload the contents of this directory to your GitHub Pages branch.');
  }

  console.log('Deployment process completed successfully!');
} catch (error) {
  console.error('Deployment failed:', error.message);
  process.exit(1);
}