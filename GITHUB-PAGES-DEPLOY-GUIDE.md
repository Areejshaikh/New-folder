# Deployment to GitHub Pages

This document provides instructions for deploying the PAHR (Physical AI & Humanoid Robotics) Book to GitHub Pages.

## Prerequisites

- Git installed on your system
- A GitHub repository created for your project
- Docusaurus site properly configured and tested locally

## Manual Deployment Process

### Step 1: Prepare Your Repository

1. Create a new GitHub repository or use an existing one
2. Push your Docusaurus code to the repository (usually to a branch like `main` or `master`)

### Step 2: Enable GitHub Pages

1. Go to your GitHub repository
2. Click on the "Settings" tab
3. Scroll down to the "Pages" section
4. Under "Source", select "Deploy from a branch"
5. Choose the branch `gh-pages` and directory `/` (root)
6. Click "Save"

### Step 3: Build and Deploy

Run the following command in your project directory:

```bash
npm run build
```

This will create a `build/` directory with the static content of your site.

### Step 4: Deploy Using Docusaurus Command

If your GitHub Pages is set up correctly, you can deploy with:

```bash
GIT_USER=<your-github-username> USE_SSH=true yarn deploy
```

Or for npm users:

```bash
GIT_USER=<your-github-username> USE_SSH=true npm run deploy
```

## Automated Deployment with GitHub Actions

For automatic deployment on every push to the main branch:

1. Create `.github/workflows/deploy.yml` with the following content:

```yaml
name: Deploy Docusaurus to GitHub Pages

on:
  push:
    branches: [ main, master ]
  workflow_dispatch:

jobs:
  deploy:
    name: Deploy to GitHub Pages
    runs-on: ubuntu-latest

    steps:
    - uses: actions/checkout@v4

    - name: Setup Node.js
      uses: actions/setup-node@v4
      with:
        node-version: '18'
        cache: 'npm'

    - name: Install dependencies
      run: npm install

    - name: Build website
      run: npm run build

    - name: Deploy to GitHub Pages
      uses: peaceiris/actions-gh-pages@v4
      with:
        github_token: ${{ secrets.GITHUB_TOKEN }}
        publish_dir: ./build
        publish_branch: gh-pages
        force_orphan: true
```

2. This workflow will automatically build and deploy your site when you push to the main branch.

## Configuration

Make sure your `docusaurus.config.js` includes proper GitHub Pages settings:

```javascript
const config = {
  // ...
  url: 'https://<your-username>.github.io',
  baseUrl: '/<your-repository-name>/',
  organizationName: '<your-username>', // Usually your GitHub org/user name
  projectName: '<your-repository-name>', // Usually your repo name
  deploymentBranch: 'gh-pages',
  // ...
};
```

## Troubleshooting

### Site not showing up
- Verify the GitHub Pages source is set to the `gh-pages` branch
- Check that the build completed without errors
- Ensure the `baseUrl` in `docusaurus.config.js` matches your repository name

### Images or assets not loading
- Confirm that all asset paths are relative
- Check that the `baseUrl` is properly configured in your Docusaurus config

### Custom domain issues
If using a custom domain:
1. Add a `CNAME` file to the `static/` directory with your domain name
2. Update DNS settings to point to GitHub Pages

## Best Practices

- Always test your site locally before deploying (`npm run start`)
- Use relative paths for all internal links
- Regularly check for broken links using Docusaurus' built-in tools
- Keep the `baseUrl` in `docusaurus.config.js` consistent with your repository name