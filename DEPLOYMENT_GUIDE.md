# Deployment and Maintenance Guide

## Overview
This document provides instructions for deploying and maintaining the AI-Driven Book with RAG Chatbot system.

## Architecture Overview
The system consists of:
1. **Frontend**: Docusaurus-based static site hosted on GitHub Pages
2. **Backend**: FastAPI application for RAG functionality
3. **Vector Database**: Qdrant Cloud for semantic search
4. **Metadata Storage**: Neon Postgres for conversation history
5. **AI Service**: OpenAI API for response generation

## Prerequisites
- Node.js 18+ (for Docusaurus)
- Python 3.11+ (for FastAPI backend)
- Qdrant Cloud account
- Neon Postgres account
- OpenAI API key
- GitHub account for Pages hosting

## Backend Deployment

### 1. Environment Configuration
Create a `.env` file with the following variables:

```env
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_cloud_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_DATABASE_URL=your_neon_postgres_connection_string
JWT_SECRET_KEY=your_jwt_secret_key
```

### 2. Python Dependencies
```bash
pip install -r requirements.txt
```

### 3. Deploy Backend Service
The backend needs to be deployed to a cloud platform that supports Python/FastAPI applications, such as:
- AWS EC2 with load balancer
- Google Cloud Run
- Azure App Service
- DigitalOcean App Platform
- Railway
- Render

#### Example deployment to Render:
1. Create a `render.yaml` file:
```yaml
services:
  - type: web
    name: rag-api
    env: python
    buildCommand: pip install -r requirements.txt
    startCommand: uvicorn rag.fastapi_app.main:app --host 0.0.0.0 --port $PORT
    envVars:
      - key: OPENAI_API_KEY
        sync: false
      - key: QDRANT_URL
        sync: false
      - key: QDRANT_API_KEY
        sync: false
      - key: NEON_DATABASE_URL
        sync: false
      - key: JWT_SECRET_KEY
        sync: false
```

### 4. Initialize Content Indexing
After deploying the backend, index the book content:

```bash
python rag/loader/load_markdown.py
```

## Frontend Deployment (GitHub Pages)

### 1. Install Dependencies
```bash
cd book
npm install
```

### 2. Build the Site
```bash
npm run build
```

### 3. Deploy to GitHub Pages
#### Via GitHub Actions (Recommended):
1. Create `.github/workflows/deploy.yml`:
```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [ main ]
  workflow_dispatch:

jobs:
  deploy:
    name: Deploy to GitHub Pages
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18

      - name: Install dependencies
        run: npm install
        working-directory: book

      - name: Build website
        run: npm run build
        working-directory: book

      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./book/build
          # cname: your-domain.com  # If using custom domain
```

2. Enable GitHub Pages in your repository settings:
   - Go to Settings > Pages
   - Source: "Deploy from a branch"
   - Branch: `gh-pages`, `/ (root)`

#### Manual Deployment:
```bash
# Ensure you're in the book directory
cd book

# Build the site
npm run build

# Navigate to the build directory and create a new branch
cd build
git init
git add .
git commit -m "Deploy to GitHub Pages"
git remote add origin https://github.com/your-username/your-repo.git
git push -f origin main:gh-pages
```

## Configuration

### Backend Configuration
The backend service is configured through environment variables:

- `OPENAI_API_KEY`: API key for OpenAI services
- `QDRANT_URL`: URL for your Qdrant Cloud instance
- `QDRANT_API_KEY`: API key for Qdrant Cloud
- `NEON_DATABASE_URL`: Connection string for Neon Postgres
- `JWT_SECRET_KEY`: Secret key for JWT token generation
- `ACCESS_TOKEN_EXPIRE_MINUTES`: Token expiration time (default: 30)

### Frontend Configuration
Update `book/docusaurus.config.js` with your specific settings:

- `url`: Your production domain
- `baseUrl`: Base path for the site (for GitHub Pages, this is usually `/`)
- `organizationName` and `projectName`: Your GitHub username and repository name

## Maintenance

### Monitoring
The system includes logging capabilities in `rag/fastapi_app/logging.py`. Monitor your platform's logs for:

1. **Performance**: Look for slow response times in the logs
2. **Errors**: Monitor for API failures or database connection issues
3. **Usage**: Track API call volumes and popular content

### Content Updates
To add or update book content:

1. Edit markdown files in `book/docs/` with proper frontmatter
2. Rebuild and redeploy the frontend
3. Re-run content indexing: `python rag/loader/load_markdown.py`

### Backup Strategy
- **Qdrant**: Export and backup your vector collections periodically
- **Neon Postgres**: Use Neon's built-in backup functionality
- **Content**: Back up the markdown files in version control

### Scaling Considerations
- **Qdrant**: Scale your Qdrant Cloud tier based on vector search volume
- **Backend**: Monitor API response times and scale instances accordingly
- **Frontend**: GitHub Pages handles static assets efficiently at scale

## Troubleshooting

### Common Issues

#### 1. Chatbot Returns Generic Responses
**Symptoms**: The bot responds but doesn't reference specific content
**Solutions**:
- Verify Qdrant is properly populated with content embeddings
- Check that the query contains content that exists in your book
- Review the similarity threshold in the search

#### 2. Slow Response Times
**Symptoms**: Chat responses take longer than 5 seconds
**Solutions**:
- Check API quotas and rate limits for OpenAI and Qdrant
- Verify database connection performance
- Consider caching frequently requested queries

#### 3. GitHub Pages Not Loading
**Symptoms**: Frontend site returns 404 or blank page
**Solutions**:
- Verify the correct branch is selected in GitHub Pages settings
- Check that the base URL is correctly configured
- Ensure all assets are properly linked

### Health Checks
The backend service exposes a health check endpoint:
```
GET /health
```
This returns `{"status": "healthy"}` when the service is operational.

## Security

### API Security
- Use HTTPS for all API communications
- Implement rate limiting based on IP/client ID
- Validate and sanitize all user inputs
- Use JWT tokens for authenticated endpoints (if enabled)

### Data Security
- Store sensitive data (API keys) in environment variables, not code
- Encrypt sensitive data in transit using TLS
- Implement proper access controls for database connections

## Versioning and Updates

### Updating Dependencies
1. **Backend**: Update `requirements.txt` and redeploy
2. **Frontend**: Update `package.json` and rebuild
3. **Test**: Thoroughly test after any major updates

### API Versioning
API endpoints are versioned in the path (e.g., `/v1/chat`). When making breaking changes, introduce a new version path and maintain the old one for a transition period.