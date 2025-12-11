# Physical AI & Humanoid Robotics Book (PAHR Book)

Welcome to the Physical AI & Humanoid Robotics Book project! This repository contains a comprehensive curriculum and platform for learning about humanoid robotics, integrating ROS 2, simulation environments, AI tools, and natural language processing.

## Table of Contents
1. [Overview](#overview)
2. [Features](#features)
3. [Prerequisites](#prerequisites)
4. [Installation](#installation)
5. [Usage](#usage)
6. [Architecture](#architecture)
7. [Contributing](#contributing)
8. [License](#license)

## Overview

This project provides an educational platform for learning humanoid robotics concepts using modern tools and technologies. It includes curriculum modules on ROS 2, simulation environments (Gazebo/Unity), NVIDIA Isaac tools, and voice-command capabilities.

The curriculum is designed to bridge the gap between digital AI and embodied physical systems, focusing on practical implementation of humanoid robots.

## Features

- **Modular Curriculum Architecture**: Comprehensive curriculum divided into focused modules
- **ROS 2 Integration**: Deep insights into Robot Operating System 2 for humanoid control
- **Simulation Environments**: Integration with Gazebo and Unity for safe experimentation
- **AI Integration**: Modules on NVIDIA Isaac tools for perception and autonomy
- **Voice Command Processing**: Natural language interfaces using OpenAI Whisper
- **Progress Tracking**: System to monitor learner progress through the curriculum
- **RAG Chatbot**: Intelligent Q&A system connected to curriculum content
- **Security & Privacy**: Features to protect user data and privacy
- **Responsive Design**: Mobile-friendly educational platform

## Prerequisites

Before setting up the project, ensure you have:

- **OS**: Windows 10/11 or Linux (Ubuntu 20.04+)
- **Python**: 3.10 or higher
- **Node.js**: 16 or higher (for Docusaurus)
- **Git**: Version control system
- **Docker** (optional): For containerized deployments
- **ROS 2**: Humble Hawksbill or later for robotics examples
- **Unity Hub**: For Unity simulation environments (optional)
- **NVIDIA Isaac ROS** (optional): For advanced perception modules

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/your-username/pahr-book.git
cd pahr-book
```

### 2. Set Up Python Environment

```bash
# Create virtual environment
python -m venv venv

# Activate virtual environment
# On Windows:
venv\Scripts\activate
# On macOS/Linux:
source venv/bin/activate

# Install Python dependencies
pip install -r requirements.txt
```

### 3. Set Up Docusaurus Book

```bash
cd docusaurus-book
npm install
```

### 4. Build and Run the Book Locally

```bash
cd docusaurus-book
npm run start
```

Your book will be available at `http://localhost:3000`.

## Architecture

The PAHR Book project consists of several interconnected components:

### 1. Content Management System
- Stores and manages curriculum content
- Handles versioning and updates
- Provides search capabilities

### 2. API Layer
- FastAPI-based REST API
- Authentication and authorization
- Progress tracking endpoints

### 3. Simulation Integration
- Gazebo physics simulation
- Unity high-fidelity rendering
- Isaac Sim for synthetic data generation

### 4. AI Components
- RAG (Retrieval-Augmented Generation) chatbot
- Natural language processing for voice commands
- Machine learning models for perception tasks

### 5. Security Layer
- User authentication and authorization
- Data encryption for sensitive information
- Privacy controls and data anonymization

## Usage

### Running the Local Development Server

```bash
cd docusaurus-book
npm run start
```

### Building for Production

```bash
cd docusaurus-book
npm run build
```

### Deploying to GitHub Pages

The project supports deployment to GitHub Pages via GitHub Actions with the following configuration in `.github/workflows/deploy.yml`:

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

### Adding New Curriculum Content

To add new curriculum content, create a markdown file in the `docs/` directory with frontmatter metadata:

```markdown
---
title: "New Topic in Robotics"
description: "Learn about this new robotics concept"
sidebar_label: "New Topic"
---

# New Topic in Robotics

Content goes here...
```

## Contributing

We welcome contributions to the PAHR Book project! To contribute:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Make your changes
4. Commit your changes (`git commit -m 'Add amazing feature'`)
5. Push to the branch (`git push origin feature/amazing-feature`)
6. Open a pull request

Please ensure your code adheres to the project's style guidelines and includes appropriate tests.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- The ROS 2 community for the excellent robotics framework
- The Docusaurus team for the documentation platform
- NVIDIA Isaac team for perception and simulation tools
- All educators and researchers who inspired this curriculum