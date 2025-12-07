
---
id: 1
title: Create-project-constitution
stage: constitution
date_iso: 2025-12-07
surface: agent
model: Qwen
feature: Physical AI & Humanoid Robotics Book
branch: main
user: default_user
command: /sp.constitution
labels:
  - constitution
  - project-setup
  - ai
  - robotics
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files_yaml:
  - .specify/memory/constitution.md
tests_yaml: []
---

## Prompt Text

```
/sp.constitution

Project: Physical AI & Humanoid Robotics Book with Integrated RAG Chatbot

Core principles:

Accuracy: validate all technical content via simulations, ROS2, NVIDIA Isaac, and authoritative sources

Clarity: suitable for advanced AI & robotics students

Reproducibility: all code, simulations, and demos must run as documented

Integration: book content and RAG chatbot aligned; chatbot answers based on selected text

Practicality: hands-on humanoid robot applications bridging digital AI and physical embodiment

Key standards:

Include diagrams, step-by-step explanations, and code examples

Minimum 50% peer-reviewed or official sources (ROS2, NVIDIA Isaac, Unity)

Chatbot answers traceable to book content

Writing style: Flesch-Kincaid 12–14

Plagiarism: 0% tolerance

Constraints:

Docusaurus book deployed to GitHub Pages with embedded RAG chatbot

Modules: ROS2 (Robotic Nervous System), Gazebo & Unity (Digital Twin), NVIDIA Isaac (AI-Robot Brain), Vision-Language-Action (VLA)

Capstone: autonomous humanoid robot performing voice-to-action tasks, obstacle navigation, and object manipulation

Word count: 50,000–70,000 words

Include all code, simulations, and notebooks in GitHub repo with instructions

GitHub Integration:

Repository: https://github.com/Areejshaikh/Physical_AI_-_Humanoid_Robotics.git

All code, notebooks, diagrams, and book content must be automatically pushed/updated to this repository

Maintain proper folder structure:

/book-content

/simulations

/notebooks

/RAG-chatbot

Commits must be descriptive, organized, and maintain reproducibility

Success criteria:

Deployable book & fully functional RAG chatbot

Chatbot answers ≥95% of questions correctly

Demos reproduce expected results

Peer review validates content

Fully cited and plagiarism-free

All content and code continuously synced with GitHub repo
```

## Response Text

Constitution document has been created for the Physical AI & Humanoid Robotics Book project with the following principles:

1. Accuracy: Validate all technical content via simulations, ROS2, NVIDIA Isaac, and authoritative sources
2. Clarity: Content must be suitable for advanced AI & robotics students
3. Reproducibility: All code, simulations, and demos must run as documented
4. Integration: Book content and RAG chatbot aligned; chatbot answers based on selected text
5. Practicality: Focus on hands-on humanoid robot applications bridging digital AI and physical embodiment
6. Quality Assurance: All content and code continuously synced with GitHub repo

The constitution includes key standards, constraints, and governance information, with version 1.0.0.

## Outcome

The project constitution was successfully created at `.specify/memory/constitution.md` with all provided principles, standards, and constraints included.

## Evaluation

The generated constitution accurately reflects all the specified project requirements and principles, with proper structure and governance information.