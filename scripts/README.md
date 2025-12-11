# Scripts Directory

This directory contains utility scripts for the Physical AI & Humanoid Robotics book project.

## `generate-book.mjs`

This script generates Docusaurus markdown files for the book based on a structured configuration.

### Usage

To run the book generator:

```bash
node scripts/generate-book.mjs --input=<path_to_config.json>
```

### Configuration File Format

The input JSON file should have the following structure:

```json
{
  "title": "Physical AI & Humanoid Robotics",
  "modules": [
    {
      "title": "ROS 2: The Robotic Nervous System",
      "chapters": [
        "What is Physical AI & ROS 2",
        "ROS 2 Nodes",
        "Topics & Messages"
      ]
    },
    {
      "title": "Simulation: Gazebo & Unity",
      "chapters": [
        "What is a Digital Twin",
        "Gazebo Basics"
      ]
    }
  ]
}
```

### Requirements

- Node.js (LTS version recommended)
- Google Cloud credentials configured for access to the Gemini API
- A Docusaurus project at `docusaurus-book/`

### Output

The generated markdown files will be saved to `docusaurus-book/docs/` in a format compatible with Docusaurus.