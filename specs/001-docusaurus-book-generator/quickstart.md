# Quickstart Guide: Docusaurus Book Generator

This guide explains how to use the book generator to create the Docusaurus book content.

## Prerequisites

-   Node.js and npm installed.
-   Docusaurus project set up.

## Generating the Book

1.  **Prepare the input**:
    Create a configuration file (e.g., `book-definition.json`) that defines the structure of the book, including modules and chapters.

    *Example `book-definition.json`*:
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

2.  **Run the generator**:
    Execute the generation script, passing the path to the configuration file.

    ```bash
    npm run generate-book -- --input book-definition.json
    ```
    *(Note: The exact command and script `generate-book` will be created during the implementation phase.)*

3.  **Verify the output**:
    The generated markdown files will be placed in the `docusaurus-book/docs` directory. Check that the files are created with the correct content and structure.

4.  **Run Docusaurus**:
    Start the Docusaurus development server to see the generated book.

    ```bash
    cd docusaurus-book
    npm run start
    ```
