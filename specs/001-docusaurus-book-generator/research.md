# Research for Docusaurus Book Generator

## Decision: Use Docusaurus for Book Generation

**Rationale**: The project is already a Docusaurus project, and the feature request explicitly asks for Docusaurus markdown files. Docusaurus is well-suited for creating documentation and books, with features like versioning, search, and theming.

**Alternatives considered**: None, as Docusaurus is a hard constraint from the project's nature and the user's request.

## Best Practices for Docusaurus Content

-   **File Naming**: Use kebab-case for file names (e.g., `my-doc.md`).
-   **Frontmatter**: Use frontmatter to add metadata to pages, such as titles and sidebar positions.
-   **Sidebars**: Use `sidebars.js` to define the structure of the book's table of contents.
-   **Static Assets**: Store images and other static assets in the `static` directory.
-   **Code Blocks**: Use appropriate language identifiers for syntax highlighting.
