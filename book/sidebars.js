// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: [
        'chapter-1/intro',
      ],
      link: {
        type: 'generated-index',
        description: 'Learn about AI-driven books and RAG systems.'
      }
    },
    {
      type: 'category',
      label: 'Implementation',
      items: [
        'chapter-2/rag-implementation',
        'chapter-2/content-types',
      ],
      link: {
        type: 'generated-index',
        description: 'Explore how RAG systems are implemented in educational contexts.'
      }
    }
  ],
};

module.exports = sidebars;