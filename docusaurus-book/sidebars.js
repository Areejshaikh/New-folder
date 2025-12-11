/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'index', // Home page
    {
      type: 'category',
      label: 'ROS 2: The Robotic Nervous System',
      collapsible: true,
      collapsed: false,
      items: [
        'ros2-nervous-system',
        'ros2-nodes',
        'ros2-topics-messages',
        'interactive-ros2-content',
        'turtlesim-exercises',
        'assessment-quiz',
        'curriculum-rag-integration',
      ],
    },
  ],
};

module.exports = sidebars;