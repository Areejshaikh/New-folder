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
      label: 'ROS 2 Basics',
      collapsible: true,
      collapsed: false,
      items: [
        'ros2-basics/introduction-to-ros2-middleware',
        'ros2-basics/python-agents-ros2-integration',
        'ros2-basics/humanoid-robot-modeling-urdf',
      ],
    },
  ],
};

module.exports = sidebars;