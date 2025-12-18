// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'modules/ros2/chapter1-foundations',
        'modules/ros2/chapter2-communication',
        'modules/ros2/chapter3-urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'modules/digital-twin/index',
        'modules/digital-twin/chapter-1-physics-simulation',
        'modules/digital-twin/chapter-2-environment-simulation',
        'modules/digital-twin/chapter-3-sensor-simulation',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'modules/ai-robot-brain-isaac/index',
        'modules/ai-robot-brain-isaac/perception-vslam',
        'modules/ai-robot-brain-isaac/navigation',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA) Integration',
      items: [
        'modules/vla-integration/index',
        'modules/vla-integration/vla-foundations',
        'modules/vla-integration/voice-to-action',
        'modules/vla-integration/cognitive-planning',
      ],
    },
  ],

  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    'intro',
    'hello',
    {
      type: 'category',
      label: 'Tutorial',
      items: ['tutorial-basics/create-a-document'],
    },
  ],
   */
};

export default sidebars;
