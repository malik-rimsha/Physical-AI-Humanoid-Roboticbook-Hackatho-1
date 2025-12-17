# Quickstart Guide: Module 1 – The Robotic Nervous System (ROS 2)

**Feature**: 1-ros2-robotic-nervous-system
**Date**: 2025-12-17

## Prerequisites

- Node.js (v18 or higher)
- npm or yarn package manager
- Git for version control
- A modern web browser

## Setup Instructions

### 1. Install Docusaurus

```bash
# Create a new Docusaurus project
npx create-docusaurus@latest website classic

# Navigate to the project directory
cd website

# Install additional dependencies for educational features
npm install @docusaurus/module-type-aliases @docusaurus/types
```

### 2. Configure the Site

Update `docusaurus.config.js` with the following configuration:

```javascript
// docusaurus.config.js
import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI Humanoid Robotics Course',
  tagline: 'Learn ROS 2 and humanoid robotics fundamentals',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://your-username.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub Pages, this is usually /<project-name>/
  baseUrl: '/Physical-AI-Humanoid-Roboticbook-Hackatho-1/',

  // GitHub pages deployment config
  organizationName: 'your-username', // Usually your GitHub org/user name
  projectName: 'Physical-AI-Humanoid-Roboticbook-Hackatho-1', // Usually your repo name
  deploymentBranch: 'gh-pages', // Branch that GitHub Pages will deploy from

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/facebook/docusaurus/tree/main/packages/create-docusaurus/templates/shared/',
        },
        blog: false, // Disable blog for educational content
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI Course',
        logo: {
          alt: 'Course Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Tutorial',
          },
          {
            href: 'https://github.com/your-username/Physical-AI-Humanoid-Roboticbook-Hackatho-1',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Tutorial',
                to: '/docs/intro',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/docusaurus',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/docusaurus',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/facebook/docusaurus',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI Course. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;
```

### 3. Create Sidebar Configuration

Create a `sidebars.js` file:

```javascript
// sidebars.js
/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
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
  ],
};

export default sidebars;
```

### 4. Create Educational Content Structure

Create the directory structure for the ROS 2 module:

```bash
mkdir -p docs/modules/ros2
```

### 5. Add the First Chapter

Create `docs/modules/ros2/chapter1-foundations.md`:

```markdown
---
sidebar_position: 1
title: "Chapter 1: ROS 2 Foundations"
---

# Chapter 1: ROS 2 Foundations

## What is ROS 2 and Why It Matters in Physical AI

Robot Operating System 2 (ROS 2) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robot platforms.

### Key Concepts:
- Middleware for robot communication
- Distributed system architecture
- Role in humanoid robotics

[Content continues with detailed explanations, examples, and exercises]
```

### 6. Run the Development Server

```bash
npm run start
```

This will start the development server and open your browser to the site.

## Running Tests

```bash
# Run build to ensure site compiles correctly
npm run build

# Run linting
npm run lint
```

## Deployment

To deploy to GitHub Pages:

1. Ensure your GitHub repository settings allow GitHub Pages from the `gh-pages` branch
2. Run the deployment command:

```bash
GIT_USER=<Your GitHub username> npm run deploy
```

## Next Steps

1. Add the remaining chapters (chapter2-communication.md, chapter3-urdf.md)
2. Implement exercises and assessments
3. Add code examples and diagrams
4. Test navigation and user experience