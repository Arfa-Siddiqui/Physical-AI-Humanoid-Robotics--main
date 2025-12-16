// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive guide to ROS 2, Digital Twins, NVIDIA Isaac, and Vision-Language-Action systems',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  // For Vercel deployment, this will be automatically set
  url: process.env.VERCEL_URL ? `https://${process.env.VERCEL_URL}` : 'http://localhost:3000',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For Vercel deployment, use '/' as the base URL
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  // TODO: Replace with your actual GitHub username and repo name
  organizationName: 'yourusername', // Usually your GitHub org/user name.
  projectName: 'Physical-AI-Humanoid-Robotics', // Usually your repo name.

  onBrokenLinks: 'throw',

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
          routeBasePath: 'docs', // Serve docs at /docs
          // Disable edit links for this educational project
          editUrl: undefined,
        },
        blog: false, // Disable blog for book project
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  // Add Mermaid for diagrams
  markdown: {
    mermaid: true,
  },
  themes: ['@docusaurus/theme-mermaid'],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Social card for sharing
      image: 'img/social-card.png',
      metadata: [
        {name: 'keywords', content: 'robotics, ROS 2, humanoid robots, physical AI, NVIDIA Isaac, VLA, vision-language-action, digital twin, embodied AI'},
        {name: 'description', content: 'A comprehensive technical guide to Physical AI and Humanoid Robotics covering ROS 2, Digital Twins, NVIDIA Isaac, and Vision-Language-Action systems.'},
        {property: 'og:title', content: 'Physical AI & Humanoid Robotics - Technical Guide'},
        {property: 'og:description', content: 'Master ROS 2, Digital Twins, NVIDIA Isaac, and Vision-Language-Action Systems for humanoid robotics development.'},
        {property: 'og:type', content: 'book'},
        {name: 'twitter:card', content: 'summary_large_image'},
        {name: 'twitter:title', content: 'Physical AI & Humanoid Robotics'},
        {name: 'twitter:description', content: 'Comprehensive guide to ROS 2, NVIDIA Isaac, and Vision-Language-Action systems'},
      ],
      colorMode: {
        respectPrefersColorScheme: true,
      },
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Physical AI Logo',
          src: 'img/logo.svg',
          href: '/',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'bookSidebar',
            position: 'left',
            label: 'Book',
          },
          {
            href: 'https://github.com/yourusername/Physical-AI-Humanoid-Robotics',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Book',
            items: [
              {
                label: 'Introduction',
                to: '/docs/intro/what-is-physical-ai',
              },
              {
                label: 'ROS 2',
                to: '/docs/module-1-ros2/prerequisites',
              },
              {
                label: 'Simulation',
                to: '/docs/module-2-simulation/prerequisites',
              },
              {
                label: 'NVIDIA Isaac',
                to: '/docs/module-3-isaac/prerequisites',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'ROS 2 Docs',
                href: 'https://docs.ros.org/en/humble/',
              },
              {
                label: 'NVIDIA Isaac',
                href: 'https://developer.nvidia.com/isaac',
              },
              {
                label: 'GitHub',
                href: 'https://github.com/yourusername/Physical-AI-Humanoid-Robotics',
              },
            ],
          },
        ],
        copyright: `Licensed under MIT/CC-BY-4.0 • Copyright © ${new Date().getFullYear()}`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
        additionalLanguages: ['python', 'bash', 'yaml'],
      },
    }),
};

export default config;
