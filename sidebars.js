// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Physical AI & Humanoid Robotics Book Structure
 *
 * This sidebar configuration defines the complete book organization:
 * - Introduction: Physical AI concepts
 * - Foundations: Sensors, actuators, digital twins
 * - Module 1: ROS 2 fundamentals
 * - Module 2: Simulation with Gazebo & Unity
 * - Module 3: NVIDIA Isaac platform
 * - Module 4: Vision-Language-Action systems
 * - Capstone: Full system integration
 *
 * @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  bookSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      collapsed: false,
      items: [
        'intro/what-is-physical-ai',
        'intro/embodied-intelligence',
        'intro/digital-vs-physical-ai',
        'intro/robotics-landscape',
      ],
    },
    {
      type: 'category',
      label: 'Foundations',
      collapsed: false,
      items: [
        'foundations/sensors',
        'foundations/actuators',
        'foundations/digital-twin',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      collapsed: true,
      items: [
        'module-1-ros2/prerequisites',
        'module-1-ros2/ros2-architecture',
        'module-1-ros2/python-controllers',
        'module-1-ros2/urdf-humanoids',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Simulation)',
      collapsed: true,
      items: [
        'module-2-simulation/prerequisites',
        'module-2-simulation/physics-fundamentals',
        'module-2-simulation/gazebo-simulation',
        'module-2-simulation/unity-visualization',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      collapsed: true,
      items: [
        'module-3-isaac/prerequisites',
        'module-3-isaac/isaac-sim',
        'module-3-isaac/isaac-ros',
        'module-3-isaac/nav2-planning',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action Systems',
      collapsed: true,
      items: [
        'module-4-vla/prerequisites',
        'module-4-vla/voice-to-action',
        'module-4-vla/llm-planning',
        'module-4-vla/capstone-project',
      ],
    },
    {
      type: 'category',
      label: 'Capstone',
      collapsed: true,
      items: [
        'capstone/architecture',
        'capstone/integration',
        'capstone/next-steps',
      ],
    },
  ],
};

export default sidebars;
