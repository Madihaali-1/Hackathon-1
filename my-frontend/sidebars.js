// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // Custom sidebar with only Modules section (no intro/tutorial sections)
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Modules',
      items: [
        {
          type: 'category',
          label: 'Module 1: The Robotic Nervous System (ROS 2)',
          items: [
            {
              type: 'doc',
              id: 'modules/ros2-humanoid-system/intro-to-ros2',
              label: 'Chapter 1: Introduction to ROS 2',
            },
            {
              type: 'doc',
              id: 'modules/ros2-humanoid-system/urdf-robot-structure',
              label: 'Chapter 2: URDF Robot Structure',
            },
            {
              type: 'doc',
              id: 'modules/ros2-humanoid-system/ros2-communication',
              label: 'Chapter 3: ROS 2 Communication',
            },
          ],
        },
        {
          type: 'category',
          label: 'Module 2: AI Robot Brain (Isaac ROS)',
          items: [
            'modules/ai-robot-brain-isaac/isaac-ros-vslam',
            'modules/ai-robot-brain-isaac/nav2-path-planning',
            'modules/ai-robot-brain-isaac/nvidia-isaac-sim',
            'modules/ai-robot-brain-isaac/summary',
          ],
        },
        {
          type: 'category',
          label: 'Module 3: Digital Twin (Gazebo & Unity)',
          items: [
            {
              type: 'category',
              label: 'Chapter 1: Physics Simulation',
              items: [
                'modules/digital-twin-gazebo-unity/chapter-1-physics-simulation/intro-to-gazebo',
                'modules/digital-twin-gazebo-unity/chapter-1-physics-simulation/setting-up-simulation-worlds',
                'modules/digital-twin-gazebo-unity/chapter-1-physics-simulation/physics-parameters',
                'modules/digital-twin-gazebo-unity/chapter-1-physics-simulation/humanoid-models',
                'modules/digital-twin-gazebo-unity/chapter-1-physics-simulation/running-simulations',
                'modules/digital-twin-gazebo-unity/chapter-1-physics-simulation/exercises',
              ],
            },
            {
              type: 'category',
              label: 'Chapter 2: Digital Twins & HRI',
              items: [
                'modules/digital-twin-gazebo-unity/chapter-2-digital-twins-hri/unity-digital-twins',
                'modules/digital-twin-gazebo-unity/chapter-2-digital-twins-hri/hri-principles',
                'modules/digital-twin-gazebo-unity/chapter-2-digital-twins-hri/interaction-patterns',
                'modules/digital-twin-gazebo-unity/chapter-2-digital-twins-hri/visualization-techniques',
                'modules/digital-twin-gazebo-unity/chapter-2-digital-twins-hri/exercises',
              ],
            },
            {
              type: 'category',
              label: 'Chapter 3: Sensor Simulation',
              items: [
                'modules/digital-twin-gazebo-unity/chapter-3-sensor-simulation/lidar-simulation',
                'modules/digital-twin-gazebo-unity/chapter-3-sensor-simulation/imu-simulation',
                'modules/digital-twin-gazebo-unity/chapter-3-sensor-simulation/depth-camera-simulation',
                'modules/digital-twin-gazebo-unity/chapter-3-sensor-simulation/validation-techniques',
                'modules/digital-twin-gazebo-unity/chapter-3-sensor-simulation/exercises',
              ],
            },
            'modules/digital-twin-gazebo-unity/intro',
            'modules/digital-twin-gazebo-unity/summary',
          ],
        },
        {
          type: 'category',
          label: 'Module 4: Vision-Language-Action (VLA)',
          items: [
            {
              type: 'doc',
              id: 'modules/vla-module/voice-to-action-whisper',
              label: 'Chapter 1: Voice-to-Action using OpenAI Whisper',
            },
            {
              type: 'doc',
              id: 'modules/vla-module/cognitive-planning-llms-ros2',
              label: 'Chapter 2: Cognitive Planning with LLMs for ROS 2 Actions',
            },
            {
              type: 'doc',
              id: 'modules/vla-module/capstone-autonomous-humanoid',
              label: 'Chapter 3: Capstone - Autonomous Humanoid',
            },
            'modules/vla-module/summary',
          ],
        },
      ],
    },
  ],
};

export default sidebars;