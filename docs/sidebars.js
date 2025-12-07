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
  // Manual sidebar structure for Physical AI & Humanoid Robotics Course
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Introduction to Physical AI',
      collapsed: false,
      items: [
        'intro/what-is-physical-ai',
        'intro/embodied-intelligence',
        'intro/sensor-ecosystems',
        'intro/course-roadmap',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Robot Control',
      collapsed: false,
      items: [
        {
          type: 'category',
          label: 'Chapter 1: ROS 2 Fundamentals',
          items: [
            'module-1-ros2/ch1-fundamentals/installation',
            'module-1-ros2/ch1-fundamentals/nodes-topics',
            'module-1-ros2/ch1-fundamentals/services-actions',
            'module-1-ros2/ch1-fundamentals/cli-tools',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 2: Python Integration (rclpy)',
          items: [
            'module-1-ros2/ch2-python-rclpy/first-node',
            'module-1-ros2/ch2-python-rclpy/message-types',
            'module-1-ros2/ch2-python-rclpy/timers-callbacks',
            'module-1-ros2/ch2-python-rclpy/parameters',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 3: Robot Modeling (URDF)',
          items: [
            'module-1-ros2/ch3-urdf/urdf-syntax',
            'module-1-ros2/ch3-urdf/humanoid-description',
            'module-1-ros2/ch3-urdf/rviz-visualization',
            'module-1-ros2/ch3-urdf/joint-controllers',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 4: Package Development',
          items: [
            'module-1-ros2/ch4-packages/workspace-setup',
            'module-1-ros2/ch4-packages/launch-files',
            'module-1-ros2/ch4-packages/debugging',
            'module-1-ros2/ch4-packages/assessment',
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation Environments',
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'Chapter 5: Physics Simulation (Gazebo)',
          items: [
            'module-2-simulation/ch5-gazebo/gazebo-architecture',
            'module-2-simulation/ch5-gazebo/spawning-robots',
            'module-2-simulation/ch5-gazebo/physics-parameters',
            'module-2-simulation/ch5-gazebo/sensor-plugins',
            'module-2-simulation/ch5-gazebo/ros2-integration',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 6: High-Fidelity Visualization (Unity)',
          items: [
            'module-2-simulation/ch6-unity/unity-robotics-hub',
            'module-2-simulation/ch6-unity/importing-models',
            'module-2-simulation/ch6-unity/tcp-ros2-comm',
            'module-2-simulation/ch6-unity/interactive-scenarios',
          ],
        },
        'module-2-simulation/assessment',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI-Powered Perception & Navigation',
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'Chapter 7: NVIDIA Isaac Sim',
          items: [
            'module-3-isaac/ch7-isaac-sim/installation',
            'module-3-isaac/ch7-isaac-sim/photorealistic-environments',
            'module-3-isaac/ch7-isaac-sim/synthetic-data',
            'module-3-isaac/ch7-isaac-sim/domain-randomization',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 8: Perception with Isaac ROS',
          items: [
            'module-3-isaac/ch8-perception/vslam',
            'module-3-isaac/ch8-perception/object-detection',
            'module-3-isaac/ch8-perception/depth-estimation',
            'module-3-isaac/ch8-perception/pose-estimation',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 9: Autonomous Navigation (Nav2)',
          items: [
            'module-3-isaac/ch9-navigation/map-building',
            'module-3-isaac/ch9-navigation/path-planning',
            'module-3-isaac/ch9-navigation/obstacle-avoidance',
            'module-3-isaac/ch9-navigation/behavior-trees',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 10: Reinforcement Learning',
          items: [
            'module-3-isaac/ch10-rl/policy-training',
            'module-3-isaac/ch10-rl/reward-design',
            'module-3-isaac/ch10-rl/sim-to-real',
            'module-3-isaac/ch10-rl/domain-randomization-techniques',
          ],
        },
        'module-3-isaac/assessment',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action Systems',
      collapsed: true,
      items: [
        {
          type: 'category',
          label: 'Chapter 11: Voice Recognition (Whisper)',
          items: [
            'module-4-vla/ch11-whisper/audio-capture',
            'module-4-vla/ch11-whisper/whisper-api',
            'module-4-vla/ch11-whisper/speech-to-text',
            'module-4-vla/ch11-whisper/command-parsing',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 12: Cognitive Planning (LLM)',
          items: [
            'module-4-vla/ch12-llm-planning/llm-integration',
            'module-4-vla/ch12-llm-planning/prompt-engineering',
            'module-4-vla/ch12-llm-planning/action-generation',
            'module-4-vla/ch12-llm-planning/error-handling',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 13: Humanoid Kinematics & Control',
          items: [
            'module-4-vla/ch13-humanoid-control/forward-inverse-kinematics',
            'module-4-vla/ch13-humanoid-control/bipedal-locomotion',
            'module-4-vla/ch13-humanoid-control/balance-control',
            'module-4-vla/ch13-humanoid-control/manipulation',
          ],
        },
        {
          type: 'category',
          label: 'Chapter 14: Multi-Modal Integration',
          items: [
            'module-4-vla/ch14-multimodal/vla-pipeline',
            'module-4-vla/ch14-multimodal/object-grounding',
            'module-4-vla/ch14-multimodal/task-monitoring',
            'module-4-vla/ch14-multimodal/human-robot-interaction',
          ],
        },
        'module-4-vla/assessment',
      ],
    },
  ],
};

export default sidebars;
