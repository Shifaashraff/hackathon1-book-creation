// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1',
      items: [
        'module-1-intro/setup',
        'module-1-intro/basics',
        'module-1-intro/exercises',
      ],
    },
    {
      type: 'category',
      label: 'Module 2',
      items: [
        'module-2-ros-basics/ros-fundamentals',
        'module-2-ros-basics/communication',
        'module-2-ros-basics/exercises',
      ],
    },
    {
      type: 'category',
      label: 'Module 3',
      items: [
        'module-3-ai-robot-brain/isaac-sim/setup',
        'module-3-ai-robot-brain/isaac-sim/photorealistic-simulation',
        'module-3-ai-robot-brain/isaac-sim/synthetic-data-generation',
        'module-3-ai-robot-brain/isaac-sim/exercises',
        'module-3-ai-robot-brain/isaac-ros/vslam-setup',
        'module-3-ai-robot-brain/isaac-ros/perception-pipelines',
        'module-3-ai-robot-brain/isaac-ros/ai-navigation',
        'module-3-ai-robot-brain/isaac-ros/exercises',
        'module-3-ai-robot-brain/nav2/path-planning',
        'module-3-ai-robot-brain/nav2/bipedal-navigation',
        'module-3-ai-robot-brain/nav2/obstacle-avoidance',
        'module-3-ai-robot-brain/nav2/exercises',
        'module-3-ai-robot-brain/quickstart',
        'module-3-ai-robot-brain/troubleshooting',
      ],
    },
    {
      type: 'category',
      label: 'Module 4',
      items: [
        'module-4-vla/intro',
        'module-4-vla/voice-to-action',
        'module-4-vla/cognitive-planning',
        'module-4-vla/task-execution',
        'module-4-vla/exercises',
        'module-4-vla/capstone-project',
        'module-4-vla/demo-guide',
      ],
    },
    {
      type: 'category',
      label: 'Assessments',
      items: [
        'assessments/index',
        'assessments/module-1-assessment',
        'assessments/module-2-assessment',
        'assessments/module-3-assessment',
        'assessments/module-4-assessment',
        'assessments/rubrics',
      ],
    },
  ],
};

module.exports = sidebars;