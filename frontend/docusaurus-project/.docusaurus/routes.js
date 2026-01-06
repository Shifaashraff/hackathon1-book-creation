import React from 'react';
import ComponentCreator from '@docusaurus/ComponentCreator';

export default [
  {
    path: '/markdown-page',
    component: ComponentCreator('/markdown-page', '3d7'),
    exact: true
  },
  {
    path: '/docs',
    component: ComponentCreator('/docs', 'aca'),
    routes: [
      {
        path: '/docs',
        component: ComponentCreator('/docs', '734'),
        routes: [
          {
            path: '/docs',
            component: ComponentCreator('/docs', '9ce'),
            routes: [
              {
                path: '/docs/assessments',
                component: ComponentCreator('/docs/assessments', 'd17'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/assessments/chapter-1',
                component: ComponentCreator('/docs/assessments/chapter-1', '257'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/assessments/module-1-assessment',
                component: ComponentCreator('/docs/assessments/module-1-assessment', 'd01'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/assessments/module-2-assessment',
                component: ComponentCreator('/docs/assessments/module-2-assessment', '2c4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/assessments/module-3-assessment',
                component: ComponentCreator('/docs/assessments/module-3-assessment', '997'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/assessments/module-4-assessment',
                component: ComponentCreator('/docs/assessments/module-4-assessment', '210'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/assessments/rubrics',
                component: ComponentCreator('/docs/assessments/rubrics', 'd52'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/intro',
                component: ComponentCreator('/docs/intro', '61d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-intro/basics',
                component: ComponentCreator('/docs/module-1-intro/basics', '18c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-intro/exercises',
                component: ComponentCreator('/docs/module-1-intro/exercises', 'd9a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-1-intro/setup',
                component: ComponentCreator('/docs/module-1-intro/setup', 'fbe'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-ros-basics/communication',
                component: ComponentCreator('/docs/module-2-ros-basics/communication', '212'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-ros-basics/exercises',
                component: ComponentCreator('/docs/module-2-ros-basics/exercises', '897'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-2-ros-basics/ros-fundamentals',
                component: ComponentCreator('/docs/module-2-ros-basics/ros-fundamentals', '707'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain',
                component: ComponentCreator('/docs/module-3-ai-robot-brain', 'f86'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/assessments',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/assessments', 'c2b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/assessments/isaac-ros-assessment',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/assessments/isaac-ros-assessment', 'e26'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/assessments/isaac-sim-assessment',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/assessments/isaac-sim-assessment', '30c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/assessments/nav2-assessment',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/assessments/nav2-assessment', 'c4b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/assets',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/assets', 'f84'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/deliverables',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/deliverables', '93b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/faq',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/faq', '123'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/intro',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/intro', '10e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/isaac-ros',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/isaac-ros', '1ed'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/isaac-ros/ai-navigation',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/isaac-ros/ai-navigation', '84b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/isaac-ros/exercises',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/isaac-ros/exercises', 'a34'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/isaac-ros/perception-pipelines',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/isaac-ros/perception-pipelines', 'aa1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/isaac-ros/vslam-setup',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/isaac-ros/vslam-setup', 'f9b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/isaac-sim',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/isaac-sim', '9bf'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/isaac-sim/exercises',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/isaac-sim/exercises', '206'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/isaac-sim/photorealistic-simulation',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/isaac-sim/photorealistic-simulation', 'b8e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/isaac-sim/setup',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/isaac-sim/setup', 'fe4'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/isaac-sim/synthetic-data-generation',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/isaac-sim/synthetic-data-generation', '5e0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/nav2',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/nav2', '1f1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/nav2/bipedal-navigation',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/nav2/bipedal-navigation', '343'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/nav2/exercises',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/nav2/exercises', 'b1e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/nav2/obstacle-avoidance',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/nav2/obstacle-avoidance', '03c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/nav2/path-planning',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/nav2/path-planning', '8f1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/quickstart',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/quickstart', 'bed'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/success_criteria_verification',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/success_criteria_verification', '382'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-3-ai-robot-brain/troubleshooting',
                component: ComponentCreator('/docs/module-3-ai-robot-brain/troubleshooting', 'b2e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla/assessments/rubrics',
                component: ComponentCreator('/docs/module-4-vla/assessments/rubrics', 'b0f'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla/assessments/vla-assessment',
                component: ComponentCreator('/docs/module-4-vla/assessments/vla-assessment', '12d'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla/autonomous-execution/capstone-project',
                component: ComponentCreator('/docs/module-4-vla/autonomous-execution/capstone-project', 'c0e'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla/autonomous-execution/exercises',
                component: ComponentCreator('/docs/module-4-vla/autonomous-execution/exercises', '521'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla/autonomous-execution/task-execution',
                component: ComponentCreator('/docs/module-4-vla/autonomous-execution/task-execution', '7cb'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla/capstone-project',
                component: ComponentCreator('/docs/module-4-vla/capstone-project', 'e0b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla/cognitive-planning',
                component: ComponentCreator('/docs/module-4-vla/cognitive-planning', '6ba'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla/cognitive-planning/exercises',
                component: ComponentCreator('/docs/module-4-vla/cognitive-planning/exercises', '02b'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla/cognitive-planning/planning-algorithms',
                component: ComponentCreator('/docs/module-4-vla/cognitive-planning/planning-algorithms', '575'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla/cognitive-planning/ros2-conversion',
                component: ComponentCreator('/docs/module-4-vla/cognitive-planning/ros2-conversion', '0ca'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla/demo-guide',
                component: ComponentCreator('/docs/module-4-vla/demo-guide', 'da1'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla/exercises',
                component: ComponentCreator('/docs/module-4-vla/exercises', '7ca'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla/intro',
                component: ComponentCreator('/docs/module-4-vla/intro', '51c'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla/quickstart',
                component: ComponentCreator('/docs/module-4-vla/quickstart', '8ba'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla/task-execution',
                component: ComponentCreator('/docs/module-4-vla/task-execution', 'e00'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla/voice-to-action',
                component: ComponentCreator('/docs/module-4-vla/voice-to-action', 'e0a'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla/voice-to-action/audio-processing',
                component: ComponentCreator('/docs/module-4-vla/voice-to-action/audio-processing', 'df3'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla/voice-to-action/exercises',
                component: ComponentCreator('/docs/module-4-vla/voice-to-action/exercises', '9b0'),
                exact: true,
                sidebar: "tutorialSidebar"
              },
              {
                path: '/docs/module-4-vla/voice-to-action/whisper-setup',
                component: ComponentCreator('/docs/module-4-vla/voice-to-action/whisper-setup', '928'),
                exact: true,
                sidebar: "tutorialSidebar"
              }
            ]
          }
        ]
      }
    ]
  },
  {
    path: '/',
    component: ComponentCreator('/', '2e1'),
    exact: true
  },
  {
    path: '*',
    component: ComponentCreator('*'),
  },
];
