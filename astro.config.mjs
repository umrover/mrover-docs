// @ts-check
import { defineConfig } from 'astro/config';
import starlight from '@astrojs/starlight';
import remarkMath from 'remark-math';
import rehypeKatex from 'rehype-katex';

export default defineConfig({
  site: 'https://docs.mrover.org',
  markdown: {
    remarkPlugins: [remarkMath],
    rehypePlugins: [rehypeKatex],
  },
  integrations: [
    starlight({
      expressiveCode: {
        frames: false,
      },
      title: 'MRover Docs',
      favicon: '/favicon.ico',
      social: [
        { icon: 'github', label: 'GitHub', href: 'https://github.com/umrover/mrover-ros2' },
      ],
      editLink: {
        baseUrl: 'https://github.com/umrover/mrover-docs/edit/main/',
      },
      customCss: [
        './src/styles/custom.css',
      ],
      head: [
        {
          tag: 'link',
          attrs: {
            rel: 'stylesheet',
            href: 'https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/katex.min.css',
            crossorigin: 'anonymous',
          },
        },
      ],
      sidebar: [
        { label: 'Introduction', slug: 'introduction' },
        {
          label: 'ROS and the Development Environment',
          collapsed: true,
          items: [
            { label: '1. Introduction to ROS', slug: 'getting-started/intro-to-ros' },
            { label: '2. Install ROS', slug: 'getting-started/install-ros' },
            { label: '3. IDE Configuration', slug: 'getting-started/ide-configuration' },
            { label: '4. Fundamentals of ROS', slug: 'getting-started/fundamentals-of-ros' },
            {
              label: 'Platform-Specific Setup',
              collapsed: true,
              items: [
                { label: 'Install ROS on macOS', slug: 'getting-started/install-ros-macos' },
                { label: 'macOS VM Setup', slug: 'getting-started/macos-vm-setup' },
                { label: 'Setting up the Jetson', slug: 'getting-started/setting-up-the-jetson' },
                { label: 'USB Passthrough for UTM', slug: 'getting-started/usb-passthrough-utm' },
              ],
            },
          ],
        },
        {
          label: 'Projects',
          collapsed: true,
          items: [
            { label: '2025-2026 Projects', slug: 'projects/2025-2026-projects' },
            { label: '2024 Projects', slug: 'projects/2024-projects' },
          ],
        },
        {
          label: 'General Tutorials and Resources',
          collapsed: true,
          items: [
            { label: 'Git', slug: 'general/git' },
            { label: 'Best Practices', slug: 'general/best-practices' },
            { label: 'Code Style', slug: 'general/code-style' },
            { label: 'Contributing to the Wiki', slug: 'general/contributing-to-the-wiki' },
            { label: 'ROS Tools: rqt_bag', slug: 'general/ros-tools-rqt-bag' },
          ],
        },
        {
          label: 'Dev-Ops',
          collapsed: true,
          items: [
            { label: 'Updating CI', slug: 'devops/updating-ci' },
            { label: 'URC vs. CIRC Switch', slug: 'devops/urc-vs-circ-switch' },
          ],
        },
        {
          label: 'ESW',
          collapsed: true,
          items: [
            { label: 'ESW Overview', slug: 'esw/overview' },
            { label: 'ESW Starter Project', slug: 'esw/starter-project' },
            { label: 'Electrical System Overview', slug: 'esw/electrical-system-overview' },
            { label: 'Science', slug: 'esw/science' },
            { label: 'Brushless Motors', slug: 'esw/brushless-motors' },
            { label: 'Brushed Motors', slug: 'esw/brushed-motors' },
            { label: 'Brushless ROS', slug: 'esw/brushless-ros' },
            { label: 'Brushed Motors Planning', slug: 'esw/brushed-motors-planning' },
            { label: 'Brushed Controller ROS', slug: 'esw/brushed-controller-ros' },
            { label: 'Brushed Controller STM', slug: 'esw/brushed-controller-stm' },
            { label: 'Raman', slug: 'esw/raman' },
            { label: 'STM32 Startup Boot Options', slug: 'esw/stm32-startup-boot-options' },
            { label: 'PDB STM', slug: 'esw/pdb-stm' },
            { label: 'Science Board STM', slug: 'esw/science-board-stm' },
            { label: 'CANalyzer', slug: 'esw/canalyzer' },
            { label: 'Camera Streaming', slug: 'esw/camera-streaming' },
            { label: 'Directional Antenna', slug: 'esw/directional-antenna' },
          ],
        },
        {
          label: 'Teleop',
          collapsed: true,
          items: [
            { label: 'Teleop Overview', slug: 'teleop/overview' },
            { label: 'Teleop Quickstart', slug: 'teleop/quickstart' },
            { label: 'Teleop FAQ', slug: 'teleop/faq' },
            { label: 'GUI Style Checking', slug: 'teleop/gui-style-checking' },
            { label: 'Teleop Starter Project', slug: 'teleop/starter-project' },
            { label: 'Sample Vue Component', slug: 'teleop/sample-vue-component' },
            { label: 'Downloading Offline Map', slug: 'teleop/downloading-offline-map' },
            {
              label: 'Framework Introductions',
              collapsed: true,
              items: [
                { label: 'Vue Introduction', slug: 'teleop/vue-introduction' },
                { label: 'Tailwind Introduction', slug: 'teleop/tailwind-introduction' },
                { label: 'Bootstrap Introduction', slug: 'teleop/bootstrap-introduction' },
              ],
            },
            { label: 'Convert to Django', slug: 'teleop/convert-to-django' },
            { label: 'Teleop Consumers Lookup', slug: 'teleop/consumers-lookup' },
          ],
        },
        {
          label: 'Autonomy',
          collapsed: true,
          items: [
            { label: 'Autonomy Overview', slug: 'autonomy/overview' },
            { label: 'Autonomy Quickstart', slug: 'autonomy/quickstart' },
            { label: '3D Poses, Transforms, and Rotations', slug: 'autonomy/3d-poses-transforms-rotations' },
            { label: 'Vectorization', slug: 'autonomy/vectorization' },
            { label: 'Autonomy Starter Project', slug: 'autonomy/starter-project' },
            { label: 'Autonomy Starter Project - Perception', slug: 'autonomy/starter-project-perception' },
            { label: 'Drone', slug: 'autonomy/drone' },
          ],
        },
        {
          label: 'Navigation',
          collapsed: true,
          items: [
            { label: 'Navigation', slug: 'navigation/overview' },
            { label: 'State Machine Library', slug: 'navigation/state-machine-library' },
            {
              label: 'Path Planning',
              collapsed: true,
              items: [
                { label: 'Path Execution', slug: 'navigation/path-execution' },
                { label: 'Path Smoothing', slug: 'navigation/path-smoothing' },
                { label: 'Pure Pursuit', slug: 'navigation/pure-pursuit' },
                { label: 'Adaptive Pure Pursuit', slug: 'navigation/adaptive-pure-pursuit' },
                { label: 'Cost Map', slug: 'navigation/cost-map' },
                { label: 'Surface Normals Costmap', slug: 'navigation/surface-normals-costmap' },
                { label: 'Costmap Path Planning', slug: 'navigation/costmap-path-planning' },
              ],
            },
            {
              label: 'States',
              collapsed: true,
              items: [
                { label: 'Approach Target Base State', slug: 'navigation/approach-target-base-state' },
                { label: 'Approach Object State', slug: 'navigation/approach-object-state' },
                { label: 'Lander Auto Align', slug: 'navigation/lander-auto-align' },
                { label: 'Inward Spiraling', slug: 'navigation/inward-spiraling' },
                { label: 'Obstacle Avoidance', slug: 'navigation/obstacle-avoidance' },
                { label: 'Second Camera Integration', slug: 'navigation/second-camera-integration' },
                { label: 'Stuck Detector', slug: 'navigation/stuck-detector' },
              ],
            },
            {
              label: 'Arm Control',
              collapsed: true,
              items: [
                { label: 'Arm Velocity Control', slug: 'navigation/arm-velocity-control' },
                { label: 'Arm IK', slug: 'navigation/arm-ik' },
                { label: 'Arm IK Testing', slug: 'navigation/arm-ik-testing' },
                { label: '5-DOF IK', slug: 'navigation/5dof-ik' },
                { label: 'Click IK', slug: 'navigation/click-ik' },
              ],
            },
          ],
        },
        {
          label: 'Perception',
          collapsed: true,
          items: [
            { label: 'Perception', slug: 'perception/overview' },
            { label: 'Object Detection', slug: 'perception/object-detection' },
            { label: 'Object Detector Model', slug: 'perception/object-detector-model' },
            { label: 'Key Detection', slug: 'perception/key-detection' },
            { label: 'Long Range Tag Detection', slug: 'perception/long-range-tag-detection' },
            { label: 'Light Detector', slug: 'perception/light-detector' },
          ],
        },
        {
          label: 'Localization',
          collapsed: true,
          items: [
            { label: 'Localization', slug: 'localization/overview' },
            { label: 'How to Calibrate the IMU', slug: 'localization/imu-calibration' },
            { label: 'Invariant EKF', slug: 'localization/invariant-ekf' },
            { label: 'Data Caching State Machine', slug: 'localization/data-caching-state-machine' },
            { label: 'Globally Accurate Orientation', slug: 'localization/globally-accurate-orientation' },
            { label: 'Dual Antenna RTK', slug: 'localization/dual-antenna-rtk' },
          ],
        },
      ],
    }),
  ],
});
