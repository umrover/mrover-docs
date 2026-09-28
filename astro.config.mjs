// @ts-check
import { defineConfig } from "astro/config";
import starlight from "@astrojs/starlight";
import starlightLinksValidator from "starlight-links-validator";
import remarkMath from "remark-math";
import rehypeKatex from "rehype-katex";
import { unified } from "@astrojs/markdown-remark";

export default defineConfig({
  site: "https://docs.mrover.org",
  markdown: {
    processor: unified({
      remarkPlugins: [remarkMath],
      rehypePlugins: [rehypeKatex],
    }),
  },
  integrations: [
    starlight({
      plugins: [starlightLinksValidator()],
      expressiveCode: {
        frames: {
          showCopyToClipboardButton: true,
        },
      },
      title: "MRover Docs",
      favicon: "/favicon.ico",
      social: [
        {
          icon: "github",
          label: "GitHub",
          href: "https://github.com/umrover/mrover-ros2",
        },
      ],
      editLink: {
        baseUrl: "https://github.com/umrover/mrover-docs/edit/main/",
      },
      routeMiddleware: "./src/routeData.ts",
      customCss: ["./src/styles/custom.css"],
      components: {
        SocialIcons: "./src/components/SocialIcons.astro",
      },
      head: [
        {
          tag: "link",
          attrs: {
            rel: "stylesheet",
            href: "https://cdn.jsdelivr.net/npm/katex@0.16.11/dist/katex.min.css",
            crossorigin: "anonymous",
          },
        },
      ],
      // each top-level group is a section; src/routeData.ts shows only the current one
      sidebar: [
        {
          label: "Software",
          items: [
        // SIDEBAR_ITEMS_START
        { label: "Home", slug: "software" },
        { label: "MRover Software Introduction", slug: "software/introduction" },
        {
          label: "General Resources",
          collapsed: true,
          items: [
            {
              label: "Best Practices",
              slug: "software/general-resources/best-practices",
            },
            { label: "Git", slug: "software/general-resources/git" },
            {
              label: "Contributing",
              slug: "software/general-resources/contributing-to-the-wiki",
            },
            {
              label: "IDE Configuration",
              slug: "software/general-resources/ide-configuration",
            },
            {
              label: "ROS & Environment",
              collapsed: true,
              items: [
                {
                  label: "1. Introduction to ROS",
                  slug: "software/general-resources/ros/intro-to-ros",
                },
                {
                  label: "2. Fundamentals of ROS",
                  slug: "software/general-resources/ros/fundamentals-of-ros",
                },
                {
                  label: "ROS Tools: rqt_bag",
                  slug: "software/general-resources/ros/ros-tools-rqt-bag",
                },
              ],
            },
            {
              label: "Setting up the Jetson",
              slug: "software/general-resources/setting-up-the-jetson",
            },
          ],
        },
        {
          label: "Setup",
          items: [
            { label: "Getting Started", slug: "software/setup/getting-started" },
            { label: "Installing Ubuntu", slug: "software/setup/installing-ubuntu" },
            { label: "Native ROS Installation", slug: "software/setup/native-install" },
            {
              label: "Portable ROS Installation",
              slug: "software/setup/portable-install",
            },
            {
              label: "VM Setup",
              collapsed: true,
              items: [
                {
                  label: "macOS VM Setup (Deprecated)",
                  slug: "software/general-resources/vm/macos-vm-setup",
                },
                {
                  label: "USB Passthrough for UTM",
                  slug: "software/general-resources/vm/usb-passthrough-utm",
                },
              ],
            },
          ],
        },
        // ESW_SIDEBAR_START
        {
          label: 'ESW',
          collapsed: true,
          items: [
            { label: 'Home', slug: 'software/esw' },
            {
              label: 'Getting Started',
              collapsed: true,
              items: [
                { label: 'Introduction', slug: 'software/esw/getting-started/intro' },
                { label: 'STM32Cube', slug: 'software/esw/getting-started/stm32cube' },
                {
                  label: 'Starter Projects',
                  collapsed: true,
                  items: [
                    { label: 'LED', slug: 'software/esw/getting-started/starter/led' },
                    {
                      label: 'Servo',
                      collapsed: true,
                      items: [
                        { label: 'Part 1: PWM', slug: 'software/esw/getting-started/starter/servo/part1-pwm' },
                        { label: 'Part 2: CAN', slug: 'software/esw/getting-started/starter/servo/part2-can' }
                      ]
                    },
                    { label: 'Temp-Humidity', slug: 'software/esw/getting-started/starter/temp-humidity' }
                  ]
                }
              ]
            },
            {
              label: 'Projects',
              collapsed: true,
              items: [
                { label: 'Overview', slug: 'software/esw/projects/overview26' },
                {
                  label: 'Boards',
                  collapsed: true,
                  items: [
                    { label: 'Overview', slug: 'software/esw/projects/boards' },
                    { label: 'ABS', slug: 'software/esw/projects/boards/abs' },
                    { label: 'BMC', slug: 'software/esw/projects/boards/bmc' },
                    { label: 'LIM', slug: 'software/esw/projects/boards/lim' },
                    { label: 'PDB', slug: 'software/esw/projects/boards/pdlb' },
                    { label: 'Science', slug: 'software/esw/projects/boards/science' }
                  ]
                }
              ]
            },
            {
              label: 'Reference',
              collapsed: true,
              items: [
                {
                  label: 'Build System',
                  collapsed: true,
                  items: [
                    { label: 'Overview', slug: 'software/esw/reference/build' },
                    { label: 'Project Anatomy', slug: 'software/esw/reference/build/project-layout' },
                    { label: 'Toolchain and Presets', slug: 'software/esw/reference/build/toolchain' },
                    { label: 'Generated Libraries', slug: 'software/esw/reference/build/codegen' },
                    { label: 'Build Script Internals', slug: 'software/esw/reference/build/build-script' },
                    { label: 'Continuous Integration', slug: 'software/esw/reference/build/ci' }
                  ]
                },
                {
                  label: 'Configuration',
                  collapsed: true,
                  items: [
                    { label: 'Overview', slug: 'software/esw/reference/config' },
                    { label: 'Register Definitions', slug: 'software/esw/reference/config/schema' },
                    { label: 'Device Values', slug: 'software/esw/reference/config/devices' },
                    { label: 'CAN Configuration Interface', slug: 'software/esw/reference/config/can-interface' }
                  ]
                },
                {
                  label: 'Python Tools',
                  collapsed: true,
                  items: [
                    { label: 'Overview', slug: 'software/esw/reference/python' },
                    { label: 'esw.can', slug: 'software/esw/reference/python/can' },
                    { label: 'esw.config', slug: 'software/esw/reference/python/config' },
                    { label: 'esw.cubemx', slug: 'software/esw/reference/python/cubemx' },
                    { label: 'esw.stlink', slug: 'software/esw/reference/python/stlink' },
                    { label: 'esw.visualization', slug: 'software/esw/reference/python/visualization' },
                    { label: 'Script Reference', slug: 'software/esw/reference/python/scripts' }
                  ]
                },
                { label: 'Maintaining Documentation', slug: 'software/esw/reference/maintaining-docs' }
              ]
            },
            {
              label: 'Useful Information',
              collapsed: true,
              items: [
                { label: 'Build Tools', slug: 'software/esw/info/build' },
                { label: 'Timers', slug: 'software/esw/info/timers' },
                { label: 'Communication Protocols', slug: 'software/esw/info/communication-protocols' },
                { label: 'Brushed Motors', slug: 'software/esw/info/brushed' },
                { label: 'Brushless Motors', slug: 'software/esw/info/brushless' },
                { label: 'Cameras', slug: 'software/esw/info/cameras' },
                { label: 'Nucleo Information', slug: 'software/esw/info/nucleos' },
                { label: 'STM32 Boot', slug: 'software/esw/info/stm32-boot' }
              ]
            }
          ]
        },
        // ESW_SIDEBAR_END
        {
          label: "Teleop",
          collapsed: true,
          items: [
            {
              label: "Getting Started",
              collapsed: true,
              items: [
                { label: "Teleop Overview", slug: "software/teleop/overview" },
                { label: "Teleop Quickstart", slug: "software/teleop/quickstart" },
                {
                  label: "Teleop Starter Project",
                  slug: "software/teleop/starter-project",
                },
              ],
            },
            {
              label: "Guides",
              collapsed: true,
              items: [
                { label: "Vue Introduction", slug: "software/teleop/vue-introduction" },
                {
                  label: "Sample Vue Component",
                  slug: "software/teleop/sample-vue-component",
                },
                {
                  label: "Tailwind Introduction",
                  slug: "software/teleop/tailwind-introduction",
                },
                {
                  label: "Websockets Introduction",
                  slug: "software/teleop/websockets-introduction",
                },
                {
                  label: "SQLite Introduction",
                  slug: "software/teleop/sqlite-introduction",
                },
              ],
            },
            {
              label: "Teleop Organization and Tools",
              collapsed: true,
              items: [
                {
                  label: "Teleop Codebase Organization",
                  slug: "software/teleop/organization",
                },
                {
                  label: "GUI Style Checking",
                  slug: "software/teleop/gui-style-checking",
                },
                { label: "Camera Client", slug: "software/teleop/camera-client" },
                {
                  label: "Downloading Offline Map",
                  slug: "software/teleop/downloading-offline-map",
                },
                {
                  label: "WebSocket Handlers Lookup",
                  slug: "software/teleop/consumers-lookup",
                },
              ],
            },
            { label: "Teleop Projects", slug: "software/teleop/projects" },
            { label: "Feature Request", slug: "software/teleop/feature-request" },
            { label: "Teleop FAQ", slug: "software/teleop/faq" },
          ],
        },
        {
          label: "Autonomy",
          collapsed: true,
          items: [
            { label: "Autonomy Overview", slug: "software/autonomy/overview" },
            { label: "Autonomy Quickstart", slug: "software/autonomy/quickstart" },
            {
              label: "Starter Project",
              collapsed: true,
              items: [
                {
                  label: "Overview",
                  slug: "software/autonomy/starter-project/overview",
                },
                {
                  label: "Localization",
                  slug: "software/autonomy/starter-project/localization",
                },
                {
                  label: "Perception",
                  slug: "software/autonomy/starter-project/perception",
                },
                {
                  label: "Navigation",
                  slug: "software/autonomy/starter-project/navigation",
                },
                {
                  label: "Testing & Completion",
                  slug: "software/autonomy/starter-project/testing",
                },
              ],
            },
            {
              label: "Projects 2026-27",
              collapsed: true,
              items: [
                {
                  label: "Localization",
                  slug: "software/autonomy/projects-2026-27/localization",
                },
                {
                  label: "Perception",
                  slug: "software/autonomy/projects-2026-27/perception",
                },
                {
                  label: "Navigation",
                  slug: "software/autonomy/projects-2026-27/navigation",
                },
              ],
            },
            {
              label: "Localization",
              collapsed: true,
              items: [
                {
                  label: "Localization",
                  slug: "software/autonomy/localization/overview",
                },
                {
                  label: "Dual Antenna RTK",
                  slug: "software/autonomy/localization/dual-antenna-rtk",
                },
                {
                  label: "How to Calibrate the IMU",
                  slug: "software/autonomy/localization/imu-calibration",
                },
                {
                  label: "In Search of Globally Accurate Orientation",
                  slug: "software/autonomy/localization/globally-accurate-orientation",
                },
                {
                  label: "Invariant EKF",
                  slug: "software/autonomy/localization/invariant-ekf",
                },
                {
                  label: "Localization Data Caching State Machine",
                  slug: "software/autonomy/localization/data-caching-state-machine",
                },
              ],
            },
            {
              label: "Navigation",
              collapsed: true,
              items: [
                { label: "Navigation", slug: "software/autonomy/navigation/overview" },
                {
                  label: "Arm IK",
                  slug: "software/autonomy/navigation/arm-ik-overview",
                },
                {
                  label: "Path Planning",
                  slug: "software/autonomy/navigation/path-planning",
                },
                {
                  label: "Hybrid A*",
                  slug: "software/autonomy/navigation/hybrid-astar",
                },
                {
                  label: "Search Trajectory",
                  slug: "software/autonomy/navigation/search-trajectory",
                },
                {
                  label: "Modulation",
                  slug: "software/autonomy/navigation/modulation",
                },
              ],
            },
            {
              label: "Perception",
              collapsed: true,
              items: [
                { label: "Perception", slug: "software/autonomy/perception/overview" },
                {
                  label: "Key Detection",
                  slug: "software/autonomy/perception/key-detection",
                },
                {
                  label: "Light Detector",
                  slug: "software/autonomy/perception/light-detector",
                },
                {
                  label: "Long Range Tag Detection",
                  slug: "software/autonomy/perception/long-range-tag-detection",
                },
                {
                  label: "Object Detection",
                  slug: "software/autonomy/perception/object-detection",
                },
                {
                  label: "Object Detector Model",
                  slug: "software/autonomy/perception/object-detector-model",
                },
              ],
            },
            {
              label: "Resources",
              collapsed: true,
              items: [
                {
                  label: "3D Poses, Transforms, and Rotations",
                  slug: "software/autonomy/resources/3d-poses-transforms-rotations",
                },
                {
                  label: "Vectorization",
                  slug: "software/autonomy/resources/vectorization",
                },
              ],
            },
          ],
        },
        {
          label: "Drone",
          collapsed: true,
          items: [
            { label: "Drone Overview", slug: "software/drone/overview" },
            { label: "900x Radio", slug: "software/drone/900x-radio" },
            { label: "Docker Setup (Experimental)", slug: "software/drone/docker" },
            { label: "Flying the Drone", slug: "software/drone/flying" },
            {
              label: "Full Development Bringup",
              slug: "software/drone/development-bringup",
            },
            { label: "macOS Ubuntu VM", slug: "software/drone/macos-vm" },
            {
              label: "Multi-Machine ROS2 Networking",
              slug: "software/drone/networking",
            },
            { label: "NIX Video Capture Card", slug: "software/drone/nix-capture-card" },
            { label: "Resources", slug: "software/drone/resources" },
            { label: "Software Install", slug: "software/drone/software-install" },
            {
              label: "Software Starter Project 2026-27",
              slug: "software/drone/starter-project",
            },
            { label: "System Architecture", slug: "software/drone/system-architecture" },
          ],
        },
        {
          label: "Archive",
          collapsed: true,
          items: [
            { label: "2024 Projects", slug: "software/archive/2024-projects" },
            { label: "2025-2026 Projects", slug: "software/archive/2025-2026-projects" },
            {
              label: "Projects",
              collapsed: true,
              items: [
                { label: "5-DOF IK", slug: "software/archive/projects/5dof-ik" },
                {
                  label: "Adaptive Pure Pursuit",
                  slug: "software/archive/projects/adaptive-pure-pursuit",
                },
                {
                  label: "Approach Object State",
                  slug: "software/archive/projects/approach-object-state",
                },
                {
                  label: "Approach Target Base State",
                  slug: "software/archive/projects/approach-target-base-state",
                },
                { label: "Arm IK", slug: "software/archive/projects/arm-ik" },
                {
                  label: "Arm IK Testing Visualization",
                  slug: "software/archive/projects/arm-ik-testing",
                },
                {
                  label: "Arm Velocity Control",
                  slug: "software/archive/projects/arm-velocity-control",
                },
                { label: "Click IK", slug: "software/archive/projects/click-ik" },
                { label: "Cost Map", slug: "software/archive/projects/cost-map" },
                {
                  label: "Costmap Path Planning",
                  slug: "software/archive/projects/costmap-path-planning",
                },
                {
                  label: "Inward Spiraling",
                  slug: "software/archive/projects/inward-spiraling",
                },
                {
                  label: "Lander Auto Align",
                  slug: "software/archive/projects/lander-auto-align",
                },
                {
                  label: "Navigation State Machine Library",
                  slug: "software/archive/projects/state-machine-library",
                },
                {
                  label: "Obstacle Avoidance",
                  slug: "software/archive/projects/obstacle-avoidance",
                },
                {
                  label: "Path Execution",
                  slug: "software/archive/projects/path-execution",
                },
                {
                  label: "Path Smoothing",
                  slug: "software/archive/projects/path-smoothing",
                },
                {
                  label: "Pure Pursuit",
                  slug: "software/archive/projects/pure-pursuit",
                },
                {
                  label:
                    "Second Camera Navigation Integration (LongRangeState)",
                  slug: "software/archive/projects/second-camera-integration",
                },
                {
                  label: "Stuck Detector",
                  slug: "software/archive/projects/stuck-detector",
                },
                {
                  label: "Surface Normals Costmap",
                  slug: "software/archive/projects/surface-normals-costmap",
                },
                {
                  label: "URC vs. CIRC Switch",
                  slug: "software/archive/projects/urc-vs-circ-switch",
                },
              ],
            },
          ],
        },
        // SIDEBAR_ITEMS_END
          ],
        },
        { label: "Mechanical", items: [{ label: "Home", slug: "mechanical" }] },
        { label: "Electrical", items: [{ label: "Home", slug: "electrical" }] },
        { label: "Business", items: [{ label: "Home", slug: "business" }] },
      ],
    }),
  ],
});
