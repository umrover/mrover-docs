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
      sidebar: [
        // SIDEBAR_ITEMS_START
        { label: "MRover Software Introduction", slug: "introduction" },
        {
          label: "General Resources",
          collapsed: true,
          items: [
            {
              label: "Best Practices",
              slug: "general-resources/best-practices",
            },
            { label: "Git", slug: "general-resources/git" },
            {
              label: "Contributing",
              slug: "general-resources/contributing-to-the-wiki",
            },
            {
              label: "IDE Configuration",
              slug: "general-resources/ide-configuration",
            },
            {
              label: "ROS & Environment",
              collapsed: true,
              items: [
                {
                  label: "1. Introduction to ROS",
                  slug: "general-resources/ros/intro-to-ros",
                },
                {
                  label: "2. Fundamentals of ROS",
                  slug: "general-resources/ros/fundamentals-of-ros",
                },
                {
                  label: "ROS Tools: rqt_bag",
                  slug: "general-resources/ros/ros-tools-rqt-bag",
                },
              ],
            },
            {
              label: "Setting up the Jetson",
              slug: "general-resources/setting-up-the-jetson",
            },
          ],
        },
        // ESW_SIDEBAR_START
        {
          label: "Setup",
          items: [
            { label: "Getting Started", slug: "setup/getting-started" },
            { label: "Installing Ubuntu", slug: "setup/installing-ubuntu" },
            { label: "Native ROS Installation", slug: "setup/native-install" },
            {
              label: "Portable ROS Installation",
              slug: "setup/portable-install",
            },
            {
              label: "VM Setup",
              collapsed: true,
              items: [
                {
                  label: "macOS VM Setup (Deprecated)",
                  slug: "general-resources/vm/macos-vm-setup",
                },
                {
                  label: "USB Passthrough for UTM",
                  slug: "general-resources/vm/usb-passthrough-utm",
                },
              ],
            },
          ],
        },
        {
          label: "ESW",
          collapsed: true,
          items: [
            { label: "Home", slug: "esw" },
            {
              label: "Getting Started",
              collapsed: true,
              items: [
                { label: "Introduction", slug: "esw/getting-started/intro" },
                { label: "STM32Cube", slug: "esw/getting-started/stm32cube" },
                {
                  label: "Starter Projects",
                  collapsed: true,
                  items: [
                    { label: "LED", slug: "esw/getting-started/starter/led" },
                    {
                      label: "Servo",
                      collapsed: true,
                      items: [
                        {
                          label: "Part 1: PWM",
                          slug: "esw/getting-started/starter/servo/part1-pwm",
                        },
                        {
                          label: "Part 2: CAN",
                          slug: "esw/getting-started/starter/servo/part2-can",
                        },
                      ],
                    },
                    {
                      label: "Temp-Humidity",
                      slug: "esw/getting-started/starter/temp-humidity",
                    },
                  ],
                },
              ],
            },
            {
              label: "Projects",
              collapsed: true,
              items: [
                { label: "Overview", slug: "esw/projects/overview26" },
                {
                  label: "Boards",
                  collapsed: true,
                  items: [
                    { label: "Overview", slug: "esw/projects/boards" },
                    { label: "ABS", slug: "esw/projects/boards/abs" },
                    { label: "BMC", slug: "esw/projects/boards/bmc" },
                    { label: "LIM", slug: "esw/projects/boards/lim" },
                    { label: "PDB", slug: "esw/projects/boards/pdlb" },
                    { label: "Science", slug: "esw/projects/boards/science" },
                  ],
                },
              ],
            },
            {
              label: "Reference",
              collapsed: true,
              items: [
                {
                  label: "Build System",
                  collapsed: true,
                  items: [
                    { label: "Overview", slug: "esw/reference/build" },
                    {
                      label: "Project Anatomy",
                      slug: "esw/reference/build/project-layout",
                    },
                    {
                      label: "Toolchain and Presets",
                      slug: "esw/reference/build/toolchain",
                    },
                    {
                      label: "Generated Libraries",
                      slug: "esw/reference/build/codegen",
                    },
                    {
                      label: "Build Script Internals",
                      slug: "esw/reference/build/build-script",
                    },
                    {
                      label: "Continuous Integration",
                      slug: "esw/reference/build/ci",
                    },
                  ],
                },
                {
                  label: "Configuration",
                  collapsed: true,
                  items: [
                    { label: "Overview", slug: "esw/reference/config" },
                    {
                      label: "Register Definitions",
                      slug: "esw/reference/config/schema",
                    },
                    {
                      label: "Device Values",
                      slug: "esw/reference/config/devices",
                    },
                    {
                      label: "CAN Configuration Interface",
                      slug: "esw/reference/config/can-interface",
                    },
                  ],
                },
                {
                  label: "Python Tools",
                  collapsed: true,
                  items: [
                    { label: "Overview", slug: "esw/reference/python" },
                    { label: "esw.can", slug: "esw/reference/python/can" },
                    {
                      label: "esw.config",
                      slug: "esw/reference/python/config",
                    },
                    {
                      label: "esw.cubemx",
                      slug: "esw/reference/python/cubemx",
                    },
                    {
                      label: "esw.stlink",
                      slug: "esw/reference/python/stlink",
                    },
                    {
                      label: "esw.visualization",
                      slug: "esw/reference/python/visualization",
                    },
                    {
                      label: "Script Reference",
                      slug: "esw/reference/python/scripts",
                    },
                  ],
                },
                {
                  label: "Maintaining Documentation",
                  slug: "esw/reference/maintaining-docs",
                },
              ],
            },
            {
              label: "Useful Information",
              collapsed: true,
              items: [
                { label: "Build Tools", slug: "esw/info/build" },
                { label: "Timers", slug: "esw/info/timers" },
                {
                  label: "Communication Protocols",
                  slug: "esw/info/communication-protocols",
                },
                { label: "Brushed Motors", slug: "esw/info/brushed" },
                { label: "Brushless Motors", slug: "esw/info/brushless" },
                { label: "Cameras", slug: "esw/info/cameras" },
                { label: "Nucleo Information", slug: "esw/info/nucleos" },
                { label: "STM32 Boot", slug: "esw/info/stm32-boot" },
              ],
            },
          ],
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
                { label: "Teleop Overview", slug: "teleop/overview" },
                { label: "Teleop Quickstart", slug: "teleop/quickstart" },
                {
                  label: "Teleop Starter Project",
                  slug: "teleop/starter-project",
                },
              ],
            },
            {
              label: "Guides",
              collapsed: true,
              items: [
                { label: "Vue Introduction", slug: "teleop/vue-introduction" },
                {
                  label: "Sample Vue Component",
                  slug: "teleop/sample-vue-component",
                },
                {
                  label: "Tailwind Introduction",
                  slug: "teleop/tailwind-introduction",
                },
                {
                  label: "Websockets Introduction",
                  slug: "teleop/websockets-introduction",
                },
                {
                  label: "SQLite Introduction",
                  slug: "teleop/sqlite-introduction",
                },
              ],
            },
            {
              label: "Teleop Organization and Tools",
              collapsed: true,
              items: [
                {
                  label: "Teleop Codebase Organization",
                  slug: "teleop/organization",
                },
                {
                  label: "GUI Style Checking",
                  slug: "teleop/gui-style-checking",
                },
                { label: "Camera Client", slug: "teleop/camera-client" },
                {
                  label: "Downloading Offline Map",
                  slug: "teleop/downloading-offline-map",
                },
                {
                  label: "WebSocket Handlers Lookup",
                  slug: "teleop/consumers-lookup",
                },
              ],
            },
            { label: "Teleop Projects", slug: "teleop/projects" },
            { label: "Feature Request", slug: "teleop/feature-request" },
            { label: "Teleop FAQ", slug: "teleop/faq" },
          ],
        },
        {
          label: "Autonomy",
          collapsed: true,
          items: [
            { label: "Autonomy Overview", slug: "autonomy/overview" },
            { label: "Autonomy Quickstart", slug: "autonomy/quickstart" },
            {
              label: "Starter Project",
              collapsed: true,
              items: [
                {
                  label: "Overview",
                  slug: "autonomy/starter-project/overview",
                },
                {
                  label: "Localization",
                  slug: "autonomy/starter-project/localization",
                },
                {
                  label: "Perception",
                  slug: "autonomy/starter-project/perception",
                },
                {
                  label: "Navigation",
                  slug: "autonomy/starter-project/navigation",
                },
                {
                  label: "Testing & Completion",
                  slug: "autonomy/starter-project/testing",
                },
              ],
            },
            {
              label: "Projects 2026-27",
              collapsed: true,
              items: [
                {
                  label: "Localization",
                  slug: "autonomy/projects-2026-27/localization",
                },
                {
                  label: "Perception",
                  slug: "autonomy/projects-2026-27/perception",
                },
                {
                  label: "Navigation",
                  slug: "autonomy/projects-2026-27/navigation",
                },
              ],
            },
            {
              label: "Localization",
              collapsed: true,
              items: [
                {
                  label: "Localization",
                  slug: "autonomy/localization/overview",
                },
                {
                  label: "Dual Antenna RTK",
                  slug: "autonomy/localization/dual-antenna-rtk",
                },
                {
                  label: "How to Calibrate the IMU",
                  slug: "autonomy/localization/imu-calibration",
                },
                {
                  label: "In Search of Globally Accurate Orientation",
                  slug: "autonomy/localization/globally-accurate-orientation",
                },
                {
                  label: "Invariant EKF",
                  slug: "autonomy/localization/invariant-ekf",
                },
                {
                  label: "Localization Data Caching State Machine",
                  slug: "autonomy/localization/data-caching-state-machine",
                },
              ],
            },
            {
              label: "Navigation",
              collapsed: true,
              items: [
                { label: "Navigation", slug: "autonomy/navigation/overview" },
                {
                  label: "Arm IK",
                  slug: "autonomy/navigation/arm-ik-overview",
                },
                {
                  label: "Path Planning",
                  slug: "autonomy/navigation/path-planning",
                },
                {
                  label: "Hybrid A*",
                  slug: "autonomy/navigation/hybrid-astar",
                },
                {
                  label: "Search Trajectory",
                  slug: "autonomy/navigation/search-trajectory",
                },
                {
                  label: "Modulation",
                  slug: "autonomy/navigation/modulation",
                },
              ],
            },
            {
              label: "Perception",
              collapsed: true,
              items: [
                { label: "Perception", slug: "autonomy/perception/overview" },
                {
                  label: "Key Detection",
                  slug: "autonomy/perception/key-detection",
                },
                {
                  label: "Light Detector",
                  slug: "autonomy/perception/light-detector",
                },
                {
                  label: "Long Range Tag Detection",
                  slug: "autonomy/perception/long-range-tag-detection",
                },
                {
                  label: "Object Detection",
                  slug: "autonomy/perception/object-detection",
                },
                {
                  label: "Object Detector Model",
                  slug: "autonomy/perception/object-detector-model",
                },
              ],
            },
            {
              label: "Resources",
              collapsed: true,
              items: [
                {
                  label: "3D Poses, Transforms, and Rotations",
                  slug: "autonomy/resources/3d-poses-transforms-rotations",
                },
                {
                  label: "Vectorization",
                  slug: "autonomy/resources/vectorization",
                },
              ],
            },
          ],
        },
        {
          label: "Drone",
          collapsed: true,
          items: [
            { label: "Drone Overview", slug: "drone/overview" },
            { label: "900x Radio", slug: "drone/900x-radio" },
            { label: "Docker Setup (Experimental)", slug: "drone/docker" },
            { label: "Flying the Drone", slug: "drone/flying" },
            {
              label: "Full Development Bringup",
              slug: "drone/development-bringup",
            },
            { label: "macOS Ubuntu VM", slug: "drone/macos-vm" },
            {
              label: "Multi-Machine ROS2 Networking",
              slug: "drone/networking",
            },
            { label: "NIX Video Capture Card", slug: "drone/nix-capture-card" },
            { label: "Resources", slug: "drone/resources" },
            { label: "Software Install", slug: "drone/software-install" },
            {
              label: "Software Starter Project 2025-26",
              slug: "drone/starter-project",
            },
            { label: "System Architecture", slug: "drone/system-architecture" },
          ],
        },
        {
          label: "Archive",
          collapsed: true,
          items: [
            { label: "2024 Projects", slug: "archive/2024-projects" },
            { label: "2025-2026 Projects", slug: "archive/2025-2026-projects" },
            {
              label: "Projects",
              collapsed: true,
              items: [
                { label: "5-DOF IK", slug: "archive/projects/5dof-ik" },
                {
                  label: "Adaptive Pure Pursuit",
                  slug: "archive/projects/adaptive-pure-pursuit",
                },
                {
                  label: "Approach Object State",
                  slug: "archive/projects/approach-object-state",
                },
                {
                  label: "Approach Target Base State",
                  slug: "archive/projects/approach-target-base-state",
                },
                { label: "Arm IK", slug: "archive/projects/arm-ik" },
                {
                  label: "Arm IK Testing Visualization",
                  slug: "archive/projects/arm-ik-testing",
                },
                {
                  label: "Arm Velocity Control",
                  slug: "archive/projects/arm-velocity-control",
                },
                { label: "Click IK", slug: "archive/projects/click-ik" },
                { label: "Cost Map", slug: "archive/projects/cost-map" },
                {
                  label: "Costmap Path Planning",
                  slug: "archive/projects/costmap-path-planning",
                },
                {
                  label: "Inward Spiraling",
                  slug: "archive/projects/inward-spiraling",
                },
                {
                  label: "Lander Auto Align",
                  slug: "archive/projects/lander-auto-align",
                },
                {
                  label: "Navigation State Machine Library",
                  slug: "archive/projects/state-machine-library",
                },
                {
                  label: "Obstacle Avoidance",
                  slug: "archive/projects/obstacle-avoidance",
                },
                {
                  label: "Path Execution",
                  slug: "archive/projects/path-execution",
                },
                {
                  label: "Path Smoothing",
                  slug: "archive/projects/path-smoothing",
                },
                {
                  label: "Pure Pursuit",
                  slug: "archive/projects/pure-pursuit",
                },
                {
                  label:
                    "Second Camera Navigation Integration (LongRangeState)",
                  slug: "archive/projects/second-camera-integration",
                },
                {
                  label: "Stuck Detector",
                  slug: "archive/projects/stuck-detector",
                },
                {
                  label: "Surface Normals Costmap",
                  slug: "archive/projects/surface-normals-costmap",
                },
                {
                  label: "URC vs. CIRC Switch",
                  slug: "archive/projects/urc-vs-circ-switch",
                },
              ],
            },
          ],
        },
        // SIDEBAR_ITEMS_END
      ],
    }),
  ],
});
