// @ts-check
import { defineConfig } from "astro/config";
import starlight from "@astrojs/starlight";
import starlightLinksValidator from "starlight-links-validator";
import remarkMath from "remark-math";
import rehypeKatex from "rehype-katex";
import { unified } from "@astrojs/markdown-remark";
import software from "./src/sidebars/software.mjs";
import mechanical from "./src/sidebars/mechanical.mjs";
import electrical from "./src/sidebars/electrical.mjs";
import science from "./src/sidebars/science.mjs";

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
        SiteTitle: "./src/components/SiteTitle.astro",
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
      // one file per section in src/sidebars/; src/routeData.ts shows only the current one
      sidebar: [
        { label: "Software", items: software },
        { label: "Mechanical", items: mechanical },
        { label: "Electrical", items: electrical },
        { label: "Science", items: science },
      ],
    }),
  ],
});
