// @ts-check
import { defineConfig } from 'astro/config';
import starlight from '@astrojs/starlight';
import starlightLinksValidator from 'starlight-links-validator';
import keystatic from '@keystatic/astro';
import remarkMath from 'remark-math';
import rehypeKatex from 'rehype-katex';
import { unified } from '@astrojs/markdown-remark';
import react from '@astrojs/react';
import fs from 'fs';
const sidebarData = JSON.parse(fs.readFileSync('./src/components/sidebar.json', 'utf-8')).sidebarItems;
export default defineConfig({
  output: 'server',
  site: 'https://docs.mrover.org',
  markdown: {
    processor: unified({ remarkPlugins: [remarkMath], rehypePlugins: [rehypeKatex] }),
  },
  integrations: [
    {
      name: 'watch-sidebar',
      hooks: {
        'astro:config:setup': ({ addWatchFile }) => {
          addWatchFile(new URL('./src/components/sidebar.json', import.meta.url));
        }
      }
    },
    react({
      include: ['**/*.{ts,tsx,js,jsx}'], 
    }),
    keystatic(),
    starlight({
      plugins: [starlightLinksValidator()],
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
      components: {
        SocialIcons: './src/components/SocialIcons.astro',
      },
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
      sidebar: sidebarData,
    }),
  ],
});
