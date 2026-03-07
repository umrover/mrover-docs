# MRover Docs

Documentation site for MRover Software, built with [Astro Starlight](https://starlight.astro.build/).

## Local Development

```bash
npm install
npm run dev
```

## Adding a Page

1. Create a `.md` file under `src/content/docs/` in the appropriate section folder.

2. Add a `title` in the frontmatter:

```md
---
title: "Your Page Title"
---
```

3. Add a sidebar entry in `astro.config.mjs` under the relevant section:

```js
{ label: 'Your Page Title', slug: 'section/your-filename' },
```

The `slug` matches the file path under `src/content/docs/` without the `.md` extension.

## Modifying a Page

Edit the `.md` file directly. If renaming or moving, update the `slug` in `astro.config.mjs`.

## Deleting a Page

Delete the `.md` file and remove its entry from the `sidebar` array in `astro.config.mjs`.
