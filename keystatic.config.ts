import { config, collection, fields, singleton } from '@keystatic/core';
// Define bounded nested schema (5 levels deep)
const level5Items = fields.array(
  fields.object({
    label: fields.text({ label: 'Label' }),
    slug: fields.text({ label: 'Slug' }),
  }),
  { label: 'Items', itemLabel: props => props.fields.label.value }
);
const level4Items = fields.array(
  fields.object({
    label: fields.text({ label: 'Label' }),
    slug: fields.text({ label: 'Slug' }),
    collapsed: fields.checkbox({ label: 'Collapsed', defaultValue: false }),
    items: level5Items,
  }),
  { label: 'Items', itemLabel: props => props.fields.label.value }
);
const level3Items = fields.array(
  fields.object({
    label: fields.text({ label: 'Label' }),
    slug: fields.text({ label: 'Slug' }),
    collapsed: fields.checkbox({ label: 'Collapsed', defaultValue: false }),
    items: level4Items,
  }),
  { label: 'Items', itemLabel: props => props.fields.label.value }
);
const level2Items = fields.array(
  fields.object({
    label: fields.text({ label: 'Label' }),
    slug: fields.text({ label: 'Slug' }),
    collapsed: fields.checkbox({ label: 'Collapsed', defaultValue: false }),
    items: level3Items,
  }),
  { label: 'Items', itemLabel: props => props.fields.label.value }
);
export default config({
  storage: { kind: 'local' },
//   storage: { kind: 'github', repo: 'mrover/mrover-docs' },
  collections: {
    docs: collection({
      label: 'Documentation',
      path: 'src/content/docs/**',
      slugField: 'title',
      format: { contentField: 'content' },
      schema: {
        title: fields.slug({ name: { label: 'Title' } }),
        content: fields.markdoc({ label: 'Content', extension: 'md' }),
      },
    }),
  },
  singletons: {
    sidebar: singleton({
      label: 'Sidebar Navigation',
      path: 'src/components/sidebar',
      format: { data: 'json' },
      schema: {
        sidebarItems: fields.array(
          fields.object({
            label: fields.text({ label: 'Label' }),
            slug: fields.text({ label: 'Slug' }),
            collapsed: fields.checkbox({ label: 'Collapsed', defaultValue: false }),
            items: level2Items,
          }),
          { label: 'Main Sidebar Items', itemLabel: props => props.fields.label.value }
        ),
      },
    }),
  },
});