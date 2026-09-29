// Sidebar for this section. Each entry is either a page or a group of pages.
//
//   page:  { label: "Text shown in the sidebar", slug: "electrical/my-folder/my-page" }
//
//   group: { label: "My Group", items: [ ...pages or more groups... ] }
//          add collapsed: true to start the group closed
//
// Example: adding a page inside a group
//   1. create src/content/docs/electrical/my-folder/my-page.md starting with
//        ---
//        title: My Page
//        ---
//   2. add it below:
//        {
//          label: "My Group",
//          items: [
//            { label: "My Page", slug: "electrical/my-folder/my-page" },
//          ],
//        },
export default [
  { label: "Home", slug: "electrical" },
  { label: "EHW", slug: "electrical/ehw" }
  { label: "Comms", slug: "electrical/ehw" }
];
