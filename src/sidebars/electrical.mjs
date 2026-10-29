// Sidebar for this section. Each entry is either a page or a group of pages.
//
//   page:  { label: "Text shown in the sidebar", slug: "electrical/my-folder/my-page" }
//
//   group: { label: "My Group", items: [ ...pages or more groups... ] }
//          add collapsed: true to start the group closed
//
// Example: adding a new board page
//   1. create src/content/docs/electrical/ehw/boards/my-board.md starting with
//        ---
//        title: My Board
//        ---
//   2. add it to the Boards group below:
//        { label: "My Board", slug: "electrical/ehw/boards/my-board" },
export default [
  { label: "Home", slug: "electrical" },
  { label: "Starter Project", slug: "electrical/starter-project" },
  { label: "Tech Talks", slug: "electrical/tech-talks" },
  {
    label: "Embedded Hardware",
    collapsed: true,
    items: [
      { label: "Overview", slug: "electrical/ehw" },
      {
        label: "Boards",
        collapsed: true,
        items: [
          { label: "ABS", slug: "electrical/ehw/boards/abs" },
          { label: "BLMC", slug: "electrical/ehw/boards/blmc" },
          { label: "BMC", slug: "electrical/ehw/boards/bmc" },
          { label: "Fuse", slug: "electrical/ehw/boards/fuse" },
          { label: "LIM", slug: "electrical/ehw/boards/lim" },
          { label: "PDB", slug: "electrical/ehw/boards/pdb" },
          { label: "Science", slug: "electrical/ehw/boards/science" },
        ],
      },
    ],
  },
  { label: "Comms", slug: "electrical/comms" },
];
