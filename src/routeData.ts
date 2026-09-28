import { defineRouteMiddleware } from "@astrojs/starlight/route-data";

// show only the sidebar group matching the first path segment, e.g. /software/... -> "Software"
export const onRequest = defineRouteMiddleware(({ locals, url }) => {
  const route = locals.starlightRoute;
  const section = url.pathname.split("/")[1];
  const group = route.sidebar.find(
    (e) => e.type === "group" && e.label.toLowerCase() === section,
  );
  if (group?.type === "group") route.sidebar = group.entries;
});
