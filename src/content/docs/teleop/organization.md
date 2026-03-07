---
title: "Teleop Codebase Organization"
---

### Lockfile Tracking

Teleop does not track bun.lock. Lockfile conflicts are frequent acorss branches, and package.json should contain critical information regarding dependency and version control. Further granulated control of dependencies are not needed, and therefore the lockfile should not be tracked. 