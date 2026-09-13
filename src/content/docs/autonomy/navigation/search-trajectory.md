---
title: "Search Trajectory Research"
---

# Concept
Currently, we use search trajectories in our search state. The only search state utilized is outward spiriling. This path prioritizes full ground coverage over full camera coverage over the area. As URC states that we only have to detect objects then stop, this year we will be doing research on different search trajectories.

We will also be looking into potential high area of interest detection. Because we have LIDAR data, we may be able to flag areas of interest and further investigate them. This includes areas like shelfs, were the rover might accidentically pass over the object and never detect it again.

To accomplish this, you will modify the `SearchTrajectory` class, overloading the current Search Trajectory to create different patterns. Additionally, it may need to subscribe to LIDAR data to detect areas of interest and generate a path based on this data.

Here are some proposed search trajectories:

<img width="478" height="413" alt="Screenshot from 2026-09-12 19-45-53" src="https://github.com/user-attachments/assets/a9374198-61cd-4bed-a410-24f71f82fe29" />

<img width="215" height="223" alt="Screenshot from 2026-09-12 19-46-05" src="https://github.com/user-attachments/assets/d1d960bd-4c62-4e67-a991-8ee5a3d419f2" />
