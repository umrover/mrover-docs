---
title: "Search Trajectory Research"
---
Currently, we use search trajectories in our search state. The only search state utilized is outward spiriling. This path prioritizes full ground coverage over full camera coverage over the area. As URC states that we only have to detect objects then stop, this year we will be doing research on different search trajectories.

We will also be looking into potential high area of interest detection. Because we have LIDAR data, we may be able to flag areas of interest and further investigate them. This includes areas like shelfs, were the rover might accidentically pass over the object and never detect it again.

To accomplish this, you will modify the `SearchTrajectory` class, overloading the current Search Trajectory to create different patterns. Additionally, it may need to subscribe to LIDAR data to detect areas of interest and generate a path based on this data.

Here are some proposed search trajectories:
