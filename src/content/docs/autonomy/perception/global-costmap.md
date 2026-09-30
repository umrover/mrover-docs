---
title: "Global Costmap"
---
**"Tags"**: C++, Transforms/Maps, ZED Depth Camera

**TL;DR**: During the autonomous route-finding portion of the Autonomy Mission, we need to be able to autonomously navigate the rover along a difficult path. While we will likely use the current cost map to detect immediate obstacles, we also want the overall context of the environment. Since the URC rules give us the 1x1 square mile location of the task, we want to generate a cost map based on available elevation data of the area. The end goal is a file that represents the cost map of the task environment area.