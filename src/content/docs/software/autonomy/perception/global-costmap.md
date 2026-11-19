---
title: "Global Costmap"
---
**"Tags"**: C++, Transforms/Maps, ZED Depth Camera

**Motivation:** During the Autonomous Route-Finding Sub-mission (see 1.e.xii through 1.e.xviii of the URC rules), we need to be able to autonomously navigate the rover up a hill that won't be approachable from all directions. This implies the route will be pretty complex. 

Since the URC rules give us a 1x1 square mile within which the task will take place, we want to scout out that area beforehand and provide useful information to Navigation so that they can better navigate up the hill. To do this, we will provide a costmap based on available elevation data of the area. This global costmap will be combined with the local costmap generated using the ZED's pointcloud data to provide as much context as possible for path planning .   

**Goal:** Provide a static, global costmap based on available elevation data to aid Navigation in long-distance path planning and large obstacle avoidance. It should be useful (the data actually benefits Navigation) and feasible (the costmap is not ridiculously large in terms of cell count and file size on the Jetson).

**High-level Overview:** Using data from DEM (Digital Elevation Model) maps, we will derive a costmap based on the slope, roughness, step, and potentially other factors. Searching around for DEM maps online, it seems the best data and resolution we can get is 1x1 m squares where each square only has its measured elevation. With this information, we will calculate the following:
* Slope: The slope of a cell via an algorithm like Horn's method, which utilizes a 3x3 area around the target cell to calculate its steepness. 
* Roughness: The bumpiness of the terrain. One way to calculate is fitting a plane to a 3x3 area, and then calculating the residual of the cell to the plane.    
* Step: The largest height difference between the target cell and any of its neighbors.

These factors are just a starting point, and the end costmap might not be reflective of this description. Feel free to come up with other factors if you think it will benefit the final costmap's accuracy! 

Finally, the global costmap should be aligned with the local ZED costmap. The two costmaps are misaligned in multiple ways, including the frame, grid, and array order. In particular, the local costmap is oriented ENU (East-North-Up) at a reference point, while the DEM maps are oriented using UTM. Thus, a marginal transformation needs to be applied to align the two. (Disclaimer: I (Daniel) don't fully understand what this means; this will be updated later when it is clear). 

**Inputs, Outputs, and Interfaces:** The input DEM maps are .tif files, and the output file will be a .csv or similar that contains the costs of each cell. This file will be stored on the Jetson and referenced to by Navigation. In order to produce the costmap, I would recommend you write a script (e.g. python) that performs the relevant calculations. In particular, try to make the script adaptable so that we can weight the different factors for testing purposes. The relevant interfaces for this project are Navigation (since they are using the map) and Localization (since you might need localization information to reorient the map).   

**TL;DR**: During the autonomous route-finding portion of the Autonomy Mission, we need to be able to autonomously navigate the rover along a difficult path. While we will likely use the current cost map to detect immediate obstacles, we also want the overall context of the environment. Since the URC rules give us the 1x1 square mile location of the task, we want to generate a costmap based on available elevation data of the area. This costmap will aid with long-distance path planning and obstacle avoidance. The end goal is a file that represents the cost map of the task environment area.