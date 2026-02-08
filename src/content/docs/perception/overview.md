---
title: "Perception"
---
# ArUco Markers

### Problem

ArUco markers (which are a type of fiducial markers) are special patterns than encode numbers. Often time the course will have these placed around so we can orient our rover and execute a certain task.

![image](https://user-images.githubusercontent.com/20666629/172561442-05b84fd3-aab9-4d5b-88d1-87579985dcff.png)

### High Level Overview

[OpenCV](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html) is used to run detection on the camera stream. This gives us information about where the tag is in pixel space, specifically its four corners. We can then fuse this with point cloud data, which gives us the xyz position for any given pixel relative to the camera. Specifically, we query the pointcloud at the center of the marker and thus find its transform relative to the rover.

![image](https://user-images.githubusercontent.com/20666629/168959240-7a4b6670-731c-42c0-8a82-066cf0825432.png)

We then publish the tags to the [tf](http://wiki.ros.org/tf) tree.

### Details

Update Loop:
1) Detect the IDs and vertices in pixel space of ArUco tags from the current camera frame.
2) Add any new tags to the "immediate" map or update existing ones. We calculate the center here by finding the average of the four vertices. If we also have a point cloud reading for this tag publish it to the TF tree as an immediate tag relative to the rover. These readings are filled in by another callback.
3) Decrement the hit counter of any tags that were not seen this frame. If it reaches zero remove them entirely from the immediate map.
4) Publish all tags to the TF tree that have been seen enough times. Importantly this time they will be relative to the map frame not the rover.
5) Draw the detected markers onto an image and then publish it

# Object Detection

### Problem

As part of URC the rover must be able to identify two objects, a hammer and water bottle. These objects will be placed in close proximity to two GNSS coordinates and the rover has to identify, locate, and drive to these objects.

![Objects](https://github.com/umrover/mrover-ros/assets/134429827/60b235f1-41d0-4566-8cd7-b309939f314c)

### High Level Overview

At a high level, the detection algorithm uses a custom ML model trained (Using [Roboflow](https://roboflow.com/)) to identify hammers and water bottles. This model is then loaded and executed on the GPU using NVIDIA's [TensorRT](https://developer.nvidia.com/tensorrt) framework. TensorRT takes advantage of the Tensor Cores available on the Jetson's GPU to accelerate the network's forward pass. The forward pass returns  a pair of coordinates in image space, and using the pointcloud this location is then converted into an xyz position. Finally, the xyz position is published to the tf tree in map space where navigation can then move towards the object.

### Example Video
https://youtu.be/1mlohZMx3wQ

### Details

Update Loop:
1) Grab Image from the ZED and convert to CNN input format.
2) Perform forward pass of CNN.
3) Locate the object in 3D space.
4) Add immediate object to ZED camera frame and increment hit counter.
5) If hit count is above a certain threshold publish object to map frame.
3) Decrement the hit counter of the object is not seen. If the hit count is below a certain threshold then stop publishing the object's location.
5) Draw the objects bounding box onto the image and then publish it.

# Cost Map

### Problem

An important part of autonomy at any competition we attend is knowing where---and more importantly where not---to go. To solve this problem, we use a cost map to classify the terrain around us as low or high cost (safe or potentially dangerous).

<img width="1850" height="600" alt="Screenshot from 2025-07-19 12-29-39" src="https://github.com/user-attachments/assets/296f51c4-cbef-43d3-830a-c143c9dec000" />

### High Level Overview

We take in a pointcloud from the ZED and filter out points that may throw off our determination of cost (the sky, too far out, etc.). We then bin the remaining points into 0.8x0.8 meter boxes and figure out what percentage of the points are considered high cost by using the ZED's ability to give [surface normal](https://www.stereolabs.com/docs/depth-sensing/using-depth#getting-normal-map) information. Finally, if the percentage of high-cost points is above some threshold, the bin is marked as high cost. Bins we are not currently looking at are remembered for better path planning. 

### Details

1) Grab pointcloud from ZED, down sample, and filter out any points that are outside of predetermined clips
2) For each point that is within bounds, check if the surface normal at the point is below a predetermined threshold
3) Increment that bin's total as well as the number of points with normals that are too low
4) For all bins with enough points (>16), use a TF transform to find its coordinates in world frame and discard if it >50% within the ZED's FOV (using simple ray casting)
5) For all accepted bins, check to see if the percentage of points that are high-cost is below some threshold, if not mark as high cost
6) After marking cost, split each bin up into 4 smaller bins and mark a one bin buffer around any high-cost bins as a special dilated cost value (splitting up for finer detail)
7) Publish the final cost map, then repeat for the next frame

# Visual Odometry

Visual Odometry is a method of determining where we are by tracking unique features across multiple camera frames.

We have the option of using the [Zed built-in tracking](https://www.stereolabs.com/docs/positional-tracking/) or [rtabmap](http://wiki.ros.org/rtabmap_ros) stereo odometry. We have found that both are high quality but the Zed built-in tracking runs at a higher refresh rate at the cost of being more of a black box.

# Nodelet Design

Communication between nodes has to use sockets in ROS by default since they all run in separate processes. We use [nodelets](https://www.stereolabs.com/docs/ros/zed-nodelets/) instead which all run inside of the same process. In this way they share a virtual address space and can share messages via pointers (zero-copy) which is ~50x faster.

One important note is that this message now needs to be thread safe. For this reason a new point cloud message is made every update by the point cloud publisher thread (instead of reusing the same one).

# Best ZED Settings

At least 720p is recommended. Anything lower will not work at long ranges. We also try to hit at least 15 Hz so information propagates fast enough to navigation. The ZED has four quality modes that increase in quality in exchange for more resources. They are (from lowest to highest quality): PERFORMANCE, QUALITY, ULTRA, and NEURAL.

For depth quality, we found that NEURAL is best. While other settings, like PERFORMANCE, allow up to 50hz `.grab()`, we found that image quality took a big hit, especially in the cost map. Speeding up our existing nodes allowed us to use the more resource-intensive NEURAL setting.  

We limit the maximum depth of the ZED 2i to around 10-14 meters (the default is 20). The ArUco detector cannot really find tags beyond this depth so it is not necessary. For the cost map, we shorten this range even further (around 5-7 meters). Cost map quality is degraded passed this point. 
***


# Testing

For off-rover testing a `zed_test.launch.py` file is provided. If you would like to run the entire perception stack, you may use the `perception.launch.py` file.

Usages:
`ros2 launch mrover <file_name>.launch.py`

Other configurable options: `run_rviz` (Useful for looking at TF tree), `run_dynamic_reconfigure` (useful for configuring tag detection settings), `run_tag_detector`

# Glossary

- **ArUco**: Special pattern of black and white blocks that encode a number. Often times called markers/tags/targets
- **Stereo Camera**: A camera that uses [stereo matching](https://web.eecs.umich.edu/~fouhey/teaching/EECS442_W22/slides/stereo.pdf) to produce point clouds
- **Features**: Unique patterns in an image that are persistent across frames. Corners are a good example
- [**Zed 2i**](https://www.stereolabs.com/zed-2i/): The stereo camera that we use
***

- [**OpenCV**](https://opencv.org/): A computer vision library
- **Point Cloud**: A collection of 3D points that roughly describe a scene
- **Odometry**: The "pose" of an object, in other words description of where it is in the world (usually position and rotation)
- **Pixel Space (or Camera Space)**: x and y coordinates of where a pixel is in an image
- **Cost Map**: A map of terrain that specifies the difficulty of traversing a certain region (high or low cost)
***
