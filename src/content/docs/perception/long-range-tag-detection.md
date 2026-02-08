---
title: "Long Range Tag Detection"
---
**Context**: [URC rules](https://urc.marssociety.org/home/requirements-guidelines) dictate that we must detect ArUco tags that are placed upon posts:

> The 3 posts will have GNSS coordinates that are within the vicinity of the posts,
increasing in range from approximately 5-20 m.

Please also read the Wiki page on [perception](/perception/overview).

**Problem**: The [ZED stereo camera](https://www.stereolabs.com/zed-2/) can only detect tags around 8 meters away. As such we have to run a spiral search to find the tag. This takes precious time since the autonomy period is only 30 minutes long.

**Solution**: Longer distance tag detection via a secondary camera (compared to just the ZED):

* Process at higher resolution and lower frame rate
* Use optical magnification via a higher focal length lens
* Detect direction of tag much earlier
* Does not have to be stereo
* Navigation will run a control loop to center the tag on the screen, and then drive forward
* Navigation will obey 3D readings from ZED when close enough

<hr>

**Interface** (subject to change)

Node: `long_range_tag_detector`

Subscribes: [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)

Publishes:

`Tags.msg`
```
Tag[] tags
```

`Tag.msg`
```
uint8 id
float32 bearing
```

Note: This will most likely have to run as a [nodelet](http://wiki.ros.org/nodelet) under the perception_nodelet_manager since passing Image messages is costly otherwise.

Rough Steps:
1. Create a subscriber for the Image topic
2. Write a function that takes an Image and detects tags
3. Calculate the bearing of the tag based on the camera FOV (see below)
3. Create a publisher for the Tag topic
4. Write a function that publishes the detected tags from the Image message to the Tag topic

<hr>

**Tag Bearing**
To tell navigation where to drive, we must compute a bearing from the tag information. This can be done using information about the ZED's camera [intrinsics](https://support.stereolabs.com/hc/en-us/articles/360007395634-What-is-the-camera-focal-length-and-field-of-view-) and the tag's location in pixel space.

![FOV](https://github.com/umrover/mrover-ros/assets/51866496/1c8f8142-ea06-454e-bea2-0f90bf4e9772)


<hr>

We also need a Node that can deliver an Image message. Try out this premade [usb_cam](http://wiki.ros.org/usb_cam) package first. You should be able to install it via `sudo apt install ros-noetic-usb-cam`. If it is too slow we can try rolling our own using V4L or [`cv::VideoCapture`](https://docs.opencv.org/4.2.0/d8/dfe/classcv_1_1VideoCapture.html).

Rough Steps:
1. Try the usb_cam package first
2. If not, summarize your findings to a lead. We are targeting around 15-30 Hz update rate.
3. The lead will create a C++ node if we are not satisfied
4. Try to use `cv::VideoCapture` to read a `cv::Mat` and publish that as an Image message
5. If too slow, try V4L directly
6. We will go from there