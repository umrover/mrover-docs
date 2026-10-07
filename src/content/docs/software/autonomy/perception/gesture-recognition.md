---
title: "Gesture Recognition"
---
**"Tags"**: C++, Computer Vision, AI/ML, ZED Depth Camera

**TL;DR**: During the Autonomy Mission, the rover will need to detect body gestures the astronaut makes. We want to perform gesture recognition on at least one command, follow (via a beckoning gesture), and potentially several others. The full list of commands can be found in the [URC rules](https://urc.marssociety.org/home/requirements-guidelines) on the website. While we are unsure what the EVA suit will look like at this point, we will likely train an AI model to perform human pose estimation with keypoints to identify the gestures. The ZED 2i camera on the rover has built-in pose estimation capabilities, but we are unsure how reliable it is and thus it needs to be tested. The end goal is creating a ROS node that accurately detects gestures and informs Navigation when the rover needs to change state. 