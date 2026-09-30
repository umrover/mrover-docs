---
title: "Rock Pick Pose Estimation"
---
**"Tags"**: Python, C++, TensorRT, AI/ML, USB Camera

**TL;DR**: During the Autonomy Mission, the rover needs to autonomously pick up the rock pick. To do this, we want to know the orientation of the pick lying on the ground. There will be a USB Camera pointing directly downwards above the rock pick. This project will use an AI model, probably some YOLO variant, to estimate the position and orientation of the pick so that another subsystem can correctly obtain it.