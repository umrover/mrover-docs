---
title: "Ros Bagging Data"
---
# What is it?
"ros2 bag" is a command used to collect information from sensors and store it in a file for easy playback later. The goal of this project is to bag some data from localization testing (done), and we want to be able to replay it in sim and see how our localization performs with the given data compared to the ground truth. This would allow for more consistent testing since we could test systems against a static baseline of data.

# Requirements
1. Ros bag some data (done)
2. Figure out how to play it back in the sim
3. We need a way to visualize the pose published by the filter compared with the ground truth, as well as to save the plot so we can compare by overlaying with results from other algorithms

