---
title: "Keyboard Typing"
---
**"Tags"**: C++, OpenCV, Arm Control, Computer Vision, USB Camera

**TL;DR**: During the ES mission, we want to type on the lander keyboard autonomously with the robotic arm. There are four ArUco tags located at the corners of the keyboard, which we will use to automatically align with the keyboard using inverse kinematics. Once we have aligned with one of the keys, we move the arm purely vertically and horizontally with joint A and by opening gripper to press the rest of the keys. Currently, there is no method to align along the z-axis; however, it seems possible to get this information by analyzing the size of the detected tags. The end goal is to get the typing service, that when called with a parameter (the code to be typed), to correctly manipulate the arm to align with the keyboard and type the correct code.

<!--
arm base joints diagram

demonstrate how the gripper opens and closes to press keys

**Motivation**: We were not able to type on the keyboard autonomously last year at URC. Additionally, the arm is being redesigned as joint B (formerly a linear actuator) is now a brushless motor and the gripper is changing.  

**Goal**: We want autonomous keyboard typing to be run at competition. We should meet the following:
* Using the ArUco tags, we can autonomously align the arm with the keyboard.
* We can press any letter key on the keyboard with some amount of tolerance.
* The typing process is reliable and trustworthy without risk of failure or damage to the arm. 

**Interface** 
ros topic interfaces here

-->