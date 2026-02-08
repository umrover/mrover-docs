---
title: "Brushless ROS"
---
**Context**: On our rover, we have brushless motors. We control them using brushless motor controllers called Moteus.

![ESW System 2024 drawio](https://github.com/umrover/mrover-ros/assets/71603173/13eb49e7-a9c3-4e05-b9bf-7684708c8dbd)

**Problem**: We need to write ROS programs that communicate with these brushless motor controllers. Teleop will ask ESW to move the joints/motors, and ESW's ROS node needs to tell eventually communicate to the motors over CAN about how to reach that target (target velocity or target position). 

**Solution**:

* Develop all the ROS software in C++.
* Implement the bridge software
* Implement the motor_library library software for the brushless side only
* Implement the CAN node (technically a separate project but it is very closely related)

**Interface**:

* There is currently not a good description of the interface since it is still in the works of being developed
* However, we can expect the ROS node to be able to do the following: Send velocity commands in terms of rad/s. Receive and process angular data based on encoder/hall effector sensor data. More.
* See the below picture to see how the software that you are writing fits into the bigger picture

![ESW Software drawio](https://github.com/umrover/mrover-ros/assets/71603173/a55d9f73-6df6-4c02-932a-4935b6ff1c42)

**Rough Steps To Get Started**:

* Get the developer kit running using the GUI
* Talk to the moteus with a C++ testing file (either move at a certain velocity, or continuously print out angles)
* Integrate software with rest of actual brushless motor control
* Need to see how to send the moteus commands as raw commands
