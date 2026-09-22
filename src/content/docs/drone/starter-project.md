---
title: "Software Starter Project 2026-27"
---

# Drone Software Starter Project

This year, we are looking to develop capabilities for the drone to operate autonomously during the URC missions. The aim of the drone software starter project is to familiarize you with the key frameworks and software tools we will use to operate the drone and implement capability for autonomous flight. Once you have installed [all necessary software](/setup/getting-started), proceed through the sections of this page to complete the starter project.

## ROS2 Introduction

ROS2 is a 'meta operating system' for robotics applications. The goal of ROS2 is to provide a set of tools and libraries to simplify the process of developing and running robotics software. Before beginning the starter project, work through the tutorials on the ROS2 website. Do these tutorials in a new directory in `~/ros2_ws/src`.
1. Do the <b>CLI tools</b> tutorial to understand the basics of ROS2 and how to use it: [https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools.html). Skim over the 'Configuring environment' section, but do the rest thoroughly. 
2. Do the **Publisher & Subscriber** tutorials, in the language you are more comfortable in: [https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)

These are the fundamentals of ROS2. The docs linked have many additional tutorials that cover pretty much anything you would ever do with ROS2, making them a fantastic resource.

## Clone and create starter project branch

Once you have completed the ROS2 tutorials, move onto the starter project. Here we will create a ROS2 node that will control PX4 through MAVROS, moving the drone! 

We will do this using `starter_project`, a ROS2 node that you will create. You will write most of your code in `starter-project/src/starter_project_mavros_node.cpp`, but integral to this structure is also `starter-project/launch/starter_project.launch.py`, which launches the controller node, and `drone_gs/launch/mavros.launch.py`, which launches MAVROS.

## Project Structure

The data flow is:

```text
PX4 <-> MAVLink <-> MAVROS <-> ROS 2 starter_project_mavros_node
```

The node will receive the drone's state and pose (position data) from MAVROS, and use them to publish setpoints from a list for the drone to go to. While keeping a drone in the air is complex, PX4 makes sure that the drone can fly and controls the motors, making your job a lot easier.

### Tasks
1. Skim through the code in starter-project/src/starter_project_mavros_node.cpp. Feel free to ask questions if anything looks confusing!
2. Complete the position subscription and poseCallback() function
3. Complete the updateTargetIfReached function. This function helps the drone cycle through the setpoints
4. Feel free to put in your own list of setpoints! 

## Building and Running

Building this project can seem confusing at first, but as you keep working on it it should get a lot easier! Here is a list of commands in order to run to launch a drone simulator (Gazebo and QGroundControl), MAVROS, and your ROS2 Node.

### Build

First, you have to create a simulation drone using PX4 and Gazebo.**Open a new terminal** and paste in the following commands.

```zsh
cd ~/ros2_ws/src/mrover_drone/deps/PX4-Autopilot
make px4_sitl gz_x500
```

**Open a new Terminal.** Then, run the following code to launch QGroundControl, which will give you a top down view of the simulation drone over a satellite image.

```
cd ~/ros2_ws
source /opt/ros/jazzy/setup.zsh
source ~/ros2_ws/install/setup.zsh
colcon build --symlink-install --packages-select drone_sim  --symlink-install
ros2 launch drone_sim px4_sitl_gz_ros2.launch.py
```

Your screen should now look like this. If not, ensure you have the proper software installed and ran the right commands.

<img width="2880" height="1800" alt="Screenshot from 2026-09-21 23-24-37" src="https://github.com/user-attachments/assets/c2a8fe70-dcfb-4a41-9e88-3ca9c6316b5a" />

**Open another new Terminal.** Then, run the following code to initialize MAVROS, which will allow for PX4 to communicate with your node.

```
cd ~/ros2_ws
unset AMENT_CURRENT_PREFIX AMENT_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/jazzy/setup.zsh
source ~/ros2_ws/install/setup.zsh
colcon build --symlink-install --packages-select drone_gs
ros2 launch drone_gs mavros.launch.py \ fcu_url:='udp://:14540@127.0.0.1:14580'
```

You're almost there! **Open another new Terminal.** Run the following code to start up your node!
```
cd ~/ros2_ws
unset AMENT_CURRENT_PREFIX AMENT_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/jazzy/setup.zsh
colcon build --symlink-install --packages-select starter_project drone_gs
source ~/ros2_ws/install/setup.zsh
ros2 run starter_project starter_project_mavros_node --ros-args -p auto_offboard:=true -p auto_arm:=true
```
Now you should be able to see your drone flying on to the points you set in QGroundControl! Gazebo can also show you a visual of this. In the future, we can also experiment with running this code on our real drone!


After cloning the drone repo, switch to the starter project branch:
```
git fetch
git checkout starterproject27
```
Now, create a branch for your own starter project:
```
git checkout -b <your name>/starterproject27
```


