---
title: "ESW Overview"
---
# Embedded Software

The embedded software team of MRover (ESW) writes the driver code that allows the other programming subteams to utilize the electronic equipment on the rover. ESW works primarily with libraries in C, Python, and C++ to abstract the functions needed by the other teams for easy use. To do this we use different communication protocols such as I2C and CAN to transmit and receive relative data from sensors.

This page will list all the systems that the ESW sub-team must write code for. If you are interested in helping out or in any of these systems or offering suggestions for improvement, please reach out! You can do this via by sending a message on Slack or by submitting a GitHub issue ticket.

The steps for onboarding are listed below in the separate section.

Wiki has been updated as of 01 September 2025.

## Quick Summary of Systems

Brushless motors (for the drive system and parts of the arm) are driven by moteus brushless motor controllers.

Brushed motors (for parts of the Robotic Arm system, the Science Payload system, and the mast gimbal) are driven by STM32 chips. We are currently developing custom brushed motor controller PCBs.

The science system is primarily controlled by a different STM32 chip. We are currently developing a custom science board PCB.

The camera system is run on the Jetson to stream camera feed.

<!--

## System Block Diagram

Below is an OUTDATED system block diagram (this was for Dahlia, the 2022-2023 rover). It includes systems such as the brushless motors (moteus), brushed motors, science, and camera systems.

![ESW System 2023 drawio](https://user-images.githubusercontent.com/71603173/232123443-c7f17ebb-9b77-43d4-bc94-eca1750b5d6c.png)

-->

## Onboarding

Below are the all the things you need to do for onboarding (Steps 1-2 are independent of 3-7)

1. [Install Ubuntu OS](/getting-started/install-ros)
2. [Install ROS on Ubuntu](/getting-started/install-ros)
3. Follow the steps on the [ESW Starter Project](https://umrover.github.io/mrover-esw/getting-started/intro/) page. This has a few more steps.
4. Finished! Now go ask a lead to get involved on our [projects](https://umrover.github.io/mrover-esw/projects/overview26/)!

