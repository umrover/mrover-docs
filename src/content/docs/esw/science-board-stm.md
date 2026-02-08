---
title: "Science Board STM"
---
## Context
The science system, which is designed for the science mission at URC, uses sensors, heaters, LEDs (think what is inside of ISH system). We want to control all of these with one STM chip on a science PCB. 

Look at the following diagram to get an idea of how this PCB fits in with the rest of hardware and software:

![ESW System 2024 drawio](https://github.com/umrover/mrover-ros/assets/71603173/13eb49e7-a9c3-4e05-b9bf-7684708c8dbd) 

## Problem
ISH is iterating their system, and want to use different components, which we have to write code for.

## Solution
* Develop STM32 software (firmware) using the CubeIDE in C and C++
* The firmware should be able to receive CAN messages and process them
* The CAN messages interface should be defined 
* The firmware should be able to communicate with sensors, LEDs, and heaters (hardware specifics are defined by ISH)

## Interfaces
* For sensors: STM32 chip should read in and update values constantly, and send them to the Jetson when requested via CAN
* For other actions, the STM32 chip should be able to process commands for LEDs, and states for heaters

## Getting started
* Start with the interface between the STM32 and the sensors, LEDs, heaters
* Add in the CAN communication
* Add ROS science_hw_bridge

