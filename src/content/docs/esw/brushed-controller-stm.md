---
title: "Brushed Controller STM"
---
**Context**: On our rover, we have various PCBs that are responsible for the brushed motors on our rover. We call these PCBs our brushed DC motor controllers (BDCMC) PCBs. Take a look at the below picture to see a draft of all the devices on the rover and see where the BDCMCs fit into the system.

![ESW System 2024 drawio](https://github.com/umrover/mrover-ros/assets/71603173/13eb49e7-a9c3-4e05-b9bf-7684708c8dbd)

**Problem**: These PCBs need an MCU to process all incoming CAN requests and control the motor based on that. Thus, we use an STM32 chip as our main MCU. The issue is that we need code for that.

**Solution**:

* Develop STM32 software (firmware) using the CubeIDE in C++
* The firmware should be able to receive CAN messages and process them
* The CAN messages should be consistent with whatever the corresponding ROS program is sending (see the [Brushed Controller ROS project](/esw/brushed-controller-ros))
* The CAN messages interface should be defined 
* The firmware should be able to control the movement of a particular motor

**Interface**:

* There is currently not a good description of the interface since it is still in the works of being developed
* However, one can expect the STM32 to be able to do the following: Process velocity commands in terms of rad/s. Report angular data based on quadrature encoder data. Respect limits based on limit switches.
* See the below picture to see how the firmware that you are writing fits into the bigger picture

![ESW Software drawio](https://github.com/umrover/mrover-ros/assets/71603173/a55d9f73-6df6-4c02-932a-4935b6ff1c42)

**Rough Steps To Get Started**:

* This project is big. So it might be overwhelming
* We are developing all code in the mrover-ros repository in the branch embedded
* The code is located in the src/esw/fw/bdcmc folder
* CubeIDE should be installed
* As for hardware, we currently do not have actual hardware to test on. This will change once we have EHW's dev board and on actual motors once we have EHW's actual BDCMC.
