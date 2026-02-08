---
title: "Brushed Controller ROS"
---
**Context**: On our rover, we have various PCBs that are responsible for the brushed motors on our rover. We call these PCBs our brushed DC motor controllers (BDCMC) PCBs. Take a look at the below picture to see a draft of all the devices on the rover and see where the BDCMCs fit into the system.

![ESW System 2024 drawio](https://github.com/umrover/mrover-ros/assets/71603173/13eb49e7-a9c3-4e05-b9bf-7684708c8dbd)

**Problem**: We need to write ROS programs that communicate with these BDCMC PCBs. Teleop will ask ESW to move the joints/motors, and ESW's ROS node needs to tell eventually communicate to the motors over CAN about how to reach that target (target velocity or target position). 

**Solution**:

* Develop all the ROS software in C++.
* Implement the bridge software
* Implement the motor_library library software
* Implement the CAN node (technically a separate project but it is very closely related)

**Interface**:

* There is currently not a good description of the interface since it is still in the works of being developed
* However, we can expect the ROS node to be able to do the following: Send velocity commands in terms of rad/s. Receive and process angular data based on quadrature encoder data. More.
* See the below picture to see how the software that you are writing fits into the bigger picture

![ESW Software drawio](https://github.com/umrover/mrover-ros/assets/71603173/a55d9f73-6df6-4c02-932a-4935b6ff1c42)

**Rough Steps To Get Started**:

* This project is big. So it might be overwhelming
* We are developing all code in the mrover-ros repository in the branch embedded
* The code is located in the src/esw/ folder in various folders
* DONE/NOT-TESTED: We need help developing the CAN node and making sure that we can process ROS messages and send out actual CAN messages
* For the CAN node, once we receive CAN messages back, we need to be able to understand it as well
* DONE: For the motors_library library, we need to be able to abstract between brushed motor controllers and brushless controllers. This project does not deal with brushless controllers but it is a consequence of working with the same software.
* DONE: For the bridges, you will need to call on the motors_library and keep track of instances that are Motors.
* DONE: Whenever you ask a Motor to set_velocity (or something similar to that), then you are asking for it to send a request to the CAN Node to send a message. 
* DONE: This request to the CAN Node should basically have the information of a raw CAN frame. The formatting of this raw CAN frame should be consistent with whatever is being written for the STM32 firmware. See the Brushed Controller STM32 for more information on that.
* As for hardware, you can perhaps try testing if you are sending out CAN messages once the CAN node is implemented. If you want to test on actual motors, then we currently do not have actual hardware to test on. This will change once we have EHW's dev board and on actual motors once we have EHW's actual BDCMC. Also, in order to test this actual hardware, then you will need to wait for the firmware for that hardware to be developed. See the Brushed Controller STM32 project for more information on that.
