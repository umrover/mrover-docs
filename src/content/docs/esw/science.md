---
title: "Science"
---
## Project Overview

The science board on our team's rover is a PCB with an STM32 MCU and various components that allow the science board to control our team's ISH system. The board is responsible for interfacing with various sensors, controlling heaters via MOSFETs and thermistors, controlling LEDs via MOSFETS, and sending data to the team's Jetson over CAN.

---

## System Overview

The 2024-2025 rover used two science boards in its ISH system. Each board is responsible for two heaters, two thermistors, and one LED. Additionally, one board manages the sensor suite (a protoboard with two I2C sensors and two analog sensors). Each heater and LED has its own MOSFET connected to a GPIO pin on the MCU, allowing us to enable and disable the devices. All analog devices including the thermistors were connected to one of the two ADCs on the MCU. Finally, each I2C sensor had its own I2C bus. Sensor data, heater states, heater temperatures, and LED states were sent to the Jetson over CAN using the CAN transceiver on the science board.

can_bridge.cpp uses SocketCAN to publish CAN traffic from the science boards to ROS 2 topics. science_hw_bridge.cpp then subscribes to these topics and handles decoding messages and publishing data to multiple topics. science_hw_bridge.cpp also handles the control logic for enabling and disabling the heaters and LEDs controlled by the science board, sending CAN messages to the board when the user makes requests through the team's teleop GUI.

---

## MCU Code and IOC

MCU code and IOC can be found [here](https://github.com/umrover/mrover-ros2/tree/science_board/esw/fw/science)

---

## ROS Bridge

ROS bridge can be found [here](https://github.com/umrover/mrover-ros2/blob/urc-25/esw/science_hw_bridge.cpp)

---

## Services and Topics

The science board services and topics can be found in the telop ESW ICD sheet in the google drive.

---

## Configuration

The various science board parameters should be configurable using a YAML config file.


