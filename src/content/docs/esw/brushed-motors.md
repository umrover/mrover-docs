---
title: "Brushed Motors"
---
## Project Overview

On the rover exists a MCU board which essentially is a PCB powered by 12V which has
various voltage lines and MCUs embedded on it. Two of the MCUs embedded on it is what
we call the Motor MCUs, which are two STM32s chip whose tasks are to control the brushed motors
of the rover.

The brushed motors which the STM32s control are the arm joints (Joints A, B, F, Finger, Gripper), Mast Gimbal (Pitch and Yaw),
ISH carousel system, the SA system (Joints 1, 2, 3, Scoop, and Microscope).

---

## System Overview

The two STM32 MCUs are connected via the same I2C line. They are both follower devices and the leader device in the I2C bus is the Raspberry Pi.

---

## Top Level Code

The code is written in C++.

The I2C.cpp file is a wrapper for I2C. 
The Controller.cpp defines the Controller class which basically represents a motor.
The ControllerMap.cpp file basically creates Controller devices based on the config (esw.yaml) file.
The ROSHandler.cpp file defines all the functions that are called in response to services and topics.

---

## MCU Code

Code can be found in embedded-testbench in dev/motors.

https://github.com/umrover/embedded-testbench/tree/dev/motors

---

## Appendix

### Motor MCU Pinouts (Blurry - sorry)

There are some pins that happened to be swapped on the actual motors MCU board.

![image](https://user-images.githubusercontent.com/71603173/232115107-abc45131-1248-4b18-8ab7-3e385a60c9c4.png)


### Motor MCU 0 Pinout

Source is Motor MCU Pinout Google Sheets

![image](https://user-images.githubusercontent.com/71603173/232101670-2e1a04c3-24d8-49e5-b70d-46f441a2067c.png)

![image](https://user-images.githubusercontent.com/71603173/232110356-082e2aa7-58db-469c-852e-39e9ae832fef.png)

### Motor MCU 1 Pinout

Source is Motor MCU Pinout Google Sheets

![image](https://user-images.githubusercontent.com/71603173/232105468-14ba3667-ce8b-4e2c-816c-c558beb01b3a.png)


### Arm Topics

Source is Teleop ESW ICD Sheet Google Sheets
![image](https://user-images.githubusercontent.com/71603173/232098536-82320f6e-15d0-4969-a2f4-570f5039ee4f.png)

### Arm Services

Source is Teleop ESW ICD Sheet Google Sheets
![image](https://user-images.githubusercontent.com/71603173/232098331-d1c5d0a7-f28f-4019-b8cd-edab25fd9cc2.png)

### Science Topics

![image](https://user-images.githubusercontent.com/71603173/232098764-7a6da651-6f77-443c-99c7-0869c0f9f2e7.png)

### Science Services

![image](https://user-images.githubusercontent.com/71603173/232098938-864d9c5e-0764-4b11-a064-e8e9365506e2.png)




