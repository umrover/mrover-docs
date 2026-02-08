---
title: "Electrical System Overview"
---
# Intro

In order to understand the embedded system of the rover, it is first important to understand the electrical system of the rover.
Below is the electrical system information for the 2023 rover Dahlia.

---

## EBox

Our rover uses one 40 V Li-ion battery to power on the entire rover. The battery voltage goes into a power distribution board (PDB) PCB. Multiple converters on the PDB drop the voltage levels into other voltage lines (3.3V, 5V, and 12V). We also have an 24V converter that is separate from the PDB.

There is an MCU board. This board takes in 12V as an input. It has 3 MCUs (the STM32 MCU): 2 of them are used for controlling brushed motors (communicating with the Arduino Uno UART to I2C bridge via I2C) and 1 of them is used for controlling science operations and controlling MOSFETs (communicating via UART). There are various MOSFETs on the MCU board.

There is an ethernet switch inside the rover and a POE used for handling communication between the radio and the two main devices on the rover (Jetson Xavier and Raspberry Pi).

The Jetson Xavier is powered using the 12V line. It has the USB hub connected to it, and also has the GPS, IMU, Raman nucleo, cameras, and Arduino Uno used to control the Auton LEDs. It also connects to the science MCU located on the MCU board using UART (two wires).

The Arduino Uno used to control the Auton LEDs takes in serial messages (over USB) from the Jetson Xavier and controls pins that connect to the auton LED PCB. There are 4 wires that are connected between the Arduino Uno and the Auton LED PCB. 

Our Auton LED PCB takes in 3.3V and it has 3 inputs that are used to control each of the auton LEDs (red, green, and blue). When the input goes low, then the LED turns on since there is a voltage differential between the 3.3V and the input that has been  pulled low. When the input goes high (either 3.3V - 5V or disconnected), the LED turns off since there is no completed circuit or because there is no voltage differential in the correct direction of the diode.

The Raspberry Pi 4 has a pi3hat on it. The pi3hat takes in 12V and it powers the Raspberry Pi 4 too. The pi3hat is connected to 3 CAN buses (2 for drive and 1 for arm). The Raspberry Pi is connected to an Arduino Uno used as an UART to I2C bridge connected to it.

The Arduino Uno used as a UART to I2C bridge is used to convert serial messages (over USB) from the Jetson to I2C to the motors MCU. This was added because the current MCU board was likely killing I2C pins on the Raspberry Pi and Jetson Xavier, so it was added as a temporary solution. It has two UART wires connected to the Jetson and two I2C wires connected to the MCU board.

---

## Drive

Because there are 6 wheels, we have 6 brushless motor controllers (moteus) placed beside each wheel to control it. Each moteus takes 36V. The three moteus on the left suspension are on one CAN bus while the other three moteus on the right suspension are on one CAN bus.

---

## Arm

Only joints C, D, and E use brushless motors (everything else uses brushed motors). Each moteus takes 36V. All of the moteus are on one CAN bus, separate from the CAN buses on the drive system.





