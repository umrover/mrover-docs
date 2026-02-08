---
title: "Brushless Motors"
---
# Brushless Motors (Moteus)

## Project Overview (last updated 4/3/2024)

The Moteus controllers are used to control the brushless motors on the rover. We use the Raspberry Pi and the Pi3Hat to communicate to the motei that are located by the drive and arm joints via CAN. The three arm brushless motors (Joints C, D, and E) are connected on one CAN bus, all the left drive wheels are connected on another CAN bus, and all the right drive wheels are connected on the last CAN bus (result in three total CAN buses).

---
### _The following sections are in the order that they should be executed in._

# Download Firmware
Connect the moteus to CAN and power the moteus with ~36V. **Do not connect the motor yet.** 

If the moteus has never been used before, run tview with: `sudo python3 -m tview --target 1`

Otherwise, run tview with the CAN ID of the moteus (in decimal). CAN IDs can be found in [the software commands document](https://docs.google.com/document/d/1xOSqh_SqRyvgayEssh9qdBPRjPWQhUd8Kiva0eOGTvw/edit?usp=sharing).

In the Telemetry tab, check the Firmware Version. If it is not 262, you will need to update the firmware.

To update firmware, go to the [releases page](https://github.com/mjbots/moteus/releases) on the MJBots Github. Download the most recent firmware version available, this is the .elf file that does **not** have the word "bootloader" in it.

Once the .elf is downloaded, make sure the .elf is in the Downloads folder and run the command ``.

# Configure Moteus (Hall encoder)
1. Look at which Aux connector the Hall encoder should be wired to. Aux ports are labeled on the Moteus.
2. Using the [tables in the MJBots Reference Document](https://github.com/mjbots/moteus/blob/main/docs/reference.md#pin-options) figure out which Aux pin numbers the 3 Hall lines are connected to. 

For Aux1:

![Aux1 Pins](https://github.com/wisabel0/RoverImages/blob/main/aux1.png)

For Aux2:

![Aux2 Pins](https://github.com/wisabel0/RoverImages/blob/main/aux2.png)

To read the tables, match the labeling on the Moteus to the Aux Pin number in the same row.

3. Update the Moteus config in tview to match, check all Aux1 and Aux2 configurations by using the dropdown menus in the Config tab on tview.
4. Change the CAN ID to the appropriate ID by finding and editing the Device ID. Make sure the set the CAN ID using decimal numbers.
5. In the terminal in tview, run the command `conf write`.
6. Close tview

# Run Calibration
Make sure the moteus is **powered off** then plug the phase wires and encoder wires into the moteus. Connect the moteus CAN.

Make sure the motor is secured and positioned so that it can spin freely. It may vibrate slightly during calibration so be careful. The black "housing" part of the motor will also spin.

In the terminal run the following command:

`sudo python3 -m moteus.moteus_tool --target <CANID> --calibrate --cal-hall --cal-motor-poles <POLES>`

The number of poles and CANIDs can be found in [the software commands document](https://docs.google.com/document/d/1xOSqh_SqRyvgayEssh9qdBPRjPWQhUd8Kiva0eOGTvw/edit?usp=sharing).

# PID Tuning
TODO

# Commit a working version of the configuration


# Set up additional hardware
All motei need a commutation source, for Rover applications this would be the hall encoder. Any other encoders or hardware that need to be set up for the moteus firmware to handle need to be on the other Aux port. For example, if using a hall encoder on Aux1, the absolute encoder must be on Aux2.
## Absolute Encoder
The moteus supports a limited selection of absolute encoders. We use the [AS5048B](https://look.ams-osram.com/m/287d7ad97d1ca22e/original/AS5048-DS000298.pdf).

Configure the moteus to read the absolute encoder data:
1. Configure I2C device 0 on the Aux port that the absolute encoder is wired to to be type: AS5048
2. If there is only one absolute encoder on the moteus, the address should be 64. If there are multiple absolute encoders, refer to the reference sheet and selection pins
3. Change the motor_position.output or motor_position.reference_source as needed

## Limit switches
We handle limit switches in application layer code (ROS). However, we need to set the moteus up to have data on the limit switches ready. 
1. Change the Pins configuration in tview. Go to aux[1,2]->Pins. Configure the pin(s) that the limit switch(s) are connected to to be digital_in and pull_up.
2. Check that the pin values are updating as expected with the pin value in the Telemetry tab. Connect the limit switches and press them to see the values change. The values are a decimal conversion of binary bits based on which pins are high or low.

# Other helpful commands
## Resetting all Moteus configs
If restarting with Moteus setup, you can connect to the Moteus in tview and run the command `conf default`. Note: this will change the CAN ID back to 1 as well as configure the Moteus to use SPI.

## Check Hall States with Moteus
To check if the motor's hall encoders are working or if the Hall configurations on the Moteus is set up correctly: 
1. Connect the motor encoder wires to the Moteus
2. Open tview and go to the telemetry tab. Under the Aux port that the encoder is connected to, click Hall and read the Bits value as you turn the motor by hand.
3. Bits values should be 1, 2, 3, 4, 5, 6. If you see any numbers other than these (0 or 7) or not all of those numbers, either the Hall sensor in the motor is broken, the wiring between the hall encoder and the Moteus is wrong/bad, and/or the configuration in the Moteus is wrong.

## Configuration and PID tuning

To configure the moteus, you would first like to change some of the settings through the GUI called tview. Settings you'd like to change include the current limit, PID values, CAN ID, and others.

To save a particular configuration, use `sudo moteus_tool -t X --dump-config > filename.cfg`. Run this through the the moteusConfigCalib.py script found in /scripts/moteusConfigCalib.py. It will spit out "filename_write.cfg". You can use `sudo moteus_tool -t X --write-config filename_write.cfg` to write the config to moteus X.

To open tview, run ```python3 -m moteus_gui.tview```. If you would like to run various targets, run something like ```python3 -m moteus_gui.tview --target 1,2```. If you are unable to detect a CAN-FD transport (if you have the fdcanusb connected), you should run the command ```sudo chmod a+rw /dev/ttyACM0```.
  
In case you want to configure the Moteus Manually you can follow these steps: 
* In order to set the CAN ID open config > id > change value to desired ID
![2023-02-24-004111_1920x1080_scrot](https://user-images.githubusercontent.com/45600974/224767434-1aabc0b0-ecf4-44b8-9a19-664bfe6c779b.png)
* Re-open tview and run `conf write`
* Open aux 2 or the port (your hall lines are on) int the config page of tview
* Disable spi
* Set hall enable to true
* Set pins 2 and 3 to work on hall mode (these are the debug pins on the Moteus), also enable pull ups
* Find out if the 3rd pin is connected to pin 1 or pin 0 [here](https://github.com/mjbots/moteus/blob/main/docs/reference.md#aux2--abs), set that to work on hall mode also enable pull ups.
![2023-02-24-004159_1920x1080_scrot](https://user-images.githubusercontent.com/45600974/224767679-a2110b0a-faf7-4f45-a012-8614be7283e7.png)
* Go to motor position > sources > 0 > aux number and set it to 2, also set it to hall mode
![2023-02-24-004235_1920x1080_scrot](https://user-images.githubusercontent.com/45600974/224767776-bcc2e86a-0ea6-4e47-b42b-956edfb1e0da.png)
* Go to config => servo => pid_position; set the PID values 
  kp = 0.5; ki = 0; kd = 0.05 for the Maxon motors.
* Set max current to 4 A
* Set max_position_slip to 10
![2023-02-24-004304_1920x1080_scrot](https://user-images.githubusercontent.com/45600974/224767880-270bd5f1-536a-4951-9a50-a2b72657797a.png)
* config > servopos  > position_min = nan
* config > servopos  > position_max = nan
![2023-02-24-004323_1920x1080_scrot](https://user-images.githubusercontent.com/45600974/224767917-8f0d06cb-3df7-4bc7-8048-20cae6f73e3e.png)
* Run `conf write` to save the configuration to memory

Changing the I limit does not do anything if the ilimit is 0 (which is fine for basic velocity control applications).

## Watchdog

A watchdog timeout of 1.0 seconds is used.
If communication between the jetson_teleop node and the brushless_motors node is lost (after 0.1s), then the brushless_motors node will tell the moteus to turn its velocity to 0. This basically means means that jetson_teleop stopped receiving updated commands from the basestation (or from navigation).

If communication between the rover node and moteus is lost (if the moteus does not receive a new CAN message after 0.1s), then the moteus driver will be placed in brake mode. 

MOTEUS_RESPONSE_TIME_INDICATING_DISCONNECTED_S = 0.01
ROVER_NODE_TO_MOTEUS_WATCHDOG_TIMEOUT_S = 0.1

# Appendix

### Pinout

Source is Moteus Documentation Google Sheet
![image](https://user-images.githubusercontent.com/71603173/232091633-c050fcc9-e785-41a2-868f-a25930c68b9d.png)

### Topics

Source is Teleop ESW ICD Sheet Google Sheets
![image](https://user-images.githubusercontent.com/71603173/232092947-f2d5ca7d-614e-49f6-bd63-a4b3e74e78f0.png)

