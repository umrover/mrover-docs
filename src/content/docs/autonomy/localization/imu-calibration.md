---
title: "How to Calibrate the IMU"
---
# Introduction

Calibrating the IMU is a magical process. It is a ritual we must follow almost to a T otherwise the IMU will remain uncalibrated for a very long and sad time. However, the good thing is that calibrating the IMU is very easy! As long as you follow a couple of simple steps **exactly**, you will very quickly have an IMU that is fully calibrated.


# Process

Start by taking the IMU off of the stand. Be sure that there is slack in the USB cable and that we have comms from the base station and can read the IMU calibration status numbers on the GUI.

## GUI numbers:

The IMU calibration status can be found in the bottom left side of the Auton GUI (and probably the others too, TODO make sure this is right)

<img width="317" alt="image" src="https://user-images.githubusercontent.com/20312121/230968843-b16b843a-5576-426d-b350-2a34c7fc60ef.png">

## Step by Step Process

* Start the IMU level with the ground then turn the IMU relatively quickly in one step-like motion **45 degrees** about the axis of the USB cable
* Stop and hold still for around ten seconds
* Repeat this process until the Accelerometer calibration status is 3.
* Once the accelerometer number is 3, if the other numbers are not yet 3, give the IMU a gentle shake and spin it about the other two axes until the numbers all turn 3; this should happen quickly.

## Diagram

<img width="1124" alt="image" src="https://user-images.githubusercontent.com/20312121/230969190-76a3edf7-2d5e-4a89-a377-b74b00c7936e.png">


# Expected Time to Complete and Progression of Status Numbers.

The Accelerometer calibration number will start at 0, then it will go to 1, then it will go back to 0, then it will go back to 1 and finally, it will hit 3 (yes, it's magic). 

Generally, we get to 3 a little bit after one full rotation about the axis of the USB cable. ***Do not pay attention to the numbers that are not Accelerometer until the accelerometer is fully calibrated***

# Troubleshooting

Usually, the process will work exactly as described above. Generally issues arise from not following the process laid out above correctly. Common mistakes are rotating the IMU about the wrong axis, and not holding still in between each rotation for a long enough period of time.

If the calibration numbers of the IMU do not seem to be updating at all (including non-accelerometer calibration statuses):

Verify that the IMU driver is running by running the following command:

`rosnode list | grep imu`. If the node is not running start it with:

`rosrun mrover imu_driver.py`. Check for errors, if there are none proceed to calibration as usual (but notify someone that the IMU driver crashed) if there are errors it is likely because the IMU is not plugged in on either the IMU or jetson end correctly.

If the IMU driver is in fact running:

Run `rostopic echo /imu/calibration` in a shell that is connected and can communicate with the jetson and verify that there are numbers coming back. If there are numbers coming back, keep trying to calibrate. If no numbers are coming back or if all the numbers are 0:

* unplug the IMU USB.
* replug in the IMU.
* kill the IMU driver node if it did not die from unplugging IMU (`rosnode kill imu_driver`)
* rerun the IMU driver (`rosrun mrover imu_driver.py`)

For other issues please slack the relevant people, probably Riley

# Theory

While it may seem magical, there is (probably) logic in why we calibrate the IMU the way we do. TODO: figure out what the logic is