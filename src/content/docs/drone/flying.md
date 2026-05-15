---
title: "Flying the Drone"
---

## 1. Obtain needed components
#### Drone
* Li-ion battery
* Drone 
* 4x Props (2CW, 2CCW)
* 2x M6 right-handed hex nuts
* 2x M6 left-handed hex nuts

#### Base station
* Laptop
* Laptop charger
* Battery charger
* Battery charger cable
* RFD-900x US radio modem
* 2x 5dBi 900MHz dipole antennas
* Xbox controller
* Ethernet cable
* Network switch
* ElectrifyRC1200 VRX
* 3.5dBi 1200MHz antenna
* RCA Video capture card
* 2x USB extension cords

### Tools
* M10 wrench

## 2. Pre-flight actions
* Attach antennas to FPV headset.
* Plug in radio modem to laptop.
* Open QGroundControl.
* Plug in drone & FPV batteries.
* Drone GPS will flash blue, then red. Do not move the drone during this process.
* Wait for GPS to turn green. It will appear in QGroundControl.
* Calibrate sensors.
* Calibrate accelerometers.
* Configure connection loss behavior.
* Plug in X-box controller.

## 3. Flying the drone
* Select 'Position' flight mode.
* When you see 'Ready to fly', click on the 'Ready to fly' text and click 'Arm'
* After the propellers start spinning, you can go and fly!

## 4. Software setup
```
ifconfig # Find IP
ros2 launch drone_gs mavros.launch.py fcu_url:="udp://localhost:14555@35.3.247.114:14550"
```