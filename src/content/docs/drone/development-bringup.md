---
title: "Full Development Bringup"
---

# Full Development Environment Startup
This is a tutorial and explanation on how to bringup the development environment from scratch. 

Set up the environment:
```
cd ~/ros2_ws/src/mrover_drone/
source /opt/ros/humble/setup.zsh
source install/setup.zsh
```
Run the drone_sim launch file:
```
ros2 launch drone_sim px4_sitl_gz_ros2.launch.py
```
If testing base-station code, connect mavros:

This reflects how the groundstation code will interact with PX4. Fcu_url is the address of the flight controller.
Because we are running the software simulator locally, its the loopback address (127.0.0.1) at the default port of 14557.
```
cd ~/ros2_ws/src/mrover_drone/
source /opt/ros/humble/setup.zsh
ros2 launch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
```
If you are running with the actual drone, run `ifconfig` to find the radio link's remote address. Then run:
```
ros2 launch mavros px4.launch fcu_url:="udp://localhost:14555@35.3.235.213:14550"
```
(assuming `35.3.235.213` is the remote address)

### Checking for issues

To make sure everything is working as expected, run:
```
ros2 topic list
```
You should see a series of topics prepended with 'mavros' and some with 'fmu'.

Now, side by side, run:
```
ros2 topic echo /mavros/global_position/global
ros2 topic echo /fmu/out/vehicle_global_position
```
You should see two different data formats, but if you check the latitude and longitude they should be the same.