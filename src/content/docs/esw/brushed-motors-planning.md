---
title: "Brushed Motors Planning"
---
## Task: Write brushed motors ROS code
Location: `mrover-ros/embedded/esw/brushed_motors`
- [ ] Get Quintin's Code to build
### Arm to Can Node
Reads in arm message from rostopic ra_cmd. This message contains a motor name, target velocity, and target position. Then, the node will convert this message into a can message and publish to rostopic can_messages. A different can node will deal brushed vs. brushless
Will probably look very similar to last year's code. 
```
motor_message:
	motor_name
	velocity
	position

can_message:
        bus_name
        can frame
```
#### Implementation Details:
Written in Python 
***
## Task: Write brushed motors open loop control
Location: `mrover-ros/embedded/esw/fw/motor_mother`

#### Implementation Details:


***
## Task: Test brushed firmware code w/out motor

#### Implementation Details:
