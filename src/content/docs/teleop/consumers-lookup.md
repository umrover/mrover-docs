---
title: "WebSocket Handlers Lookup"
---
WebSocket handlers manage communication between the frontend and ROS2. Each handler maintains a persistent WebSocket connection for a specific subsystem, forwarding ROS2 topics to the GUI and publishing GUI inputs to ROS2.

Handlers are defined in `teleoperation/basestation_gui/backend/ws/`.

---

# WebSocket Usage per View

Each view connects to the WebSocket handlers it needs via the `:topics` prop on `BaseGridView`:

### ESView (DM/ES missions)
- `arm`, `drive`

### DMView (Device Mission)
- `arm`, `drive`, `chassis`, `nav`

### AutonView
- `drive`, `nav`, `science`, `chassis`

### ScienceView (SA mission)
- `arm`, `chassis`, `drive`, `nav`, `science`

---

# Handler Reference

## ArmHandler

WebSocket: `arm`

### Publishing
- `/arm_thr_cmd` (Throttle)
- `/ik_pos_cmd` (IK)
- `/ik_vel_cmd` (Twist)

### Forwarding
- `/arm_controller_state` -> `arm_state`
- `/arm_ik` -> `ik_target`

### Receiving
- `ra_controller` - gamepad axes and buttons

---

## DriveHandler

WebSocket: `drive`

### Publishing
- `/joystick_vel_cmd` (Twist)
- `/controller_vel_cmd` (Twist)

### Forwarding
- `/left_controller_state` -> `drive_left_state`
- `/right_controller_state` -> `drive_right_state`

### Receiving
- `joystick` - drive joystick axes and buttons
- `controller` - drive controller axes and buttons

---

## ScienceHandler

WebSocket: `science`

### Publishing
- `/sp_thr_cmd` (Throttle)

### Forwarding
- `/sp_humidity_data` -> `sp_humidity`
- `/sp_temp_data` -> `sp_temp`
- `/sp_oxygen_data` -> `sp_oxygen`
- `/sp_uv_data` -> `sp_uv`
- `/sp_ozone_data` -> `sp_ozone`
- `/sp_co2_data` -> `sp_co2`
- `/sp_pressure_data` -> `sp_pressure`
- `/sp_controller_state` -> `sp_controller_state`

### Receiving
- `sp_controller` - science arm gamepad axes and buttons

---

## NavHandler

WebSocket: `nav`

### Forwarding
- `/gps/fix` -> `gps_fix`
- `basestation/position` -> `basestation_position`
- `/drone_odometry` -> `drone_waypoint`
- `/led` -> `led_color`

### Subscribing
- `/nav_state` - triggers LED manager state updates

### Additional
- Publishes TF-based localization data (orientation) at 10 Hz

---

## ChassisHandler

WebSocket: `chassis`

### Forwarding
- `/gimbal_controller_state` -> `gimbal_controller_state`

---

## AutonHandler

WebSocket: `auton`

### Receiving
- `code` - typing code string or cancel command

### Actions
- `/es_typing_code` (TypingCode action)
- Sends back: `typing_error`, `typing_accepted`, `typing_feedback`, `typing_cancelled`

---

## LatencyHandler

WebSocket: `latency`

### Receiving
- `ping` - timestamp, sequence, payload

### Responding
- `pong` - server timestamp, mock motor data

---
