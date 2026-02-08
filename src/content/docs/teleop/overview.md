---
title: "Teleop Overview"
---
# What is the role of the Teleoperations team?

The Teleoperations (Teleop) team ensures the rover's physical systems are easily controllable by a human operator. We build and maintain key systems that allow operators to command the rover and receive feedback in real time.

## Base Station GUI

The Base Station GUI is a Vue.js web app that serves as the main interface for operating the rover. It:

- Captures controller inputs
- Displays GPS, camera, and sensor data
- Sends waypoints to auton
- Controls devices used in science sample acquisition

and more...

## Django Backend

To support the frontend, we have a Django backend that connects to ROS via websockets. It:

- Handles all ROS callbacks
- Stores persistent data like GPS waypoints
- Computes robotic arm commands

# Resources

[Teleop Quickstart](/teleop/quickstart)

[Teleop FAQ](/teleop/faq)

[Vue Introduction](/teleop/vue-introduction)

[Bootstrap Introduction](/teleop/bootstrap-introduction)

[GUI Styling Guidelines](/teleop/gui-style-checking)

[Consumers Lookup](/teleop/consumers-lookup)

[Teleop Starter Project](/teleop/starter-project)

[Sample Vue Component](/teleop/sample-vue-component)