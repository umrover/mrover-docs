---
title: "Localization in the Loop"
---
# What is it?
The goal here is to be able to fully simulate localization in sim.

# Overview
The rover drives around in the simulation. We would like to be able to simulate the received GPS messages' (and other sensors - IMU and mag) noise, to then apply that noise and generate fake GPS data to use in our filters.

Even better would be if we can use this fake GPS positions to generate artificial GPS messages and RTK corrections, then feed those into the rover_gps_driver and have it figure out where we are and call the relevant callbacks.
