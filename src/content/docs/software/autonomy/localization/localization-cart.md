---
title: "Localization Testing Cart"
---
# What is it?
Localization does a lot of testing outside, but we often do it on a grey cart from the Wilson Center. We place the zed and antennas on the cart, but the issue is that we just kind of eyeball it and there is almost certainly some angular and position error compared to how the antennas and zed are located on the rover. This leads to inaccurate testing conditions. The goal of this project is to work with the testing subteam to make a cart so we can test outside with accurate and consistent conditions like on the rover.

# Requirements
The main requirement are the following:
1. There's a place to put a computer - we need a computer on the cart to run the localization stack
2. There's a place to put the antennas, distanced and angled the same as on the rover - antennas give us GPS position and heading data, and are a vital component of localization.
3. There's a place to put the ZED, angled the same as on the rover - The ZED gives us IMU and mag data, and is necessary to run the IEKF.

# Useful features
Some non-required features would be helpful
- A place to put a comms battery and charger - this would prevent us from needing to bring a longass extension cord from the NAME building and be tethered to both that AND the basestation.

