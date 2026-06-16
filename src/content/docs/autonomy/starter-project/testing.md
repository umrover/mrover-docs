---
title: "Testing & Completion"
sidebar:
  order: 5
---
# Testing & Completion

## Testing your Work

Now open `starter_project.launch.py` which is in the `mrover/starter_project/autonomy/launch` folder. Make sure the lines declaring a node in the localization, perception, and navigation sections are uncommented.

To run the rover in the simulator:
1. In your terminal run `ros2 launch mrover starter_project.launch.py` to open the simulator, you should see the rover not moving in the simulator.
2. In a new terminal run `ros2 run mrover visualizer.py` so you can monitor which state the rover once you start it.
3. In the simulator press `p` to enable physics. Watch the rover move to (8, 2) while being in the DriveState, transition to TagSeekState, see the tag, and finally reach the tag and move to the DoneState.

<img width="194" alt="starter_project_states" src="https://github.com/umrover/mrover-ros/assets/101608185/b4d2a19b-1281-4907-b337-a5979898b4ee">

## Completion

Congrats! You've finished the autonomy starter project!
