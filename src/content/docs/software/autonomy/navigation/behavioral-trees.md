---
title: "Behavioral Trees"
---

# Context
Navigation's high-level decision making is currently a state machine. As we add more behaviors, it keeps growing in conditionals and transitions.

# Problem
The current state machine is dense with conditionals and multiple transitions. The Astronaut Assistance mission would require even more states and conditions, making it harder to read, modify, and test.

# Solution
Convert the state machine into a behavioral tree. Behavioral trees are designed for decision-making applications and give us:

- Easier modularity and reusability of subtrees without adding additional state
- A more intuitive structure for our rover
- A clean way to add Astronaut Assistance without bloating the existing logic

We will use the library `py_trees_ros` for the implementation. [(learn more here)](https://py-trees-ros-tutorials.readthedocs.io/en/release-2.4.x/tutorials.html#tutorials-section)

## Architecture

- Separate the mission into **two nodes**: Route Finding and Astronaut Assistance. Each contains its own behavioral tree.
- Remove unnecessary code and convert the current state machine into the Route Finding behavioral tree.
- Architect the Astronaut Assistance mission as a new behavioral tree.
- Both trees use the shared **Context** to keep track of outside data. Context still owns Course (waypoints), Environment (targets, costmap), Rover, and the Drive Controller, exactly as it does today.
- The existing ROS services (toggle costmap, move/dilate costmap, toggle auton, toggle pure pursuit, etc.) stay the same.

## Behavioral Tree Diagrams

 * [Astronaut Assistance](/software/autonomy/navigation/astro-assistance)

 * [Route Finding](/software/autonomy/navigation/route-finding)