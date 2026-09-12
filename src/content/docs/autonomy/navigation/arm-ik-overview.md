---
title: "Continued Arm IK Development (Arm Controller)"
---
# Context: 
Last year, we attempted to fully integrate IK(position & velocity) to both enable the operator to more easily control the arm and use it for the autonomous typing challenge. However, we had issues with the arm sagging, mainly caused from Joint B, causing our confidence for the algorithm to decrease.

Additionally, every year we change the robotic arm, and because of issues with Joint B, it is being changed. We are hoping this change on the arm will fix some of the issues we faced last year.

# Problem: 
With these new changes, we have to ensure that Arm IK is updated with new limitations and parameters. This includes updating our current code and fully testing the program on the rover. Additionally, we want to ensure that our implementation of IK is smooth and is intuitive for operators.

# Solution: 
Continue our development of the carrot-on-a-stick approach found [here](/archive/projects/arm-velocity-control). Since we had issues last year, our implementation of velocity control constantly changes. Since we are hoping the new joint will fix some problems, we need to cement our previous algorithm and ensure it works. Furthermore, to improve our arm-typing and confidence in our position IK, we will implment trajectory generation and velocity profiling. This means that the arm will follow a straight line to a given cartesian coordiante. It will also keep a smooth and constant speed while it follows this path.

## Rough Steps for Position IK:
1. Obtain a given position from '/ik_pos_cmd', given from teleop.
2. Given the current arm's position, create a vector/path that points towards the target position.
3. After generating a vector, pass it into a function which generates a velocity profile. This will be a function of expected EE position with time. This means we need to determine the speed of the arm in cartesain coordinates. 
4. Each iteration, compare expected EE position with actual position. Error will be used to correct and send joint velocities each loop. If we stray off the path for too long, pause the current time to allow the arm to catch up and to prevent too much error accumilation.

General Overview of Proposed Algorithm:

<img width="523" height="510" alt="Screenshot from 2026-09-12 18-15-42" src="https://github.com/user-attachments/assets/50323b71-a051-49a2-9700-fe038df7edfd" />

## Rough Steps for Velocity IK:

Continue with the previous design found [here](/archive/projects/arm-velocity-control). Investigate considering position IK for when teleop stops sending commands.

General Overview of Algorithm:

<img width="523" height="510" alt="Screenshot from 2026-09-12 18-15-55" src="https://github.com/user-attachments/assets/99c97426-50db-49c4-b075-708234860690" />
