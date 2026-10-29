---
title: "Continued Arm IK Development (Arm Controller)"
---
# Context: 
Last year, we attempted to fully integrate IK(position & velocity) to both enable the operator to more easily control the arm and use it for the autonomous typing challenge. However, we had issues with the arm sagging, mainly caused from Joint B, causing our confidence for the algorithm to decrease. Every year we change the robotic arm, and because of issues with Joint B, it is being changed. We are hoping this change on the arm will fix some of the issues we faced last year.

Additionally, the Auton Payload is new to the rover this year.

# Problem: 
With these new changes, we have to ensure that Arm IK is updated with new limitations and parameters. This includes updating our current code and fully testing the program on the rover. Additionally, we want to ensure that our implementation of IK is smooth and is intuitive for operators. 

We also need a new controller for the Auton Payload that can be driven autonomously. It needs to specifically be able to grasp and lift a rock pick hammer on the astronaut's command.

# Solution: 
Continue our development of the carrot-on-a-stick approach found [here](/software/archive/projects/arm-velocity-control). Since we had issues last year, our implementation of velocity control constantly changes. Since we are hoping the new joint will fix some problems, we need to cement our previous algorithm and ensure it works.

In addition, create a Auton Payload controller.

## Rough Steps for Velocity IK:

Continue with the previous design found [here](/software/archive/projects/arm-velocity-control). Investigate considering position IK for when teleop stops sending commands.

General Overview of Algorithm:

<img width="523" height="510" alt="Screenshot from 2026-09-12 18-15-55" src="https://github.com/user-attachments/assets/99c97426-50db-49c4-b075-708234860690" />

## Rough Architecture for Auton Payload:

<img width="1003" height="450" alt="Screenshot from 2026-10-29 17-42-31" src="https://github.com/user-attachments/assets/20eb77d9-0540-4451-87f5-52014cff935a" />
