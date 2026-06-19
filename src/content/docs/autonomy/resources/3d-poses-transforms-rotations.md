---
title: "3D Poses, Transforms, and Rotations"
---
### Pose
An object's pose refers to both its position and orientation in space. Representation of poses is a very important topic in robotics. While it might seem simple to pass around state variables like three Euler angles and a three element position vector, this will quickly lead to some serious pitfalls. For one thing, we will often be manipulating poses and passing around this many variables is messy. However, the larger issue with this simplistic representation is that it does not lend itself well to certain tasks. 

### Motivating example

Lets say I'm trying to compute a target orientation for the robot relative to the world coordinate frame. In particular, I have a target angle from perception relative to the frame of the camera, the current angle of the rover relative to the global frame, and the current angle of the Camera's gimbal relative relative to the frame of the rover. However, heres the catch: all these angles are measured slightly differently. The GPS returns an angle from [-180, 180], the gimbal angle is between [0, 360], and the target angle is from [-90, 90]. Worse yet, the target angle is actually measured in a left handed coordinate system meaning the positive angle is achieved by going clockwise. 

How do I compose these rotations? If I just add them up and modulo by 360 I will produce a nonsense result. Instead, I will need to write dozens of lines of confusing conditional logic to handle a variety of wrap around cases, axis flips, and frame conversions to make summing headings like this possible. On the other hand, lets say we were to represent all of these angles as matrices clearly denoted with their parent frame. Here, we use the notation $R_{object}^{parent}$. Now I can compute the composition as follows:

![CodeCogsEqn (1)](https://user-images.githubusercontent.com/10037572/172074100-fa816cf6-5b1b-4211-9cd9-74f18feb88e0.png)


This is not only much easier to read, but it is also extremely computationally efficient due to the existence of highly optimized matrix routines. The point of this example is to highlight how we need to be flexible in our representation of poses. While Euler angles may be nice to print out for a user to debug, they are often not good for internal state-keeping or operations.

### SE(3)
Extending on the above example even further, there are many representations of rotations. There are matrices, quaternions, euler angles, and Rodrigues vectors. Depending on what you are doing, there is likely an optimal representation. All of these representations are a way to encode information about a 3D rotation. In formal terms, they are all a way to represent an element of the SO3 group*, which is the group of all 3D rotations. 

To represent position, we use the R^3 group which you might be familiar with already. This is just the group of all 3-vectors, or the group of all 3D translations. 

Poses are a combination of position and rotation, and thus belong to the group of R3 x SO3. This combination is so common that it has a name of its own. That group is the special euclidean group denoted as SE(3). You can think of it as the set of all possible 3D poses.

We created an `SE3` class in both C++ and Python in order to represent 3D poses and provide the power to request different representations of the same rotation on the fly. This class provides utility functions for common operations on poses. See the documentation in this wiki about SE3 as well as the code docstrings for more information. 

Finally, it is worth noting that there are open source libraries out there that implement a similar thing such as [Sophus](https://github.com/strasdat/Sophus). This may seem contrary to our advice in the autonomy philosophy about not re-inventing the wheel. However, use of Sophus requires understanding some terminology from a very technical field of mathematics called Lie Groups, so to simplify things we have opted to provide a simpler implementation that is consistent with the ethos of our philosophy on writing libraries. 

### Pose vs Transform 
While there is a semantic difference between a pose and a transform, mathematically we represent both with `SE3`. See best practices for more info on this. 

\* There is some more nuance with quaternions since they technically belong to a different group called S3, but this is not super important since we can map elements of S^3 to SO3.

