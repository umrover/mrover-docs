---
title: "Best Practices"
---
## Representing Poses 
* Always represent pose with the `SE3` type from our library for internal states
* Function signatures that require a pose should always have an SE3, this creates a unified interface for our libraries. Functions can internally unpack SE3 into the necessary forms for work
* It is worth noting that there is a semantic difference between the word "pose" and "transform", but they are mathematically related by the same object - an SE3. One could have the pose of an object in the world frame, which when thought of or applied as a transform is the transform from the object's frame to the world. For this reason, unlike the ROS messages, we have a single type for poses and transforms. 
* Because of the fact that we do not distinguish between poses and transforms with the type, it is critical to follow a consistent naming convention to make it clear what a particular instance of `SE3` actually encodes. If you have a transform, name your variable like `{frameA}to{frameB}`. If you have a pose, name your variable `{frameA}in{frameB}`. Make sure to following the naming conventions of the language you are using. Here are some examples: 

Rover's pose in the world for python: 
`rover_in_world = SE3(...)`

Transformation from camera frame to rover base frame in C++
`SE3 cameraToBase(...)`

## Logging
* Use `RCLCPP_INFO()`, `RCLCPP_WARN()`, `RCLCPP_ERROR()` etc. to print information to console for rclcpp.
* Use `get_logger().info()`, `get_logger().warn()`, `get_logger().error()` etc. to print information to console for rclpy.
* More information about logging can be found [here](https://docs.ros.org/en/foxy/Concepts/About-Logging.html)

## Python (General) 
* Keep logic associated with objects as much as possible. Sometimes this means making use of `@classmethod` or `@staticmethod`. Do not write stuff like `utils.py` with completely random helper functions. 
* Classes that are essentially data storage containers should be `@dataclass`. In general prefer to make classes data storage containers and do complex initialization logic with static factory methods instead of relying on `__init__`. For those familiar with C and C++, this should be most similar to C style programming.

## Contributing to `mrover-ros`
If you are making a change/working on a ticket, please do NOT fork the `mrover-ros2` repository. Merging in forked commits can lead to history rewriting and a lot of headaches when trying to clean up or revert any potential issues. Instead, as described in the workflow tutorials, checkout a new branch, and use the format `<initials or github username>/<feature or change name>` when naming the branch. Doing this method will make merging in changes from `main` easier, and avoid rebasing.