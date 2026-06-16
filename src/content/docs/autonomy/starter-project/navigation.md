---
title: "Navigation"
sidebar:
  order: 4
---
# Navigation

For navigation, you will write a ROS node that uses the outputs of the localization and perception nodes in order to command the rover towards the point and then the AR tag. In the end, the rover will turn towards a given point (8 meters, 2 meters) and then drive towards it until it is in a set threshold. Once there, the rover should be able to see an AR tag. The rover will turn towards the AR tag so it is directly aligned, then drive forward until it is in a set threshold. 

It is often easiest to think in terms of inputs and outputs so here are the inputs and outputs for this node:

### Inputs
- Rover pose (the rover’s position and orientation in space): transform from TF tree
- Tag message published from the perception node (from channel `tag`)

### Outputs
- Velocity data: [Twist message](http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/Twist.html)

### General Project Structure

We've created the base structure of the project for you in order to mimic the structure of our current code. We recognize that there are other ways to do this, and probably ways to do the same thing in less code, however we want you to gain an understanding of our state machine and the general way we structure code in the navigation subsystem. With that said, here is an outline of each subcomponent of the navigation system and what it is responsible for as well as what you will need to do in each one in order to complete the project. Start by implementing the underlying infrastructure required for the navigation node to be able to properly read its inputs and send its outputs. Then write the logic for each of the two states that you will need for the project.

The files you need to edit are under `mrover/starter_project/autonomy/src/navigation`.

#### Context class (no TODOs, fully provided)

In `context.py`

The goal of this class is to provide access to an interface for reading and writing information regarding the rover's environment and the rover itself. It contains objects of `Rover` and `Environment` both of which you will need to implement various functions in. It also contains the ROS publisher and subscriber objects that you will need to use to listen to and send the messages for this node.

 
#### Rover class

In `context.py`

Lets start by implementing the Rover class. The rover object (accessed through the context object) is responsible for the interface for all things related to the rover. In our case, this will mean we need to implement three functions: 

1. `get_pose(self) -> Optional[SE3]`

Here we will need to fill in the code so that the function returns an SE3 object that represents where the rover is in the world. For more information about SE3s and what they are checkout [this wikipage](/autonomy/resources/3d-poses-transforms-rotations). Additionally, read over the docstrings within `mrover/starter_project/autonomy/src/util/SE3.py` to see the full SE3 interface and understand what the interface is. 

Once you do that, you're going to want to use the `SE3.from_tf_tree` function in order to get the SE3 that represents the rover's pose. This function is found in the `SE3.py` file. We have imported this class into the context file. The function’s parameters include:

`tf_buffer`: the tf buffer used to query the TF tree
`parent_frame`: the parent frame of the desired transform
`child_frame`: the child frame of the desired transform

Note that the `tf_buffer` has already been created for you and is owned by the Context object which is called `ctx` in the rover class. Once again, since this is the pose of the base of the rover relative to the map, you should specify your parent frame as "map" and your child frame as "rover_base_link" (this is the name used for the base of the rover).

2. `send_drive_command(self, twist: Twist)` 

Here we will need to fill in the code so that the function publishes a twist message using the velocity command publisher object that is also already a member of the context (`vel_cmd_publisher`). This is of type Publisher so you will need to use the function `publish()` on this publisher. If you wish, you can learn more about publishers and subscribers [here](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)

3. `send_drive_stop(self)`

Here we will need to fill in the function so that it stops the rover when run (hint #1: use `send_drive_command()`) (hint #2: `Twist()` initializes velocities to be 0 in its message).

#### Environment class

In `context.py`

The environment class is similar to the rover class except instead of providing an interface to things going on on the rover, it provides an interface to things going on in the environment around the rover. For us this means implementing two functions:

1. `receive_fid_data(self, message : StarterProjectTag)`

This function is the callback function that we have set up for you such that it will be invoked every time we receive an AR tag position message. You will need to update a member variable in the Environment class to hold the StarterProjectTag message passed to this function (hint: fid stands for fiducial, which is the AR tag in this case).

2. `get_fid_data(self) -> Optional[StarterProjectTag]`

This function should return either `None` or your AR tag position message. It should return `None` if you don't know where the tag is, or it should return the most recent ROS message regarding the AR tag's position if you do know where the tag is.
HINT: Use the same member variable you just set in your `receive_fid_data` function.

Now we have finished creating the base-level interfaces that we will use to actually write the logic on top! Basically, all of the interaction with the outside world is complete, we now will use the abstractions we've created to code the logic of our state machine. Let's take a look at how we structure single states and build the states that we will need. After that, we will hook our states into the greater state machine and we will have completed the project!

First of all, what is a state? Please read [this page](/autonomy/navigation/overview) to learn what a state is.

#### Done state (no TODOs, fully provided)

See `state.py`

We have provided a Done State that represents the rover in its "Done State" because we don't want our program to just end when the rover completes, we have a state that essentially just loops and does nothing. We have also provided a Fail State that represents failure to finish the task.

#### Drive State

In `drive_state.py`

The goal of this state is to drive towards the set point (8, 2). We've created the class for you but you will need to implement the `on_loop()` function. We've provided a general outline in the comments of how this function might be structured and have also providing a function `get_drive_command()` imported from `drive.py` that you can use to do some of the math in this step for you. The function should return the state it needs to transition to next.

Hint: Use the functions from context.rover we've already written

#### Tag Seek State

In `tag_seek.py`

The goal of this state is to drive towards the AR tag after arriving at the set point (8, 2). We've created the class for you but just like the Drive State you will need to implement the `on_loop()` function. This will be a bit trickier as you don't actually have a pose to drive to you just have the same general measurements regarding angular and distance offsets that you calculated earlier.

You can get information about how close the rover is to the tag by using the function `get_fid_data` in context.env. You want the rover to be within a certain distance (`DISTANCE_TOLERANCE`) from the tag, and face the tag within a certain angular distance (`ANGULAR_TOLERANCE`), to be able to transition to the Done state. We have set these tolerances to be `DISTANCE_TOLERANCE = 0.99`, `ANGULAR_TOLERANCE = 0.3`. Hint: `get_fid_data()` returns `StarterProjectTag`, this includes information about how close the rover is to the tag and the measurements of where the center of the tag is in our view (x and y).

If the rover is not within the angular and distance tolerances, create a twist command and change the linear.x value and/or the angular.z value so that the rover becomes within the tolerances. Then send this twist command to the rover and stay in the TagSeekState.

#### Navigation class

In `navigation_starter_project.py`

The navigation class is where the whole state machine comes together. We've already done the hard work of creating the whole state machine now we just need to put it all together. We've already added the DoneState to the state machine, and use a similar pattern for adding the TagSeek and DriveState states to the state machine. 
