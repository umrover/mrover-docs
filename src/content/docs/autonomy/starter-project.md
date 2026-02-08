---
title: "Autonomy Starter Project"
---
# Overview

![Starter Project Scene](https://media.githubusercontent.com/media/umrover/mrover-ros/JRA/starter-project/data/starter_project/rover_and_tag.png)

Welcome to the autonomy starter project tutorial! In this tutorial, you will write code for all three subsystems encompassed by autonomy: perception, navigation, and localization. You will then put them all together in order to complete a task with a simulated rover. The goal is to familiarize yourself with what type of problems we tackle and how to do that with our codebase.

When you complete the project you will have a rover that uses its localization system to navigate to a set waypoint and then uses its perception system to drive it to a tag, here is an example of a working solution (click the image below, it's a link):

[![Solution Video](https://i9.ytimg.com/vi/H43zXWK88_c/mq2.jpg?sqp=CJi8mbcG-oaymwEmCMACELQB8quKqQMa8AEB-AH-BYAC4AOKAgwIABABGHIgTyg7MA8=&rs=AOn4CLBdUoSc71sv2S9d0wsMf_-6L_yIdw)](https://youtu.be/H43zXWK88_c)

# Getting Started

Use the `mrover` command to navigate to `~/ros2_ws/src/mrover`:

Follow the setup instructions [here](/getting-started/install-ros) to setup up the codebase. Note the following folder `./starter_project`, this is where the majority of the work will be done.

# Perception

Make sure you read the "Overview" of the [perception page](/perception/overview) before working on this part of the project. Note that this is the overall Wiki page for our non starter project code and there is one specific to the starter project later in this subsection.

For perception, you will write a ROS node that uses camera data to detect ArUco tags using [OpenCV](https://github.com/opencv/opencv), a popular computer vision library. Navigation will read this data in order to align the rover with it.

### Inputs
- Image data: `Image` messages published to the `zed/left/image` topic via the [`image_transport`](http://wiki.ros.org/image_transport) package

### Outputs
- Custom message published to `tag` topic with:
    - Tag center location in camera space
    - An approximation for rover to tag distance

Now proceed to [perception-specific page](/autonomy/starter-project-perception) for the implementation! You will notice a common theme on MRover is breaking down problems into inputs and outputs then providing more details on specifics in separate locations. And remember, the [ROS2 humble wiki](https://docs.ros.org/en/humble/index.html) is one of your greatest resources!!


# Localization
Make sure you read the "GPS", "IMU", "GPS Linearization", and "Guide to Localization Frames" sections from the [Localization page](/localization/overview) before working on this part of the project (RTK is beyond the scope of our starter project but recommended).

For localization, you will write a ROS node that uses GPS and IMU data to figure out where the rover is, and then publish this information as a transform in the TF tree. Here is the interface your node should follow:

### Inputs
- GPS data: [NavSatFix messages](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/NavSatFix.html) published to the `/gps/fix` topic
- IMU data: [Imu messages](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Imu.html) published to the `/imu/data_raw` topic

### Outputs
- Rover pose: transform in meters published to TF tree

### Subscribing to Sensor Data

You should start by subscribing to the GPS and IMU sensor data provided as inputs. To do this, you will have to create two `rclpy.Subscriber` objects. The python syntax for creating a subscriber looks like this:
```py
self.create_subscription(msg_type, topic_name, callback_function, buffer_size)
```
What this effectively does is tell ROS that your node should listen to, or "subscribe" to a ros topic `topic_name` that carries `msg_type` messages, and whenever you receive one, `callback_function` should be called. The callback function headers for `gps_callback` and `imu_callback` have already been provided for you, so make sure to use them when creating the subscribers.

In your callback functions, all you should do for now is print out the data you're receiving (which comes in the form of the `msg` argument to the function), that way you can make sure you're successfully getting the data. For more help on writing a subscriber, read [this ROS tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) on writing a Publisher/Subscriber Node.

### Adding your node to the launch file
In order to run your node along with the rest of the starter project, you will need to add it to the starter project launch file. Open `starter_project.launch.py` in the launch folder and uncomment the line declaring a node in the localization section. Notice the syntax for launching a node:
```py
...
localization_node = Node(package="mrover", executable="localization.py", name="localization")
...
return LaunchDescription(
     [
          ...
          localization_node
     ]
)
```
As you can see, we are launching a node that we have named "localization", is from the mrover package, and has an executable `localization.py` which is what will actually get run.

Once this line is uncommented, you should be able to launch the file with
```bash
ros2 launch mrover starter_project.launch.py
```


### Linearize GPS coordinates into euclidean coordinates
Now that you have your data, you need to use it to figure out where the rover is; Let's start with GPS. The GPS latitude and longitude coordinates tell us where we are in spherical earth coordinates, but what we really want are cartesian (x, y, z) coordinates in meters. Because our rover is driving in an area that is tiny relative to the size of the earth, we can approximate the area the rover drives in as a flat plane tangent to the surface of the earth:

![image](https://user-images.githubusercontent.com/32557768/187351874-6eb6fe51-17bd-49cd-bf08-f6d99abbad5e.png)

For this to work, we will need to define a point on the earth in (latitude, longitude) coordinates where the center of our tangent plane will be; this point will be called the reference point. As long as this reference point is 
close (within a few hundred miles probably) to where the rover is going to be, this tangent plane will be a pretty good approximation. For this project we will simulate the rover as driving near the Mars Desert Research Centre in Utah. Use the reference latitude and longitude found in `config/reference_coords.yaml` under the Sim section.

![image](https://user-images.githubusercontent.com/32557768/189253869-78c9585a-86ec-479f-8043-85bd5a9590ef.png)

To convert our spherical coordinates to cartesian coordinates, you can imagine zooming in on the tangent plane. We can find our latitude and longitude "distances" by subtracting our reference coordinates from our spherical coordinates, and then we just need to convert the units of those distances from degrees to meters. We are going to pretend the earth is a perfect sphere to make things simple. 

For north/south latitude distance, which will become our x coordinate, this is pretty simple. If the earth is a perfect sphere, then each meridian (vertical band) around the earth has the same circumference as the equator. Since we know the circumference of the earth at the equator (6371000 meters), we know that 1 degree of latitude is equal to the circumference divided by 360.

For east/west longitude distance, which will become our y coordinate, things get a little trickier because each horizontal band around the earth does not have the same circumference. To account for this, we have to scale the circumference with the cosine of latitude (think about why this makes sense). Then we can use the same equation as we used for latitude. Here are the equations you need to implement in code:

![image](https://user-images.githubusercontent.com/32557768/189777103-9f289c94-d0f9-4fa7-9432-8c47bb3a33c8.png)

This equation assumes that our reference heading is 0 degrees (North), but in the simulator, it is actually 90 degrees so you will need to rotate the point by 90 degrees. You should implement this conversion in the `spherical_to_cartesian` function. This is a static method, so it should be called with the class name instead of an object name: `Localization.spherical_to_cartesian()`. Read the function docstring for more details. Make sure to use numpy for the trig functions (`np.sin()`, `np.cos()`, etc) and make sure to convert between radians and degrees when necessary (check the ROS message definitions and function docstrings when in doubt - GPS coordinates are given to you in degrees, what do the trig functions from numpy (and python math) operate on?. For this simple starter project, we don't care about the Z coordinate, so just set it to zero. Once you are done, make sure to test your code by printing out your cartesian coordinates. You can do this by using the rospy.loginfo(msg) function.

You don't have to do any conversions for the IMU data, since it will give you an orientation quaternion, which is exactly what you need in order to make an [SE3](/autonomy/3d-poses-transforms-rotations).

### Update and publish pose to TF tree
At the end of each callback function, you will now have either [x,y,z] coordinates or an orientation quaternion. How do we give this data to the rest of the autonomy system? It's pretty simple, there are just a couple steps.

First, we have to update our classes global state. Inside of our Localization class, we can keep track of our rover's pose using the SE3 `self.pose`. For this beginner project, we will go with a simple update policy, where we want to update our global state every time we get any new sensor data. This means we want to update `self.pose` in both callback functions, and in each one we only want to change the data that we know. So in the GPS callback, you will need to set `self.pose` equal to a new SE3 using your new [x, y, z] position and the existing orientation data in `self.pose`. The GPS callback function is called whenever the localization node receives a message on the /gps topic that we subscribed to in the init function. Conversely, in the IMU callback, you need to set `self.pose` equal to a new SE3 with the same position data and your new orientation data.

Once you've done that, you can simply call the `publish_to_tf_tree()` method on your SE3. As the name implies, this will publish your pose to the TF tree. Since this is the pose of the base of the rover relative to the map, you should specify your parent frame as "map" and your child frame as "rover_base_link" (this is the name used for the base of the rover).


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

Here we will need to fill in the code so that the function returns an SE3 object that represents where the rover is in the world. For more information about SE3s and what they are checkout [this wikipage](/autonomy/3d-poses-transforms-rotations). Additionally, read over the docstrings within `mrover/starter_project/autonomy/src/util/SE3.py` to see the full SE3 interface and understand what the interface is. 

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

First of all, what is a state? Please read [this page](/navigation/overview) to learn what a state is.

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

## Testing your Work
Now open `starter_project.launch.py` which is in the `mrover/starter_project/autonomy/launch` folder. Make sure the lines declaring a node in the localization, perception, and navigation sections are uncommented.

To run the rover in the simulator:
1. In your terminal run `ros2 launch mrover starter_project.launch.py` to open the simulator, you should see the rover not moving in the simulator.
2. In a new terminal run `ros2 run mrover visualizer.py` so you can monitor which state the rover once you start it.
3. In the simulator press `p` to enable physics. Watch the rover move to (8, 2) while being in the DriveState, transition to TagSeekState, see the tag, and finally reach the tag and move to the DoneState.

<img width="194" alt="starter_project_states" src="https://github.com/umrover/mrover-ros/assets/101608185/b4d2a19b-1281-4907-b337-a5979898b4ee">

## Completion

Congrats! You've finished the autonomy starter project!


