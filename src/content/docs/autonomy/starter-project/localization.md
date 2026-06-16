---
title: "Localization"
sidebar:
  order: 2
---
# Localization

Make sure you read the "GPS", "IMU", "GPS Linearization", and "Guide to Localization Frames" sections from the [Localization page](/autonomy/localization/overview) before working on this part of the project (RTK is beyond the scope of our starter project but recommended).

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

You don't have to do any conversions for the IMU data, since it will give you an orientation quaternion, which is exactly what you need in order to make an [SE3](/autonomy/resources/3d-poses-transforms-rotations).

### Update and publish pose to TF tree
At the end of each callback function, you will now have either [x,y,z] coordinates or an orientation quaternion. How do we give this data to the rest of the autonomy system? It's pretty simple, there are just a couple steps.

First, we have to update our classes global state. Inside of our Localization class, we can keep track of our rover's pose using the SE3 `self.pose`. For this beginner project, we will go with a simple update policy, where we want to update our global state every time we get any new sensor data. This means we want to update `self.pose` in both callback functions, and in each one we only want to change the data that we know. So in the GPS callback, you will need to set `self.pose` equal to a new SE3 using your new [x, y, z] position and the existing orientation data in `self.pose`. The GPS callback function is called whenever the localization node receives a message on the /gps topic that we subscribed to in the init function. Conversely, in the IMU callback, you need to set `self.pose` equal to a new SE3 with the same position data and your new orientation data.

Once you've done that, you can simply call the `publish_to_tf_tree()` method on your SE3. As the name implies, this will publish your pose to the TF tree. Since this is the pose of the base of the rover relative to the map, you should specify your parent frame as "map" and your child frame as "rover_base_link" (this is the name used for the base of the rover).
