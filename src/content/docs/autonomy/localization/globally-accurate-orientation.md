---
title: "In Search of Globally Accurate Orientation"
---
# What's the deal?
Our IMU, like anyone, has its strengths & weaknesses. It's very good at being locally accurate - i.e. over short periods of time - the readings that the IMU spits out are splendid. However, over longer time periods, our readings have a tendency to drift. This has the unfortunate consequence of causing the rover's performance to slowly worsen, the longer that the rover goes without having its IMU re-calibrated. This wiki page details our attempts to correct this.

To accomplish this, we want to fuse the IMU's sensors' data ourselves, off the IMU. By doing this, we can manually filter the IMU's sensor data using several algorithms that are commonly used in industry, run a test on the rover, & see if any of these algorithms will provide better performance than the IMU's onboard processing.

# The nitty gritty deets... 🕵️
I've been told that the most commonly used algorithms in industry are [Madgwick](https://x-io.co.uk/open-source-imu-and-ahrs-algorithms), [Mahony](https://hal.archives-ouvertes.fr/hal-00488376/document), & the [Extended Kalman Filter](https://en.wikipedia.org/wiki/Extended_Kalman_filter). Conveniently, the Python package [AHRS](https://ahrs.readthedocs.io/en/latest/index.html) has all three algorithms implemented, & should allow us to get all three algorithms up & running relatively quickly so we can test them on the rover.

## Maybe Madgwick?
_find the code in [ss/test-ahrs-for-imu-drift](https://github.com/umrover/mrover-ros/tree/ss/test-ahrs-for-imu-drift)!_

The Madgwick implementation in AHRS allows us to feed it IMU readings as we receive them, & will spit out a _corrected_ orientation vector with each sample. The code leverages this by subscribing to the topics where the IMU is publishing its data (in the simulator, that's `imu/data` for orientation, angular velocity, & linear acceleration, & `imu/mag` for magnetometer data). In the callback function for the `imu/data` topic, we're giving these values to our Madgwick filter, & the output for this is the _corrected_ orientation vector.

### Testing In Sim
Currently, the raw & corrected orientation vectors are being written to a csv file. I'm using this to do a sanity check on my code, & also for logging purposes. Additionally, I'm taking these corrected orientation vectors & visualizing them in RViz, which lets me watch the filter in action while the rover is completing a course. Some more specific details about getting this code running - 
1. To get gazebo to publish simulated magnetometer data, I'm using [GazeboRosMagnetic](http://wiki.ros.org/hector_gazebo_plugins) from hector_gazebo_plugins. This is set up in the codebase in the `rover_gazebo_plugins.xacro` file.
2. To run the testing code, first do `roslaunch mrover auton_sim.launch` to get the simulator running. Then, you can do `python3 src/esw/test_ahrs.py`. You can also run `rosrun mrover debug_course_publisher.py` to get the rover to complete a course & be able to watch the filter in action.
3. The corrected orientation vectors are visualized in RViz - it should be a large, chonky, neon green arrow. This is accomplished by setting up a **Orientation Marker** visualization in `auton_sim.rviz`. This tells RViz to expect messages describing Markers in the `visualization_marker` topic, & how to visualize them.

