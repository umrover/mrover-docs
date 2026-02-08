---
title: "Autonomy Starter Project - Perception"
---
## Implementation

### Creating a Custom Tag Message

One can think of an entire ROS project as a collection of nodes that talk to each other via named [topics](http://wiki.ros.org/Topics). Without any extra info, the data flowing between the nodes are just bytes. [Messages](http://wiki.ros.org/msg) identify how this data is structured (go ahead and read the linked Wiki page). ROS has a large amount of predefined message types that it uses, but it also gives you the ability to create your own message types. We want to make a custom message that tells Navigation where the ArUco tag is. Here is one possible solution:

```
int32 tag_id
float32 x_tag_center_pixel
float32 y_tag_center_pixel
float32 closeness_metric
```

Take a look at the file called `StarterProjectTag.msg` under the `msg` folder in the `starter_project` directory. There should be no other messages in there yet.

You may be asking now, how do I use this in C++? I just made some text file? CMake will *automatically* generate the C++ code for this message!

In `AutonomyStarterProject.cmake` take a look at:

```cmake
file(GLOB_RECURSE STARTER_PROJ_MESSAGE_PATHS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} CONFIGURE_DEPENDS 
        ${CMAKE_CURRENT_LIST_DIR}/msg/*.msg
)
```

which will tell CMake to generate all of the necessary files to be able to use each custom message in the `msg` directory.

Now run `./build.sh` in terminal to build the new message file.

### Some ROS Syntax

Take a look at the constructor `Perception::Perception`. For any of the functions below to work, we first need an image to process. This is where the subscriber comes in. A subscriber simply takes in a message of a specific type and directs the data to a callback function (`imageCallback` in our case) for processing. The [lambda](https://en.cppreference.com/w/cpp/language/lambda.html) syntax may look confusing, but all it's doing is taking in a message of a certain type and calling our callback function, passing the message as an argument.

Similarly, uncomment the line starting with `mTagPublisher`. After we're done processing an image, we need to hand it off to ROS so other nodes can use our newly processed data. A publisher takes care of this by publishing a message of a specific type to some topic. If the topic doesn't exist, ROS will create it for you!

### Detecting ArUco tags

Direct your attention to the `Perception::findTagsInImage` function. Our first task will be to extract the ArUco tags from the `image` parameter and place them into the `tags` output vector.

You will want to use the [`cv::aruco::detectMarkers`](https://docs.opencv.org/4.2.0/d5/dae/tutorial_aruco_detection.html) function for this. Read the hint to understand what parameters you need to pass.

Make sure to also fill in `Perception::getClosenessMetricFromTagCorners` and `Perception::getCenterFromTagCorners`. You should use these in the `Perception::findTagsInImage`.

Implementing these two functions will require some thought and we will not provide a way to do it. Discuss with your partners or others about how to solve both. `x_tag_center_pixel` and `y_tag_center_pixel` can be thought of as the center of the four corners of the tags, which you have access to via `std::vector<cv::Point2f>`. Note the types carefully! It is worth reading them in `perception.hpp`. For closeness metric, you only need an approximation. It will be used to drive towards the tag and stop within a distance. Consider how you as a human would estimate how far something is from you using your eyes. How would you approximate how far the vehicles in the image below are from you? How do you know they are different distances away? Be creative! However you choose to implement the closeness metric, make sure to scale the number between 0 and 1, as your navigation starter project will need a range between these two numbers.

![Cars on a road](https://upload.wikimedia.org/wikipedia/commons/9/9a/Depth_cues_1.png)

### Selecting the Closest Tag

Next you will want to select the tag from this vector that is closest to the camera. In other words, the tag with the highest closeness metric. Go ahead and fill in the `Perception::selectTag` function.

### Publishing the Tag

Now that we have our desired tag, it is time to publish it to the proper topic. Implement `Perception::publishTag`.

### Testing your Work

To test your tag detection algorithm run `ros2 launch mrover starter_project.launch.py` to open the simulator. Then run `ros2 topic echo /tag` to monitor the output of perception. Make sure your node (the code you wrote) is not crashing in the log output!

### Debugging

First comment out launching our node in the `starter_project.launch.py` file. You will instead be launching it from VSCode.

Now run `ros2 launch mrover starter_project.launch.py` in a terminal.

Then hit Ctrl-Shift-P and run `Cmake: Debug`. Select "Unspecified" if it asks for a kit. Select the starter_project_perception target to run (these settings are also on the bottom bar).

Make sure to set breakpoints in the source code files! They are almost always better than print statements.

## Extra

#### What is Camera Space?

![ArUco Tag Camera Space Example](https://user-images.githubusercontent.com/20666629/188225154-544900f7-f7c1-41b3-82fd-4d0ccfa1dcd9.png)

Consider the following image. Let's say it is 400x400 pixels. We can define a coordinate system that starts at the top left corner, consider that `(0,0)` with positive values of x extending right and positive values of y extending downward. The center of the tag would be about `(300,100)` in this space. Here is a diagram to aid understanding:

![Image Coordinate System](https://user-images.githubusercontent.com/20666629/191894080-e574e180-15bd-474a-aded-052bf541cdea.png)


