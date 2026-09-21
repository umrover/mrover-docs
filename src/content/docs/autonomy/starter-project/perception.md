---
title: "Perception"
sidebar:
  order: 3
---
# Overview

For the perception starter project, you will implement a ROS2 node that uses camera data to detect ArUco tags using [OpenCV](https://github.com/opencv/opencv), a popular computer vision library. Navigation will read this data in order to drive towards and align the rover with it. 

Example ArUco tags are pictured below. ArUco tags have an ID that corresponds to the pattern it contains.
![image](https://user-images.githubusercontent.com/20666629/172561442-05b84fd3-aab9-4d5b-88d1-87579985dcff.png)

The [ROS2 jazzy wiki](https://docs.ros.org/en/jazzy/index.html), software leads, and fellow members are a great resource if you are struggling with anything. Don't be afraid to ask questions; it's how we learn!

### Nodes, Topics, Publishers, and Subscribers
Before we start, let's clarify some unique ROS features and vocabulary. What are nodes and topics? What does "publish" or "subscribe" mean? First, let's talk about nodes. You may have written programming projects in the past that start in a `main()` function, and then run sequentially before exiting. In contrast, you can think of ROS2 projects as being a collection of independent processes constantly running at the same time. Each of the processes is called a "node", and they usually do not exit unless the user stops it.

<!-- insert pictures here -->

It usually isn't very useful to have a bunch of nodes that are unable to communicate with each other. This is where publishers, subscribers, and topics come in. Nodes can instantiate **publishers**, which send, or "publish", data to a named **topic**. Other nodes then might instantiate **subscribers**, which retrieve, or "subscribe", data from a named topic. The data being sent back and forth are called **messages**. Publishers and subscribers do not know about *who* is sending or retrieving the information, just the name of the topic it is reading from. Multiple publishers can publish to the same topic, and multiple subscribers can subscribe to the same topic. Additionally, nodes may have any amount of publishers/subscribers.

<!-- Useful picture of pub/sub here-->
![Publishers and Subscribers](https://github-production-user-asset-6210df.s3.amazonaws.com/113308723/656059215-545f19cb-f656-40da-aead-77ef643dceaa.jpg?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Credential=AKIAVCODYLSA53PQK4ZA%2F20260921%2Fus-east-1%2Fs3%2Faws4_request&X-Amz-Date=20260921T205745Z&X-Amz-Expires=300&X-Amz-Signature=eca66b24e1e126bae3d477456214f05f95dc01bfb091e1e28a84c196295d6f1a&X-Amz-SignedHeaders=host&response-content-type=image%2Fjpeg)

A helpful analogy might be an anonymous online forum. Unknown users (ROS nodes) may publish messages to a thread (topic) while other users can go to that thread and see (subscribe) that post. In this starter project, the Perception node subscribes to the `/zed/left/image` topic, which sends image frames from the [ZED stereo camera](https://www.stereolabs.com/products/zed-2) approximately 60 times per second. The Perception node also publishes to the `/tag` topic, which the Navigation node will subscribe to.

### Custom Messages

Different nodes often want to transmit different types and amounts of data. For example, a camera feed would be a continuous stream of massive RGB pixel matrices while a button sensor might publish a simple boolean state (pressed or not pressed). ROS enables this customization through [Messages](https://wiki.ros.org/msg) by allowing you to define your own message templates. Below, you can see the custom message type we have defined for you in `msg/StarterProjectTag.msg`. The name of the `.msg` file defines what the message type is called inside your C++ code.  

```
int32 tag_id            
float32 x_tag_center_pixel
float32 y_tag_center_pixel
float32 closeness_metric
```

## Perception Starter Project

### Inputs
- Image data in the form of `Image` messages published to the `/zed/left/image` topic

### Outputs
- One message published to `/tag` topic containing data about the closest detected ArUco tag:
    - The tag's ID
    - The x-coordinate of the center of the tag in the image
    - The y-coordinate of the center of the tag in the image
    - A closeness metric representing how far away the tag is

For this project, you will implement the functions in `perception.cpp` to identify the values of the above four variables, use them to construct a `StarterProjectTag`, and publish the message to the `/tag` topic for Navigation to read ("subscribe") from.

From the terminal, make sure you are in the correct repository by running `auton_starter` and then `build_starter` to build the message file.

## Code Overview
To complete the Perception starter project, you will implement 7 functions of the `Perception` class:
- the Perception() constructor
- imageCallback()
- findTagsInImage()
- getClosenessMetricFromTagCorners()
- getCenterFromTagCorners()
- selectTag()
- publishTag()

Each of these functions have corresponding headers in `perception.hpp` that might be helpful. You shouldn't have to create any other functions. 

It's a good idea to figure out what each function should do and how they interact with each other before you start writing code, so let's go over what each function does at a very high level. The `Perception()` constructor creates the actual ROS node and sets up any persistent state it might need. `imageCallback()` is the only function that gets called every time a new image frame is published to the `/zed/left/image` topic (hint: it must cause all of the other functions to be called). `findTagsInImage()` makes the OpenCV call to detect the ArUco tags. `getClosenessMetricFromTagCorners()` should calculate a closeness metric for the detected tag. `getCenterFromTagCorners()` identifies the x and y coordinates of the center of the detected tag. `selectTag()` picks the closest tag, and `publishTag()` publishes it.

Based on the names of the functions, the parameters/return variables for each function, and various comments, **identify:**
- The inputs and outputs of each function
- The order each function will be called in
- Which functions call each other

Talk with the people around you and the autonomy leads if you have any questions! One possible working example is shown below. The direction of the arrows indicates the movement of data, not which function is calling which.

![Example function call graph for perception starter project](https://github-production-user-asset-6210df.s3.amazonaws.com/113308723/656052102-aba7fd6a-0a78-4d58-969e-abab3e138e58.png?X-Amz-Algorithm=AWS4-HMAC-SHA256&X-Amz-Credential=AKIAVCODYLSA53PQK4ZA%2F20260921%2Fus-east-1%2Fs3%2Faws4_request&X-Amz-Date=20260921T204323Z&X-Amz-Expires=300&X-Amz-Signature=6d933b1fe055699ec4e5517a1ea6377f7599f41b51c7ea698266e0c6c874b5c6&X-Amz-SignedHeaders=host&response-content-type=image%2Fpng)


### The Perception() constructor
Let's take a look at the constructor `Perception::Perception` in `perception.cpp`. This constructor sets up the ROS node and is only run once. Once the constructor exits, any temporary information like local variables will be thrown away. Thus, we want any important work the constructor does to be assigned to class variables so that subsequent calls to class methods can benefit.

##### Setting the `/zed/left/image` Subscriber
Before we can figure out any high-level information like where the center of the detected tag is, we first need to get the actual camera frame. As detailed in the Inputs section, each frame from the camera is published to the topic `/zed/left/image`. The first thing the constructor does is read the camera frames using a subscriber. Take a look at the following code block:

```
mImageSubscriber = create_subscription<sensor_msgs::msg::Image>("/zed/left/image", 1, [this](sensor_msgs::msg::Image::ConstSharedPtr const& frame) {
    imageCallback(frame);
});
```

This code defines a subscriber that subscribes to the topic `/zed/left/image`. The [this] [lambda](https://en.cppreference.com/w/cpp/language/lambda.html) syntax may look confusing, but all it's doing is calling the `imageCallback` function, passing the camera frame as an argument. At a high level, when a new camera frame gets published to `/zed/left/image`, the subscriber calls `imageCallback` with the new camera frame. You can essentially think of this code ensuring that `imageCallback` gets called roughly 60 times per second every time a ZED frame is published. 

##### Setting the `/tag` Publisher
Now, uncomment the line starting with `mTagPublisher`. Once we're done processing an image, we need to publish it to a topic so that other nodes, such as the Navigation node, can use our newly processed data. This is the job of a publisher, which publishes a message of a specific type to some topic. Therefore, we define a publisher that publishes `StarterProjectTag`s to the topic `/tag`:

```
// TODO: uncomment me!
// mTagPublisher = create_publisher<msg::StarterProjectTag>("/tag", 1);
```

##### Setting the ArUco dictionary
Finally, we define the format of the tags we will be detecting by calling `cv::aruco::getPredefinedDictionary()`. The OpenCV library allows you to pick from various sets of valid markers. In this case we pick `DICT_4x4_50`, which means we guarantee that any ArUco tags we see will have 5x5 bit markers and an ID between 0-49. We'll discuss this topic further when we actually perform the detections.

For now, by uncommenting the publisher, you've finished implementing the first function! Lucky for you, the rest of the functions will be a slightly more complex and require a bit of a creativity :)

### imageCallback()

Let's move on to the `imageCallback()` function. From here on out, the details will be relatively sparse, since there is no "correct" solution or structure, so be creative! Consider what work this function should do and what functions it should call. In the example above, it called `findTagsInImage()`, `publishTag()`, and `selectTag()`.

##### Converting the Image
There are a few lines already in `imageCallback()` for you:

```
cv::Mat imageBGRA{static_cast<int>(imageMessage->height), static_cast<int>(imageMessage->width), 
              CV_8UC4, const_cast<uint8_t*>(imageMessage->data.data())};
cv::Mat image;

cv::cvtColor(imageBGRA, image, cv::COLOR_BGRA2BGR);
```

These three lines convert from BGRA to BGR (blue-green-red) by removing the alpha (transparency) channel, and store the resulting BGR image in the `image` variable. This is to help you, since you do not need `imageBGRA` for the starter project and should only use the `image` variable. 

### findTagsInImage()

As the name suggests, this function's goal is to extract all ArUco tags from the `image` parameter and place them into the `tags` output vector. To achieve this goal, consider what other functions you might want to call from this function. In the example diagram above, this function called `getCenterFromTagCorners()` and `getClosenessMetricFromTagCorners()`.

##### Detecting ArUco Tags with OpenCV
First, we want to detect any ArUco tags in the current camera frame. You will want to use the `cv::aruco::detectMarkers()` function for this. [This example](https://docs.opencv.org/4.2.0/d5/dae/tutorial_aruco_detection.html) from OpenCV should give you an idea for what parameters you need to pass. Try to write it yourself, and then check with the answer below.

<details>
  <summary>The correct call</summary>

  `cv::aruco::detectMarkers(image, mTagDictionary, mTagCorners, mTagIds);`
</details>

### getCenterFromTagCorners()

This function's goal is to find the xy-coordinates of the center between the four corners for each tag. There's a few different ways to implement this function, so we won't provide a strategy or design to do it. Discuss with your partners/people around you and leads! There are a few edge cases, but this function does not need to account for them. For this project, simple can be better than complicated.

##### Coordinates in Camera Space
The xy-coordinates are based off of (0,0) being the top left of the image. Both the corners populated by `detectMarkers()` and the coordinates expected by navigation in the output message follow this convention, so you don't need to worry about converting anything.

Consider the following representation of a 23x15 image. The coordinate system starts at the top left corner with positive values of x extending right and positive values of y extending downward. 
![Image Coordinate System](https://user-images.githubusercontent.com/20666629/191894080-e574e180-15bd-474a-aded-052bf541cdea.png)


Hint: once `cv::aruco::detectMarkers()` is called, `mTagCorners` and `mTagIds` should contain the xy-coordinates of the corners and IDs (0-49) of the tags. Each four-tuple of corners is ordered as follows: top-left, top-right, bottom-right, bottom-left. `mTagIds` and `mTagCorners` are indexed the same; that is, the corners in `mTagCorners[0]` represent the corners of the tag with ID `mTagIds[0]`.

### getClosenessMetricFromTagCorners()

It's important to have some kind of closeness metric, since it identifies how far away a tag is (and if we're moving, whether or not we're getting closer to it). Again, there are a few different ways to implement this function, so we won't tell you how to do it. Be creative! 

For a closeness metric, you only need a rough estimate. It will be used to drive towards the tag and stop within a distance. Consider how you as a human would estimate how far something is from you. How would you approximate how far the vehicles in the image below are from you? How do you know they are different distances away? Be creative! 

![Cars on a road](https://upload.wikimedia.org/wikipedia/commons/9/9a/Depth_cues_1.png)

However you choose to implement the closeness metric, make sure to scale the number between 0 and 1 (where 0 is very close and 1 is far away), as the navigation starter project expects a range between these two numbers. 

### selectTag()

We want to select the closest tag from the vector `findTagsInImage()` populates, since it makes sense to drive towards closer objects compared to farther ones. In other words, we want to pick the tag with the highest closeness metric. 
If there are no valid ArUco tags in frame, you should publish a "dummy" tag with an ID of -1.

### publishTag()

Now that we have our desired tag, it is time to publish it to the proper topic. Implement `Perception::publishTag`. If you are unsure of the syntax, this [example](https://ros2course.readthedocs.io/en/latest/Writing%20publisher%20and%20subscriber%20nodes.%20C++.html) or a quick Google search will help you out. Check the answer once you think you've got it.

<details>
  <summary>The correct call</summary>

  `mTagPublisher->publish(tag);`
</details>

## Putting It All Together

To test, run `ros2 launch mrover_autonomy_starter starter_project.launch.py` to open the simulator. In another terminal window, run `ros2 topic echo /tag` to monitor the output of perception. Don't forget to `auton_starter` to set up each terminal window. Make sure your node (the code you wrote) is not crashing in the log output! `ros2 topic echo /tag` should at least be continuously publishing invalid tags with ID of -1.

You can drive the rover around by pressing `p` to enable physics, and then move with `i`,`j`,`l`, and `,`. In the RViz window, press "Add" in the bottom left, then click the "by topic" tab, scrolling down and selecting the "/zed", "/left", "/image", "Image" option, and click "OK". This should open a small window in the bottom left displaying what the ZED camera sees inside the simulator. If you navigate the rover such that you can see the ArUco tag in the camera feed, but `ros2 topic echo /tag` isn't publishing a valid (not -1) tag, you might have some errors in your code.

Otherwise, congratulations! You have successfully completed the perception starter project!

### Debugging with VS Code

As a disclaimer, it can be difficult to use a debugger with MRover software, and various tools like `ros2 topic echo` and print statements are usually more helpful. However, it is possible to use VS Code to debug the starter project. First, comment out `perception_node` from the `starter_project.launch.py` file. Now, when the starter project is launched, it won't launch the perception node. You will instead be separately launching it from VS Code.

Now run `ros2 launch mrover_autonomy_starter starter_project.launch.py` in a terminal to launch the simulator and other two nodes.

In VS Code, hit Ctrl-Shift-P and run `Cmake: Debug`. Select "Unspecified" if it asks for a kit. Select the starter_perception target to run (these settings are also on the bottom bar).

Make sure to set breakpoints in the source code files! They can provide useful information that print statements can't.