---
title: "Object Detection"
---
**Context**: As of this year, [URC Rules](https://urc.marssociety.org/home/requirements-guidelines) specify two new waypoints during the Autonomy mission that require the Rover to detect and navigate toward two objects placed on the ground.

> The 2 objects will have GNSS coordinates within the vicinity of the objects (<10 m). Autonomous detection of the tools will be required. The first object will be an orange rubber mallet. The second object will be a standard 1 L wide-mouthed plastic water bottle of unspecified color/markings (approximately 21.5 cm tall by 9 cm diameter).

Currently, the perception system does not support detection of objects besides ARTags so we must experiment with and implement such a detection system. One way to do this is using a learning-based instance segmentation model such as [YOLO](https://github.com/opencv/opencv/blob/4.x/samples/dnn/object_detection.cpp) to extract the mallet and waterbottle from the ZED camera feed.

<hr>

**Interface**: (Subject to change)

Node: `detect_objects`

Subscribes: [sensor_msgs/Image](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Image.html)

Publishes: 

`Object.msg`

```
string object_type
float32 detection_confidence
float32 distance
float32 bearing
```
Object pose to tf tree in map frame

Rough Steps:
1. Run a pre-trained object detection model outside of ROS to confirm performance
2. Create a subscriber to the ZED point cloud topic
3. Write a function that takes the point cloud, compresses it into a 2D cv::Mat, and passes it into the model to detect the objects (see tag_detector.processing.cpp for an example of this)
4. Find the point that corresponds to the center of the object bounding box in the Point Cloud and assign that as the Object's distance from the rover
5. Determine the relative bearing of the Object from the rover's current position. This can be done similar to the tag bearing calculation in the [long range detection project](/perception/long-range-tag-detection).
5. Create a publisher for the Object topic
6. Write a function that publishes the detected Objects from the Point Cloud message to the Object topic as well as the tf tree
7. Determine a more robust way of detecting the Object's distance from ZED. Picking the center point of the bounding box could often be NaN or located somewhere in open space
8. Revisit the image segmentation model and fine-tune with our own images if necessary

See: https://github.com/umrover/mrover-ros/tree/percep/obj-detect/src/perception/object_detector