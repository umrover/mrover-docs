---
title: "ROS Tools: rqt_bag"
---
rqt_bag is a ROS tool used to play back bag files. To record a bag file, use the command `rosbag record -o bag_name /topics /to /record`. The generated bag file now contains data from the various recorded rostopics which can be subscribed to when the bag file is played back. To play back a bag file, in the directory containing the bag, run:

`rqt_bag bag_name.bag`

This should open a window:

![Screenshot from 2022-11-16 22-06-36](https://user-images.githubusercontent.com/51866496/202348379-ae5f6aab-cde0-4a42-bec7-d2e312b58c47.png)

To play back topic(s), right-click on the panel's empty space and select Publish > Publish All:

![Screenshot from 2022-11-16 22-38-50](https://user-images.githubusercontent.com/51866496/202349210-55cce7e9-2ead-4fcd-93d3-a4b379459db9.png)

You can also publish individual topics by right-clicking a specific topic and selecting Publish. After selecting topics to be published, hit the play button in the top right. If you `rostopic list`, you should now see the topics you are publishing and can view their content in rviz, subscribe to them, etc.

For more information on rqt_bag and its other uses, refer to the [ROS wiki page](http://wiki.ros.org/rqt_bag).