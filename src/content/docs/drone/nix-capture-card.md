---
title: "NIX Video Capture Card"
---

Plugable HDMI Video Capture Card, USB 3.0 or USB C, Record, Stream and Go Live with DSLR, 1080P 60FPS with HDMI Pass Through - Compatible with Windows, Mac OS, Linux, OBS Streaming

https://www.amazon.com/dp/B07JLCJF96?maas=maas_adg_7D42C894F89C463AD3824DB1248FEE9F_afap_abs


## Getting started:

1. Plug HDMI cable into NIX capture card HDMI OUT port, Plug other end of HDMI cable into laptop HDMI port.
2. Plug FPV receiver into NIX capture card HDMI IN port.
3. Install dependencies
```
sudo apt update
sudo apt install v4l-utils
sudo apt install ffmpeg
```
3. Identify video port corresponding to camera (/dev/video0, /dev/video1, ...) and verify that this command pops up a window of the drone FPV feed: (change /dev/video4 to whatever your video port is.)
```
ffplay -f v4l2 -framerate 60 -video_size 1920x1080 /dev/video4
```



## Sample code for nodes using FPV feed

Publisher
```
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):

        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.cap = cv2.VideoCapture(4)
        self.bridge = CvBridge()
        self.get_logger().info('Camera Publisher Started')

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        else:
            self.get_logger().warn("Camera frame not read!")

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```
Subscriber
```
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            'image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info('Camera Subscriber Started')

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("Camera Feed", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```