#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('realsense_image_subscriber1')
        self.bridge = CvBridge()

        self.color_subscription = self.create_subscription(
            Image,
            'camera/color/image_raw1',
            self.color_callback,
            10)
        self.color_subscription 

        self.depth_subscription = self.create_subscription(
            Image,
            'camera/depth/image_raw1',
            self.depth_callback,
            10)
        self.depth_subscription  

    def color_callback(self, msg):
        self.get_logger().info('Received color image')
        color_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow("Color Image", color_image)
        cv2.waitKey(1)

    def depth_callback(self, msg):
        self.get_logger().info('Received depth image')
        depth_image = self.bridge.imgmsg_to_cv2(msg, 'mono16')
        cv2.imshow("Depth Image", depth_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
