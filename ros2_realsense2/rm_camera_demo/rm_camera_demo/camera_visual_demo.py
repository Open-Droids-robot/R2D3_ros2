#!/usr/bin/env python3

# // Copyright RealMan
# // License(BSD/GPL/...)
# // Author: Lamont
# // 整体demo完成对相机RGB图像和深度图像话题订阅，并通过OPENCV可视化图像信息。

import rclpy
import cv2
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


bridge = CvBridge()
 
 
class NodeSubscribe(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("这是RM的D435相机demo，利用opencv展示摄像头RGB和深度图像，按下ctrl+c退出当前demo" )
        # 接收话题图像数据，并可视化
        self.create_subscription(Image,'/camera/color/image_raw', self.color_callback, 10)
        self.create_subscription(Image,'/camera/depth/image_rect_raw', self.depth_callback, 10)
 
    def color_callback(self,data):
        cv_img = bridge.imgmsg_to_cv2(data, "bgr16")
        cv2.imshow("color_frame" , cv_img)
        cv2.waitKey(1)
        
    def depth_callback(self,data):
        cv_img = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        cv2.imshow("color_depth_frame" , cv_img)
        cv2.waitKey(1)
 
 
def main(args=None):
    rclpy.init()
    # 建立一个节点(sub_image_node)用来接受以下两个话题中的图像数据
    node = NodeSubscribe("sub_image_node")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
