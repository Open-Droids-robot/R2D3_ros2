#!/usr/bin/env python3

# // Copyright RealMan
# // License(BSD/GPL/...)
# // Author: Lamont
# // The overall demo completes the acquisition of three-dimensional coordinates of the center pixel point of the image (640*480).First, subscribe to image topics and camera parameter topics through ROS, obtain image data and camera internal references, align the depth image and RGB image through camera internal references, obtain a reliable depth of the center pixel point, and then calculate the three-dimensional coordinates based on the internal references and depth, and visualize the depth image and print the coordinates.
import cv2
import pyrealsense2 as rs2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo

 
class pic_pix:
    '''
    Pixel coordinate selection
    '''
    def __init__(self):
        self.pix_x = 320
        self.pix_y = 240
 
class NodeSubscribe(Node):
    '''
    Subscribe to the camera image topic and camera parameter topic, align the depth image and RGB image through the camera's internal reference to obtain a reliable depth of the center pixel point, and then calculate the three-dimensional coordinates based on the internal reference and depth, and visualize the depth image and print the coordinates.
    '''
    def __init__(self,name):
        super().__init__(name)
        self.bridge = CvBridge()
        self.get_logger().info("This is RM's D435 camera demo. Use the depth alignment image to obtain the coordinate value of the image center point (640*480), press ctrl+c to exit the current demo" )
        self.sub = self.create_subscription(msg_Image, '/camera/depth/image_rect_raw', self.imageDepthCallback, 10)            # Receive image topic data
        self.sub_info = self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.imageDepthInfoCallback, 10)     # Receive camera parameter topic data
        self.intrinsics = None
        self.pix_grade = None
        self.pix = pic_pix()
    def imageDepthCallback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)                                            # Image format conversion, ROS topic format conversion to CV format
            print('******************************')
            if self.intrinsics:
                depth = cv_image[self.pix.pix_y, self.pix.pix_x]
                result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [self.pix.pix_x, self.pix.pix_y], depth)    # Calculate the three-dimensional coordinates of pixel points
                print('Image center point coordinate value (mm):',result)
            cv2.imshow("aligned_depth_to_color_frame" , cv_image)
            key = cv2.waitKey(1)

        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            return

    def imageDepthInfoCallback(self, cameraInfo):
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.k[2]
            self.intrinsics.ppy = cameraInfo.k[5]
            self.intrinsics.fx = cameraInfo.k[0]
            self.intrinsics.fy = cameraInfo.k[4]
            if cameraInfo.distortion_model == 'plumb_bob':
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == 'equidistant':
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.d]
        except CvBridgeError as e:
            print(e)
            return
 
def main(args=None):
    rclpy.init()
    # 建立一个节点(Center_Coordinate_node)用来接受图像中心深度值数据
    node = NodeSubscribe("Center_Coordinate_node")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
