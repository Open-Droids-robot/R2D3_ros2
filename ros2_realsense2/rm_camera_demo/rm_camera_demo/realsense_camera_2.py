import pyrealsense2 as rs
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class RealSensePublisher(Node):
    def __init__(self, serial_number, width, height, fps):
        super().__init__('realsense_publisher2')
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_device(serial_number)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.pipeline.start(self.config)
        self.bridge = CvBridge()
        self.color_publisher = self.create_publisher(Image, 'camera/color/image_raw2', 10)
        self.depth_publisher = self.create_publisher(Image, 'camera/depth/image_raw2', 10)
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
    
    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return

        # 发布彩色图像
        color_image = np.asanyarray(color_frame.get_data())
        color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        self.color_publisher.publish(color_msg)

        # 发布深度图像
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='mono16')
        self.depth_publisher.publish(depth_msg)

        # 显示彩色图像
        #cv2.imshow('RealSense Color', color_image)
        #if cv2.waitKey(1) == ord('q'):
        #    rclpy.shutdown()

def list_realsense_devices():
    """列出所有连接的RealSense设备"""
    ctx = rs.context()
    devices = ctx.query_devices()
    device_list = []
    for i, dev in enumerate(devices):
        device_info = {}
        device_info['name'] = dev.get_info(rs.camera_info.name)
        device_info['serial'] = dev.get_info(rs.camera_info.serial_number)
        device_list.append(device_info)
        print(f"Device {i}: {device_info['name']} (Serial: {device_info['serial']})")
    return device_list

def main(args=None):
    rclpy.init(args=args)
    devices = list_realsense_devices()

    if not devices:
        print("No RealSense devices found.")
        return

    # 自动选择第一个设备
    selected_serial = devices[2]['serial']
    print(f"Automatically selected device with serial number: {selected_serial}")

    # 设置分辨率和帧率
    width = 640
    height = 480
    fps = 15

    # 启动节点
    node = RealSensePublisher(selected_serial, width, height, fps)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
