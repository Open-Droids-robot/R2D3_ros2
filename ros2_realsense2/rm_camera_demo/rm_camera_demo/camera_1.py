#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import subprocess

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_1_node')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw1', 10)
        self.timer = self.create_timer(1/30, self.timer_callback)  # 30Hz
        
        # 列出所有USB摄像头并选择一个摄像头
        camera_devices = self.list_usb_cameras()
        if not camera_devices:
            self.get_logger().error("No USB cameras found.")
            rclpy.shutdown()
            return

        self.get_logger().info("Available USB cameras:")
        for idx, (name, paths) in enumerate(camera_devices):
            self.get_logger().info(f"{idx}: {name} - {', '.join(paths)}")

        selected_camera_paths = camera_devices[1][1]
        self.get_logger().info(f"Selected camera paths: {selected_camera_paths}")
        
        self.cap = cv2.VideoCapture(selected_camera_paths[0])  # 打开指定摄像头
        # 设置摄像头的分辨率和帧率 
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640) # 宽度 
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480) # 高度 
        self.cap.set(cv2.CAP_PROP_FPS, 30) # 帧率
        self.bridge = CvBridge()
        
    def list_usb_cameras(self):
        result = subprocess.run(['v4l2-ctl', '--list-devices'], stdout=subprocess.PIPE, text=True)
        devices = result.stdout.strip().split('\n\n')
        
        camera_devices = []
        for device in devices:
            lines = device.split('\n')
            device_name = lines[0]
            if 'RealSense' in device_name or 'Intel' in device_name:
                continue
            device_paths = [line.strip() for line in lines[1:] if '/dev/video' in line]
            if device_paths:
                camera_devices.append((device_name, device_paths))
        return camera_devices

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        else:
            self.get_logger().error('Failed to capture image')

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
