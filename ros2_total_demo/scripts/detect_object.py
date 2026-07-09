#!/usr/bin/env python3
# -*- coding=UTF-8 -*-

"""
YOLOv8读取模型 显示识别出来物体的位置和图像
"""

import cv2
import pyrealsense2 as rs
import time
import os
import math
import numpy as np
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from ros2_total_demo.msg import ObjectPose

class ModelDetect(Node):
    def __init__(self):
        super().__init__('model_detect')
        self.publisher_ = self.create_publisher(ObjectPose, 'object_pose_bottle', 10)
        self.object_info_msg = ObjectPose()
        
        # 获取当前文件的绝对路径
        current_file_path = os.path.abspath(__file__)
        # 获取当前文件所在目录的路径
        current_dir = os.path.dirname(current_file_path)
        # 构建 weight 文件夹内 best.pt 文件的绝对路径
        best_pt_path = os.path.join(current_dir, 'weight', 'best.pt')
        
        self.get_logger().info(f"best_pt_path: {best_pt_path}")

        # 加载 YOLOv8 模型
        self.model = YOLO(best_pt_path)

        # 配置 RealSense
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # 获取所有连接的设备列表
        devices = [dev for dev in rs.context().devices]
        ################################################################################
        # 打印所有设备的serial_number
        for i, dev in enumerate(devices):
            print(f"Device {i}: {dev.get_info(rs.camera_info.serial_number)}")
            
        serial_number = devices[0].get_info(rs.camera_info.serial_number)
        # serial_number = '150622072996'
        self.config.enable_device(serial_number)
        ################################################################################

        # 启用指定的设备
        self.config.enable_device(devices[0].get_info(rs.camera_info.serial_number))
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        
        # 启动相机流
        self.pipeline.start(self.config)
        self.align_to = rs.stream.color  # 与color流对齐
        self.align = rs.align(self.align_to)
        
        # 创建定时器
        self.timer = self.create_timer(1/30, self.timer_callback)

    def get_aligned_images(self):
        frames = self.pipeline.wait_for_frames()  # 等待获取图像帧
        aligned_frames = self.align.process(frames)  # 获取对齐帧
        aligned_depth_frame = aligned_frames.get_depth_frame()  # 获取对齐帧中的depth帧
        color_frame = aligned_frames.get_color_frame()  # 获取对齐帧中的color帧

        # 相机参数的获取
        intr = color_frame.profile.as_video_stream_profile().intrinsics  # 获取相机内参
        depth_intrin = aligned_depth_frame.profile.as_video_stream_profile().intrinsics  # 获取深度参数（像素坐标系转相机坐标系会用到）

        depth_image = np.asanyarray(aligned_depth_frame.get_data())  # 深度图（默认16位）
        depth_image_8bit = cv2.convertScaleAbs(depth_image, alpha=0.03)  # 深度图（8位）
        color_image = np.asanyarray(color_frame.get_data())  # RGB图

        return intr, depth_intrin, color_image, depth_image, aligned_depth_frame

    def get_3d_camera_coordinate(self, depth_pixel, aligned_depth_frame, depth_intrin):
        x = depth_pixel[0]
        y = depth_pixel[1]
        dis = aligned_depth_frame.get_distance(x, y)  # 获取该像素点对应的深度
        camera_coordinate = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, dis)
        return dis, camera_coordinate

    def timer_callback(self):
        # 等待获取一对连续的帧：深度和颜色
        intr, depth_intrin, color_image, depth_image, aligned_depth_frame = self.get_aligned_images()

        if not depth_image.any() or not color_image.any():
            return

        # 使用 YOLOv8 进行目标检测
        results = self.model.predict(color_image, conf=0.7)
        annotated_frame = results[0].plot()
        detected_boxes = results[0].boxes.xyxy  # 获取边界框坐标
        detected_classes = results[0].names  # 获取检测到的类别名称

        for i, box in enumerate(detected_boxes):
            x1, y1, x2, y2 = map(int, box)  # 获取边界框坐标
            class_name = detected_classes.get(results[0].boxes.cls[i])

            # 计算步长
            xrange = max(1, math.ceil(abs((x1 - x2) / 30)))
            yrange = max(1, math.ceil(abs((y1 - y2) / 30)))
            print(f"class_name:{class_name}--detected_boxes:{detected_boxes}")

            # 显示中心点坐标
            ux = int((x1 + x2) / 2)
            uy = int((y1 + y2) / 2)
            dis, camera_coordinate = self.get_3d_camera_coordinate([ux, uy], aligned_depth_frame, depth_intrin)  # 获取对应像素点的三维坐标

            # 组织信息
            self.object_info_msg.x = camera_coordinate[0]
            self.object_info_msg.y = camera_coordinate[1]
            self.object_info_msg.z = camera_coordinate[2]
            self.publisher_.publish(self.object_info_msg)

            formatted_camera_coordinate = f"({camera_coordinate[0]:.2f}, {camera_coordinate[1]:.2f}, {camera_coordinate[2]:.2f})"
            #print(f"formatted_camera_coordinate:{formatted_camera_coordinate}")
            cv2.circle(annotated_frame, (ux, uy), 4, (255, 255, 255), 5)  # 标出中心点
            cv2.putText(annotated_frame, formatted_camera_coordinate, (ux + 20, uy + 10), 0, 1, [225, 255, 255], thickness=1, lineType=cv2.LINE_AA)  # 标出坐标

        cv2.imshow('YOLOv8 RealSense', annotated_frame)
        key = cv2.waitKey(1)

        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ModelDetect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
