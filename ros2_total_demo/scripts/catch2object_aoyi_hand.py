#!/usr/bin/python3
# -*- coding: utf-8 -*-
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R

import socket
import time
from rclpy.time import Time
from rclpy.duration import Duration

from std_msgs.msg import String, Empty, Bool
from rm_ros_interfaces.msg import Liftheight,Movejp,Armstate,Armoriginalstate
from ros2_total_demo.msg import ObjectPose
from threading import Thread, Event

class CatchNode(Node):
    def __init__(self):
        super().__init__('catch')
        
        ip = '169.254.128.19'
        port_no = 8080
    
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((ip, port_no))
        
        # 初始化遨意灵巧手
        self.get_power_ready()
        
        self.ready_pose = [-0.3449670076370239, 0.0190420001745224, 0.013027999550104141, -0.2925779360661274, -0.6215936373866948,  0.675687305208183, -0.2673315672870813] #机械臂拍照位姿
        self.transition_pose = [-0.3549630045890808, -0.0012580000329762697, 0.22927699983119965, 0.3167968185090858, 0.6104470831856154, -0.6516798538218731, 0.3198554400590959] #机械臂过渡位姿
        
        self.speed = 10
        
        #抓取时的姿态（四元数）
        self.grasping_pose = [0.5687317123, 0.43448958209489674, -0.3871068, 0.581301384107]
        
        # 发布一个改变工作坐标系的消息
        self.change_work_frame_pub = self.create_publisher(String, '/right_arm_controller/rm_driver/change_work_frame_cmd', 10)
        # 发布一个控制升降机的消息
        self.set_lift_height_pub = self.create_publisher(Liftheight, '/left_arm_controller/rm_driver/set_lift_height_cmd', 10)  
        # 发布一个控制机械臂movej_p运动的消息
        self.pub_to_pose = self.create_publisher(Movejp, '/right_arm_controller/rm_driver/movej_p_cmd', 10) 
        # 发布一个请求机械臂当前状态的消息
        self.pub_get_pose = self.create_publisher(Empty, '/right_arm_controller/rm_driver/get_current_arm_state_cmd', 10)  
        # 订阅机械臂movej_p运动规划是否成功
        self.movej_p_plan_result_sub = self.create_subscription(Bool,'/right_arm_controller/rm_driver/movej_p_result', self.movej_p_callback,10)
        # 订阅机械臂当前状态-返回各关节弧度和四元数
        self.armstate_sub = self.create_subscription(Armstate,'/right_arm_controller/rm_driver/get_current_arm_state_result', self.get_arm_state_callback2, 10)  
        # 获取机械臂当前状态-返回各关节角度和欧拉角
        self.joint_positi_sub = self.create_subscription(Armoriginalstate,'/right_arm_controller/rm_driver/get_current_arm_original_state_result',self.joint_position_callback,10)  
        # 订阅视觉识别物体位置的消息
        self.object_pose_subscriber = self.create_subscription( ObjectPose, 'object_pose_bottle', self.object_pose_callback, 10) 
        
        self.object_pose = None
        # 轨迹规划状态
        self.run_state = False
        # 储存位置、四元数信息
        self.curret_pose2 = None
        # 位姿
        self.curret_pose3 = None 
        
        self.change_work_frame()
        
        self.data_ready_event = Event() 
        self.worker_thread = Thread(target=self.worker_function) 
        self.worker_thread.start()
    
    def change_work_frame(self):
        # 机械臂切换到base坐标系
        # 创建消息 
        msg = String() 
        msg.data = 'Base' 
        # 发布消息 
        self.change_work_frame_pub.publish(msg)
        #print('机械臂切换到base坐标系')
        
    # 设置升降机高度 目标高度，单位 mm，范围：0~2600 #速度百分比，1~100  block 是否阻塞   
    def set_lift_height(self, height=100, speed=20,block=True):
        # 创建消息 
        msg = Liftheight() 
        msg.height = height
        msg.speed = speed
        msg.block = block
        # 发布消息 
        self.set_lift_height_pub.publish(msg)
        print(msg)
        #print(f'设置升降机高度，目标高度：{height} mm，速度百分比：{speed}%')
        
    def send_cmd(self, cmd_6axis=''):

        self.client.send(cmd_6axis.encode('utf-8'))
    
        return True
    
    # 傲意灵巧手modbus初始化
    def get_power_ready(self):
    
        point6_00 = '{"command":"set_modbus_mode","port":1,"baudrate":115200,"timeout ":2}\r\n'
        _ = self.send_cmd(cmd_6axis=point6_00)
        time.sleep(2)
        #self.get_logger().info("配置通讯端口通ModbusRTU")
    
    def open_hand(self):
        # 傲意灵巧手打开
        point6_00 = '{"command":"write_registers","port":1,"address":1135,"num":6,"data":[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 0],"device":2}\r\n'
        _ = self.send_cmd(cmd_6axis=point6_00)
        time.sleep(1)
        #self.get_logger().info(_)
        #self.get_logger().info("傲意灵巧手打开")
    
    def catch_dumb(self):
        
        # 傲意灵巧手握住瓶子
        point6_00 = '{"command":"write_registers","port":1,"address":1135,"num":6,"data":[255,255,255,255,255,255,255,255,255,255,0,0],"device":2}\r\n'
        _ = self.send_cmd(cmd_6axis=point6_00)
        time.sleep(1)
        #self.get_logger().info(_) 
        #self.get_logger().info("傲意灵巧手抓东西")
        
    # 机械臂拍照位置
    def ready_catch(self):
        
        # 机械臂拍照识别位置 
        moveJ_P_ReadyPose = Movejp()
        moveJ_P_ReadyPose.pose.position.x = self.ready_pose[0]
        moveJ_P_ReadyPose.pose.position.y = self.ready_pose[1]
        moveJ_P_ReadyPose.pose.position.z = self.ready_pose[2]
        moveJ_P_ReadyPose.pose.orientation.x = self.ready_pose[3]
        moveJ_P_ReadyPose.pose.orientation.y = self.ready_pose[4]
        moveJ_P_ReadyPose.pose.orientation.z = self.ready_pose[5]
        moveJ_P_ReadyPose.pose.orientation.w = self.ready_pose[6]
        moveJ_P_ReadyPose.speed = self.speed
        moveJ_P_ReadyPose.block = True # 是否阻塞
        
        self.pub_to_pose.publish(moveJ_P_ReadyPose)
                
    def object_pose_callback(self, msg): 
        self.object_pose = msg 
        self.check_data_ready()
              
    def get_arm_state_callback2(self,msg):

        # 返回各关节弧度和四元数
        self.curret_pose2 = msg
        #print(self.curret_pose2)
        #print('self.curret_pose2')
        self.check_data_ready()
        
    def joint_position_callback(self,msg):
        
        # 返回各关节角度和欧拉角
        self.curret_pose3 = msg
        #print(self.curret_pose3)
        #print('self.curret_pose3')
        self.check_data_ready()
        
    def movej_p_callback(self,msg):
        #当订阅的机械臂执行状态消息到达时，会调用此回调函数
        self.run_state=msg.data
        self.check_data_ready()
               
    def get_pose_pub(self): 
        msg = Empty() 
        self.pub_get_pose.publish(msg) 
        
    def convert(self,x, y, z, x1, y1, z1, rx, ry, rz):
    
        
        #使用深度相机识别到的物体坐标（x, y, z）和机械臂末端的位姿（x1,y1,z1,rx,ry,rz）来计算物体在机械臂基坐标下的位置（x, y, z）
        
        #Args:
        #    x : 相机坐标系下物体位置x
        #    y : 相机坐标系下物体位置y
        #    z : 相机坐标系下物体位置z
        #    x1 : 机械臂末端位姿 x
        #    y1 : 机械臂末端位姿 y
        #    z1 : 机械臂末端位姿 z
        #    rx : 机械臂末端位姿 rx
        #    ry : 机械臂末端位姿 ry
        #    rz : 机械臂末端位姿 rz
    
        #Returns:
        #    _type_: 物体在机械臂基坐标系下的位置 x y z
    
        # 相机坐标系到机械臂末端坐标系的旋转矩阵和平移向量
        rotation_matrix = np.array([[0.0790325, 0.98958949, -0.12027678],[-0.99682423,0.07726969, -0.0192575],[-0.00976327, 0.12141678, 0.9925536]])
        
        translation_vector = np.array([-0.0974,0.041261, 0.00108])
    
        obj_camera_coordinates = np.array([x, y, z])     # 深度相机识别物体返回的坐标
    
        end_effector_pose = np.array([x1, y1, z1,
                                      rx, ry, rz])     # 机械臂末端的位姿，单位为弧度
    
        # 将旋转矩阵和平移向量转换为齐次变换矩阵
        T_camera_to_end_effector = np.eye(4)
        T_camera_to_end_effector[:3, :3] = rotation_matrix
        T_camera_to_end_effector[:3, 3] = translation_vector
        # 机械臂末端的位姿转换为齐次变换矩阵
        position = end_effector_pose[:3]
        orientation = R.from_euler('xyz', end_effector_pose[3:], degrees=False).as_matrix()
        T_base_to_end_effector = np.eye(4)
        T_base_to_end_effector[:3, :3] = orientation
        T_base_to_end_effector[:3, 3] = position
        # 计算物体相对于机械臂基座的位姿
        obj_camera_coordinates_homo = np.append(obj_camera_coordinates, [1])  # 将物体坐标转换为齐次坐标
        # obj_end_effector_coordinates_homo = np.linalg.inv(T_camera_to_end_effector).dot(obj_camera_coordinates_homo)
        obj_end_effector_coordinates_homo = T_camera_to_end_effector.dot(obj_camera_coordinates_homo)
        obj_base_coordinates_homo = T_base_to_end_effector.dot(obj_end_effector_coordinates_homo)
        obj_base_coordinates = obj_base_coordinates_homo[:3]  # 从齐次坐标中提取物体的x, y, z坐标
    
        x, y, z = obj_base_coordinates

        return x, y, z
                
    def check_data_ready(self): 
        if self.curret_pose2 is not None and self.curret_pose3 is not None: 
            self.data_ready_event.set()
        if self.run_state is not False:
            self.data_ready_event.set()
        if self.object_pose is not None:
            if self.object_pose.z > 0:
                self.data_ready_event.set()
        
            
                
    def worker_function(self):
        # 打开傲意灵巧收
        self.open_hand()

        # 1.机械臂到达准备抓取位置
        self.ready_catch()
        
        # 判断机械臂是否规划成功，如果不成功，不会执行后续程序
        while self.run_state is False:
            self.get_logger().info("Waiting for plan succeed...")
            
        print(self.run_state)
        self.run_state=False
        
        
        # 2.升降机到达视觉可看到要抓取的物体的位置
        self.set_lift_height(360,50)
        # 3.获取当前机械臂状态
        self.get_pose_pub()
        # 判断机械臂当前状态数据是否获取，如果获取不到，不会执行后续程序
        while self.curret_pose2 is None or self.curret_pose3 is None:
            self.get_logger().info("Waiting for curret_pose2 and curret_pose3 to be updated...")
            
        print(self.curret_pose2.pose.orientation) 
        print(self.curret_pose3.pose)
        
        x1 = self.curret_pose3.pose[0]
        y1 = self.curret_pose3.pose[1]
        z1 = self.curret_pose3.pose[2]
        rx = self.curret_pose3.pose[3]
        ry = self.curret_pose3.pose[4]
        rz = self.curret_pose3.pose[5]
        
        # 4.获取相机识别到物体的坐标
        # 如果获取不到不会执行后续程序
        while self.object_pose is None or (self.object_pose.x == 0 and self.object_pose.y == 0 and self.object_pose.z == 0):
            self.get_logger().info("Waiting for camera coordinate...")   
            
        camera_x = self.object_pose.x 
        camera_y = self.object_pose.y 
        camera_z = self.object_pose.z
        print(camera_x)
        print(camera_y)
        print(camera_z)
        # 5.计算物体在基座标的位置
        x,y,z=self.convert(camera_x,camera_y,camera_z,x1,y1,z1,rx,ry,rz)
        # 6.机械臂取抓取物体
        # 机械臂到达过渡位置
        
        moveJ_P_TargetPose = Movejp()
        moveJ_P_TargetPose.pose.position.x = self.transition_pose[0]
        moveJ_P_TargetPose.pose.position.y = self.transition_pose[1]
        moveJ_P_TargetPose.pose.position.z = self.transition_pose[2]
        moveJ_P_TargetPose.pose.orientation.x = self.transition_pose[3]
        moveJ_P_TargetPose.pose.orientation.y = self.transition_pose[4]
        moveJ_P_TargetPose.pose.orientation.z = self.transition_pose[5]
        moveJ_P_TargetPose.pose.orientation.w = self.transition_pose[6]
        moveJ_P_TargetPose.speed = self.speed
        moveJ_P_TargetPose.block = True # 是否阻塞

        self.pub_to_pose.publish(moveJ_P_TargetPose)
        
        # 判断机械臂是否规划成功，如果不成功，不会执行后续程序
        while self.run_state is False:
            self.get_logger().info("Waiting for plan succeed...")
        print(self.run_state)
        self.run_state=False
        
        # 机械臂到达物体前方位置
        moveJ_P_TargetPose = Movejp()
        moveJ_P_TargetPose.pose.position.x = x
        moveJ_P_TargetPose.pose.position.y = y + 0.1
        moveJ_P_TargetPose.pose.position.z = z
        moveJ_P_TargetPose.pose.orientation.x = self.grasping_pose[0]
        moveJ_P_TargetPose.pose.orientation.y = self.grasping_pose[1]
        moveJ_P_TargetPose.pose.orientation.z = self.grasping_pose[2]
        moveJ_P_TargetPose.pose.orientation.w = self.grasping_pose[3]
        moveJ_P_TargetPose.speed = self.speed
        moveJ_P_TargetPose.block = True # 是否阻塞
        self.pub_to_pose.publish(moveJ_P_TargetPose)
        # 判断机械臂是否规划成功，如果不成功，不会执行后续程序
        while self.run_state is False:
            self.get_logger().info("Waiting for plan succeed...")
        print(self.run_state)
        self.run_state=False
      
        # 机械臂到达物体位置
        moveJ_P_TargetPose = Movejp()
        moveJ_P_TargetPose.pose.position.x = x
        moveJ_P_TargetPose.pose.position.y = y + 0.06 
        moveJ_P_TargetPose.pose.position.z = z
        moveJ_P_TargetPose.pose.orientation.x = self.grasping_pose[0]
        moveJ_P_TargetPose.pose.orientation.y = self.grasping_pose[1]
        moveJ_P_TargetPose.pose.orientation.z = self.grasping_pose[2]
        moveJ_P_TargetPose.pose.orientation.w = self.grasping_pose[3]
        moveJ_P_TargetPose.speed = self.speed
        moveJ_P_TargetPose.block = True # 是否阻塞
        
        self.pub_to_pose.publish(moveJ_P_TargetPose)
        # 判断机械臂是否规划成功，如果不成功，不会执行后续程序
        while self.run_state is False:
            self.get_logger().info("Waiting for plan succeed...")
        print(self.run_state)
        self.run_state=False
        
        # 7.关闭夹爪
        self.catch_dumb()
      
        # 7.机械臂回到初始状态
        # 机械臂到达物体上方位置
        moveJ_P_TargetPose = Movejp()
        moveJ_P_TargetPose.pose.position.x = x + 0.05
        moveJ_P_TargetPose.pose.position.y = y + 0.05
        moveJ_P_TargetPose.pose.position.z = z + 0.05
        moveJ_P_TargetPose.pose.orientation.x = self.grasping_pose[0]
        moveJ_P_TargetPose.pose.orientation.y = self.grasping_pose[1]
        moveJ_P_TargetPose.pose.orientation.z = self.grasping_pose[2]
        moveJ_P_TargetPose.pose.orientation.w = self.grasping_pose[3]
        moveJ_P_TargetPose.speed = self.speed
        moveJ_P_TargetPose.block = True # 是否阻塞
        
        self.pub_to_pose.publish(moveJ_P_TargetPose)
        # 判断机械臂是否规划成功，如果不成功，不会执行后续程序
        # 判断机械臂是否规划成功，如果不成功，不会执行后续程序
        while self.run_state is False:
            self.get_logger().info("Waiting for plan succeed...")
        print(self.run_state)
        self.run_state=False
        
        # 机械臂到达过渡位置
        
        moveJ_P_TargetPose = Movejp()
        moveJ_P_TargetPose.pose.position.x = self.transition_pose[0]
        moveJ_P_TargetPose.pose.position.y = self.transition_pose[1]
        moveJ_P_TargetPose.pose.position.z = self.transition_pose[2]
        moveJ_P_TargetPose.pose.orientation.x = self.transition_pose[3]
        moveJ_P_TargetPose.pose.orientation.y = self.transition_pose[4]
        moveJ_P_TargetPose.pose.orientation.z = self.transition_pose[5]
        moveJ_P_TargetPose.pose.orientation.w = self.transition_pose[6]
        moveJ_P_TargetPose.speed = self.speed
        moveJ_P_TargetPose.block = True # 是否阻塞

        self.pub_to_pose.publish(moveJ_P_TargetPose)
        
        # 判断机械臂是否规划成功，如果不成功，不会执行后续程序
        while self.run_state is False:
            self.get_logger().info("Waiting for plan succeed...")
        print(self.run_state)
        self.run_state=False
        
        # 机械臂到达准备抓取位置
        self.ready_catch()
        # 判断机械臂是否规划成功，如果不成功，不会执行后续程序
        while self.run_state is False:
            self.get_logger().info("Waiting for plan succeed...")
        print(self.run_state)
        self.run_state=False

def main(args=None):
    rclpy.init(args=args)
    node = CatchNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
