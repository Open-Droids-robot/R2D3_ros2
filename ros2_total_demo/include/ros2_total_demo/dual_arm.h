// Copyright RealMan
// License(BSD/GPL/...)
// Author: Haley

/*对左右臂的封装,实现的方法有：
    1.MoveJ
    2.MoveJP
    3.MoveL
    4.获取左右臂关节状态
    5.控制升降机构的高度和速度
    6.控制头部关节升降告诉和速度
*/

#ifndef DUAL_ARM_H
#define DUAL_ARM_H

#include <chrono>
#include <functional>
#include <memory>
#include <unistd.h>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "rm_ros_interfaces/msg/movej.hpp"
#include "rm_ros_interfaces/msg/movejp.hpp"
#include "rm_ros_interfaces/msg/movel.hpp"
#include "rm_ros_interfaces/msg/armoriginalstate.hpp"
#include "rm_ros_interfaces/msg/liftheight.hpp"
#include "servo_interfaces/msg/servo_move.hpp"

using std::placeholders::_1;

class DualArm
{
public:
    DualArm(rclcpp::Node::SharedPtr node);                // 构造函数
    void execute_movej();                                 // 执行movej接口
    void execute_movejp();                                // 执行movejp接口
    void execute_movel();                                 // 执行movel接口
    void execute_get_arm_state();                         // 执行获取机械臂状态接口
    void execute_lift_height(int height, int speed);      // 执行升降动作接口
    void execute_head_servo(int id, int angle);           // 执行头部舵机动作接口
    rclcpp::Node::SharedPtr node_;                        // 声明节点指针

private:
    /********************************************************common****************************************************/
    int arm_dof_ = 7;                                                               // 定义机械臂关节数
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_work_frame_left;  // 声明左臂坐标发布器
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_work_frame_right; // 声明右臂坐标发布器
    void init_frame_left();                                                         // 初始化左臂工作坐标系
    void init_frame_right();                                                        // 初始化右臂工作坐标系
    /********************************************************common****************************************************/

    /**********************************************************MoveJ************************************************* */
    void movej_demo_left();                                                             // 发布左臂MoveJ规划指令
    void movej_demo_right();                                                            // 发布右臂MoveJ规划指令
    void DualArm_Callback_left(const std_msgs::msg::Bool::SharedPtr msg);               // 左臂结果回调函数
    void DualArm_Callback_right(const std_msgs::msg::Bool::SharedPtr msg);              // 右臂结果回调函数
    rclcpp::Publisher<rm_ros_interfaces::msg::Movej>::SharedPtr movej_pub_left_;  // 声明左臂发布器
    rclcpp::Publisher<rm_ros_interfaces::msg::Movej>::SharedPtr movej_pub_right_; // 声明右臂发布器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movej_sub_left_;               // 声明左臂订阅器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movej_sub_right_;              // 声明右臂订阅器
    rm_ros_interfaces::msg::Movej movej_way_left;                                 // 实例化左臂Movej对象
    rm_ros_interfaces::msg::Movej movej_way_right;                                // 实例化左臂Movej对象
    /********************************************************MoveJ End*********************************************** */

    /*********************************************************MoveJP************************************************* */
    rclcpp::Publisher<rm_ros_interfaces::msg::Movejp>::SharedPtr movejp_pub_left_;  // 声明左臂movejp发布器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movejp_sub_left_;                // 声明左臂订阅器
    rclcpp::Publisher<rm_ros_interfaces::msg::Movejp>::SharedPtr movejp_pub_right_; // 声明右臂movejp发布器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movejp_sub_right_;               // 声明右臂订阅器
    void movejp_demo_left();                                                              // 发布左臂movejp指令
    void movejp_demo_right();                                                             // 发布右臂movejp指令
    void MoveJPDemo_Callback_left(const std_msgs::msg::Bool::SharedPtr msg);              // 左臂结果回调函数
    void MoveJPDemo_Callback_right(const std_msgs::msg::Bool::SharedPtr msg);             // 右臂结果回调函数
    /*******************************************************MoveJP End*********************************************/

    /*******************************************************MoveL**************************************************/
    rclcpp::Publisher<rm_ros_interfaces::msg::Movel>::SharedPtr movel_pub_left_;  // 声明左臂发布器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movel_sub_left_;               // 声明左臂订阅器
    rclcpp::Publisher<rm_ros_interfaces::msg::Movel>::SharedPtr movel_pub_right_; // 声明右臂发布器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr movel_sub_right_;              // 声明右臂订阅器
    void movel_demo_left();                                                             // 左臂movel运动规划函数
    void movel_demo_right();                                                            // 右臂movel运动规划函数
    void MoveLDemo_Callback_Left(const std_msgs::msg::Bool::SharedPtr msg);             // 左臂结果回调函数
    void MoveLDemo_Callback_Right(const std_msgs::msg::Bool::SharedPtr msg);            // 右臂结果回调函数
    void moveL_publisher_left();                                                        // 发布左臂moveL指令
    void moveL_publisher_right();                                                       // 发布右臂moveL指令
    /*******************************************************MoveL End**************************************************/

    /*******************************************************Lift Height************************************************/
    rclcpp::Publisher<rm_ros_interfaces::msg::Liftheight>::SharedPtr lift_height_pub_; // 声明升降关节升降高度、速度发布器
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lift_height_result_;                // 声明升降关节升降高度、速度结果订阅器
    void lift_height_callback(const std_msgs::msg::Bool::SharedPtr msg);                     // 声明升降关节升降高度结果回调函数
    /*******************************************************Lift Height End********************************************/

    /*******************************************************Lift Height************************************************/
    rclcpp::Publisher<servo_interfaces::msg::ServoMove>::SharedPtr head_servo_pub_; // 声明头部关节角度发布器 
    /*******************************************************Lift Height End********************************************/

    /*******************************************************Get Arm State**********************************************/
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr get_state_pub_left_;                                 // 声明左臂发布器
    rclcpp::Subscription<rm_ros_interfaces::msg::Armoriginalstate>::SharedPtr get_state_sub_left_;    // 声明左臂订阅器
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr get_state_pub_right_;                                // 声明右臂发布器
    rclcpp::Subscription<rm_ros_interfaces::msg::Armoriginalstate>::SharedPtr get_state_sub_right_;   // 声明右臂订阅器
    void get_arm_state_left();                                                                              // 获取左臂状态的发布函数
    void get_arm_state_right();                                                                             // 获取右臂状态的发布函数
    void get_arm_state_Callback_left(const rm_ros_interfaces::msg::Armoriginalstate::SharedPtr msg);  // 左臂结果回调函数
    void get_arm_state_Callback_right(const rm_ros_interfaces::msg::Armoriginalstate::SharedPtr msg); // 右臂结果回调函数
    /*******************************************************Get Arm State End*********************************************/
};

#endif
