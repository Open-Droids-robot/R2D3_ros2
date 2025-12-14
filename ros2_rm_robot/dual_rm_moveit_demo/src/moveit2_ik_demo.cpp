// Copyright RealMan
// License(BSD/GPL/...)
// Author: Starry

// 使用Moveit2提供的C++接口实现左右臂的笛卡尔空间运动（逆运动学，给定笛卡尔空间中的某个位置，求解各个关节的角度）

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

class MoveItIkDemo
{
public:
    MoveItIkDemo() : node_(rclcpp::Node::make_shared("hello_moveit2_ik_demo")),
                     logger_(rclcpp::get_logger("log")),
                     move_group_interface_left_(node_, "left_arm"),
                     move_group_interface_right_(node_, "right_arm")
    {
        RCLCPP_INFO(logger_, "hello moveit2!");

        move_group_interface_left_.setGoalPositionTolerance(0.1);    // 设置目标位置的容差
        move_group_interface_left_.setGoalOrientationTolerance(0.1); // 设置了目标姿态的容差
        move_group_interface_left_.setPlanningTime(8);               // 设置了规划时间

        move_group_interface_right_.setGoalPositionTolerance(0.1);
        move_group_interface_right_.setGoalOrientationTolerance(0.1);
        move_group_interface_right_.setPlanningTime(10);

        setTargetPose_left();  // 设置左臂的目标姿态
        setTargetPose_right(); // 设置右臂的目标姿态
    }

    void run_left()
    {
        if (planAndExecute_left())
        {
            RCLCPP_INFO(logger_, "Execution successful.");
        }
        else
        {
            RCLCPP_ERROR(logger_, "Planning failed!");
        }
    }

    void run_right()
    {
        if (planAndExecute_right())
        {
            RCLCPP_INFO(logger_, "Execution successful.");
        }
        else
        {
            RCLCPP_ERROR(logger_, "Planning failed!");
        }
    }

private:
    rclcpp::Node::SharedPtr node_;                                              // 声明节点指针
    rclcpp::Logger logger_;                                                     // 声明日志对象
    moveit::planning_interface::MoveGroupInterface move_group_interface_left_;  // 声明左臂moveit接口
    moveit::planning_interface::MoveGroupInterface move_group_interface_right_; // 声明右臂moveit接口
    moveit::planning_interface::MoveGroupInterface::Plan plan_left_;            // 声明左臂规划器
    moveit::planning_interface::MoveGroupInterface::Plan plan_right_;           // 声明右臂规划器
    // 设置目标姿态
    void setTargetPose_left()
    {
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.w = 1.0;
        target_pose.position.x = 0.56;
        target_pose.position.y = -0.23;
        target_pose.position.z = 1.158;
        move_group_interface_left_.setPoseTarget(target_pose); // 设置左臂目标位姿
    }
    void setTargetPose_right()
    {
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.w = 1.0;
        target_pose.position.x = -0.71;
        target_pose.position.y = -0.31;
        target_pose.position.z = 0.92;
        move_group_interface_right_.setPoseTarget(target_pose); // 设置右臂目标位姿
    }
    // 左臂规划和执行方法
    bool planAndExecute_left()
    {
        bool success_left_ = false;
        while (!success_left_)
        {
            success_left_ = static_cast<bool>(move_group_interface_left_.plan(plan_left_));
        }

        move_group_interface_left_.execute(plan_left_);

        return success_left_;
    }
    // 右臂规划和执行的方法
    bool planAndExecute_right()
    {
        bool success_right_ = false;
        while (!success_right_)
        {
            success_right_ = static_cast<bool>(move_group_interface_right_.plan(plan_right_));
        }

        move_group_interface_right_.execute(plan_right_);

        return success_right_;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // 初始化ROS2客户端
    MoveItIkDemo demo;        // 实例化对象
    demo.run_left();          // 调用左臂运行方法
    demo.run_right();         // 调用右臂运行方法
    rclcpp::shutdown();       // 释放资源
    return 0;
}
