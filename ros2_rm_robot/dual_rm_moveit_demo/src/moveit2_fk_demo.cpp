// Copyright RealMan
// License(BSD/GPL/...)
// Author: Starry

// 使用Moveit2提供的C++接口实现左右臂的关节空间运动（正运动学，给各个关节的位置和速度去求解末端执行器的位姿）

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

// 使用std标准命令空间
using namespace std;

// 自定义节点类
class MoveItFkDemo
{
public:
  MoveItFkDemo() : node_(rclcpp::Node::make_shared("rm_dual_arm_moveit2_fk_demo")),
                   logger_(rclcpp::get_logger("log")),
                   move_group_left_(node_, "left_arm"),
                   move_group_right_(node_, "right_arm"),
                   arm_dof(7)
  {
    node_->declare_parameter("arm_dof", 7);
    arm_dof = node_->get_parameter("arm_dof").as_int();
    RCLCPP_INFO(logger_, "hello moveit2 realman dual arm!");
    RCLCPP_INFO(logger_, "arm_dof = %d", arm_dof);
  }
  // 运行demo的方法
  void runDemo()
  {
    if (arm_dof == 7)
    {
      // 定义初始化位姿
      std::vector<double> start_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      // 设置每个关节的角度值
      move_group_left_.setJointValueTarget(start_pos);
      move_group_right_.setJointValueTarget(start_pos);
      // 执行
      move_group_left_.move();
      move_group_right_.move();
      RCLCPP_INFO(logger_, "Successfully returned to the start position!");

      rclcpp::sleep_for(std::chrono::seconds(1));
      // l_joint1->l_joint7
      // 设置目标位姿
      std::vector<double> target_pose = {-0.5683, -1.0275, -0.5354, -1.5626, -2.8461, -0.1654, 2.7269};
      move_group_left_.setJointValueTarget(target_pose);
      move_group_right_.setJointValueTarget(target_pose);
      move_group_left_.move();
      move_group_right_.move();
      RCLCPP_INFO(logger_, "Successfully reached the desired target pose!");

      rclcpp::sleep_for(std::chrono::seconds(1));
      // 回到初始位姿
      move_group_left_.setJointValueTarget(start_pos);
      move_group_right_.setJointValueTarget(start_pos);
      move_group_left_.move();
      move_group_right_.move();
      RCLCPP_INFO(logger_, "Successfully returned to the start position!");
    }
    else
    {
      // 定义初始化位姿
      std::vector<double> start_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      // 设置每个关节的角度值
      move_group_left_.setJointValueTarget(start_pos);
      move_group_right_.setJointValueTarget(start_pos);
      // 执行
      move_group_left_.move();
      move_group_right_.move();
      RCLCPP_INFO(logger_, "Successfully returned to the start position!");

      rclcpp::sleep_for(std::chrono::seconds(1));
      // l_joint1->l_joint7
      // 设置目标位姿
      std::vector<double> target_pose = {-0.5683, -1.0275, -0.5354, -1.5626, -2.8461, -0.1654};
      move_group_left_.setJointValueTarget(target_pose);
      move_group_right_.setJointValueTarget(target_pose);
      move_group_left_.move();
      move_group_right_.move();
      RCLCPP_INFO(logger_, "Successfully reached the desired target pose!");

      rclcpp::sleep_for(std::chrono::seconds(1));
      // 回到初始位姿
      move_group_left_.setJointValueTarget(start_pos);
      move_group_right_.setJointValueTarget(start_pos);
      move_group_left_.move();
      move_group_right_.move();
      RCLCPP_INFO(logger_, "Successfully returned to the start position!");
    }
  }

private:
  rclcpp::Node::SharedPtr node_;                                    // 声明节点对象指针
  rclcpp::Logger logger_;                                           // 声明日志对象
  moveit::planning_interface::MoveGroupInterface move_group_left_;  // 声明moveit2左臂规划接口
  moveit::planning_interface::MoveGroupInterface move_group_right_; // 声明moveit2右臂规划接口
  int arm_dof;                                                      // 自由度
};

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv); // 初始化ROS2客户端
  MoveItFkDemo demo;        // 实例化MoveItFkDemo对象
  demo.runDemo();           // 调用runDemo方法
  rclcpp::shutdown();       // 释放资源
  return 0;
}
