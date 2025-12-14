// Copyright RealMan
// License(BSD/GPL/...)
// Author: Haley

#include "total.h"
// 总测试demo
TotalDemo::TotalDemo(rclcpp::Node::SharedPtr node) : DualArm(node), node_(node)
{
    RCLCPP_INFO(node_->get_logger(), "All classes initialized successfully");
}

void TotalDemo::run_demo()
{
    RCLCPP_INFO(node_->get_logger(), "Running TotalDemo...");
    std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // 休眠

    /*****************************Dual Arm Contrl*************************************/
    // execute_movej();                                              // 执行movej指令
    // std::this_thread::sleep_for(std::chrono::milliseconds(4000)); // 休眠
    // execute_movejp();                                             // 执行movejp指令
    // std::this_thread::sleep_for(std::chrono::milliseconds(4000)); // 休眠
    // execute_movel();                                              // 执行movel指令
    // std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // 休眠
    // execute_get_arm_state();                                      // 执行获取机械臂状态
    /*****************************Dual Arm Contrl End*********************************/

    /***********************************************Total demo***************************************************************/

    execute_movej();

    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 休眠s
    execute_lift_height(200, 20);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // 休眠
    execute_lift_height(600, 10);

    std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // 休眠s
    execute_head_servo(1,400);                                    // 舵机1可设置范围400-600 舵机2可设置的范围200-800
    std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // 休眠s
    execute_head_servo(2,300);                                    // 舵机1可设置范围400-600 舵机2可设置的范围200-800
   
    RCLCPP_INFO(node_->get_logger(), "TotalDemo Over!");
    /***********************************************Total Demo End***********************************************************/
}

int main(int argc, char **argv)
{
    // 初始化ROS2客户端
    rclcpp::init(argc, argv);
    // 创建节点指针
    auto node = rclcpp::Node::make_shared("total_demo_node");
    // 创建TotalDemo指针，传入node节点指针
    auto total_demo = std::make_shared<TotalDemo>(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000)); // 休眠
    // 调用run_demo()
    total_demo->run_demo();
    // 调用spin循环
    rclcpp::spin(node);
    // 释放资源
    rclcpp::shutdown();
    return 0;
}
