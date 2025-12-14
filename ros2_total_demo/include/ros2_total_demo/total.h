// Copyright RealMan
// License(BSD/GPL/...)
// Author: Haley

/*
    总的demo，继承了机械臂控制、升降机控制、头部舵机控制
*/

#ifndef TOTAL_H
#define TOTAL_H

#include "dual_arm.h"

using namespace std;
using namespace std::chrono_literals;

string status = "idle";

class TotalDemo : public DualArm
{
public:
    // 构造函数
    TotalDemo(rclcpp::Node::SharedPtr node);
    // 运行demo接口
    void run_demo();

private:
    rclcpp::Node::SharedPtr node_; // 声明节点指针
};

#endif