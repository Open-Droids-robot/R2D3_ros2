#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

using namespace std;

class MoveitHandDemo
{
public:
    MoveitHandDemo(): node_(rclcpp::Node::make_shared("rm_hand_moveit2_demo")),
                      logger_(rclcpp::get_logger("log")),
                      move_group_left_hand_(node_, "left_hand"),
                      move_group_right_hand_(node_, "right_hand") 
    {
        RCLCPP_INFO(logger_, "hello moveit2 realman hand demo!");
    }

    void runDemo() 
    {
        // 定义初始化位姿
        std::vector<double> start_pos = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        // 设置每个关节的角度值
        move_group_left_hand_.setJointValueTarget(start_pos);
        move_group_right_hand_.setJointValueTarget(start_pos);
        // 执行
        move_group_left_hand_.move();
        move_group_right_hand_.move();
        RCLCPP_INFO(logger_, "Successfully returned to the start position!");

        rclcpp::sleep_for(std::chrono::seconds(1));
        // 设置目标位姿
        std::vector<double> target_pose = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
        move_group_left_hand_.setJointValueTarget(target_pose);
        move_group_right_hand_.setJointValueTarget(target_pose);
        move_group_left_hand_.move();
        move_group_right_hand_.move();
        RCLCPP_INFO(logger_, "Successfully reached the desired target pose!");

        rclcpp::sleep_for(std::chrono::seconds(1));
        // 回到初始位姿
        move_group_left_hand_.setJointValueTarget(start_pos);
        move_group_right_hand_.setJointValueTarget(start_pos);
        move_group_left_hand_.move();
        move_group_right_hand_.move();
        RCLCPP_INFO(logger_, "Successfully returned to the start position!");
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Logger logger_;
    moveit::planning_interface::MoveGroupInterface move_group_left_hand_;
    moveit::planning_interface::MoveGroupInterface move_group_right_hand_;
};

int main(int argc, char** argv) 
{
  rclcpp::init(argc, argv); 
  MoveitHandDemo demo;       
  demo.runDemo();           
  rclcpp::shutdown();
  return 0;
}