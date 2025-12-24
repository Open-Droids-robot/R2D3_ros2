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
        RCLCPP_INFO(logger_, "hello moveit2 inspire hand demo!");
    }

    void runDemo() 
    {
        // Fully open hand (all joints at 0)
        std::vector<double> open_hand = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

        // Closed fist 
        // [thumb_yaw, thumb_pitch, index, middle, ring, pinky]
        std::vector<double> closed_fist = {1.0, 0.5, 1.4, 1.4, 1.4, 1.4};

        RCLCPP_INFO(logger_, "Opening hands...");
        move_group_left_hand_.setJointValueTarget(open_hand);
        move_group_right_hand_.setJointValueTarget(open_hand);
        move_group_left_hand_.move();
        move_group_right_hand_.move();
        RCLCPP_INFO(logger_, "Hands fully opened!");

        rclcpp::sleep_for(std::chrono::seconds(3));

        RCLCPP_INFO(logger_, "Closing hands into fist...");
        move_group_left_hand_.setJointValueTarget(closed_fist);
        move_group_right_hand_.setJointValueTarget(closed_fist);
        move_group_left_hand_.move();
        move_group_right_hand_.move();
        RCLCPP_INFO(logger_, "Hands closed into fist!");

        rclcpp::sleep_for(std::chrono::seconds(3));

        RCLCPP_INFO(logger_, "Opening hands again...");
        move_group_left_hand_.setJointValueTarget(open_hand);
        move_group_right_hand_.setJointValueTarget(open_hand);
        move_group_left_hand_.move();
        move_group_right_hand_.move();
        RCLCPP_INFO(logger_, "Hands opened again!");
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