#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace r2d3_test
{

/**
 * @brief Simple node that sends JointTrajectory goals to the left and
 *        right arm controllers via the FollowJointTrajectory action.
 *
 * Sequence:
 *   1. Move left arm to "wave" pose   (joints 1-6)
 *   2. Move left arm back to home     (all zeros)
 *   3. Move right arm to "wave" pose
 *   4. Move right arm back to home
 *
 * Joint names for 65b model:
 *   left:  l_joint1 … l_joint6
 *   right: r_joint1 … r_joint6
 */
class TestArmMotion : public rclcpp::Node
{
public:
  using FollowJT = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJT>;

  TestArmMotion();

private:
  /// Drive the step-by-step motion sequence.
  void step_sequence();

  /// Send a trajectory goal to the specified action server.
  void send_goal(
    rclcpp_action::Client<FollowJT>::SharedPtr client,
    const std::vector<std::string> & joint_names,
    const std::vector<double> & positions,
    double duration_sec,
    const std::string & label);

  /// Callbacks for the action client.
  void goal_response_cb(const GoalHandleFJT::SharedPtr & goal_handle);
  void result_cb(const GoalHandleFJT::WrappedResult & result);

  rclcpp_action::Client<FollowJT>::SharedPtr left_client_;
  rclcpp_action::Client<FollowJT>::SharedPtr right_client_;

  /// Timer that drives the step-by-step sequence.
  rclcpp::TimerBase::SharedPtr seq_timer_;
  int step_{0};

  // Joint names
  const std::vector<std::string> left_joints_{
    "l_joint1", "l_joint2", "l_joint3", "l_joint4", "l_joint5", "l_joint6"};
  const std::vector<std::string> right_joints_{
    "r_joint1", "r_joint2", "r_joint3", "r_joint4", "r_joint5", "r_joint6"};
};

}  // namespace r2d3_test
