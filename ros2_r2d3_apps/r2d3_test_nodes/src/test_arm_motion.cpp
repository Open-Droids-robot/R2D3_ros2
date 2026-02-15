#include "r2d3_test_nodes/test_arm_motion.hpp"

namespace r2d3_test
{

TestArmMotion::TestArmMotion()
: Node("test_arm_motion")
{
  // Action clients for left and right arm controllers
  left_client_ = rclcpp_action::create_client<FollowJT>(
    this, "/left_arm_controller/follow_joint_trajectory");
  right_client_ = rclcpp_action::create_client<FollowJT>(
    this, "/right_arm_controller/follow_joint_trajectory");

  RCLCPP_INFO(this->get_logger(), "Waiting for arm action servers...");

  // Wait for both servers (blocks up to 10 s each)
  if (!left_client_->wait_for_action_server(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(this->get_logger(), "Left arm action server not available!");
    rclcpp::shutdown();
    return;
  }
  if (!right_client_->wait_for_action_server(std::chrono::seconds(10)))
  {
    RCLCPP_ERROR(this->get_logger(), "Right arm action server not available!");
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Both arm action servers connected.");
  RCLCPP_INFO(this->get_logger(),
    "Sequence: left wave → left home → right wave → right home");

  // Drive the sequence with a 5-second timer (each step gets 5 s to execute)
  seq_timer_ = this->create_wall_timer(
    std::chrono::seconds(5),
    [this]() { this->step_sequence(); });

  // Kick off the first step immediately
  step_sequence();
}

// ── Step-by-step sequence ──────────────────────────────────────
void TestArmMotion::step_sequence()
{
  // Home position: all joints at 0 rad
  const std::vector<double> home{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  // "Wave" pose: modest joint angles (radians) — safe for 65b
  //   joint1=0.5, joint2=-0.3, joint3=0.4, joint4=0.0, joint5=0.5, joint6=0.0
  const std::vector<double> wave{0.5, -0.3, 0.4, 0.0, 0.5, 0.0};

  switch (step_)
  {
    case 0:
      send_goal(left_client_, left_joints_, wave, 3.0, "Left arm → wave pose");
      break;
    case 1:
      send_goal(left_client_, left_joints_, home, 3.0, "Left arm → home");
      break;
    case 2:
      send_goal(right_client_, right_joints_, wave, 3.0, "Right arm → wave pose");
      break;
    case 3:
      send_goal(right_client_, right_joints_, home, 3.0, "Right arm → home");
      break;
    default:
      seq_timer_->cancel();
      RCLCPP_INFO(this->get_logger(), "Arm test sequence complete. Shutting down.");
      rclcpp::shutdown();
      return;
  }
  step_++;
}

// ── Send a FollowJointTrajectory goal ──────────────────────────
void TestArmMotion::send_goal(
  rclcpp_action::Client<FollowJT>::SharedPtr client,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & positions,
  double duration_sec,
  const std::string & label)
{
  RCLCPP_INFO(this->get_logger(), "[Step %d/4] %s", step_ + 1, label.c_str());

  FollowJT::Goal goal;
  goal.trajectory.joint_names = joint_names;

  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions = positions;
  point.time_from_start = rclcpp::Duration::from_seconds(duration_sec);
  goal.trajectory.points.push_back(point);

  auto send_opts = rclcpp_action::Client<FollowJT>::SendGoalOptions();
  send_opts.goal_response_callback =
    std::bind(&TestArmMotion::goal_response_cb, this, std::placeholders::_1);
  send_opts.result_callback =
    std::bind(&TestArmMotion::result_cb, this, std::placeholders::_1);

  client->async_send_goal(goal, send_opts);
}

// ── Action callbacks ───────────────────────────────────────────
void TestArmMotion::goal_response_cb(
  const GoalHandleFJT::SharedPtr & goal_handle)
{
  if (!goal_handle)
    RCLCPP_WARN(this->get_logger(), "Goal was rejected by server.");
  else
    RCLCPP_INFO(this->get_logger(), "Goal accepted.");
}

void TestArmMotion::result_cb(
  const GoalHandleFJT::WrappedResult & result)
{
  switch (result.code)
  {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Goal succeeded.");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(this->get_logger(), "Goal aborted.");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "Goal canceled.");
      break;
    default:
      RCLCPP_WARN(this->get_logger(), "Unknown result code.");
      break;
  }
}

}  // namespace r2d3_test

// ── main ──────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<r2d3_test::TestArmMotion>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
