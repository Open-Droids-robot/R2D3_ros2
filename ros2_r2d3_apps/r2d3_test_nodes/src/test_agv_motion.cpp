#include "r2d3_test_nodes/test_agv_motion.hpp"

namespace r2d3_test
{

TestAgvMotion::TestAgvMotion()
: Node("test_agv_motion")
{
  // diff_drive_controller expects TwistStamped on this topic
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    "/diff_drive_controller/cmd_vel", 10);

  // 100 ms timer → 10 Hz publish rate
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&TestAgvMotion::timer_callback, this));

  RCLCPP_INFO(this->get_logger(),
    "TestAgvMotion started — publishing TwistStamped to /diff_drive_controller/cmd_vel");
  RCLCPP_INFO(this->get_logger(),
    "Sequence: forward(3s) → stop(2s) → rotate CW(3s) → stop(2s) → backward(3s) → stop");
}

void TestAgvMotion::timer_callback()
{
  geometry_msgs::msg::TwistStamped msg;
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = "base_footprint";

  switch (phase_)
  {
    case 0:  // ── Forward ───────────────────────────────────
      msg.twist.linear.x = 0.2;   // 0.2 m/s forward
      msg.twist.angular.z = 0.0;
      if (ticks_ == 0) RCLCPP_INFO(this->get_logger(), "[Phase 1/6] Moving FORWARD at 0.2 m/s");
      break;

    case 1:  // ── Stop ──────────────────────────────────────
      // zero twist (default)
      if (ticks_ == 0) RCLCPP_INFO(this->get_logger(), "[Phase 2/6] STOP");
      break;

    case 2:  // ── Rotate clockwise ──────────────────────────
      msg.twist.angular.z = -0.5;  // -0.5 rad/s (CW)
      if (ticks_ == 0) RCLCPP_INFO(this->get_logger(), "[Phase 3/6] Rotating CLOCKWISE at 0.5 rad/s");
      break;

    case 3:  // ── Stop ──────────────────────────────────────
      if (ticks_ == 0) RCLCPP_INFO(this->get_logger(), "[Phase 4/6] STOP");
      break;

    case 4:  // ── Backward ──────────────────────────────────
      msg.twist.linear.x = -0.2;  // 0.2 m/s backward
      if (ticks_ == 0) RCLCPP_INFO(this->get_logger(), "[Phase 5/6] Moving BACKWARD at 0.2 m/s");
      break;

    case 5:  // ── Final stop ────────────────────────────────
      if (ticks_ == 0) RCLCPP_INFO(this->get_logger(), "[Phase 6/6] STOP — sequence complete");
      break;

    default:
      // Done — cancel timer
      timer_->cancel();
      RCLCPP_INFO(this->get_logger(), "Test finished. Shutting down.");
      rclcpp::shutdown();
      return;
  }

  cmd_pub_->publish(msg);

  ticks_++;
  // Phases 1,3,5 (stops) are shorter: 20 ticks = 2 s
  int max_ticks = (phase_ % 2 == 1) ? 20 : PHASE_TICKS;
  if (ticks_ >= max_ticks)
  {
    ticks_ = 0;
    phase_++;
  }
}

}  // namespace r2d3_test

// ── main ──────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<r2d3_test::TestAgvMotion>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
