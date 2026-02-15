#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace r2d3_test
{

/**
 * @brief Simple node that publishes TwistStamped commands to the
 *        diff_drive_controller for testing AGV base motion.
 *
 * Sequence: forward → stop → rotate CW → stop → backward → stop.
 * Each phase lasts a few seconds so you can observe the motion.
 */
class TestAgvMotion : public rclcpp::Node
{
public:
  TestAgvMotion();

private:
  void timer_callback();

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int phase_{0};          // current motion phase
  int ticks_{0};          // timer ticks within current phase
  static constexpr int PHASE_TICKS = 30;  // ticks per phase (30 × 100 ms = 3 s)
};

}  // namespace r2d3_test
