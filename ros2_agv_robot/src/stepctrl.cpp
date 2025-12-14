#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "woosh_ros_msgs/action/step_control.hpp"

// clang-format off
/// ros2 cli command:
/// ros action send_goal /woosh_robot/ros/StepControl woosh_ros_msgs/action/StepControl "{arg:{action:{value: 1}, steps:[{mode:{value: 2}, speed: 1, value: 3}]}}" --feedback
/// action.value: 0 -> 取消
/// action.value: 1 -> 执行
/// action.value: 2 -> 暂停
/// action.value: 3 -> 继续
/// mode.value: 1 -> 直行
/// mode.value: 2 -> 旋转
/// mode.value: 3 -> 横移
/// mode.value: 4 -> 斜移
/// speed: folat -> 角速度[弧度/s]/线速度[米/s]
/// value: float -> 旋转弧度/行驶距离, 正前负后, 正逆负顺, 正左负右
/// angle: float -> 斜向运动角度, 正左负右
// clang-format on

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  // clang-format off
  /// ros2 run woosh_robot_demo stepctrl --ros-args -p mode:=1 -p action:=1 -p speed:=0.2 -p value:=0.5
  // clang-format on
  auto node = rclcpp::Node::make_shared("stepctrl");
  node->declare_parameter("mode", 1);
  int32_t mode = node->get_parameter("mode").as_int();
  node->declare_parameter("action", 1);
  int32_t action = node->get_parameter("action").as_int();
  node->declare_parameter("speed", 0.2);
  float_t speed = node->get_parameter("speed").as_double();
  node->declare_parameter("value", 0.5);
  float_t value = node->get_parameter("value").as_double();
  node->declare_parameter("angle", 0.0);
  float_t angle = node->get_parameter("angle").as_double();

  RCLCPP_INFO(node->get_logger(),
              "mode: %d, action: %d, speed: %f, value: %f, angle: %f", mode,
              action, speed, value, angle);

  // call action

  auto exec_task_cli =
      rclcpp_action::create_client<woosh_ros_msgs::action::StepControl>(
          node, "woosh_robot/ros/StepControl");
  if (!exec_task_cli->wait_for_action_server(std::chrono::seconds(20))) {
    RCLCPP_ERROR(node->get_logger(), "等待服务超时!");
    return 1;
  }

  // 请求参数
  auto send_goal = woosh_ros_msgs::action::StepControl::Goal();
  woosh_ros_msgs::msg::StepControlStep step;
  send_goal.arg.action.value = action;
  step.mode.value = mode;
  step.value = value;
  step.speed = speed;
  step.angle = angle;
  send_goal.arg.steps.push_back(step);

  // 步进过程回调
  auto send_goal_options = rclcpp_action::Client<
      woosh_ros_msgs::action::StepControl>::SendGoalOptions();
  send_goal_options.feedback_callback =
      [&node](rclcpp_action::ClientGoalHandle<
                  woosh_ros_msgs::action::StepControl>::SharedPtr,
              const std::shared_ptr<
                  const woosh_ros_msgs::action::StepControl::Feedback>
                  feedback) {
        woosh_ros_msgs::msg::Feedback fb = feedback->fb;
        RCLCPP_INFO(node->get_logger(),
                    "步进状态更新, 状态: %d, 状态码: %ld, 消息: %s",
                    fb.state.value, fb.code, fb.msg.c_str());
      };

  // 步进执行请求
  auto goal_handle_future =
      exec_task_cli->async_send_goal(send_goal, send_goal_options);
  if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "步进执行请求失败!");
    return 1;
  }

  rclcpp_action::ClientGoalHandle<
      woosh_ros_msgs::action::StepControl>::SharedPtr goal_handle =
      goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "请求被服务拒绝!");
    return 1;
  }

  // 等待步进执行结果
  auto result_future = exec_task_cli->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "获取结果失败!");
    return 1;
  }

  rclcpp_action::ClientGoalHandle<
      woosh_ros_msgs::action::StepControl>::WrappedResult wrapped_result =
      result_future.get();

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(node->get_logger(), "步进异常结束");
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(node->get_logger(), "步进被取消");
    default:
      RCLCPP_WARN(node->get_logger(), "未知错误");
  }

  auto task_proc = wrapped_result.result->ret;
  if (task_proc.state.value == woosh_ros_msgs::msg::State::K_ROS_SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Result: 步进完成.");
  } else if (task_proc.state.value ==
             woosh_ros_msgs::msg::State::K_ROS_CANCEL) {
    RCLCPP_INFO(node->get_logger(), "Result: 步进取消.");
  } else if (task_proc.state.value ==
             woosh_ros_msgs::msg::State::K_ROS_FAILURE) {
    RCLCPP_INFO(node->get_logger(), "Result: 步进失败.");
  }

  exec_task_cli.reset();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
