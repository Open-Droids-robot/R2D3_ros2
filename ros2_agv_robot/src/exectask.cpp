#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "woosh_robot_msgs/action/exec_task.hpp"

// clang-format off
/// ros2 cli command:
/// ros2 action send_goal /woosh_robot/robot/ExecTask woosh_robot_msgs/action/ExecTask "{arg:{type:{value: 1}, mark_no: A1}}" --feedback
// clang-format on

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  /// ros2 run woosh_robot_demo exectask --ros-args -p mark_no:="A1"
  auto node = rclcpp::Node::make_shared("exectask");
  node->declare_parameter("task_id", 0);
  int64_t task_id = node->get_parameter("task_id").as_int();
  node->declare_parameter("type", 1);
  int32_t type = node->get_parameter("type").as_int();
  node->declare_parameter("dir", 0);
  int32_t dir = node->get_parameter("dir").as_int();
  node->declare_parameter("type_no", 0);
  int32_t type_no = node->get_parameter("type_no").as_int();
  node->declare_parameter("mark_no", "");
  std::string mark_no = node->get_parameter("mark_no").as_string();

  RCLCPP_INFO(node->get_logger(),
              "task_id: %ld, type: %d, dir: %d, type_no: %d, mark_no: %s",
              task_id, type, dir, type_no, mark_no.c_str());
  if (mark_no.empty()) {
    RCLCPP_ERROR(node->get_logger(), "储位号(mark_no)不能为空!");
    return 1;
  }
  if (task_id == 0) task_id = node->now().seconds();

  // call action

  auto exec_task_cli =
      rclcpp_action::create_client<woosh_robot_msgs::action::ExecTask>(
          node, "woosh_robot/robot/ExecTask");
  if (!exec_task_cli->wait_for_action_server(std::chrono::seconds(20))) {
    RCLCPP_ERROR(node->get_logger(), "等待服务超时!");
    return 1;
  }

  // 请求参数
  auto send_goal = woosh_robot_msgs::action::ExecTask::Goal();
  send_goal.arg.task_id = task_id;       // 任务ID
  send_goal.arg.type.value = type;       // 任务类型
  send_goal.arg.direction.value = dir;   // 动作方向
  send_goal.arg.task_type_no = type_no;  // 动作组合
  send_goal.arg.mark_no = mark_no;       // 目标点储位号

  // 任务执行过程回调
  auto send_goal_options = rclcpp_action::Client<
      woosh_robot_msgs::action::ExecTask>::SendGoalOptions();
  send_goal_options.feedback_callback =
      [&node](rclcpp_action::ClientGoalHandle<
                  woosh_robot_msgs::action::ExecTask>::SharedPtr,
              const std::shared_ptr<
                  const woosh_robot_msgs::action::ExecTask::Feedback>
                  feedback) {
        auto task_proc = feedback->fb;
        RCLCPP_INFO(node->get_logger(),
                    "任务进度更新, 目的地: %s, 动作: %d, 动作状态: "
                    "%d, 任务状态: %d",
                    task_proc.dest.c_str(), task_proc.action.type.value,
                    task_proc.action.state.value, task_proc.state.value);
        if (task_proc.state.value ==
            woosh_task_msgs::msg::State::K_ACTION_WAIT) {
          RCLCPP_INFO(node->get_logger(), "Feedback: 进入等待.");
        }
      };

  // 任务执行请求
  auto goal_handle_future =
      exec_task_cli->async_send_goal(send_goal, send_goal_options);
  if (rclcpp::spin_until_future_complete(node, goal_handle_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "任务执行请求失败!");
    return 1;
  }

  rclcpp_action::ClientGoalHandle<woosh_robot_msgs::action::ExecTask>::SharedPtr
      goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "请求被服务拒绝!");
    return 1;
  }

  // 等待任务执行结果
  auto result_future = exec_task_cli->async_get_result(goal_handle);
  if (rclcpp::spin_until_future_complete(node, result_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "获取结果失败!");
    return 1;
  }

  rclcpp_action::ClientGoalHandle<
      woosh_robot_msgs::action::ExecTask>::WrappedResult wrapped_result =
      result_future.get();

  switch (wrapped_result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(node->get_logger(), "任务异常结束");
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(node->get_logger(), "任务被取消");
    default:
      RCLCPP_WARN(node->get_logger(), "未知错误");
  }

  auto task_proc = wrapped_result.result->ret;
  if (task_proc.state.value == woosh_task_msgs::msg::State::K_COMPLETED) {
    RCLCPP_INFO(node->get_logger(), "Result: 任务完成.");
  } else if (task_proc.state.value == woosh_task_msgs::msg::State::K_CANCELED) {
    RCLCPP_INFO(node->get_logger(), "Result: 任务取消.");
  } else if (task_proc.state.value == woosh_task_msgs::msg::State::K_FAILED) {
    RCLCPP_INFO(node->get_logger(), "Result: 任务失败.");
  }

  exec_task_cli.reset();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
