#include "woosh_robot_msgs/srv/exec_pre_task.hpp"

#include "rclcpp/rclcpp.hpp"

// clang-format off
/// ros2 cli command:
/// ros2 service call /woosh_robot/robot/ExecPreTask woosh_robot_msgs/srv/ExecPreTask "{arg:{task_set_id: 666}}"
// clang-format on

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  /// ros2 run woosh_robot_demo exec_pre_task --ros-args -p id:=666
  auto node = rclcpp::Node::make_shared("exec_pre_task");
  node->declare_parameter("id", 0);
  int32_t id = node->get_parameter("id").as_int();

  if (id == 0) {
    RCLCPP_ERROR(node->get_logger(), "预定义任务ID不能为空!");
    return 1;
  }

  auto exec_pre_task_cli =
      node->create_client<woosh_robot_msgs::srv::ExecPreTask>(
          "woosh_robot/robot/ExecPreTask");
  while (not exec_pre_task_cli->wait_for_service(std::chrono::seconds(1))) {
    if (not rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "等待服务异常!");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "等待服务出现...");
  }

  auto request =
      std::make_shared<woosh_robot_msgs::srv::ExecPreTask::Request>();
  request->arg.task_set_id = id;
  auto request_future = exec_pre_task_cli->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, request_future) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "请求执行预定义任务失败!");
    return 1;
  }

  auto request_ret = request_future.get();
  if (request_ret->ok) {
    RCLCPP_INFO(node->get_logger(), "执行预定义任务请求成功, %s",
                request_ret->msg.c_str());
  } else {
    RCLCPP_INFO(node->get_logger(), "执行预定义任务请求失败, %s",
                request_ret->msg.c_str());
  }

  exec_pre_task_cli.reset();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
