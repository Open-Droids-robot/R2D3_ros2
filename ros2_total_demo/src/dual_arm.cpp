// Copyright RealMan
// License(BSD/GPL/...)
// Author: Starry

#include "dual_arm.h"

// 构造函数
DualArm::DualArm(rclcpp::Node::SharedPtr node) : node_(node)
{
  node_->declare_parameter<int>("arm_dof", arm_dof_);
  node_->get_parameter("arm_dof", arm_dof_);
  RCLCPP_INFO(node_->get_logger(), "arm_dof is %d\n", arm_dof_);
  if (arm_dof_ == 6)
  {
    movej_way_left.joint.resize(6);
    movej_way_right.joint.resize(6);
  }
  else if (arm_dof_ == 7)
  {
    movej_way_left.joint.resize(7);
    movej_way_right.joint.resize(7);
  }
  /*******************************************************Movej*************************************************************** */
  movej_sub_left_ = node_->create_subscription<std_msgs::msg::Bool>("/left_arm_controller/rm_driver/movej_result", 10,
                                                                    std::bind(&DualArm::DualArm_Callback_left, this, _1));
  movej_pub_left_ = node_->create_publisher<rm_ros_interfaces::msg::Movej>("/left_arm_controller/rm_driver/movej_cmd", 10);

  movej_sub_right_ = node_->create_subscription<std_msgs::msg::Bool>("/right_arm_controller/rm_driver/movej_result", 10,
                                                                     std::bind(&DualArm::DualArm_Callback_right, this, _1));
  movej_pub_right_ = node_->create_publisher<rm_ros_interfaces::msg::Movej>("/right_arm_controller/rm_driver/movej_cmd", 10);

  publisher_work_frame_left = node_->create_publisher<std_msgs::msg::String>("/left_arm_controller/rm_driver/change_work_frame_cmd", 10);
  publisher_work_frame_right = node_->create_publisher<std_msgs::msg::String>("/right_arm_controller/rm_driver/change_work_frame_cmd", 10);
  /*******************************************************Movej End*************************************************************/

  /*******************************************************Movejp**************************************************************/
  movejp_sub_left_ = node_->create_subscription<std_msgs::msg::Bool>("/left_arm_controller/rm_driver/movej_p_result", 10,
                                                                     std::bind(&DualArm::MoveJPDemo_Callback_left, this, _1));
  movejp_pub_left_ = node_->create_publisher<rm_ros_interfaces::msg::Movejp>("/left_arm_controller/rm_driver/movej_p_cmd", 10);

  movejp_sub_right_ = node_->create_subscription<std_msgs::msg::Bool>("/right_arm_controller/rm_driver/movej_p_result", 10,
                                                                      std::bind(&DualArm::MoveJPDemo_Callback_right, this, _1));
  movejp_pub_right_ = node_->create_publisher<rm_ros_interfaces::msg::Movejp>("/right_arm_controller/rm_driver/movej_p_cmd", 10);

  publisher_work_frame_left = node_->create_publisher<std_msgs::msg::String>("/left_arm_controller/rm_driver/change_work_frame_cmd", 10);
  publisher_work_frame_right = node_->create_publisher<std_msgs::msg::String>("/right_arm_controller/rm_driver/change_work_frame_cmd", 10);
  /*******************************************************Movejp End***************************************************************/

  /*******************************************************MoveL*********************************************************************/
  movel_sub_left_ = node_->create_subscription<std_msgs::msg::Bool>("/left_arm_controller/rm_driver/movel_result", 10,
                                                                    std::bind(&DualArm::MoveLDemo_Callback_Left, this, _1));
  movel_pub_left_ = node_->create_publisher<rm_ros_interfaces::msg::Movel>("/left_arm_controller/rm_driver/movel_cmd", 10);
  movel_sub_right_ = node_->create_subscription<std_msgs::msg::Bool>("/right_arm_controller/rm_driver/movel_result", 10,
                                                                     std::bind(&DualArm::MoveLDemo_Callback_Right, this, _1));
  movel_pub_right_ = node_->create_publisher<rm_ros_interfaces::msg::Movel>("/right_arm_controller/rm_driver/movel_cmd", 10);
  /*******************************************************MoveL End***************************************************************/

  /*******************************************************Lift Height***************************************************************/
  lift_height_pub_ = node_->create_publisher<rm_ros_interfaces::msg::Liftheight>("/left_arm_controller/rm_driver/set_lift_height_cmd", 10);
  lift_height_result_ = node_->create_subscription<std_msgs::msg::Bool>("/left_arm_controller/rm_driver/set_lift_height_result", 10,
                                                                        std::bind(&DualArm::lift_height_callback, this, _1));
  /*******************************************************Lift Height End***********************************************************/

  /*******************************************************Head Lift Height***************************************************************/
  head_servo_pub_ = node_->create_publisher<servo_interfaces::msg::ServoMove>("/servo_control/move", 1);
  
  /*******************************************************Head Height End***********************************************************/

  /*******************************************************Get Arm State*********************************************************/
  get_state_sub_left_ = node_->create_subscription<rm_ros_interfaces::msg::Armoriginalstate>("/left_arm_controller/rm_driver/get_current_arm_original_state_result", 10,
                                                                                                   std::bind(&DualArm::get_arm_state_Callback_left, this, _1));
  get_state_pub_left_ = node_->create_publisher<std_msgs::msg::Empty>("/left_arm_controller/rm_driver/get_current_arm_state_cmd", 10);
  get_state_sub_right_ = node_->create_subscription<rm_ros_interfaces::msg::Armoriginalstate>("/right_arm_controller/rm_driver/get_current_arm_original_state_result", 10,
                                                                                                    std::bind(&DualArm::get_arm_state_Callback_right, this, _1));
  get_state_pub_right_ = node_->create_publisher<std_msgs::msg::Empty>("/right_arm_controller/rm_driver/get_current_arm_state_cmd", 10);
  /*******************************************************Get Arm State End******************************************************/

  // 初始化左右臂工作坐标系
  init_frame_left();
  init_frame_right();
}

// 初始化左臂工作坐标系
void DualArm::init_frame_left()
{
  std_msgs::msg::String msg;
  msg.data = "Base";
  publisher_work_frame_left->publish(msg);
}

// 初始化右臂工作坐标系
void DualArm::init_frame_right()
{
  std_msgs::msg::String msg;
  msg.data = "Base";
  publisher_work_frame_right->publish(msg);
}

// 接收到订阅的机械臂执行状态消息后，会进入消息回调函数
void DualArm::DualArm_Callback_left(const std_msgs::msg::Bool::SharedPtr msg)
{
  // 将接收到的消息打印出来，显示是否执行成功
  if (msg->data)
  {
    RCLCPP_INFO(node_->get_logger(), "*******Left Movej succeeded\n");
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "*******Left Movej Failed\n");
  }
}
// 接收到订阅的机械臂执行状态消息后，会进入消息回调函数
void DualArm::DualArm_Callback_right(const std_msgs::msg::Bool::SharedPtr msg)
{
  // 将接收到的消息打印出来，显示是否执行成功
  if (msg->data)
  {
    RCLCPP_INFO(node_->get_logger(), "*******Right Movej succeeded\n");
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "*******Right Movej Failed\n");
  }
}

// 发布左臂movej指令函数
void DualArm::movej_demo_left()
{
  if (arm_dof_ == 6)
  {
    movej_way_left.joint[0] = -0.360829;
    movej_way_left.joint[1] = 0.528468;
    movej_way_left.joint[2] = 1.326293;
    movej_way_left.joint[3] = -0.000454;
    movej_way_left.joint[4] = 1.221748;
    movej_way_left.joint[5] = 0.000052;
    movej_way_left.speed = 20;
    movej_way_left.dof = 6;
  }
  if (arm_dof_ == 7)
  {
    movej_way_left.joint[0] = 0.176278;
    movej_way_left.joint[1] = 0.0;
    movej_way_left.joint[2] = 0.3543;
    movej_way_left.joint[3] = 0.53;
    movej_way_left.joint[4] = 0.00873;
    movej_way_left.joint[5] = 0.3595;
    movej_way_left.joint[6] = 0.3595;
    movej_way_left.speed = 20;
    movej_way_left.dof = 7;
  }
  movej_way_left.block = true;
  movej_pub_left_->publish(movej_way_left);
}

// 发布右臂movej指令函数
void DualArm::movej_demo_right()
{
  if (arm_dof_ == 6)
  {
    movej_way_right.joint[0] = -0.360829;
    movej_way_right.joint[1] = 0.528468;
    movej_way_right.joint[2] = 1.326293;
    movej_way_right.joint[3] = -0.000454;
    movej_way_right.joint[4] = 1.221748;
    movej_way_right.joint[5] = 0.000052;
    movej_way_right.speed = 20;
    movej_way_right.dof = 6;
  }
  if (arm_dof_ == 7)
  {
    movej_way_right.joint[0] = 0.176278;
    movej_way_right.joint[1] = 0.0;
    movej_way_right.joint[2] = 0.3543;
    movej_way_right.joint[3] = 0.53;
    movej_way_right.joint[4] = 0.00873;
    movej_way_right.joint[5] = 0.3595;
    movej_way_right.joint[6] = 0.3595;
    movej_way_right.speed = 20;
    movej_way_right.dof = 7;
  }
  movej_way_right.block = true;
  movej_pub_right_->publish(movej_way_right);
}

void DualArm::MoveJPDemo_Callback_left(const std_msgs::msg::Bool::SharedPtr msg)
{
  // 将接收到的消息打印出来，显示是否执行成功
  if (msg->data)
  {
    RCLCPP_INFO(node_->get_logger(), "*******Left MoveJP succeeded\n");
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "*******Left MoveJP Failed\n");
  }
}
void DualArm::MoveJPDemo_Callback_right(const std_msgs::msg::Bool::SharedPtr msg)
{
  // 将接收到的消息打印出来，显示是否执行成功
  if (msg->data)
  {
    RCLCPP_INFO(node_->get_logger(), "*******Right MoveJP succeeded\n");
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "*******Right MoveJP Failed\n");
  }
}

// 发布左臂movejp指令
void DualArm::movejp_demo_left()
{
  rm_ros_interfaces::msg::Movejp moveJ_P_TargetPose;
  moveJ_P_TargetPose.pose.position.x = -0.317239;
  moveJ_P_TargetPose.pose.position.y = 0.120903;
  moveJ_P_TargetPose.pose.position.z = 0.255765 + 0.04;
  moveJ_P_TargetPose.pose.orientation.x = -0.983404;
  moveJ_P_TargetPose.pose.orientation.y = -0.178432;
  moveJ_P_TargetPose.pose.orientation.z = 0.032271;
  moveJ_P_TargetPose.pose.orientation.w = 0.006129;
  moveJ_P_TargetPose.speed = 20;
  moveJ_P_TargetPose.block = true;
  movejp_pub_left_->publish(moveJ_P_TargetPose);
}

// 发布右臂movejp指令
void DualArm::movejp_demo_right()
{
  rm_ros_interfaces::msg::Movejp moveJ_P_TargetPose;
  moveJ_P_TargetPose.pose.position.x = -0.317239;
  moveJ_P_TargetPose.pose.position.y = 0.120903;
  moveJ_P_TargetPose.pose.position.z = 0.255765 + 0.04;
  moveJ_P_TargetPose.pose.orientation.x = -0.983404;
  moveJ_P_TargetPose.pose.orientation.y = -0.178432;
  moveJ_P_TargetPose.pose.orientation.z = 0.032271;
  moveJ_P_TargetPose.pose.orientation.w = 0.006129;
  moveJ_P_TargetPose.speed = 20;
  moveJ_P_TargetPose.block = true;
  movejp_pub_right_->publish(moveJ_P_TargetPose);
}

// 接收到订阅的机械臂执行状态消息后，会进入消息回调函数
void DualArm::MoveLDemo_Callback_Left(const std_msgs::msg::Bool::SharedPtr msg)
{
  // 将接收到的消息打印出来，显示是否执行成功
  if (msg->data)
  {
    RCLCPP_INFO(node_->get_logger(), "*******Left MoveL succeeded\n");
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "*******Left MoveL Failed\n");
  }
}

void DualArm::MoveLDemo_Callback_Right(const std_msgs::msg::Bool::SharedPtr msg)
{
  // 将接收到的消息打印出来，显示是否执行成功
  if (msg->data)
  {
    RCLCPP_INFO(node_->get_logger(), "*******Right MoveL succeeded\n");
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "*******Right MoveL Failed\n");
  }
}

void DualArm::moveL_publisher_left()
{
  rm_ros_interfaces::msg::Movel moveL_TargetPose;
  moveL_TargetPose.pose.position.x = -0.317239;
  moveL_TargetPose.pose.position.y = 0.120903;
  moveL_TargetPose.pose.position.z = 0.295765;
  moveL_TargetPose.pose.orientation.x = -0.983404;
  moveL_TargetPose.pose.orientation.y = -0.178432;
  moveL_TargetPose.pose.orientation.z = 0.032271;
  moveL_TargetPose.pose.orientation.w = 0.006129;
  moveL_TargetPose.speed = 20;
  moveL_TargetPose.block = true;

  movel_pub_left_->publish(moveL_TargetPose);
}
void DualArm::moveL_publisher_right()
{
  rm_ros_interfaces::msg::Movel moveL_TargetPose;
  moveL_TargetPose.pose.position.x = -0.317239;
  moveL_TargetPose.pose.position.y = 0.120903;
  moveL_TargetPose.pose.position.z = 0.295765;
  moveL_TargetPose.pose.orientation.x = -0.983404;
  moveL_TargetPose.pose.orientation.y = -0.178432;
  moveL_TargetPose.pose.orientation.z = 0.032271;
  moveL_TargetPose.pose.orientation.w = 0.006129;
  moveL_TargetPose.speed = 20;
  moveL_TargetPose.block = true;

  movel_pub_right_->publish(moveL_TargetPose);
}

// 接收到订阅的机械臂执行状态消息后，会进入消息回调函数
void DualArm::get_arm_state_Callback_left(const rm_ros_interfaces::msg::Armoriginalstate::SharedPtr msg)
{
  // 将接收到的消息打印出来，显示是否执行成功
  if (msg->dof == 7)
  {
    RCLCPP_INFO(node_->get_logger(), "joint state is: [%lf, %lf, %lf, %lf, %lf, %lf, %lf]\n",
                msg->joint[0], msg->joint[1], msg->joint[2], msg->joint[3], msg->joint[4], msg->joint[5], msg->joint[6]);
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "joint state is: [%lf, %lf, %lf, %lf, %lf, %lf]\n",
                msg->joint[0], msg->joint[1], msg->joint[2], msg->joint[3], msg->joint[4], msg->joint[5]);
  }
  RCLCPP_INFO(node_->get_logger(), "pose state is: [%lf, %lf, %lf, %lf, %lf, %lf]\n",
              msg->pose[0], msg->pose[1], msg->pose[2], msg->pose[3], msg->pose[4], msg->pose[5]);
  RCLCPP_INFO(node_->get_logger(), "arm_err is: %d\n", msg->arm_err);
  RCLCPP_INFO(node_->get_logger(), "sys_err is: %d\n", msg->sys_err);
}

// 接收到订阅的机械臂执行状态消息后，会进入消息回调函数
void DualArm::get_arm_state_Callback_right(const rm_ros_interfaces::msg::Armoriginalstate::SharedPtr msg)
{
  // 将接收到的消息打印出来，显示是否执行成功
  if (msg->dof == 7)
  {
    RCLCPP_INFO(node_->get_logger(), "joint state is: [%lf, %lf, %lf, %lf, %lf, %lf, %lf]\n",
                msg->joint[0], msg->joint[1], msg->joint[2], msg->joint[3], msg->joint[4], msg->joint[5], msg->joint[6]);
  }
  else
  {
    RCLCPP_INFO(node_->get_logger(), "joint state is: [%lf, %lf, %lf, %lf, %lf, %lf]\n",
                msg->joint[0], msg->joint[1], msg->joint[2], msg->joint[3], msg->joint[4], msg->joint[5]);
  }
  RCLCPP_INFO(node_->get_logger(), "pose state is: [%lf, %lf, %lf, %lf, %lf, %lf]\n",
              msg->pose[0], msg->pose[1], msg->pose[2], msg->pose[3], msg->pose[4], msg->pose[5]);
  RCLCPP_INFO(node_->get_logger(), "arm_err is: %d\n", msg->arm_err);
  RCLCPP_INFO(node_->get_logger(), "sys_err is: %d\n", msg->sys_err);
}
// 升降高度结果回调函数
void DualArm::lift_height_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data == true)
  {
    RCLCPP_INFO(node_->get_logger(), "set lift height successfully!");
  }
  else
  {
    RCLCPP_ERROR(node_->get_logger(), "set lift height failed!");
  }
}

// 执行升降动作接口
void DualArm::execute_lift_height(int height, int speed)
{
  auto msg = rm_ros_interfaces::msg::Liftheight();
  msg.height = height;
  msg.speed = speed;
  lift_height_pub_->publish(msg);
}

// 执行头部舵机动作接口
void DualArm::execute_head_servo(int id, int angle)
{
  rclcpp::sleep_for(std::chrono::milliseconds(500));
  auto msg = servo_interfaces::msg::ServoMove();
  msg.servo_id = id;     // 舵机ID
  msg.angle = angle;      // 舵机角度
  RCLCPP_INFO(node_->get_logger(), "send msg: servo_id = %d,angle = %d", msg.servo_id, msg.angle);
  // 发布消息
  head_servo_pub_->publish(msg);
}

// 获取左臂位姿函数
void DualArm::get_arm_state_left()
{
  std_msgs::msg::Empty get_state;
  get_state_pub_left_->publish(get_state);
}

// 获取右臂位姿函数
void DualArm::get_arm_state_right()
{
  std_msgs::msg::Empty get_state;
  get_state_pub_right_->publish(get_state);
}

// 执行movej
void DualArm::execute_movej()
{
  movej_demo_left();
  movej_demo_right();
}

// 执行movejp
void DualArm::execute_movejp()
{
  movejp_demo_left();
  movejp_demo_right();
}

// 执行movel
void DualArm::execute_movel()
{
  moveL_publisher_left();
  moveL_publisher_right();
}

// 执行获取双臂状态
void DualArm::execute_get_arm_state()
{
  get_arm_state_left();
  get_arm_state_right();
}
