# Observation.msg — the single message a participant policy subscribes to.
#
# The R2D3 Isaac Sim adapter publishes this at the rate configured in the
# task YAML (default ≥ 10 Hz). It bundles all sensor data + robot state the
# policy needs into one message, simplifying policy authoring.
#
# Modeled on aic_model_interfaces/Observation.msg.

std_msgs/Header header

# ===== Head camera (the only physical D435 on V1 hardware) =====
sensor_msgs/Image       head_color
sensor_msgs/Image       head_depth
sensor_msgs/CameraInfo  head_camera_info

# ===== Joint state (all joints: both arms + lift + head pan/tilt + fingers) =====
sensor_msgs/JointState  joint_states

# ===== Per-arm RealMan state (kinematics + status) =====
rm_ros_interfaces/Armstate  left_arm_state
rm_ros_interfaces/Armstate  right_arm_state

# ===== Optional force-torque (zeros when the task isn't contact-driven) =====
geometry_msgs/WrenchStamped left_wrench
geometry_msgs/WrenchStamped right_wrench

# ===== Body lift state =====
rm_ros_interfaces/Liftstate lift_state
