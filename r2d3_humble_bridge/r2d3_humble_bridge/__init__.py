"""r2d3_humble_bridge — translates custom rm_ros_interfaces to/from std_msgs.

Runs in the ros_humble (Python 3.11) env. Subscribes to the participant-
facing topics (`/{left,right}_arm_controller/rm_driver/*`), republishes as
sensor_msgs/JointState + std_msgs/Float64 under `/r2d3/sim/cmd/*` for the
Isaac sim_adapter. Also subscribes the Isaac-published state on
`/r2d3/sim/joint_states` etc. and aggregates into
`r2d3_model_interfaces/Observation` on `/r2d3/observations` at 10 Hz.
"""
__version__ = "0.1.0"
