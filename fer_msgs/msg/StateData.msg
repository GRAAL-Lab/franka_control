# Driver feedback in header.frame_id (the configured base frame).
# JointState uses rad, rad/s and Nm, ordered Panda joints 1..7; tool_pose uses metres.
std_msgs/Header header
sensor_msgs/JointState joint_state
geometry_msgs/Pose tool_pose
# Base-frame end-effector Jacobian, 6 x 7, column-major (libfranka convention).
float64[42] zero_jacobian
# Flange-to-end-effector and inertia matrices are column-major.
float64[16] flange_to_tool
float64[49] mass_matrix
float64[7] coriolis
float64[7] commanded_torques
bool connected
bool ready
bool control_active
bool has_error
uint8 robot_mode
float64 control_command_success_rate
string error
