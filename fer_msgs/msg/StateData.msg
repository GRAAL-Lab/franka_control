std_msgs/Header header
sensor_msgs/JointState joint_state
geometry_msgs/Pose tool_pose
# Base-frame end-effector Jacobian, 6 x 7, column-major (libfranka convention).
float64[42] zero_jacobian
bool connected
bool ready
bool control_active
bool has_error
uint8 robot_mode
float64 control_command_success_rate
string error
