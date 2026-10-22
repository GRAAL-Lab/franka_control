# Joint references for the torque backend, ordered Panda joints 1..7.
# Position: rad; velocity: rad/s; acceleration: rad/s^2. header.frame_id is the base frame.
std_msgs/Header header
bool control_enabled
string ctrl_state
float64[7] joint_positions
float64[7] joint_velocities
float64[7] joint_accelerations
string status
