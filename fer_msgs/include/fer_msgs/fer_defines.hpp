/** @file fer_defines.hpp */

#ifndef FER_MSGS_FER_DEFINES_HPP
#define FER_MSGS_FER_DEFINES_HPP

namespace fer {
/** @brief Relative ROS topic and service names, resolved within the node namespace. */
namespace topicnames {
// Relative names allow the whole stack to run under another ROS namespace.
inline constexpr char state_data[] = "state/state_data";
inline constexpr char ctrl_data[] = "ctrl/ctrl_data";
inline constexpr char dynamic_control[] = "ctrl/dynamic_control";
inline constexpr char tpik_action[] = "ctrl/tpik_action";
inline constexpr char task_status[] = "ctrl/task_status";
inline constexpr char joint_states[] = "joint_states";
inline constexpr char gripper_state[] = "gripper/state";
inline constexpr char control_command[] = "srv/control_command";
inline constexpr char driver_command[] = "driver/command";
inline constexpr char gripper_command[] = "gripper/command";
}
}

#endif
