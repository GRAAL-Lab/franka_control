#pragma once

namespace fer::topics {
// Relative names allow the whole stack to run under another ROS namespace.
inline constexpr char state_data[] = "state/state_data";
inline constexpr char ctrl_data[] = "ctrl/ctrl_data";
inline constexpr char task_status[] = "ctrl/task_status";
inline constexpr char joint_states[] = "joint_states";
inline constexpr char gripper_state[] = "gripper/state";
inline constexpr char control_command[] = "srv/control_command";
inline constexpr char driver_command[] = "driver/command";
inline constexpr char gripper_command[] = "gripper/command";
}  // namespace fer::topics
