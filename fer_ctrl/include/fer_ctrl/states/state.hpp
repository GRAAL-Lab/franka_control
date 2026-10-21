#pragma once

namespace fer::states {
enum class State { Disabled, Enabling, Idle, Parking, Parked, Unparking,
                   JointPosition, JointVelocity, Cartesian, Fault };
inline const char* name(State state) {
  switch (state) {
    case State::Disabled: return "disabled";
    case State::Enabling: return "enabling";
    case State::Idle: return "idle";
    case State::Parking: return "parking";
    case State::Parked: return "parked";
    case State::Unparking: return "unparking";
    case State::JointPosition: return "move_joints_pos";
    case State::JointVelocity: return "move_joints_vel";
    case State::Cartesian: return "move_cartesian";
    case State::Fault: return "fault";
  }
  return "fault";
}
}  // namespace fer::states
