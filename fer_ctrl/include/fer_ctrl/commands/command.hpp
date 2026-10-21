#pragma once

#include <string>
#include <vector>
#include "fer_model/panda_model.hpp"

namespace fer::commands {
enum class Type { Enable, Disable, Stop, Recover, Park, Unpark, JointPosition,
  SingleJointPosition, JointVelocity, SingleJointVelocity, Cartesian,
  GripperMove, GripperGrasp, GripperHome, GripperStop, Unknown };

inline Type parse(const std::string& name) {
  if (name == "enable") return Type::Enable;
  if (name == "disable") return Type::Disable;
  if (name == "stop") return Type::Stop;
  if (name == "recover") return Type::Recover;
  if (name == "park") return Type::Park;
  if (name == "unpark") return Type::Unpark;
  if (name == "move_joints_pos") return Type::JointPosition;
  if (name == "move_joint_i_pos") return Type::SingleJointPosition;
  if (name == "move_joints_vel") return Type::JointVelocity;
  if (name == "move_joint_i_vel") return Type::SingleJointVelocity;
  if (name == "move_cartesian") return Type::Cartesian;
  if (name == "gripper_move") return Type::GripperMove;
  if (name == "gripper_grasp") return Type::GripperGrasp;
  if (name == "gripper_home") return Type::GripperHome;
  if (name == "gripper_stop") return Type::GripperStop;
  return Type::Unknown;
}

struct Command {
  Type type{Type::Unknown};
  bool relative{false};
  bool valid_move_type{true};
  int joint_index{0};
  std::vector<double> joints;
  Eigen::Vector3d xyz{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rpy{Eigen::Vector3d::Zero()};
  int frame{1};  // 0: end effector, 1: base
};
}  // namespace fer::commands
