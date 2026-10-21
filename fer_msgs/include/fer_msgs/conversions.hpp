#pragma once

#include <stdexcept>
#include <vector>
#include "fer_model/panda_model.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace fer::ros {
inline geometry_msgs::msg::Pose pose_message(const Pose& pose) {
  geometry_msgs::msg::Pose msg;
  msg.position.x = pose.translation().x();
  msg.position.y = pose.translation().y();
  msg.position.z = pose.translation().z();
  const Eigen::Quaterniond q(pose.linear());
  msg.orientation.x = q.x(); msg.orientation.y = q.y();
  msg.orientation.z = q.z(); msg.orientation.w = q.w();
  return msg;
}
inline Pose pose(const geometry_msgs::msg::Pose& msg) {
  const Eigen::Quaterniond q(msg.orientation.w, msg.orientation.x,
                             msg.orientation.y, msg.orientation.z);
  if (!q.coeffs().allFinite() || std::abs(q.norm() - 1.) > 1e-5) {
    throw std::invalid_argument("Invalid feedback quaternion");
  }
  Pose result = Pose::Identity();
  result.translation() = Eigen::Vector3d(msg.position.x, msg.position.y, msg.position.z);
  result.linear() = q.toRotationMatrix();
  return result;
}
inline std::vector<std::string> joint_names() {
  std::vector<std::string> result;
  for (int i = 1; i <= 7; ++i) result.push_back("panda_joint" + std::to_string(i));
  return result;
}
}  // namespace fer::ros
